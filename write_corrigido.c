#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

// ============================================================================
// DEFINIÇÕES E CONSTANTES
// ============================================================================

#define BAUDRATE B38400
#define _POSIX_SOURCE 1
#define MAX_PAYLOAD_SIZE 1024

// Protocolo Camada de Ligação
#define FLAG 0x7E
#define ESC  0x7D
#define A_TX 0x03  // Comandos do Emissor / Respostas do Recetor
#define A_RX 0x01  // Comandos do Recetor / Respostas do Emissor

#define C_SET  0x03
#define C_UA   0x07
#define C_DISC 0x0B
#define C_I0   0x00
#define C_I1   0x40
#define C_RR0  0x05
#define C_RR1  0x85
#define C_REJ0 0x01
#define C_REJ1 0x81

// Protocolo Camada de Aplicação
#define APP_DATA  0x01
#define APP_START 0x02
#define APP_END   0x03
#define T_SIZE    0x00
#define T_NAME    0x01

// Configurações de retransmissão
#define MAX_ATTEMPTS 3
#define TIMEOUT 3

// Variáveis Globais
struct termios oldtio;
int alarmEnabled = 0;
int alarmCount = 0;
int ns = 0; // Número de sequência da trama I (0 ou 1)

// ----------------------------------------------------------------------------
// GESTÃO DE ALARME
// ----------------------------------------------------------------------------
void alarmHandler(int signal) {
    alarmEnabled = 1;
    alarmCount++;
    printf("\n[ALARM] Tentativa %d...\n", alarmCount);
}

// ----------------------------------------------------------------------------
// MÁQUINA DE ESTADOS (Leitura de Supervisão)
// ----------------------------------------------------------------------------
typedef enum { STATE_START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STATE_STOP } State;

int readSupervision(int fd, unsigned char targetA, unsigned char targetC) {
    unsigned char byte;
    State state = STATE_START;
    
    // Continua a ler enquanto não chegar ao estado STOP e não houver timeout
    while (state != STATE_STOP && alarmEnabled == 0) {
        if (read(fd, &byte, 1) <= 0) continue;

        switch (state) {
            case STATE_START:
                if (byte == FLAG) state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if (byte == targetA) state = A_RCV;
                else if (byte != FLAG) state = STATE_START;
                break;
            case A_RCV:
                if (byte == targetC) state = C_RCV;
                else if (byte == FLAG) state = FLAG_RCV;
                else state = STATE_START;
                break;
            case C_RCV:
                if (byte == (targetA ^ targetC)) state = BCC_OK;
                else if (byte == FLAG) state = FLAG_RCV;
                else state = STATE_START;
                break;
            case BCC_OK:
                if (byte == FLAG) state = STATE_STOP;
                else state = STATE_START;
                break;
            default: break;
        }
    }
    return (state == STATE_STOP) ? 0 : -1;
}

// ----------------------------------------------------------------------------
// LLOPEN
// ----------------------------------------------------------------------------
int llopen(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) { perror(port); return -1; }

    tcgetattr(fd, &oldtio);
    struct termios newtio;
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 1;
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    signal(SIGALRM, alarmHandler);
    unsigned char set_frame[5] = {FLAG, A_TX, C_SET, A_TX ^ C_SET, FLAG};

    alarmCount = 0;
    while (alarmCount < MAX_ATTEMPTS) {
        write(fd, set_frame, 5);
        alarm(TIMEOUT);
        alarmEnabled = 0;
        
        if (readSupervision(fd, A_TX, C_UA) == 0) {
            alarm(0); // Desativa alarme
            printf("[LLOPEN] Conexão estabelecida.\n");
            return fd;
        }
    }

    printf("[LLOPEN] Falha ao estabelecer conexão.\n");
    return -1;
}

// ----------------------------------------------------------------------------
// LLWRITE
// ----------------------------------------------------------------------------
int llwrite(int fd, unsigned char *buffer, int length) {
    unsigned char frame[MAX_PAYLOAD_SIZE * 2 + 10];
    int frameIdx = 0;
    unsigned char ctrl = (ns == 0) ? C_I0 : C_I1;

    // Header
    frame[frameIdx++] = FLAG;
    frame[frameIdx++] = A_TX;
    frame[frameIdx++] = ctrl;
    frame[frameIdx++] = A_TX ^ ctrl;

    // BCC2 (calculado sobre o buffer de dados)
    unsigned char bcc2 = 0;
    for (int i = 0; i < length; i++) bcc2 ^= buffer[i];

    // Byte Stuffing (Dados + BCC2)
    for (int i = 0; i <= length; i++) {
        unsigned char b = (i < length) ? buffer[i] : bcc2;
        if (b == FLAG) {
            frame[frameIdx++] = ESC;
            frame[frameIdx++] = 0x5E;
        } else if (b == ESC) {
            frame[frameIdx++] = ESC;
            frame[frameIdx++] = 0x5D;
        } else {
            frame[frameIdx++] = b;
        }
    }
    frame[frameIdx++] = FLAG;

    // Protocolo Stop-and-Wait
    unsigned char expectedRR = (ns == 0) ? C_RR1 : C_RR0;
    unsigned char expectedREJ = (ns == 0) ? C_REJ0 : C_REJ1;

    alarmCount = 0;
    while (alarmCount < MAX_ATTEMPTS) {
    write(fd, frame, frameIdx);
    alarm(TIMEOUT);
    alarmEnabled = 0;

    unsigned char byte;
    State state = STATE_START;
    unsigned char A, C;

    // Read supervision frame manually (to detect RR or REJ)
    while (state != STATE_STOP && alarmEnabled == 0) {
        if (read(fd, &byte, 1) <= 0) continue;

        switch (state) {
            case STATE_START:
                if (byte == FLAG) state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if (byte == A_TX) {
                    A = byte;
                    state = A_RCV;
                } else if (byte != FLAG) state = STATE_START;
                break;

            case A_RCV:
                if (byte == C_RR0 || byte == C_RR1 ||
                    byte == C_REJ0 || byte == C_REJ1) {
                    C = byte;
                    state = C_RCV;
                } else if (byte == FLAG) state = FLAG_RCV;
                else state = STATE_START;
                break;

            case C_RCV:
                if (byte == (A ^ C)) state = BCC_OK;
                else state = STATE_START;
                break;

            case BCC_OK:
                if (byte == FLAG) state = STATE_STOP;
                else state = STATE_START;
                break;

            default:
                break;
        }
    }

    alarm(0);

    if (state != STATE_STOP) {
        // Timeout → retransmit
        continue;
    }

    // RR received
    if (C == expectedRR) {
        ns = (ns + 1) % 2;
        return length;
    }

    // REJ received retransmission
    if (C == expectedREJ) {
        printf("[LLWRITE] REJ recebido → retransmitir imediatamente\n");
        continue;
    }

    // Ignore other cases → retransmit
}

    return -1;
}

// ----------------------------------------------------------------------------
// LLCLOSE
// ----------------------------------------------------------------------------
int llclose(int fd) {
    unsigned char disc_frame[5] = {FLAG, A_TX, C_DISC, A_TX ^ C_DISC, FLAG};
    
    alarmCount = 0;
    while (alarmCount < MAX_ATTEMPTS) {
        write(fd, disc_frame, 5);
        alarm(TIMEOUT);
        alarmEnabled = 0;

        // Espera DISC do recetor (A_RX pois o recetor envia comando DISC)
        if (readSupervision(fd, A_RX, C_DISC) == 0) {
            alarm(0);
            // Envia UA final (A_RX pois é uma resposta do emissor a um comando do recetor)
            unsigned char ua_frame[5] = {FLAG, A_RX, C_UA, A_RX ^ C_UA, FLAG};
            write(fd, ua_frame, 5);
            printf("[LLCLOSE] Conexão terminada com sucesso.\n");
            break;
        }
    }

    sleep(1); // Margem para o UA sair
    tcsetattr(fd, TCSANOW, &oldtio);
    close(fd);
    return 0;
}

// ----------------------------------------------------------------------------
// FUNÇÕES DE APLICAÇÃO
// ----------------------------------------------------------------------------

int sendControlPacket(int fd, unsigned char ctrl, const char *filename, int filesize) {
    unsigned char packet[256];
    int idx = 0;

    packet[idx++] = ctrl;

    // Tamanho do ficheiro (T, L, V)
    packet[idx++] = T_SIZE;
    packet[idx++] = 4;
    packet[idx++] = (filesize >> 24) & 0xFF;
    packet[idx++] = (filesize >> 16) & 0xFF;
    packet[idx++] = (filesize >> 8) & 0xFF;
    packet[idx++] = filesize & 0xFF;

    // Nome do ficheiro (T, L, V)
    int namelen = strlen(filename);
    packet[idx++] = T_NAME;
    packet[idx++] = namelen;
    memcpy(&packet[idx], filename, namelen);
    idx += namelen;

    return llwrite(fd, packet, idx);
}

// ----------------------------------------------------------------------------
// MAIN
// ----------------------------------------------------------------------------
int main(int argc, char** argv) {
    if (argc < 3) {
        printf("Uso: %s <PortaSerie> <Ficheiro>\n", argv[0]);
        return 1;
    }

    const char* port = argv[1];
    const char* filename = argv[2];

    FILE *f = fopen(filename, "rb");
    if (!f) { perror("Erro ao abrir ficheiro"); return 1; }

    fseek(f, 0, SEEK_END);
    int filesize = ftell(f);
    fseek(f, 0, SEEK_SET);

    int fd = llopen(port);
    if (fd < 0) { fclose(f); return 1; }

    printf("[MAIN] A enviar pacote START...\n");
    sendControlPacket(fd, APP_START, filename, filesize);

    unsigned char fileBuf[512];
    unsigned char appPacket[512 + 3];
    int bytesRead;

    printf("[MAIN] A enviar dados do ficheiro...\n");
    while ((bytesRead = fread(fileBuf, 1, sizeof(fileBuf), f)) > 0) {
        // Cabeçalho de aplicação (3 bytes conforme o guião): C | L2 | L1
        appPacket[0] = APP_DATA;
        appPacket[1] = (bytesRead >> 8) & 0xFF; // L2 (High byte)
        appPacket[2] = bytesRead & 0xFF;        // L1 (Low byte)
        memcpy(&appPacket[3], fileBuf, bytesRead);

        if (llwrite(fd, appPacket, bytesRead + 3) < 0) {
            printf("[MAIN] Erro fatal no llwrite.\n");
            break;
        }
    }

    printf("[MAIN] A enviar pacote END...\n");
    sendControlPacket(fd, APP_END, filename, filesize);

    fclose(f);
    llclose(fd);
    return 0;
}