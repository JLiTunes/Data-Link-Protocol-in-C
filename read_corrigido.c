#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>



#define BAUDRATE B38400
#define _POSIX_SOURCE 1
#define MAX_PAYLOAD_SIZE 1024

// Protocolo Camada de Ligação
#define FLAG 0x7E
#define ESC  0x7D
#define A_TX 0x03  // Comandos do Emissor
#define A_RX 0x01  // Comandos do Recetor / Respostas do Emissor

#define C_SET  0x03
#define C_UA   0x07
#define C_DISC 0x0B
#define C_I0   0x00 //PACKET 0
#define C_I1   0x40 //PACKET 1
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

// Variáveis Globais
struct termios oldtio;
int expected_ns = 0; // Próximo Ns esperado (0 ou 1)


typedef enum { STATE_START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STATE_STOP } State;

int readSupervision(int fd, unsigned char targetA, unsigned char targetC) { //parte de ler o SET 
    unsigned char byte;
    State state = STATE_START;

    while (state != STATE_STOP) {
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
    return 0;
}


int llopen(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) { perror(port); return -1; }

    tcgetattr(fd, &oldtio);
    struct termios newtio;
    memset(&newtio, 0, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_cc[VTIME] = 0; //tempo timeout do read
    newtio.c_cc[VMIN] = 1; //bytes min do read antes de dar timeout
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    printf("[LLOPEN] À espera do SET...\n");
    readSupervision(fd, A_TX, C_SET);
    
    printf("[LLOPEN] SET recebido. A enviar UA...\n");
    unsigned char ua[5] = {FLAG, A_TX, C_UA, A_TX ^ C_UA, FLAG};
    write(fd, ua, 5);

    return fd;
}


int llread(int fd, unsigned char *packet) {
    State state = STATE_START;
    unsigned char byte, control;
    int i = 0;

    while (state != STATE_STOP) {
        if (read(fd, &byte, 1) <= 0) continue;

        switch (state) {
            case STATE_START:
                if (byte == FLAG) state = FLAG_RCV;
                break;
            case FLAG_RCV:
                if (byte == A_TX) state = A_RCV;
                else if (byte != FLAG) state = STATE_START;
                break;
            case A_RCV:
                if (byte == C_I0 || byte == C_I1 || byte == C_DISC) {
                    control = byte;
                    state = C_RCV;
                } else if (byte == FLAG) state = FLAG_RCV;
                else state = STATE_START;
                break;
            case C_RCV:
                if (byte == (A_TX ^ control)) state = BCC_OK;
                else state = STATE_START;
                break;
            case BCC_OK:
                if (byte == FLAG) {
                    state = STATE_STOP;
                } else if (byte == ESC) {
                    // Destuffing
                    read(fd, &byte, 1);
                    if (byte == 0x5E) packet[i++] = FLAG;
                    else if (byte == 0x5D) packet[i++] = ESC;
                } else {
                    packet[i++] = byte;
                }
                break;
            default: break;
        }
    }

    // Se recebermos um DISC, retornar erro/fechar
    if (control == C_DISC) return -2;

    // tirar o bcc do packet
    unsigned char bcc2_received = packet[--i];
    unsigned char bcc2_calc = 0;
    for (int j = 0; j < i; j++) bcc2_calc ^= packet[j];

    unsigned char res[5];
    res[0] = FLAG;
    res[1] = A_TX;

    // se o xor não for igual a bcc2 enviar REJ
    if (bcc2_calc != bcc2_received) {
        printf("[LLREAD] Erro de BCC2! A enviar REJ...\n");
        res[2] = (expected_ns == 0) ? C_REJ0 : C_REJ1; //if ns = 0 C_REJ0 else C_REJ1
        res[3] = res[1] ^ res[2];
        res[4] = FLAG;
        write(fd, res, 5);
        return -1;
    }

    // Verificar número de sequência
    int ns_received = (control == C_I0) ? 0 : 1;
    if (ns_received == expected_ns) {
        expected_ns = (expected_ns + 1) % 2;
        res[2] = (expected_ns == 0) ? C_RR0 : C_RR1;
        res[3] = res[1] ^ res[2];
        res[4] = FLAG;
        write(fd, res, 5);
        return i; // Sucesso, retorna tamanho dos dados
    } else {
        // Pacote duplicado: aceitamos mas descartamos os dados para a app
        printf("[LLREAD] Pacote duplicado detectado. A enviar RR...\n");
        res[2] = (expected_ns == 0) ? C_RR0 : C_RR1;
        res[3] = res[1] ^ res[2];
        res[4] = FLAG;
        write(fd, res, 5);
        return -1; 
    }
}


int llclose(int fd) {
    printf("[LLCLOSE] A enviar DISC...\n");
    unsigned char disc[5] = {FLAG, A_RX, C_DISC, A_RX ^ C_DISC, FLAG};
    write(fd, disc, 5);

    printf("[LLCLOSE] A aguardar UA final...\n");
    readSupervision(fd, A_RX, C_UA);

    tcsetattr(fd, TCSANOW, &oldtio);
    close(fd);
    printf("[LLCLOSE] Ligação terminada.\n");
    return 0;
}


int main(int argc, char** argv) {
    if (argc < 2) {
        printf("Uso: %s <PortaSerie>\n", argv[0]);
        return 1;
    }

    int fd = llopen(argv[1]);
    if (fd < 0) return 1;

    unsigned char packet[2048];
    FILE *f = NULL;
    char filename[256] = "";

    printf("[MAIN] Pronto para receber dados...\n");

    while (1) {
        int n = llread(fd, packet);

        if (n == -2) { // Recebeu DISC
            printf("[MAIN] Comando DISC recebido do emissor.\n");
            break;
        }
        if (n <= 0) continue;

        unsigned char control = packet[0];

        if (control == APP_START) {
            int idx = 1;
            while (idx < n) {
                unsigned char type = packet[idx++];
                unsigned char length = packet[idx++];
                if (type == T_NAME) {
                    memcpy(filename, &packet[idx], length);
                    filename[length] = '\0';
                    idx += length;
                } else {
                    idx += length;
                }
            }
            char savePath[300];
            sprintf(savePath, "received_%s", filename);
            f = fopen(savePath, "wb");
            printf("[MAIN] A receber ficheiro: %s\n", filename);
        } 
        else if (control == APP_DATA) {
            // Cabeçalho de 3 bytes: C | L2 | L1
            int dataSize = (packet[1] << 8) | packet[2];
            if (f) {
                fwrite(&packet[3], 1, dataSize, f);
                printf("[MAIN] Bloco de dados recebido (%d bytes)\n", dataSize);
            }
        } 
        else if (control == APP_END) {
            if (f) {
                fclose(f);
                f = NULL;
            }
            printf("[MAIN] Transferência do ficheiro concluída.\n");
        }
    }

    llclose(fd);
    return 0;
}