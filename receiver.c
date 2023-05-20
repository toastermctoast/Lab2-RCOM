/*Non-Canonical Input Processing*/

#include <sys/types.h>
/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BAUDRATE B38400
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1


#define FLAG 0x5c   // header delimitation
#define A    0x01   //Answer sent by receiver
#define C_UA    0x07   //Control field - UA
#define C_SET   0x03   //Control field - SET
#define BCC_UA  (A^C_UA)
#define BCC_SET (A^C_SET)

#define FRAME_SIZE 5

volatile int STOP=FALSE;
int fd;

typedef enum
{
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_A,
} message_state;

message_state state = START;

int state_handler()
{
    char buff[3],buf;
    int res;

    while (state != STOP_A) { 
        res = read(fd, &buf, sizeof(buf));
        printf("%x\n", buf);
        if(res < 0 )
            return 1; // erro


        switch (state)
        {

            case START:
                if (buf == FLAG){
                    state = FLAG_RCV;
                }
                else
                    state = START;

            break;
                 

            case FLAG_RCV:

                if (buf == A)
                {
                    buff[0] = buf;
                    state = A_RCV;
                }
            
                else if (buf == FLAG)
                    state = FLAG_RCV;
            
                
                else
                    state = START;        
            break;

            case A_RCV:
                if (buf == C_SET)
                {
                    buff[1] = buf;
                    state = C_RCV;
                }
                else if (buf == FLAG)
                {
                    state = FLAG_RCV;
                }
                else
                {
                    state = START;
                }
            break;
            
            case C_RCV:
                if (buff[0] ^ buff[1])
                {
                    state = BCC_OK;

                }
                else if (FLAG)
                {
                    state = FLAG_RCV;

                }
                else
                {
                    state = START;

                }
            break;

            case BCC_OK:
                if (buf == FLAG)
                {
                    state = STOP_A;
                }
                else
                {
                    state = START;
                }
                 break;

            case STOP_A:
                break;               
            
        }
    }
    return 0;
}

int main(int argc, char** argv)
{
    int c, res, res_read;
    struct termios oldtio,newtio;
    char buff;

    if ( (argc < 2) || ((strcmp("/dev/ttyS10", argv[1])!=0) &&  (strcmp("/dev/ttyS11", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }

    fd = open(argv[1], O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(argv[1]); exit(-1); }

    if (tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;   /* blocking read until 5 chars received */

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

        res_read = state_handler(); 
        if(res == 0) 
            printf("Leu corretamente\n");
        else 
            printf("ERRO\n");
            

        printf("SET state achieved\n");
        char ua[FRAME_SIZE] = {FLAG, A, C_UA, BCC_UA, FLAG};
        res = write(fd, ua, FRAME_SIZE);
        printf("%d BYTES SENT\n", res);
            

    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
