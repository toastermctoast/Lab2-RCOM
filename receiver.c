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

int read_command(int fd, char* recv){

    char FLAG_COMMAND = recv[0];
    char A_COMMAND = recv[1];
    char C_COMMAND = recv[2];
    char BCC_COMMAND = recv[3];

    //Estados
    typedef enum {
        START,
        FLAG_RCV,
        A_RCV,
        C_RCV,
        BCC_OK,
        STOP
    } statesSET;

    statesSET state = START;
    int i = 0;

    char byte;

    while(1){

        read(fd,&byte,1); //reads chars one by one
        printf("\nVOLTA : %d ---- Byte lido: %x\n", i,byte);

        switch(state){

		    case START:
		        printf("start\n");
		        if(byte==FLAG_COMMAND) state=FLAG_RCV;
		        else state=START; 
		    break;
		   
            case FLAG_RCV:
		        printf("flag_rcv\n");
		        if(byte==A_COMMAND){ state = A_RCV; break;}
		        if (byte == FLAG_COMMAND){ state = FLAG_RCV; break;}
                state = START;
		    break;

            case A_RCV:
		        printf("a_rcv\n");
                if(byte == C_COMMAND){ state = C_RCV; break;}
		        if(byte == FLAG_COMMAND){ state = FLAG_RCV; break;}
                state = START;
		    break;

            case C_RCV:
		        printf("c_rcv\n");
                if(byte == BCC_COMMAND){ state = BCC_OK; break;}
		        if(byte == FLAG_COMMAND){ state = FLAG_RCV; break;}
                state = START;
		    break;

            case BCC_OK:
		        printf("bcc_ok\n");
                if(byte == FLAG_COMMAND) state = STOP;
                else state = START;
		    break;
		   

        }
        if (state==STOP) return 1;
        i++;
    }

    return -1;
}

int send_answer(int fd, char* command){

    int res;

    res = write(fd,command,FRAME_SIZE);            //returns number of written bytes in the driver file and saves in res
    printf("%d BYTES SENT\n", res);

    if (res == 5) return 1;

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

    //RECEIVES SET COMMAND
    char set[FRAME_SIZE] =  {FLAG, A, C_SET, BCC_SET, FLAG};
    if (read_command(fd,set) != 1) printf ("Error reading command\n"); 
    else printf("SET state achieved\n");

    //SENDS UA ANSWER
    char ua[FRAME_SIZE] = {FLAG, A, C_UA, BCC_UA, FLAG};
    if (send_answer(fd,ua) != 1) printf("ERROR SENDING UA ANSWER\n");
    else printf("UA ANSWER SENT\n");

    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
