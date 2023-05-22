/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS1"
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
#define TIME_OUT 5

int timer_on = 0, alarm_counter = 0;
int fd;
char set[FRAME_SIZE] =  {FLAG, A, C_SET, BCC_SET, FLAG};
char ua[FRAME_SIZE] = {FLAG, A, C_UA, BCC_UA, FLAG};


void flag_time_out(){ 
    timer_on = 0;
    alarm_counter++;
    if (alarm_counter > 3) {
        printf("Timed out for the third time\n");
        exit(1);
    } 
    printf("Alarm timed out\n Retransmitting...\n");
    send_command(fd,set);
}

int send_command(int fd, char* command){

    int res;

    res = write(fd,command,FRAME_SIZE);            //returns number of written bytes in the driver file and saves in res
    printf("%d BYTES SENT\n", res);

    if (res == 5) return 1;

}

int read_answer(int fd, char* answer){

    char FLAG_ANSWER = answer[0];
    char A_ANSWER = answer[1];
    char C_ANSWER = answer[2];
    char BCC_ANSWER = answer[3];

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

        if (!timer_on) {
            alarm(TIME_OUT);  // ativa alarme de TIME_OUT segundos
            printf("Alarm set\n");
            timer_on=1;
        }


        read(fd,&byte,1); //reads chars one by one

        //printf("\nVOLTA : %d ---- Byte lido: %x\n", i,byte);

        switch(state){

		    case START:
		        printf("start\n");
		        if(byte==FLAG_ANSWER) state=FLAG_RCV;
		        else state=START; 
		    break;
		   
            case FLAG_RCV:
		        printf("flag_rcv\n");
		        if(byte==A_ANSWER){ state = A_RCV; break;}
		        if (byte == FLAG_ANSWER){ state = FLAG_RCV; break;}
                state = START;
		    break;

            case A_RCV:
		        printf("a_rcv\n");
                if(byte == C_ANSWER){ state = C_RCV; break;}
		        if(byte == FLAG_ANSWER){ state = FLAG_RCV; break;}
                state = START;
		    break;

            case C_RCV:
		        printf("c_rcv\n");
                if(byte == BCC_ANSWER){ state = BCC_OK; break;}
		        if(byte == FLAG_ANSWER){ state = FLAG_RCV; break;}
                state = START;
		    break;

            case BCC_OK:
		        printf("bcc_ok\n");
                if(byte == FLAG_ANSWER) state = STOP;
                else state = START;
		    break;
		   

        }
        if (state==STOP) return 1;
        i++;
    }

    return -1;
}

int main(int argc, char** argv){
    int c;
    struct termios oldtio,newtio;
    int i, sum = 0, speed = 0;

    if ( (argc < 2) || ((strcmp("/dev/ttyS10", argv[1])!=0) &&  (strcmp("/dev/ttyS11", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }


    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */


    fd = open(argv[1], O_RDWR | O_NOCTTY );     //abre o ficheiro e retorna o file descriptor
    if (fd < 0) { perror(argv[1]); exit(-1); }

    if ( tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 1;   /* blocking read until 5 chars received */

    /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) próximo(s) caracter(es)
    */


    tcflush(fd, TCIOFLUSH);     //TCIOFLUSH - Flushes both input and output data on the terminal

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    (void) signal(SIGALRM, flag_time_out);

    //SEND SET COMMAND
    if (send_command(fd,set) != 1) printf("ERROR SENDING SET COMMAND\n");
    else printf("SET COMMAND SENT\n");

    //RECEIVE UA ANSWER
    if (read_answer(fd,ua) == 1) {
        printf("UA answer received\n");
        alarm(0);
    }
    else printf("ERRO ua\n");

    if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {     //set attributes again
        perror("tcsetattr");
        exit(-1);
    }


    close(fd);
    return 0;
}
