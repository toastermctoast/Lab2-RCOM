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

#define BAUDRATE B9600
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

int timed_out = 0, timer_on = 0;

int send_command(int fd, unsigned int* command){

    int res;

    res = write(fd,command,FRAME_SIZE);            //returns number of written bytes in the driver file and saves in res
    printf("%d BYTES SENT\n", res);

    if (res == 5) return 1;

}

void flag_time_out(){ timed_out=1; }

int read_answer(int fd, unsigned int* answer, unsigned int* recv){

    int FLAG_ANSWER = answer[0];
    int A_ANSWER = answer[1];
    int C_ANSWER = answer[2];
    int BCC_ANSWER = answer[3];

    //Estados
    typedef enum {
        START,
        FLAG_RCV,
        A_RCV,
        C_RCV,
        BCC_OK,
        STOP
    } statesSET;

    statesSET state;
    int i;
    
    for(i=0; i<FRAME_SIZE;i++){

        if (!timer_on) {
            alarm(3);  // activa alarme de 3s
            timer_on=1;
        }

        if (timed_out) break;

        read(fd,recv+i,1); //reads chars one by one

        printf("BYTE LIDO %d\n", recv[i]);

        switch(state){

		    case START:
		        printf("start\n");
		        if(recv[i]==FLAG_ANSWER){
			        state=FLAG_RCV;
		        }
		        else state=START; 
		    break;
		   
            case FLAG_RCV:
		        printf("flag_rcv\n");
		        if(recv[i]==A_ANSWER) state = A_RCV;
		        if (recv[i] == FLAG_ANSWER) state = FLAG_RCV;
                else state = START;
		    break;

            case A_RCV:
		        printf("a_rcv\n");
                if(recv[i]==C_ANSWER) state = C_RCV;
		        if(recv[i] == FLAG_ANSWER) state = FLAG_RCV;
                else state = START;
		    break;

            case C_RCV:
		        printf("c_rcv\n");
                if(recv[i]==BCC_ANSWER) state = BCC_OK;
		        if(recv[i] == FLAG_ANSWER) state = FLAG_RCV;
                else state = START;
		    break;

            case BCC_OK:
		        printf("bcc_ok\n");
                if(recv[i] == FLAG_ANSWER) state = STOP;
                else state = START;
		    break;

            case STOP:
		        printf("stop\n");
		    break;

		   
        }
        if (state==STOP) break;
    }
    recv[i+1] = 0; //so we can printf
    printf("COMMAND RECEIVED: %s\n",recv);

    return 1;
}

int main(int argc, char** argv){
    int fd,c;
    struct termios oldtio,newtio;
    int i, sum = 0, speed = 0;
if(TRUE){
    if ( (argc < 2) || ((strcmp("/dev/ttyS0", argv[1])!=0) &&  (strcmp("/dev/ttyS1", argv[1])!=0) )) {
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


}

    /*-----------------------------------------------*/
    /*-------------------TRABALHO--------------------*/
    /*-----------------------------------------------*/



    //SEND SET COMMAND
    unsigned int set[FRAME_SIZE] =  {FLAG, A, C_SET, BCC_SET, FLAG};
    if (send_command(fd,set) != 1) printf("ERROR SENDING %s COMMAND\n",set);

    //RECEIVE UA ANSWER
    unsigned int ua[FRAME_SIZE] = {FLAG, A, C_UA, BCC_UA, FLAG};
    unsigned recv[FRAME_SIZE];

    (void) signal(SIGALRM, flag_time_out);  //flag_time_out é chamado quando acaba o tempo
    
    while(read_answer(fd,ua,recv) != 1){

        if (timed_out){
            send_command(fd,set);
            printf("Retransmitting command...\n");
        }
        else {
            printf("Unknown error reading answer\n");
            return -1;
        }
    }

    printf("Correct answer received.\n");
    

    if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {     //set attributes again
        perror("tcsetattr");
        exit(-1);
    }


    close(fd);
    return 0;
}
