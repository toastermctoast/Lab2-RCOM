/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#define BAUDRATE B9600
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define FLAG 0X5C
#define A 0X01
#define Cset 0X03
#define Cua 0X07
#define BCCset (A^Cset)
#define BCCua (A^Cua)


volatile int STOP=FALSE;
//função de envio do UA
void sentUA(unsigned char *UA, int *file){
		    int  env = write(file, UA, 5);
      printf("%d bytes written\n", env);
}

//maquina de estado de receção do set com o envio do UA no ultimo estado
void receiveset (unsigned char *BUFFER ,unsigned char *set, int *stat, int *file){
	int res;
    while (STOP==FALSE) {       /* loop for input */
      res = read(file,BUFFER,5);   /* returns after 5 chars have been input */
      while( i<res){
		  printf("Hello");
		 switch(stat){
			 
			  printf("%d\n", set[i]);
		   case 0:
		   printf("state0\n");
		   if(BUFFER[i]==FLAG){
			   set[i] = BUFFER[i];
			   i++;
			   stat=1;
		   }
		   else stat=0; 
		   break;
		   
		   
		   case 1:
		   printf("state1\n");
		   if(BUFFER[i]==A){
				set[i] = BUFFER[i];
				i++;
				stat=2;  
			}
			else if(BUFFER[i]==FLAG){
				stat=1;
			}
			else{
				stat=0;
				i--;
			}
			break;
		   
		   case 2:
		   printf("state2\n");
		   if(BUFFER[i]==Cset){
				set[i] = BUFFER[i];
				i++;
				stat=3;  
			}
			else if(BUFFER[i]==FLAG){
				stat=1;
				i--;
			}
			else {
				stat=0;
				i=0;
			}
			break;
			
		   
		   case 3:
		   printf("state3\n");
		   if(BUFFER[i]==BCCset){
				set[i] = BUFFER[i];
				i++;
				stat=4;  
			}
			else if(BUFFER[i]==FLAG){
				stat=1;
				i=1;
			}
			else {
				stat=0;
				i=0;
			}
			break;
		   
		   case 4:
		   printf("state4\n");
		   if(BUFFER[i]==FLAG){
				set[i] = BUFFER[i];
				i++;
				stat=5;  
			}
			else{
				stat=0;
				i=0;
			}
		   break;

			case 5:

		  	sentUA(Ua, file);
		  Break;
		  //printf("%d\n", set[i]);
	    }
     }

      
      BUFFER[res]= 0; 
      printf("%d\n", set[0]);
      printf("%d\n", set[1]); 
      printf("%d\n", set[2]); 
      printf("%d\n", set[3]); 
      printf("%d\n", set[4]);               /* so we can printf... */
}

int main(int argc, char** argv)
{
    int fd,c, i=0, state=0;
    struct termios oldtio,newtio;
    unsigned char buf[5];
	unsigned char Set[5];
	unsigned char Ua[5]=[FLAG, A,Cua , BCCua, FLAG];
    if ( (argc < 2) || 
  	     ((strcmp("/dev/ttyS0", argv[1])!=0) && 
  	      (strcmp("/dev/ttyS1", argv[1])!=0) )) {
      printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
      exit(1);
    }


  /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
  */
  
    
    fd = open(argv[1], O_RDWR | O_NOCTTY );
    if (fd <0) {perror(argv[1]); exit(-1); }

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
    leitura do(s) pr�ximo(s) caracter(es)
  */



    tcflush(fd, TCIOFLUSH);

    if ( tcsetattr(fd,TCSANOW,&newtio) == -1) {
      perror("tcsetattr");
      exit(-1);
    }

    printf("New termios structure set\n");

	receiveset(buf,Set,state, fd);
      printf(":%s:%d\n", Set, res);
     
      if (buf[0]=='z') STOP=TRUE;
      
    }
    
    



  /* 
    O ciclo WHILE deve ser alterado de modo a respeitar o indicado no gui�o 
  */

	sleep(1);

    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}


/*
volatile int STOP=FALSE;

int main(int argc, char** argv)
{
    int fd,c, res, i;
    struct termios oldtio,newtio;
    char buf[255];

    if ( (argc < 2) ||
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }


    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    *//*


    fd = open(argv[1], O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(argv[1]); exit(-1); }

    if (tcgetattr(fd,&oldtio) == -1) { /* save current port settings *//*
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // set input mode (non-canonical, no echo,...) 
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;   // inter-character timer unused 
    newtio.c_cc[VMIN]     = 1;   // blocking read until 1 chars received 

    /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) próximo(s) caracter(es)
    *//*


    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

     //RECEIVING STRING
    for(i=0; i<255;i++){
        read(fd,buf+i,1); //reads chars one by one
        if (buf[i] == 'z') break;
    }

    buf[i+1] = 0; //so we can printf
    printf("STRING: %s\n",buf);

    //SENDING IT BACK
    res = write(fd,buf,i+1);            //returns number of written bytes in the driver file and saves in res
    printf("%d bytes written\n", res);

    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
*/
