#include <unistd.h>
#include <signal.h>
#include <stdio.h>

int running=0, acabar=0;

void atende()                   // atende alarme
{
    acabar=1;
}


int main()
{
    (void) signal(SIGALRM, atende);  // instala rotina que atende interrupcao - atende é chamado quando acaba o alarme

    while (!acabar) {
        if (!running) {
            alarm(3);  // activa alarme de 3s
            running=1;
        }

        printf("O meu nome é Rebeka!\n");
    }
    printf("Vou terminar.\n");

    return 0;
}
