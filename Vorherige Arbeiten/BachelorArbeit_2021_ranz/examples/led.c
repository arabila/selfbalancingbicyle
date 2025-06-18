#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <gpiod.h>
#include <signal.h>
#include <syslog.h>
//#include <iostream>


// Doku :
// von der Kommandozeile aus geht's auch so:
// apt install libgpiod-dev libgpiod-doc
// gpioinfo
// gpioset --mode=signal --background gpiochip1 12=1
// gpioget gpiochip1 12

// failhandler deklarieren:
void failhandler(int sig);

// globale Variablen, damit der failsave handler sie findet
struct gpiod_chip *gpiochip;
struct gpiod_line *gpio12, *gpio13;

int main(int argc, char **argv)
{

  if (signal(SIGTERM,failhandler) == SIG_ERR)  // kill <pid>
  {
    printf("Installation of Sighandler failhandler for SIGHUP failed\n");
    exit(1);
  }
  if (signal(SIGINT,failhandler) == SIG_ERR)   // Strg-C 
  {
    printf("Installation of Sighandler failhandler for SIGINT failed\n");
    exit(1);
  }

  // if (signal(SIGTERM,failhandler) == SIG_ERR)
  // {
  //   printf("Installation of Sighandler failhandler for SIGTERM failed\n");
  //   exit(1);
  // }

  //	::gpiod::chip chip(1);
  gpiochip = gpiod_chip_open("/dev/gpiochip1");
  if (gpiochip == NULL) {
    fprintf(stderr,"Error opening Chip \n");
    exit(1);
  }
  gpio12 = gpiod_chip_get_line(gpiochip,12);
  if (gpio12 == NULL) {
    fprintf(stderr,"Error opening Line 12 \n");
    exit(1);
  }
  gpio13 = gpiod_chip_get_line(gpiochip,13);
  if (gpio13 == NULL) {
    fprintf(stderr,"Error opening Line 13 \n");
    exit(1);
  }
  printf("gpiochip and line open is ok\r\n");
  int ret = gpiod_line_request_output(gpio13, "gpio", 0);
  if (ret != 0) {
    fprintf(stderr,"Error opening Line 12 (P8 - Pin 12 GPIO 44 for writing \n");
    exit(1);
  }
  ret = gpiod_line_request_output(gpio12, "gpio", 0);
  if (ret != 0) {
    fprintf(stderr,"Error opening Line 12 (P8 - Pin 12 GPIO 44 for writing \n");
    exit(1);
  }

  gpiod_line_set_value(gpio13,1);

  for(int i=0;i<1000000;i++) {
    // printf("toogle one\r\n");
    gpiod_line_set_value(gpio12,1);
    // sleep(1);
    // printf("toogle zero\r\n");
    gpiod_line_set_value(gpio12,0);
    // sleep(1);
  } // diese Schleife kann in 3 Sekunden 1 million mal ausgefuehrt werden
    // => richtig schnell
  

  gpiod_line_release(gpio13);
   gpiod_line_release(gpio12);
 
}
    



void failhandler(int sig) {

  
  fprintf(stdout,"Signal %d caught. Shuttdingdown io... \n",sig);

  // here is the code for shutting down all uarts i2c motors and so on 

  gpiod_line_set_value(gpio13,0);
  gpiod_line_set_value(gpio12,0);
  gpiod_line_release(gpio13);
  gpiod_line_release(gpio12);

  sleep(1);
  
  fprintf(stderr, "... done. Exiting \n");

  exit(sig);

}
