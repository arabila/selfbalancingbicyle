#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <gpiod.h>


// Global Buffer 
char serialBuffer[10] = {0};  			// serial buffer for UART
int uart_fd;


int set_up_uart();
int write_bytes_uart(int fd, int count);
int read_bytes_uart(int fd, int count);
int failsave_exit();



int main(int argc, char *argv[])
{
  int uart=set_up_uart();
  while (true) {
    int ret=read_bytes_uart(uart,1);
    if (ret>=0) {
      printf("Gelesen %c \n",serialBuffer[0]);
    } else {
      printf("Fehler beim Lesen % \n",ret);
    }
    sleep(1);
  }
  return 0 ;
}

 /* ------------------------------------------------------------ *
 * @brief   sets up UART BBB P9.11,P9.13 -> /dev/ttyO4            
 * @param   uart_fd
 *          uart file descriptor
 * @return  uart_fd
 *          uart file descriptor
 * ------------------------------------------------------------ */



int set_up_uart(){

    char *port_Name = "/dev/ttyO4";							    // Name of the UART4 on BeagleBone
    struct termios options;                                 	// Port options


    uart_fd = open(port_Name, O_RDWR | O_NOCTTY | O_NONBLOCK); 				// Open port for read and write not making it a controlling terminal
    
    if (uart_fd == -1){
            perror("openPort: Unable to open port ");	
    }
    tcgetattr(uart_fd, &options);
    cfsetispeed(&options, B38400);                          	// Set baud rate to 38400
    cfsetospeed(&options, B38400);
    cfmakeraw(&options);
    tcflush(uart_fd, TCIFLUSH);
    tcsetattr(uart_fd, TCSANOW, &options);

    usleep(10000);                                          	// Sleep for UART to power up and set options

    return uart_fd;

}



/* ------------------------------------------------------------ *
 * @brief   reads var count number of bytes from the serial buffer           
 * @param   uart_fd
 *          uart file descriptor
 * @param   count
 *          int number of bytes to be written
 * @return  0 if process is successful 

 * ------------------------------------------------------------ */
int read_bytes_uart(int file_descriptor, int count) {
  static int failed_read=0;
  static int succesfull_read=0;
  static int longest_fail=0;
  int ret = read(file_descriptor, serialBuffer, count);
    if ( ret == -1) {		// Read back data into buf[]
    	// printf("Error: UART read failure in function read_bytes_uart");
        // perror("Error reading reading from UART");
        //close(file_descriptor);										// Close port if there is an error
      failed_read++;
      longest_fail++;
    } else {
      succesfull_read++;
      longest_fail=0;
    }
    printf(" succes %d failed %d longest_fail %d  ret %d \n",succesfull_read,
           failed_read,longest_fail,ret);
    if (longest_fail>10) {
      failsave_exit();
    }
    return ret;
}


  /* ------------------------------------------------------------ *
 * @brief   writes var count number of bytes into the serial buffer           
 * @param   uart_fd
 *          uart file descriptor
 * @param   count
 *          int number of bytes to be written
 * @return  0 if process is successful 
 * ------------------------------------------------------------ */
int write_bytes_uart(int file_descriptor, int count) {
    
    if ((write(file_descriptor, serialBuffer, count)) == -1) {	// Send data out
    	printf("Error: UART write failure in function write_bytes_uart");
        perror("Error writing on UART");
        close(file_descriptor);										// Close port if there is an error
        return(-1);
    }

    return 0;
}

int failsave_exit()
{
  close(uart_fd);
  printf("Failsave Exit \n  ...  uart closed\n   ...bye bye \n");
  exit(0);
}
