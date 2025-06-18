#include <stdio.h>
#include <unistd.h>

#include "interfaces/ab_i2c.h"
#include "interfaces/ab_bno055.h"
#include "interfaces/ab_bts7960.h"
#include "interfaces/ab_md49.h"
#include "interfaces/ab_hnm.h"
#include "ab_error_handling.h"



#define  DEGREE_FACTOR  16.0
// comment


i2c_fd_t bno055_autobike_init()
{

    i2c_fd_t fd = bno055_device_open("/dev/i2c-2", 0x28, "p9.19", "p9.20"); 
    
    bno055_set_page0(fd);
    bno055_set_mode(fd, 0x00);      // config mode
    
    // TODO: Check whether the next two statements are needed and wanted
    bno055_device_write_byte(fd, 0x41, 0x21);  // map axis orientation
    bno055_device_write_byte(fd, 0x42, 0x04);  // map axis sign

    uint8_t unit_sel = bno055_device_read_byte(fd, 0x38);
    unit_sel &= 0x7B;  // unit selection euler angles in degrees and fusion data output format to windows;
    bno055_device_write_byte(fd, 0x38, unit_sel);

    bno055_set_mode(fd, 0x08);      // imu mode
    
    return fd;
}

void safe_exit(){
    system("../all_pin_default.sh");
    //execl("/bin/sh", "sh", "/var/lib/cloud9/BeagleBone/Black/autobike/all_pin_default.sh", NULL);
    printf("pins have been reset.");
    fflush(stdout);
}

int main(int argc, char* argv[])
{
    
    // all necessary file descriptors
    hnm_fd_t hnm_fd;
    i2c_fd_t bno055_fd;
    f_euler_degrees_t euler_angles;
    uart_fd_t md49_fd;
    bts7960_fd_t bts_fd;

    // opening all the connections to the pins.
    bno055_fd = bno055_autobike_init();
    md49_fd = md49_device_open("/dev/ttyO4", "p9.11", "p9.13");
    bts7960_device_open(&bts_fd, "115", "49", "1", "1", "p9.22", "1", "0", "p9.21");
    hnm_device_open(&hnm_fd, "4", "0", "p9.14");
    hnm_set_pwm(&hnm_fd, 0);
    hnm_enable_speed_reading(&hnm_fd);

    // resetting the bno055 and setting the encoder values of the md49 to 0
    bno055_fd = bno055_soft_reset(bno055_fd);
    md49_device_reset(md49_fd);

    // closing all file descriptors
    hnm_device_close(&hnm_fd);
    bts7960_device_close(&bts_fd);
    md49_device_close(md49_fd);
    bno055_device_close(bno055_fd);
    hnm_disable_speed_reading(&hnm_fd);
    
    printf("All devices closed.\n");
    fflush(stdout);
    
    sleep(1);
    // putting all pins into gpio mode in order to turn off all pwm signals.
    safe_exit();
    printf("\n\nRESET SUCCESSFUL.\n");
}