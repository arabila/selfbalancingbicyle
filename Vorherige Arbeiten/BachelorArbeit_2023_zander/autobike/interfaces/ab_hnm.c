#include <stdio.h>

#include <fcntl.h>
#include <unistd.h>


#include "ab_hnm.h"
#include "ab_io.h"
#include "../ab_error_handling.h"

#define PULSES_PER_TURN         46
#define PRU_INPUT_PIN           "p8.15"
#define WHEEL_DIAMETER          0.635f       // the size of bike tire is 35 - 559, but measured seems to be about 63.5 cm.
#define PI                      3.1415926
#define MEMORY_ADDRESS          0x4A300000
#define CYCLE_TIME              0.2f

#define PRU_PATH                "/sys/class/remoteproc/remoteproc1/state"

#define MEMORY_PATH             "/dev/mem"
#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)


void hnm_device_open(hnm_fd_t *fd, char *pwm_chip, char *pwm_channel, char *pwm_pin)
{
    pwm_open(&fd->hnm_pwm, pwm_chip, pwm_channel, pwm_pin, 100000);
}

void hnm_disable(hnm_fd_t *fd){
    pwm_disable(&fd->hnm_pwm);
}

void hnm_enable(hnm_fd_t *fd){
    pwm_enable(&fd->hnm_pwm);
}


void hnm_set_pwm(hnm_fd_t *fd, int speed)
{
    pwm_set_duty_cycle(&fd->hnm_pwm, speed);
}


void hnm_device_close(hnm_fd_t *fd){
    hnm_disable(fd);
    pwm_close(&fd->hnm_pwm);
}

void hnm_enable_speed_reading(hnm_fd_t *fd){
    system("config-pin " PRU_INPUT_PIN " pruin");
    int temp_fd;
    if((temp_fd = open(PRU_PATH, O_WRONLY | O_NOCTTY)) < 0){
        AB_ERROR(-2, "Failed to open.", AB_ERR_ABORT);
    }
    if(write(temp_fd, "start", 5) != 5){
        AB_WARN(-2, "Failed to write start into pru status.", AB_ERR_CONTINUE);
    }
    fd->target = MEMORY_ADDRESS;
    if((fd->mem_fd = open(MEMORY_PATH, O_RDWR | O_SYNC)) == -1){
        AB_ERROR(-1, "Failed to open memory. Did you open with sudo rights?", AB_ERR_ABORT);
    }
    fd->map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd->mem_fd,
                    fd->target& ~MAP_MASK);
    if(fd->map_base == (void *) -1){
        AB_ERROR(-1, "Failed to map memory.", AB_ERR_ABORT);
    }
}

void hnm_disable_speed_reading(hnm_fd_t *fd){
    int temp_fd;
    if((temp_fd = open(PRU_PATH, O_WRONLY | O_NOCTTY)) < 0){
        AB_ERROR(-3, "Failed to open.", AB_ERR_ABORT);
    }
    if(write(temp_fd, "stop", 4) != 4){
        AB_WARN(-3, "Failed to write.", AB_ERR_CONTINUE);
    }
    if(munmap(fd->map_base, MAP_SIZE) == -1){
        AB_ERROR(0, "Failed to map memory.", AB_ERR_CONTINUE);
    }
    close(fd->mem_fd);
}

float hnm_speed(hnm_fd_t *fd){
    void *virt_addr;
    unsigned long value;
    float speed;
    virt_addr = fd->map_base + (fd->target & MAP_MASK);
    value = *((unsigned long *) virt_addr);
    speed = (((PI * WHEEL_DIAMETER) / PULSES_PER_TURN) * value) / CYCLE_TIME;
    return speed;
}

float hnm_distance(hnm_fd_t *fd){
    void *virt_addr;
    unsigned long value;
    float distance;
    virt_addr = fd->map_base + ((fd->target+0x8) & MAP_MASK);
    value = *((unsigned long *) virt_addr);
    distance = ((PI * WHEEL_DIAMETER) / PULSES_PER_TURN) * value;
    return distance;
}