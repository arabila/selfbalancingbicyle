#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include "../interfaces/ab_hnm.h"

hnm_fd_t testi_hnm;

void safe_exit(){
    system("/var/lib/cloud9/fahrradprojekt/autobike/all_pin_default.sh");
    printf("Error occured, pins have been reset.");
    fflush(stdout);
}


void closing_all_devices(){
    printf("Closing of all devices started.");
    fflush(stdout);
    hnm_device_close(&testi_hnm);
    printf("all devices closed successfully.");
    fflush(stdout);
}

int main(int argc, char* argv[])
{
    if(atexit(safe_exit) != 0){
        printf("Exit handler failed.");
        fflush(stdout);
        return -1;
    }
    
    if (signal(SIGINT, closing_all_devices) == SIG_ERR){
        printf("Signal handler failed.");
        fflush(stdout);
        return -1;
    }

    hnm_device_open(&testi_hnm, "4", "0", "p9.14");

    printf("enableing speed reading\n");
    fflush(stdout);
    hnm_enable_speed_reading(&testi_hnm);
    sleep(1);
    printf("speed reading enabled\n");
    fflush(stdout);
        
    int pwm_value = 24000;

    // hnm_set_pwm(&testi_hnm, 31000);
    
    for(int i = 0;i<1000;i++){
        if(i%75==0){
    
            pwm_value += 2000;
            hnm_set_pwm(&testi_hnm, pwm_value);
            printf("pwm Wert: %d", pwm_value);
            fflush(stdout);
        }
        printf("speed of bike: %4.3f m/s; distance travelled: %4.3f m\n", hnm_speed(&testi_hnm), hnm_distance(&testi_hnm));
        fflush(stdout);
        usleep(200000);
    }
    hnm_disable_speed_reading(&testi_hnm);
    hnm_device_close(&testi_hnm);
    
    return 0;

}