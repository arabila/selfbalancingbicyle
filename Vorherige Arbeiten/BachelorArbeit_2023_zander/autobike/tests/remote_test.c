#include <stdio.h>
#include <unistd.h>

#include "../interfaces/ab_remote.h"

int main(int argc, char* argv[])
{

    remote_fd_t test_remote = remote_open("1", "0");
    printf("got opened\n");
    fflush(stdout);
    int remote_val;
    char* steer_val;
    char* throt_val;
    for(;;){
        remote_val = remote_get_steering(&test_remote);
        switch (remote_val){
            case 1:
                steer_val = "left    ";
                break;
            case 2:
                steer_val = "right   ";
                break;
            case 0:
                steer_val = "straight";
                break;
            default:
                steer_val = "error   ";
        }
        remote_val = remote_get_throttle(&test_remote);
        switch (remote_val){
            case 1:
                throt_val = "forward";
                break;
            case 2:
                throt_val = "reverse";
                break;
            case 0:
                throt_val = "stop   ";
                break;
            default:
                throt_val = "error  ";
        }
        printf("the bike is going %s\t, with speed\t %s\n", steer_val, throt_val);
        fflush(stdout);
        usleep(75000);
    }
    
    
    
    return 0;
}