## Steering Control for a Self-Balancing-Bicycle

For [MD49](https://www.robot-electronics.co.uk/htm/md49tech.htm), [EMG49](https://www.robot-electronics.co.uk/htm/emg49.htm) and [BNO055](https://www.bosch-sensortec.com/products/smart-sensors/bno055).

The folder motortest contains the functions for motor control isolated, as does the folder imutest for the IMU. The main C file ´bicycle.c´ contains the main loop and ´demowrapper.c´ just wraps this loop - it was used in my presentation.

Before starting do not forget to run `pin-setup.sh` to set the GPIO for UART and I2C

If connected via ethernet, the BeagleBone Black should use **192.168.10.1**
User: debian
Pass: temppwd

It does not use dhcp and does not provide a dhcp.
Use static network configuration. 

### For more information have a look into the [Wiki](http://tocotronic.em.informatik.uni-frankfurt.de/arbeiten/ba_ranz/-/wikis/BeagleBone-Black)
- how to build for the [BeagleBone Black](http://tocotronic.em.informatik.uni-frankfurt.de/arbeiten/ba_ranz/-/wikis/BeagleBone-Black)
- how to implement functions of the [BNO055 IMU](http://tocotronic.em.informatik.uni-frankfurt.de/arbeiten/ba_ranz/-/wikis/BeagleBone-Black)
- if you want to have a look at some of the papers cited in my thesis you can find them [here](http://tocotronic.em.informatik.uni-frankfurt.de/arbeiten/ba_ranz/-/wikis/Used-Papers)
