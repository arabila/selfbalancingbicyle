unter /sys/devices/platform/ocp sind die konfigurierbaren pins sichtbar

wenn da einer nicht auftacuth wie P9_24 dann ist der schon fest vorgegeben

uboot_overlay_addr0=/lib/firmware/BB-UART1-00A0.dtbo
fuer UART4 geht das auch: P11 und P13 
uboot_overlay_addr3=/lib/firmware/BB-UART4-00A0.dtbo
in diesem Fall durch das overlay in /boot/uEnv.txt


wenn unter  /sys/devices/platform/ocp die konfigurierbaren pins sichtbar sind,
kann man diese  mit

config-pin -q p9.24 auslesen 
config-pin -l p9.24 oder
mit
config-pin p9.24 uart
umschreiben

wenn die Pins mit einem overlay über uEnv-txt umgebogen sind,
kann man die nicht mehr mit config-pin lesen und setzen.

###################
kontrollieren kann man das mit gpioinfo dort steht P9_24 als UART1 drin.
das gpioinfo schreibt diesen kram nicht mit  scheint komisch zu sein


--------------


uart debuggen:
in uEnv.txt eingebaut :
uboot_overlay_addr0=/lib/firmware/BB-UART1-00A0.dtbo
#### UART 1 ist an P9_24 und P9_26 und die sind dann mit config-pin nicht mehr konfigurierbar
uboot_overlay_addr3=/lib/firmware/BB-UART4-00A0.dtbo
### UART4 hat P9_11 und P9_13 als Pins.

konnte mit
screen /dev/tty01 38400
screen /dev/tty04 38400
debuggt werden 
