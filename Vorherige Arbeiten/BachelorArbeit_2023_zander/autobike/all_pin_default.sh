# this will set all pins to default. GPIO pins will keep their value,
# but pwm pins where pwm is not default will be turned off.

#for i in $(seq 0 45); do
#    config-pin P8.$i gpio
#    config-pin P9.$i gpio
#done

logfile="all_pin_default.log"
if [ -e "$logfile" ]; then
    rm "$logfile"
fi

date >> "$logfile" 2>&1
config-pin P8.10 gpio >> "$logfile" 2>&1
config-pin P8.11 gpio >> "$logfile" 2>&1
config-pin P8.12 gpio >> "$logfile" 2>&1
config-pin P8.13 gpio >> "$logfile" 2>&1
config-pin P8.14 gpio >> "$logfile" 2>&1
config-pin P8.15 gpio >> "$logfile" 2>&1
config-pin P8.16 gpio >> "$logfile" 2>&1
config-pin P8.17 gpio >> "$logfile" 2>&1
config-pin P8.18 gpio >> "$logfile" 2>&1
config-pin P8.19 gpio >> "$logfile" 2>&1
config-pin P8.26 gpio >> "$logfile" 2>&1

config-pin P9.11 gpio >> "$logfile" 2>&1
config-pin P9.12 gpio >> "$logfile" 2>&1
config-pin P9.13 gpio >> "$logfile" 2>&1
config-pin P9.14 gpio >> "$logfile" 2>&1
config-pin P9.15 gpio >> "$logfile" 2>&1
config-pin P9.16 gpio >> "$logfile" 2>&1
config-pin P9.17 gpio >> "$logfile" 2>&1
config-pin P9.18 gpio >> "$logfile" 2>&1
config-pin P9.19 gpio >> "$logfile" 2>&1
config-pin P9.20 gpio >> "$logfile" 2>&1
config-pin P9.21 gpio >> "$logfile" 2>&1
config-pin P9.22 gpio >> "$logfile" 2>&1
config-pin P9.23 gpio >> "$logfile" 2>&1
config-pin P9.24 gpio >> "$logfile" 2>&1
config-pin P9.26 gpio >> "$logfile" 2>&1
config-pin P9.27 gpio >> "$logfile" 2>&1
config-pin P9.30 gpio >> "$logfile" 2>&1
config-pin P9.41 gpio >> "$logfile" 2>&1
config-pin P9.42 gpio >> "$logfile" 2>&1

echo "Pin-Modes reset to GPIO"