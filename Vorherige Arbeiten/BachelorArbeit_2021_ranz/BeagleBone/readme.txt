Beagle bone
 PRUs:
   - koennen einen Timer benutzen (siehe hier https://community.element14.com/products/devtools/single-board-computers/next-genbeaglebone/b/blog/posts/beaglebone-pru---timer-functionality )
   - koennen eine eigene dedizierte UART benutzen
   - I2C muss von hand reinprogrammiert werden, gibt's aber in Projekten
       z.B. hier:  https://github.com/chanakya-vc/PRU-I2C_SPI_master/wiki
   - Können auch eine PWM erzeugen uebr ein eCAP Modul
       ob nur 1 oder 3 weiss ich nicht. 
   - haben 200MHz Clock
   - haben zwischen PRU1 und 2 einen shared memory

   - Kommunikation mit PRU über RemotePRocMEssaging

Reference Manual (5000 Seiten) aber mit den Details zu den PRUs usw.
https://www.ti.com/lit/ug/spruh73q/spruh73q.pdf



gute doku: https://beagleboard.org/static/prucookbook/

Motorsteuerung mit PWM und PRU und remote Prog MEssaging:
https://github.com/Greg-R/pru-pid-motor/blob/master/doc/PRUPIDMOTORlatex/pru-pid.pdf


