        .cdecls "pru_bike_speed.c"
        .clink
        .global start
start:
        LDI32   r3,  0x00000000
        ;LDI32   r15, 0x05f5e100         ; Set the max cycle count to be 1/2 s
                                        ; 100000000 Clock cycles
        LDI32   r15, 0x02625a00         ; 0.2 s
        LDI32   r27, 0x00022000         ; load r27 with the PRU0 control address
        LDI32   r17, 0x00000000         ; initialize reg to hold count.
        LDI32   r11, 0x00000000         ; initialize reg to hold count.
        LDI32   r10, 0x00000000         ; next value reg
        LDI32   r13, 0x00008000         ; value to find next value and pos of in pin
        LBBO    &r0, r27, 0, 4          ; load the value of the control register
        
        
PULSECOUNT:
        
        SBBO    &r11, r3, 0, 4          ; store the current pulse count in memory
        ADD     r17, r17, r11           ; add to the absolut total of pulse counts in this register
        SBBO    &r22, r3, 4, 4          ; store the times checked
        SBBO    &r17, r3, 8, 4          ; store total count in memory
        LDI32   r11, 0x00000000         ; current pulse count
        LDI32   r22, 0x00000000         ; how many times can I poll in the time

        
        CLR     r0, r0, 3               ; Set the CTR_EN bit to disable cycle counter
        SBBO    &r0, r27, 0, 4          ; store this new control value into the register
        SBBO    &r3, r27, 0xc, 4        ; store any value in cycle count to reset it.
        SET     r0, r0, 3               ; CTR_EN to enable counter
        SBBO    &r0, r27, 0, 4          ; write CTR_EN value back to start counter
        
CHECKPULSE:
        ADD     r22, r22, 1
        QBEQ    NOPULSE, r10, r31       ; check if input pin has changed
                                        ; if input and current value are not equal we count a change
        ADD     r11, r11, 1             ; inc pulsecount
        XOR     r10, r10, r13           ; set the next value to either 0 or 0x00008000
NOPULSE:

        LBBO    &r14, r27, 0xc, 4       ; store cycle count in r14
        QBGE    PULSECOUNT, r15, r14    ; Do a new pulsecount if more than 1/2 s elapsed
        JMP     CHECKPULSE

