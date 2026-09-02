set pagination off
set print pretty on
target remote localhost:3333
file .pio/build/stm32u083/firmware.elf
echo \n--- CPU State ---\n
info registers
echo \n--- Exception frame (stacked regs at MSP=0x20009ea0) ---\n
x/8wx 0x20009ea0
echo \n--- Stacked PC (faulting address) is at MSP+0x18 ---\n
x/1wx 0x20009eb8
echo \n--- Backtrace ---\n
bt
echo \n--- Full backtrace with locals ---\n
bt full
echo \n--- Stack memory around MSP ---\n
x/32wx 0x20009e80
quit
