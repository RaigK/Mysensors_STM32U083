set pagination off
set print pretty on
target remote localhost:3333
file .pio/build/stm32u083/firmware.elf

echo \n=== Connected. Reading current fault state ===\n

# Print all registers
info registers

echo \n=== Exception frame at MSP (Cortex-M0+ stacking: R0,R1,R2,R3,R12,LR,PC,xPSR) ===\n
# MSP = 0x20009ea0
x/8wx 0x20009ea0

echo \n=== Faulting PC is at MSP+0x18 = 0x20009eb8 ===\n
x/1wx 0x20009eb8

echo \n=== LR (return addr) is at MSP+0x14 = 0x20009eb4 ===\n
x/1wx 0x20009eb4

echo \n=== Stack walk (above exception frame) ===\n
x/32wx 0x20009ec0

# Now set a breakpoint at the HardFault handler so we catch the next fault too
# and can decode the backtrace from a fresh boot
echo \n=== Setting breakpoint at HardFault_Handler for next run ===\n
break *0x0800cfcc
commands 1
  silent
  echo \n>>> HARDFAULT CAUGHT <<<\n
  info registers
  echo \n>>> Stacked frame (faulting context): <<<\n
  x/8wx $sp
  echo \n>>> Faulting PC (sp+0x18): <<<\n
  x/1wx (unsigned int*)($sp+0x18)
  echo \n>>> LR (sp+0x14): <<<\n
  x/1wx (unsigned int*)($sp+0x14)
  bt
end

echo \n=== Done reading current state. Resetting to catch live fault... ===\n
monitor reset halt
continue
