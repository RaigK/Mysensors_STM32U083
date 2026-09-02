#!/bin/bash
cd "C:/claude/Mysensors_STM32U083"
OPENOCD="C:/Users/raigk/.platformio/packages/tool-openocd/bin/openocd.exe"
SCRIPTS="C:/Users/raigk/.platformio/packages/tool-openocd/openocd/scripts"
GDB="C:/Users/raigk/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gdb.exe"

"$OPENOCD" -s "$SCRIPTS" -f interface/stlink.cfg -f stm32u0x.cfg \
    -c "init; reset halt" &
OCDPID=$!
sleep 4

"$GDB" -batch -x debug_session.gdb 2>&1
STATUS=$?

kill $OCDPID 2>/dev/null
wait $OCDPID 2>/dev/null
exit $STATUS
