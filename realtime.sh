#!/usr/bin/env sh
# bin/realtime.sh
# Martin Miller
# Created: 2014/07/07
# Simulate real-time
RTOPTS="data/bodyHist3.txt data/altHist.hex data/aHistF.hex data/dt.fifo data/qbwHistF.hex data/wHistF.hex"
FASTOPTS="data/bodyHist3.txt data/altHist.hex data/aHistF.hex data/dtHist.txt data/qbwHistF.hex data/wHistF.hex"
mkfifo data/dt.fifo 2>/dev/null
./bin/sensor-emu data/dtHist.txt >data/dt.fifo &
./bin/slam $FASTOPTS


