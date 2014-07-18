#!/usr/bin/env sh
# bin/realtime.sh
# Martin Miller
# Created: 2014/07/07
# Simulate real-time
#RTOPTS="data/bodyHist3.txt data/altHist.hex data/aHistF.hex data/dt.fifo data/qbwHistF.hex data/wHistF.hex"
#FASTOPTS="data/bodyHist3.txt data/altHist.hex data/aHistF.hex \
#data/dtHist.txt data/qbwHistF.hex data/wHistF.hex"
#FASTOPTS="data/bodyHist4.txt data/altHist.fifo data/aHistF.hex \
#data/dtHist.txt data/qbwHistF.hex data/wHistF.hex"
#./bin/slam $FASTOPTS

#DATA=/home/marty/ARC/data/Nov2720131205
DATA=/home/marty/ARC/data/2nd
SLAMPP=/home/marty/ARC/slampp/output
#
#BODY=data/bodyHist3.txt
#DT=data/dt.fifo
#ALT=data/alt.fifo
#ACC=data/acc.fifo
#QBW=data/qbw.fifo
ANGVEL=data/w.fifo
#
killall -9 multitap
rm -f data/*.fifo
#mkfifo $ALT 2>/dev/null
#mkfifo $ACC 2>/dev/null
#mkfifo $QBW 2>/dev/null
mkfifo $ANGVEL 2>/dev/null
#mkfifo $DT 2>/dev/null
#
#./bin/sensor-emu $DATA/alt|stdbuf -oL -eL sed 's/[0-9]*,//'| \
#stdbuf -oL -eL sed 's/,$//' | \
#stdbuf -oL -eL ./bin/multitap $ALT &
#
#./bin/sensor-emu $SLAMPP/accfiltbias2|stdbuf -oL -eL sed 's/[0-9]*,//'| \
#    stdbuf -oL -eL ./bin/multitap $ACC &
#
#./bin/sensor-emu $DATA/attitude|stdbuf -oL -eL sed 's/[0-9]*,//'| \
#    stdbuf -oL -eL ./bin/euler2qbw | \
#    stdbuf -oL -eL ./bin/multitap $QBW &
#
./bin/sensor-emu $SLAMPP/gyro.tr|stdbuf -oL -eL sed 's/[0-9]*,//'| \
    stdbuf -oL -eL ./bin/multitap $ANGVEL &
#
## no multitap because all samples needed
#./bin/sensor-emu $DATA/dt d |stdbuf -oL -eL sed 's/[0-9]*,//' > $DT &
#
FASTOPTS="data/bodyHist3.txt data/altHist.hex data/aHistF.hex \
data/dtHist.txt data/qbwHistF.hex"
#valgrind --leak-check=full ./bin/slam $FASTOPTS $ANGVEL || killall -9 multitap
./bin/slam $FASTOPTS $ANGVEL || killall -9 multitap
rm -f data/*.fifo


