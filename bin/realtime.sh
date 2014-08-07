#!/usr/bin/env sh
# bin/realtime.sh
# Martin Miller
# Created: 2014/07/07
# Simulate real-time
DATA=raw/2nd
#BODY=data/bodyHist3.txt
DT=data/dt.fifo
ALT=data/alt.fifo
ACC=data/acc.fifo
QBW=data/qbw.fifo
ANGVEL=data/w.fifo

rm -f data/*.fifo
mkfifo $ALT 2>/dev/null
mkfifo $ACC 2>/dev/null
mkfifo $QBW 2>/dev/null
mkfifo $ANGVEL 2>/dev/null
mkfifo $DT 2>/dev/null

./bin/sensor-emu $DATA/alt | \
    stdbuf -eL -oL sed 's/[0-9]*,\(.*\),/\1/' | \
    stdbuf -eL -oL ./bin/multitap $ALT &

./bin/sensor-emu $DATA/ds/acc | \
    stdbuf -eL -oL sed 's/[0-9]*,//' | \
    stdbuf -eL -oL ./bin/fir config/gravity.txt | \
    stdbuf -eL -oL ./bin/rmbias -- -0.0171 0.0116 0.0158 > $ACC &

./bin/sensor-emu $DATA/ds/attitude | \
    stdbuf -eL -oL sed 's/[0-9]*,//' | \
    stdbuf -eL -oL ./bin/euler2qbw  > $QBW &

./bin/sensor-emu $DATA/gyro | \
    stdbuf -eL -oL sed 's/[0-9]*,//' | \
    stdbuf -eL -oL ./bin/rmbias -- 0.0006 0.0009 -0.0011 | \
    stdbuf -eL -oL ./bin/fir ./config/coeffs.txt | \
    stdbuf -eL -oL ./bin/multitap $ANGVEL &

./bin/sensor-emu $DATA/dt -d | \
    stdbuf -eL -oL sed 's/[0-9]*,//' > $DT &

FASTOPTS="data/bodyHist3.txt $ALT $ACC $DT $QBW $ANGVEL"
#valgrind --leak-check=full ./bin/slam $FASTOPTS
stdbuf -eL -oL ./bin/slam $FASTOPTS 
rm -f data/*.fifo


