#!/usr/bin/env sh
# bin/realtime.sh
# Martin Miller
# Created: 2014/07/07
# Simulate real-time
DATA=raw/clake2
BODY=data/bodyHist3.txt
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

<$DATA/ds/alt    stdbuf -eL -oL cut -d, -f2 > $ALT &

<$DATA/ds/acc   stdbuf -eL -oL cut -d, -f2,3,4 | \
    stdbuf -eL -oL ./bin/fir config/gravity.txt | \
    stdbuf -eL -oL ./bin/rmbias -- -0.0171 0.0116 0.0158 > $ACC &

<$DATA/ds/attitude stdbuf -eL -oL cut -d, -f2,3,4 | \
    stdbuf -eL -oL ./bin/euler2qbw  > $QBW &

<$DATA/ds/gyro  stdbuf -eL -oL cut -d, -f2,3,4 | \
    stdbuf -eL -oL ./bin/rmbias -- 0.0006 0.0009 -0.0011 | \
    stdbuf -eL -oL ./bin/fir ./config/coeffs.txt > $ANGVEL &

stdbuf -eL -oL ./bin/getdt $DATA/framedata| \
    stdbuf -eL -oL cut -d, -f2 > $DT &

BODY=/home/marty/ARC/git/segment2/output/clake2.arc
FASTOPTS="$BODY $ALT $ACC $DT $QBW $ANGVEL"
#valgrind --leak-check=full ./bin/slam $FASTOPTS
stdbuf -eL -oL ./bin/slam $FASTOPTS 
rm -f data/*.fifo


