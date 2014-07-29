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

./bin/downsample $DATA/framedata $DATA/alt | \
    cut -d, -f2 > $ALT &

./bin/downsample $DATA/framedata $DATA/acc | \
    cut -d, -f2,3,4 | \
    ./bin/fir config/gravity.txt | \
    ./bin/rmbias -- -0.0171 0.0116 0.0158 > $ACC &

./bin/downsample $DATA/framedata $DATA/attitude| \
    cut -d, -f2,3,4 | \
    ./bin/euler2qbw  > $QBW &

./bin/downsample $DATA/framedata $DATA/gyro| \
    cut -d, -f2,3,4 | \
    ./bin/rmbias -- 0.0006 0.0009 -0.0011 | \
    ./bin/fir ./config/coeffs.txt > $ANGVEL &

./bin/getdt $DATA/framedata| \
    cut -d, -f2 > $DT &

FASTOPTS="data/bodyHist3.txt $ALT $ACC $DT $QBW $ANGVEL"
#valgrind --leak-check=full ./bin/slam $FASTOPTS
./bin/slam $FASTOPTS 
rm -f data/*.fifo


