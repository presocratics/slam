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

./bin/downsample $DATA/framedata $DATA/alt | sed 's/[0-9]*,//'| \
    sed 's/,$//' > $ALT &

./bin/downsample $DATA/framedata $DATA/acc |  sed 's/[0-9]*,//'| \
    ./bin/fir config/98.txt | \
    ./bin/rmbias -0.0171 0.0116 0.0158 > $ACC &

./bin/downsample $DATA/framedata $DATA/attitude| sed 's/[0-9]*,//'| \
    ./bin/euler2qbw  > $QBW &

./bin/downsample $DATA/framedata $DATA/gyro| \
    sed 's/[0-9]*,//' | \
    ./bin/rmbias 0.0006 0.0009 -0.0011 | \
    ./bin/fir ./config/coeffs.txt > $ANGVEL &

cat data/dtHist.txt > $DT &
## no multitap because all samples needed
#./bin/sensor-emu $DATA/dt d |stdbuf -oL -eL sed 's/[0-9]*,//' > $DT &

FASTOPTS="data/bodyHist3.txt $ALT $ACC $DT $QBW $ANGVEL"
#valgrind --leak-check=full ./bin/slam $FASTOPTS $ANGVEL 
./bin/slam $FASTOPTS 
rm -f data/*.fifo


