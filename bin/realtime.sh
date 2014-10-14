#!/usr/bin/env sh
# bin/realtime.sh
# Martin Miller
# Created: 2014/07/07
# Simulate real-time
DATA=raw/clake_boat
BODY=data/bodyHist_9_30.txt
DT=data/dt.fifo
ALT=data/alt.fifo
ACC=data/acc.fifo
QBW=data/qbw.fifo
ANGVEL=data/w.fifo
DISP=data/disp.fifo
BODYFF=data/body.fifo

pkill slam
pkill sensor
pkill multitap
#rosrun rviz rviz -d config/rviz_display.rviz & 
rm -f data/*.fifo
mkfifo $ALT 2>/dev/null
mkfifo $ACC 2>/dev/null
mkfifo $QBW 2>/dev/null
mkfifo $ANGVEL 2>/dev/null
mkfifo $DT 2>/dev/null
mkfifo $DISP 2>/dev/null
#mkfifo $BODYFF 2>/dev/null

./bin/sensor-emu $DATA/alt | \
    stdbuf -eL -oL sed 's/[0-9]*,\(.*\),/\1/' | \
    stdbuf -eL -oL ./bin/multitap $ALT &

./bin/sensor-emu $DATA/acc | \
    stdbuf -eL -oL sed 's/[0-9]*,//' | \
    stdbuf -eL -oL ./bin/fir config/gravity.txt | \
    stdbuf -eL -oL ./bin/rmbias -- -0.0171 0.0116 0.0158 | \
    stdbuf -eL -oL ./bin/multitap $ACC &

./bin/sensor-emu $DATA/attitude | \
    stdbuf -eL -oL sed 's/[0-9]*,//' | \
    stdbuf -eL -oL ./bin/euler2qbw  | \
    stdbuf -eL -oL ./bin/multitap $QBW &

./bin/sensor-emu $DATA/gyro | \
    stdbuf -eL -oL sed 's/[0-9]*,//' | \
    stdbuf -eL -oL ./bin/rmbias -- 0.0006 0.0009 -0.0011 | \
    stdbuf -eL -oL ./bin/fir ./config/coeffs.txt | \
    stdbuf -eL -oL ./bin/multitap $ANGVEL &

./bin/sensor-emu $DATA/dt -d | \
    stdbuf -eL -oL sed 's/[0-9]*,//' > $DT &

./bin/sensor-emu $DATA/bodyHist | \
    stdbuf -eL -oL sed 's/[0-9]*,//' #> $BODYFF &

FASTOPTS="$BODYFF $ALT $ACC $DT $QBW $ANGVEL"
#valgrind --leak-check=full ./bin/slam $FASTOPTS
stdbuf -eL -oL ./bin/slam $FASTOPTS  #> $DISP & 

#rosrun using_markers display_realtime $DISP  
rm -f data/*.fifo


