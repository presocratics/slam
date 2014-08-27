#!/usr/bin/env sh
# bin/realtime.sh
# Martin Miller
# Created: 2014/07/07
# Simulate real-time

###################
## Configuration ##
###################
#DATA=raw/clake2
DATA=raw/2nd
BODY=data/bodyHist3.txt
#BODY=$DATA/body2.txt
DT=data/dt.fifo
ALT=data/alt.fifo
ACC=data/acc.fifo
QBW=data/qbw.fifo
ANGVEL=data/w.fifo
DISP=data/disp.fifo

rm -f data/*.fifo
mkfifo $ALT 2>/dev/null
mkfifo $ACC 2>/dev/null
mkfifo $QBW 2>/dev/null
mkfifo $ANGVEL 2>/dev/null
mkfifo $DT 2>/dev/null
mkfifo $DISP 2>/dev/null

##############
## Testing  ##
##############

# run display 
rosrun rviz rviz -d config/rviz_display.rviz & 

# data to fifo
<$DATA/ds/alt    stdbuf -eL -oL cut -d, -f2 | \
    stdbuf -eL -oL ./bin/rmbias -- -1.021 0 0 > $ALT &

    #stdbuf -eL -oL ./bin/rmbias -- -0.0171 0.0116 0.0158 > $ACC &
    #stdbuf -eL -oL ./bin/rmbias -- -0.0435 -0.0397 -0.0512 > $ACC &
<$DATA/ds/acc   stdbuf -eL -oL cut -d, -f2,3,4 | \
    stdbuf -eL -oL ./bin/rmbias -- -0.0435 -0.0397 -0.0512 | \
    stdbuf -eL -oL ./bin/rotate -e 0 0 p | \
    stdbuf -eL -oL ./bin/fir config/gravity.txt > $ACC &

<$DATA/ds/attitude stdbuf -eL -oL cut -d, -f2,3,4 | \
    stdbuf -eL -oL ./bin/rotate -e 0 0 p | \
    stdbuf -eL -oL ./bin/euler2qbw  > $QBW &

<$DATA/ds/gyro  stdbuf -eL -oL cut -d, -f2,3,4 | \
    stdbuf -eL -oL ./bin/rmbias -- 0.0006 0.0009 -0.0011 | \
    stdbuf -eL -oL ./bin/rotate -e 0 0 p | \
    stdbuf -eL -oL ./bin/fir ./config/coeffs.txt > $ANGVEL &

stdbuf -eL -oL ./bin/getdt $DATA/framedata| \
    stdbuf -eL -oL cut -d, -f2 > $DT &

# run slam
FASTOPTS="$BODY $ALT $ACC $DT $QBW $ANGVEL"
#valgrind --leak-check=full ./bin/slam $FASTOPTS
#stdbuf -eL -oL ./bin/slam $FASTOPTS 
stdbuf -eL -oL ./bin/slam $FASTOPTS | tee  $DISP &

# slam data to display
rosrun using_markers display_realtime $DISP  

rm -f data/*.fifo


