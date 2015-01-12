#!/usr/bin/env sh
# bin/realtime.sh
# Martin Miller
# Created: 2014/07/07
# Simulate real-time

###################
## Configuration ##
###################
#DATA=raw/clake2

DATA=raw/clake_boat/fastForC/txt
#DATA=raw/nov13
#BODY=$DATA/bodyHist
#BODY=$DATA/body2.txt
#DT=data/dt.fifo
#ALT=data/alt.fifo
#ACC=data/acc.fifo
#QBW=data/qbw.fifo
#ANGVEL=data/w.fifo
#DISP=data/disp.fifo
#BODY=data/body.fifo

rm -f data/*.fifo
#mkfifo $ALT 2>/dev/null
#mkfifo $ACC 2>/dev/null
#mkfifo $QBW 2>/dev/null
#mkfifo $ANGVEL 2>/dev/null
#mkfifo $DT 2>/dev/null
#mkfifo $DISP 2>/dev/null
#mkfifo $BODY 2>/dev/null

##############
## Testing  ##
##############

# run display 
#rosrun rviz rviz -d config/rviz_display.rviz & 

# data to fifo
#<$DATA/fastForC/bodyHist  stdbuf -eL -oL cut -d, -f2,3,4,5,6  > $BODY &
BODY=$DATA/bodyHist

#<$DATA/alt   stdbuf -eL -oL cut -d, -f2  > $ALT &
ALT=$DATA/alt

#<$DATA/fastForC/accFast stdbuf -eL -oL cut -d, -f1,2,3 > $ACC & #  stdbuf -eL -oL cut -d, -f2,3,4 | \
    #stdbuf -eL -oL ./bin/rmbias -- -0.0435 -0.0397 -0.0512 | \
    #stdbuf -eL -oL ./bin/rotate -e 0 0 p | \
    #stdbuf -eL -oL ./bin/fir config/gravity.txt  > $ACC &
ACC=$DATA/accFast

#<$DATA/fastForC/attitude stdbuf -eL -oL cut -d, -f2,3,4 | \
#    #stdbuf -eL -oL ./bin/rotate -e 0 0 p | \
#    stdbuf -eL -oL ./bin/euler2qbw  > $QBW &

#<$DATA/fastForC/attitude stdbuf -eL -oL ./bin/euler2qbw  > $QBW &
QBW=$DATA/qbwFast

#<$DATA/fastForC/gyro  stdbuf -eL -oL cut -d, -f2,3,4  > $ANGVEL & # | \
    #stdbuf -eL -oL ./bin/rmbias -- 0.0006 0.0009 -0.0011 | \
    #stdbuf -eL -oL ./bin/rotate -e 0 0 p | \
    #stdbuf -eL -oL ./bin/fir ./config/coeffs.txt  > $ANGVEL &
ANGVEL=$DATA/gyro

#<$DATA/fastForC/dt stdbuf -eL -oL cut -d, -f2  > $DT  &
#<$DATA/dt stdbuf -eL -oL cut -d, -f2  > $DT  &

DT=$DATA/dt


# run slam
FASTOPTS="$BODY $ALT $ACC $DT $QBW $ANGVEL"
#valgrind --leak-check=full --log-file="valgrind_log" ./bin/slam $FASTOPTS
stdbuf -eL -oL ./bin/slam $FASTOPTS 
#stdbuf -eL -oL ./bin/slam $FASTOPTS | tee  $DISP &

# slam data to display
#rosrun using_markers display_realtime $DISP  

rm -f data/*.fifo


