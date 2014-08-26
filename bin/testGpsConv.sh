#!/usr/bin/env sh
# testGpsConv.sh
# Hong-Bin Yoon
# Created:Hong-Bin Yoon
DATA=raw/clake
GPS=data/gps.fifo
mkfifo $GPS 2>/dev/null

./bin/sensor-emu $DATA/loc |\
    stdbuf -eL -oL sed 's/[0-9]*,\(.*\),/\1/' |\
    stdbuf -eL -oL ./bin/gps2xyz |\
    stdbuf -eL -oL ./bin/multitap $GPS &
