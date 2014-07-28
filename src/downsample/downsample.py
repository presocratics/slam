#!/usr/bin/env python2.7
# slampp.py
# Martin Miller
# Created: 2014/07/11
# Downsamples sensor data to match camera rate.
from sys import argv

def get_framedata_timestamps( fn ):
    """Opens framedata and returns timestamps"""
    list=[]
    fh = open(fn, "r")
    for line in fh:
        line=line.strip()
        if (line == "" or line.startswith('#')):
            continue
        time,name = line.split(',')
        list.append(int(time))
    fh.close()
    return list

def get_file_list( fn ):
    """Reads in a data file and stores it in a list"""
    list = []
    fh = open(fn, "r")
    for line in fh:
        line=line.strip()
        if (line == "" or line[0].isalpha() ):
            continue
        time,data=line.split(',',1)
        list.append((int(time),data))
    fh.close()
    return list

def find_nearest( time, list ):
    """Returns entry in list with timestamp nearest to time"""
    delta=abs(time-list[0][0])
    val=list.pop(0)
    while abs(time-list[0][0])<delta:
        delta=abs(time-list[0][0])
        val=list.pop(0)
    return val

def main():
    if len(argv)<3:
        print("Usage: %s framedata data" % (argv[0]) )
        exit()
    """Get file inputs"""
    timestamps = get_framedata_timestamps(argv[1])
    """Open outputs for writing"""
    downsample_list = get_file_list(argv[2])
    for ts in timestamps:
        ds=find_nearest( ts, downsample_list );
        print("%d,%s" % (ds[0], ds[1]) )

if __name__=='__main__':
    main()

