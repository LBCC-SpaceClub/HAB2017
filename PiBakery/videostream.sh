#!/bin/bash

# Begins streaming video from the attached raspberry pi camera via RSTP
# Saves a copy of the recorded video in /home/pi/videos/<incrementing filename>

filename=vid001
if [[ -e $filename.h264 ]] ; then
    i=0
    while [[ -e $filename-$i.h264 ]] ; do
        let i++
    done
    name=$filename-$i.h264
fi
# Not sure if this command is right, see existing pi code
#raspivid -o - -t 0 | cvlc -v stream:///dev/stdin --sout '#rtp{sdp=rtsp://:8554/}' :demux=h264
#raspivid -o - -t 0 | tee "$filename".h264 | cvlc -v stream:///dev/stdin --sout '#standard{access=http,mux=ts,dest=:8080' :demux=h26

echo $name


#raspivid -o - -t 0 -hf -w 1920 -h 1080 -fps 25 | \
#tee test_video.h264 | \
#cvlc -vvv stream:///dev/stdin --sout '#rtp{sdp=rtsp://:8080}' :demux=h264

# Alternately, see picamera (http://picamera.readthedocs.io/en/release-1.10/recipes1.html)
# for tips including specifying encoding quality as a bit rate.. could be useful!
