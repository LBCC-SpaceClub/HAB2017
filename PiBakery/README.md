A few reminders while learning how this all works:

raspivid -o - -t 0 -n | cvlc -vvv stream:///dev/stdin --sout '#rtp{sdp=rtsp://:8080/}' :demux=h264

To run pibakery in linux: browse to git/pibakery
sudo npm start

Add folders using ctrl shift +

The pi image Levi created on 2/3/17 has been update/upgraded, and has avconv and vlc installed.
