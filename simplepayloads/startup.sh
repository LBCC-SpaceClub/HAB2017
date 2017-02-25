#!/bin/bash

DATE=$(date +"%Y-%m-%d_%H%M")

# -o <output filename>
# -t <record time in ms>
# -md <standard resolutions, 1=, 2=
# -n (no preview)
# -fps <framerate>
# -w <width> default 1920
# -h <height> default 1080

raspivid -o $DATE.h264 -t 5400000 -md 2 -n

