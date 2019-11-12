#!/bin/bash

echo "will compile video ${1}.mp4"

ffmpeg -r 24 -i frame_%5d.png -vcodec libx264 -y -an ${1}.mp4 -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2"

# clean up the directory
rm frame_*.png