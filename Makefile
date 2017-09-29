.DEFAULT_GOAL = start

start:
	python track.py

push:
	rsync -a ./ pi@rpi-04:following-camera

stream:
	raspivid -o - \
	  -t 0 \
	  -w 320 \
	  -h 240 \
	  -fps 15 | cvlc stream:///dev/stdin --aout=alsa --sout '#standard{access=http,mux=ts,dst=:8160}' :demux=h264 :file-caching=200M -vvv

.PHONY: start push stream
