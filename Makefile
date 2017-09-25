.DEFAULT_GOAL = start

start:
	python track.py

push:
	rsync -a ./ pi@rpi-04:following-camera
	
.PHONY: start push