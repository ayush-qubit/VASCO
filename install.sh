#!/bin/bash
if [[ -d "build" ]]; then
	#statements
	rm -r build
fi
mkdir build
cd build
cmake .. && make
sudo make install