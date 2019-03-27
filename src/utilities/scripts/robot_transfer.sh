#!/bin/bash
rsync -rav -e ssh --exclude='.git/' --exclude='.git*' --exclude='.idea/' \
--exclude='build/' --exclude='devel/' --exclude='src/CMakeLists.txt' --exclude='cmake-build-debug/' \
~/NRMC2019/ nrmc@up2:/home/nrmc/NRMC2019
