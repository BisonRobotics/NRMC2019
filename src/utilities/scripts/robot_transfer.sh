#!/bin/bash
rsync -rav -e ssh --exclude='.git/' --exclude='.git*' --exclude='.idea/' \
--exclude='*.bag' --exclude='*.zip' --exclude='*.tar.gz' --exclude='*.tar.bz2' \
--exclude='build/' --exclude='devel/' --exclude='src/CMakeLists.txt' --exclude='cmake-build-debug/' \
~/NRMC2019/ nrmc@192.168.1.10:/home/nrmc/NRMC2019
#~/NRMC2019/ nrmc@up2:/home/nrmc/NRMC2019
