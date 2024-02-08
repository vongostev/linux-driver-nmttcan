sudo apt-get install make gcc-11
make
sudo rmmod nmttcan
sudo make install
sudo modprobe nmttcan
sudo dmesg
