# Ethercat_code

### Linux kernel change
Download all the relative files from the selected kernel

```
https://kernel.ubuntu.com/~kernel-ppa/mainline/
```
Install Libssl 1.1
```
wget http://archive.ubuntu.com/ubuntu/pool/main/o/openssl/libssl1.1_1.1.0g-2ubuntu4_amd64.deb
sudo dpkg -i libssl1.1_1.1.0g-2ubuntu4_amd64.deb
```
Install kernel  
```
sudo dpkg -i *.deb
sudo apt install net-tools
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install autoconf
sudo apt-get install -y libtool
sudo apt-get install -y pkg-config 
sudo apt-get install build-essential
```



Ethercat code for running novanta and Synapticon drives
```
./bootstrap
./configure --disable-8139too --enable-sii-assign --disable-8139too --enable-hrtimer --enable-cycles --enable-e1000e --prefix="/opt/etherlab" --disable-eoe

make all modules

sudo make modules_install install

sudo depmod

cd /etc
sudo mkdir sysconfig

sudo ln -s /opt/etherlab/etc/init.d/ethercat /etc/init.d/ethercat

sudo cp /opt/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat

sudo gedit /etc/sysconfig/ethercat

echo KERNEL==\"EtherCAT[0-9]*\", MODE=\"0664\" > /etc/udev/rules.d/99-EtherCAT.rules

sudo /etc/init.d/ethercat restart

```
Command to build and run
```
sudo gcc novanta_drive_working.c -I/usr/local/include -Wall /usr/local/lib/libethercat.a -pthread -lrt
sudo g++ sterile_engagement.cpp -pthread -lrt -o sterile_engagement
sudo ./a.out
sudo ./sterile_engagement
```

```
sudo g++ sterile_engagement.cpp -pthread -lrt -o sterile_engagement
sudo gcc novanta_drive_working.c -I/usr/local/include -Wall /usr/local/lib/libethercat.a -pthread -lrt
sudo g++ websocket_server_async.cpp -pthread -lrt -o websocket_server_async -I ./include


```
