# Ethercat_code
Ethercat code for running novanta and Synapticon drives

./configure --disable-8139too --enable-sii-assign --disable-8139too --enable-hrtimer --enable-cycles --enable-e1000e --prefix="/opt/etherlab" --disable-eoe

make all modules

sudo make modules_install install

sudo depmod

sudo ln -s /opt/etherlab/etc/init.d/ethercat /etc/init.d/ethercat

sudo cp /opt/etherlab/etc/sysconfig/ethercat /etc/sysconfig/ethercat

sudo gedit /etc/sysconfig/ethercat

sudo gcc novanta_drive.c -I/usr/local/include -Wall /usr/local/lib/libethercat.a
