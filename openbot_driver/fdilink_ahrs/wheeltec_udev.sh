#CP2102 串口号0003 设置别名为fdilink_ahrs
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="fdilink_ahrs"' >/etc/udev/rules.d/wheeltec_fdi_imu_gnss.rules
#CH9102，同时系统安装了对应驱动 串口号0003 设置别名为fdilink_ahrs
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="fdilink_ahrs"' >/etc/udev/rules.d/wheeltec_fdi_imu_gnss2.rules
#CH9102，同时系统没有安装对应驱动 串口号0003 设置别名为fdilink_ahrs
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0003", MODE:="0777", GROUP:="dialout", SYMLINK+="fdilink_ahrs"' >/etc/udev/rules.d/wheeltec_fdi_imu_gnss3.rules
#CH340，直接设置别名为fdilink_ahrs
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout", SYMLINK+="fdilink_ahrs"' >/etc/udev/rules.d/wheeltec_fdcontroller_340.rules

service udev reload
sleep 2
service udev restart


