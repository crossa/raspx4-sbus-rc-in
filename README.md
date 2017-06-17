# raspx4-sbus-rc-in

# sbus 用户空间解析代码，适用于Linux、UNIX、各类嵌入式系统，包括nuttx。

# Mac OS下不可用，因为不支持非标串口通信。在不支持非标串口通信的主机上，安装Linux虚拟机，也无法使用此程序

# 使用方法

# $ git clone https://github.com/crossa/raspx4-sbus-rc-in

# $ cd raspx4-sbus-rc-in/sbus

# $ ./configure && make && make install

# $ sbus -d <你的串口设备地址> 



# 关于sbus如何转串口，请自行询问搜索引擎，或者去淘宝购买sbus转串口模块，很便宜。

# sbus的输入的遥控信号，使用我之前编写的ppmdecode可以读出并打印在屏幕的命令 ppmdecode P

# 之前的ppmdecode程序地址：https://github.com/crossa/raspberry-pi-ppm-rc-in

# 之前已经使用我编写的 ppmdecode程序的用户，请在切换到sbus驱动之前，先将飞行器的桨移除，通电后修改地面站的遥控器PWM相关参数，再飞行，否则会因为新的PWM值触发了自动解锁，会发生事故

# 普通遥控器使用sbus encode，sbus解析出来的PWM值范围为344～1710，具体数值请用我提供的ppmdecode查看
