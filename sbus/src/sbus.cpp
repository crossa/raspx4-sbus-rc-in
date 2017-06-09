/*
 * sbus.cpp
 *
 *  Created on: 2017年6月2日
 *      Author: secondstupid
 */

#include "sbus.h"

//--------------------------------------------------------------------------------------------------//
Sbus::Sbus(char *_device, bool _all_channel){
	strcpy(this->_device,_device);
	this->_all_channel = _all_channel;
	this->max_channels_count=(_all_channel)?18:8;
	failsafe_status = SBUS_SIGNAL_OK;
	_device_fd=-1;
	_key=2048;
	_shmid=-1;
	channels_data=nullptr;
}
//--------------------------------------------------------------------------------------------------//
Sbus::~Sbus() {
	delete this->channels_data;
	close(this->_device_fd);
}
//--------------------------------------------------------------------------------------------------//
void Sbus::init() {
	//初始化共享内存
	if ((_shmid = shmget(_key, sizeof(uint16_t*) * max_channels_count,
	IPC_CREAT | 0666)) < 0) {
		perror("无法创建内存共享区\n");
		exit(0x01);
	}

	/*
	 * Now we attach the segment to our data space.
	 */
	void *p;
	if ((p = shmat(_shmid, NULL, 0)) == (void*) -1) {
		printf("无法映射共享内存区块\n");
		exit(0x02);
	}
	this->channels_data = (int16_t*) p;

	//开启设备
	_device_fd = open(_device, O_RDWR);
	if (-1 == _device_fd) {
		perror("无法打指定的开串口设备\n");
		exit(-1);
	}

	//设定串口设备
	_device_fd = open(_device, O_RDWR | O_NONBLOCK | O_CLOEXEC);
	if (_device_fd != -1) {
		printf("Opened SBUS input %s fd=%d\n", _device, (int) _device_fd);
		fflush(stdout);
		struct termios2 tio { };

		if (ioctl(_device_fd, TCGETS2, &tio) != 0) {
			close(_device_fd);
			_device_fd = -1;
			return;
		}
		tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR
				| ICRNL | IXON);
		tio.c_iflag |= (INPCK | IGNPAR);
		tio.c_oflag &= ~OPOST;
		tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
		tio.c_cflag &= ~(CSIZE | CRTSCTS | PARODD | CBAUD);
		// use BOTHER to specify speed directly in c_[io]speed member
		tio.c_cflag |= (CS8 | CSTOPB | CLOCAL | PARENB | BOTHER | CREAD);
		tio.c_ispeed = 100000;
		tio.c_ospeed = 100000;
		// see select() comment below
		tio.c_cc[VMIN] = 25;
		tio.c_cc[VTIME] = 0;
		if (ioctl(_device_fd, TCSETS2, &tio) != 0) {
			close(_device_fd);
			_device_fd = -1;
			return;
		}
	}

}

//--------------------------------------------------------------------------------------------------//
void Sbus::begin() {
	uint8_t loc_sbusData[25] = { 0x0f, 0x01, 0x04, 0x20, 0x00, 0xff, 0x07, 0x40,
			0x00, 0x02, 0x10, 0x80, 0x2c, 0x64, 0x21, 0x0b, 0x59, 0x08, 0x40,
			0x00, 0x02, 0x10, 0x80, 0x00, 0x00 };
	int16_t loc_channels[18] = { 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
			1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 0, 0 };
	int16_t loc_servos[18] = { 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023,
			1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 0, 0 };
	this->init();
	memcpy(_sbusData, loc_sbusData, 25);
	//memcpy(_channels, loc_channels, 18);
	//memcpy(_servos, loc_servos, 18);
	fflush(stdout);
	int nread;
	//开始解析SBUS数据
	while(1){
		fflush(stdout);
		nread = read(_device_fd, &_sbusData, sizeof(_sbusData));
		if(nread==25){
		    if(0x0f==_sbusData[0] && 0x00==_sbusData[24]){
			   this->UpdateChannels();
			   fflush(stdout);
			}
		}
		(this->_all_channel)?usleep(6700):usleep(4700);
	}
}
//--------------------------------------------------------------------------//
int16_t Sbus::Channel(uint8_t ch) {
	// Read channel data
	return ((ch > 0) && (ch <= 16))?_channels[ch - 1]:1023;
}
//--------------------------------------------------------------------------//
uint8_t Sbus::DigiChannel(uint8_t ch) {
	// Read digital channel data
	return ((ch > 0) && (ch <= 2))?_channels[15 + ch]:0;
}
//--------------------------------------------------------------------------//
uint8_t Sbus::Failsafe(void) {
	return failsafe_status;
}
//--------------------------------------------------------------------------//
void Sbus::UpdateChannels(void) {

	_channels[0] = ((_sbusData[1] | _sbusData[2] << 8) & 0x07FF);
	_channels[1] = ((_sbusData[2] >> 3 | _sbusData[3] << 5) & 0x07FF);
	_channels[2] = ((_sbusData[3] >> 6 | _sbusData[4] << 2 | _sbusData[5] << 10)
			& 0x07FF);
	_channels[3] = ((_sbusData[5] >> 1 | _sbusData[6] << 7) & 0x07FF);
	_channels[4] = ((_sbusData[6] >> 4 | _sbusData[7] << 4) & 0x07FF);
	_channels[5] = ((_sbusData[7] >> 7 | _sbusData[8] << 1 | _sbusData[9] << 9)
			& 0x07FF);
	_channels[6] = ((_sbusData[9] >> 2 | _sbusData[10] << 6) & 0x07FF);
	_channels[7] = ((_sbusData[10] >> 5 | _sbusData[11] << 3) & 0x07FF); // & the other 8 + 2 channels if you need them
	if (this->_all_channel) {
		_channels[8] = ((_sbusData[12] | _sbusData[13] << 8) & 0x07FF);
		_channels[9] = ((_sbusData[13] >> 3 | _sbusData[14] << 5) & 0x07FF);
		_channels[10] = ((_sbusData[14] >> 6 | _sbusData[15] << 2
				| _sbusData[16] << 10) & 0x07FF);
		_channels[11] = ((_sbusData[16] >> 1 | _sbusData[17] << 7) & 0x07FF);
		_channels[12] = ((_sbusData[17] >> 4 | _sbusData[18] << 4) & 0x07FF);
		_channels[13] = ((_sbusData[18] >> 7 | _sbusData[19] << 1
				| _sbusData[20] << 9) & 0x07FF);
		_channels[14] = ((_sbusData[20] >> 2 | _sbusData[21] << 6) & 0x07FF);
		_channels[15] = ((_sbusData[21] >> 5 | _sbusData[22] << 3) & 0x07FF);
	}

	//system("clear");
	int i;
	for (i = 0; i < 8; ++i) {
		printf("Channel %d: %d    ", i + 1, _channels[i]);
		if(i==7)
			printf("\n");
	}
	// DigiChannel 1
	/*if (sbusData[23] & (1<<0)) {
	 channels[16] = 1;
	 }
	 else{
	 channels[16] = 0;
	 }
	 // DigiChannel 2
	 if (sbusData[23] & (1<<1)) {
	 channels[17] = 1;
	 }
	 else{
	 channels[17] = 0;
	 }*/
	// Failsafe
	failsafe_status = SBUS_SIGNAL_OK;
	if (_sbusData[23] & (1 << 2)) {
		failsafe_status = SBUS_SIGNAL_LOST;
	}
	if (_sbusData[23] & (1 << 3)) {
		failsafe_status = SBUS_SIGNAL_FAILSAFE;
	}
	//if(SBUS_SIGNAL_OK==failsafe_status){
	memcpy(channels_data, _channels, sizeof(_channels));
	//}
}
//---------------------------------------------------------------------------------------//
