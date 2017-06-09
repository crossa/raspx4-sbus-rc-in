/*
 * sbus.cpp
 *
 *  Created on: 2017年6月2日
 *      Author: secondstupid
 */

#include "sbus.h"

//--------------------------------------------------------------------------------------------------//
Sbus::Sbus(char *_device, bool _all_channel) :
		max_channels_count(18), _key(2048) {
	memcpy(this->_device, "/dev/ttyUSB0", sizeof("/dev/ttyUSB0"));
	this->_all_channel = _all_channel;
	this->channels_data = new int16_t[this->max_channels_count];
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
	memcpy(sbusData, loc_sbusData, 25);
	memcpy(channels, loc_channels, 18);
	memcpy(servos, loc_servos, 18);
	failsafe_status = SBUS_SIGNAL_OK;
	sbus_passthrough = 1;
	toChannels = 0;
	bufferIndex = 0;
	feedState = 0;
	this->FeedLine();
}

int16_t Sbus::Channel(uint8_t ch) {
	// Read channel data
	if ((ch > 0) && (ch <= 16)) {
		return channels[ch - 1];
	} else {
		return 1023;
	}
}
uint8_t Sbus::DigiChannel(uint8_t ch) {
	// Read digital channel data
	if ((ch > 0) && (ch <= 2)) {
		return channels[15 + ch];
	}
	return 0;
}
void Sbus::Servo(uint8_t ch, int16_t position) {
	// Set servo position
	if ((ch > 0) && (ch <= 16)) {
		if (position > 2048) {
			position = 2048;
		}
		servos[ch - 1] = position;
	}
}
void Sbus::DigiServo(uint8_t ch, uint8_t position) {
	// Set digital servo position
	if ((ch > 0) && (ch <= 2)) {
		if (position > 1) {
			position = 1;
		}
		servos[15 + ch] = position;
	}
}
uint8_t Sbus::Failsafe(void) {
	return failsafe_status;
}

void Sbus::PassthroughSet(int mode) {
	// Set passtrough mode, if true, received channel data is send to servos
	sbus_passthrough = mode;
}

int Sbus::PassthroughRet(void) {
	// Return current passthrough mode
	return sbus_passthrough;
}
void Sbus::UpdateServos(void) {
	// Send data to servos
	// Passtrough mode = false >> send own servo data
	// Passtrough mode = true >> send received channel data
	uint8_t i;
	if (sbus_passthrough == 0) {
		// clear received channel data
		for (i = 1; i < 24; i++) {
			sbusData[i] = 0;
		}

		// reset counters
		ch = 0;
		bit_in_servo = 0;
		byte_in_sbus = 1;
		bit_in_sbus = 0;

		// store servo data
		for (i = 0; i < 176; i++) {
			if (servos[ch] & (1 << bit_in_servo)) {
				sbusData[byte_in_sbus] |= (1 << bit_in_sbus);
			}
			bit_in_sbus++;
			bit_in_servo++;

			if (bit_in_sbus == 8) {
				bit_in_sbus = 0;
				byte_in_sbus++;
			}
			if (bit_in_servo == 11) {
				bit_in_servo = 0;
				ch++;
			}
		}

		// DigiChannel 1
		if (channels[16] == 1) {
			sbusData[23] |= (1 << 0);
		}
		// DigiChannel 2
		if (channels[17] == 1) {
			sbusData[23] |= (1 << 1);
		}

		// Failsafe
		if (failsafe_status == SBUS_SIGNAL_LOST) {
			sbusData[23] |= (1 << 2);
		}

		if (failsafe_status == SBUS_SIGNAL_FAILSAFE) {
			sbusData[23] |= (1 << 2);
			sbusData[23] |= (1 << 3);
		}
	}
	// send data out
	//serialPort.write(sbusData,25);
	for (i = 0; i < 25; i++) {
		//port.write(sbusData[i]);
	}
}
void Sbus::UpdateChannels(void) {
	//uint8_t i;
	//uint8_t sbus_pointer = 0;
	// clear channels[]
	/*for (i=0; i<16; i++) {
	 channels[i] = 0;
	 }
	 // reset counters
	 byte_in_sbus = 1;
	 bit_in_sbus = 0;
	 ch = 0;
	 bit_in_channel = 0;
	 //this method is much slower than the other method
	 // process actual sbus data
	 for (i=0; i<176; i++) {
	 if (sbusData[byte_in_sbus] & (1<<bit_in_sbus)) {
	 channels[ch] |= (1<<bit_in_channel);
	 }
	 bit_in_sbus++;
	 bit_in_channel++;
	 if (bit_in_sbus == 8) {
	 bit_in_sbus =0;
	 byte_in_sbus++;
	 }
	 if (bit_in_channel == 11) {
	 bit_in_channel =0;
	 ch++;
	 }
	 }*/

	channels[0] = ((sbusData[1] | sbusData[2] << 8) & 0x07FF);
	channels[1] = ((sbusData[2] >> 3 | sbusData[3] << 5) & 0x07FF);
	channels[2] = ((sbusData[3] >> 6 | sbusData[4] << 2 | sbusData[5] << 10)
			& 0x07FF);
	channels[3] = ((sbusData[5] >> 1 | sbusData[6] << 7) & 0x07FF);
	channels[4] = ((sbusData[6] >> 4 | sbusData[7] << 4) & 0x07FF);
	channels[5] = ((sbusData[7] >> 7 | sbusData[8] << 1 | sbusData[9] << 9)
			& 0x07FF);
	channels[6] = ((sbusData[9] >> 2 | sbusData[10] << 6) & 0x07FF);
	channels[7] = ((sbusData[10] >> 5 | sbusData[11] << 3) & 0x07FF); // & the other 8 + 2 channels if you need them
	if (this->_all_channel) {
		channels[8] = ((sbusData[12] | sbusData[13] << 8) & 0x07FF);
		channels[9] = ((sbusData[13] >> 3 | sbusData[14] << 5) & 0x07FF);
		channels[10] = ((sbusData[14] >> 6 | sbusData[15] << 2
				| sbusData[16] << 10) & 0x07FF);
		channels[11] = ((sbusData[16] >> 1 | sbusData[17] << 7) & 0x07FF);
		channels[12] = ((sbusData[17] >> 4 | sbusData[18] << 4) & 0x07FF);
		channels[13] = ((sbusData[18] >> 7 | sbusData[19] << 1
				| sbusData[20] << 9) & 0x07FF);
		channels[14] = ((sbusData[20] >> 2 | sbusData[21] << 6) & 0x07FF);
		channels[15] = ((sbusData[21] >> 5 | sbusData[22] << 3) & 0x07FF);
	}

	//system("clear");
	int i;
	for (i = 0; i < 8; ++i) {
		printf("Channel %d: %d    ", i + 1, channels[i]);
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
	if (sbusData[23] & (1 << 2)) {
		failsafe_status = SBUS_SIGNAL_LOST;
	}
	if (sbusData[23] & (1 << 3)) {
		failsafe_status = SBUS_SIGNAL_FAILSAFE;
	}
	//if(SBUS_SIGNAL_OK==failsafe_status){
	memcpy(channels_data, channels, sizeof(channels));
	//}

}
void Sbus::FeedLine(void) {
	fflush(stdout);
	uint8_t inBuffer[25];
	int nread;
	while(1){
		fflush(stdout);
		nread = read(_device_fd, &inBuffer, sizeof(inBuffer));
		if(nread==25){
		    if(0x0f==inBuffer[0] && 0x00==inBuffer[24]){
			   //printf("FindIt\n");
			   memcpy(sbusData, inBuffer, 25);
			   this->UpdateChannels();
			   fflush(stdout);
			}
		}
		usleep(4700);
	}
	/*
	uint8_t buffer[25];
	fd_set fds;
	struct timeval tv;
	FD_ZERO(&fds);
	FD_SET(_device_fd, &fds);
	tv.tv_sec = 0;
	tv.tv_usec = 0;
	if (select(_device_fd + 1, &fds, nullptr, nullptr, &tv) != 1) {
		//return;
	}*/
	/*
	while (1) {
	    int nread;
	    uint8_t bytes[25];
	    do {
	    	fflush(stdout);
	        nread = read(_device_fd, &bytes, sizeof(bytes));
	        if(0x0f==bytes[0]){
	        	printf("FindIt\n");
	        }
	    } while (nread == sizeof(bytes));
	}*/
	/*
	 if (port.available() > 24) {
	 while (port.available() > 0) {
	 inData = port.read();
	 switch (feedState) {
	 case 0:
	 if (inData != 0x0f) {
	 while (port.available() > 0) { //read the contents of in buffer this should resync the transmission
	 inData = port.read();
	 }
	 return;
	 } else {
	 bufferIndex = 0;
	 inBuffer[bufferIndex] = inData;
	 inBuffer[24] = 0xff;
	 feedState = 1;
	 }
	 break;
	 case 1:
	 bufferIndex++;
	 inBuffer[bufferIndex] = inData;
	 if (bufferIndex < 24 && port.available() == 0) {
	 feedState = 0;
	 }
	 if (bufferIndex == 24) {
	 feedState = 0;
	 if (inBuffer[0] == 0x0f && inBuffer[24] == 0x00) {
	 memcpy(sbusData, inBuffer, 25);
	 toChannels = 1;
	 }
	 }
	 break;
	 }
	 }
	 }*/
}

