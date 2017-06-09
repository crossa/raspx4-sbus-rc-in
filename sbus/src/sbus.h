#ifndef FUTABA_SBUS_h
#define FUTABA_SBUS_h
#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
#define BAUDRATE 100000
#define TRUE 1
#define FALSE 0

#include <stdio.h>
#include <stdlib.h>
#include  <stdint.h>
#include  <string.h>
#include  <unistd.h>     /*Unix 标准函数定义*/
#include  <fcntl.h>      /*文件控制定义*/
#include <sys/ioctl.h>
#include <asm-generic/termbits.h>
#include  <errno.h>      /*错误号定义*/
#include  <sys/types.h>
#include  <sys/stat.h>
#include  <sys/types.h>
#include  <sys/ipc.h>
#include  <sys/shm.h>

//#define ALL_CHANNELS

class Sbus {
public:

	int *channels_data;
	int max_channels_count;
	uint8_t failsafe_status;
	Sbus(char*,bool);
	~Sbus();
	void init();
	void begin(void);
	int16_t Channel(uint8_t ch);
	uint8_t DigiChannel(uint8_t ch);
	void Servo(uint8_t ch, int16_t position);
	void DigiServo(uint8_t ch, uint8_t position);
	uint8_t Failsafe(void);
	void PassthroughSet(int mode);
	int PassthroughRet(void);
	void UpdateServos(void);
	void UpdateChannels(void);
	void FeedLine(void);
private:
	bool _debug = false;
	uint8_t _sbusData[25]; // SBUS帧数据
	int16_t _channels[18]; // 18个通道的数据
	int16_t _servos[18];   //18个舵机的数据
	bool _all_channel; //是否完整开启16通道
	const char *_device; //设备标识
	int _device_fd; //设备具柄
	//uint8_t _channels;
	int _shmid;
	int _key;
};

#endif
