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
	uint8_t sbusData[25];
	int16_t channels[18];
	int16_t *channels_data;
	int max_channels_count;
	int16_t servos[18];
	uint8_t failsafe_status;
	int sbus_passthrough;
	int toChannels;
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
	bool _all_channel; //是否完整开启16通道
	char _device[30]; //设备标识
	int _device_fd; //设备具柄
	struct termios2 _device_opt; //通信属性
	int _setDeviceParity(int fd, int databits, int stopbits, int parity); //设定通信参数
	uint8_t _channels;
	uint8_t byte_in_sbus;
	uint8_t bit_in_sbus;
	uint8_t ch;
	uint8_t bit_in_channel;
	uint8_t bit_in_servo;
	int _shmid;
	int *_mem;
	int _key;
	int bufferIndex;
	uint8_t inData;
	int feedState;
};

#endif
