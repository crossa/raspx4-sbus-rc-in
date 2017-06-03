/*
 * function.cpp
 *
 *  Created on: 2017年6月1日
 *      Author: secondstupid
 */
#include "function.h"
//-----------------------------------查参数--------------------------------//
char* getArg(int argc, char**arg, char *term) {
	int i;
	for (i = 0; i < argc; ++i) {
		if (0 == strcmp(*(arg + i), term)) {
			if ((i + 1) == argc)
				return NULL;
			return *(arg + i + 1);
		}
	}
	return NULL;
}
//----------------------------------打印帮助信息
void help() {
	printf("\n");
	printf("使用方法：\n\n");
	printf("sbus -d <设备节点> -c <通道数>\n");
	printf("\n");
}
//----------------------------------启动Sbus
void startSbus(char *device,bool all){
	Sbus instance(device,all);
	instance.begin();
}
