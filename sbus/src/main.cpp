//============================================================================
// Name        : sbus.cpp
// Author      : seoncondstupid
// Version     :
// Copyright   : since May 2017
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "function.h"

int main(int argc, char**argv) {
	char *device;//*channels; //从命令行接收的参数，

	//int channel_count;
	char *all;
	/*
	if (NULL == (device = getArg(argc, argv, "-d"))) {
		help();
		exit(0x00);
	}*/

	if (NULL == (all = getArg(argc, argv, "-a"))) {
		all="n";
	}
	bool all_channel = (strcmp("y", all) == 0) ? true : false;
	//channel_count = atoi(channels);
	startSbus(device, all_channel);
	return 0x00;
}
