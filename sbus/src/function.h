/*
 * function.h
 *
 *  Created on: 2017年6月1日
 *      Author: secondstupid
 */

#ifndef SRC_FUNCTION_H_
#define SRC_FUNCTION_H_
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "sbus.h"
/**
 * 从命令行搜索参数并返回参数的指针
 */
char* getArg(int,char**,char *);

/**
 *  打印帮助信息
 */
void help();

/**
 *
 */
void startSbus(char *device,bool all);
#endif /* SRC_FUNCTION_H_ */
