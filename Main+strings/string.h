/*
 * string.h
 *
 *  Created on: Feb 4, 2026
 *      Author: jacob
 */

#ifndef STRING_H_
#define STRING_H_

#include <stdint.h>
#include <stdbool.h>
#include "uart0.h"

// Function prototypes
void parseFields(USER_DATA *data);
char* getFieldString(USER_DATA *data, uint8_t fieldNumber);
int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber);
float getFieldFloat(USER_DATA *data, uint8_t fieldNumber);
bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments);
bool myStrCmp(const char *s1, const char *s2);
char* toAsciiHex(char* buff, uint32_t val);
int32_t atoi_simple(const char *str);
float atof_simple(const char *str);
void* memset(void* ptr, int value, uint32_t num);
uint32_t strlen(const char* str);
int strcmp(const char* str1, const char* str2);
char* strncpy(char* dest, const char* src, uint32_t n);
void intToStr(int32_t value, char* buffer);

#endif /* STRING_H_ */

