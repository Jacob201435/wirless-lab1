/*
 * string.c
 *
 *  Created on: Feb 4, 2026
 *      Author: jacob
 */


#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "string.h"


void parseFields(USER_DATA *data)
{
    data->fieldCount = 0;
    char last = 'd';  // delimiter
    uint8_t i = 0;

    while (data->buffer[i] != '\0' && data->fieldCount < MAX_FIELDS)
    {
        char c = data->buffer[i];
        char type;

        if ((c > 64 && c < 91) || (c > 96 && c < 123) || c == '-' || c == '.') //check char (added - and . for negative numbers and floats)
        {
            type = 'a';
        }
        else if ((c > 47 && c < 58)) //check num
        {
            type = 'n';
        }
        else // else delim
        {
            type = 'd';
        }

        if (type != 'd' && last == 'd') //if not a delimiter
        {
            data->fieldPosition[data->fieldCount] = i;
            data->fieldType[data->fieldCount] = type;
            data->fieldCount++;
        }

        if (type == 'd') // replace with null
            data->buffer[i] = '\0';

        last = type;
        i++;
    }
}

char* getFieldString(USER_DATA *data, uint8_t fieldNumber)
{
    if (fieldNumber >= data->fieldCount)
        return 0;
    return &data->buffer[data->fieldPosition[fieldNumber]];
}

int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber)
{
    if (fieldNumber >= data->fieldCount)
        return 0;
    if (data->fieldType[fieldNumber] != 'n')
        return 0;

    char *str = &data->buffer[data->fieldPosition[fieldNumber]];
    int32_t result = 0;
    bool isNegative = false;

    if (*str == '-')
    {
        isNegative = true;
        str++;
    }

    while (*str >= '0' && *str <= '9')
    {
        result = result * 10 + (*str - '0');
        str++;
    }

    return isNegative ? -result : result;
}

float getFieldFloat(USER_DATA *data, uint8_t fieldNumber)
{
    if (fieldNumber >= data->fieldCount)
        return 0.0f;

    char *str = &data->buffer[data->fieldPosition[fieldNumber]];
    return atof_simple(str);
}

bool myStrCmp(const char *s1, const char *s2)
{
    uint8_t i = 0;
    while (s1[i] != '\0' && s2[i] != '\0')
    {
        if (s1[i] != s2[i])
            return false;
        i++;
    }
    return (s1[i] == '\0' && s2[i] == '\0');  // Both end at the same time
}

bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments)
{
    if (data->fieldCount == 0)
        return false;

    char *command = &data->buffer[data->fieldPosition[0]];
    return myStrCmp(command, strCommand) && ((data->fieldCount - 1) >= minArguments);
}

char* toAsciiHex(char* buff, uint32_t val)
{
    // Array of hex characters
    const char hexDigits[] = "0123456789abcdef";
    int i;

    // Iterate over each 4 bits from highest to lowest
    for (i = 7; i >= 0; i--)
    {
        buff[i] = hexDigits[val & 0xF];  // Extract the lowest 4 bits and convert to ASCII
        val >>= 4;  // Shift right by 4 bits to process the next 4 bits
    }

    buff[8] = '\0'; // Null-terminate the string
    return buff;
}

int32_t atoi_simple(const char *str)
{
    int32_t result = 0;
    int sign = 1;

    // Skip leading whitespace
    while (*str == ' ' || *str == '\t' || *str == '\n' || *str == '\r')
        str++;

    // Handle optional sign
    if (*str == '-')
    {
        sign = -1;
        str++;
    }
    else if (*str == '+')
    {
        str++;
    }

    // Convert digits
    while (*str >= '0' && *str <= '9')
    {
        result = result * 10 + (*str - '0');
        str++;
    }

    return result * sign;
}

float atof_simple(const char *str)
{
    float result = 0.0f;
    float sign = 1.0f;
    float divisor = 1.0f;
    bool pastDecimal = false;

    // Skip leading whitespace
    while (*str == ' ' || *str == '\t' || *str == '\n' || *str == '\r')
        str++;

    // Handle optional sign
    if (*str == '-')
    {
        sign = -1.0f;
        str++;
    }
    else if (*str == '+')
    {
        str++;
    }

    // Convert digits
    while ((*str >= '0' && *str <= '9') || *str == '.')
    {
        if (*str == '.')
        {
            pastDecimal = true;
            str++;
            continue;
        }

        if (pastDecimal)
        {
            divisor *= 10.0f;
            result = result + (float)(*str - '0') / divisor;
        }
        else
        {
            result = result * 10.0f + (float)(*str - '0');
        }
        str++;
    }

    return result * sign;
}

//-----------------------------------------------------------------------------
// Custom String Library Functions (replacing standard library)
//-----------------------------------------------------------------------------

// Custom memset - fill memory with a value
void* memset(void* ptr, int value, uint32_t num)
{
    uint8_t* p = (uint8_t*)ptr;
    uint32_t i;

    for (i = 0; i < num; i++)
    {
        p[i] = (uint8_t)value;
    }

    return ptr;
}

// Custom strlen - get string length
uint32_t strlen(const char* str)
{
    uint32_t len = 0;

    while (str[len] != '\0')
    {
        len++;
    }

    return len;
}

// Custom strcmp - compare two strings
// Returns: 0 if equal, <0 if str1 < str2, >0 if str1 > str2
int strcmp(const char* str1, const char* str2)
{
    uint32_t i = 0;

    while (str1[i] != '\0' && str2[i] != '\0')
    {
        if (str1[i] != str2[i])
        {
            return (int)(str1[i] - str2[i]);
        }
        i++;
    }

    // Handle case where one string ends before the other
    return (int)(str1[i] - str2[i]);
}

// Custom strncpy - copy n characters from src to dest
// Note: Does NOT guarantee null termination if src is >= n characters
char* strncpy(char* dest, const char* src, uint32_t n)
{
    uint32_t i;

    // Copy characters from src to dest
    for (i = 0; i < n && src[i] != '\0'; i++)
    {
        dest[i] = src[i];
    }

    // Pad with null bytes if src is shorter than n
    for (; i < n; i++)
    {
        dest[i] = '\0';
    }

    return dest;
}

//-----------------------------------------------------------------------------
// Simple integer to string converter
//-----------------------------------------------------------------------------
void intToStr(int32_t value, char* buffer)
{
    int pos = 0;
    int temp;
    char rev[20];
    int rpos = 0;

    if (value == 0)
    {
        buffer[0] = '0';
        buffer[1] = '\0';
        return;
    }

    if (value < 0)
    {
        buffer[pos++] = '-';
        value = -value;
    }

    temp = value;
    while (temp > 0)
    {
        rev[rpos++] = '0' + (temp % 10);
        temp /= 10;
    }

    while (rpos > 0)
    {
        buffer[pos++] = rev[--rpos];
    }

    buffer[pos] = '\0';
}

