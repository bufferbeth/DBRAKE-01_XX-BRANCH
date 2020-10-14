#include <asf.h>
//#include "samd20.h"
#include "dbrakeDefs.h"
#include "driverLCD.h"

const uint8_t ScreenHome[1024]=
{
0xFF,0x01,0xF9,0x49,0x49,0x49,0x09,0x01,0xF9,0x09,0x09,0x09,0xF9,0x01,0xF9,0x49,
0xC9,0x49,0x21,0x10,0xF9,0x09,0x09,0x09,0x09,0x01,0xF9,0x49,0x49,0x49,0x09,0x01,
0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x24,0x24,0x24,0xD8,0x00,
0xFC,0x24,0x64,0xA4,0x18,0x00,0xF8,0x44,0x44,0x44,0xF8,0x00,0xFC,0x20,0x50,0x88,
0x04,0x00,0xFC,0x24,0x24,0x24,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0xFC,0x04,0x04,
0x04,0xFC,0x00,0x00,0xFC,0x20,0x50,0x88,0x04,0x00,0x00,0x00,0x40,0xC0,0xC0,0x80,
0xC0,0x60,0x30,0x18,0x03,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

 
0xFF,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x03,0x02,0x02,0x02,0x03,0x00,0x03,0x00,
0x00,0x01,0x02,0x00,0x03,0x02,0x02,0x02,0x02,0x00,0x03,0x02,0x02,0x02,0x02,0x00,
0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x11,0x11,0xF1,0x11,0x10,0x00,
0x11,0x10,0xF0,0x10,0x11,0x00,0xF1,0x90,0x90,0x90,0x61,0x00,0xF1,0x90,0x90,0x90,
0x11,0x00,0x61,0x91,0x91,0x91,0x11,0x00,0x00,0x00,0x00,0x00,0x00,0xF1,0x11,0x11,
0x11,0xF1,0x00,0x00,0xF1,0x80,0x40,0x20,0x11,0x00,0x00,0x00,0x00,0x00,0x01,0x01,
0x00,0x80,0xC0,0x60,0x30,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,


0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0x00,0x00,0x00,
0x04,0x04,0x07,0x04,0x04,0x00,0x07,0x00,0x01,0x02,0x04,0x00,0x07,0x04,0x04,0x04,
0x04,0x00,0x04,0x04,0x04,0x20,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x04,0x04,
0x04,0x07,0x00,0x00,0x07,0x00,0x01,0x02,0x04,0x00,0x00,0x00,0x01,0x03,0x07,0x06,
0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,


0x0F,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08, 
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
0x0F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x80,
0xC0,0xC0,0x60,0x60,0x30,0x30,0x18,0x18,0x0C,0x0C,0x06,0x06,0x03,0xFF,0xFF,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,


0x00,0x3E,0x2A,0x3E,0x00,0x3E,0x1A,0xAE,0x80,0xBE,0x08,0xB6,0x80,0xB0,0x1C,0x86,
0x80,0xBE,0x0A,0xBE,0x80,0x3E,0x18,0xBE,0x00,0xBE,0x0A,0x3E,0x00,0x0E,0x38,0x0E,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x81,
0x80,0xC0,0xC0,0xE0,0xE0,0xF0,0xF0,0xF8,0xF8,0xFC,0xFC,0xFE,0xFF,0xFF,0xFF,0x01,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,


0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0F,0x06,0x0B,0x00,0x0F,0x0A,0x0A,0x00,0x0F,
0x02,0x0F,0x00,0x0F,0x08,0x07,0x00,0x03,0x0E,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x80,0xC0,0xC0,0xE0,0xE0,0xF0,0xF0,0xF8,0xF8,0xFC,0xFC,0xFE,0xFE,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xC0,
0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xFF,0xFF,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,


0xF0,0x10,0x90,0x90,0x90,0x90,0x90,0x10,0x90,0x90,0x90,0x90,0x90,0x10,0x90,0x90,
0x90,0x90,0x10,0x10,0x90,0x90,0x90,0x90,0x90,0x10,0x90,0x90,0x90,0x90,0x90,0x10,
0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,
0x10,0x10,0x10,0x10,0x90,0x10,0x10,0x10,0x90,0x10,0x90,0x90,0x90,0x90,0x90,0x10,
0x90,0x10,0x10,0x10,0x90,0x10,0x90,0x10,0x10,0x10,0x90,0x10,0x10,0x10,0x10,0xF0,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,
0x10,0x90,0x90,0x90,0x90,0x90,0x10,0x90,0x90,0x90,0x90,0x90,0x10,0x90,0x90,0x90,
0x90,0x10,0x10,0x90,0x90,0x90,0x90,0x90,0x10,0x10,0x90,0x90,0x90,0x90,0x10,0xF0,


0xFF,0x00,0x3F,0x04,0x04,0x04,0x00,0x00,0x3F,0x20,0x20,0x20,0x3F,0x00,0x3F,0x04,
0x0C,0x14,0x23,0x00,0x3F,0x20,0x20,0x20,0x20,0x00,0x3F,0x24,0x24,0x24,0x20,0x00,
0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,
0x00,0x00,0x00,0x00,0x3F,0x01,0x06,0x01,0x3F,0x00,0x3F,0x24,0x24,0x24,0x20,0x00,
0x3F,0x03,0x0C,0x10,0x3F,0x00,0x3F,0x20,0x20,0x20,0x3F,0x00,0x00,0x00,0x00,0xFF,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,
0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x20,0x20,0x3F,0x20,0x20,0x00,0x3F,0x04,0x0C,
0x14,0x23,0x00,0x3F,0x24,0x24,0x24,0x20,0x00,0x23,0x24,0x24,0x24,0x18,0x00,0xFF

};



 