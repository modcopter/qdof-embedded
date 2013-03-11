//Board = qDOF /w ATMega328P (16MHz)
#define ARDUINO 102
#define __AVR_ATmega328P__
#define F_CPU 16000000L
#define __AVR__
#define __cplusplus
#define __attribute__(x)
#define __inline__
#define __asm__(x)
#define __extension__
#define __ATTR_PURE__
#define __ATTR_CONST__
#define __inline__
#define __asm__ 
#define __volatile__
#define __builtin_va_list
#define __builtin_va_start
#define __builtin_va_end
#define __DOXYGEN__
#define prog_void
#define PGM_VOID_P int
#define NOINLINE __attribute__((noinline))

typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {}

//already defined in arduno.h
//already defined in arduno.h
void serialEvent();
void sendAttitude();
void initHardware();
void initPIDs();

#include "D:\Programme\portable\PortableApps\Arduino\hardware\arduino\variants\standard\pins_arduino.h" 
#include "D:\Programme\portable\PortableApps\Arduino\hardware\arduino\cores\arduino\arduino.h"
#include "C:\Users\Simon\Documents\Eigene Dateien\Dropbox\Quadrocopter\Software\qDOF_V2\qDOF_V2.ino"
#include "C:\Users\Simon\Documents\Eigene Dateien\Dropbox\Quadrocopter\Software\qDOF_V2\Initialisation.ino"
#include "C:\Users\Simon\Documents\Eigene Dateien\Dropbox\Quadrocopter\Software\qDOF_V2\MPU6050.cpp"
#include "C:\Users\Simon\Documents\Eigene Dateien\Dropbox\Quadrocopter\Software\qDOF_V2\MPU6050.h"
#include "C:\Users\Simon\Documents\Eigene Dateien\Dropbox\Quadrocopter\Software\qDOF_V2\crc.c"
#include "C:\Users\Simon\Documents\Eigene Dateien\Dropbox\Quadrocopter\Software\qDOF_V2\crc.h"
#include "C:\Users\Simon\Documents\Eigene Dateien\Dropbox\Quadrocopter\Software\qDOF_V2\pid.h"
#include "C:\Users\Simon\Documents\Eigene Dateien\Dropbox\Quadrocopter\Software\qDOF_V2\qMotorArray.cpp"
#include "C:\Users\Simon\Documents\Eigene Dateien\Dropbox\Quadrocopter\Software\qDOF_V2\qMotorArray.h"
#include "C:\Users\Simon\Documents\Eigene Dateien\Dropbox\Quadrocopter\Software\qDOF_V2\types.h"
