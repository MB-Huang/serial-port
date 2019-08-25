/*
 * Serial port interface program
 * This file let Linux Computer communicate 
 * with STM32 through RS232 serial port
 */

#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <stdio.h>      // standard input/output functions definitions
#include <stdlib.h>     // standard library definitions
#include <string.h>		// string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions
#include <errno.h>      // Error number definitions
#include <sys/types.h> 
#include <sys/stat.h>  

/* RX TX data bytes length */
#define SERIALPORT_TX_LEN 12	// tx bytes length
#define SERIALPORT_RX_LEN 17	// rx bytes length

class SerialPort
{
private:
	/* serial port parameter */
	int BAUDRATE;
	const char *devName;
	struct termios tty;
	int fd;	// file description for the serial port
	bool isOpen;

public:

	/* force the unit of the package is 1 byte, without doing this, the byte legth will be 4/8 bytes for 32/64 bit computer */
#pragma pack(push)	/*push current alignment to stack*/
#pragma pack(1)	 /*set alignment to 1 byte boundary*/

	/* ------------------------------------------------------------------ */
	/* union make tx rx data struct and  char array share the same memory */
	union Tx{
		/* struct type data */
		struct{
			//unsigned char = 1 byte, short = 2 bytes
			unsigned char startCode;	// should be 255

			unsigned char bodyMotionCode;
			short dis;
			short angle;

			unsigned char headMotionCode;
			unsigned short tiltPos;
			unsigned short panPos;

			unsigned char checksum;
		}data;
		/* array type data */
		unsigned char arr[SERIALPORT_TX_LEN];
	}tx;

	union Rx{
		/* struct type data */
		struct{
			//unsigned char = 1 byte, short = 2 bytes
			unsigned char start1Code;	// should be 255
			unsigned char start2Code;	// should be 254

			unsigned char dip;
			unsigned short tilePos;
			unsigned short panPos;

			unsigned short waistPosRoll;	// fix 1908
			unsigned short waistPosTilt;	// fix 1908
			unsigned short waistPosPan;		// fix 1908

			unsigned char bodyDoingCode;
			unsigned short azimuth;
			unsigned char checksum;
		}data;
		/* array type data */
		unsigned char arr[SERIALPORT_RX_LEN];
	}rx;

	/* ------------------------------------------------------------------ */
	/* functions */
	SerialPort(void);
	~SerialPort(void){close(fd);};
	bool openPort(const char *deviceName = "/dev/ttyS0", int baudrate = B115200);
	bool readPort(void);
	bool writePort(void);
	void setCheckSum( unsigned char* arr ,int len );
};

#endif