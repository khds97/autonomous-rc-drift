#ifndef _I2CDEV_H_
#define _I2CDEV_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>

class I2Cdev {
    public:
        I2Cdev();
        ~I2Cdev();
        typedef struct fd_struct
	{
		int fdW;
		int fdR;
		int fdWordW;

		int _I2C_error_code_;
	}FDS;

	int startI2C(uint8_t Addr, FDS *fds);
	void endI2C(FDS fds);  // You should use this function at the end of main func, because this structure uses file desriptor.  - before return 0; (The point that the program will be exited.)

	int8_t readBytes(FDS fds, uint8_t regAddr, uint8_t length, uint8_t *data);
	int8_t readByte(FDS fds, uint8_t regAddr, uint8_t *data);
	int8_t readWords(FDS fds, uint8_t regAddr, uint8_t length, uint16_t *data);
	int8_t readWord(FDS fds, uint8_t regAddr, uint16_t *data);
	int8_t readBit(FDS fds, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
	int8_t readBitW(FDS fds, uint8_t regAddr, uint8_t bitNum, uint16_t *data);
	int8_t readBits(FDS fds, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
	int8_t readBitsW(FDS fds, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data);



	int writeBytes(FDS fds, uint8_t regAddr, uint8_t length, uint8_t *data);
	int writeByte(FDS fds, uint8_t regAddr, uint8_t data);
	int writeWords(FDS fds, uint8_t regAddr, uint8_t length, uint16_t *data);
	int writeWord(FDS fds, uint8_t regAddr, uint16_t data);
	int writeBit(FDS fds, uint8_t regAddr, uint8_t bitNum, uint8_t data);
	int writeBitW(FDS fds, uint8_t regAddr, uint8_t bitNum, uint16_t data);
	int writeBits(FDS fds, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
	int writeBitsW(FDS fds, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
};

#endif /* _I2CDEV_H_ */
