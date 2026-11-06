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
#include "I2Cdev.h"

I2Cdev::I2Cdev(){
}

I2Cdev::~I2Cdev(){
}

int I2Cdev::startI2C(uint8_t Addr,FDS *fds)
{
	fds->fdR = open("/dev/i2c-1", O_RDONLY);
	fds->fdW = open("/dev/i2c-1", O_WRONLY);
        fds->_I2C_error_code_ = 0;
	if (fds->fdR < 0 || fds->fdW < 0)
	{
		char str[100];
		sprintf(str, "failed to get two file desriptors (fdw: %d, fdr: %d)\n", fds->fdW, fds->fdR);
		fputs(str, stderr);
		fds->fdR = -1, fds->fdW = -1;
		return -1;
	}

	if (ioctl(fds->fdW, I2C_SLAVE, Addr) < 0) {
		fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
		fds->fdW = -1;
		return -1;
	}

	if (ioctl(fds->fdR, I2C_SLAVE, Addr) < 0) {
		fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
		fds->fdR = -1;
		return(-1);
	}

	return 0;
}


// You should use this function at the end of main func, because this structure uses file desriptor.  - before return 0; (The point that the program will be exited.)
// You may know that, if you did not close the file descriptor, then the number of file descriptor which could be generate would get shorter.
void I2Cdev::endI2C(FDS fds)
{
	close(fds.fdR);
	close(fds.fdW);
}

//* Read single byte from an 8-bit device register.
//* @param devAddr I2C slave device address
//* @param regAddr Register regAddr to read from
//* @param data Container for byte value read from device
//* @return Status of read operation (true = success)
int8_t I2Cdev::readBytes(FDS fds, uint8_t regAddr, uint8_t length, uint8_t *data)
{
	int8_t count = 0;
	if (write(fds.fdW, &regAddr, 1) != 1)
	{
		fprintf(stderr, "Failed to write reg: %s\n", strerror(errno));
		return(-1);
	}
	count = read(fds.fdR, data, length);
	if (count < 0)
	{
		fprintf(stderr, "Failed to read device(%d): %s\n", count, strerror(errno));
		return(-1);
	}
	else if (count != length)
	{
		fprintf(stderr, "Short read  from device, expected %d, got %d\n", length, count);
		return(-1);
	}

	return count;
}

//* Read single byte from an 8-bit device register.
//* @param devAddr I2C slave device address
//* @param regAddr Register regAddr to read from
//* @param data Container for byte value read from device
//* @return Status of read operation (true = success)
int8_t I2Cdev::readByte(FDS fds, uint8_t regAddr, uint8_t *data)
{
	return readBytes(fds, regAddr, 1, data);
}

//* Read multiple words from a 16-bit device register.
//* @param devAddr I2C slave device address
//* @param regAddr First register regAddr to read from
//* @param length Number of words to read
//* @param data Buffer to store read data in
//* @return Number of words read (0 indicates failure)
int8_t I2Cdev::readWords(FDS fds, uint8_t regAddr, uint8_t length, uint16_t *data){
        int8_t count = 0;

        // Open the I2C bus
        int fd = open("/dev/i2c-0", O_RDWR);
        if (fd < 0) {
            fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
            return -1;
        }

        // Set the device address
        if (ioctl(fd, I2C_SLAVE, fds) < 0) {
            fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
            close(fd);
            return -1;
        }

        // Send the register address to read from
        if (write(fd, &regAddr, 1) != 1) {
            fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
            close(fd);
            return -1;
        }

        // Read the data
        uint8_t buffer[length * 2]; // Each word is 2 bytes
        if (read(fd, buffer, length * 2) != length * 2) {
            fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
            close(fd);
            return -1;
        }

        // Close the I2C bus
        close(fd);

        // Process the received data
        for (int i = 0; i < length; i++) {
            data[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
            count++;
        }

        return count;
}

//* Read single word from a 16-bit device register.
//* @param devAddr I2C slave device address
//* @param regAddr Register regAddr to read from
//* @param data Container for word value read from device
//* @return Status of read operation (true = success)
int8_t I2Cdev::readWord(FDS fds, uint8_t regAddr, uint16_t *data)
{
	uint8_t arr[2] = {0, 0};
	int8_t save = readBytes(fds,regAddr,2,arr);
	if (save >= 0)
	{
		*data = arr[0]<<8 | arr[1];	// leave *data untouched when the read failed
	}
	return save;
}

//* Read a single bit from an 8-bit device register.
//* @param devAddr I2C slave device address
//* @param regAddr Register regAddr to read from
//* @param bitNum Bit position to read (0-7)
//* @param data Container for single bit value
//* @return Status of read operation (true = success)
int8_t I2Cdev::readBit(FDS fds, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
	uint8_t b;
	uint8_t count = readByte(fds, regAddr, &b);
	*data = b & (1 << bitNum);
	return count;
}

//* Read a single bit from an 8-bit device register.
//* @param devAddr I2C slave device address
//* @param regAddr Register regAddr to read from
//* @param bitNum Bit position to read (0-7)
//* @param data Container for single bit value
//* @return Status of read operation (true = success)
int8_t I2Cdev::readBitW(FDS fds, uint8_t regAddr, uint8_t bitNum, uint16_t *data)
{
	uint8_t b;
	uint8_t count = readByte(fds, regAddr, &b);
	*data = b & (1 << bitNum);
	return count;
}

//* Read multiple bits from an 8-bit device register.
//* @param devAddr I2C slave device address
//* @param regAddr Register regAddr to read from
//* @param bitStart First bit position to read (0-7)
//* @param length Number of bits to read (not more than 8)
//* @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
//* @return Status of read operation (true = success)
int8_t I2Cdev::readBits(FDS fds, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
	// 01101001 read byte
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	//    010   masked
	//   -> 010 shifted
	uint8_t count, b;
	if ((count = readByte(fds, regAddr, &b)) != 0) {
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		b &= mask;
		b >>= (bitStart - length + 1);
		*data = b;
	}
	return count;
}

//* Read multiple bits from a 16-bit device register.
//* @param devAddr I2C slave device address
//* @param regAddr Register regAddr to read from
//* @param bitStart First bit position to read (0-15)
//* @param length Number of bits to read (not more than 16)
//* @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
//* @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
//* @return Status of read operation (1 = success, 0 = failure, -1 = timeout)
int8_t I2Cdev::readBitsW(FDS fds, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data)
{
	// 1101011001101001 read byte
	// fedcba9876543210 bit numbers
	//    xxx           args: bitStart=12, length=3
	//    010           masked
	//           -> 010 shifted
	uint8_t count;
	uint16_t w;
	if ((count = readWord(fds, regAddr, &w)) != 0) {
		uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		w &= mask;
		w >>= (bitStart - length + 1);
		*data = w;
	}
	return count;
}

int I2Cdev::writeBytes(FDS fds, uint8_t regAddr, uint8_t length, uint8_t *data)
{
	if (length > 127) {
		fprintf(stderr, "Byte write count (%d) > 127\n", length);
		return -1;
	}

	int8_t count = 0;
	uint8_t buf[128];

	buf[0] = regAddr;
	memcpy(buf + 1, data, length);

	count = write(fds.fdW, buf, length + 1);

	if (count < 0) {
		fprintf(stderr, "Failed to write device(%d): %s\n", count, strerror(errno));
		return -1;
	}
	else if (count != length + 1) {
		fprintf(stderr, "Short write to device, expected %d, got %d\n", length + 1, count);
		return -1;
	}

	return 0;
}

int I2Cdev::writeByte(FDS fds, uint8_t regAddr, uint8_t data)
{
	return writeBytes(fds, regAddr, 1, &data);
}

int I2Cdev::writeWords(FDS fds, uint8_t regAddr, uint8_t length, uint16_t *data)
{
	printf("writeWords() not implemented\n");
	// Use readBytes() and potential byteswap
	*data = 0; // keep the compiler quiet

	return 0;
}

int I2Cdev::writeWord(FDS fds, uint8_t regAddr, uint16_t data)
{
	uint8_t data_bytes[2] = {static_cast<uint8_t>((data & 0xFF00) >> 8), static_cast<uint8_t>(data & 0x00FF)};
	return writeBytes(fds, regAddr, 2, data_bytes);
}

int I2Cdev::writeBit(FDS fds, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
	uint8_t b;
	readByte(fds, regAddr, &b);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	return writeByte(fds, regAddr, b);
}

int I2Cdev::writeBitW(FDS fds, uint8_t regAddr, uint8_t bitNum, uint16_t data)
{
	uint16_t w;
	readWord(fds, regAddr, &w);
	w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
	return writeWord(fds, regAddr, w);
}

int I2Cdev::writeBits(FDS fds, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
	//      010 value to write
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	uint8_t b;
	if (readByte(fds, regAddr, &b) != 0)
	{
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		data <<= (bitStart - length + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		b &= ~(mask); // zero all important bits in existing byte
		b |= data; // combine data with existing byte
		return writeByte(fds, regAddr, b);
	}
	else
		return -1;
}

int I2Cdev::writeBitsW(FDS fds, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data)
{
	//              010 value to write
	// fedcba9876543210 bit numbers
	//    xxx           args: bitStart=12, length=3
	// 0001110000000000 mask byte
	// 1010111110010110 original value (sample)
	// 1010001110010110 original & ~mask
	// 1010101110010110 masked | value
	uint16_t w;
	if (readWord(fds, regAddr, &w) != 0)
	{
		uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
		data <<= (bitStart - length + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		w &= ~(mask); // zero all important bits in existing word
		w |= data; // combine data with existing word
		return writeWord(fds, regAddr, w);
	}
	else
		return -1;
}
