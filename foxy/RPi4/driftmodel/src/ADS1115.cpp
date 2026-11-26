#include "ADS1115.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <thread>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

ADS1115::ADS1115()
    : i2c_fd_(-1), ok_(false)
{
    ok_ = openI2c();

    if (!ok_)
    {
        fprintf(stderr, "ADS1115 initialization failed\n");
    }
}

ADS1115::~ADS1115()
{
    if (i2c_fd_ >= 0)
    {
        close(i2c_fd_);
        i2c_fd_ = -1;
    }
}

bool ADS1115::openI2c()
{
    i2c_fd_ = open(I2C_DEVICE, O_RDWR);

    if (i2c_fd_ < 0)
    {
        fprintf(stderr,
                "ADS1115: Failed to open %s: %s\n",
                I2C_DEVICE, strerror(errno));
        return false;
    }

    if (ioctl(i2c_fd_, I2C_SLAVE, ADS1115_ADDRESS) < 0)
    {
        fprintf(stderr,
                "ADS1115: Failed to select address 0x%02X: %s\n",
                ADS1115_ADDRESS, strerror(errno));

        close(i2c_fd_);
        i2c_fd_ = -1;
        return false;
    }

    return true;
}

bool ADS1115::writeRegister(uint8_t reg, uint16_t value)
{
    uint8_t buffer[3] =
    {
        reg,
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>(value & 0xFF)
    };

    struct i2c_msg message = {};
    message.addr  = ADS1115_ADDRESS;
    message.flags = 0;
    message.len   = sizeof(buffer);
    message.buf   = buffer;

    struct i2c_rdwr_ioctl_data transaction = {};
    transaction.msgs  = &message;
    transaction.nmsgs = 1;

    if (ioctl(i2c_fd_, I2C_RDWR, &transaction) < 0)
    {
        fprintf(stderr,
                "ADS1115: Write failed "
                "(reg=0x%02X, value=0x%04X): %s\n",
                reg, value, strerror(errno));

        return false;
    }

    return true;
}

bool ADS1115::readRegisters(uint8_t reg, uint8_t *data, uint8_t length)
{
    struct i2c_msg messages[2] = {};

    messages[0].addr  = ADS1115_ADDRESS;
    messages[0].flags = 0;
    messages[0].len   = 1;
    messages[0].buf   = &reg;

    messages[1].addr  = ADS1115_ADDRESS;
    messages[1].flags = I2C_M_RD;
    messages[1].len   = length;
    messages[1].buf   = data;

    struct i2c_rdwr_ioctl_data transaction = {};
    transaction.msgs  = messages;
    transaction.nmsgs = 2;

    if (ioctl(i2c_fd_, I2C_RDWR, &transaction) < 0)
    {
        fprintf(stderr,
                "ADS1115: Read failed (reg=0x%02X): %s\n",
                reg, strerror(errno));
        return false;
    }

    return true;
}

bool ADS1115::readWord(uint8_t reg, uint16_t *value)
{
    uint8_t buffer[2] = {};

    if (!readRegisters(reg, buffer, sizeof(buffer)))
    {
        return false;
    }

    *value = (static_cast<uint16_t>(buffer[0]) << 8) |
             static_cast<uint16_t>(buffer[1]);

    return true;
}

bool ADS1115::readRaw(int channel, int16_t *out)
{
    if (!ok_ || i2c_fd_ < 0 || out == nullptr)
    {
        return false;
    }

    uint16_t config =
        ADS1115_CONFIG_REGISTER_OS_SINGLE |
        ADS1115_CONFIG_REGISTER_PGA_2_048 |
        ADS1115_CONFIG_REGISTER_MODE_SINGLE |
        ADS1115_CONFIG_REGISTER_DR_128_SPS |
        ADS1115_CONFIG_REGISTER_COMP_MODE_TRADITIONAL_COMPARATOR |
        ADS1115_CONFIG_REGISTER_COMP_POL_ACTIVE_LOW |
        ADS1115_CONFIG_REGISTER_COMP_LAT_NONE |
        ADS1115_CONFIG_REGISTER_COMP_QUE_DISABLE;

    switch (channel)
    {
        case 0:
            config |= ADS1115_CONFIG_REGISTER_MUX_SINGLE_0;
            break;

        case 1:
            config |= ADS1115_CONFIG_REGISTER_MUX_SINGLE_1;
            break;

        case 2:
            config |= ADS1115_CONFIG_REGISTER_MUX_SINGLE_2;
            break;

        case 3:
            config |= ADS1115_CONFIG_REGISTER_MUX_SINGLE_3;
            break;

        default:
            fprintf(stderr,
                    "ADS1115: Invalid channel number: %d\n",
                    channel);
            return false;
    }

    if (!writeRegister(ADS1115_CONFIG_REGISTER_ADDRESS, config))
    {
        return false;
    }

    // Wait for single-shot conversion to complete.
    bool done = false;

    for (int i = 0; i < 20 && !done; ++i)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        uint16_t status = 0;

        if (readWord(ADS1115_CONFIG_REGISTER_ADDRESS, &status))
        {
            done = (status & ADS1115_CONFIG_REGISTER_OS_SINGLE) != 0;
        }
    }

    if (!done)
    {
        fprintf(stderr, "ADS1115: Conversion timeout\n");
        return false;
    }

    uint16_t raw = 0;

    if (!readWord(ADS1115_CONVERSION_REGISTER_ADDRESS, &raw))
    {
        return false;
    }

    *out = static_cast<int16_t>(raw);

    return true;
}

bool ADS1115::readVoltage(int channel, float *volts)
{
    if (volts == nullptr)
    {
        return false;
    }

    int16_t raw = 0;

    if (!readRaw(channel, &raw))
    {
        return false;
    }

    *volts = static_cast<float>(raw) *
             ADS1115_LSB_VOLTS_PGA_2_048;

    return true;
}

int16_t ADS1115::getVal(int channel)
{
    int16_t raw = 0;

    if (!readRaw(channel, &raw))
    {
        return 0;
    }

    return raw;
}
