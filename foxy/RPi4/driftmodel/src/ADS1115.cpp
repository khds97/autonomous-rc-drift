#include "ADS1115.h"
#include <chrono>
#include <cstdio>
#include <memory>
#include <thread>

ADS1115::ADS1115() : ok_(false)
{
	fds.fdW = -1;
	fds.fdR = -1;
	fds._I2C_error_code_ = 0;

	ADS1115::dev_ = std::make_shared<I2Cdev>();

	ok_ = (ADS1115::dev_->startI2C(ADS1115_ADDRESS, &fds) == 0);
	if (!ok_)
	{
		fprintf(stderr, "ADS1115 could not be opened on the I2C bus\n");
	}
}

ADS1115::~ADS1115()
{
	if (ok_)
	{
		dev_->endI2C(fds);
	}
}

bool ADS1115::readRaw(int channel, int16_t *out)
{
	if (!ok_ || fds.fdW < 1 || fds.fdR < 1)
	{
		return false;
	}

	uint16_t config = ADS1115_CONFIG_REGISTER_OS_SINGLE |
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
			fprintf(stderr, "Invalid ADS1115 channel number: %d\n", channel);
			return false;
	}

	if (ADS1115::dev_->writeWord(fds, ADS1115_CONFIG_REGISTER_ADDRESS, config) != 0)
	{
		return false;
	}

	// One conversion takes about 7.8 ms at 128 SPS. Wait until the device reports that it is done
	// (OS bit = 1) instead of relying on a fixed delay.
	bool done = false;
	for (int i = 0; i < 20 && !done; ++i)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		uint16_t status = 0;
		if (ADS1115::dev_->readWord(fds, ADS1115_CONFIG_REGISTER_ADDRESS, &status) >= 0)
		{
			done = (status & ADS1115_CONFIG_REGISTER_OS_SINGLE) != 0;
		}
	}
	if (!done)
	{
		return false;
	}

	uint16_t raw = 0;
	if (ADS1115::dev_->readWord(fds, ADS1115_CONVERSION_REGISTER_ADDRESS, &raw) < 0)
	{
		return false;
	}

	// The conversion register holds a two's complement value: keep it signed.
	*out = static_cast<int16_t>(raw);
	return true;
}

bool ADS1115::readVoltage(int channel, float *volts)
{
	int16_t raw = 0;
	if (!readRaw(channel, &raw))
	{
		return false;
	}
	*volts = static_cast<float>(raw) * ADS1115_LSB_VOLTS_PGA_2_048;
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
