#include "ADS1115.h"
#include <memory>
#include <chrono>
#include <thread>

ADS1115::ADS1115()
{   
	ADS1115::dev_ = std::make_shared<I2Cdev>();
	
	ADS1115::dev_->startI2C(ADS1115_ADDRESS, &fds);
}
ADS1115::~ADS1115()
{
}

uint16_t ADS1115::getVal(int channel)
{
	return readADC(fds, channel);
}

int16_t ADS1115::readADC(I2Cdev::FDS &fds, int channel)
{
	if(fds.fdW < 1 || fds.fdR < 1)
	{
		fprintf(stderr, "Connect ADS1115 with raspberry pi by using startI2C function\n");
		return -1;
	}

	uint16_t data;

	data=			ADS1115_CONFIG_REGISTER_OS_SINGLE				|
				//ADS1115_CONFIG_REGISTER_MUX_SINGLE_X				|
			  	ADS1115_CONFIG_REGISTER_PGA_2_048				|
		  		ADS1115_CONFIG_REGISTER_MODE_SINGLE 				|
		  		ADS1115_CONFIG_REGISTER_DR_128_SPS				|
			  	ADS1115_CONFIG_REGISTER_COMP_MODE_TRADITIONAL_COMPARATOR	|
				  ADS1115_CONFIG_REGISTER_COMP_POL_ACTIVE_LOW			|
		  		ADS1115_CONFIG_REGISTER_COMP_LAT_NONE				|
	  			ADS1115_CONFIG_REGISTER_COMP_QUE_DISABLE;
	switch (channel)
	{
		case 0:
			data |= ADS1115_CONFIG_REGISTER_MUX_SINGLE_0;
			break;
		case 1:
			data |= ADS1115_CONFIG_REGISTER_MUX_SINGLE_1;
			break;
		case 2:
			data |= ADS1115_CONFIG_REGISTER_MUX_SINGLE_2;
			break;
		case 3:
			data |= ADS1115_CONFIG_REGISTER_MUX_SINGLE_3;
			break;
		default:
			fprintf(stderr,"There is not Invalid such a channel number.\n");
			return -1;
	}
	ADS1115::dev_->writeWord(fds, ADS1115_CONFIG_REGISTER_ADDRESS, data);
	//usleep(8000);
	std::this_thread::sleep_for(std::chrono::microseconds(8000));
	ADS1115::dev_->readWord(fds, ADS1115_CONVERSION_REGISTER_ADDRESS, &data);
	return data;
	
	
	/*ADS1115::dev_->writeWord(fds, ADS1115_CONFIG_REGISTER_ADDRESS, data);
	static std::chrono::system_clock::time_point StartTime = std::chrono::system_clock::now();
	std::chrono::system_clock::time_point EndTime = std::chrono::system_clock::now();
	std::chrono::microseconds micro = std::chrono::duration_cast<std::chrono::microseconds>(EndTime - StartTime);
	
	if (micro.count() >= 8000){
	        ADS1115::dev_->readWord(fds, ADS1115_CONVERSION_REGISTER_ADDRESS, &data);
		StartTime = std::chrono::system_clock::now();
		return data;
	}*/
}
