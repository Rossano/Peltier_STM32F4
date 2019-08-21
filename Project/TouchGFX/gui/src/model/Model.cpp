#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

#if defined(WIN32) || defined (_WIN32)

#else
#include <FreeRTOS.h>
#include <queue.h>
#endif

//uint16_t pwm_step = PWM_STEP;

#if defined(WIN32) || defined(_WIN32)
#else
extern QueueHandle_t xLowLevelData;
#endif

msg_t msg;

controlMode_t controlMode;

//char dummy[] = TEXT("-0123456789"); // { '-', '0', "1", "2", "3", "4", "5", "6", "7", "8", "9" };

Model::Model() : modelListener(0)
{
}

void Model::tick()
{
	//modelListener->notifyPWMChange(pwmPeltier);
#if defined(WIN32) || defined(_WIN32)
	peltierTemperature = 22.5f;
	extTemperature = 23.0f;
	//pwmPeltier = 50;
	if (modelListener != 0)
	{
		modelListener->notifyPeltierTemperature(peltierTemperature);
		modelListener->notifyExternalTemperature(extTemperature);
		modelListener->notifyPWMChange(pwmPeltier);
	}
#else
	BaseType_t status;
	if((status = xQueueReceive(xLowLevelData, &msg, 0)) == pdTRUE)
	{
		peltierTemperature = msg.peltier;
		extTemperature = msg.ext;
		pwmPeltier = msg.pwm;
		if(modelListener != 0)
		{
			modelListener->notifyPeltierTemperature(peltierTemperature);
			modelListener->notifyExternalTemperature(extTemperature);
			modelListener->notifyPWMChange(pwmPeltier);
		}
	}
#endif
}

float Model::getExtTemperature()
{
	return float(extTemperature);
}

float Model::getPeltierTemperature()
{
	return float(peltierTemperature);
}

float Model::getInternalTempSensor()
{
	return float(internalTempSensor);
}

uint16_t Model::getPWM()
{
	return uint16_t(pwmPeltier);
}

bool Model::getPWMState()
{
	return peltierActive;
}

bool Model::getPWMAuto()
{
	return peltierAuto;
}

uint16_t Model::getPWMStep()
{
	return uint16_t(pwm_step);
}

void Model::setExtTemperature(float temperature)
{
	extTemperature = temperature;
}

void Model::setPeltierTemperature(float temperature)
{
	peltierTemperature = temperature;
}

void Model::setInternalTempSensor(float temperature)
{
	internalTempSensor = temperature;
}

void Model::setPWM(uint16_t PWM)
{
	pwmPeltier = PWM;
}

void Model::setPWMState(bool state)
{
	peltierActive = state;
}

/*void Model::setPWMAuto(bool state)
{
	peltierAuto = state;
}*/

bool Model::isAutoMode()
{
	if (controlMode == controlMode_t::AUTO) return true;
	else return false;
}

void Model::setPWMStep(uint16_t val)
{
	pwm_step = val;
}

void Model::setPWMMode(bool mode)
{
	if (mode)
	{
		controlMode = controlMode_t::AUTO;
	}
	else {
		controlMode = controlMode_t::MANUAL;
	}
}
/*
void Model::setManualMode()
{
	controlMode = controlMode_t::MANUAL;
}*/
