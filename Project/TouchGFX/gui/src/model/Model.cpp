#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

#include <FreeRTOS.h>
#include <queue.h>

//uint16_t pwm_step = PWM_STEP;

extern QueueHandle_t xLowLevelData;

msg_t msg;

Model::Model() : modelListener(0)
{
}

void Model::tick()
{
	//modelListener->notifyPWMChange(pwmPeltier);
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

void Model::setPWMAuto(bool state)
{
	peltierAuto = state;
}

void Model::setPWMStep(uint16_t val)
{
	pwm_step = val;
}
