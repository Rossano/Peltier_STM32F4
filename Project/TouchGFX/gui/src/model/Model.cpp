#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

//uint16_t pwm_step = PWM_STEP;

Model::Model() : modelListener(0)
{
}

void Model::tick()
{
	modelListener->notifyPWMChange(pwmPeltier);
}

int8_t Model::getExtTemperature()
{
	return int8_t(extTemperature);
}

int8_t Model::getPeltierTemperature()
{
	return int8_t(peltierTemperature);
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

void Model::setExtTemperature(int8_t temperature)
{
	extTemperature = temperature;
}

void Model::setPeltierTemperature(int8_t temperature)
{
	peltierTemperature = temperature;
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
