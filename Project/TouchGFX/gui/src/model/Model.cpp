#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

Model::Model() : modelListener(0)
{
}

void Model::tick()
{
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
