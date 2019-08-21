#include <gui/mainscreen_screen/MainScreenView.hpp>
#include <gui/mainscreen_screen/MainScreenPresenter.hpp>

MainScreenPresenter::MainScreenPresenter(MainScreenView& v)
    : view(v)
{
}

void MainScreenPresenter::activate()
{

}

void MainScreenPresenter::deactivate()
{

}

void MainScreenPresenter::notifyPWMStart()
{
	model->setPWMState(true);
}

void MainScreenPresenter::notifyPWMStop()
{
	model->setPWMState(false);
}

void MainScreenPresenter::notifyPWMUp(uint16_t val)
{
	if (model->isAutoMode()) return;
	uint16_t pwm = model->getPWM();
	if (val + pwm <= MAX_PWM)
	{
		model->setPWM(val + pwm);
	}
	else
	{
		model->setPWM(MAX_PWM);
	}
}

void MainScreenPresenter::notifyPWMDown(uint16_t val)
{
	if (model->isAutoMode()) return;
	uint16_t pwm = model->getPWM();
	if (pwm - val >= MIN_PWM)
	{
		model->setPWM(pwm - val);
	}
	else
	{
		model->setPWM(MIN_PWM);
	}
}

void MainScreenPresenter::notifyAuto()
{
	model->setPWMMode(true);
}

void MainScreenPresenter::notifyManual()
{
	model->setPWMMode(false);
}

void MainScreenPresenter::notifyPWMChange(uint16_t val)
{
	view.notifyPWMChange(val);
}

void MainScreenPresenter::peltierTempChange()
{
	view.notifyPeltierTemperature(model->getPeltierTemperature());
}

void MainScreenPresenter::extTempChange()
{
	view.notifyExternalTemperature(model->getExtTemperature());
}

void MainScreenPresenter::pwmChange()
{
	view.notifyPWMChange(model->getPWM());
}

void MainScreenPresenter::notifyPeltierTemperature(float temperature)
{
	view.notifyExternalTemperature(model->getInternalTempSensor());
}

void MainScreenPresenter::notifyExternalTemperature(float temperature)
{
	view.notifyExternalTemperature(model->getExtTemperature());
}

void MainScreenPresenter::notifyInternalTempSensorChange()
{

}
