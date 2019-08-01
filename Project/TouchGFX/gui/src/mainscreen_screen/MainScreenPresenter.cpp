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
	if ((model->getPWM() + val) <= 100)
	{
		model->setPWM(val + model->getPWM());
	}
	else
	{
		model->setPWM(100);
	}
}

void MainScreenPresenter::notifyPWMDown(uint16_t val)
{
	if ((model->getPWM() - val) > 0)
	{
		model->setPWM(model->getPWM() - val);
	}
	else
	{
		model->setPWM(0);
	}
}

void MainScreenPresenter::notifyAuto(bool)
{
	model->setPWMAuto(true);
}

void MainScreenPresenter::notifyManual(bool)
{
	model->setPWMAuto(false);
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
