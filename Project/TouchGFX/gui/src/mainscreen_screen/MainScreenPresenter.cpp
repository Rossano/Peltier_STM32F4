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
