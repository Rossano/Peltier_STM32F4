#include <gui/mainscreen_screen/MainScreenView.hpp>
#include <gui\model\Model.hpp>

MainScreenView::MainScreenView()
{

}

void MainScreenView::setupScreen()
{
    MainScreenViewBase::setupScreen();
}

void MainScreenView::tearDownScreen()
{
    MainScreenViewBase::tearDownScreen();
}

void MainScreenView::PWMUPClicked()
{
}

void MainScreenView::PWMDownClicked()
{
}

void MainScreenView::notifyPeltierTemperature(int8_t temperature)
{
	Unicode::itoa(temperature, PeltierTempBuffer, 3, 10);
	PeltierTempText.invalidate();
}

void MainScreenView::notifyExternalTemperature(int8_t temperature)
{
	Unicode::itoa(temperature, ExtTempBuffer, 3, 10);
	ExtTempText.invalidate();
}

void MainScreenView::notifyPWMChange(uint16_t pwm)
{

}
