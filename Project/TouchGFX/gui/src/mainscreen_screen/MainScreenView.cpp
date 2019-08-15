#include <gui/mainscreen_screen/MainScreenView.hpp>
#include <gui\model\Model.hpp>

MainScreenView::MainScreenView():PWMUpClickedCallback(this, &MainScreenView::PWMUPClicked),
	PWMDownClickedCallback(this, &MainScreenView::PWMDownClicked),
	PWMStartClickedCallback(this, &MainScreenView::PWMStartClicked),
	PWMStopClickedCallback(this, &MainScreenView::PWMStopClicked)
{
	PWMUpButton.setClickAction(PWMUpClickedCallback);
	PWMDownButton.setClickAction(PWMDownClickedCallback);
	StartButton.setClickAction(PWMStartClickedCallback);
	StopButton.setClickAction(PWMStopClickedCallback);
}

void MainScreenView::setupScreen()
{
    MainScreenViewBase::setupScreen();
}

void MainScreenView::tearDownScreen()
{
    MainScreenViewBase::tearDownScreen();
}

void MainScreenView::PWMUPClicked(const Image &b, const ClickEvent &e)
{
	presenter->notifyPWMUp(PWM_STEP);
	touchgfx_printf("PWM Up");
}

void MainScreenView::PWMDownClicked(const Image &b, const ClickEvent &e)
{
	presenter->notifyPWMDown(PWM_STEP);
	touchgfx_printf("PWM Down");
}

void MainScreenView::PWMStartClicked(const Image &b, const ClickEvent &e)
{
	presenter->notifyPWMStart();
}

void MainScreenView::PWMStopClicked(const Image &b, const ClickEvent &e)
{
	presenter->notifyPWMStop();
}

void MainScreenView::PWMAutoClicked(const Image &b, const ClickEvent &e)
{
	presenter->notifyAuto(true);
}

void MainScreenView::notifyPeltierTemperature(float temperature)
{
	Unicode::itoa((uint16_t)temperature, PeltierTempBuffer, PELTIERTEMP_SIZE, 10);
	PeltierTempText.invalidate();
}

void MainScreenView::notifyExternalTemperature(float temperature)
{
	Unicode::itoa((uint16_t)temperature, ExtTempBuffer, EXTTEMP_SIZE, 10);
//	Unicode::snprintf(ExtTempBuffer, 10, "%d", (uint16_t)temperature);
	ExtTempText.invalidate();
}

void MainScreenView::notifyPWMChange(uint16_t pwm)
{
	PWMProgress.setValue(pwm);
	Unicode::itoa(pwm, PWMTextBuffer, PWMTEXT_SIZE, 10);
//	Unicode::snprintf(PWMTextBuffer, 10, "%d", pwm);
	PWMText.invalidate();
}
