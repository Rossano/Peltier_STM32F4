#include <gui/mainscreen_screen/MainScreenView.hpp>
#include <gui\model\Model.hpp>

MainScreenView::MainScreenView():PWMUpClickedCallback(this, &MainScreenView::PWMUPClicked),
	PWMDownClickedCallback(this, &MainScreenView::PWMDownClicked),
	PWMStartClickedCallback(this, &MainScreenView::PWMStartClicked),
	PWMStopClickedCallback(this, &MainScreenView::PWMStopClicked),
	AutoModeClickedCallback(this, &MainScreenView::PWMAutoClicked),
	ManualModelClickCallback(this, &MainScreenView::PWMManualClicked)
{
	PWMUpButton.setClickAction(PWMUpClickedCallback);
	PWMDownButton.setClickAction(PWMDownClickedCallback);
	StartButton.setClickAction(PWMStartClickedCallback);
	StopButton.setClickAction(PWMStopClickedCallback);
	AutoMode.setClickAction(AutoModeClickedCallback);
	manualMode.setClickAction(ManualModelClickCallback);
	
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
	presenter->notifyManual();
	notifyPWMManualMode();
}

void MainScreenView::PWMManualClicked(const Image & b, const ClickEvent & e)
{
	presenter->notifyAuto();
	notifyPWMAutoMode();
}

void MainScreenView::notifyPeltierTemperature(float temperature)
{
	PeltierTemp.resizeToCurrentText();
	PeltierTempText.invalidate();
}

void MainScreenView::notifyExternalTemperature(float temperature)
{
	Unicode::snprintf(ExtTempBuffer, 10, "%d", (uint16_t)temperature);
	ExtTemp.resizeToCurrentText();
	ExtTemp.invalidate();
}

void MainScreenView::notifyPWMChange(uint16_t pwm)
{
	PWMProgress.setValue((int)pwm);
	double val = (double)pwm * 100 / MAX_PWM;
	Unicode::snprintf(PWMTextBuffer, 6, "%d", (uint16_t)val);
	PWMText.setVisible(false);
	PWMText.invalidate();
	PWMText.setVisible(true);
	PWMText.resizeToCurrentText();
	PWMText.invalidate();
	PWMProgress.invalidate();
}

void MainScreenView::notifyPWMManualMode()
{
	presenter->notifyManual();
	AutoMode.setVisible(false);
	AutoMode.invalidate();
	manualMode.setVisible(true);
	manualMode.invalidate();
}

void MainScreenView::notifyPWMAutoMode()
{
	presenter->notifyAuto();
	manualMode.setVisible(false);
	manualMode.invalidate();
	AutoMode.setVisible(true);
	AutoMode.invalidate();
}
