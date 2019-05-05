#ifndef MAINSCREEN_VIEW_HPP
#define MAINSCREEN_VIEW_HPP

#include <gui_generated/mainscreen_screen/MainScreenViewBase.hpp>
#include <gui/mainscreen_screen/MainScreenPresenter.hpp>

class MainScreenView : public MainScreenViewBase
{
public:
    MainScreenView();
    virtual ~MainScreenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

	/**
	 * Custom Action Handlers
	 */
	virtual void PWMUPClicked();
	virtual void PWMDownClicked();

	/**
	 * UI interaction
	 */
	void notifyPeltierTemperature(int8_t temperature);
	void notifyExternalTemperature(int8_t temperature);
	void notifyPWMChange(uint16_t pwm);

protected:
};

#endif // MAINSCREEN_VIEW_HPP
