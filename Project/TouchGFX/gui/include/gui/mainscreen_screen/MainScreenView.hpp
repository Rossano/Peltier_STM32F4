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
	virtual void PWMUPClicked(const Image &b, const ClickEvent &e);
	virtual void PWMDownClicked(const Image &b, const ClickEvent &e);
	virtual void PWMStartClicked(const Image &b, const ClickEvent &e);
	virtual void PWMStopClicked(const Image &b, const ClickEvent &e);
	virtual void PWMAutoClicked(const Image &b, const ClickEvent &e);

	/**
	 * UI interaction
	 */
	void notifyPeltierTemperature(float temperature);
	void notifyExternalTemperature(float temperature);
	void notiryInternalTempSensor(float temperature);
	void notifyPWMChange(uint16_t pwm);

	
protected:
	/**
	*	Event Callbacks
	*/
	Callback<MainScreenView, const Image &, const ClickEvent &> PWMUpClickedCallback;
	Callback<MainScreenView, const Image &, const ClickEvent &> PWMDownClickedCallback;
	Callback<MainScreenView, const Image &, const ClickEvent &> PWMStartClickedCallback;
	Callback<MainScreenView, const Image &, const ClickEvent &> PWMStopClickedCallback;
};

#endif // MAINSCREEN_VIEW_HPP
