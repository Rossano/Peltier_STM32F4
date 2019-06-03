#ifndef MAINSCREEN_PRESENTER_HPP
#define MAINSCREEN_PRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class MainScreenView;

class MainScreenPresenter : public Presenter, public ModelListener
{
public:
    MainScreenPresenter(MainScreenView& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~MainScreenPresenter() {};

	///
	/// Get Notification from HW
	///
	void notifyPWMStart();
	void notifyPWMStop();
	void notifyPWMUp(uint16_t);
	void notifyPWMDown(uint16_t);
	void notifyAuto(bool);
	void notifyManual(bool);
	void notifyPWMChange(uint16_t);

	///
	///	Event Handlers
	///
/*	void PWMUpClickHandler(const Image &, const ClickEvent &);
	void PWMDownClickHandler(const Image &, const ClickEvent &);
	void PWMStartClickHandler(const Image &, const ClickEvent &);
	void PWMStopClickEvent(const Image &, const ClickEvent &);
	*/
protected:
	

private:
    MainScreenPresenter();

    MainScreenView& view;

	void peltierTempChange();
	void extTempChange();
	void pwmChange();
};


#endif // MAINSCREEN_PRESENTER_HPP
