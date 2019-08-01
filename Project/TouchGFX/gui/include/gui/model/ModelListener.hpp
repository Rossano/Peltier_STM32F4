#ifndef MODELLISTENER_HPP
#define MODELLISTENER_HPP

#include <gui/model/Model.hpp>

/**
 * ModelListener is the interface through which the Model can inform the currently
 * active presenter of events. All presenters should derive from this class.
 * It also provides a model pointer for the presenter to interact with the Model.
 *
 * The bind function is called automatically.
 *
 * Add the virtual functions Model should be able to call on the active presenter to this class.
 */
class ModelListener
{
public:
    ModelListener() : model(0) {}

    virtual ~ModelListener() {}

    /**
     * Sets the model pointer to point to the Model object. Called automatically
     * when switching screen.
     */
    void bind(Model* m)
    {
        model = m;
    }

	/**
	 * Virtual Functions to update UI
	 */
	virtual void notifyPeltierTemperature(float temperature) { }
	virtual void notifyExternalTemperature(float temperature) { }
	virtual void notifyInternalTempSensorChange() {}
	virtual void notifyPWMChange(uint16_t pwm) { }
	/*virtual void notifyPWMStart();
	virtual void notifyPWMStop();
	virtual void notifyPWMUp(uint16_t);
	virtual void notifyPWMDown(uint16_t);
	virtual void notifyAuto(bool);
	virtual void notifyManual(bool);
	*/
protected:
    Model* model;
};

#endif /* MODELLISTENER_HPP */
