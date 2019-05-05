#ifndef MODEL_HPP
#define MODEL_HPP

#include <touchgfx/Utils.hpp>

class ModelListener;

/**
 * The Model class defines the data model in the model-view-presenter paradigm.
 * The Model is a singular object used across all presenters. The currently active
 * presenter will have a pointer to the Model through deriving from ModelListener.
 *
 * The Model will typically contain UI state information that must be kept alive
 * through screen transitions. It also usually provides the interface to the rest
 * of the system (the backend). As such, the Model can receive events and data from
 * the backend and inform the current presenter of such events through the modelListener
 * pointer, which is automatically configured to point to the current presenter.
 * Conversely, the current presenter can trigger events in the backend through the Model.
 */
class Model
{
public:
    Model();

    /**
     * Sets the modelListener to point to the currently active presenter. Called automatically
     * when switching screen.
     */
    void bind(ModelListener* listener)
    {
        modelListener = listener;
    }

    /**
     * This function will be called automatically every frame. Can be used to e.g. sample hardware
     * peripherals or read events from the surrounding system and inject events to the GUI through
     * the ModelListener interface.
     */
    void tick();
	
	/**
	 * Functions to get UI elements that are protected in the model
	 */
	int8_t getExtTemperature();
	int8_t getPeltierTemperature();
	uint16_t getPWM();
	/**
	* Functions to get UI elements that are protected in the model
	*/
	void setExtTemperature(int8_t);
	void setPeltierTemperature(int8_t);
	void setPWM(uint16_t);

protected:
    /**
     * Pointer to the currently active presenter.
     */
    ModelListener* modelListener;

	uint16_t pwmPeltier = 0;
	int8_t peltierTemperature = 25;
	int8_t extTemperature = 25;
};

#endif /* MODEL_HPP */
