//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Ryan McKay
//  January 2024
//  ELEX 4618
//  Lab 3: Embedded Controls 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*********************************************************************
 * @file  CControl.h
 * @author Ryan McKay 2024
 * @brief Communicate with tm4c123 and perform basic operations.
 *********************************************************************/


#pragma once
#include "Serial.h"
#include <vector>
#include <sstream>
#include <iomanip>
#include <iostream>

#define BaudRate 115200
#define Buffer 10

#define LongAssTime 10000
#define ONE_SECOND 1000

#define JOY_X_CHAN 11
#define JOY_Y_CHAN 4
#define BUTTON_DELAY 10
#define DEADZONE 20

#define ACCELEROMETER_X_CHAN 7
#define ACCELEROMETER_Y_CHAN 6
#define ACCELEROMETER_Z_CHAN 5

#define ADC_VALUE 4096
#define ESC 27

#define BOOSTER_BUTTON_1 32
#define BOOSTER_BUTTON_2 33

#define RGBLED_RED_PIN 39
#define RGBLED_GRN_PIN 38
#define RGBLED_BLU_PIN 37

#define LOW 0
#define HIGH 1

#define SERVO_LEFT 180
#define SERVO_MIDDLE 82		//Should be 90, but the servo attaches off center
#define SERVO_RIGHT 1


 /**
  * @class CControl
  * @brief Class to control and communicate with the tm4c123 microcontroller.
  *
  * @detail The CControl class provides methods for communicating with the
  * tm4c123 microcontroller, handling data, and performing various
  * control functions.
  */
class CControl {
private:
	Serial _com;	///< Serial communication object.

	
	enum _type;

	std::string _comport;		///< Holds the COM port string for the object
	

protected:

	/**
	* @name Helper Functions
	 * @brief Perform various small tasks to ensure larger functions run smoothly
	 * @param rx_str this is the string to be loaded with the received value
	 * @return integers to be passed on for data selections.
	 * @return Point2f returns a point type containing the x and y inputs from the analog stick
	 */
	
	int get_type();
	int get_channel();
	int get_servoPort();
	int get_pin();
	int get_value();
	int num_check();
	int val_check();
	void print_menu();	
	/** @} */ // end of Helper Functions

public:
	/**
	 * @brief Constructor for CControl class.
	 */
	CControl();
	/**
	 * @brief Destructor for CControl class.
	 */
	~CControl();

	/**
	 *@name Communication Initializers 
	 *@brief Initialize the communication port.
	 *
	 * @param comport Name of the communication port to initialize.
	 */
	void init_com(std::string comport);
	void COM_search();
	/** @} */ // end of Communication Initializers

	cv::Point2f get_analog();
	cv::Point3f get_accelerometer();
	void MakeTheAnalogInputMoreUsable(cv::Point2f& point, int x, int y);
	int percentToInt(const float& percentage);

	/**
	* @name Getter/Setter
	 * @brief Get data from the microcontroller.
	 *
	 * @param type Type of the data to get.
	 * @param channel Channel from which to get the data.
	 * @param result Variable to store the retrieved data.
	 * @param value Value to be set.
	 * @param rx_str String to receive the transmission for the controller
	 * @return true if data retrieval is successful, false otherwise.
	 */
	bool get_data(int type, int channel, int& result);
	bool set_data(int type, int channel, int value);
	bool receive_transmission(std::string& rx_str);
	/** @} */ // end of Getter/Setter


	/**
	 * @name Control Functions
	 * @brief Group of public functions to perform various control operations.
	 * 
	 */
	void display_analog();	///< Show the raw and percent postions of the analog x and y
	void display_accelerometer();
	bool get_button(int Button);		///< After debouncing the button for 1 second, display the number of button pushes
	void button_LED();		///< Control an LED by pushing a button
	void button_COUNT();	///< DIsplays the number of button pushes coninuously
	void servo_TEST();		///< Moves the servo through it's full range of motion
	void servo_PLAY();		///< Allows the user to control the servo with the Analog Stick
	void servo_SLOW();
	void disable_LED();
	void enable_LED(int light);
	/** @} */ // end of Control Functions


	/**
	 * @brief Display and handle the control menu.
	 *
	 * @param Control Reference to the CControl object.
	 * @return Integer representing the status or command selected in the menu.
	 */
	int menu(CControl Control);
	
	
	
};