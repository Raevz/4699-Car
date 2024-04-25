//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Ryan McKay
//  January 2024
//  ELEX 4618
//  Lab 3: Embedded Controls 
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"

#include "CControl.h"

CControl::CControl() 
{
	/*std::string comport = "COM4";
	_com.flush();
	init_com(comport);*/
}

CControl::~CControl() 
{
	_com.flush();
}

enum CControl::_type
{
	Digital,
	Analog,
	Servo
};

void CControl::COM_search()
{
	//Create a vector of strings "COM1", "COM2" ... up to "COM100"
	int i = 0;
	std::vector<std::string> COM;
	for (i = 1; i <= 255; i++)
	{
		COM.push_back("COM" + std::to_string(i));		//Append the value of 'i' to the end of "COM"
	}
	 
	std::cout << "\n\nSEARCHING FOR DEVICE...";
	std::cout << "\nCTRL+C to cancel\n";

	//Cycle COM ports until one opens
	i = 0;
	do
	{
		init_com(COM[i]);
		//std::cout << COM[i] << std::endl;			//Print the search for fun
		//Sleep(10);	<--- Unnecessary :)
		i++;

		if (i >= 255)
		{
			i = 0;
		}		

	} while (!_com.is_open());

	std::cout << "\nDEVICE ACQUIRED\n\n";
	std::cout << COM[i-1] << std::endl;

	_comport = COM[i-1];
}

void CControl::init_com(std::string comport)
{
	_com.flush();
	_com.open(comport, BaudRate);
}

void CControl::display_analog()
{
	int xRaw = 0;
	int yRaw = 0;
	

	std::cout << "\nClick the small window and hit 'Escape' to exit";

	// Create a blank image for imshow
	cv::Mat blankImage = cv::Mat::zeros(100, 100, CV_8UC1);

	// Open a window using imshow
	cv::imshow("esc to exit", blankImage);

	while (1)
	{
		cv::Point2f JoystickPosition = get_analog();

		xRaw = (JoystickPosition.x * ADC_VALUE) / 100;
		yRaw = (JoystickPosition.y * ADC_VALUE) / 100;
		
		std::cout << "\nX: " << xRaw << " (" << JoystickPosition.x  << ")%";
		std::cout << "\t\tY: " << yRaw << " (" << JoystickPosition.y  << ")%";

		if (cv::waitKey(2) == ESC)
			break;
	}

	// Close the imshow window
	cv::destroyWindow("esc to exit");
}

void CControl::disable_LED()
{
	set_data(Digital, RGBLED_RED_PIN, LOW);
	set_data(Digital, RGBLED_GRN_PIN, LOW);
	set_data(Digital, RGBLED_BLU_PIN, LOW);
}

void CControl::enable_LED(int light)
{
	int RGB[3] = { RGBLED_RED_PIN, RGBLED_GRN_PIN, RGBLED_BLU_PIN };
	set_data(Digital, RGB[light], HIGH);
}

cv::Point2f CControl::get_analog()
{
	int xResult = 0;
	float xPercent = 0; 
	int yResult = 0;
	float yPercent = 0;

	get_data(Analog, JOY_X_CHAN, xResult);
	xPercent = xResult * 100 / ADC_VALUE;

	get_data(Analog, JOY_Y_CHAN, yResult);
	yPercent = yResult * 100 / ADC_VALUE;

	cv::Point2f JoystickPosition = cv::Point2f(xPercent, yPercent);	

	return JoystickPosition;
}

cv::Point3f CControl::get_accelerometer()
{
	int xResult, yResult, zResult;

	get_data(Analog, ACCELEROMETER_X_CHAN, xResult);
	get_data(Analog, ACCELEROMETER_Y_CHAN, yResult);
	get_data(Analog, ACCELEROMETER_Z_CHAN, zResult);
	

	cv::Point3f accelerometer = cv::Point3f(xResult, yResult, zResult);

	return accelerometer;
}

void CControl::display_accelerometer()
{
	

	while (1)
	{
		cv::Point3f acc = get_accelerometer();
		std::cout << "\nX: " << acc.x << "\tY: " << acc.y << "\tZ: " << acc.z;
	}
}

bool CControl::get_button(int Button)
{
	int state = 0;
	int count = 0;

		get_data(Digital, Button, state);

		if (state == 0)
		{
			Sleep(BUTTON_DELAY);
			return true;			
		}

		return false;
	

}

void CControl::button_COUNT()
{
	int count = 0;

	float timer_start = GetTickCount();
	while (GetTickCount() - timer_start < LongAssTime)
	{
		std::cout << "\nButton Presses: " << count;

		if (get_button(BOOSTER_BUTTON_1))
		{
			++count;
			float punishment_timer = GetTickCount();
			do
			{
				std::cout << "\nButton Presses: " << count;
				Sleep(10);

			} while (GetTickCount() - punishment_timer < ONE_SECOND);
		}
		Sleep(10);
	}
}

void CControl::button_LED()
{
	int state = 0;

	float timer_start = GetTickCount();
	while (GetTickCount() - timer_start < LongAssTime)
	{
		get_data(Digital, BOOSTER_BUTTON_1, state);

		std::cout << "\nButton State: " << state;

		set_data(Digital, RGBLED_BLU_PIN, !state);		//Button is Active Low, Light should turn on when button pushed.
		set_data(Digital, RGBLED_GRN_PIN, !state);		
		Sleep(10);
	}
	set_data(Digital, RGBLED_BLU_PIN, LOW);
	set_data(Digital, RGBLED_GRN_PIN, LOW);
}

void CControl::servo_SLOW()
{
	int servoPort = get_servoPort();

	set_data(Servo, servoPort, SERVO_RIGHT);
	Sleep(500);

	for (int i = SERVO_RIGHT; i <= SERVO_LEFT; i++)
	{
		set_data(Servo, servoPort, i);

		std::cout << "\nServo @ " << i;
		Sleep(5);
	}

	for (int i = SERVO_LEFT; i >= SERVO_RIGHT; i--)
	{
		set_data(Servo, servoPort, i);

		std::cout << "\nServo @ " << i;
		Sleep(5);
	}

	set_data(Servo, servoPort, SERVO_MIDDLE);
}

void CControl::servo_TEST()
{
	int servoPort = get_servoPort();
	
	//MANUAL TESTING PROTOCOL	
	//int cmd;
	/*while (1)
	{
		std::cin >> cmd;
		set_data(Servo, servoPort, cmd);
	}*/
		
	set_data(Servo, servoPort, SERVO_MIDDLE+45);
	Sleep(500);

	set_data(Servo, servoPort, SERVO_MIDDLE-45);
	Sleep(500);

	set_data(Servo, servoPort, SERVO_LEFT);
	Sleep(500);

	set_data(Servo, servoPort, SERVO_RIGHT);
	Sleep(500);

	/*set_data(Servo, servoPort, SERVO_LEFT);
	Sleep(500);*/

	set_data(Servo, servoPort, SERVO_MIDDLE);
}

void CControl::servo_PLAY()
{
	int servoPort = get_servoPort();
	
	std::cout << "\nNow controlling Servo using joystick";
	
	float timer_start = GetTickCount();
	while(1) //(GetTickCount() - timer_start < LongAssTime)
	{
		cv::Point2f JoystickPosition = get_analog();
		int InvertedValue = SERVO_LEFT - (JoystickPosition.x * 1.8) + SERVO_RIGHT;		//Input needs to be inverted to match analog stick when looking head on at Servo
		set_data(Servo, servoPort, InvertedValue);
	}

	set_data(Servo, servoPort, SERVO_MIDDLE);
}

bool CControl::get_data(int type, int channel, int& result)
{
	result = 0;

	std::ostringstream command;
	
	//take in the command input that was passed to the get_data function
	// assign to string and pass to the write function 
	command << "G " << type << " " << channel << "\n";
	_com.write(command.str().c_str(), command.str().length());

	std::string rx_str;
	rx_str = "";
	if (!(receive_transmission(rx_str)))
	{
		result = 1;
		return true;
	}
	
	//extract last part of string as the value
	int lastSpace = rx_str.rfind(' ');
		
	result = std::stoi(rx_str.substr(lastSpace + 1));

	
	return true; 
}

bool CControl::set_data(int type, int channel, int value)
{

	//if comms fail, exit function
	if (!_com.is_open())
	{
		std::cout << "\nCOMMUNICATION FAILURE";
		return false;
	}

	// Construct the command to set data
	std::ostringstream command;
	command << "S " << type << " " << channel << " " << value << "\n";
	_com.write(command.str().c_str(), command.str().length());

	//Read acknowledgement from uC
	std::string rx_str;
	receive_transmission(rx_str);
	
	return true;
}

bool CControl::receive_transmission(std::string& rx_str)
{
	char buff[Buffer] = { 0 };
	
	//wait for acknowledgment transmission receival
	//Thank you for providing this code
	float timer_start = GetTickCount();
	while (buff[0] != '\n' && GetTickCount() - timer_start < ONE_SECOND)
	{
		if (_com.read(buff, 1) > 0)
		{
			rx_str += buff[0];
		}
	}
	
	if (rx_str.empty() || !(rx_str.back() == '\n'))
	{
		std::cout << "\nCONNECTION INTERRUPTED";
		COM_search();
		return false;
	}
	return true;
}

int CControl::menu(CControl Control)
{
	char cmd1 = -1;
	int type = -1;
	int channel = -1;
	int value = -1;

	do
	{
		cmd1 = -1;
		type = -1;
		channel = -1;
		value = -1;

		Control.print_menu();
		
		std::cin >> cmd1;
		switch (cmd1)
		{
		case 'a':
		case 'A': display_analog();
			break;

		case 'b':
		case 'B': button_LED();
			break;

		/*case 'c':
		case 'C': get_button();
			break;*/

		case 'd':
		case 'D': button_COUNT();
			break;

		case 'g':
		case 'G':
		{
			std::cout << "\nGet Data";								//Left over code from building the get/set functions
			type = Control.get_type();
			channel = Control.get_channel();

			if (Control.get_data(type, channel, value) == false)
				std::cout << "\nCOMMUNICATION ERROR\n\n";					

			break;
		}
		case 'p':
		case 'P': Control.servo_PLAY();
			break;

		case 's':
		case 'S': Control.servo_TEST();
			break;

		case 'r':
		case 'R': display_accelerometer(); break;
		
		case 'q':
		case 'Q': cmd1 = 'q';
			break;

		default:
			std::cout << "\nunsupported character \n\n";
			break;
		}
	}while (cmd1 != 'q' && 'Q');
	return 0;
}

void CControl::print_menu()
{
	std::cout << "\n**************************************************";
	std::cout << "\n* ELEX4618 Lab3: Embedded Control, by Ryan McKay *";
	std::cout << "\n**************************************************";

	std::cout << "\nTHIS------> IS THE MENU";
	std::cout << "\nEnter Command";
	std::cout << "\n(A)nalog \n(B)utton LED Control \n(C)ount Button presses/Debounce\n(D)umb Counter \n(P)lay with Servo \n(S)ervo Test \n(R)eally Slow Servo \n(Q)uit";
	std::cout << "\nCMD> ";
}

int CControl::get_type()
{
	int type;
	std::cout << "\nType? (0=DIGITAL, 1=ANALOG, 2=SERVO)";
	std::cout << "\nCMD> ";
	while (1)
	{
		type = num_check();
		if (type >= 0 && type <= 2)
			return type;
		else
			std::cout << "\nInvalid Input";
	}
}

int CControl::get_channel()
{
	std::cout << "\nChannel? (0-15)";
	std::cout << "\nCMD> ";
	while (1)
	{
		int channel = num_check();
		if (channel >= 0 && channel <= 15)
			return channel;
		else
			std::cout << "\nInvalid Input";
	}
}

int CControl::get_servoPort()
{
	std::cout << "\nServo? (0-3)";
	std::cout << "\nCMD> ";
	while (1)
	{
		int channel = num_check();
		if (channel >= 0 && channel <= 3)
			return channel;
		else
			std::cout << "\nInvalid Input";
	}
}

int CControl::percentToInt(const float& percentage)
{
	if (percentage >= 0 && percentage < 15) {
		return 3;
	}
	else if (percentage >= 15 && percentage < 30) {
		return 2;
	}
	else if (percentage >= 30 && percentage < 45) {
		return 1;
	}
	else if (percentage >= 45 && percentage <= 55) {
		return 0;
	}
	else if (percentage > 55 && percentage < 70) {
		return 4;
	}
	else if (percentage >= 70 && percentage < 85) {
		return 5;
	}
	else if (percentage >= 85 && percentage <= 100) {
		return 6;
	}
	else {
		return 0; // Return -1 or some error code if percentage is out of expected range
	}
}


int CControl::get_pin()
{
	std::cout << "\nPin #?";
	std::cout << "\nCMD> ";
	return num_check();
}

int CControl::get_value()
{
	std::cout << "\nvalue?";
	std::cout << "\nCMD> ";
	return val_check();
}

int CControl::num_check()
{
	int num;
	std::string input;

	while (1)
	{
		std::cin >> input;
		std::stringstream stream(input);                      //Convert input string to stringstream

		if (stream >> num && (stream.eof() == true))        //Try to convert the stream to a float, failures mean illegal characters entered 
		{

			if (num >= 0 && num <= 40)
			{
				return num;
			}
			else
			{
				std::cerr << "\nInvalid Input \nCMD> ";
			}
		}
		else
		{
			std::cerr << "\nNumbers only please \nCMD> ";
		}

		// Clear the error flag and ignore the rest of the line
		std::cin.clear();
		std::cin.ignore(1000, '\n');
	}
}

int CControl::val_check()
{
	int num;
	std::string input;

	while (1)
	{
		std::cin >> input;
		std::stringstream stream(input);                      //Convert input string to stringstream

		if (stream >> num && (stream.eof() == true))        //Try to convert the stream to a float, failures mean illegal characters entered 
		{
			return num;
		}
		else
		{
			std::cerr << "\nNumbers only please \nCMD> ";
		}

		// Clear the error flag and ignore the rest of the line
		std::cin.clear();
		std::cin.ignore(1000, '\n');
	}
}
