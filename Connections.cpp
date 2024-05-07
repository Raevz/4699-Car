#include "stdafx.h"
#include "Connections.h"
#include "cvui.h"

const std::string Car_IP = "192.168.0.112";
const std::string Arena_IP = "192.168.0.101";
const int Car_Port_cmd = 6969;
const int ArenaData_Port = 4008;
const int ArenaImage_Port = 5008;


using namespace cv;


Connections::Connections()
{
	_do_exit = false;
	_auto = false;
	_reset = false;
	_start = false;

	_Kp = 0.07;
	_Ki = 0.0008;
	_Kd = 0.6;
	_lastError = 0;

	timeout_start_ma = 0;
	timeout_start_d = 0;
	timeout_start_i = 0;
	_baseSpeed = 200;

	///// THESE IDENTIFY THE BLUE FOR MKR2
	_LowHue = 109;
	_HighHue = 140;
	_LowSat = 164;
	_HighSat = 255;
	_LowVal = 57;
	_HighVal = 160;
	//////////////////////////////////////

	_control = cv::Mat::zeros(cv::Size(250, 900), CV_8UC1);
	_control = cv::Scalar(49, 52, 49);  // Dark gray background

	_alpha = 0.45;
}
Connections::~Connections()
{

}

void Connections::send_command(CClient& client, std::string cmd)
{
	std::string str;
	str.clear();
	client.tx_str(cmd);
	//std::cout << "\nClient Tx: " << cmd;
	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(300));
	if (client.rx_str(str) == true)
	{
		timeout_start_ma = cv::getTickCount();
		std::cout << "\nClient Rx: " << str;
	}
	else
	{
		if ((cv::getTickCount() - timeout_start_ma) / cv::getTickFrequency() > 1000)
		{
			// No response, disconnect and reconnect
			timeout_start_ma = cv::getTickCount();
			client.close_socket();
			client.connect_socket(Car_IP, Car_Port_cmd);
		}
	}
}

int Connections::main_menu()
{
	_do_exit = false;

	std::thread t1(&Connections::arenaData_thread, this);
	std::thread t2(&Connections::arenaImage_thread, this);
	t1.detach();
	t2.detach();

	int cmd = -1;
	do
	{
		print_menu();

		std::cin >> cmd;
		switch (cmd)
		{
		case 1: { manual(); break; }
		case 2: { autonomous(); break; }
		default: { std::cout << "\nInvalid\n"; break; }
		}
	} while (cmd != 0);
	 
	_do_exit = true;
	return 0;
}

void Connections::print_menu()
{
	std::cout << "\n*****************************************";
	std::cout << "\n* ELEX4699 Arena Car by Teylor and Ryan *";
	std::cout << "\n*****************************************";
	std::cout << "\n(1) Manual";
	std::cout << "\n(2) Auto";
	std::cout << "\n(0) Exit";
	std::cout << "\nCMD> ";
}

void Connections::manual()
{
	CControl Control;
	Control.COM_search();

	CClient client;
	client.connect_socket(Car_IP, Car_Port_cmd);
	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(300));
	
	while (!_do_exit)
	{
		cv::Point2f analogInput = Control.get_analog();

		std::string pos = joy(analogInput);
		
		int t = Control.get_button(BOOSTER_BUTTON_1);
		
		//client.tx_str("S " + std::to_string(l) + " " + std::to_string(r) + " " + std::to_string(t) + " \n");

		std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(5));
	}

}

std::string Connections::joy(cv::Point2f& in)
{
	std::string out;
	float l, r;
	in.y -= 50;
	in.x -= 50;
	if (in.y > 0)
	{
		l = in.y * 2;
		r = l;
		if (std::abs(in.x) < DEADZONE)
		{
			in.x = 0;
		}
		else if (in.x > 0)
		{
			r -= in.x;
		}

		
	}
	else
	{

	}
	return "k";

}

void Connections::autonomous()
{
	_auto = false;
	std::thread t_overlay(&Connections::overlay_thread, this);
	t_overlay.detach();

	CClient client;
	client.connect_socket(Car_IP, Car_Port_cmd);
	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(300));

	
	do
	{
		//get aruco position && orientation
		//calculate orientation in relation to checkpoint
		//calculate closest point on line
		//calculate positional deviance from closest point
		//PID motor commands
		Point position;
		int error = 0 - position.x;
		int P = error;
		int I = I + error;
		int D = error - _lastError;
		_lastError = error;
		float motorspeed = P * _Kp + I * _Ki + D * _Kd;
		//translate commands to strings
		//transmit command strings

		//client.tx_str("S " + std::to_string(y) + " " + std::to_string(x) + " " + std::to_string(t) + " \n");
		if (_exit.contains(position))
		{
			_auto = false;
		}

	} while (!_do_exit && _auto);
}

void Connections::arenaData()
{
	//Open server connection
	CClient client;
	client.connect_socket(Arena_IP, ArenaData_Port);
	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(500));
	timeout_start_d = cv::getTickCount();

	std::string cmd = "G 0";
	std::string data;

	//Initialize Matrix 
	cv::imshow("DATA", _control);
	cvui::init("DATA");

	// Set text properties
	int fontFace = cv::FONT_HERSHEY_SIMPLEX;
	double fontScale = 0.5;
	int thickness = 1;
	int baseline = 0;
	int lineHeight = cv::getTextSize("Text", fontFace, fontScale, thickness, &baseline).height + 5;

	do
	{
		client.tx_str(cmd);		
		data.clear();
		
		if (client.rx_str(data))
		{
			std::string time;
			parse_data(data, time, _d1, _d2, _d3, _d4);
		
			cv::Point gui_position;

			//TIMER
			gui_position = cv::Point(10, 20);
			cv::putText(_control, "TIME: " + time, gui_position, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);

			// MARKER 1
			gui_position.y += lineHeight;
			cv::putText(_control, "LEFT: " + _d1, gui_position, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);

			// MARKER 2
			gui_position.y += lineHeight;
			cv::putText(_control, "TOP: " + _d2, gui_position, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);

			// MARKER 3
			gui_position.y += lineHeight;
			cv::putText(_control, "RIGHT: " + _d3, gui_position, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);

			// MARKER 4
			gui_position.y += lineHeight;
			cv::putText(_control, "BOTTOM: " + _d4, gui_position, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);

			//Open window for controls
			gui_position.y += lineHeight;
			gui_position += Point(-10, 0);
			cvui::window(_control, gui_position.x, gui_position.y, 250, 300, "CONTROLS");

			///////////// CONTROLS FOR PID VALUES //////////////////////////////////////////////////////////
			gui_position += Point(0, 10);
			cvui::text(_control, gui_position.x, gui_position.y, "Kp");
			gui_position += cv::Point(0, 15);
			cvui::trackbar(_control, gui_position.x, gui_position.y, 200, &_Kp, 0.0f, 0.2f);

			gui_position += Point(0, 10);
			cvui::text(_control, gui_position.x, gui_position.y, "Ki");
			gui_position += cv::Point(0, 15);
			cvui::trackbar(_control, gui_position.x, gui_position.y, 200, &_Ki, 0.0f, 0.05f);

			gui_position += Point(0, 10);
			cvui::text(_control, gui_position.x, gui_position.y, "Kd");
			gui_position += cv::Point(0, 15);
			cvui::trackbar(_control, gui_position.x, gui_position.y, 200, &_Kd, 0.0f, 1.0f);
			////////////////////////////////////////////////////////////////////////////////////////////////
			// AUTO BUTTON
			gui_position += cv::Point(0, 20);
			cvui::checkbox(_control, gui_position.x, gui_position.y, "Auto Mode", &_auto);

			// RESET BUTTON
			gui_position = cv::Point((10 , _control.rows - 50)); // Centered at bottom
			if (cvui::button(_control, gui_position.x, gui_position.y, 100, 30, "RESET")) {
				_reset = true;
			}
			
			// EXIT BUTTON
			gui_position = cv::Point((_control.cols - 100) / 2, _control.rows - 50); // Centered at bottom
			if (cvui::button(_control, gui_position.x, gui_position.y, 100, 30, "EXIT")) {
				_do_exit = true;
			}			
			timeout_start_d = cv::getTickCount();			
		}
		else
		{			
			
			if ((cv::getTickCount() - timeout_start_d) / cv::getTickFrequency() > 1000)
			{
				// No response, disconnect and reconnect
				timeout_start_d = cv::getTickCount();
				client.close_socket();
				client.connect_socket(Arena_IP, ArenaData_Port);
			}
		}
		cvui::update();
		cv::imshow("DATA", _control);
		cv::waitKey(10);

	} while(!_do_exit);

}

void Connections::parse_data(std::string& data, std::string& time, std::string& _d1, std::string& _d2, std::string& _d3, std::string& _d4)
{
	size_t start, end;

	// Parse TIME
	start = data.find("TIME=\"") + 6;
	end = data.find("\"", start);
	time = data.substr(start, end - start);

	// Parse D1
	start = data.find("D1=\"") + 4;
	end = data.find("\"", start);
	_d1 = data.substr(start, end - start);

	// Parse D2
	start = data.find("D2=\"") + 4;
	end = data.find("\"", start);
	_d2 = data.substr(start, end - start);

	// Parse D3
	start = data.find("D3=\"") + 4;
	end = data.find("\"", start);
	_d3 = data.substr(start, end - start);

	// Parse D4
	start = data.find("D4=\"") + 4;
	end = data.find("\"", start);
	_d4 = data.substr(start, end - start);	
}

void Connections::arenaImage()
{
	CClient client;
	timeout_start_i = cv::getTickCount();
	client.connect_socket(Arena_IP, ArenaImage_Port);
	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(300));

	do
	{
		client.tx_str("G 1");
		std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(20));
		imgrab.lock();
		if (client.rx_im(_image) == true)
		{
			imgrab.unlock();
			timeout_start_i = cv::getTickCount();
			if (_image.empty() == false)
			{
				cv::imshow("Arena Image", _image);
				if (_auto)
				{
					cv::addWeighted(_overlay, _alpha, _image, 1 - _alpha, 0, _image);
				}
				cv::waitKey(10);
			}
		}
		else
		{
			imgrab.unlock();
			if ((cv::getTickCount() - timeout_start_i) / cv::getTickFrequency() > 1000)
			{
				// No response, disconnect and reconnect
				timeout_start_i = cv::getTickCount();
				client.close_socket();
				client.connect_socket(Arena_IP, ArenaImage_Port);
			}
		}
		//std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(150));
	} while (!_do_exit);
}

void Connections::overlay()
{
	imgrab.lock();
	_image.copyTo(_overlay);
	imgrab.unlock();

	int rows = _image.rows;
	int cols = _image.cols;

	// Number of cells in the grid
	int gridRows = 15;
	int gridCols = 15;

	///// JUST SOME SETUP CODE
	int cellWidth = cols / gridCols;
	int cellHeight = rows / gridRows;
	int zoneWidth = 3 * cellWidth;
	int zoneHeight = 3 * cellHeight;
	int middleMarkersY = 6 * cellHeight;
	cv::Scalar black = cv::Scalar(0, 0, 0);
	cv::Scalar rectColor(180, 0, 0); // blue color for the rectangle

	///// FIND THE TOP MARKER
	cv::Point mk2 = locateMkr2(_image);

	///// MAKE THE SHOOT ZONES IN FRONT OF THE MARKERS
	cv::Rect zone_1 = cv::Rect(cellWidth, middleMarkersY, zoneWidth, zoneHeight);
	cv::Rect zone_2 = cv::Rect(mk2.x - cellWidth, mk2.y + cellHeight, zoneWidth, zoneHeight);
	cv::Rect zone_3 = cv::Rect(cols - (zoneWidth + cellWidth), middleMarkersY, zoneWidth, zoneHeight);
	cv::Rect zone_4 = cv::Rect(cols / 2 - zoneWidth / 2, cols - (zoneHeight + cellHeight), zoneWidth, zoneHeight);
	///// EXIT BOX
	cv::Rect _exit = cv::Rect(cols - zoneWidth, rows - zoneHeight, zoneWidth, zoneHeight);	

	///// PLACE STRAIGHT LINES NORMAL TO TARGETS THROUGH THE SHOOT ZONES
	_mk1_line = { Point(zone_1.x + 0.5 * zoneWidth, zone_1.y + zoneHeight), Point(zone_1.x + 0.5 * zoneWidth, zone_1.y) };
	_mk2_line = { Point(zone_2.x, zone_2.y + 0.5 * zoneHeight), Point(zone_2.x + zoneWidth, zone_2.y + 0.5 * zoneHeight) };
	_mk3_line = { Point(zone_3.x + 0.5 * zoneWidth, zone_1.y + zoneHeight), Point(zone_3.x + 0.5 * zoneWidth, zone_1.y) };
	_mk4_line = { Point(zone_4.x, zone_4.y + 0.5 * zoneHeight), Point(zone_4.x + zoneWidth, zone_4.y + 0.5 * zoneHeight) };

	///// PLOT THE PATH ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///// START TO MK1
	_line1 = { Point2f(1.5 * cellWidth, rows - 2.5 * cellHeight), Point2f(1.5 * cellWidth, rows - 4.5 * cellHeight), Point2f(2.5 * cellWidth, rows - 6 * cellHeight), Point2f(zone_1.x + 0.5 * zoneWidth, zone_1.y + zoneHeight) };
	///// MK1 TO MK2 (VARIABLE)
	_line2 = { Point2f(zone_1.x + 0.5 * zoneWidth, zone_1.y), Point2f(2 * cellWidth, 3 * cellHeight), Point2f(3.5 * cellWidth, 2 * cellHeight), Point2f(zone_2.x, zone_2.y + 0.5 * zoneHeight) };
	///// MK2 (VARIABLE) TO MK3
	_line3 = { Point2f(zone_2.x + zoneWidth, zone_2.y + 0.5 * zoneHeight), Point2f(cols - 1.5 * cellWidth, 2.5 * cellHeight), Point2f(cols - 1.5 * cellWidth, 4.5 * cellHeight), Point2f(zone_3.x + 0.5 * zoneWidth, zone_1.y) };
	///// MK3 TO MK4
	_line4 = { Point2f(zone_3.x + 0.5 * zoneWidth, zone_1.y + zoneHeight), Point2f(cols - 3.5 * cellWidth, 9.5 * cellHeight), Point2f(cols - 4.5 * cellWidth, 10.5 * cellHeight), Point2f(cols - 4.5 * cellWidth, rows - 2.5 * cellHeight), Point2f(cols - 6.5 * cellWidth, rows - 2.5 * cellHeight), Point2f(zone_4.x + zoneWidth, zone_4.y + 0.5 * zoneHeight) };
	///// MK4 TO EXIT
	_line5 = { Point2f(zone_4.x, zone_4.y + 0.5 * zoneHeight), Point2f(4 * cellWidth, rows - 5.5 * cellHeight), Point2f(cols - 7.5 * cellWidth, rows - 6.5 * cellHeight), Point2f(cols - 5.5 * cellWidth, rows - 7.5 * cellHeight), Point2f(cols - 2.5 * cellWidth, rows - 6.5 * cellHeight), Point2f(cols - 1.5 * cellWidth, rows - 4.5 * cellHeight) };
	_line5f = { Point2f(cols - 1.5 * cellWidth, rows - 4.5 * cellHeight), Point2f(cols - 1.5 * cellWidth, rows - 2.5 * cellHeight), Point2f(cols - 1.5 * cellWidth, rows - 1.5 * cellHeight) };

	///// THIS SHIT SMOOTHS THE LINES OUT AND STOPS JANKY TURNS
	std::vector<Point> L1 = smooth(_line1);
	std::vector<Point> L2 = smooth(_line2);
	std::vector<Point> L3 = smooth(_line3);
	std::vector<Point> L4 = smooth(_line4);
	std::vector<Point> L5 = smooth(_line5);
	std::vector<Point> L5f = smooth(_line5f);

	_reset = false;

	do
	{
		while (_d1 == "0")
		{
			over.lock();
			cv::rectangle(_overlay, zone_1, rectColor, -1); // -1 means filled
			cv::polylines(_overlay, _mk1_line, false, black, 3, LINE_AA);
			cv::polylines(_overlay, L1, false, black, 3, LINE_AA);	
			over.unlock();
			std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(50));
		}
		while (_d1 == "1" && _d2 == "0")
		{
			over.lock();
			cv::rectangle(_overlay, zone_2, rectColor, -1); // -1 means filled
			cv::polylines(_overlay, _mk2_line, false, black, 3, LINE_AA);
			cv::polylines(_overlay, L2, false, black, 3, LINE_AA);
			over.unlock();
			std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(50));
		}
		while (_d1 == "1" && _d2 == "1" && _d3 == "0")
		{
			over.lock();
			cv::rectangle(_overlay, zone_3, rectColor, -1); // -1 means filled
			cv::polylines(_overlay, _mk3_line, false, black, 3, LINE_AA);
			cv::polylines(_overlay, L3, false, black, 3, LINE_AA);
			over.unlock();
			std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(50));
		}
		while (_d1 == "1" && _d2 == "1" && _d3 == "1" && _d4 == "0")
		{
			over.lock();
			cv::rectangle(_overlay, zone_4, rectColor, -1); // -1 means filled
			cv::polylines(_overlay, _mk4_line, false, black, 3, LINE_AA);
			cv::polylines(_overlay, L4, false, black, 3, LINE_AA);
			over.unlock();
			std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(50));
		}
		if (_d1 == "1" && _d2 == "1" && _d3 == "1" && _d4 == "1")
		{
			over.lock();
			cv::rectangle(_overlay, _exit, cv::Scalar(180, 60, 255), -1); // -1 means filled
			cv::polylines(_overlay, L5, false, black, 3, LINE_AA);
			cv::polylines(_overlay, L5f, false, black, 3, LINE_AA);
			over.unlock();
			std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(50));
		}
	} while (!_reset && !_do_exit);
}

void Connections::img()
{
	cv::Mat image = cv::imread("Arena.png", cv::IMREAD_COLOR);
	cv::Mat overlay;
	image.copyTo(overlay);

	int rows = image.rows;
	int cols = image.cols;

	// Number of cells in the grid
	int gridRows = 15;
	int gridCols = 15;

	///// JUST SOME SETUP CODE
	int cellWidth = cols / gridCols;
	int cellHeight = rows / gridRows;
	int zoneWidth = 3 * cellWidth;
	int zoneHeight = 3 * cellHeight;
	int middleMarkersY = 6 * cellHeight;
	cv::Scalar black = cv::Scalar(0, 0, 0);

	///// FIND THE TOP MARKER
	cv::Point mk2 = locateMkr2(image);	

	///// MAKE THE SHOOT ZONES IN FRONT OF THE MARKERS
	cv::Rect zone_1 = cv::Rect(cellWidth, middleMarkersY, zoneWidth, zoneHeight);
	cv::Rect zone_2 = cv::Rect(mk2.x - cellWidth, mk2.y + cellHeight, zoneWidth, zoneHeight);
	cv::Rect zone_3 = cv::Rect(cols - (zoneWidth + cellWidth), middleMarkersY, zoneWidth, zoneHeight);
	cv::Rect zone_4 = cv::Rect(cols/2 - zoneWidth/2, cols - (zoneHeight + cellHeight), zoneWidth, zoneHeight);

	/////EXIT BOX
	cv::Rect exit = cv::Rect(cols - zoneWidth, rows - zoneHeight, zoneWidth, zoneHeight);
	cv::rectangle(overlay, exit, cv::Scalar(180, 60, 255), -1); // -1 means filled

	///// DRAW THE SHOOT ZONES
	cv::Scalar rectColor(180, 0, 0); // blue color for the rectangle
	cv::rectangle(overlay, zone_1, rectColor, -1); // -1 means filled
	cv::rectangle(overlay, zone_2, rectColor, -1); // -1 means filled
	cv::rectangle(overlay, zone_3, rectColor, -1); // -1 means filled
	cv::rectangle(overlay, zone_4, rectColor, -1); // -1 means filled

	///// PLACE STRAIGHT LINES NORMAL TO TARGETS THROUGH THE SHOOT ZONES
	_mk1_line = { Point(zone_1.x + 0.5 * zoneWidth, zone_1.y + zoneHeight), Point(zone_1.x + 0.5 * zoneWidth, zone_1.y) };
	_mk2_line = { Point(zone_2.x, zone_2.y + 0.5 * zoneHeight), Point(zone_2.x + zoneWidth, zone_2.y + 0.5 * zoneHeight) };
	_mk3_line = { Point(zone_3.x + 0.5 * zoneWidth, zone_1.y + zoneHeight), Point(zone_3.x + 0.5 * zoneWidth, zone_1.y) };
	_mk4_line = { Point(zone_4.x, zone_4.y + 0.5 * zoneHeight), Point(zone_4.x + zoneWidth, zone_4.y + 0.5 * zoneHeight) };

	///// DRAW NORMAL LINES
	cv::polylines(overlay, _mk1_line, false, black, 3, LINE_AA);
	cv::polylines(overlay, _mk2_line, false, black, 3, LINE_AA);
	cv::polylines(overlay, _mk3_line, false, black, 3, LINE_AA);
	cv::polylines(overlay, _mk4_line, false, black, 3, LINE_AA);	

	///// PLOT THE PATH ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///// START TO MK1
	_line1 = { Point2f(1.5 * cellWidth, rows - 2.5 * cellHeight), Point2f(1.5 * cellWidth, rows - 4.5 * cellHeight), Point2f(2.5 * cellWidth, rows - 6 * cellHeight), Point2f(zone_1.x + 0.5 * zoneWidth, zone_1.y + zoneHeight) };
	///// MK1 TO MK2 (VARIABLE)
	_line2 = { Point2f(zone_1.x + 0.5 * zoneWidth, zone_1.y), Point2f(2 * cellWidth, 3 * cellHeight), Point2f(3.5 * cellWidth, 2 * cellHeight), Point2f(zone_2.x, zone_2.y + 0.5 * zoneHeight) };
	///// MK2 (VARIABLE) TO MK3
	_line3 = { Point2f(zone_2.x + zoneWidth, zone_2.y + 0.5 * zoneHeight), Point2f(cols - 1.5 * cellWidth, 2.5 * cellHeight), Point2f(cols - 1.5 * cellWidth, 4.5 * cellHeight), Point2f(zone_3.x + 0.5 * zoneWidth, zone_1.y) };
	///// MK3 TO MK4
	_line4 = { Point2f(zone_3.x + 0.5 * zoneWidth, zone_1.y + zoneHeight), Point2f(cols - 3.5 * cellWidth, 9.5 * cellHeight), Point2f(cols - 4.5 * cellWidth, 10.5 * cellHeight), Point2f(cols - 4.5 * cellWidth, rows - 2.5 * cellHeight), Point2f(cols - 6.5 * cellWidth, rows - 2.5 * cellHeight), Point2f(zone_4.x + zoneWidth, zone_4.y + 0.5 * zoneHeight)};
	///// MK4 TO EXIT
	_line5 = { Point2f(zone_4.x, zone_4.y + 0.5 * zoneHeight), Point2f(4 * cellWidth, rows - 5.5 * cellHeight), Point2f(cols - 7.5 * cellWidth, rows - 6.5 * cellHeight), Point2f(cols - 5.5 * cellWidth, rows - 7.5 * cellHeight), Point2f(cols - 2.5 * cellWidth, rows - 6.5 * cellHeight), Point2f(cols - 1.5 * cellWidth, rows - 4.5 * cellHeight) };
	_line5f = { Point2f(cols - 1.5 * cellWidth, rows - 4.5 * cellHeight), Point2f(cols - 1.5 * cellWidth, rows - 2.5 * cellHeight), Point2f(cols - 1.5 * cellWidth, rows - 1.5 * cellHeight) };
	
	///// THIS SHIT SMOOTHS THE LINES OUT AND STOPS JANKY TURNS
	std::vector<Point> L1 = smooth(_line1);
	std::vector<Point> L2 = smooth(_line2);
	std::vector<Point> L3 = smooth(_line3);
	std::vector<Point> L4 = smooth(_line4);
	std::vector<Point> L5 = smooth(_line5);
	std::vector<Point> L5f = smooth(_line5f);
	
	///// PRINT THE MOTHERFUCKIN LINES
	cv::polylines(overlay, L1, false, black, 3, LINE_AA);
	cv::polylines(overlay, L2, false, black, 3, LINE_AA);
	cv::polylines(overlay, L3, false, black, 3, LINE_AA);
	cv::polylines(overlay, L4, false, black, 3, LINE_AA);
	cv::polylines(overlay, L5, false, black, 3, LINE_AA);
	cv::polylines(overlay, L5f, false, black, 3, LINE_AA);
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///// WELCOME TO THE GRID [TRON NOISES]
	// Draw horizontal lines
	for (int i = 1; i < gridRows; ++i) 
	{
		int y = i * rows / gridRows;
		cv::line(overlay, cv::Point(0, y), cv::Point(cols, y), cv::Scalar(0, 255, 0), 1);
	}

	// Draw vertical lines
	for (int j = 1; j < gridCols; ++j) {
		int x = j * cols / gridCols;
		cv::line(overlay, cv::Point(x, 0), cv::Point(x, rows), cv::Scalar(0, 255, 0), 1);
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///// THIS IS THE OVERLAY FACTOR SON
	double alpha = 0.45;  // Transparency factor [0 = invisible, 1 = opposite of invisible, 2 = ???????]
	cv::addWeighted(overlay, alpha, image, 1 - alpha, 0, image);

	cv::imshow("arena", image);
	cv::waitKey(0);
}

// Function to interpolate between two points
cv::Point2f Connections::interpolate(cv::Point2f p1, cv::Point2f p2, float t) 
{
	return cv::Point2f((1 - t) * p1.x + t * p2.x, (1 - t) * p1.y + t * p2.y);
}

// Recursive function to calculate Bezier curve point using de Casteljau's algorithm
cv::Point2f Connections::bezierPoint(float t, const std::vector<cv::Point2f>& controlPoints) 
{
	///// DON'T ASK QUESTIONS ABOUT THIS ONE, I DON'T UNDERSTAND IT
	std::vector<cv::Point2f> points = controlPoints;  // Copy control points to modify them
	int n = points.size();
	for (int r = 1; r < n; ++r) 
	{
		for (int i = 0; i < n - r; ++i) 
		{
			points[i] = interpolate(points[i], points[i + 1], t);
		}
	}
	return points[0];  // First element of the last remaining point
}

std::vector<cv::Point> Connections::smooth(std::vector<cv::Point2f>& line)
{
	// Calculate points on the Bezier curve
	std::vector<cv::Point> bezierPoints;
	for (float t = 0; t <= 1; t += 0.01)
	{
		bezierPoints.push_back(bezierPoint(t, line));
	}

	return bezierPoints;
}

cv::Point Connections::locateMkr2(const cv::Mat& image)
{
	// Define the region of interest as the top 10% of the image
	int roiHeight = image.rows * 0.1;  // Adjust this percentage as needed
	cv::Rect roi(0, 0, image.cols, roiHeight);  // Top portion of the image
	cv::Mat croppedImage = image(roi);

	// Convert the cropped image from BGR to HSV color space
	cv::Mat hsvImage;
	cv::cvtColor(croppedImage, hsvImage, cv::COLOR_BGR2HSV);

	// Define the range of blue colors in HSV
	cv::Scalar lowerBlue(_LowHue, _LowSat, _LowVal);  // Adjust these values
	cv::Scalar upperBlue(_HighHue, _HighSat, _HighVal); // Adjust these values
	cv::Mat blueMask;

	// Threshold the HSV image to get only blue colors
	cv::inRange(hsvImage, lowerBlue, upperBlue, blueMask);

	// Find contours in the mask
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(blueMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// Find the largest contour assuming it's the marker
	double maxArea = 0;
	cv::Rect boundingBox;
	for (const auto& contour : contours) {
		double area = cv::contourArea(contour);
		if (area > maxArea) {
			maxArea = area;
			boundingBox = cv::boundingRect(contour);
		}
	}
	// Adjust the marker's position relative to the full image
	cv::Point markerCenter(boundingBox.x + boundingBox.width / 2, boundingBox.y + boundingBox.height / 2 + roi.y);

	return markerCenter;
}

void Connections::calibrate()
{
	cv::imshow("Control", _control);
	cvui::init("Control");
	std::vector<std::vector<cv::Point>> contours;
	do
	{
		cv::Scalar lower(_LowHue, _LowSat, _LowVal);
		cv::Scalar upper(_HighHue, _HighSat, _HighVal);
		ControlPanelCal();
		cv::Mat hsv, cap, gMask;


		cv::Mat image = cv::imread("Arena.png", cv::IMREAD_COLOR);
		cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
		cv::inRange(hsv, lower, upper, gMask);
		cv::erode(gMask, gMask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		cv::dilate(gMask, gMask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		contours.clear();
		cv::findContours(gMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		drawBox(image, contours, "Blue", cv::Scalar(255, 0, 0));

		cv::imshow("Range", image);
		cv::imshow("Mask", gMask);


	} while (cv::waitKey(1) != 'q' && _do_exit == false);

	cv::destroyAllWindows();

}

double Connections::drawBox(cv::Mat& frame, const std::vector<std::vector<cv::Point>>& box, const std::string& label, const cv::Scalar& colour)
{

	double max = 0;
	double limit = 30;
	std::vector<cv::Point> largestArea;

	for (const auto& contour : box)
	{
		double area = cv::contourArea(contour);
		if (area > max && area >= limit)
		{
			max = area;
			largestArea = contour;
		}
	}

	if (!largestArea.empty())
	{
		cv::Rect boundRect = cv::boundingRect(largestArea);
		cv::rectangle(frame, boundRect, colour, 2);
		cv::putText(frame, label, cv::Point(boundRect.x, boundRect.y - 5), cv::FONT_HERSHEY_SIMPLEX, 0.75, colour, 2);
	}

	if (max > 0)
		return max;
	else
		return 0;
}

void Connections::ControlPanelCal()
{
	cv::Point gui_position;

	//Control Pane 
	gui_position = cv::Point(0, 0);
	cvui::window(_control, gui_position.x, gui_position.y, 250, 600, "RECYCLE CONTROL");

	//Low Hue Trackbar
	gui_position += cv::Point(15, 40);
	cvui::text(_control, gui_position.x, gui_position.y, "Low Hue");
	gui_position += cv::Point(0, 15);
	cvui::trackbar(_control, gui_position.x, gui_position.y, 200, &_LowHue, 0, 179);

	//High Hue Trackbar
	gui_position += cv::Point(0, 45);
	cvui::text(_control, gui_position.x, gui_position.y, "High Hue");
	gui_position += cv::Point(0, 15);
	cvui::trackbar(_control, gui_position.x, gui_position.y, 200, &_HighHue, 0, 179);

	//Low Saturation Trackbar
	gui_position += cv::Point(0, 45);
	cvui::text(_control, gui_position.x, gui_position.y, "Low Saturation");
	gui_position += cv::Point(0, 15);
	cvui::trackbar(_control, gui_position.x, gui_position.y, 200, &_LowSat, 0, 255);

	//High Saturation Trackbar
	gui_position += cv::Point(0, 45);
	cvui::text(_control, gui_position.x, gui_position.y, "High Saturation");
	gui_position += cv::Point(0, 15);
	cvui::trackbar(_control, gui_position.x, gui_position.y, 200, &_HighSat, 0, 255);

	//Low Value Trackbar
	gui_position += cv::Point(0, 45);
	cvui::text(_control, gui_position.x, gui_position.y, "Low Value");
	gui_position += cv::Point(0, 15);
	cvui::trackbar(_control, gui_position.x, gui_position.y, 200, &_LowVal, 0, 255);

	//High Value Trackbar
	gui_position += cv::Point(0, 45);
	cvui::text(_control, gui_position.x, gui_position.y, "High Value");
	gui_position += cv::Point(0, 15);
	cvui::trackbar(_control, gui_position.x, gui_position.y, 200, &_HighVal, 0, 255);

	//Exit button
	gui_position += cv::Point(-53, 40);
	if (cvui::button(_control, gui_position.x, gui_position.y, 100, 30, "EXIT"))
	{
		_do_exit = true;
	}
	cvui::update();
	cv::imshow("Control", _control);
}

void Connections::arenaData_thread(Connections* ptr)
{
	while (ptr->_do_exit == false)
	{
		ptr->arenaData();
	}
}
void Connections::arenaImage_thread(Connections* ptr)
{
	while (ptr->_do_exit == false)
	{
		ptr->arenaImage();
	}
}
void Connections::overlay_thread(Connections* ptr)
{
	while (ptr->_do_exit == false)
	{
		ptr->overlay();
	}
}

//double Connections::areaToDistance(double area, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
//	// Define parameters for distance conversion
//	// These are starting points and may need adjustment based on your setup and calibration
//	const double focalLength = 1000.0; // Focal length of the camera in pixels
//	const double markerSize = 100.0;   // Size of the ArUco marker in millimeters
//
//	// Convert area to distance using simple geometric relationship
//	// This is a basic implementation and may not provide accurate results without proper calibration
//	double distance = focalLength * markerSize / std::sqrt(area);
//
//	return distance;
//}
//
//
//void Connections::arucoMarkerTracking()
//{
//	cv::VideoCapture vid;
//	vid.open(0, CAP_DSHOW);
//	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
//	if (vid.isOpened() == TRUE)
//	{
//		do
//		{
//			cv::Mat frame;
//			vid >> frame;
//			if (frame.empty() == false)
//			{
//				std::vector<int> ids;
//				std::vector<std::vector<cv::Point2f> > corners;
//				cv::aruco::detectMarkers(frame, dictionary, corners, ids);
//				if (ids.size() > 0)
//				{
//					cv::aruco::drawDetectedMarkers(frame, corners, ids);
//
//					// Convert pixel measurements to real-world units
//					for (size_t i = 0; i < ids.size(); ++i) {
//						// Calculate area of the polygon formed by marker's corners
//						double area = 0.0;
//						for (size_t j = 0; j < corners[i].size(); ++j) {
//							size_t k = (j + 1) % corners[i].size();
//							area += corners[i][j].x * corners[i][k].y - corners[i][k].x * corners[i][j].y;
//						}
//						area = std::abs(area) / 2.0;
//
//						// Use the calculated area as the distance measurement
//						// You may need to calibrate this value based on known distances
//						double distance = area;
//
//						// Print or process the calculated distance
//						std::cout << "Marker " << ids[i] << " Distance: " << distance << " (arbitrary units)" << std::endl;
//					}
//
//					//cv::aruco::drawDetectedMarkers(frame, corners, ids);
//
//					//// Calculate area for each marker
//					//for (size_t i = 0; i < ids.size(); ++i) {
//					//   // Calculate area using the formula provided
//					//   double area = 0.0;
//					//   for (size_t j = 0; j < corners[i].size(); ++j) {
//					//      size_t k = (j + 1) % corners[i].size();
//					//      area += corners[i][j].x * corners[i][k].y - corners[i][k].x * corners[i][j].y;
//					//   }
//					//   area = std::abs(area) / 2.0;
//
//					//   // Print area or perform further processing
//					//   std::cout << "Marker " << ids[i] << " Area: " << area << " pixels" << std::endl;
//
//					//   // Optionally, you can convert pixel area to real-world units using camera calibration parameters
//					//}
//				}
//			}
//			cv::imshow("VID", frame);
//		} while (cv::waitKey(10) != 'q');
//	}
//}
