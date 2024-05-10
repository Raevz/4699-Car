#include "stdafx.h"
#include "Connections.h"
#include "cvui.h"

const std::string Car_IP = "192.168.0.112";
const std::string Arena_IP = "192.168.0.100";
const int Car_Port_cmd = 6969;
const int ArenaData_Port = 4008;
const int ArenaImage_Port = 5008;


using namespace cv;


Connections::Connections()
{
	_do_exit = false;
	_auto = false;
	_reset = false;	

	_Kp = 0.07;
	_Ki = 0.0008;
	_Kd = 0.6;
	_lastError = 0;

	timeout_start_ma = 0;
	timeout_start_d = 0;
	timeout_start_i = 0;
	_baseSpeed = 200;

	///// THESE IDENTIFY THE BLUE FOR MKR2 ///// CALIBRATION VALUES
	_LowHue = 109;
	_HighHue = 140;
	_LowSat = 164;
	_HighSat = 255;
	_LowVal = 57;
	_HighVal = 160;
	//////////////////////////////////////

	low_mk2 = cv::Scalar(109, 164, 57);
	hi_mk2 = cv::Scalar(140, 255, 160);
	low_others = cv::Scalar(93, 32, 51);
	hi_others = cv::Scalar(115, 157, 182);

	_control = cv::Mat::zeros(cv::Size(250, 500), CV_8UC1);
	_control = cv::Scalar(49, 52, 49);  // Dark gray background
	_cal = cv::Mat::zeros(cv::Size(250, 500), CV_8UC1);

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
	t1.detach();
	std::thread t2(&Connections::arenaImage_thread, this);	
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
	CClient client;
	client.connect_socket(Car_IP, Car_Port_cmd);
	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(300));
	
	do
	{
		arucoMarkerTracking();
		sendControlCommands(client);

		//client.tx_str("S " + std::to_string(y) + " " + std::to_string(x) + " " + std::to_string(t) + " \n");
		/*if (_exit.contains(position))
		{
			client.tx_str("S +00 +00 0 \n");
			_auto = false;
		}*/
	} while (!_do_exit);
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
		_control = cv::Mat::zeros(cv::Size(250, 500), CV_8UC1);
		_control = cv::Scalar(49, 52, 49);  // Dark gray background
		
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
			gui_position += Point(-10, 20);
			cvui::window(_control, gui_position.x, gui_position.y, 250, 400, "CONTROLS");

			///////////// CONTROLS FOR PID VALUES //////////////////////////////////////////////////////////
			gui_position += Point(10, 40);
			cvui::text(_control, gui_position.x, gui_position.y, "Kp");
			gui_position += cv::Point(0, 15);
			cvui::trackbar(_control, gui_position.x, gui_position.y, 200, &_Kp, 0.0f, 0.2f);

			gui_position += Point(0, 40);
			cvui::text(_control, gui_position.x, gui_position.y, "Ki");
			gui_position += cv::Point(0, 15);
			cvui::trackbar(_control, gui_position.x, gui_position.y, 200, &_Ki, 0.0f, 0.05f);

			gui_position += Point(0, 60);
			cvui::text(_control, gui_position.x, gui_position.y, "Kd");
			gui_position += cv::Point(0, 15);
			cvui::trackbar(_control, gui_position.x, gui_position.y, 200, &_Kd, 0.0f, 1.0f);
			////////////////////////////////////////////////////////////////////////////////////////////////
			// AUTO BUTTON
			gui_position += cv::Point(0, 40);
			cvui::checkbox(_control, gui_position.x, gui_position.y, "Auto Mode", &_auto);

			// RESET BUTTON
			gui_position += cv::Point(65, 65);
			if (cvui::button(_control, gui_position.x, gui_position.y, 100, 30, "RESET")) 
			{
				_reset = true;
			}
			
			// EXIT BUTTON
			gui_position = cv::Point((_control.cols - 100) / 2, _control.rows - 50); // Centered at bottom
			if (cvui::button(_control, gui_position.x, gui_position.y, 100, 30, "EXIT")) 
			{
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

	/////FOR SETUP OF LIVE IMAGE COLOURS
	/*std::thread t3(&Connections::calibrate_thread, this);
	t3.detach();*/

	std::thread t_overlay(&Connections::overlay_thread, this);
	t_overlay.detach();

	do
	{
		client.tx_str("G 1");
		std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(20));
		imgrab.lock();
		if (client.rx_im(_image) == true)
		{
			
			timeout_start_i = cv::getTickCount();
			if (_image.empty() == false)
			{		
				if (!_overlay.empty())
				{
					cv::addWeighted(_overlay, _alpha, _image, 1 - _alpha, 0, _image);
				}
				cv::imshow("Arena Image", _image);
				
				cv::waitKey(10);
			}
			imgrab.unlock();
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

void Connections::overlay_init(const cv::Mat& overlay)
{
	int rows = overlay.rows;
	int cols = overlay.cols;

	// Number of cells in the grid
	int gridRows = 15;
	int gridCols = 15;
	int mkrArea = 40;

	///// JUST SOME SETUP CODE
	int cellWidth = cols / gridCols;
	int cellHeight = rows / gridRows;
	int zoneWidth = 3 * cellWidth;
	int zoneHeight = 3 * cellHeight;
	int middleMarkersY = 6 * cellHeight;

	///// FIND THE MARKERs
	cv::Point mk1 = locateGenericMkr(overlay, 1);
	cv::Point mk2 = locateMkr2(overlay);
	cv::Point mk3 = locateGenericMkr(overlay, 3);
	cv::Point mk4 = locateGenericMkr(overlay, 4);

	cv::Rect _mk1Box = cv::Rect(mk1.x - mkrArea / 2, mk1.y - mkrArea / 2, mkrArea, mkrArea);
	cv::Rect _mk2Box = cv::Rect(mk2.x - mkrArea / 2, mk2.y - mkrArea / 2, mkrArea, mkrArea);
	cv::Rect _mk3Box = cv::Rect(mk3.x - mkrArea / 2, mk3.y - mkrArea / 2, mkrArea, mkrArea);
	cv::Rect _mk4Box = cv::Rect(mk4.x - mkrArea / 2, mk4.y - mkrArea / 2, mkrArea, mkrArea);

	///// MAKE THE SHOOT ZONES IN FRONT OF THE MARKERS/////////////////////////////////////////////////////////////////////////////////
	cv::Rect zone_1 = cv::Rect(mk1.x + cellWidth, mk1.y - 1.5 * cellHeight, zoneWidth, zoneHeight);
	cv::Rect zone_2 = cv::Rect(mk2.x - cellWidth, mk2.y + cellHeight, zoneWidth, zoneHeight);
	cv::Rect zone_3 = cv::Rect(mk3.x - (cellWidth + zoneWidth), mk3.y - 1.5 * cellHeight, zoneWidth, zoneHeight);
	cv::Rect zone_4 = cv::Rect(mk4.x - 1.5 * cellWidth, mk4.y - (cellHeight + zoneHeight), zoneWidth, zoneHeight);
	cv::Rect exit = cv::Rect(cols - zoneWidth, rows - zoneHeight, zoneWidth, zoneHeight);

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
	_L1 = smooth(_line1);
	_L2 = smooth(_line2);
	_L3 = smooth(_line3);
	_L4 = smooth(_line4);
	_L5 = smooth(_line5);
	_L5f = smooth(_line5f);

	std::cout << "\n\nPATH READY\n\n";
}

void Connections::overlay()
{
	while(_image.empty())
	{
		std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(100));
	}
	cv::Mat OG, blank;
	imgrab.lock();
	_image.copyTo(OG);
	imgrab.unlock();
	overlay_init(OG);
	blank = cv::Mat::zeros(OG.size(), CV_8UC4);
	blank.copyTo(_overlay);

	cv::Scalar black = cv::Scalar(0, 0, 0);
	cv::Scalar rectColor(180, 0, 0); // blue color for the rectangle

	_reset = false;	

	do
	{
		if (_reset)
		{
			while (_image.empty())
			{
				std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(10));
			}

			imgrab.lock();
			_image.copyTo(OG);
			imgrab.unlock();
			overlay_init(OG);
			blank = cv::Mat::zeros(OG.size(), CV_8UC4);
			blank.copyTo(_overlay);
			_reset = false;
		}
		over.lock();
		blank.copyTo(_overlay);

		cv::rectangle(_overlay, zone_1, rectColor, -1); // -1 means filled
		cv::rectangle(_overlay, _mk1Box, cv::Scalar(0, 255, 0), 3); // -1 means filled

		cv::rectangle(_overlay, zone_2, rectColor, -1); // -1 means filled
		cv::rectangle(_overlay, _mk2Box, cv::Scalar(0, 255, 0), 3); // -1 means filled

		cv::rectangle(_overlay, zone_3, rectColor, -1); // -1 means filled
		cv::rectangle(_overlay, _mk3Box, cv::Scalar(0, 255, 0), 3); // -1 means filled

		cv::rectangle(_overlay, zone_4, rectColor, -1); // -1 means filled
		cv::rectangle(_overlay, _mk4Box, cv::Scalar(0, 255, 0), 3); // -1 means filled
		cv::rectangle(_overlay, _exit, cv::Scalar(180, 60, 255), -1); // -1 means filled
		over.unlock();
		if (_d1 == "0" && _d2 == "0" && _d3 == "0" && _d4 == "0")
		{
			over.lock();
			cv::polylines(_overlay, _mk1_line, false, black, 3, LINE_AA);
			cv::polylines(_overlay, _L1, false, black, 3, LINE_AA);	
			over.unlock();

			if (_activeLine != _L1)
			{
				_activeLine = _L1;
			}
		}
		if (_d1 == "1" && _d2 == "0" && _d3 == "0" && _d4 == "0")
		{
			over.lock();			
			cv::polylines(_overlay, _mk2_line, false, black, 3, LINE_AA);
			cv::polylines(_overlay, _L2, false, black, 3, LINE_AA);			
			over.unlock();

			if (_activeLine != _L2)
			{
				_activeLine = _L2;
			}
		}
		if (_d1 == "1" && _d2 == "1" && _d3 == "0" && _d4 == "0")
		{
			over.lock();			
			cv::polylines(_overlay, _mk3_line, false, black, 3, LINE_AA);
			cv::polylines(_overlay, _L3, false, black, 3, LINE_AA);			
			over.unlock();

			if (_activeLine != _L3)
			{
				_activeLine = _L3;
			}
		}
		if (_d1 == "1" && _d2 == "1" && _d3 == "1" && _d4 == "0")
		{
			over.lock();			
			cv::polylines(_overlay, _mk4_line, false, black, 3, LINE_AA);
			cv::polylines(_overlay, _L4, false, black, 3, LINE_AA);			
			over.unlock();		

			if (_activeLine != _L4)
			{
				_activeLine = _L4;
			}
		}
		if (_d1 == "1" && _d2 == "1" && _d3 == "1" && _d4 == "1")
		{
			over.lock();			
			cv::polylines(_overlay, _L5, false, black, 3, LINE_AA);			
			cv::polylines(_overlay, _L5f, false, black, 3, LINE_AA);
			over.unlock();		

			if (_activeLine != _L5)
			{
				_activeLine = _L5;
			}
		}
		std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(100));
	} while (!_do_exit);
}

void Connections::arucoMarkerTracking() 
{
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	// Detect markers
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners;
	imgrab.lock();
	if (!_image.empty()) {
		cv::aruco::detectMarkers(_image, dictionary, corners, ids);
		int idx = -1;
		for (int i = 0; i < ids.size(); i++) {
			if (ids[i] == 4) {  // Assuming '4' is the ID of the car's marker
				idx = i;
				break;
			}
		}
		imgrab.unlock();

		if (idx != -1) {
			// Marker is detected
			std::vector<cv::Point2f> marker_corners = corners[idx];
			//cv::aruco::drawDetectedMarkers(_image, corners, ids);

			// Calculate the center and orientation of the marker
			cv::Point2f center(0, 0);
			for (const cv::Point2f& corner : marker_corners) {
				center += corner;
			}
			center.x /= 4.0;
			center.y /= 4.0;

			// Orientation towards the second corner for a more frontal alignment reference
			cv::Point2f vector = marker_corners[1] - marker_corners[0];
			float angle = atan2(vector.y, vector.x);

			// PID Control calculations
			updatePIDControl(center, angle);
		}
		else {
			imgrab.unlock();
		}
	}
	else {
		imgrab.unlock();
	}
}
void Connections::updatePIDControl(const cv::Point2f& currentPos, float currentAngle) 
{
	// Find the nearest point on the path
	double minDistance = std::numeric_limits<double>::max();
	cv::Point2f nearestPoint;
	for (const cv::Point2f& point : _activeLine) 
	{
		double dist = cv::norm(currentPos - point);
		if (dist < minDistance) 
		{
			minDistance = dist;
			nearestPoint = point;
		}
	}

	// Calculate cross-track error
	double cte = cv::pointPolygonTest(_activeLine, currentPos, true);

	// Calculate orientation error
	// Assuming the desired path direction between two successive points
	cv::Vec2f pathDirection = nearestPoint - (currentPos - cv::Point2f(cos(currentAngle), sin(currentAngle)) * 100.0f);
	float pathAngle = atan2(pathDirection[1], pathDirection[0]);
	float angleError = pathAngle - currentAngle;

	// Normalize angle error to be within -pi to pi
	angleError = atan2(sin(angleError), cos(angleError));

	// Apply PID control to calculate steering and throttle
	_steering = _Kp * cte + _Kd * (cte - _lastError) + _Ki * _sumError;
	_throttle = _baseSpeed - fabs(_steering); // Reduce speed on curves

	// Update for next iteration
	_lastError = _error;
	_sumError += _error;
}
void Connections::sendControlCommands(CClient& client) 
{
	// Convert throttle from -1.0 to 1.0 range to -200 to 200
	int throttleValue = static_cast<int>(_throttle * 200);
	// Clamp the throttle value to ensure it stays within the expected range
	throttleValue = std::max(-200, std::min(throttleValue, 200));

	// Convert steering from -1.0 to 1.0 range to -55 to 55
	int steeringValue = static_cast<int>(_steering * 55);
	// Clamp the steering value to ensure it stays within the expected range
	steeringValue = std::max(-55, std::min(steeringValue, 55));

	// Format the throttle and steering values for the command string
	std::string throttleCmd = (throttleValue >= 0 ? "+" : "-") + std::to_string(throttleValue);
	std::string steeringCmd = (steeringValue >= 0 ? "+" : "-") + std::to_string(abs(steeringValue));

	// Turret control (placeholder, replace with actual logic)
	char turretControl = '0'; // Default to '0'

	// Create the command string
	std::string cmd = "S " + throttleCmd + " " + steeringCmd + " " + turretControl + " \n";

	// Send the command to the car
	client.tx_str(cmd);
}
//void Connections::arucoMarkerTracking() {
//	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
//
//
//	// Detect markers
//	std::vector<int> ids;
//	std::vector<std::vector<cv::Point2f>> corners;
//	imgrab.lock();
//	
//	
//	auto it = std::find(ids.begin(), ids.end(), 4);
//
//	
//	if(!_image.empty())
//	{
//		if (it != ids.end())
//		{
//			cv::aruco::detectMarkers(_image, dictionary, corners, ids);
//			// Extract marker corner points
//			cv::Point2f tl = corners[0][0];
//			cv::Point2f br = corners[0][2];
//
//			// Calculate the center point of the marker
//			cv::Point2f marker_center((tl.x + br.x) / 2, (tl.y + br.y) / 2);
//
//
//			// Draw detected markers
//			cv::aruco::drawDetectedMarkers(_image, corners, ids);
//
//			// Draw a red rectangle around the top-left corner
//			cv::rectangle(_image, tl, tl + cv::Point2f(10, 10), cv::Scalar(0, 0, 255), 2);
//
//			// Draw a dot in the middle of the marker
//			cv::circle(_image, marker_center, 5, cv::Scalar(0, 255, 255), -1);
//
//			// Determine whether to go left or right based on the position of the marker relative to the polyline
//			double distance = cv::pointPolygonTest(_activeLine, marker_center, true);
//			
//			if (distance < 0)
//			{
//				std::cout << "Go Left" << std::endl;
//			}
//			else if (distance > 0)
//			{
//				std::cout << "Go Right" << std::endl;
//			}
//			else
//			{
//				std::cout << "Checkpoint Detected" << std::endl;
//			}
//		}
//	}
//	imgrab.unlock();
//}
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

	int mkrArea = 40;	

	///// FIND THE MARKERs
	cv::Point mk1 = locateGenericMkr(image, 1);
	cv::Point mk2 = locateMkr2(image);	
	cv::Point mk3 = locateGenericMkr(image, 3);
	cv::Point mk4 = locateGenericMkr(image, 4);

	cv::Rect mk1Box = cv::Rect(mk1.x - mkrArea / 2, mk1.y - mkrArea / 2, mkrArea, mkrArea);
	cv::Rect mk2Box = cv::Rect(mk2.x - mkrArea / 2, mk2.y - mkrArea / 2, mkrArea, mkrArea);
	cv::Rect mk3Box = cv::Rect(mk3.x - mkrArea / 2, mk3.y - mkrArea / 2, mkrArea, mkrArea);
	cv::Rect mk4Box = cv::Rect(mk4.x - mkrArea / 2, mk4.y - mkrArea / 2, mkrArea, mkrArea);

	cv::rectangle(overlay, mk1Box, cv::Scalar(0,255,0), 3); // -1 means filled
	cv::rectangle(overlay, mk2Box, cv::Scalar(0, 255, 0), 3); // -1 means filled
	cv::rectangle(overlay, mk3Box, cv::Scalar(0, 255, 0), 3); // -1 means filled
	cv::rectangle(overlay, mk4Box, cv::Scalar(0, 255, 0), 3); // -1 means filled
	///// MAKE THE SHOOT ZONES IN FRONT OF THE MARKERS/////////////////////////////////////////////////////////////////////////////////
	cv::Rect zone_1 = cv::Rect(mk1.x + cellWidth, mk1.y - 1.5*cellHeight, zoneWidth, zoneHeight);
	cv::Rect zone_2 = cv::Rect(mk2.x - cellWidth, mk2.y + cellHeight, zoneWidth, zoneHeight);
	cv::Rect zone_3 = cv::Rect(mk3.x - (cellWidth + zoneWidth), mk3.y - 1.5*cellHeight, zoneWidth, zoneHeight);
	cv::Rect zone_4 = cv::Rect(mk4.x - 1.5*cellWidth, mk4.y - (cellHeight + zoneHeight), zoneWidth, zoneHeight);
	///// THESE ARE THE GENERIC / HARDCODED ZONES (NOT YOU ZONE 2, YOU'RE DOING GREAT)
	/*cv::Rect zone_1 = cv::Rect(cellWidth, middleMarkersY, zoneWidth, zoneHeight);
	cv::Rect zone_2 = cv::Rect(mk2.x - cellWidth, mk2.y + cellHeight, zoneWidth, zoneHeight);
	cv::Rect zone_3 = cv::Rect(cols - (zoneWidth + cellWidth), middleMarkersY, zoneWidth, zoneHeight);
	cv::Rect zone_4 = cv::Rect(cols/2 - zoneWidth/2, cols - (zoneHeight + cellHeight), zoneWidth, zoneHeight);*/
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
	_line4 = { Point2f(zone_3.x + 0.5 * zoneWidth, zone_1.y + zoneHeight), Point2f(cols - 3.5 * cellWidth, 9.5 * cellHeight), Point2f(cols - 4.5 * cellWidth, 10.5 * cellHeight), Point2f(cols - 4.5 * cellWidth, rows - 2.5 * cellHeight), /*Point2f(cols - 6.5 * cellWidth, rows - 2.5 * cellHeight),*/ Point2f(zone_4.x + zoneWidth, zone_4.y + 0.5 * zoneHeight)};
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

	cv::Mat Mask;

	// Threshold the HSV image to get only blue colors
	cv::inRange(hsvImage, low_mk2, hi_mk2, Mask);

	// Find contours in the mask
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(Mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

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

cv::Point Connections::locateGenericMkr(const cv::Mat& image, int mkr)
{
	cv::Mat croppedImage;
	cv::Rect roi;
	/////////////////DEPENDS ON WHICH MARKER WE WANT///////////////////////////////////////////////////////////////////
	switch (mkr)
	{
		case 1: 
		{
			// Define the region of interest as the left 10% of the image
			int roiWidth = image.cols * 0.05;  // Adjust this percentage as needed
			int roiHeight = image.rows * 0.1;
			roi = cv::Rect(0, image.rows/2 - roiHeight / 2, roiWidth, roiHeight);
			croppedImage = image(roi);
			break;
		}
		case 3:
		{
			// Define the region of interest as the right 10% of the image
			int roiWidth = image.cols * 0.05;  // Adjust this percentage as needed
			int roiHeight = image.rows * 0.1;
			roi = cv::Rect(image.cols - roiWidth, image.rows / 2 - roiHeight/2, roiWidth, roiHeight);
			croppedImage = image(roi);
			break;
		}
		case 4:
		{
			// Define the region of interest as the bottom 10% of the image
			int roiWidth = image.cols * 0.1;
			int roiHeight = image.rows * 0.05;  // Adjust this percentage as needed
			roi = cv::Rect(image.cols/2 - roiWidth/2, image.rows - roiHeight, roiWidth, roiHeight);
			croppedImage = image(roi);
			break;
		}
		default:
		{
			return cv::Point(0, 0);
			break;
		}
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Convert the cropped image from BGR to HSV color space
	cv::Mat hsvImage;
	cv::cvtColor(croppedImage, hsvImage, cv::COLOR_BGR2HSV);

	cv::Mat Mask;

	// Threshold the HSV image to get only blue colors
	cv::inRange(hsvImage, low_others, hi_others, Mask);

	// Find contours in the mask
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(Mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

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
	cv::Point markerCenter(boundingBox.x + boundingBox.width / 2 + roi.x, boundingBox.y + boundingBox.height / 2 + roi.y);

	return markerCenter;
}

void Connections::calibrate()
{
	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(5000));
	
	cv::imshow("Calibrate", _cal);
	cvui::init("Calibrate");
	std::vector<std::vector<cv::Point>> contours;
	do
	{
		cv::Scalar lower(_LowHue, _LowSat, _LowVal);
		cv::Scalar upper(_HighHue, _HighSat, _HighVal);
		ControlPanelCal();
		cv::Mat hsv, cap, gMask;

		// COMMENT THIS OUT FOR REAL USE/////////////////////////////
		cv::Mat _image = cv::imread("Arena.png", cv::IMREAD_COLOR);
		/////////////////////////////////////////////////////////////
		imgrab.lock();
		if(!_image.empty())
		{	
			cv::cvtColor(_image, hsv, cv::COLOR_BGR2HSV);
			cv::inRange(hsv, lower, upper, gMask);
			cv::erode(gMask, gMask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			cv::dilate(gMask, gMask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			contours.clear();
			cv::findContours(gMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);			
			drawBox(_image, contours, "Blue", cv::Scalar(255, 0, 0));
			
			// COMMENT THIS OUT FOR REAL USE/////////////////////////////
			cv::imshow("Range", _image);	
			/////////////////////////////////////////////////////////////
			cv::imshow("Mask", gMask);
		}
		imgrab.unlock();
		
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
	cvui::window(_cal, gui_position.x, gui_position.y, 250, 600, "colors");

	//Low Hue Trackbar
	gui_position += cv::Point(15, 40);
	cvui::text(_cal, gui_position.x, gui_position.y, "Low Hue");
	gui_position += cv::Point(0, 15);
	cvui::trackbar(_cal, gui_position.x, gui_position.y, 200, &_LowHue, 0, 179);

	//High Hue Trackbar
	gui_position += cv::Point(0, 45);
	cvui::text(_cal, gui_position.x, gui_position.y, "High Hue");
	gui_position += cv::Point(0, 15);
	cvui::trackbar(_cal, gui_position.x, gui_position.y, 200, &_HighHue, 0, 179);

	//Low Saturation Trackbar
	gui_position += cv::Point(0, 45);
	cvui::text(_cal, gui_position.x, gui_position.y, "Low Saturation");
	gui_position += cv::Point(0, 15);
	cvui::trackbar(_cal, gui_position.x, gui_position.y, 200, &_LowSat, 0, 255);

	//High Saturation Trackbar
	gui_position += cv::Point(0, 45);
	cvui::text(_cal, gui_position.x, gui_position.y, "High Saturation");
	gui_position += cv::Point(0, 15);
	cvui::trackbar(_cal, gui_position.x, gui_position.y, 200, &_HighSat, 0, 255);

	//Low Value Trackbar
	gui_position += cv::Point(0, 45);
	cvui::text(_cal, gui_position.x, gui_position.y, "Low Value");
	gui_position += cv::Point(0, 15);
	cvui::trackbar(_cal, gui_position.x, gui_position.y, 200, &_LowVal, 0, 255);

	//High Value Trackbar
	gui_position += cv::Point(0, 45);
	cvui::text(_cal, gui_position.x, gui_position.y, "High Value");
	gui_position += cv::Point(0, 15);
	cvui::trackbar(_cal, gui_position.x, gui_position.y, 200, &_HighVal, 0, 255);

	//Exit button
	gui_position += cv::Point(0, 40);
	if (cvui::button(_cal, gui_position.x, gui_position.y, 100, 30, "EXIT"))
	{
		_do_exit = true;
	}
	cvui::update();
	cv::imshow("Calibrate", _cal);
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
void Connections::calibrate_thread(Connections* ptr)
{
	while (ptr->_do_exit == false)
	{
		ptr->calibrate();
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
