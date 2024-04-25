#include "stdafx.h"
#include "Connections.h"
#include "cvui.h"

const std::string Car_IP = "192.168.137.232";
const std::string Arena_IP = "192.168.0.101";
const int Car_Port_cmd = 6969;
const int ArenaData_Port = 4008;
const int ArenaImage_Port = 5008;

Connections::Connections()
{
	_do_exit = false;

	timeout_start_ma = 0;
	timeout_start_d = 0;
	timeout_start_i = 0;
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
	/*t1.join();
	t2.join();*/
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
	//timeout_start_ma = cv::getTickCount();
	client.connect_socket(Car_IP, Car_Port_cmd);
	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(300));
	send_command(client, "G 0 \n");
	send_command(client, "\n");

	while (!_do_exit)
	{
		cv::Point2f analogInput = Control.get_analog();

		int x = Control.percentToInt(analogInput.x);
		int y = Control.percentToInt(analogInput.y);
		int t = Control.get_button(BOOSTER_BUTTON_2);

		client.tx_str("S " + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(t) + " \n");

		std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(5));
	}

}

void Connections::autonomous()
{
	//WIP
}

void Connections::arenaData()
{
	CClient client;
	client.connect_socket(Arena_IP, ArenaData_Port);
	std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(500));
	timeout_start_d = cv::getTickCount();
	std::string cmd = "G 0";
	std::string data;
	cv::Mat control = cv::Mat::zeros(150, 200, CV_8UC3);
	cv::imshow("DATA", control);
	cvui::init("DATA");

	do
	{
		client.tx_str(cmd);		
		data.clear();
		
		if (client.rx_str(data))
		{
			std::string time;
			parse_data(data, time, _d1, _d2, _d3, _d4);
			
			control = cv::Scalar(49, 52, 49);  // Dark gray background
			cv::Point gui_position;

			// Set text properties
			int fontFace = cv::FONT_HERSHEY_SIMPLEX;
			double fontScale = 0.5;
			int thickness = 1;
			int baseline = 0;
			int lineHeight = cv::getTextSize("Text", fontFace, fontScale, thickness, &baseline).height + 5;

			//TIMER
			gui_position = cv::Point(10, 20);
			cv::putText(control, "TIME: " + time, gui_position, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);

			// MARKER 1
			gui_position.y += lineHeight;
			cv::putText(control, "LEFT: " + _d1, gui_position, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);

			// MARKER 2
			gui_position.y += lineHeight;
			cv::putText(control, "TOP: " + _d2, gui_position, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);

			// MARKER 3
			gui_position.y += lineHeight;
			cv::putText(control, "RIGHT: " + _d3, gui_position, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);

			// MARKER 4
			gui_position.y += lineHeight;
			cv::putText(control, "BOTTOM: " + _d4, gui_position, fontFace, fontScale, cv::Scalar(255, 255, 255), thickness);

			// EXIT BUTTON
			gui_position = cv::Point((control.cols - 100) / 2, control.rows - 50); // Centered at bottom
			if (cvui::button(control, gui_position.x, gui_position.y, 100, 30, "EXIT")) {
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
		cv::imshow("DATA", control);
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

	cv::Mat im;

	do
	{
		client.tx_str("G 1");
		std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(30));
		if (client.rx_im(im) == true)
		{
			timeout_start_i = cv::getTickCount();
			if (im.empty() == false)
			{
				//std::cout << "\nClient Rx: Image received";
				cv::imshow("Arena Image", im);
				cv::waitKey(10);
			}
		}
		else
		{
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
