#pragma once
#include "Client.h"
#include "CControl.h"


class Connections
{
private:
		
	float timeout_start_ma, timeout_start_d, timeout_start_i;
	bool _do_exit;
	std::string _d1, _d2, _d3, _d4;

public:
	Connections();
	~Connections();

	void send_command(CClient& client, std::string cmd);
	void manual();
	void autonomous();
	int main_menu();
	void print_menu();
	void parse_data(std::string& data, std::string& time, std::string& _d1, std::string& _d2, std::string& _d3, std::string& _d4);

	void arenaData();
	void arenaImage();


	static void arenaData_thread(Connections* ptr);
	static void arenaImage_thread(Connections* ptr);
	//static void carImage_thread();
};