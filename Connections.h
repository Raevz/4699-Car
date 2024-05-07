#pragma once
#include "Client.h"
#include "CControl.h"

#include <vector>

#define DEADZONE 5

class Connections
{
private:
		
	float timeout_start_ma, timeout_start_d, timeout_start_i;
	bool _do_exit, _auto, _reset, _start;
	std::string _d1, _d2, _d3, _d4;
	cv::Point mk2;
	double _alpha;

	float _Kp, _Ki, _Kd;
	int _lastError, _baseSpeed;

	int _HighHue, _LowHue;
	int _HighSat, _LowSat;
	int _HighVal, _LowVal;

	cv::Mat _control, _image, _overlay;
	cv::Rect zone_1, zone_2, zone_3, zone_4, _exit;
	std::vector<cv::Point2f> _line1, _line2, _line3, _line4, _line5, _line5f;
	std::vector<cv::Point> _mk1_line, _mk2_line, _mk3_line, _mk4_line;

	std::mutex imgrab, over;

public:
	Connections();
	~Connections();

	void send_command(CClient& client, std::string cmd);
	void manual();
	void autonomous();
	int main_menu();
	void print_menu();
	void parse_data(std::string& data, std::string& time, std::string& _d1, std::string& _d2, std::string& _d3, std::string& _d4);
	std::string joy(cv::Point2f& in);

	void arenaData();
	void arenaImage();
	void overlay();

	void img();
	cv::Point locateMkr2(const cv::Mat& image);

	
	void ControlPanelCal();
	void calibrate();
	double drawBox(cv::Mat& frame, const std::vector<std::vector<cv::Point>>& box, const std::string& label, const cv::Scalar& colour);


	cv::Point2f bezierPoint(float t, const std::vector<cv::Point2f>& controlPoints);
	cv::Point2f interpolate(cv::Point2f p1, cv::Point2f p2, float t);
	std::vector<cv::Point> smooth(std::vector<cv::Point2f>& line);

	static void arenaData_thread(Connections* ptr);
	static void arenaImage_thread(Connections* ptr);
	static void overlay_thread(Connections* ptr);
	//static void carImage_thread();

	void arucoMarkerTracking();
	double areaToDistance(double area, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
};