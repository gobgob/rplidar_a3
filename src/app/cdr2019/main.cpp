#include <unistd.h>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <csignal>
#include <vector>
#include "rplidar.h"
#include "DataSocket.hpp"

#define DEBUG true

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifdef _WIN32
#include <Windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <unistd.h>
static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}
#endif

using namespace rp::standalone::rplidar;

static const char* SERVER_ADDRESS=	"127.0.0.1";
static const uint16_t SERVER_PORT=		17685;
using namespace std;

bool running=true; //Variable to manage stops when signals are received
void signal_handler(int signo){
	if(signo==SIGTERM || signo==SIGINT)
		running = false;
}





string measures_to_string(rplidar_response_measurement_node_t measure)
{
	//printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
	//			    (measure.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ", 
	//			    (measure.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
	//			    measure.distance_q2/4.0f,
	//			    measure.sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
	
	ostringstream result;
	ostringstream angle;
	ostringstream distance;
	ostringstream quality;

	angle << std::fixed << setprecision(4) << (measure.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
	distance << std::fixed << std::setprecision(2) << measure.distance_q2/4.0f;
	quality << std::fixed << std::setprecision(2) << (measure.sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
	result << angle.str() << ":" << distance.str() << ":" << quality.str() << ";";
    return result.str();
}

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result op_result;

    // Health info
    rplidar_response_device_health_t healthinfo;
    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) {
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }
    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}


#include <signal.h>
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int main(int argc, char** argv)
{
	signal(SIGTERM, signal_handler);
	signal(SIGINT, signal_handler);
	running = true;
	const char * opt_com_path = NULL;
	opt_com_path = "/dev/ttyUSB0";
	_u32 baudrateArray[2] = {115200, 256000};
	ostringstream measures;
	u_result op_result;
	std::vector<RplidarScanMode> scanModes;
	rplidar_response_device_info_t devinfo;
	bool connectSuccess = false;

	// make connection...
	DataSocket HL(SERVER_ADDRESS, SERVER_PORT); // Connection to the client
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
	if (!drv) {
		fprintf(stderr, "insufficent memory, exit\n");
		exit(-2);
	}
	size_t baudRateArraySize = (sizeof(baudrateArray))/ (sizeof(baudrateArray[0]));
    for(size_t i = 0; i < baudRateArraySize; ++i)
    {
        if(!drv)
        {
            drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
        }
        if(IS_OK(drv->connect(opt_com_path, baudrateArray[i])))
        {
            op_result = drv->getDeviceInfo(devinfo);
            if (IS_OK(op_result))
            {
                connectSuccess = true;
                break;
            }
            else
            {
                delete drv;
                drv = NULL;
            }
        }
    }
	if (!connectSuccess) {
		fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
		, opt_com_path);
		drv->stop();
		drv->stopMotor();
		return -1;
	}

	// print out the device serial number, firmware and hardware version number..
	printf("RPLIDAR S/N: ");
	for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);

	// check health...
	if (!checkRPLIDARHealth(drv))
	{
		drv->stop();
		drv->stopMotor();
		return -1;
	}

	signal(SIGINT, ctrlc);

	while(running) {
		std::cout<<"Waiting for client..."<<std::endl;

		while(!HL.accept_client() && running);
		if(!running) continue;

		std::cout<<" Connected !"<<std::endl;

		drv->stop();
		drv->stopMotor();

		std::cout<<"Motor stopped!"<<std::endl;

		drv->startMotor();

		std::cout<<"Motor started!"<<std::endl;

		sleep(1);	//Let motor spin

		// start scan...
		/*drv->getAllSupportedScanModes(scanModes);
		drv->startScan(false, scanModes[0].id);*/
		drv->startScan(false, 1);

		std::cout<<"start express scan ok!"<<std::endl;

		int result = 0;
		do{
			rplidar_response_measurement_node_t nodes[8192];
			size_t   count = _countof(nodes);

			//Update current scan (one turn of measurements)
			op_result = drv->grabScanData(nodes, count);
			if (IS_OK(op_result)) 
			{
				drv->ascendScanData(nodes, count);
				for (int pos = 0; pos < (int)count ; ++pos) 
				{
					result=HL.send_data(measures_to_string(nodes[pos]).c_str());
					//measures << measures_to_string(nodes[pos]);
				}
				//measures << "M\0";

				//Send the data to client
				//std::cout << "Scan measure: " << measures.str().c_str();
				result=HL.send_data("M");
				//measures.flush();
				//	std::cout<<result<<std::endl;
			}
			else
			{
				std::cout << "Problem\n";
			}
			usleep(100);

			
		}while(result>=0 && running);
		drv->stop();
		drv->stopMotor(); //Stop motor if already running
	}
	drv->stop();
	drv->stopMotor();
	//drv->disconnect();
	return 0;
}

