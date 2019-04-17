/*
 *  RPLIDAR A3
 *  Driver for the RPlidar A3
 *  Connects (and re-connects) automatically to the lidar
 *  Gather its measurement data and writes them on a socket
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <wiringPi.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header
#include "DataSocket.hpp"
#include "delay.h"

/* Settings */
#define SERVER_ADDRESS      "127.0.0.1"
#define SERVER_PORT         17685
#define DEFAULT_SERIAL_PORT "/dev/ttyAMA0"
#define DEFAULT_BAUDRATE    256000
#define DEFAULT_MOTOR_SPEED 165     // 8-bit PWM (default is 65% of the maximum speed)
#define MAX_FAILURE_COUNT   1       // maximum consecutive scan failures allowed before restarting the lidar
#define SORT_OUTPUT_DATA    1       // 1 => output data will be sorted by angle; 0 => output unsorted
#define LIDAR_SCAN_MODE     RPLIDAR_CONF_SCAN_COMMAND_BOOST
#define OUTPUT_BUFFER_SIZE  100

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;

/* Signal handler for CTRL+C */
bool ctrl_c_pressed = false;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

/* Check the operational status of the Lidar */
bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result op_result;
    rplidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) {
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            return false;
        }
        else {
            return true;
        }
    }
    else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

/* Set the rotation speed of the Lidar */
void runMotor(uint8_t pwm)
{
    if (pwm == 0) {
        digitalWrite(12, LOW);
    }
    else {
        digitalWrite(12, HIGH);
    }
}

int main(int argc, const char * argv[])
{
    signal(SIGINT, ctrlc);
    wiringPiSetupGpio();
    pinMode(12, OUTPUT);
    printf("SDK Version: %s\n", RPLIDAR_SDK_VERSION);

	DataSocket output_socket;
    const char * opt_com_path = DEFAULT_SERIAL_PORT;
    _u32 opt_com_baudrate = DEFAULT_BAUDRATE;
    uint8_t motor_speed = DEFAULT_MOTOR_SPEED;
    u_result op_result;
    rplidar_response_device_info_t devinfo;
    RplidarScanMode scanmode;
    rplidar_response_measurement_node_hq_t nodes[8192];
    char output_buffer[100] = { '\0', };

    // create the driver instance
	RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    // read serial port from the command line if specified...
    if (argc > 1)
    {
        opt_com_path = argv[1];
    }

    // read baud rate from the command line if specified...
    if (argc > 2)
    {
        unsigned long bd = strtoul(argv[2], NULL, 10);
        if (bd > 0) {
            opt_com_baudrate = bd;
        }
        else {
            fprintf(stderr, "invalid baudrate provided, ignored\n");
        } 
    }

    // read motor speed from the command line if specified...
    if (argc > 3)
    {
        unsigned long speed = strtoul(argv[3], NULL, 10);
        if (speed > 0) {
            motor_speed = speed;
        }
        else {
            fprintf(stderr, "invalid motor speed provided, ignored\n");
        }
    }

    // try to open the serial port
    if(IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate)))
    {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s, exit\n"
            , opt_com_path);
        RPlidarDriver::DisposeDriver(drv);
        drv = NULL;
        exit(-3);
    }

    printf("Serial port %s opened with baudrate %u\n", opt_com_path, opt_com_baudrate);
    
    // try to open the output socket
    int ret = output_socket.open(SERVER_ADDRESS, SERVER_PORT);
    if (ret != 0) {
        fprintf(stderr, "Error, cannot open the socket %s:%u, exit\n",
            SERVER_ADDRESS, SERVER_PORT);
        RPlidarDriver::DisposeDriver(drv);
        drv = NULL;
        exit(ret);
    }

    printf("Socket opened on %s:%u\n", SERVER_ADDRESS, SERVER_PORT);

    while (!ctrl_c_pressed)
    {
        output_socket.accept_client();

        // Try to get S/N from the lidar
        op_result = drv->getDeviceInfo(devinfo);
        if (IS_OK(op_result))
        {// print out the device serial number, firmware and hardware version number..
            printf("RPLIDAR S/N: ");
            for (int pos = 0; pos < 16; pos++) {
                printf("%02X", devinfo.serialnum[pos]);
            }
            printf("\nFirmware Ver: %d.%02d\nHardware Rev: %d\n",
                devinfo.firmware_version>>8,
                devinfo.firmware_version & 0xFF,
                (int)devinfo.hardware_version
            );
        }
        else
        {
            fprintf(stderr, "Failed to communicate with the lidar\n");
            continue;
        }

        // check health...
        if (!checkRPLIDARHealth(drv)) {
            drv->reset();
            delay(1000);
            continue;
        }

        // spin motor...
        runMotor(motor_speed);
        // start scan...
        op_result = drv->startScanExpress(false, LIDAR_SCAN_MODE, 0, &scanmode);
        if (IS_FAIL(op_result)) {
            drv->stop();
            runMotor(0);
            fprintf(stderr, "Failed to start scan\n");
            continue;
        }

        // fetch results and print them out...
        int fail_count = 0;
        while (!ctrl_c_pressed && fail_count <= MAX_FAILURE_COUNT)
        {
            output_socket.accept_client();
            size_t count = _countof(nodes);
            op_result = drv->grabScanDataHq(nodes, count);
            if (IS_FAIL(op_result)) {
                fail_count++;
                continue;
            }

#if SORT_OUTPUT_DATA
            op_result = drv->ascendScanData(nodes, count);
            if (IS_FAIL(op_result)) {
                fail_count++;
                continue;
            }
#endif
            for (size_t pos = 0; pos < count ; pos++)
            {
                float angle_deg = nodes[pos].angle_z_q14 * 90.f / 16384.0f;
                float dist_mm = nodes[pos].dist_mm_q2 / 4.0f;
                uint8_t quality = nodes[pos].quality;
                printf("Theta: %03.2f Dist: %08.2f Q: %u\n", angle_deg, dist_mm, quality);
                int ret = snprintf(output_buffer, OUTPUT_BUFFER_SIZE,
                    "%.4f:%.2f:%u;", angle_deg, dist_mm, quality);
                if (ret < 0) {
                    fprintf(stderr, "Failed format output\n");
                    continue;
                }
                if (ret >= OUTPUT_BUFFER_SIZE) {
                    fprintf(stderr, "Output buffer is too small\n");
                    continue;
                }
                output_socket.send_data(output_buffer);
            }
            output_socket.send_data("M");
            fail_count = 0;
        }
        if (!ctrl_c_pressed) {
            drv->stop();
            runMotor(0);
            fprintf(stderr, "Lidar disconnected\n");
        }
    }

    drv->stop();
    runMotor(0);
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return 0;
}
