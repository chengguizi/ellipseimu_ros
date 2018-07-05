// ROS Node for SBG Ellipse N IMU Sensor
// Cheng Huimin, June 2018
//
//

#include <sbgEComLib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cassert>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>

ros::Publisher _imu_pub;
ros::Publisher _mag_pub;
ros::Publisher _alt_pub;

sensor_msgs::MagneticField _mag_msg;
sensor_msgs::Imu _imu_msg;
sensor_msgs::FluidPressure _alt_msg;

ros::Time _ros_time_first_frame;
uint32 _imu_time_first_frame;

//----------------------------------------------------------------------//
//  Call backs                                                          //
//----------------------------------------------------------------------//

/*!
 *	Callback definition called each time a new log is received.
 *	\param[in]	pHandle									Valid handle on the sbgECom instance that has called this callback.
 *	\param[in]	logCmd									Contains the binary received log command id.
 *	\param[in]	pLogData								Contains the received log data as an union.
 *	\param[in]	pUserArg								Optional user supplied argument.
 *	\return												SBG_NO_ERROR if the received log has been used successfully.
 */
SbgErrorCode onLogReceived(SbgEComHandle *pHandle, SbgEComCmdId logCmd, const SbgBinaryLogData *pLogData, void *pUserArg)
{
	//
	// Handle separately each received data according to the log ID
	//

	// wait for comm channel to stabilise
	static int ignore64 = 0;
	if (ignore64++ < 64)
		return SBG_NO_ERROR;

	static uint32 prev_timestamp = 0;
	static uint32 carry32 = 0;
	uint32 timestamp_ring = 0;

	switch (logCmd)
	{
	case SBG_ECOM_LOG_STATUS:
		timestamp_ring = pLogData->statusData.timeStamp;
	case SBG_ECOM_LOG_IMU_DATA:	
		timestamp_ring = pLogData->imuData.timeStamp;
		break;
	case SBG_ECOM_LOG_MAG:
		timestamp_ring = pLogData->magData.timeStamp;
		break;
	case SBG_ECOM_LOG_PRESSURE:
		timestamp_ring = pLogData->pressureData.timeStamp;
		break;
	default:
		printf("FAIL TO GET TIMESTAMP:%d\n" , logCmd);
		break;
	}

	if (_ros_time_first_frame == ros::Time(0)) // this is the first time reiceving data, set ros time reference here
	{
		_ros_time_first_frame = ros::Time::now();
		_imu_time_first_frame = timestamp_ring;
		assert(prev_timestamp == 0); // make sure this is the first ever frame
	}

	if (prev_timestamp > timestamp_ring + 1e10)
	{
		printf("timestamp: CARRY 1 bit at bit 32\n");
		printf("%d %d\n",prev_timestamp,timestamp_ring);
		carry32++;
	}
	prev_timestamp = timestamp_ring;

	// getting the data timestamp in imu system
	unsigned long long imu_time = (unsigned long long)carry32*((unsigned long long)1<<32) + (unsigned long long)timestamp_ring;
	unsigned long long imu_time_duration = imu_time - _imu_time_first_frame;
	// getting the data timestamp in ros system
	auto ros_data_time = _ros_time_first_frame + ros::Duration(imu_time_duration/1000000,imu_time_duration%1000000*1000);
	
	//printf("timestamp_ring: %d imu_time: %llu\n",timestamp_ring,imu_time);

	switch (logCmd)
	{
	case SBG_ECOM_LOG_IMU_DATA:
		_imu_msg.header.stamp = ros_data_time;
		_imu_msg.linear_acceleration.x = pLogData->imuData.accelerometers[0];
		_imu_msg.linear_acceleration.y = pLogData->imuData.accelerometers[1];
		_imu_msg.linear_acceleration.z = pLogData->imuData.accelerometers[2];
		_imu_msg.angular_velocity.x = pLogData->imuData.gyroscopes[0];
		_imu_msg.angular_velocity.y = pLogData->imuData.gyroscopes[1]; 
		_imu_msg.angular_velocity.z = pLogData->imuData.gyroscopes[2]; 
		_imu_pub.publish(_imu_msg);
		break;
	case SBG_ECOM_LOG_MAG:
		_mag_msg.header.stamp = ros_data_time;
		_mag_msg.magnetic_field.x = pLogData->magData.magnetometers[0];
		_mag_msg.magnetic_field.y = pLogData->magData.magnetometers[1];
		_mag_msg.magnetic_field.z = pLogData->magData.magnetometers[2];
		_mag_pub.publish(_mag_msg);
		break;
	case SBG_ECOM_LOG_PRESSURE:
		_alt_msg.header.stamp = ros_data_time;
		_alt_msg.fluid_pressure = pLogData->pressureData.height;
		_alt_pub.publish(_alt_msg);
		break;
	case SBG_ECOM_LOG_STATUS:
		printf("PPS Status Received: %u\n", pLogData->statusData.timeStamp);
		break;
	default:
		printf("data type:%d\n" , logCmd);
		break;
	}
	
	return SBG_NO_ERROR;
}

//----------------------------------------------------------------------//
//  Main program                                                        //
//----------------------------------------------------------------------//

/*!
 *	Main entry point.
 *	\param[in]	argc		Number of input arguments.
 *	\param[in]	argv		Input arguments as an array of strings.
 *	\return					0 if no error and -1 in case of error.
 */
int main(int argc, char** argv)
{
	SbgEComHandle			comHandle;
	SbgErrorCode			errorCode;
	SbgInterface			sbgInterface;
	int32					retValue = 0;
	SbgEComDeviceInfo		deviceInfo;

	ros::init(argc, argv, "ellipseimu_ros");
	ros::NodeHandle nh("~");

	_imu_pub = nh.advertise<sensor_msgs::Imu>("imu0",5);
	_mag_pub = nh.advertise<sensor_msgs::MagneticField>("mag0",5);
	_alt_pub = nh.advertise<sensor_msgs::FluidPressure>("alt0",5);
	_ros_time_first_frame = ros::Time(0);
	//
	// Create an interface: 
	// We can choose either a serial for real time operation, or file for previously logged data parsing
	// Note interface closing is also differentiated !
	//
	errorCode = sbgInterfaceSerialCreate(&sbgInterface, "/dev/ttyUSB0", 921600);		// Example for Unix using a FTDI Usb2Uart converter
	//errorCode = sbgInterfaceSerialCreate(&sbgInterface, "COM20", 115200);								// Example for Windows serial communication

	//
	// Test that the interface has been created
	//
	if (errorCode == SBG_NO_ERROR)
	{
		//
		// Create the sbgECom library and associate it with the created interfaces
		//
		errorCode = sbgEComInit(&comHandle, &sbgInterface);

		//
		// Test that the sbgECom has been initialized
		//
		if (errorCode == SBG_NO_ERROR)
		{
			//
			// Get device inforamtions
			//
			errorCode = sbgEComCmdGetInfo(&comHandle, &deviceInfo);

			//
			// Display device information if no error
			//
			if (errorCode == SBG_NO_ERROR)
			{
				printf("Device : %09u found\n", deviceInfo.serialNumber);
			}
			else
			{
				fprintf(stderr, "ellipseMinimal: Unable to get device information.\n");
			}

			//
			// Configure some output logs to 25 Hz
			//
			if (sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_IMU_DATA, SBG_ECOM_OUTPUT_MODE_MAIN_LOOP) != SBG_NO_ERROR)
			{
				fprintf(stderr, "ellipseMinimal: Unable to configure output log SBG_ECOM_LOG_IMU_DATA.\n");
			}

			if (sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_STATUS, SBG_ECOM_OUTPUT_MODE_PPS) != SBG_NO_ERROR)
			{
				fprintf(stderr, "ellipseMinimal: Unable to configure output log SBG_ECOM_LOG_STATUS.\n");
			}

			// Disable a few outputs
			if (sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_QUAT, SBG_ECOM_OUTPUT_MODE_DISABLED) != SBG_NO_ERROR)
			{
				fprintf(stderr, "ellipseMinimal: Unable to configure output log SBG_ECOM_LOG_EKF_QUAT.\n");
			}
			if (sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_EULER, SBG_ECOM_OUTPUT_MODE_DISABLED) != SBG_NO_ERROR)
			{
				fprintf(stderr, "ellipseMinimal: Unable to configure output log SBG_ECOM_LOG_EKF_EULER.\n");
			}
			if (sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_EKF_NAV, SBG_ECOM_OUTPUT_MODE_DISABLED) != SBG_NO_ERROR)
			{
				fprintf(stderr, "ellipseMinimal: Unable to configure output log SBG_ECOM_LOG_EKF_NAV.\n");
			}
			if (sbgEComCmdOutputSetConf(&comHandle, SBG_ECOM_OUTPUT_PORT_A, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_GPS1_POS, SBG_ECOM_OUTPUT_MODE_DISABLED) != SBG_NO_ERROR)
			{
				fprintf(stderr, "ellipseMinimal: Unable to configure output log SBG_ECOM_LOG_GPS1_POS.\n");
			}
			

			//
			// Display a message for real time data display
			//
			//printf("sbgECom properly Initialized.\n\nEuler Angles display with estimated standard deviation.\n");

			//
			// Define callbacks for received data
			//
			sbgEComSetReceiveCallback(&comHandle, onLogReceived, NULL);

			//
			// Loop until the user exist
			//
			printf("ros spinning\n");
			while (ros::ok())
			{
				//
				// Try to read a frame
				//
				errorCode = sbgEComHandle(&comHandle);

				//
				// Test if we have to release some CPU (no frame received)
				//
				if (errorCode != SBG_NOT_READY)
				{
					fprintf(stderr, "Error\n");
				}
			}

			//
			// Close the sbgEcom library
			//
			sbgEComClose(&comHandle);
		}
		else
		{
			//
			// Unable to initialize the sbgECom
			//
			fprintf(stderr, "ellipseMinimal: Unable to initialize the sbgECom library.\n");
			retValue = -1;
		}
		
		//
		// Close the interface
		//
		sbgInterfaceSerialDestroy(&sbgInterface);
		//sbgInterfaceFileClose(&sbgInterface);
		
	}
	else
	{
		//
		// Unable to create the interface
		//
		fprintf(stderr, "ellipseMinimal: Unable to create the interface.\n");
		retValue = -1;
	}

	//
	// Returns -1 if we have an error
	//
	printf("main() exits\n");
	return retValue;
}
