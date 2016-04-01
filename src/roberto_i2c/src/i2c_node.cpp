#include <boost/thread.hpp>
#include <signal.h>
#include <ros/xmlrpc_manager.h>
#include <vector>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
//#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "I2CBus.h"
#include "OLED.h"
#include "gpio.h"
#include "RTIMU.h"
#include "RTIMUSettings.h"
#include "RTFusionRTQF.h"


#define BUTTON_LEFT_PIN (1 << 0)
#define BUTTON_UP_PIN (1 << 1)
#define BUTTON_DOWN_PIN (1 << 2)
#define BUTTON_RIGHT_PIN (1 << 3)
#define SHUTDOWN_PIN (1 << 4)
#define IMU_INT_PIN (1 << 5)


#define G_2_MPSS 9.80665
#define uT_2_T 1000000

sig_atomic_t volatile newButtons = 0;

RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

OLED *displayR;
OLED *displayL;

boost::thread *t;

int interrupt_pin;
int GPIO_ADDR;

sig_atomic_t volatile shutdown = 0;
sig_atomic_t volatile g_request_shutdown = 0;

void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}


void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}


void interruptThread(){

	// Find inital values of all GPIOs, to cope with active high/low devices
	uint8_t initial, initial2;
	settings.I2CRead(GPIO_ADDR, 1, &initial, "Failed to read GPIO Expander");
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	settings.I2CRead(GPIO_ADDR, 1, &initial2, "Failed to read GPIO Expander");
	if(initial != initial2)
		settings.I2CRead(GPIO_ADDR, 1, &initial, "Failed to read GPIO Expander");
	
	// Setup interrupt pin
	GPIO::GPIOManager* gp = GPIO::GPIOManager::getInstance();
	gp->exportPin(interrupt_pin);
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	gp->setDirection(interrupt_pin, GPIO::INPUT);
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	gp->setEdge(interrupt_pin, GPIO::FALLING);
	boost::this_thread::sleep(boost::posix_time::milliseconds(500));

	uint8_t byte;
    while(true){
        try{
  			if(gp->waitForEdge(interrupt_pin, GPIO::FALLING, 500) > 0 && !shutdown){		// Blocking!
  				// Interrupt cant fire again until GPIO expander has been read
	  			settings.I2CRead(GPIO_ADDR, 1, &byte, "Failed to read GPIO Expander");

	  			// Only react if any pins are different from initial state
	  			if(byte != initial){
		  			if((byte & SHUTDOWN_PIN) != (initial & SHUTDOWN_PIN)){
		  				ROS_INFO("Shutdown was clicked!");
		  				shutdown = 1;
		  				g_request_shutdown = 1;
		  			}
		  			if((byte & BUTTON_LEFT_PIN) != (initial & BUTTON_LEFT_PIN)){
		  				ROS_INFO("LEFT was clicked!");
		  				newButtons = 1;
		  			}
		  			if((byte & BUTTON_UP_PIN) != (initial & BUTTON_UP_PIN)){
		  				ROS_INFO("UP was clicked!");
		  				newButtons = 1;
		  			}
		  			if((byte & BUTTON_DOWN_PIN) != (initial & BUTTON_DOWN_PIN)){
		  				ROS_INFO("DOWN was clicked!");
		  				newButtons = 1;
		  			}
		  			if((byte & BUTTON_RIGHT_PIN) != (initial & BUTTON_RIGHT_PIN)){
		  				ROS_INFO("RIGHT was clicked!");
		  				newButtons = 1;
		  			}
		  			if((byte & IMU_INT_PIN) != (initial & IMU_INT_PIN)){
		  				ROS_INFO("IMU interrupt triggered!");
		  			}
		  		}
		  	}
            boost::this_thread::interruption_point();
        }catch(boost::thread_interrupted&){
        	gp->clean();
			ROS_INFO("Cleaning GPIO & closing I2C!");

        	return;
        }
    }
}
void displayRCallback(const std_msgs::UInt8MultiArray::ConstPtr& array){
	uint8_t Arr[1024];
	int i = 0;
	for(std::vector<uint8_t>::const_iterator it = array->data.begin(); it != array->data.end(); ++it){
		Arr[i++] = *it;
	}
	displayR->setBuffer(Arr);
	displayR->display();
	return;
}

void displayLCallback(const std_msgs::UInt8MultiArray::ConstPtr& array){
	uint8_t Arr[1024];
	int i = 0;
	for(std::vector<uint8_t>::const_iterator it = array->data.begin(); it != array->data.end(); ++it){
		Arr[i++] = *it;
	}
	displayL->setBuffer(Arr);
	displayL->display();
	return;
}

int main(int argc, char **argv){
	// Set up ROS.
	ros::init(argc, argv, "i2c_node", ros::init_options::NoSigintHandler);
	signal(SIGINT, mySigIntHandler);
	ros::NodeHandle n;
	ros::NodeHandle private_nh_("~");

	ros::XMLRPCManager::instance()->unbind("shutdown");
  	ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);


	sensor_msgs::Imu imu_msg;

	//tf::TransformBroadcaster tf_broadcaster_;

	ros::Publisher magnetometer_pub_;
	ros::Publisher euler_pub_;

	std::string imu_frame_id_;

	double declination_radians_;


	int parm;
	private_nh_.param("i2c_bus", parm, 4);
	settings.m_I2CBus = parm;	// /dev/i2c-4
	
	private_nh_.param("interrupt_pin", interrupt_pin, 204);
	
	private_nh_.param("gpio_expander_address", GPIO_ADDR, 56);
	
	int DISPLAYR_ADDR;
	private_nh_.param("right_display_address", DISPLAYR_ADDR, 61);

	int DISPLAYL_ADDR;
	private_nh_.param("left_display_address", DISPLAYL_ADDR, 60);

	bool calibrateMode;
	private_nh_.param("calibrateMode", calibrateMode, false);

	settings.loadSettings(&private_nh_);

	// Initialize both OLED displays on I2C bus
	if(!calibrateMode){
		displayR = new OLED();
		if(displayR->init(&settings, DISPLAYR_ADDR, 128, 64)){
			displayR->begin();
			displayR->display();
		}else{
			ROS_WARN("Right OLED display not detected!");
		}

		displayL = new OLED();
		if(displayL->init(&settings, DISPLAYL_ADDR, 128, 64)){
			displayL->begin();
			displayL->display();
		}else{
			ROS_WARN("Left OLED display not detected!");
		}

		// Start interrupt thread
		t = new boost::thread(&interruptThread);
	}


	private_nh_.param<std::string>("frame_id", imu_frame_id_, "imu_link");

	ros::Publisher imu_pub_ = n.advertise<sensor_msgs::Imu>("imu/data",10);

	bool magnetometer;
	private_nh_.param("publish_magnetometer", magnetometer, true);
	if (magnetometer){
		magnetometer_pub_ = n.advertise<sensor_msgs::MagneticField>("imu/mag", 10, false);
	}

	bool euler;
	private_nh_.param("publish_euler", euler, false);
	if (euler){
		euler_pub_ = n.advertise<geometry_msgs::Vector3>("imu/euler", 10, false);
	}

	std::vector<double> orientation_covariance, angular_velocity_covariance, linear_acceleration_covariance;
	if (private_nh_.getParam("orientation_covariance", orientation_covariance) && orientation_covariance.size() == 9){
		for(int i=0; i<9; i++){
			imu_msg.orientation_covariance[i]=orientation_covariance[i];
		}
	}

	if (private_nh_.getParam("angular_velocity_covariance", angular_velocity_covariance) && angular_velocity_covariance.size() == 9){
		for(int i=0; i<9; i++){
			imu_msg.angular_velocity_covariance[i]=angular_velocity_covariance[i];
		}
	}

	if (private_nh_.getParam("linear_acceleration_covariance", linear_acceleration_covariance) && linear_acceleration_covariance.size() == 9){
		for(int i=0; i<9; i++){
			imu_msg.linear_acceleration_covariance[i]=linear_acceleration_covariance[i];
		}
	}


	private_nh_.param("magnetic_declination", declination_radians_, 0.0);

  	
	// Setup ROS topics - publishers and subscribers
	ros::Subscriber sub_oledR = n.subscribe("displayR", 10, displayRCallback);
	ros::Subscriber sub_oledL = n.subscribe("displayL", 10, displayLCallback);

	ros::Publisher pub_battery_stats = n.advertise<std_msgs::String>("battery_stats", 10);
	ros::Publisher pub_battery_diag = n.advertise<std_msgs::String>("battery_diag", 10);
	ros::Publisher pub_buttons = n.advertise<std_msgs::String>("buttons", 10);

	imu = RTIMU::createIMU(&settings);                        // create the imu object

	if (!imu->IMUInit()) {
    	ROS_ERROR("Failed to init IMU");
  	}

  	imu->setCalibrationMode(calibrateMode);


	// Tell ROS how fast to run this node.
  	// Run the main loop at twice the IMU rate
	ros::Rate r((int)ceil(2.0/(imu->IMUGetPollInterval()/1000.0)));
	while (!g_request_shutdown){
		// Read and publish IMU data
		if (imu->IMURead()) {                                // get the latest data if ready yet

			RTVector3 gyro = imu->getGyro();
			RTVector3 accel = imu->getAccel();
			RTVector3 compass = imu->getCompass();


    		fusion.newIMUData(gyro, accel, compass, imu->getTimestamp());

    		RTQuaternion fusionQPose = fusion.getFusionQPose();
    		ros::Time current_time = ros::Time::now();

    		// Units and Axis sorted to comply with REP-103
    		// The header frame ID should comply with REP-105

			imu_msg.header.stamp = current_time;
			imu_msg.header.frame_id = imu_frame_id_;
			imu_msg.orientation.x = -fusionQPose.y();
			imu_msg.orientation.y = -fusionQPose.x();
			imu_msg.orientation.z = -fusionQPose.z();
			imu_msg.orientation.w = fusionQPose.scalar();

			imu_msg.angular_velocity.x = -gyro.y();
			imu_msg.angular_velocity.y = -gyro.x();
			imu_msg.angular_velocity.z = -gyro.z();

			if(calibrateMode){
				imu_msg.linear_acceleration.x = -accel.y();
				imu_msg.linear_acceleration.y = -accel.x();
				imu_msg.linear_acceleration.z = -accel.z();
			}else{
				imu_msg.linear_acceleration.x = -accel.y() * G_2_MPSS;
				imu_msg.linear_acceleration.y = -accel.x() * G_2_MPSS;
				imu_msg.linear_acceleration.z = -accel.z() * G_2_MPSS;
			}
			imu_pub_.publish(imu_msg);

			if (magnetometer_pub_ != NULL && imu->getCompassCalValid())
			{
				sensor_msgs::MagneticField msg;

				msg.header.frame_id=imu_frame_id_;
				msg.header.stamp=ros::Time::now();

				if(calibrateMode){
					msg.magnetic_field.x = compass.x();
					msg.magnetic_field.y = compass.y();
					msg.magnetic_field.z = compass.z();
				}else{
					msg.magnetic_field.x = compass.x()/uT_2_T;
					msg.magnetic_field.y = compass.y()/uT_2_T;
					msg.magnetic_field.z = compass.z()/uT_2_T;
				}

				magnetometer_pub_.publish(msg);
			}

			if (euler_pub_ != NULL)
			{
				RTVector3 fusionPose = fusion.getFusionPose();
				geometry_msgs::Vector3 msg;
				msg.x = fusionPose.x();
				msg.y = fusionPose.y();
				msg.z = -fusionPose.z();
				msg.z = (-fusionPose.z()) - declination_radians_;
				euler_pub_.publish(msg);
			}
		}

		if(!calibrateMode){
			if(pub_battery_stats.getNumSubscribers() > 0){
				// Read and publish Battery stats
			}
			if(pub_battery_diag.getNumSubscribers() > 0){
				// Read and publish Battery diagnostics
			}

			if(newButtons){
				// publish Button state
				std_msgs::String test;
				test.data = "here";
				pub_buttons.publish(test);
	  			newButtons = 0;
			}

			if(shutdown){
				g_request_shutdown = 1;
				break;
			}
		}
		ros::spinOnce();
		r.sleep();

	}

	// Clean up
	if(!calibrateMode){
		displayR->close();
		displayL->close();

		t->interrupt();
		t->join();
	}
	settings.I2CClose();
	
	// Sleep for half a second to allow threads to join in
	ros::Duration(0,500000000).sleep();

	ROS_ERROR("Shutdown!");

	ros::shutdown();

	/*if(shutdown){
		system("shutdown -P now");
	}*/

	return 0;
}