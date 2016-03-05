#include <boost/thread.hpp>
#include <signal.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include "sensor_msgs/Imu.h"
#include "I2CBus.h"
#include "OLED.h"
#include "gpio.h"
#include "RTIMU.h"
#include "RTIMUSettings.h"
#include "RTFusionRTQF.h"


#define SHUTDOWN_PIN 0x10
#define BUTTON_LEFT_PIN 0x01
#define BUTTON_UP_PIN 0x02
#define BUTTON_DOWN_PIN 0x04
#define BUTTON_RIGHT_PIN 0x08

bool newButtons = false;
bool shutdown = false;

// Default value, to be overriden in case of parameter argument
uint8_t GPIO_ADDR = 0x38;
uint8_t DISPLAYR_ADDR = 0x3D;
uint8_t DISPLAYL_ADDR = 0x3C;
uint8_t interrupt_pin = 204;

RTIMU *imu;                                           // the IMU object
RTFusionRTQF fusion;                                  // the fusion object
RTIMUSettings settings;                               // the settings object

OLED *displayR;
OLED *displayL;



void interruptThread(){
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
  			gp->waitForEdge(interrupt_pin, GPIO::FALLING);		// Blocking!
  			settings.I2CRead(GPIO_ADDR, 1, &byte, "Failed to read GPIO Expander");
  			if(!(byte & SHUTDOWN_PIN)){
  				ROS_INFO("Shutdown was clicked!");
  				//shutdown = true;
  			}
  			if(!(byte & BUTTON_LEFT_PIN)){
  				ROS_INFO("LEFT was clicked!");
  				newButtons = true;
  			}
  			if(!(byte & BUTTON_UP_PIN)){
  				ROS_INFO("UP was clicked!");
  				newButtons = true;
  			}
  			if(!(byte & BUTTON_DOWN_PIN)){
  				ROS_INFO("DOWN was clicked!");
  				newButtons = true;
  			}
  			if(!(byte & BUTTON_RIGHT_PIN)){
  				ROS_INFO("RIGHT was clicked!");
  				newButtons = true;
  			}
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        }catch(boost::thread_interrupted&){
        	gp->clean();
			ROS_INFO("Cleaning GPIO!");

            return;
        }
    }
}

void displayRCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg){
	displayR->setBuffer((uint8_t*)&msg->data);
	displayR->display();
}

void displayLCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg){
	displayL->setBuffer((uint8_t*)&msg->data);
	displayL->display();
}

int main(int argc, char **argv){
	// Set up ROS.
	ros::init(argc, argv, "i2c_node");
	ros::NodeHandle n;

	// Parameters - 	TODO: attach parameter server
	
	interrupt_pin = 204;
	GPIO_ADDR = 0x38;
	DISPLAYR_ADDR = 0x3D;
	DISPLAYL_ADDR = 0x3C;

	settings.m_I2CBus = '4';	// /dev/i2c-4

	// Initialize both OLED displays on I2C bus
	displayR->init(&settings, DISPLAYR_ADDR, 128, 64);
	displayR->begin();
	displayR->display();

	displayL->init(&settings, DISPLAYL_ADDR, 128, 64);
	displayL->begin();
	displayL->display();

	// Start interrupt thread
	boost::thread t(&interruptThread);
  	
	// Setup ROS topics - publishers and subscribers
	ros::Subscriber sub_oledR = n.subscribe("displayR", 100, displayRCallback);
	ros::Subscriber sub_oledL = n.subscribe("displayL", 100, displayLCallback);
	ros::Publisher pub_battery_stats = n.advertise<std_msgs::String>("battery_stats", 10);
	ros::Publisher pub_battery_diag = n.advertise<std_msgs::String>("battery_diag", 10);
	ros::Publisher imu_pub_ = n.advertise<sensor_msgs::Imu>("imu", 100);
	ros::Publisher pub_buttons = n.advertise<std_msgs::String>("buttons", 10);


	imu = RTIMU::createIMU(&settings);                        // create the imu object

	if (imu->IMUInit() < 0) {
    	ROS_ERROR("Failed to init IMU");
  	}
  
  	if (imu->getCalibrationValid())
    	ROS_WARN("Using compass calibration");
  	else
    	ROS_WARN("No valid compass calibration data");


	// Tell ROS how fast to run this node.
	ros::Rate r(100);

	sensor_msgs::Imu imu_msg;
	while (ros::ok()){
		// Read and publish IMU data
		if (imu->IMURead()) {                                // get the latest data if ready yet

			RTVector3 gyro = imu->getGyro();
			RTVector3 accel = imu->getAccel();
			RTVector3 compas = imu->getCompass();


    		fusion.newIMUData(gyro, accel, compas, imu->getTimestamp());

    		RTQuaternion fusionQPose = fusion.getFusionQPose();
    		ros::Time current_time = ros::Time::now();


			imu_msg.header.stamp = current_time;
			//imu_msg.header.frame_id = imu_frame_id_;
			imu_msg.orientation.x = fusionQPose.x();
			imu_msg.orientation.y = fusionQPose.y();
			imu_msg.orientation.z = fusionQPose.z();
			imu_msg.orientation.w = fusionQPose.scalar();

			imu_msg.angular_velocity.x = gyro.x();
			imu_msg.angular_velocity.y = gyro.y();
			imu_msg.angular_velocity.z = gyro.z();

			imu_msg.linear_acceleration.x = accel.x();// * G_2_MPSS;
			imu_msg.linear_acceleration.y = accel.y();// * G_2_MPSS;
			imu_msg.linear_acceleration.z = accel.z();// * G_2_MPSS;

			imu_pub_.publish(imu_msg);
		}


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
  			newButtons = false;
		}

		if(shutdown){
			ros::shutdown();
		}
		ros::spinOnce();
		r.sleep();
	}
	displayR->close();
	displayL->close();

	t.interrupt();
	t.join();


	ROS_INFO("Shutdown!");


	/*if(shutdown){
		system("shutdown -P now");
	}*/

	return 0;
}