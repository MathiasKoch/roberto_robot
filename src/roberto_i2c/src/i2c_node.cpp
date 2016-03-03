#include <boost/thread.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "I2CBus.h"
#include "gpio.h"



#define SHUTDOWN_PIN 0x01
#define BUTTON_LEFT_PIN 0x01
#define BUTTON_UP_PIN 0x01
#define BUTTON_DOWN_PIN 0x01
#define BUTTON_RIGHT_PIN 0x01

bool newButtons = false;
bool shutdown = false;
I2CBus* i2c;

// Default value, to be overriden in case of parameter argument
uint16_t GPIO_ADDR = 0x38;
uint16_t interrupt_pin = 204;


void interruptThread(){
	GPIO::GPIOManager* gp = GPIO::GPIOManager::getInstance();
	gp->exportPin(interrupt_pin);
	gp->setDirection(interrupt_pin, GPIO::INPUT);
	gp->setEdge(interrupt_pin, GPIO::FALLING);

	uint8_t byte;
    while(true){
        try{
  			gp->waitForEdge(interrupt_pin, GPIO::FALLING);		// Blocking!
  			byte = (uint8_t) i2c->tryReadByte(GPIO_ADDR, 0xFF);

  			if(byte & SHUTDOWN_PIN){
  				ROS_INFO("Shutdown was clicked!");
  				//shutdown = true;
  			}
  			if(byte & BUTTON_LEFT_PIN){
  				newButtons = true;
  			}
  			if(byte & BUTTON_UP_PIN){
  				newButtons = true;
  			}
  			if(byte & BUTTON_DOWN_PIN){
  				newButtons = true;
  			}
  			if(byte & BUTTON_RIGHT_PIN){
  				newButtons = true;
  			}
            boost::this_thread::sleep(boost::posix_time::milliseconds(500));
        }catch(boost::thread_interrupted&){
            return;
        }
    }
}

void displayCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){
	// Set up ROS.
	ros::init(argc, argv, "i2c_node");
	ros::NodeHandle n;

	// Parameters - 	TODO: attach parameter server
	std::string i2cDevice = "/dev/i2c-4";
	interrupt_pin = 204;
	GPIO_ADDR = 0x38;

	
	// Initialize I2C bus
	i2c = new I2CBus(i2cDevice.c_str());


	// Start interrupt thread
	boost::thread t(&interruptThread);
  	
	// Setup ROS topics - publishers and subscribers
	ros::Subscriber sub_oled = n.subscribe("display", 100, displayCallback);
	ros::Publisher pub_battery_stats = n.advertise<std_msgs::String>("battery_stats", 10);
	ros::Publisher pub_battery_diag = n.advertise<std_msgs::String>("battery_diag", 10);
	ros::Publisher pub_imu = n.advertise<std_msgs::String>("imu", 100);
	ros::Publisher pub_buttons = n.advertise<std_msgs::String>("buttons", 10);

	// Tell ROS how fast to run this node.
	ros::Rate r(100);

	int i = 0;
	while (n.ok()){
		// Read and publish IMU data

		if((i++ % 100) == 0){		// 1 Hz -	TODO: Make this the way it's supposed to be?
			if(pub_battery_stats.getNumSubscribers() > 0){
				// Read and publish Battery stats
			}
			if(pub_battery_diag.getNumSubscribers() > 0){
				// Read and publish Battery diagnostics
			}
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
	t.interrupt();
	t.join();

	if(shutdown){
		system("shutdown -P now");
	}

	return 0;
}