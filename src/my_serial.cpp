#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <motor_msg/motor.h>
#include <string.h>

serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO_STREAM("Writing to serial port" << msg->data);
	ser.write(msg->data);
}


int get_value(const char *data)
{
	int value = 0;
	
	while((*data >= '0' && *data <= '9') || *data == '\n')
		value = value * 10 + *data++ - '0';
	
	return value;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "serial_port");
	ros::NodeHandle n;
	
	
	ros::Subscriber write_sub = n.subscribe("write", 1000, write_callback);
	ros::Publisher  read_pub = n.advertise<std_msgs::String>("read", 1000);
	ros::Publisher  plot_pub = n.advertise<motor_msg::motor>("motor", 1000);
	
	try {
		ser.setPort("/dev/ttyUSB0");
		ser.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	}
	
	catch (serial::IOException& e) {
		ROS_ERROR_STREAM("Unable to open port");
		return -1;
	}
	
	if (ser.isOpen()) {
		ROS_INFO_STREAM("Serial Port initialized");
	} else {
		return -1;	
	}
	
	ros::Rate loop_rate(50);
	while(ros::ok()) {
		if (ser.available()) {
	//		ROS_INFO_STREAM("Reading from serial port\n");
			std_msgs::String result;
		        motor_msg::motor moto;

			result.data = ser.read(ser.available());
			const char *p = result.data.c_str();	
			const char *index1 = strstr(p, "#");	
			const char *index2 = strstr(p, "$");
			if (index1 != NULL) {
				index1++;
				moto.leftRatio = get_value(index1);
				if (moto.leftRatio == 0)
					ROS_INFO("left:%d", moto.leftRatio);
			}
			if (index2 != NULL) {
				index2++;
                                moto.rightRatio = 0;
                                moto.rightRatio = get_value(index2);
				if (moto.rightRatio == 0)
					ROS_INFO("right:%d:", moto.rightRatio);
			}
			
			read_pub.publish(result);
			
			plot_pub.publish(moto);
		}
		
		ros::spinOnce();
		loop_rate.sleep();
	}
}
