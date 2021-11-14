


#include <ros/console.h>
#include <sys/time.h>

#include "lrf_odom/sensor.h"
#include "lrf_odom/sensor_odom.h"



void test_callback( lcas::OdomMsgConstPtr msg ) {
	ROS_INFO_STREAM( "test callback" );
}

int main( int argc, char** argv )
{
	/*
	
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
	lcas::SensorEgoMotion sensor_ego_motion;
	sensor_ego_motion.spin();
	*/
	
	if( false ) {
		ros::init( argc, argv, "sensor_ex_calib_odom_tracker" );
		ROS_INFO_STREAM( "starting odom sensor tracking node" );
		const std::string topic("/odometry/gazebo");
		//const std::string topic("odom");
		//lcas::OdomSensor odom_sensor( topic );
		lcas::OdomSensor* odom_sensor = new lcas::OdomSensor( topic );
		ROS_INFO( "starting spinning" );
		if( true ) {
			ros::NodeHandle nh;
			auto sub = nh.subscribe( "odom", 2, test_callback );
			ros::spin();
		}
		ros::spin();
		ROS_INFO( "exiting" );
		delete odom_sensor;
	}
	
	if( true ) {
		ros::init( argc, argv, "sensor_ex_calib_lrf_motion" );
		ROS_INFO_STREAM( "starting scan sensor motion estimater" );
		std::string topic_in( "/scanner_front/scan" );
		std::string topic_out( "/scan_1/ego_motion" );
		//lcas::ScanSensor* scan_sensor = new lcas::ScanSensor( topic_in, topic_out );
		lcas::ScanSensor* scan_sensor = new lcas::ScanSensor();
		ROS_INFO( "starting spinning" );
		ros::spin();
		ROS_INFO( "exiting" );
		delete scan_sensor;
	}
	
	return 0;
}



