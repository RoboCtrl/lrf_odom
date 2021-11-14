#pragma once

// c/c++ headers
#include <string>
#include <vector>
#include <sys/time.h>
#include <stdlib.h>


// ROS headers
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

// Eigen
#include <Eigen/Geometry>

// scan matcher headers
//#include <csm/csm_all.h>

// project headers
#include "lrf_odom/sensor.h"



namespace lcas {



// ros message pointer for /odom topics
typedef const nav_msgs::Odometry::ConstPtr OdomMsgConstPtr;



class Sensor {
	public:
		std::vector<Keyframe2d> keyframes_;
};


/** This class handles a single laser scan topic. it subscribes to the provided topic and handles the callbacks. */
class OdomSensor : public Sensor {
	public:
	
		OdomSensor( const string& topic_name ) : topic_name_(topic_name) {
			buffer_size_ = 3;
			last_timestamp_ = 0.;
			subscribe();
		}
		
		/** subscribes to the odometry topic, registering our callback method */
		void subscribe() {
			ROS_INFO_STREAM( "subscribing to topic '" << topic_name_ << "'" );
			subscriber_ = node_handle_.subscribe( topic_name_, buffer_size_, &OdomSensor::callback, this );
			if( !subscriber_ ) {
				ROS_INFO_STREAM( "failed to subscribe to topic '"  << topic_name_ << "'" );
			}
		}
		
		/** takes an odometry message and converts it into a keyframe */
		Keyframe2d* msgToKeyframe( OdomMsgConstPtr msg ) {
			auto position = &(msg->pose.pose.position);
			auto orientation = &(msg->pose.pose.orientation);
			Eigen::Quaterniond quaternion( orientation->w, orientation->x, orientation->y, orientation->z );
			Keyframe2d* keyframe = new Keyframe2d(msg->header.stamp.toSec(), msg->header.frame_id);
			keyframe->x_ = position->x;
			keyframe->y_ = position->y;
			auto euler_angles = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
			keyframe->theta_ = euler_angles[2];
			//std::cout << "euler angles: " << euler_angles[0] << "; " << euler_angles[1] << "; " << euler_angles[2] << std::endl;
			return keyframe;
		}
		
		/** this is the callback function for the topic subscriber.  */
		void callback( OdomMsgConstPtr msg ) {
			double min_delay = 0.5; // we wait at least that many seconds since the last processed message before we accept another message
			ros::Time t = msg->header.stamp;
			double time_secs = t.toSec();
			if( time_secs - last_timestamp_ < min_delay ) {
				// messages that are too close are dropped
				return;
			}
			
			auto keyframe = msgToKeyframe( msg );
			last_timestamp_ = time_secs;
			if( keyframes_.size() == 0 ) {
//				keyframes_.push_back( scan_data );
				return;
			}
			delete keyframe;
		}
	

		const string& topic_name_;
		ros::NodeHandle node_handle_;
		double last_timestamp_;
		int buffer_size_;
		ros::Subscriber subscriber_;
		
};


}


