#pragma once

// c/c++ headers
#include <string>
#include <vector>
#include <sys/time.h>


// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

// scan matcher headers
#include <csm/csm_all.h>

//#include "lrf_odom/keyframe.h"


namespace lcas {


using std::string;
//using std::vector;

// ros message pointer for /scan topics
typedef const sensor_msgs::LaserScan::ConstPtr ScanMsgConstPtr;


void setParams( sm_params &params );
void timespec_diff( struct timespec &start, struct timespec &stop, struct timespec &result );


const double pi = 3.14159265358979323846;
const double twoPi = 2.0 * pi;
const double negPi = -1.0 * pi;



/** This class contains a 2D pose and timestamp, representing a keyframe for a single sensor. */
class Keyframe2d {
	public:
		Keyframe2d( double timestamp, std::string frame_id) : timestamp_(timestamp), frame_(frame_id), x_(0.0), y_(0.0), theta_(0.0) {}
		
		const double timestamp_;	// seconds
		const std::string frame_;	// frame id
		float x_;
		float y_;
		float theta_;
};




/** This class handles a single laser scan topic. it subscribes to the provided topic and handles the callbacks. */
class ScanSensor {
	public:
	
		ScanSensor( string& scan_topic_name, string& odom_topic_name );
		
		ScanSensor();
		
		void reset() {
			last_key_scan_ = NULL;
			last_x_ = 0.;
			last_y_ = 0.;
			last_theta_ = 0.;
			odom_x_ = 0.;
			odom_y_ = 0.;
			odom_theta_ = 0.;
		}
		
		void getROSParams() {
			ros::NodeHandle nh("~");
			nh.param<std::string>( "scan_topic", scan_topic_name_, "/scan" );		// laser4 scan input topic name
			nh.param<std::string>( "odom_topic", odom_topic_name_, "sensor_odom" );	// output odom topic name
			nh.param<std::string>( "odom_frame", odom_frame_id_, "sensor_odom" );	// tf id
			nh.param<std::string>( "odom_child_frame", odom_child_frame_id_, "base_link" );	// tf id (child frame)
			
			std::cout << "node parameters:" << std::endl <<
				"    scan_topic='" << scan_topic_name_ << "'" << std::endl <<
				"    odom_topic='" << odom_topic_name_ << "'" << std::endl <<
				"    odom_frame='" << odom_frame_id_ << "'" << std::endl <<
				"    odom_child_frame='" << odom_child_frame_id_ << "'" << std::endl;
		}
		
		void subscribe() {
			std::cout << "subscribing to topic '" << this->scan_topic_name_ << "'" << std::endl;
			subscriber_ = node_handle_.subscribe( scan_topic_name_, buffer_size_, &ScanSensor::callback, this );
		}
		
		void callback( ScanMsgConstPtr msg );
		
		void publish( sm_result &result ) {
			geometry_msgs::TransformStamped odom_trans;
			double dx, dy, dtheta;
			ros::Time time_now = ros::Time::now();
			double time_delta = (time_now - last_timestamp_).toSec();
			if( time_delta == 0.0 )
				time_delta = 0.01;	// provide a sane value so that computations don't break
			
			dx = (result.x[0]) * cos( odom_theta_ ) + (result.x[1]) * -1.0 * sin( odom_theta_ );
			dy = (result.x[0]) * sin( odom_theta_ ) + (result.x[1]) * cos( odom_theta_ );
			dtheta = result.x[2];
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw( odom_theta_ + dtheta );
			odom_trans.header.stamp = time_now;
			odom_trans.header.frame_id = odom_frame_id_;
			odom_trans.child_frame_id = odom_child_frame_id_;
			odom_trans.transform.translation.x = odom_x_ + dx;
			odom_trans.transform.translation.y = odom_y_ + dy;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;
			odom_broadcaster_.sendTransform( odom_trans );
			/**/
			nav_msgs::Odometry odom;
			odom.header.stamp = time_now;
			odom.header.frame_id = odom_frame_id_;
			// position
		    odom.pose.pose.position.x = odom_x_ + dx;
			odom.pose.pose.position.y = odom_y_ + dy;
			odom.pose.pose.position.z = 0.0;
			odom.pose.pose.orientation = odom_quat;
			odom.child_frame_id = odom_child_frame_id_;
			// velocity
			odom.twist.twist.linear.x = dx / time_delta;
			odom.twist.twist.linear.y = dy / time_delta;
			odom.twist.twist.angular.z = dtheta / time_delta;
			publisher_.publish( odom );
			last_timestamp_ = time_now;
			last_x_ = result.x[0];
			last_y_ = result.x[1];
			last_theta_ = result.x[2];
		}
		
		/** takes a sensor_msgs::LaserScan message and converts it to a laser_data structure. returns a pointer to the laser_data structure when finsihed */
		LDP scanToLDP( ScanMsgConstPtr& scan ) {
			unsigned int num_rays = scan->ranges.size();
			LDP ldp = ld_alloc_new( num_rays );
			int num_valid = 0;
			int num_invalid = 0;
			
			//std::cout << "scan->ranges.size()=" << scan->ranges.size() << ",  ldp->nrays=" << ldp->nrays << std::endl;

			for( unsigned int i = 0; i < num_rays; i++ ) {
				// calculate position in laser frame
				double r = scan->ranges[i];
				// not sure if > and < or >= and <= ... the former probably is saver
				if( (r > scan->range_min) && (r < scan->range_max) ) {
					// fill in laser scan data
					ldp->valid[i] = 1;
					ldp->readings[i] = r;
					num_valid++;
				} else {
					ldp->valid[i] = 0;
					ldp->readings[i] = -1;  // for invalid range
					num_invalid++;
				}
				ldp->theta[i] = scan->angle_min + i * scan->angle_increment;
				ldp->cluster[i] = -1;
			}
			//ROS_INFO_STREAM( "scanToLDP: " << num_valid << "/" << num_invalid );

			ldp->min_theta = ldp->theta[0];
			ldp->max_theta = ldp->theta[num_rays - 1];

			ldp->odometry[0] = 0.0;
			ldp->odometry[1] = 0.0;
			ldp->odometry[2] = 0.0;

			ldp->true_pose[0] = 0.0;
			ldp->true_pose[1] = 0.0;
			ldp->true_pose[2] = 0.0;

			ldp->estimate[0] = 0.0;
			ldp->estimate[1] = 0.0;
			ldp->estimate[2] = 0.0;
			
			//print_laser_data( ldp, "scanToLDP " );
			return ldp;
		}
		
		/** simply prints some info anout the scan match result to the terminal */
		void printSMInfo( sm_result &result, LDP scan_data ) {
			double x = result.x[0];
			double y = result.x[1];
			double theta = result.x[2];
			double dist = sqrt( x*x + y*y );
			double error = result.error;
			int n_valid = result.nvalid;
			int n_total = scan_data->nrays;
			std::cout << "result: x=" << x << ", y=" << y << ", theta=" << theta << ", dist=" << dist << ", err=" << error << " (" << n_valid << "/" << n_total << ")" << std::endl;
		}
		
		/** sets the keyframe to the provided scan. frees memory, and resets the last pose */
		void setKeyframe( LDP new_keyframe ) {
			std::cout << "new keyframe, old odom=[" << odom_x_ << "," << odom_y_ << "," << odom_theta_ << "], new odom=[" << odom_x_ + last_x_ << "," << odom_y_ + last_y_ << "," << odom_theta_ + last_theta_ << "," << "]" << std::endl;
			if( last_key_scan_)
				ld_free( last_key_scan_ );
			last_key_scan_ = new_keyframe;
			odom_x_ += last_x_ * cos( odom_theta_ ) + last_y_ * -1.0 * sin( odom_theta_ );
			odom_y_ += last_x_ * sin( odom_theta_ ) + last_y_ * cos( odom_theta_ );
			odom_theta_ += last_theta_;
			if( odom_theta_ > pi )
				odom_theta_ -= twoPi;
			if( odom_theta_ < negPi )
				odom_theta_ += twoPi;
			last_x_ = 0.;
			last_y_ = 0.;
			last_theta_ = 0.;
		}
		
		/** checks if we need to add refresh the keyframe, and either frees the old keyframe or scan_data */
		void checkForKeyframe( sm_result &result, LDP scan_data ) {
			double x = result.x[0];
			double y = result.x[1];
			double theta = result.x[2];
			double dist = sqrt( x*x + y*y );
			double error = result.error;
			int n_valid = result.nvalid;
			int n_total = scan_data->nrays;
			
			if( !result.valid ) {
				std::cout << "scan matching error" << std::endl;
				setKeyframe( scan_data );
				return;
			}
			
			if( dist >= 0.5 || abs(theta) >= 0.3 ) {
				setKeyframe( scan_data );
				return;
			}
			ld_free( scan_data );
		}
		
		/** runs the scan matcher and returns the result object */
		sm_result scan_match( LDP &new_scan ) {
			timespec time_start, time_end, time_diff;
			clock_gettime( CLOCK_REALTIME, &time_start );
			
			sm_params params;
			sm_result result;
			setParams( params );
			params.laser_ref = last_key_scan_;
			params.laser_sens = new_scan;
			params.first_guess[0] = last_x_;
			params.first_guess[1] = last_y_;
			params.first_guess[2] = last_theta_;
			params.min_reading = 0.1;
			params.max_reading = 30.0;
			
			sm_icp(&params, &result);
			double* x = &result.x[0];
//			ROS_INFO_STREAM( "sensor motion: " << " x=" << x[0] << " y=" << x[1] << " theta=" << x[2] );

			clock_gettime( CLOCK_REALTIME, &time_end );
			timespec_diff( time_start, time_end, time_diff );
//			ROS_INFO( "time spent on icp: %ld.%09lds", time_diff.tv_sec, time_diff.tv_nsec );
			return result;
		}
		
		
		string scan_topic_name_;			// name of the input topic (laser scan messages)
		string odom_topic_name_;			// name of the output topic (odometry messages)
		string odom_frame_id_;				// frame id of our odometry estimate, as found in the odom message's header.frame_id field
		string odom_child_frame_id_;		// child id, as found in the odom message's child_frame_id field
		ros::NodeHandle node_handle_;
		int buffer_size_;				// size of buffer for incoming laser scan massages
		LDP last_key_scan_;				// laser data object of the last keyframe
		sm_params params_;				// parameters used for icp
		ros::Subscriber subscriber_;			// subscriber handle
		ros::Publisher publisher_;			// publisher handle
		tf::TransformBroadcaster	odom_broadcaster_;
		double last_x_;					// coordinates of the last processed message, relative to the keyframe
		double last_y_;
		double last_theta_;
		double odom_x_;					// coordinates of our odometry estimate, as published
		double odom_y_;
		double odom_theta_;
		ros::Time last_timestamp_;
		
};


} // namespace lcas


