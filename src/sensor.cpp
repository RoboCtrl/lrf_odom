

#include "lrf_odom/sensor.h"

namespace lcas {

/** sets the default parameters for csm */
void setParams( sm_params &params) {
	params.first_guess[0] = 0.0;
	params.first_guess[1] = 0.0;
	params.first_guess[2] = 0.0;
	params.max_angular_correction_deg = 45.0;
	params.max_linear_correction = 0.50;
	params.max_iterations = 10;
	params.epsilon_xy = 0.000001;
	params.epsilon_theta = 0.000001;
	params.max_correspondence_dist = 0.3;
	params.sigma = 0.010;
	params.use_corr_tricks = 1;
	params.restart = 0;
	params.restart_threshold_mean_error = 0.01;
	params.restart_dt = 1.0;
	params.restart_dtheta = 0.1;
	params.clustering_threshold = 0.25;
	params.orientation_neighbourhood = 20;
	params.use_point_to_line_distance = 1;
	params.do_alpha_test = 0;
	params.do_alpha_test_thresholdDeg = 20.0;
	params.outliers_maxPerc = 0.90;
	params.outliers_adaptive_order = 0.7;
	params.outliers_adaptive_mult = 2.0;
	params.do_visibility_test = 0;
	params.outliers_remove_doubles = 1;
	params.do_compute_covariance = 0;
	params.debug_verify_tricks = 0;
	params.use_ml_weights = 0;
	params.use_sigma_weights = 0;
	params.laser[0] = 0.0;
	params.laser[1] = 0.0;
	params.laser[2] = 0.0;
	params.min_reading = 0.01;
	params.max_reading = 50.0;
}


/** computes the time difference between two timespec structs */
void timespec_diff(struct timespec &start, struct timespec &stop, struct timespec &result) {
	if( (stop.tv_nsec - start.tv_nsec) < 0 ) {
		result.tv_sec = stop.tv_sec - start.tv_sec - 1;
		result.tv_nsec = stop.tv_nsec - start.tv_nsec + 1000000000;
	} else {
		result.tv_sec = stop.tv_sec - start.tv_sec;
		result.tv_nsec = stop.tv_nsec - start.tv_nsec;
	}
}



ScanSensor::ScanSensor( string& scan_topic_name, string& odom_topic_name ) :
	scan_topic_name_(scan_topic_name), odom_topic_name_(odom_topic_name) {
	buffer_size_ = 1;
	reset();
	publisher_ = node_handle_.advertise<nav_msgs::Odometry>( odom_topic_name_, 50 );
	subscribe();
}

ScanSensor::ScanSensor() {
	buffer_size_ = 1;
	reset();
	getROSParams();
	publisher_ = node_handle_.advertise<nav_msgs::Odometry>( odom_topic_name_, 50 );
	subscribe();
}


void ScanSensor::callback( ScanMsgConstPtr msg ) {
	static unsigned int counter = 0;
	LDP scan_data = scanToLDP( msg );
	sm_result result;
	
	if( !last_key_scan_ ) {
		std::cout << "setting first keyframe for sensor '" << scan_topic_name_ << "'" << std::endl;
		last_key_scan_ = scan_data;
		last_x_ = 0.;
		last_y_ = 0.;
		last_theta_ = 0.;
		return;
	}
	
	result = scan_match( scan_data );
	
	//if( counter % 10 == 0 )
	//	printSMInfo( result, scan_data );
	publish( result );
	checkForKeyframe( result, scan_data );
	counter++;
}



}
