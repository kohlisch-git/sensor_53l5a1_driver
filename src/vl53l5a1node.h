/**
 * @file 	vl53l5a1node.h
 * @author 	Niklas Kohlisch kohlischni71082@th-nuernberg.de
 * @brief 	Header for rosserial sensor driver
 * @version 0.1
 * @date 	2021-11-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef VL53L5A1NODE_H
#define VL53L5A1NODE_H

#include <string>
#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <dynamic_reconfigure/server.h>
#include <sensors_53l5a1_driver/SensorConfig.h>
#include <sensors_53l5a1_driver/am_per_spad.h>
#include <sensors_53l5a1_driver/distance.h>
#include <sensors_53l5a1_driver/nb_spads.h>
#include <sensors_53l5a1_driver/reflectance.h>
#include <sensors_53l5a1_driver/sig_per_spad.h>
#include <sensors_53l5a1_driver/sigma.h>
#include <sensors_53l5a1_driver/status.h>
#include <sensors_53l5a1_driver/control.h>
#include <stdint.h>
#include <cmath>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>

const char node_name  	[]  = "VL53L5A1Node";
const char topic_control[]  = "/vl53l5_control_msg";
const char topic_distance[] = "/vl53l5_distance_mm";

/**
 * @brief Class node object
 * 
 */
class VL53L5A1Node
{
	private:
    	int n_layers = 1;
		float sensor_h_fov = 45.f, sensor_v_fov = 45.f;
		std::string frame_id;

		static VL53L5A1Node* instance;

		ros::NodeHandle nh;

		ros::Publisher 	pub_control;
		ros::Publisher  pub_pcl;
		ros::Subscriber sub_distance;

		sensors_53l5a1_driver::control 		msg_control;

		pcl::PointCloud<pcl::PointXYZ> *cloud_ptr = nullptr;

	public:
		VL53L5A1Node();
		virtual ~VL53L5A1Node();
		void transmitCfg(const sensors_53l5a1_driver::SensorConfig& cfg);
		inline static 	VL53L5A1Node* getInstance(void);
		inline void run(void) { ROS_INFO("Spinning node"); ros::spin(); }
		void rangingResultsCb(const sensors_53l5a1_driver::distanceConstPtr &msg_distance);
		void makePointCloud(const sensors_53l5a1_driver::distanceConstPtr &msg);

};
VL53L5A1Node* VL53L5A1Node::instance = 0;
#endif