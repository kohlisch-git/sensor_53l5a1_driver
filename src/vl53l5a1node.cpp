/**
 * @file    vl53l5a1node.cpp
 * @author  Niklas Kohlisch kohlischni71082@th-nuernberg.de
 * @brief   Rosserial sensor driver for the Nucleo 53L5A1
 * @version 0.1
 * @date    2021-11-07
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "vl53l5a1node.h"

/**
 * @brief Construct a new VL53L5A1Node::VL53L5A1Node object
 * @details Register node, subscribers, publishers and set params
 * 
 */
VL53L5A1Node::VL53L5A1Node()
{
    ros::NodeHandle nh("~");
    std::string output_topic;
    nh.param("n_layers",  n_layers,  2);

    if (n_layers < 1 || n_layers > 2) {
        ROS_ERROR("Parameter not allowed, setting to default: 2");
        n_layers = 2;
    } else
        n_layers -= 1;

    nh.param("output_topic", output_topic, std::string("/vl53l5_cloud"));
    nh.param("angle_h_fov", sensor_h_fov, 45.f);
    nh.param("angle_v_fov", sensor_v_fov, 45.f);
    nh.param("sensor_frame_id", sensor_frame_id, std::string("base"));

    pub_control = nh.advertise<sensors_53l5a1_driver::control>(topic_control, 500);
    pub_pcl = nh.advertise<sensor_msgs::PointCloud2> (output_topic, 1);
    sub_distance = nh.subscribe(topic_distance, 1, &VL53L5A1Node::rangingResultsCb, this);
    if (!cloud_ptr)
        cloud_ptr = new pcl::PointCloud<pcl::PointXYZ>();
}

/**
 * @brief   Destroy the VL53L5A1Node::VL53L5A1Node object
 * @details Delete singelton and free point cloud
 */
VL53L5A1Node::~VL53L5A1Node()
{
    if (instance) {
        delete instance;
        instance = 0;
    }
    if (cloud_ptr) {
        cloud_ptr->clear();
        delete cloud_ptr;
    }
}

/**
 * @brief Transmit settings from reconfigure to hardware
 * 
 * @param cfg dynamic reconfigure settings
 */
void VL53L5A1Node::transmitCfg(const sensors_53l5a1_driver::SensorConfig& cfg)
{
    ROS_INFO("Parameters uptated via reconfigure");
    msg_control.mode_continuous      = cfg.ranging_mode;
    msg_control.ranging_frequency    = cfg.frequency;
    msg_control.resolution_4x4       = cfg.resolution;
    msg_control.integration_time     = cfg.integration_time;
    msg_control.sharpener_in_percent = cfg.sharpener;
    msg_control.order_by_strongest   = cfg.target_order;
    /* Removed, since not needed */
    msg_control.power_on             = 1; // cfg.power_on;
    msg_control.sensor_to_sleep      = 0; // cfg.sensor_to_sleep;
    /* Transmit to hardware */
    pub_control.publish(msg_control);
}

/**
 * @brief Get instance of singelton, if no instance yet, make new instance
 * 
 * @return VL53L5A1Node* instance
 */
VL53L5A1Node* VL53L5A1Node::getInstance(void)
{
    if(!VL53L5A1Node::instance)
        instance = new VL53L5A1Node();
    return instance;
}

/**
 * @brief Convert distances to point cloud format
 * 
 * @param msg distance message from hardware
 */
void VL53L5A1Node::makePointCloud(const sensors_53l5a1_driver::distanceConstPtr &msg)
{
    float v_angle_inc, h_angle_inc, v_angle_initial, h_angle_initial, x, y, z, h_angle, v_angle;
    v_angle_initial = sensor_v_fov * 0.5f;
    h_angle_initial = sensor_h_fov * 0.5f;
    uint8_t step = 0, v_pos, h_pos, inc;

    if (msg->resolution == 16) {
        /* 4x4 zones -> points in center of zone */
        v_angle_inc = sensor_v_fov / 5.f;
        h_angle_inc = sensor_h_fov / 5.f;
        step = 16;
        inc = 4;
    }
    else if (msg->resolution == 64) {
        /* 8x8 zones -> points in center of zone */
        v_angle_inc = sensor_v_fov / 9.f;
        h_angle_inc = sensor_h_fov / 9.f;
        step = 8;
        inc = 1;
    }
    else {
        ROS_ERROR("Error: Resolution unknown!");
        return;
    }
    /* if layer o, clear previous cloud, else append to cloud */
    if (msg->layer == 0) {
        cloud_ptr->clear(); //reset cloud
    }
    /* append points of layer to cloud */
    for (uint8_t i = 0; i < 64; i += inc) {
        /* skip invalid measurements */
        if (msg->distance_mm[i] > 0x1FFF)
            continue;
        /* align points in center of zone */
        v_pos = (i % step) / inc + 1;
        h_pos = (i / step) + 1;
        v_angle = (v_angle_initial - v_pos * v_angle_inc) * M_PI/ 180.f;
        h_angle = (h_angle_initial - h_pos * h_angle_inc) * M_PI/ 180.f;
        /* x-axis is distance, y-axis left to right on sensor, z is bottom to top on sensor */
        x = msg->distance_mm[i] * 0.001;
        if (msg->distance_mm[i] == 0)   // having points in the origin is dangerous!
            x = 0.0001;                 // -> make tem close to origin, but not 0!
        y = x * std::sin(v_angle);
        z = x * std::sin(h_angle) * std::cos(v_angle);
        cloud_ptr->insert(cloud_ptr->end(), pcl::PointXYZ(x,y,z));
    }
    /* wait for measurements of highest layer, if message received, publish cloud with all layers */
    if (msg->layer == n_layers) {
        /* convert to ROS-format and publish */
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_ptr, output);
        output.header.frame_id = sensor_frame_id;
        output.header.stamp = ros::Time::now();
        pub_pcl.publish(output);
    }
}

/**
 * @brief Callback function for distance measurements from rosserial hardware
 * 
 * @param msg_distance 
 */
void VL53L5A1Node::rangingResultsCb(const sensors_53l5a1_driver::distanceConstPtr &msg_distance)
{
    // ROS_INFO("Received msg with seq %d", msg_distance->header.seq);
    VL53L5A1Node::getInstance()->makePointCloud(msg_distance);
}

/**
 * @brief Dynamic reconfigure callback
 * 
 * @param cfg   parameter configuration
 * @param level bit mask barameters (not needed)
 */
void reconfigureCallback(sensors_53l5a1_driver::SensorConfig &cfg, uint32_t level)
{
    ROS_INFO("Parameters have been modified via reconfigure");
    VL53L5A1Node::getInstance()->transmitCfg(cfg);
}

/**
 * @brief main application entry point
 * 
 * @param argc number of command line arguments
 * @param argv command line arguments
 * @return int 
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, node_name);
    dynamic_reconfigure::Server<sensors_53l5a1_driver::SensorConfig> srv;
    dynamic_reconfigure::Server<sensors_53l5a1_driver::SensorConfig>::CallbackType f;
    f = boost::bind(&reconfigureCallback, _1, _2);
    srv.setCallback(f);
    VL53L5A1Node::getInstance()->run();
    return 0;
}