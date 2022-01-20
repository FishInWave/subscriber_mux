#ifndef SUBSCRIBER_MUX_HPP
#define SUBSCRIBER_MUX_HPP
#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <subscriber_mux/output_frequency.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <utils/lidar_utils.hpp>
#include <utils/time_utils.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <deque>
#include <mutex>
#include <thread>
#include <chrono>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>
using namespace std;
using ExactTime = message_filters::sync_policies::ExactTime<sensor_msgs::FluidPressure, geometry_msgs::PointStamped>;
class SubscriberMux
{
public:
    ros::NodeHandle nh_;
    ros::Subscriber GPS_fix_sub_;
    ros::Subscriber GPS_odom_sub_;
    ros::Subscriber raw_pointcloud_sub_, deskewed_pointcloud_sub_;

    //气压相关
    // ros::Subscriber pressure_sub_;
    // ris::Subscriber height_sub_;
    ros::Publisher math_height_pub_;
    ros::Publisher delta_height_pub_;

    ros::Publisher raw_pub_, deskew_pub_;

    string GPS_topic_;
    string GPS_odom_topic_;

    map<float, int> x_cov_map_, y_cov_map_, z_cov_map_;
    map<float, int> x_cov_odom_map_, y_cov_odom_map_, z_cov_odom_map_;

    ros::ServiceServer outputSrv_;

    mutex raw_mutex_, deskewed_mutex_;

    deque<sensor_msgs::PointCloud2> raw_deque_, deskewed_deque_;

    sensor_msgs::PointCloud2 current_raw_pc_, current_deskewed_pc_;

    SubscriberMux()
    {
        initial_parameter();
        GPS_fix_sub_ = nh_.subscribe<sensor_msgs::NavSatFix>(GPS_topic_, 10, &SubscriberMux::GPS_Callback, this, ros::TransportHints().tcpNoDelay());
        GPS_odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(GPS_odom_topic_, 10, &SubscriberMux::GPS_Odom_Callback, this, ros::TransportHints().tcpNoDelay());

        raw_pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/points_raw", 100, &SubscriberMux::raw_callback, this, ros::TransportHints().tcpNoDelay());
        deskewed_pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/lio_sam/deskew/cloud_deskewed", 100, &SubscriberMux::deskewed_callback, this, ros::TransportHints().tcpNoDelay());

        message_filters::Subscriber<geometry_msgs::PointStamped> height_sub_(nh_,"/zed2/zed_node/height",100);
        message_filters::Subscriber<sensor_msgs::FluidPressure> pressure_sub_(nh_,"/zed2/zed_node/atm_press",100);
        message_filters::Synchronizer<ExactTime> sync(ExactTime(200), pressure_sub_, height_sub_);
        sync.registerCallback(boost::bind(&SubscriberMux::pressure_callback,this,_1,_2));
        math_height_pub_ = nh_.advertise<geometry_msgs::PointStamped>("math_height",10);
        delta_height_pub_ = nh_.advertise<geometry_msgs::PointStamped>("delta_height",10);

        raw_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("raw", 10);
        deskew_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("deskew", 10);

        outputSrv_ = nh_.advertiseService("subscriber_mux/output_frequency", &SubscriberMux::outputService, this);
        ros::spin();
    }

    void raw_callback(const sensor_msgs::PointCloud2ConstPtr &raw)
    {
        {
            lock_guard<mutex> lock(raw_mutex_);
            raw_deque_.push_back(*raw);
        }
        pcl::PointCloud<VelodynePointXYZIRT> cloud;
        sensor_msgs::PointCloud2 cloud_msg(*raw);
        pcl::moveFromROSMsg(cloud_msg,cloud);
        vector<vector<float>> horizon(64);
        for(auto point : cloud){
            float angle = atan2(point.y,point.x)*180/M_PI;
            horizon.at(point.ring).push_back(angle);
        }
        for(int i = 0; i < horizon.size(); ++i){
            vector<float> ring = horizon[i];
            sort(ring.begin(),ring.end(),less<float>());
            cout << "ring number: " << i << "ring points: " << ring.size() << endl;
            for(int j = 1 ; j < ring.size();j++){
                cout << ring[j]-ring[j-1] << " " ;
            }
            cout << endl;
        }

    }

    void deskewed_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        lock_guard<mutex> lock(deskewed_mutex_);
        deskewed_deque_.push_back(*msg);
    }
    void run()
    {
        while (ros::ok())
        {
            if (raw_deque_.empty() || deskewed_deque_.empty())
            {
                this_thread::sleep_for(chrono::milliseconds(100));
                continue;
            }

            {
                lock_guard<mutex> lock(deskewed_mutex_);
                current_deskewed_pc_ = deskewed_deque_.front();
                deskewed_deque_.pop_front();
                current_deskewed_pc_.header.frame_id = "velodyne";
            }
            double current_time = ROS_TIME(&current_deskewed_pc_);
            {
                lock_guard<mutex> lock(raw_mutex_);
                while (!raw_deque_.empty())
                {
                    if (ROS_TIME(&raw_deque_.front()) - current_time <= 1e-3)
                    {
                        current_raw_pc_ = raw_deque_.front();
                        raw_deque_.pop_front();
                    }
                    else
                    {
                        break;
                    }
                }
            }
            raw_pub_.publish(current_raw_pc_);
            deskew_pub_.publish(current_deskewed_pc_);
            this_thread::sleep_for(chrono::milliseconds(100));
        }

        return;
    }
    void pressure_callback(const sensor_msgs::FluidPressureConstPtr &pressure_msgs, const geometry_msgs::PointStampedConstPtr &height_msgs)
    {
        static float h0 = 0;
        
        static float P0 = 0;
        float P = pressure_msgs->fluid_pressure;
        float h ;
        if(P0 == 0){
            P0 = P;
            h = h0;
        }else{
            h = h0 + 18435.66*(1+15.0/273.15)*log10(P0/P);
        }
        float delta_h = h - height_msgs->point.z;
        geometry_msgs::PointStamped math_height,delta_height;
        math_height = *height_msgs;
        math_height.point.z = h;
        delta_height = *height_msgs;
        delta_height.point.z = delta_h;
        math_height_pub_.publish(math_height);
        delta_height_pub_.publish(delta_height);
        
    }

    bool outputService(subscriber_mux::output_frequencyRequest &req, subscriber_mux::output_frequencyResponse &res)
    {
        cout << "------------GPS Cov-----------" << endl;
        cout << "------x freq--------" << endl;
        for (auto p : x_cov_map_)
        {
            cout << p.first << " " << p.second << endl;
        }
        cout << "------y freq--------" << endl;
        for (auto p : y_cov_map_)
        {
            cout << p.first << " " << p.second << endl;
        }
        cout << "------z freq--------" << endl;
        for (auto p : z_cov_map_)
        {
            cout << p.first << " " << p.second << endl;
        }
        cout << "------------GPS Odom Cov-----------" << endl;
        cout << "------x freq--------" << endl;
        for (auto p : x_cov_odom_map_)
        {
            cout << p.first << " " << p.second << endl;
        }
        cout << "------y freq--------" << endl;
        for (auto p : y_cov_odom_map_)
        {
            cout << p.first << " " << p.second << endl;
        }
        cout << "------z freq--------" << endl;
        for (auto p : z_cov_odom_map_)
        {
            cout << p.first << " " << p.second << endl;
        }
        res.success = true;
        return true;
    }
    ~SubscriberMux()
    {
    }
    void initial_parameter()
    {
        nh_.param<string>("GPS/GPS_topic", GPS_topic_, "/gps/fix");
        nh_.param<string>("GPS/GPS_odom_topic", GPS_odom_topic_, "/odometry/gps");
    }
    void GPS_Odom_Callback(const nav_msgs::Odometry GPS_odom_msg)
    {
        float x = GPS_odom_msg.pose.covariance[0];
        float y = GPS_odom_msg.pose.covariance[7];
        float z = GPS_odom_msg.pose.covariance[14];
        if (x_cov_odom_map_.find(x) != x_cov_odom_map_.end())
        {
            x_cov_odom_map_[x]++;
        }
        else
        {
            x_cov_odom_map_[x] = 1;
        }
        if (y_cov_odom_map_.find(y) != y_cov_odom_map_.end())
        {
            y_cov_odom_map_[y]++;
        }
        else
        {
            y_cov_odom_map_[y] = 1;
        }
        if (z_cov_odom_map_.find(z) != z_cov_odom_map_.end())
        {
            z_cov_odom_map_[z]++;
        }
        else
        {
            z_cov_odom_map_[z] = 1;
        }
    }
    void GPS_Callback(const sensor_msgs::NavSatFix GPS_msg)
    {
        // 接收GPS消息，读取其cov，建立直方图
        float x = GPS_msg.position_covariance[0];
        float y = GPS_msg.position_covariance[4];
        float z = GPS_msg.position_covariance[8];
        if (x_cov_map_.find(x) != x_cov_map_.end())
        {
            x_cov_map_[x]++;
        }
        else
        {
            x_cov_map_[x] = 1;
        }
        if (y_cov_map_.find(y) != y_cov_map_.end())
        {
            y_cov_map_[y]++;
        }
        else
        {
            y_cov_map_[y] = 1;
        }
        if (z_cov_map_.find(z) != z_cov_map_.end())
        {
            z_cov_map_[z]++;
        }
        else
        {
            z_cov_map_[z] = 1;
        }
    }
};
#endif