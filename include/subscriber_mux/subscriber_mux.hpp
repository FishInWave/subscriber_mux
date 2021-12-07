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
#include <utils/rslidar_utils.hpp>
#include <utils/time_utils.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <deque>
#include <mutex>
#include <thread>
#include <chrono>

using namespace std;
using ExactTime = message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>;
class SubscriberMux
{
public:
    ros::NodeHandle nh_;
    ros::Subscriber GPS_fix_sub_;
    ros::Subscriber GPS_odom_sub_;
    ros::Subscriber raw_pointcloud_sub_, deskewed_pointcloud_sub_;

    // message_filters::Subscriber<sensor_msgs::PointCloud2> raw_points_sub_;
    // message_filters::Subscriber<sensor_msgs::PointCloud2> deskewed_points_sub_;

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

        // message_filters::Subscriber<sensor_msgs::PointCloud2> raw_points_sub_(nh_, "/points_raw", 100);
        // message_filters::Subscriber<sensor_msgs::PointCloud2> deskewed_points_sub_(nh_, "/lio_sam/deskew/cloud_deskewed", 100);
        // message_filters::Synchronizer<ExactTime> sync(ExactTime(200), raw_points_sub_, deskewed_points_sub_);
        // sync.registerCallback(boost::bind(&SubscriberMux::cloud_sync_callback, this, _1, _2));

        raw_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("raw", 10);
        deskew_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("deskew", 10);

        outputSrv_ = nh_.advertiseService("subscriber_mux/output_frequency", &SubscriberMux::outputService, this);
    }

    void raw_callback(const sensor_msgs::PointCloud2ConstPtr &raw)
    {
        {
            lock_guard<mutex> lock(raw_mutex_);
            raw_deque_.push_back(*raw);
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
    // 同步无法用
    // void cloud_sync_callback(const sensor_msgs::PointCloud2ConstPtr &raw, const sensor_msgs::PointCloud2ConstPtr &deskewed)
    // {
    //     ROS_INFO_STREAM ("hello");
    //     if (raw->header.stamp == deskewed->header.stamp)
    //     {
    //         raw_pub_.publish(*raw);
    //         deskew_pub_.publish(*deskewed);
    //     }
    //     else
    //     {
    //         ROS_INFO_STREAM("raw stamp: " << raw->header.stamp << " deskewed stamp: " << deskewed->header.stamp);
    //     }
    // }

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