#ifndef KITTI_HELPER_HPP
#define KITTI_HELPER_HPP
#include <iostream>
#include <vector>
#include <fstream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
/**
 * @brief 读取二进制的激光点云文件
 * 
 * @param lidar_data_path 
 * @return std::vector<float> 
 */
std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);                             // 移动流指针到末尾
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float); //  tellg返回当前指针所在位置,结合seekg可以得到文本长度
    lidar_data_file.seekg(0, std::ios::beg);                             // 流指针复位

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char *>(&lidar_data_buffer[0]), num_elements * sizeof(float));
    return lidar_data_buffer;
}
/**
 * @brief Get the Point Cloud From Bin File
 * 
 * @param path 
 * @param type 
 * @param lidar_points 
 * @return pcl::PointCloud<pcl::PointXYZI> 
 */
pcl::PointCloud<pcl::PointXYZI> getPointCloudFromBin(const std::string &path, const std::string &type, std::vector<Eigen::Vector3f> &lidar_points)
{
    std::vector<float> lidar_data = read_lidar_data(path);
    std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";
    std::vector<Eigen::Vector3f> lidar_points_;
    std::vector<float> lidar_intensities;
    pcl::PointCloud<pcl::PointXYZI> laser_cloud;
    for (std::size_t i = 0; i < lidar_data.size(); i += 4)
    {
        lidar_points_.emplace_back(lidar_data[i], lidar_data[i + 1], lidar_data[i + 2]);
        lidar_intensities.push_back(lidar_data[i + 3]);
        pcl::PointXYZI point;
        point.x = lidar_data[i];
        point.y = lidar_data[i + 1];
        point.z = lidar_data[i + 2];
        point.intensity = lidar_data[i + 3];
        laser_cloud.push_back(point);
    }
    lidar_points = lidar_points_;
    return laser_cloud;
}
/**
 * @brief 判断一个点是否包含nan
 * 
 * @param point 
 * @return true 
 * @return false 
 */
bool isNanPoint(const Eigen::Vector3f &point)
{
    if (isnan(point[0]) || isnan(point[1]) || isnan(point[2]))
    {
        return true;
    }
    return false;
}
/**
 * @brief 计算两点之间的距离
 * 
 * @param p1 
 * @param p2 
 * @return float 
 */
float disBetweenPoints(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2)
{
    return (p1 - p2).norm();
}

void outputKITTIPose(std::ofstream &odom_ofs, Eigen::Isometry3d lidar_pose)
{
    for (int index = 0; index < 3; ++index)
    {
        for (int j = 0; j < 4; ++j)
        {
            odom_ofs << lidar_pose(index, j);
            if (index == 2 && j == 3)
            {
                odom_ofs << std::endl;
            }
            else
            {
                odom_ofs << " ";
            }
        }
    }
}

#endif