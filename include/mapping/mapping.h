//
// Created by echo on 2022/3/30.
//

#ifndef ICP_MAPPING_MAPPING_H
#define ICP_MAPPING_MAPPING_H
#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <PoseGraphIO.h>
#include <registration.h>
struct PointXYZIT {
    PCL_ADD_POINT4D
    float intensity;
    float timestamp;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)
                                          (float, intensity, intensity)(float, timestamp, timestamp))

typedef PointXYZIT mypcd;
typedef pcl::PointCloud<mypcd> mypcdCloud;

class mapping {
public:
    mapping()= default;
    mapping(int s,int e,std::vector<std::string> file_names){start_id_ = s; end_id_ = e;file_names_ = file_names;}
    void start();
    void linerDistortion(mypcdCloud input, Eigen::Matrix4f increase, pcl::PointCloud<pcl::PointXYZI> &output);
    registration icp;
private:
    void readPointCloud(std::string file_names,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_result,ros::Time& cur_time,
                        mypcdCloud& cloud_for_mapping);
    void trinterp(Eigen::Matrix4d T0, Eigen::Matrix4d &T1, double r, Eigen::Matrix4d &T);
    void qinterp(Eigen::Vector4d &Q1, Eigen::Vector4d &Q2, double r,Eigen::Vector4d &q_quaternion_interpolation);
    void rotMat2quaternion(Eigen::Matrix4d &T, Eigen::Vector4d &q_quaternion);
    void quatern2rotMat(Eigen::Vector4d &q_quaternion, Eigen::Matrix3d &R);
    std::vector<std::string> file_names_;
    int start_id_ = 0;
    int end_id_ = 0;
    bool first_cloud = true;
    PoseGraphIO g2osaver;
    std::vector<Eigen::Matrix4f> poses;
};


#endif //ICP_MAPPING_MAPPING_H
