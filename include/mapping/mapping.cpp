//
// Created by echo on 2022/3/30.
//

#include "mapping.h"

void mapping::start() {
    //mapping loop
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_hesai;
    ros::Time cur_time;
    mypcdCloud xyzItimeRing;
    for(int i = 0;  i <file_names_ .size();i++){
        if (i>=start_id_ && i<=end_id_) {
            readPointCloud(file_names_[i],cloud_hesai,cur_time,xyzItimeRing);
            if(first_cloud){
                first_cloud = false;
                poses.emplace_back(Eigen::Matrix4f::Identity());
                g2osaver.insertPose(Eigen::Isometry3d::Identity());
            } else{

            }
        }
    }
}

void mapping::readPointCloud(std::string file_names,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_result,ros::Time& cur_time,
                        mypcdCloud& cloud_for_mapping) {
    mypcdCloud::Ptr load_pcd(new mypcdCloud);
    mypcdCloud xyzirVLP;
    pcl::io::loadPCDFile<mypcd>(file_names, *load_pcd);
    //滤波
    mypcdCloud::Ptr xyzi_ds_ptr(new mypcdCloud);
    //1. 執行濾波 setStddevMulThresh
    pcl::StatisticalOutlierRemoval<mypcd> sor;
    sor.setInputCloud (load_pcd);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1);
    sor.filter (*xyzi_ds_ptr);
    pcl::copyPointCloud(*xyzi_ds_ptr, xyzirVLP);
    //设置时间戳
    cur_time.fromSec(xyzirVLP[xyzirVLP.size()-1].timestamp);
    xyzi_ds_ptr->clear();
    cloud_result->clear();
    for (int j = 0; j < xyzirVLP.size(); ++j) {
        mypcd temp;
        pcl::PointXYZI temp_xyzi;
        temp_xyzi.x = temp.x = xyzirVLP[j].x;
        temp_xyzi.y = temp.y = xyzirVLP[j].y;
        temp_xyzi.z =temp.z = xyzirVLP[j].z;
        temp_xyzi.intensity = temp.intensity = xyzirVLP[j].intensity;
        temp.timestamp = xyzirVLP[j].timestamp;
        //设置距离滤波
        double distance = sqrtf(temp.x*temp.x+temp.y*temp.y);
        if(distance>2){
            xyzi_ds_ptr->push_back(temp);
            cloud_result->push_back(temp_xyzi);
        }
    }
    cloud_for_mapping = *xyzi_ds_ptr;
}
