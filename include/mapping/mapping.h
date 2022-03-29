//
// Created by echo on 2022/3/30.
//

#ifndef ICP_MAPPING_MAPPING_H
#define ICP_MAPPING_MAPPING_H
#include <ros/ros.h>
#include <pcl-1.10/pcl/io/pcd_io.h>

class mapping {
public:
    mapping()= default;
    mapping(int s,int e,std::vector<std::string> file_names){start_id_ = s; end_id_ = e;file_names_ = file_names;}
    void start();
private:
    void readPointCloud(std::string file_names,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_hesai,ros::Time& cur_time,mypcdCloud& xyzItimeRing,std::string LiDAR_type = "VLP");
    std::vector<std::string> file_names_;
    int start_id_ = 0;
    int end_id_ = 0;

};


#endif //ICP_MAPPING_MAPPING_H
