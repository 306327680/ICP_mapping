//
// Created by echo on 2022/3/29.
//

#ifndef ICP_MAPPING_TOOLS_H
#define ICP_MAPPING_TOOLS_H
#include <iostream>
#include <string>
#include <vector>
#include <dirent.h>
#include <algorithm>
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4d)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Isometry3d)
class tools {
public:
    //1. Get the name of files
    tools()= default;
    void setFileName();
    bool GetIntFileNames(const std::string directory, const std::string suffix);
    void setStartEnd();
    std::string pcd_path;
    std::vector<std::string> file_names_;
    int start_id{};
    int end_id{};
};


#endif //ICP_MAPPING_TOOLS_H
