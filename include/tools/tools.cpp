//
// Created by echo on 2022/3/29.
//

#include "tools.h"

bool tools::GetIntFileNames(const std::string directory, const std::string suffix) {
    file_names_.clear();
    DIR *dp;
    struct dirent *dirp;
    dp = opendir(directory.c_str());
    if (!dp) {
        std::cerr << "cannot open directory:" << directory << std::endl;
        return false;
    }
    std::string file;
    while (dirp = readdir(dp)) {
        file = dirp->d_name;
        if (file.find(".") != std::string::npos) {
            file = directory + "/" + file;
            if (suffix == file.substr(file.size() - suffix.size())) {
                file_names_.push_back(file);
            }
        }
    }
    closedir(dp);
    std::sort(file_names_.begin(), file_names_.end());

    if (file_names_.empty()) {
        std::cerr << "directory:" << directory << "is empty" << std::endl;
        return false;
    }
    for (int i = 0; i < file_names_.size(); ++i) {
        file_names_[i]  = directory + "/" + std::to_string(i) +".pcd";
    }
    std::cerr << "路径: " << directory << " 有" << file_names_.size() << "个pcd文件" << std::endl;
    return true;
}

void tools::setStartEnd() {
    std::cout << "设置起始pcd" << std::endl;
    std::cin >> start_id;
    std::cout << "设置结束pcd" << std::endl;
    std::cin >> end_id;
    std::cout << "start: " << start_id << "end: " << end_id << std::endl;
}

void tools::setFileName() {
    std::cout << "设置pcd文件夹路径" << std::endl;
    std::cin >> pcd_path;
    std::cout << "cd文件夹路径: " << pcd_path<< std::endl;
}
