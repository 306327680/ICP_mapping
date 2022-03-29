//
// Created by echo on 2022/3/30.
//

#include "mapping.h"

void mapping::start() {
    //mapping loop
    for(int i = 0;  i <file_names_ .size();i++){
        if (i>=start_id_ && i<=end_id_) {
            readPointCloud(file_names_[i],cloud_hesai,cur_time,xyzItimeRing,"VLP");
        }
    }
}
