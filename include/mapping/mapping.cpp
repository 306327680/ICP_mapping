//
// Created by echo on 2022/3/30.
//

#include "mapping.h"

void mapping::start() {
    //mapping loop
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_hesai(new pcl::PointCloud<pcl::PointXYZI>);
    ros::Time cur_time;
    mypcdCloud xyzItime;
    pcl::PointCloud<pcl::PointXYZI> undistort,undistort_reg,result_scan;
    pcl::PointCloud<pcl::PointXYZI>::Ptr undistort_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<pcl::PointCloud<pcl::PointXYZI>>  local_map_queue;
    pcl::PointCloud<pcl::PointXYZI> local_map;
    std::vector<Eigen::Matrix4f> icp_odom;
    for(int i = 0;  i <file_names_ .size();i++){
        if (i>=start_id_ && i<=end_id_) {
            readPointCloud(file_names_[i],cloud_hesai, cur_time,xyzItime);
            if(first_cloud){
                first_cloud = false;
                poses.emplace_back(Eigen::Matrix4f::Identity());
                g2osaver.insertPose(Eigen::Isometry3d::Identity());
                local_map_queue.push_back(*cloud_hesai);
            } else{
                icp.transformation = Eigen::Matrix4f::Identity();
                linerDistortion(xyzItime,icp.increase.inverse(),undistort);// 上一次的odom 去去除畸变
                *undistort_ptr = undistort;
                icp.SetNormalICP(); //设定odom icp参数0.5+acc*0.01
                icp.normalIcpRegistration(undistort_ptr,local_map);
                icp_odom.push_back(icp.increase);//T_l_l+1
                //2.3.1 再次去畸变 再次减小范围
                linerDistortion(xyzItime,icp.increase.inverse(),undistort);//这一次的odom去除畸变
                local_map = lidarLocalMapDistance(poses,local_map_queue,0.1,20);
                //scan-local map
                icp.SetPlaneICP();
                icp.normalIcpRegistrationlocal(undistort_ptr,local_map);
                icp_odom.back() = icp_odom.back()*icp.pcl_plane_plane_icp->getFinalTransformation();
                linerDistortion(xyzItime,icp.increase.inverse(),undistort);
                //获取当前位置
                Eigen::Matrix4f current_pose = Eigen::Matrix4f::Identity();
                for (const auto & k : icp_odom) {
                    current_pose *= k;
                }
                g2osaver.insertPose(Eigen::Isometry3d(current_pose.matrix().cast<double>()));
                pcl::transformPointCloud(undistort,result_scan,current_pose);//得到最终的tf过的pcd
                //todo pub 1.current_pose 2.result_scan
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

void mapping::linerDistortion(mypcdCloud input, Eigen::Matrix4f increase, pcl::PointCloud<pcl::PointXYZI> &output) {
    output.clear();
    Eigen::Matrix4d transInput;
    Eigen::Matrix4d out;
    Eigen::Isometry3d se3;
    se3 = increase.cast<double>();
    mypcd temp_point;
    mypcd distort_point;
    mypcdCloud time_fix;
    pcl::PointCloud<mypcd>::Ptr Individual(new pcl::PointCloud<mypcd>);
    pcl::PointCloud<mypcd>::Ptr Individual_bef(new pcl::PointCloud<mypcd>);
    Eigen::Isometry3d transpc;
    for (int i = 0; i < input.size(); ++i) {
        temp_point.x = input[i].x;
        temp_point.y = input[i].y;
        temp_point.z = input[i].z;
        temp_point.intensity = input[i].intensity;
        temp_point.timestamp = (input[i].timestamp - input[input.size()-1].timestamp) * 10;
        time_fix.push_back(temp_point);
        //跟据时间戳插值
        trinterp(Eigen::Matrix4d::Identity(), transInput,  temp_point.timestamp, out);
        transpc.matrix() = out.matrix();
        Eigen::Matrix4d convert;
        convert = transpc.matrix();
        convert = transpc.matrix().inverse();
        pcl::transformPointCloud(*Individual_bef, *Individual, convert);
        pcl::PointXYZI point_out;
        point_out.x = Individual->back().x;
        point_out.y = Individual->back().y;
        point_out.z = Individual->back().z;
        point_out.intensity = Individual->back().intensity;
        output.push_back(point_out);
    }
}

void mapping::trinterp(Eigen::Matrix4d T0, Eigen::Matrix4d &T1, double r, Eigen::Matrix4d &T) {
    Eigen::Vector4d q0, q1, qr;
    Eigen::Vector3d p0, p1, pr;
    Eigen::Matrix3d R, R0;
    //R 为插值之后的旋转矩阵
    rotMat2quaternion(T0, q0);  // 位姿矩阵转换为四元数
    rotMat2quaternion(T1, q1);
    p0 << T0(0, 3), T0(1, 3), T0(2, 3);       // 提取出位置矩阵
    p1 << T1(0, 3), T1(1, 3), T1(2, 3);
    qinterp(q0, q1, r, qr);      // 进行四元数插值 10.4
    pr = p0*(1 - r) + r*p1;      // 进行位置插值
    quatern2rotMat(qr, R);       // 四元数转旋转矩阵
    T(0, 0) = R(0, 0); T(0, 1) = R(0, 1); T(0, 2) = R(0, 2); T(0, 3) = pr(0);
    T(1, 0) = R(1, 0); T(1, 1) = R(1, 1); T(1, 2) = R(1, 2); T(1, 3) = pr(1);
    T(2, 0) = R(2, 0); T(2, 1) = R(2, 1); T(2, 2) = R(2, 2); T(2, 3) = pr(2);
    T(3, 0) = 0;       T(3, 1) = 0;       T(3, 2) = 0;       T(3, 3) = 1;
}

void mapping::qinterp(Eigen::Vector4d &Q1, Eigen::Vector4d &Q2, double r, Eigen::Vector4d &q_quaternion_interpolation) {
    double k0, k1, theta, sin_theta, cos_theta;
    Eigen::Vector4d q_temp;
    cos_theta = Q1(0)*Q2(0) + Q1(1)*Q2(1) + Q1(2)*Q2(2) + Q1(3)*Q2(3);
    if (cos_theta < 0) {
        Q2 = -Q2;
        cos_theta = -cos_theta;
    }
    if ((cos_theta) > 0.9999999999) {
        k0 = 1.00 - r;
        k1 = r;

    }
    else {
        sin_theta = sqrt(1.00 - pow(cos_theta, 2));
        theta = atan2(sin_theta, cos_theta);
        k0 = sin((1.000 - r)*theta) / sin(theta);
        k1 = sin((r)*theta) / sin(theta);
    }
    q_quaternion_interpolation = k0* Q1 + k1 * Q2;
    Eigen::Quaterniond out;
    out.x() = q_quaternion_interpolation(0);
    out.y() = q_quaternion_interpolation(1);
    out.z() = q_quaternion_interpolation(2);
    out.w() = q_quaternion_interpolation(3);
    out.normalize();
    q_quaternion_interpolation(0) = out.x();
    q_quaternion_interpolation(1) = out.y();
    q_quaternion_interpolation(2) = out.z();
    q_quaternion_interpolation(3) = out.w();
}

void mapping::rotMat2quaternion(Eigen::Matrix4d &T, Eigen::Vector4d &q_quaternion) {
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    double w, x, y, z;
    R(0, 0) = T(0, 0);
    R(0, 1) = T(0, 1);
    R(0, 2) = T(0, 2);
    R(1, 0) = T(1, 0);
    R(1, 1) = T(1, 1);
    R(1, 2) = T(1, 2);
    R(2, 0) = T(2, 0);
    R(2, 1) = T(2, 1);
    R(2, 2) = T(2, 2);

    double trace = R(0, 0) + R(1, 1) + R(2, 2);
    double epsilon = 1E-12;
    if (trace > epsilon){
        double s = 0.5 / sqrt(trace + 1.0);
        w = 0.25 / s;
        x = -(R(2, 1) - R(1, 2)) * s;
        y = -(R(0, 2) - R(2, 0)) * s;
        z = -(R(1, 0) - R(0, 1)) * s;
    }
    else{
        if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2)){
            double s = 2.0 * sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2));
            w = -(R(2, 1) - R(1, 2)) / s;
            x = 0.25 * s;
            y = (R(0, 1) + R(1, 0)) / s;
            z = (R(0, 2) + R(2, 0)) / s;
        }else if (R(1, 1) > R(2, 2)){
            double s = 2.0 * sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2));
            w = -(R(0, 2) - R(2, 0)) / s;
            x = (R(0, 1) + R(1, 0)) / s;
            y = 0.25 * s;
            z = (R(1, 2) + R(2, 1)) / s;
        }else{
            double s = 2.0 * sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1));
            w = -(R(1, 0) - R(0, 1)) / s;
            x = (R(0, 2) + R(2, 0)) / s;
            y = (R(1, 2) + R(2, 1)) / s;
            z = 0.25 * s;
        }
    }
    q_quaternion(0) = w;
    q_quaternion(1) = x;
    q_quaternion(2) = y;
    q_quaternion(3) = z;
}

void mapping::quatern2rotMat(Eigen::Vector4d &q_quaternion, Eigen::Matrix3d &R) {
    Eigen::Quaterniond q;
    q.x() = q_quaternion(0);
    q.y() = q_quaternion(1);
    q.z() = q_quaternion(2);
    q.w() = q_quaternion(3);
//	R=Eigen::Matrix3d(q);
    R(0, 0) = 2.0000 * pow(q_quaternion(0), 2) - 1.0000 + 2.0000 * pow(q_quaternion(1), 2);
    R(0, 1) = 2.0000 * (q_quaternion(1)*q_quaternion(2) + q_quaternion(0)*q_quaternion(3));
    R(0, 2) = 2.0000 * (q_quaternion(1)*q_quaternion(3) - q_quaternion(0)*q_quaternion(2));
    R(1, 0) = 2.0000* (q_quaternion(1)*q_quaternion(2) - q_quaternion(0)*q_quaternion(3));
    R(1, 1) = 2.0000 * pow(q_quaternion(0), 2) - 1.0000 + 2.0000 * pow(q_quaternion(2), 2);
    R(1, 2) = 2.0000* (q_quaternion(2)*q_quaternion(3) + q_quaternion(0)*q_quaternion(1));
    R(2, 0) = 2.0000 * (q_quaternion(1)*q_quaternion(3) + q_quaternion(0)*q_quaternion(2));
    R(2, 1) = 2.0000 * (q_quaternion(2)*q_quaternion(3) - q_quaternion(0)*q_quaternion(1));
    R(2, 2) = 2.0000 * pow(q_quaternion(0), 2) - 1.0000 + 2.0000 * pow(q_quaternion(3), 2);
}

pcl::PointCloud<pcl::PointXYZI> mapping::lidarLocalMapDistance(std::vector<Eigen::Matrix4f> &poses,
                                                               std::vector<pcl::PointCloud<pcl::PointXYZI>> &clouds,
                                                               double distiance, int buffer_size) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> map_temp;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_temp_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<Eigen::Matrix4f> poses_tmp;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> clouds_tmp;
    Eigen::Matrix4f tf_all = Eigen::Matrix4f::Identity();
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor1;
    pcl::PointCloud<int> keypointIndices;
    Eigen::Matrix4f pose_latest;
    Eigen::Matrix4f pose_last;
    Eigen::Matrix4f pose_diff;
    float dx,dy,dz;
    double distance_calc;

    //室內的參數 mod 2021/11/4 //4.整体地图降采样
    float filter_param_x = 0.15;
    float filter_param_y = 0.15;
    float filter_param_z = 0.05;

    //0. 得到这一阵位姿和上一阵位姿
    if(poses.size()> 1){
        pose_latest =  poses.back();
        pose_last = poses.at(poses.size()-2);
        dx = pose_latest(0,3) - pose_last(0,3);
        dy = pose_latest(1,3) - pose_last(1,3);
        dz = pose_latest(2,3) - pose_last(2,3);
        distance_calc = sqrtf(dx*dx + dy*dy + dz*dz);
    }else{
        distance_calc = 100;
    }
    //**满足运动距离
    if(distance_calc>distiance){
        //1. 最新帧降采样
        *map_temp_ptr = clouds.back();
        sor.setInputCloud(map_temp_ptr);
        sor.setLeafSize(0.05f, 0.05f, 0.05f);//室内
        sor.filter(clouds.back());

        //点云数量超过了设置的阈值
        if (poses.size() >= buffer_size) {
            //后期点云比较多的情况
            for (int i = poses.size() - buffer_size; i < poses.size(); i++) {
                pcl::transformPointCloud(clouds[i], map_temp, poses[i]);
                //下面的if 保证了 clouds 里面都是 降采样过的点云
                *map_ptr += map_temp;
                poses_tmp.push_back(poses[i]);
                clouds_tmp.push_back(clouds[i]);
            }
            poses = poses_tmp;
            clouds = clouds_tmp;
            //3. 地图转换到当前的最后一帧的位姿
            pcl::transformPointCloud(*map_ptr, map_temp, poses.back().inverse());
            *map_ptr = map_temp;
            //4.整体地图降采样
            sor.setInputCloud(map_ptr);
            sor.setLeafSize(filter_param_x, filter_param_y, filter_param_z);
            sor.filter(map_temp);

            return map_temp;

        } else if (poses.size() > 1) {//第一开始点云不多的情况 不要第一个帧,因为没有去畸变
            for (int i = 1; i < poses.size(); i++) {
                pcl::transformPointCloud(clouds[i], map_temp, poses[i]);
                *map_ptr += map_temp;
            }
        } else {
            pcl::transformPointCloud(clouds[0], *map_ptr, poses[0]);
        }
        //3. 地图转换到当前的最后一帧的位姿
        pcl::transformPointCloud(*map_ptr, map_temp, poses.back().inverse());
        *map_ptr = map_temp;
        //4.整体地图降采样
        sor.setInputCloud(map_ptr);
        sor.setLeafSize(filter_param_x, filter_param_y, filter_param_z);
        sor.filter(map_temp);
        return map_temp;
    }else{ 	//**不满足运动距离
        //2. 这个只保留前面的点云和位姿
        if (poses.size() >= buffer_size) {
            //点云足够多的情况
            for (int i = 0; i < buffer_size; i++) {
                pcl::transformPointCloud(clouds[i], map_temp, poses[i]);
                //下面的if 保证了 clouds 里面都是 降采样过的点云
                *map_ptr += map_temp;
                poses_tmp.push_back(poses[i]);
                clouds_tmp.push_back(clouds[i]);
            }
            poses = poses_tmp;
            clouds = clouds_tmp;
            //3. 地图转换到最新的一帧下位位置,继续计算增量 last_local_map
            sor.setInputCloud(map_ptr);
            sor.setLeafSize(filter_param_x, filter_param_y, filter_param_z);
            sor.filter(map_temp);
            pcl::transformPointCloud(map_temp, *map_ptr, pose_latest.inverse());
            return *map_ptr;

        } else if (poses.size() > 1) {//第一开始点云不多的情况 不要第一个帧,因为没有去畸变
            for (int i = 1; i < poses.size(); i++) {
                pcl::transformPointCloud(clouds[i], map_temp, poses[i]);
                *map_ptr += map_temp;
            }
            //3. 地图转换到最新的一帧下位位置,继续计算增量 last_local_map
            pcl::transformPointCloud(*map_ptr, map_temp, pose_latest.inverse());
            *map_ptr = map_temp;
            //4.降采样
            sor.setInputCloud(map_ptr);
            sor.setLeafSize(filter_param_x, filter_param_y, filter_param_z);
            sor.filter(map_temp);
            return map_temp;
        } else {
            pcl::transformPointCloud(clouds[0], *map_ptr, poses[0]);
            //3. 地图转换到最新的一帧下位位置,继续计算增量 last_local_map
            pcl::transformPointCloud(*map_ptr, map_temp, pose_latest.inverse());
            *map_ptr = map_temp;
            //4.降采样
            sor.setInputCloud(map_ptr);
            sor.setLeafSize(filter_param_x, filter_param_y, filter_param_z);
            sor.filter(map_temp);
            return map_temp;
        }
    }
}
