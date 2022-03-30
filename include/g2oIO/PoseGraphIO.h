//
// Created by echo on 2019/11/11.
//
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/types_slam3d.h>
#ifndef POSEGRAPHTOOLS_POSEGRAPHIO_H
#define POSEGRAPHTOOLS_POSEGRAPHIO_H

class PoseGraphIO {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
public:
	PoseGraphIO()= default;;
	void insertPose(const Eigen::Isometry3d& pose);
	void saveGraph(std::string g2o_path);

private:
	std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> _odom_buffer;
	Eigen::Isometry3d prev;
	std::vector<g2o::VertexSE3> vertexs;
	std::vector<g2o::EdgeSE3> edges;
protected:
};
#endif //POSEGRAPHTOOLS_POSEGRAPHIO_H
