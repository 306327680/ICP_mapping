//
// Created by echo on 2019/11/26.
//

#include "registration.h"

/*void registration::setParam(std::string configFileName) {
	std::ifstream ifs(configFileName.c_str());

	if (ifs.good())
	{
		icp.loadFromYaml(ifs);
	}
	else
	{
		std::cout<<"Cannot load ICP config from YAML file " << configFileName<<std::endl;
		icp.setDefault();
	}
}

void registration::setMap(pcl::PointCloud<pcl::PointXYZI> pcin) {

	Eigen::MatrixXf testCloudP(4,pcin.size());
	for (int i = 0; i < pcin.size(); ++i) {
		
		testCloudP(0,i) = pcin[i].x;
		testCloudP(1,i) = pcin[i].y;
		testCloudP(2,i) = pcin[i].z;
		testCloudP(3,i) = 1.f;
	}
	DP::Labels labels;
	labels.push_back(DP::Label("X",1));
	labels.push_back(DP::Label("Y",1));
	labels.push_back(DP::Label("Z",1));
	DP localMap(testCloudP,labels);

	icp.setMap(localMap);

}

PM::TransformationParameters  registration::setScan(pcl::PointCloud<pcl::PointXYZI> pcin) {
	PM::TransformationParameters icp_result,T_scanner_to_localMap;
	Eigen::MatrixXf testCloudP(4,pcin.size());
	for (int i = 0; i < pcin.size(); ++i) {
		
		testCloudP(0,i) = pcin[i].x;
		testCloudP(1,i) = pcin[i].y;
		testCloudP(2,i) = pcin[i].z;
		testCloudP(3,i) = 1.f;
	}
	DP::Labels labels;
	labels.push_back(DP::Label("X",1));
	labels.push_back(DP::Label("Y",1));
	labels.push_back(DP::Label("Z",1));
	DP newPointCloud(testCloudP,labels);
	
	const int dimp1(newPointCloud.features.rows());
	T_scanner_to_localMap = PM::TransformationParameters::Identity(dimp1, dimp1);
	icp_result = icp(newPointCloud, T_scanner_to_localMap);
	return icp_result;
}*/
//给一个点云添加normal
void registration::addNormal(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
							 pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_normals(
			new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*cloud,*cloud_source_normals);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZ>);
	searchTree->setInputCloud(cloud_source_normals);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimator_pa;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
	bool omp = true;
	if(omp){
		normalEstimator_pa.setInputCloud(cloud_source_normals);
		normalEstimator_pa.setSearchMethod(searchTree);
		//normalEstimator_pa.setRadiusSearch(0.05);
		normalEstimator_pa.setKSearch(20);//20
		normalEstimator_pa.compute(*normals);
		pcl::concatenateFields(*cloud_source_normals, *normals, *cloud_with_normals);
	}else{
		normalEstimator.setInputCloud(cloud_source_normals);
		normalEstimator.setSearchMethod(searchTree);
		//normalEstimator.setRadiusSearch(0.05);
		normalEstimator.setKSearch(20);
		normalEstimator.compute(*normals);
		pcl::concatenateFields(*cloud_source_normals, *normals, *cloud_with_normals);
	}

}

void registration::SetNormalICP() {
	pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>::Ptr icp(
			new pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>());
	icp->setMaximumIterations(35);
	icp->setMaxCorrespondenceDistance(0.3);
	icp->setTransformationEpsilon(0.001);
	icp->setEuclideanFitnessEpsilon(0.001);
	this->pcl_plane_plane_icp = icp;
	
}
void registration::SetPlaneICP() {
	pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>::Ptr icp(
			new pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>());
	icp->setMaximumIterations(30);
	icp->setMaxCorrespondenceDistance(0.1);
	icp->setTransformationEpsilon(0.0001);
	icp->setEuclideanFitnessEpsilon(0.0001);
	this->pcl_plane_plane_icp = icp;
	
}
pcl::PointCloud<pcl::PointXYZI> registration::normalIcpRegistration(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
																	pcl::PointCloud<pcl::PointXYZI>  target) {
	//todo 改成xyz
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_normals(
			new pcl::PointCloud<pcl::PointXYZINormal>());
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_normals_temp(
			new pcl::PointCloud<pcl::PointXYZINormal>());
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_target_normals(
			new pcl::PointCloud<pcl::PointXYZINormal>());
	pcl::PointCloud<pcl::PointXYZI> tfed;
	//隔断一下
	pcl::PointCloud<pcl::PointXYZI>::Ptr target1(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::copyPointCloud(target,*target1);
	pcl::PointCloud<pcl::PointXYZI>::Ptr source1(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::copyPointCloud(*source,*source1);
	util tools;
	tools.timeCalcSet("1.计算normal时间");
	addNormal(source1, cloud_source_normals);
	addNormal(target1, cloud_target_normals);
	tools.timeUsed();
	*cloud_source_normals_temp = *cloud_source_normals;
	//0. 当前预测量 = 上次位姿态*增量
	icp_init = transformation * increase;
	
	//去除累计误差
	icp_init = ReOrthogonalization(Eigen::Isometry3d(icp_init.matrix().cast<double>())).matrix().cast<float>();
	//1.转换点云 给一个初值
	pcl::transformPointCloud(*cloud_source_normals_temp, *cloud_source_normals, icp_init.matrix());

	pcl_plane_plane_icp->setInputSource(cloud_source_normals);
	pcl_plane_plane_icp->setInputTarget(cloud_target_normals);
	pcl_plane_plane_icp->align(*cloud_source_normals);
	//todo 感觉不太对,这个increase 在别处不一定准确,还是应该转到上一个的坐标
	//2.当前的transform 全局准确
 	transformation = icp_init * pcl_plane_plane_icp->getFinalTransformation();//上次结果(结果加预测)
	//计算不带increase的increase 1上次位姿 * 预测 * 预测的调整 是错的 应该是 :
	//实际增量 = 上次增量* icp算出的增量误差
	//上面那个也不对
	//increase = transformation * increase * pcl_plane_plane_icp->getFinalTransformation();
	increase = increase * pcl_plane_plane_icp->getFinalTransformation();
/*	std::cout << "1.T_l-1_l - T_l_l+1 : \n" <<   pcl_plane_plane_icp->getFinalTransformation() << std::endl;
	std::cout << "2.T_l_l1 :\n" << increase << std::endl;
	std::cout << "3.第一次分数 : " << pcl_plane_plane_icp->getFitnessScore() << std::endl;*/
	pcl::transformPointCloud(*source, tfed, transformation.matrix());
	//变化量
	return tfed;
}
//旋转normalize
Eigen::Isometry3d registration::ReOrthogonalization(Eigen::Isometry3d input)  {
 
		Eigen::Isometry3d result = Eigen::Isometry3d::Identity() ;
		Eigen::Matrix3d rotation(input.rotation());
		Eigen::Matrix4d se3;
		Eigen::Matrix4d diff;
		Eigen::Quaterniond Quat;
		se3 = input.matrix();
		
		Quat = rotation;
		Quat.normalize();
		//r
		result.setIdentity();
		result.rotate(Quat);
 		//t
		result(0,3) = se3(0,3);
		result(1,3) = se3(1,3);
		result(2,3) = se3(2,3);
	 
		return result;
 
}
//scan 和 map的匹配
pcl::PointCloud<pcl::PointXYZI> registration::normalIcpRegistrationlocal(pcl::PointCloud<pcl::PointXYZI>::Ptr source,
																		 pcl::PointCloud<pcl::PointXYZI> target) {
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_normals(
			new pcl::PointCloud<pcl::PointXYZINormal>());
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_source_normals_temp(
			new pcl::PointCloud<pcl::PointXYZINormal>());
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_target_normals(
			new pcl::PointCloud<pcl::PointXYZINormal>());
	pcl::PointCloud<pcl::PointXYZI> tfed;
	
	Eigen::Matrix4f transformation_local = Eigen::Matrix4f::Identity(); //全局tf
	Eigen::Matrix4f icp_init_local = Eigen::Matrix4f::Identity();//初值
	//隔断一下
	pcl::PointCloud<pcl::PointXYZI>::Ptr target1(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::copyPointCloud(target,*target1);
	util tools;
	
/*	addNormalRadius(source, cloud_source_normals);
	addNormalRadius(target1, cloud_target_normals);*/
	addNormal(source, cloud_source_normals);
	addNormal(target1, cloud_target_normals);//map
	//mls 滤波
	 // Create a KD-Tree
/*	pcl::PointCloud<pcl::PointNormal> mls_points;
  	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
	pcl::MovingLeastSquares<pcl::PointXYZI, pcl::PointNormal> mls;
	mls.setComputeNormals (true);
	// Set parameters
	mls.setInputCloud (target1);
	mls.setPolynomialOrder (2);
	mls.setSearchMethod (tree);
	mls.setSearchRadius (0.03);
	// Reconstruct
	mls.process (mls_points);
	// Save output
	pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);*/
	
	local_map_with_normal = *cloud_target_normals;
	tools.timeUsed();
	
	tools.timeCalcSet("2.1 icp求解时间");
	*cloud_source_normals_temp = *cloud_source_normals;
	//0. 上次位姿态*增量
	icp_init_local = transformation;
	//去除累计误差
	icp_init_local = ReOrthogonalization(Eigen::Isometry3d(icp_init_local.matrix().cast<double>())).matrix().cast<float>();
	//1.转换点云 给一个初值
	pcl::transformPointCloud(*cloud_source_normals_temp, *cloud_source_normals, icp_init_local.matrix());

	pcl_plane_plane_icp->setInputSource(cloud_source_normals);
	pcl_plane_plane_icp->setInputTarget(cloud_target_normals);
	pcl_plane_plane_icp->align(*cloud_source_normals);

	//2.当前的transform 全局准确
	increase = increase * pcl_plane_plane_icp->getFinalTransformation();
	transformation_local = icp_init_local * pcl_plane_plane_icp->getFinalTransformation(); //上次结果(结果加预测)
	transformation = transformation_local;
	tools.timeUsed();
	
/*	std::cout << "2.2 T_scan_l_l+1 - Tmap_l_l+1 \n"  << pcl_plane_plane_icp->getFinalTransformation() << std::endl;
	std::cout << "2.3 第二次分数 : "  << pcl_plane_plane_icp->getFitnessScore()  << std::endl;*/
	pcl::transformPointCloud(*source, tfed, transformation.matrix());
	//变化量
	return tfed;
}

void registration::SetNormalICP(int Correspondence) {
	pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>::Ptr icp(
			new pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>());
	icp->setMaximumIterations(15);
	icp->setMaxCorrespondenceDistance(Correspondence);
	icp->setTransformationEpsilon(0.001);
	icp->setEuclideanFitnessEpsilon(0.001);
	this->pcl_plane_plane_icp = icp;
}

void registration::SetPlaneICP(int Correspondence) {
	pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>::Ptr icp(
			new pcl::IterativeClosestPointWithNormals<pcl::PointXYZINormal, pcl::PointXYZINormal>());
	icp->setMaximumIterations(25);
	icp->setMaxCorrespondenceDistance(Correspondence);
	icp->setTransformationEpsilon(0.001);
	icp->setEuclideanFitnessEpsilon(0.001);
	this->pcl_plane_plane_icp = icp;
}

void registration::addNormalRadius(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
								   pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_normals(
			new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*cloud,*cloud_source_normals);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZ>);
	searchTree->setInputCloud(cloud_source_normals);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimator_pa;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
	bool omp = true;
	if(omp){
		normalEstimator_pa.setInputCloud(cloud_source_normals);
		normalEstimator_pa.setSearchMethod(searchTree);
		normalEstimator_pa.setRadiusSearch(0.15);
		normalEstimator_pa.compute(*normals);
		pcl::concatenateFields(*cloud_source_normals, *normals, *cloud_with_normals);
	}
}

