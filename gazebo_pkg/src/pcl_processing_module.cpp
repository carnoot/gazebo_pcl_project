#include <pcl_processing_module.h>

PclProcesser::PclProcesser(int argc, char **argv) {
	ros::NodeHandle n;

	this->first_max_elements = 5;

	this->get_cloud = n.advertiseService("get_cloud", &PclProcesser::GetCloud,
			this);

	this->camera_depth_points_sub = n.subscribe("/camera/depth/points", 1000,
			&PclProcesser::SaveClouds, this);

	this->get_object_bounding = n.advertiseService("get_object_bounding",
			&PclProcesser::GetObjectBounding, this);

	this->get_cam_quaternion = n.advertiseService("get_cam_quaternion",
			&PclProcesser::GetCamQuaternion, this);

	this->get_correct_indexes = n.advertiseService("get_incorrect_indexes",
			&PclProcesser::GetInCorrectIndexes, this);

	this->can_send_next_pos = n.serviceClient<
			gazebo_pkg::ObjectCanSendNextCamPos>("get_can_send_next_cam_pos");

	this->send_clouds_to_classify = n.serviceClient<
			gazebo_pkg::ObjectInspectionClassifyClouds>(
			"get_clouds_to_classify");

	this->send_final_indexes = n.serviceClient<
			gazebo_pkg::ObjectInspectionFinalCameraPos>("get_final_indexes");

	this->orig_cloud_to_save_base_path = "/home/furdek/final_test/";
	this->final_cloud_to_save_base_path = "/home/furdek/kinect_sim_final";

	this->can_process = false;
	this->erase_bad_classifiers = false;

	this->x_axis_ok = false;
	this->y_axis_ok = false;
	this->z_axis_ok = false;

	this->transform_matrix = Eigen::Matrix4f::Identity();
	this->transform_matrix_axis = Eigen::Matrix4f::Identity();

	this->clouds_processed = 0;
	this->final_cloud_to_save_contor = 0;
	this->get_best_positions = false;

	ROS_INFO("Ready to process clouds!");

}

PclProcesser::~PclProcesser() {

}

bool PclProcesser::GetInCorrectIndexes(
		gazebo_pkg::VFHTestCorrectIndexes::Request &req,
		gazebo_pkg::VFHTestCorrectIndexes::Response &res) {

	this->incorrect_indexes = req.correct_indexes;
	this->erase_bad_classifiers = true;

	return (true);

}

void PclProcesser::EraseBadClassifiers() {

	int number_of_elements_erased = 0;

	std::cerr << "best_positions' size: " << this->best_positions.size()
			<< std::endl;
	std::cerr << "best_positions_indexes' size: "
			<< this->best_positions_indexes.size() << std::endl;
	std::cerr << "incorrect_indexes' size: " << this->incorrect_indexes.size()
			<< std::endl;

	for (size_t i = 0; i < this->incorrect_indexes.size(); i++) {
		std::cerr << "incorrect index: " << this->incorrect_indexes[i]
				<< std::endl;
		this->best_positions.erase(
				this->best_positions.begin() + this->incorrect_indexes[i]
						- number_of_elements_erased);
		this->best_positions_indexes.erase(
				this->best_positions_indexes.begin()
						+ this->incorrect_indexes[i]
						- number_of_elements_erased);
		number_of_elements_erased++;
	}

}

void PclProcesser::SendCloudsToBeClassifed() {

	gazebo_pkg::ObjectInspectionClassifyClouds classify_clouds_srv;
	classify_clouds_srv.request.clouds_to_classify.reserve(
			this->clouds_to_process_vect.size());

	for (size_t i = 0; i < this->clouds_to_process_vect.size(); i++) {
		sensor_msgs::PointCloud2 pointcloud2;
		pcl::toROSMsg(this->clouds_to_process_vect[i], pointcloud2);
		classify_clouds_srv.request.clouds_to_classify.push_back(pointcloud2);
	}

	if (this->send_clouds_to_classify.call(classify_clouds_srv)) {
		ROS_INFO("Clouds to be classified sent successfully!");
	} else {
		ROS_INFO("Clouds to be classified NOT sent!");
	}

}

bool PclProcesser::GetCamQuaternion(
		gazebo_pkg::ObjectInspectionQuaternion::Request &req,
		gazebo_pkg::ObjectInspectionQuaternion::Response &res) {

	gazebo::math::Matrix4 aux_matrix;

	std::cerr << "QUATERNION RECEIVED REQ: " << req.camQuaternion.elems[0]
			<< " " << req.camQuaternion.elems[1] << " "
			<< req.camQuaternion.elems[2] << " " << req.camQuaternion.elems[3]
			<< std::endl;

	this->quaternion_values[0] = req.camQuaternion.elems[0];
	this->quaternion_values[1] = req.camQuaternion.elems[1];
	this->quaternion_values[2] = req.camQuaternion.elems[2];
	this->quaternion_values[3] = req.camQuaternion.elems[3];

	std::cerr << "ARRAY[0,1,2,3]" << this->quaternion_values[0] << " "
			<< this->quaternion_values[1] << " " << this->quaternion_values[2]
			<< " " << this->quaternion_values[3] << std::endl;

	this->cam_quaternion = new gazebo::math::Quaternion(
			this->quaternion_values[0], this->quaternion_values[1],
			this->quaternion_values[2], this->quaternion_values[3]);

	this->aux_quaternion = new gazebo::math::Quaternion(
			this->quaternion_values[0], -this->quaternion_values[1],
			-this->quaternion_values[2], -this->quaternion_values[3]);

	//################### MUKODIK A MEGFORDITAS, FINAL QUAT = (1, 0, 0, 0) ########################//

	*this->cam_quaternion = this->aux_quaternion->operator *(
			*this->cam_quaternion);

	//################### MUKODIK A MEGFORDITAS, FINAL QUAT = (1, 0, 0, 0) ########################

	std::cout << "QUAT FINAL: " << this->cam_quaternion->w << " "
			<< this->cam_quaternion->x << " " << this->cam_quaternion->y << " "
			<< this->cam_quaternion->z << std::endl;

	//################## TRANSFORM QUAT INTO ROTATION MATRIX ####################//
	aux_matrix = this->aux_quaternion->GetAsMatrix4();
	//################## TRANSFORM QUAT INTO ROTATION MATRIX ####################//

	this->transform_matrix(0, 0) = aux_matrix[0][0];
	this->transform_matrix(0, 1) = aux_matrix[0][1];
	this->transform_matrix(0, 2) = aux_matrix[0][2];
	this->transform_matrix(1, 0) = aux_matrix[1][0];
	this->transform_matrix(1, 1) = aux_matrix[1][1];
	this->transform_matrix(1, 2) = aux_matrix[1][2];
	this->transform_matrix(2, 0) = aux_matrix[2][0];
	this->transform_matrix(2, 1) = aux_matrix[2][1];
	this->transform_matrix(2, 2) = aux_matrix[2][2];

//################### ALINIEREA TOTALA A AXELOR PCL CU GAZEBO ##########################
	this->aux_quaternion = new gazebo::math::Quaternion(1.57, 0, 1.57);

	std::cout << "AUX QUAT: " << this->aux_quaternion->w << " "
			<< this->aux_quaternion->x << " " << this->aux_quaternion->y << " "
			<< this->aux_quaternion->z << std::endl;

	*this->cam_quaternion = this->aux_quaternion->operator *(
			*this->cam_quaternion);
	//################### ALINIEREA TOTALA A AXELOR PCL CU GAZEBO ##########################

	//################## TRANSFORM QUAT INTO ROTATION MATRIX ####################//
	aux_matrix = this->aux_quaternion->GetAsMatrix4();
	//################## TRANSFORM QUAT INTO ROTATION MATRIX ####################//

	this->transform_matrix_axis(0, 0) = aux_matrix[0][0];
	this->transform_matrix_axis(0, 1) = aux_matrix[0][1];
	this->transform_matrix_axis(0, 2) = aux_matrix[0][2];
	this->transform_matrix_axis(1, 0) = aux_matrix[1][0];
	this->transform_matrix_axis(1, 1) = aux_matrix[1][1];
	this->transform_matrix_axis(1, 2) = aux_matrix[1][2];
	this->transform_matrix_axis(2, 0) = aux_matrix[2][0];
	this->transform_matrix_axis(2, 1) = aux_matrix[2][1];
	this->transform_matrix_axis(2, 2) = aux_matrix[2][2];

}

bool PclProcesser::GetObjectBounding(
		gazebo_pkg::ObjectInspectionBounding::Request &req,
		gazebo_pkg::ObjectInspectionBounding::Response &res) {

	for (int a = 0; a < 3; a++) {
		this->bounding_min[a] = req.BoundingMin.elems[a];
		this->bounding_max[a] = req.BoundingMax.elems[a];
	}

	std::cerr << "Bounding MIN in ServiceCall from ServiceCall: "
			<< this->bounding_min[0] << " " << this->bounding_min[1] << " "
			<< this->bounding_min[2] << " " << std::endl;
	std::cerr << "Bounding MAX in ServiceCall from ServiceCall: "
			<< this->bounding_max[0] << " " << this->bounding_max[1] << " "
			<< this->bounding_max[2] << " " << std::endl;

	return (true);

}

void PclProcesser::FindMaxElements(int nr_of_max_needed) {

	std::cerr << "Find Max Elements" << std::endl;

	this->best_positions.clear();
	this->best_positions_indexes.clear();

	this->best_positions.reserve(nr_of_max_needed);
	this->best_positions_indexes.reserve(nr_of_max_needed);

	int nr_of_max_found = 0;
	while (nr_of_max_found != nr_of_max_needed) {

		int max = 0;
		int index = 0;

		std::cerr << "result vect size: " << this->result_vect.size()
				<< std::endl;
		std::cerr << "pointcloud vect size: "
				<< this->clouds_to_process_vect.size() << std::endl;

		for (int i = 1; i < this->result_vect.size(); i++) {
			if (this->result_vect[i] >= max) {
				max = this->result_vect[i];
				index = i;
			}
		}
		if (max > 0) {
			std::cerr << "index: " << index << std::endl;
			this->best_positions.push_back(max);
			this->best_positions_indexes.push_back(index);
			std::cerr << "before erasing: " << std::endl;
			this->DisplayResults(this->result_vect);
			this->result_vect.erase(this->result_vect.begin() + index);
			std::cerr << "after erasing: " << std::endl;
			this->DisplayResults(this->result_vect);
			nr_of_max_found++;
		}
	}

	std::cerr << "max elements: ";
	for (int j = 0; j < nr_of_max_found; j++) {
		std::cerr << this->best_positions[j] << " ";
	}

	std::cerr << "best_positions' size: " << this->best_positions.size()
			<< std::endl;
	std::cerr << "best_positions' indexes size: "
			<< this->best_positions_indexes.size() << std::endl;

	std::cerr << "Max Elements Finished!" << std::endl;

}

void PclProcesser::DisplayPoints(pcl::PointCloud<PointType>& cloud_to_display) {

	PointType point;

	std::cerr << "Size of cloud: " << cloud_to_display.size() << std::endl;

	for (int a = 0; a < cloud_to_display.size(); a++) {
		point = cloud_to_display[a];
		if ((!isnan(point.x)) && (!isnan(point.y)) && (!isnan(point.z))) {
			std::cerr << "x: " << point.x << " y: " << point.y << " z: "
					<< point.z << std::endl;
		}

	}
}

void PclProcesser::SaveCloudPCDs() {
	for (int i = 0; i < this->clouds_to_process_vect.size(); i++) {
		std::string updated_path;
		updated_path.append(this->orig_cloud_to_save_base_path);
		updated_path.append(std::to_string(i));
		updated_path.append(".pcd");
		pcl::io::savePCDFile(updated_path, this->clouds_to_process_vect[i]);
	}

	std::cerr << "Clouds saved!" << std::endl;

}

void PclProcesser::CreateFinalCloudVector() {

	std::vector<pcl::PointCloud<PointType>> final_cloud;

	final_cloud.reserve(this->first_max_elements);

	std::cerr << "reserve" << std::endl;
	std::cerr << "clouds_to_process_vect size: "
			<< this->clouds_to_process_vect.size() << std::endl;

	this->DisplayResults(this->best_positions_indexes);

	for (size_t i = 0; i < this->first_max_elements; i++) {
		std::cerr << i << std::endl;
		std::cerr << "best_positions_indexes value: "
				<< this->best_positions_indexes[i] << std::endl;
		final_cloud.push_back(
				this->clouds_to_process_vect[this->best_positions_indexes[i]]);
	}

	this->clouds_to_process_vect = final_cloud;

	std::cerr << "Created Final Cloud Vector!" << std::endl;

}

int PclProcesser::PointsInBoundingBoxManual(
		pcl::PointCloud<PointType> cloud_to_proc) {

	this->x_axis_ok = false;
	this->y_axis_ok = false;
	this->z_axis_ok = false;

	PointType point;
	PointType min_point, max_point;

	int points_inside = 0;
	float correction = 0.02;

	pcl::PointCloud<PointType>::Ptr temp_cloud_PTR(
			new pcl::PointCloud<PointType>(cloud_to_proc));
	pcl::PointCloud<PointType>::Ptr to_remove_radius_cloud(
			new pcl::PointCloud<PointType>);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::getMinMax3D(cloud_to_proc, min_point, max_point);

	std::cerr << "min PointCloud values: " << min_point << std::endl;
	std::cerr << "max PointCloud values: " << max_point << std::endl;

	pcl::transformPointCloud(cloud_to_proc, this->rotated_axis_cloud_to_process,
			this->transform_matrix_axis);

	pcl::getMinMax3D(this->rotated_axis_cloud_to_process, min_point, max_point);

	std::cerr << "min Rotated Axis PointCloud values: " << min_point
			<< std::endl;
	std::cerr << "max Rotated Axis PointCloud values: " << max_point
			<< std::endl;

	pcl::transformPointCloud(this->rotated_axis_cloud_to_process,
			this->rotated_cloud_to_process, this->transform_matrix);

	pcl::getMinMax3D(this->rotated_cloud_to_process, min_point, max_point);

	std::cerr << "min Rotated PointCloud values: " << min_point << std::endl;
	std::cerr << "max Rotated PointCloud values: " << max_point << std::endl;

	std::cerr << this->rotated_cloud_to_process.size() << std::endl;

	for (int i = 0; i < this->rotated_cloud_to_process.size(); i++) {
		point = this->rotated_cloud_to_process[i];

		if ((point.x >= this->bounding_min[0] - correction)
				&& (point.x <= this->bounding_max[0] + correction)) {
			this->x_axis_ok = true;
		}
		if ((point.y >= this->bounding_min[1] - correction)
				&& (point.y <= this->bounding_max[1] + correction)) {
			this->y_axis_ok = true;

		}
		if ((point.z >= this->bounding_min[2] - correction)
				&& (point.z <= this->bounding_max[2] + correction)) {
			this->z_axis_ok = true;
		}

		if (this->x_axis_ok && this->y_axis_ok && this->z_axis_ok) {
			points_inside++;
			inliers->indices.reserve(points_inside);
			inliers->indices.push_back(i);
		}

		this->x_axis_ok = false;
		this->y_axis_ok = false;
		this->z_axis_ok = false;

	}

	pcl::ExtractIndices<PointType> extractor;

	extractor.setInputCloud(temp_cloud_PTR);
	extractor.setIndices(inliers);
	extractor.setNegative(false);
	extractor.filter(*to_remove_radius_cloud);

	if (to_remove_radius_cloud->size()) {
		this->clouds_to_process_vect.push_back(*to_remove_radius_cloud);
		this->final_cloud_to_save_contor++;
		std::cerr << "pushed into clouds_to_process_vect: "
				<< this->final_cloud_to_save_contor << std::endl;
		std::string local_string;
		local_string.append(this->final_cloud_to_save_base_path);
		local_string.append(std::to_string(this->final_cloud_to_save_contor));
		local_string.append(".pcd");
		std::cerr << "path to save to: " << local_string << std::endl;
		pcl::io::savePCDFile(local_string, *to_remove_radius_cloud);
	}

	this->clouds_processed++;
	std::cout << this->clouds_processed << std::endl;
	if (points_inside > 0) {
		this->result_vect.reserve(this->clouds_processed);
		this->result_vect.push_back(points_inside);
	}

	return points_inside;

}

int PclProcesser::PointsInBoundingBoxPcl(pcl::PointCloud<PointType> cloud) {

	pcl::PointCloud<PointType>::Ptr temp_cloud(
			new pcl::PointCloud<PointType>(cloud));

	std::cout << "PointCloud to process size: " << cloud.size() << std::endl;

	pcl::io::savePCDFile("/home/furdek/SIM_CLOUD.pcd", cloud);

//	this->PlaneSegmentationExtraction(temp_cloud);

	Eigen::Vector4f min_pt;
	Eigen::Vector4f max_pt;

	std::cerr << "Bounding MIN in PointsInBoundingBoxPcl from ServiceCall: "
			<< this->bounding_min[0] << " " << this->bounding_min[1] << " "
			<< this->bounding_min[2] << " " << std::endl;
	std::cerr << "Bounding MAX in PointsInBoundingBoxPcl from ServiceCall: "
			<< this->bounding_max[0] << " " << this->bounding_max[1] << " "
			<< this->bounding_max[2] << " " << std::endl;

	min_pt[0] = (float) this->bounding_min[2];
	min_pt[1] = (float) this->bounding_min[1];
	min_pt[2] = (float) this->bounding_min[0];

	max_pt[0] = (float) this->bounding_max[2];
	max_pt[1] = (float) this->bounding_max[1];
	max_pt[2] = (float) this->bounding_max[0];

	std::cerr << "Bounding MIN: " << this->min_pt[0] << " " << this->min_pt[1]
			<< " " << this->min_pt[2] << " " << std::endl;
	std::cerr << "Bounding MAX: " << this->max_pt[0] << " " << this->max_pt[1]
			<< " " << this->max_pt[2] << " " << std::endl;

	std::vector<int> indices;
	std::vector<int>::iterator iter_begin;
	iter_begin = indices.begin();

	std::vector<int>::iterator iter_end;
	iter_end = indices.end();

	pcl::getPointsInBox(cloud, min_pt, max_pt, indices);

	pcl::IndicesPtr pcl_indices(new std::vector<int>(indices));
//	pcl_indices->assign(iter_begin, iter_end);

//	pcl::PointIndicesPtr* pcl_indices = new pcl::PointIndicesPtr();
//	pcl_indices->get()->indices = indices;
//
//	std::cerr << pcl_indices->get()->indices.size() << std::endl;

//	pcl::ExtractIndices<PointType> extractor;
//	extractor.setInputCloud(temp_cloud);
//	extractor.setIndices(pcl_indices);
//	extractor.setNegative(true);
//	extractor.filter(this->cloud_after_processing);

	if (this->cloud_after_processing.size() != 0) {
		std::cout << "Saved FINAL PointCloud" << std::endl;
		pcl::io::savePCDFile("/home/furdek/SIM_FINAL.pcd",
				this->cloud_after_processing);
	}

	return indices.size();

}

void PclProcesser::SaveClouds(
		const sensor_msgs::PointCloud2::ConstPtr &message) {
	pcl::fromROSMsg(*message, this->cloud);
}

void PclProcesser::DisplayResults(std::vector<int> display_vect) {
	for (int i = 0; i < display_vect.size(); i++) {
		std::cerr << display_vect[i] << " ";
	}
	std::cerr << std::endl;
}

bool PclProcesser::GetCloud(gazebo_pkg::ObjectInspectionCloud::Request &req,
		gazebo_pkg::ObjectInspectionCloud::Response &res) {

	std::cout << "GetCloud" << std::endl;

	if (!req.can_get_best_positions) {
		this->cloud_to_process = this->cloud;
		this->get_best_positions = false;
	} else {
		this->get_best_positions = true;
	}
	std::cerr << "get_best_positions set to: " << this->get_best_positions
			<< std::endl;

	this->can_process = true;

	return true;
}

void PclProcesser::PlaneSegmentationExtraction(
		const pcl::PointCloud<PointType>::Ptr cloud) {

	pcl::PointCloud<PointType>::Ptr to_remove_radius_cloud(
			new pcl::PointCloud<PointType>);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<PointType> seg;

	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	pcl::ExtractIndices<PointType> extractor;

	extractor.setInputCloud(cloud);
	extractor.setIndices(inliers);
	extractor.setNegative(true);
	extractor.filter(*to_remove_radius_cloud);

	pcl::io::savePCDFile("/home/furdek/SIM_PLANE.pcd", *to_remove_radius_cloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr to_remove_radius_cloud_rgb(
			new pcl::PointCloud<pcl::PointXYZRGB>(*to_remove_radius_cloud));

	pcl::RadiusOutlierRemoval<PointType> outrem;

	outrem.setInputCloud(to_remove_radius_cloud_rgb);
	outrem.setRadiusSearch(0.05);
	outrem.setMinNeighborsInRadius(30);
	outrem.filter(this->filtered_cloud);

	pcl::io::savePCDFile("/home/furdek/SIM_EXTR.pcd", this->filtered_cloud);

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "pcl_processing");
	PclProcesser processer(argc, argv);

	ros::AsyncSpinner *spinner = new ros::AsyncSpinner(1);
	spinner->start();

	while (ros::ok()) {

		if (processer.can_process) {
			std::cerr << "processing" << std::endl;

			std::cerr
					<< processer.PointsInBoundingBoxManual(
							processer.cloud_to_process) << std::endl;

			if (!processer.get_best_positions) {
				processer.DisplayResults(processer.result_vect);
				gazebo_pkg::ObjectCanSendNextCamPos object_can_send_srv;
				if (processer.can_send_next_pos.call(object_can_send_srv)) {
					ROS_INFO("ObjectCanSendNextCamPos OK!");
				} else {
					ROS_ERROR("ObjectCanSendNextCamPos NOT OK!");
					return 1;
				}
			} else {
				processer.FindMaxElements(processer.first_max_elements);
				std::cerr << "Creating Final Cloud Vector!" << std::endl;
				processer.CreateFinalCloudVector();
				processer.SaveCloudPCDs();
				processer.SendCloudsToBeClassifed();
			}

			processer.can_process = false;
		}

		if (processer.erase_bad_classifiers) {

			processer.DisplayResults(processer.best_positions);
			processer.DisplayResults(processer.best_positions_indexes);

			processer.EraseBadClassifiers();

			processer.DisplayResults(processer.best_positions);
			processer.DisplayResults(processer.best_positions_indexes);

			gazebo_pkg::ObjectInspectionFinalCameraPos final_indexes_srv;
			final_indexes_srv.request.final_indexes = processer.best_positions_indexes;
			if (processer.send_final_indexes.call(final_indexes_srv)){
				ROS_INFO("Final indexes SENT!");
			}
			else
			{
				ROS_INFO("Final indexes NOT SENT!");
			}

			processer.erase_bad_classifiers = false;
		}

	}
	return 0;
}
