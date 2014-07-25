#include "ros/ros.h"
#include "ml_classifiers/CreateClassifier.h"
#include "ml_classifiers/AddClassData.h"
#include "ml_classifiers/TrainClassifier.h"
#include "ml_classifiers/ClassifyData.h"
#include "ml_classifiers/ClassDataPoint.h"
#include <cstdlib>

int main(int argc, char **argv) {

	ros::init(argc, argv, "ml_classifier_client_test");

	ros::NodeHandle n;
	ros::ServiceClient create_svm_classifier;
	ros::ServiceClient add_svm_class_data;
	ros::ServiceClient train_svm_classifier;
	ros::ServiceClient classify_svm_data;

	ml_classifiers::CreateClassifier create_classifier;
	ml_classifiers::AddClassData add_class_data;
	ml_classifiers::TrainClassifier train_classifier;
	ml_classifiers::ClassifyData classifiy_data;
	ml_classifiers::ClassifyData classifiy_data_final;

	create_svm_classifier = n.serviceClient<ml_classifiers::CreateClassifier>(
			"/ml_classifiers/create_classifier");

	add_svm_class_data = n.serviceClient<ml_classifiers::AddClassData>(
			"/ml_classifiers/add_class_data");

	train_svm_classifier = n.serviceClient<ml_classifiers::TrainClassifier>(
			"/ml_classifiers/train_classifier");

	classify_svm_data = n.serviceClient<ml_classifiers::ClassifyData>(
			"/ml_classifiers/classify_data");

	create_classifier.request.identifier = "ml_classifier_test";
	create_classifier.request.class_type = "ml_classifiers/SVMClassifier";

	if (create_svm_classifier.call(create_classifier)) {
		ROS_INFO("Created");
	} else {
		ROS_ERROR("NOT Created!");
		return -1;
	}

	add_class_data.request.identifier = "ml_classifier_test";
//	add_class_data.request.data.resize(5);

	ml_classifiers::ClassDataPoint point;
	point.target_class = "1";
	point.point.resize(2);

//	point.point[0] = 0.1;
//	point.point[1] = 0.2;

	point.point.push_back(0.1);
	point.point.push_back(0.2);

	std::cerr << "point size: " << point.point.size() << std::endl;

	add_class_data.request.data.push_back(point);

	ml_classifiers::ClassDataPoint point1;
	point1.target_class = "1";
	point1.point.resize(2);

//	point1.point[0] = 0.3;
//	point1.point[1] = 0.1;

	point1.point.push_back(0.3);
	point1.point.push_back(0.1);

	std::cerr << "point1 size: " << point1.point.size() << std::endl;
	add_class_data.request.data.push_back(point1);

	ml_classifiers::ClassDataPoint point2;
	point2.target_class = "2";
	point2.point.resize(2);
	point2.point[0] = 3.1;
	point2.point[1] = 3.2;
//	point2.point.push_back(3.1);
//	point2.point.push_back(3.2);
	add_class_data.request.data.push_back(point2);

	ml_classifiers::ClassDataPoint point3;
	point3.target_class = "2";
	point3.point.resize(2);
	point3.point[0] = 3.3;
	point3.point[1] = 4.1;
//	point3.point.push_back(3.3);
//	point3.point.push_back(4.1);
	add_class_data.request.data.push_back(point3);

	ml_classifiers::ClassDataPoint point4;
	point4.target_class = "3";
	point4.point.resize(2);
	point4.point[0] = 5.1;
	point4.point[1] = 5.2;
//	point4.point.push_back(5.1);
//	point4.point.push_back(5.2);
	add_class_data.request.data.push_back(point4);

	ml_classifiers::ClassDataPoint point5;
	point5.target_class = "3";
	point5.point.resize(2);
	point5.point[0] = 5.5;
	point5.point[1] = 5.5;
	//	point4.point.push_back(5.5);
	//	point4.point.push_back(5.5);
	add_class_data.request.data.push_back(point5);

	std::cerr << "add_class_data size: " << add_class_data.request.data.size()
			<< std::endl;

	if (add_svm_class_data.call(add_class_data)) {
		ROS_INFO("Added Points!");
	} else {
		ROS_ERROR("Points NOT Added!");
	}

	train_classifier.request.identifier = "ml_classifier_test";

	if (train_svm_classifier.call(train_classifier)) {
		ROS_INFO("Trained Points!");
	} else {
		ROS_ERROR("Points NOT Trained!");
	}

	classifiy_data_final.request.identifier = "ml_classifier_test";
	classifiy_data_final.request.data.reserve(3);

	std::cerr << "classify_data_final capacity: "
			<< classifiy_data_final.request.data.capacity() << std::endl;
	std::cerr << "classify_data_final size: "
			<< classifiy_data_final.request.data.size() << std::endl;

	classifiy_data.request.identifier = "ml_classifier_test";
//	classifiy_data.request.data.resize(3);

	ml_classifiers::ClassDataPoint test_point;
	test_point.point.reserve(2);
//	test_point.point[0] = 0.0;
//	test_point.point[1] = 0.0;
	test_point.point.push_back(0);
	test_point.point.push_back(0);
	classifiy_data.request.data.push_back(test_point);

	ml_classifiers::ClassDataPoint test_point1;
	test_point1.point.reserve(2);
//	test_point1.point[0] = 5.5;
//	test_point1.point[1] = 5.5;
	test_point1.point.push_back(5.5);
	test_point1.point.push_back(5.5);
	classifiy_data.request.data.push_back(test_point1);

	ml_classifiers::ClassDataPoint test_point2;
	test_point2.point.reserve(2);
//	test_point2.point[0] = 2.9;
//	test_point2.point[1] = 3.6;
	test_point2.point.push_back(2.9);
	test_point2.point.push_back(3.6);
	classifiy_data.request.data.push_back(test_point2);

	for (int a = 0; a < classifiy_data.request.data.size(); a++) {
		classifiy_data_final.request.data.push_back(
				classifiy_data.request.data[a]);
	}

	std::cerr << "classify data final size: "
			<< classifiy_data_final.request.data.size() << std::endl;

	for (int a = 0; a < classifiy_data_final.request.data.size(); a++) {
		std::cerr << "classify_data_final_rendre size: "
				<< classifiy_data_final.request.data[a].point.size()
				<< std::endl;
		for (int b = 0; b < classifiy_data_final.request.data[a].point.size();
				b++) {
			std::cerr << classifiy_data_final.request.data[a].point[b] << " ";
		}
		std::cerr << std::endl;
	}

	std::cerr << "classify_data_final capacity: "
			<< classifiy_data_final.request.data.capacity() << std::endl;
	std::cerr << "classify_data_final size: "
			<< classifiy_data_final.request.data.size() << std::endl;

	if (classify_svm_data.call(classifiy_data_final)) {
		ROS_INFO("Classified Points!");
		for (int i = 0;
				i < classifiy_data_final.response.classifications.size(); i++) {
			std::cerr << classifiy_data_final.response.classifications[i]
					<< std::endl;
		}
	} else {
		ROS_ERROR("Points NOT Classified!");
	}

	return 0;
}
