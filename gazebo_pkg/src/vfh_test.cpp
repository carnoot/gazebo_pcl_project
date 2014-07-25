#include <vfh_test.h>

VFHTest::VFHTest() {
	ros::NodeHandle n;

	this->test_counter = 1;
	this->vfh_files_found = 0;
//	this->primary_folder_path = "/home/furdek/VFH_data";
	this->extension_to_read = ".pcd";
	this->csv_training_file_name = "/home/furdek/csv_training.txt";
	this->csv_testing_file_name = "/home/furdek/csv_testing.txt";
	this->csv_training_file_1_name = "/home/furdek/csv_training_1.txt";
	this->libSVM_training_file_name = "/home/furdek/libSVM_training.txt";
	this->libSVM_testing_file_name = "/home/furdek/libSVM_testing.txt";
	this->libSVM_model_file_name = "/home/furdek/libSVM_model.txt";
	this->libSVM_svm_train_exe_name = "/home/furdek/svm-train";
	this->libSVM_svm_predict_exe_name = "/home/furdek/svm-predict";
	this->libSVM_results_file_name = "/home/furdek/libSVM_result.txt";

	this->svm_identifier = "test";
	this->svm_type_classifier = "ml_classifiers/NearestNeighborClassifier"; //SVMClassifier NearestNeighborClassifier

	this->create_svm_classifier = n.serviceClient<
			ml_classifiers::CreateClassifier>(
			"/ml_classifiers/create_classifier");

	this->add_svm_class_data = n.serviceClient<ml_classifiers::AddClassData>(
			"/ml_classifiers/add_class_data");

	this->train_svm_classifier =
			n.serviceClient<ml_classifiers::TrainClassifier>(
					"/ml_classifiers/train_classifier");

	this->classify_svm_data = n.serviceClient<ml_classifiers::ClassifyData>(
			"/ml_classifiers/classify_data");

	ROS_INFO("Ready to produce VFHs!");

}

VFHTest::~VFHTest() {

}

void VFHTest::GenerateLibSVMCommand(std::string &executable_path,
		std::string &training_input_path, std::string &output_path) {

	std::string command_string;
	command_string.append(executable_path);
	command_string.append(" \\");
	command_string.append(training_input_path);
	command_string.append(" \\");
	command_string.append(output_path);

	std::system(command_string.c_str());
}

void VFHTest::GenerateLibSVMCommand(std::string &executable_path,
		std::string &testing_input_path, std::string &model_path,
		std::string &output_path) {

	std::string command_string;
	command_string.append(executable_path);
	command_string.append(" \\");
	command_string.append(testing_input_path);
	command_string.append(" \\");
	command_string.append(model_path);
	command_string.append(" \\");
	command_string.append(output_path);

	std::system(command_string.c_str());
}

void VFHTest::CreateFileForLibSVM(std::string &filename_read,
		std::string &filename_write) {

	int line_counter = 1;
	std::ifstream in(filename_read);
	std::ofstream out;

	std::string line;
	std::vector<std::string> separated_line;

	out.open(filename_write, std::ofstream::out | std::ofstream::trunc);

	while (std::getline(in, line) > 0) {

		boost::tokenizer<boost::escaped_list_separator<char> > token(line);
		separated_line.assign(token.begin(), token.end());

		out << separated_line[0];
		out << " ";
		for (int i = 2; i < separated_line.size(); i++) {
			out << i - 1;
			out << ":";
			out << separated_line[i];
			if (i != separated_line.size() - 1)
				out << " ";
		}
		out << "\n";

		line_counter++;

	}

	in.close();
	out.close();

}

int VFHTest::CreateSVMClassifier() {

	std::cerr << "Creating Classifier" << std::endl;

	ml_classifiers::CreateClassifier my_create_classifier;
	my_create_classifier.request.identifier = this->svm_identifier;
	my_create_classifier.request.class_type = this->svm_type_classifier;

	if (this->create_svm_classifier.call(my_create_classifier)) {
		std::cerr << "ML Classifier Created!" << std::endl;
		return 0;
	} else {
		std::cerr << "ML Classifier NOT Created!" << std::endl;
		return -1;
	}

}

int VFHTest::AddSVMClassData() {

	int counter = 0;
	std::cerr << "Adding SVM Class Data" << std::endl;

	ml_classifiers::AddClassData my_add_class_data;
	my_add_class_data.request.identifier = this->svm_identifier;

	for (int b = 0; b < this->data.size(); b++) {
		if (this->data[b].point.size() > 0) {
			my_add_class_data.request.data.reserve(counter + 1);
			my_add_class_data.request.data.push_back(this->data[b]);
			counter++;
		}
	}

	for (int a = 0; a < my_add_class_data.request.data.size(); a++) {
		std::cerr << "a: " << a << " data point size: "
				<< my_add_class_data.request.data[a].point.size() << " Label: "
				<< my_add_class_data.request.data[a].target_class << std::endl;
	}

	std::cerr << "%%%%%%%%%%% " << my_add_class_data.request.data[190].point[0]
			<< std::endl;

	if (this->add_svm_class_data.call(my_add_class_data)) {
		std::cerr << "Adding SVM Class Data SUCCESSFUL!" << std::endl;
		return 0;
	} else {
		std::cerr << "Adding SVM Class Data NOT SUCCESSFUL!" << std::endl;
		return -1;
	}

}

int VFHTest::TrainSVMData() {

	std::cerr << "Training SVM Data" << std::endl;

	ml_classifiers::TrainClassifier my_training_classifier;
	my_training_classifier.request.identifier = this->svm_identifier;

	if (this->train_svm_classifier.call(my_training_classifier)) {
		std::cerr << "TRAINING SUCCESSFULL!" << std::endl;
		return 0;
	} else {
		std::cerr << "TRAINING NOT SUCCESSFULL!" << std::endl;
		return -1;
	}

}

std::vector<std::string> VFHTest::GetNumberOfSVMDataTypes() {

	std::cerr << "get1" << std::endl;

	std::vector<std::string> check_type_vector;

	std::cerr << "SIZE KEZDETBEN: " << check_type_vector.size() << std::endl;

	int number_of_objects = 0;
	std::cerr << "data size: " << this->data.size() << std::endl;
	for (int i = 0; i < this->data.size(); i++) {
//		if (i == 0) {
//
//			check_type_vector.resize(number_of_objects + 1);
//			check_type_vector.push_back(this->data[0].target_class);
//			number_of_objects++;
//
//		}
		if (std::find(check_type_vector.begin(), check_type_vector.end(),
				this->data[i].target_class) == check_type_vector.end()) {
			std::cerr << "Talaltam ujat:" << std::endl;
			check_type_vector.reserve(number_of_objects + 1);
			check_type_vector.push_back(this->data[i].target_class);
			number_of_objects++;
			std::cerr << "Contents: ";
			for (int k = 0; k < check_type_vector.size(); k++) {
				std::cerr << check_type_vector[k] << " ";
			}
			std::cerr << std::endl;
		}

	}

	std::cerr << "get2" << std::endl;
	std::cerr << check_type_vector.size() << std::endl;

	return check_type_vector;
}

void VFHTest::SeparateSVMData(std::vector<std::string> class_types_vector) {

	std::cerr << "class_types_vector size: " << class_types_vector.size()
			<< std::endl;
	this->separated_data.resize(class_types_vector.size());

	int every_separate_data_vector_counter;
	std::vector<ml_classifiers::ClassDataPoint> every_separate_data_vector;

	for (int i = 0; i < class_types_vector.size(); i++) {
		every_separate_data_vector_counter = 0;
		every_separate_data_vector.clear();
		for (int j = 0; j < this->data.size(); j++) {
			if (this->data[j].point.size() != 308) {
				std::cerr << "NEM 308, hanem " << this->data[j].point.size()
						<< std::endl;
				continue;
			}
			std::cerr << "Hasonlitom: " << this->data[j].target_class << " ES "
					<< class_types_vector[i] << std::endl;
			if (this->data[j].target_class == class_types_vector[i]) {
				every_separate_data_vector.reserve(
						every_separate_data_vector_counter + 1);
				every_separate_data_vector.push_back(this->data[j]);
				every_separate_data_vector_counter++;
			}
		}
		std::cerr << "new vector length: " << every_separate_data_vector_counter
				<< std::endl;
		this->separated_data[i] = every_separate_data_vector;
	}

	for (int k = 0; k < this->separated_data.size(); k++) {
		std::cerr << "Nr of elements at position: " << k << " "
				<< this->separated_data[k].size() << std::endl;
	}

}

float VFHTest::CalculatePercentageOfFitness(float number_of_elements,
		std::string class_type, std::vector<std::string> results) {

	std::cout << "PERCENTAGE FOR CLASS TYPE:" << class_type << std::endl;

	float number_found = 0;

	for (int i = 0; i < results.size(); i++) {

		std::cerr << "random results value: " << results[i] << std::endl;

		if (results[i] == class_type) {
			number_found++;
		}

	}
	return ((number_found / number_of_elements) * 100);

}

int VFHTest::ClassifySVMData() {

	int counter = 0;

	std::cerr << "Classifying SVM Input Data" << std::endl;

	ml_classifiers::ClassifyData my_data_to_be_classified;

	std::cerr << "Data size: " << this->data.size() << std::endl;

	my_data_to_be_classified.request.identifier = this->svm_identifier;

	for (int b = 0; b < this->data.size(); b++) {
		if (this->data[b].point.size() != 308)
			continue;
		else {
			std::cerr << counter << " ";
			my_data_to_be_classified.request.data.reserve(counter + 1);
			my_data_to_be_classified.request.data.push_back(this->data[b]);
			counter++;
		}
	}
	std::cerr << std::endl;

//	std::cerr << my_data_to_be_classified.request.data[708].point[0]
//			<< " + Label: "
//			<< my_data_to_be_classified.request.data[708].target_class
//			<< std::endl;

//	std::cerr << my_data_to_be_classified.request.data.size() << std::endl;
//	for (int a = 0; a < my_data_to_be_classified.request.data.size(); a++) {
//		std::cerr << my_data_to_be_classified.request.data[a].point.size()
//				<< std::endl;
//	}

	if (this->classify_svm_data.call(my_data_to_be_classified)) {
		std::cerr << "Classifying SUCCESSFULL!" << std::endl;
		this->result_labels.resize(
				my_data_to_be_classified.response.classifications.size());
		for (int k = 0;
				k < my_data_to_be_classified.response.classifications.size();
				k++) {
			this->result_labels[k] =
					my_data_to_be_classified.response.classifications[k];
		}
		return 0;
	} else {
		std::cerr << "Classifying NOT SUCCESSFULL!" << std::endl;
		return -1;
	}

}

int VFHTest::ClassifySeparateSVMData() {

	std::string type;
	float number_of_elements;
	this->SeparateSVMData(this->GetNumberOfSVMDataTypes());

	std::cerr << "type szerint szam: " << this->separated_data.size()
			<< std::endl;

	for (int i = 0; i < this->separated_data.size(); i++) {
		number_of_elements = 0;
		ml_classifiers::ClassifyData my_data_to_be_classified;
		my_data_to_be_classified.request.identifier = this->svm_identifier;
		my_data_to_be_classified.request.data.reserve(
				this->separated_data[i].size());

		std::cerr << "my_data_to_be data size: "
				<< my_data_to_be_classified.request.data.size() << std::endl;

		for (int j = 0; j < this->separated_data[i].size(); j++) {
			my_data_to_be_classified.request.data.push_back(
					this->separated_data[i][j]);
			this->classPoint = this->separated_data[i][j];
			std::cerr << this->classPoint.target_class;
			number_of_elements++;
		}

		std::cerr << "Calling Classifier" << std::endl;

		if (this->classify_svm_data.call(my_data_to_be_classified)) {
			std::cerr << "Classifying SUCCESSFULL!" << std::endl;
			this->result_labels =
					my_data_to_be_classified.response.classifications;
			std::cerr << "response size: "
					<< my_data_to_be_classified.response.classifications.size()
					<< std::endl;
			std::cerr << "PERCENTAGE: "
					<< this->CalculatePercentageOfFitness(number_of_elements,
							this->classPoint.target_class, this->result_labels)
					<< std::endl;
		} else {
			std::cerr << "Classifying NOT SUCCESSFULL!" << std::endl;
			return -1;
		}

	}

	return 0;
}

bool VFHTest::loadHist(const boost::filesystem::path &path, vfh_model &vfh) {
	int vfh_idx;
	try {
		pcl::PCLPointCloud2 cloud;
		pcl::PointCloud<PointType> cl;
		int version;
		Eigen::Vector4f origin;
		Eigen::Quaternionf orientation;
		pcl::PCDReader r;
		int type;
		unsigned int idx;
		r.readHeader(path.string(), cloud, origin, orientation, version, type,
				idx);

		vfh_idx = pcl::getFieldIndex(cloud, "vfh");
		if (vfh_idx == -1)
			return (false);
		if ((int) cloud.width * cloud.height != 1)
			return (false);
	} catch (pcl::InvalidConversionException &e) {
		std::cerr << e.detailedMessage() << std::endl;
		return (false);
	}

	pcl::PointCloud<pcl::VFHSignature308> point;
	pcl::io::loadPCDFile(path.string(), point);
	vfh.second.resize(308);

	std::vector<pcl::PCLPointField> fields;
	pcl::getFieldIndex(point, "vfh", fields);

	float vfh_max_value = -10000.00;

	for (size_t k = 0; k < fields[vfh_idx].count; k++) {
		if (point.points[0].histogram[k] > vfh_max_value)
			vfh_max_value = point.points[0].histogram[k];
	}

	for (size_t i = 0; i < fields[vfh_idx].count; ++i) {
		vfh.second[i] = point.points[0].histogram[i] / vfh_max_value;
	}

	vfh.first = path.string();
	return (true);
}

void VFHTest::CreateSCVFile(std::string &filename) {

	std::cerr << "model_label_values size: " << this->model_label_values.size()
			<< std::endl;
	std::cerr << "Models size: " << this->models.size() << std::endl;

	std::ofstream file_to_write;
	file_to_write.open(filename, std::ofstream::out | std::ofstream::trunc);

	for (int i = 0; i < this->models.size(); i++) {
		file_to_write << this->model_label_values[i];
		file_to_write << ",";
		file_to_write << this->models[i].second.size();
		file_to_write << ",";
		for (int j = 0; j < this->models[i].second.size(); j++) {
			file_to_write << this->models[i].second[j];
			if (j != this->models[i].second.size() - 1)
				file_to_write << ",";
		}
		file_to_write << "\n";
	}

	file_to_write.close();

}

void VFHTest::LoadSCVFile(std::string &filename) {

	this->data_points.clear();
	this->data_points_labels.clear();
	this->data.clear();

	int line_counter = 0;
	std::ifstream in(filename);
	std::string line;
	std::vector<std::string> separated_line;

//	this->data_points.resize(this->models.size() - 1);
//	this->data_points_labels.resize(this->models.size() - 1);
//	this->data.resize(this->models.size() - 1);

//	for (int a = 0; a < this->data_points.size(); a++){
//	this->data_points[a].resize(this->models[1].second.size());
//	}

	while (std::getline(in, line) > 0) {

		this->data_points.resize(line_counter + 1);
		this->data_points_labels.resize(line_counter + 1);
		this->data.resize(line_counter + 1);

		ml_classifiers::ClassDataPoint my_class_data_point;

		boost::tokenizer<boost::escaped_list_separator<char> > token(line);
		separated_line.assign(token.begin(), token.end());

		this->data_points_labels[line_counter] = separated_line[0];
//		std::cerr << this->data_points_labels[line_counter] << std::endl;

		my_class_data_point.target_class = separated_line[0];

		if (atoi(separated_line[1].c_str()) == 308) {
			this->data_points[line_counter].resize(separated_line.size() - 2);
			for (int b = 2; b < separated_line.size(); b++) {
				this->data_points[line_counter][b - 2] = std::atof(
						separated_line[b].c_str());
			}

			my_class_data_point.point = this->data_points[line_counter];
			std::cerr << my_class_data_point.point.size() << " == "
					<< this->data_points[line_counter].size() << std::endl;

			this->data[line_counter] = my_class_data_point;

			line_counter++;

		} else {
			std::cerr << "LINE CORRUPTED!" << std::endl;
		}

//		my_class_data_point.point = this->data_points[line_counter - 1];
//		this->data.push_back(my_class_data_point);
//		line_counter++;
		std::cerr << "line_counter: " << line_counter << std::endl;
		std::cerr << "this->data size: " << this->data.size() << std::endl;

	}

//	for (int c = 0; c < this->data_points_labels.size(); c++) {
//		std::cerr << this->data_points_labels[c] << std::endl;
//	}

	in.close();

}

pcl::PointCloud<pcl::VFHSignature308>::Ptr VFHTest::ComputeVFH(
		pcl::PointCloud<PointType> cl) {

	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>(cl));
	pcl::PointCloud<pcl::Normal>::Ptr normals(
			new pcl::PointCloud<pcl::Normal>());

	pcl::VFHEstimation<PointType, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud(cloud);
	std::cerr << "Estimating Normals" << std::endl;
	normals = this->normalEstimation(cloud);
	vfh.setInputNormals(normals);

	pcl::search::KdTree<PointType>::Ptr tree(
			new pcl::search::KdTree<PointType>());
	vfh.setSearchMethod(tree);

	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(
			new pcl::PointCloud<pcl::VFHSignature308>());

	std::cerr << "Computing VFH" << std::endl;
	vfh.compute(*vfhs);

	return vfhs;
//	pcl::io::savePCDFile("/home/furdek/vfh.pcd", *vfhs);

}

void VFHTest::SaveVFHS(std::string file_path,
		pcl::PointCloud<pcl::VFHSignature308>::Ptr VFHS) {

	pcl::io::savePCDFile(file_path, *VFHS);

}

bool VFHTest::HasDirectories(const boost::filesystem::path &directory) {

	for (boost::filesystem::directory_iterator it(directory);
			it != boost::filesystem::directory_iterator(); ++it) {

		if (boost::filesystem::is_directory(it->path())) {
			return true;
		}
	}
	return false;

}

int VFHTest::TraverseVFHDirectory(
		const boost::filesystem::path &base_directory) {

	if (!boost::filesystem::exists(base_directory)) {
		std::cerr << "The path specified does not exist!" << std::endl;
		return (-1);
	}

	for (boost::filesystem::directory_iterator it(base_directory);
			it != boost::filesystem::directory_iterator(); ++it) {

//			std::cerr << it->path().string() << std::endl;
		if (boost::filesystem::is_directory(it->path().parent_path())) {
//			std::cerr << it->path().parent_path().string() << " VS: "
//					<< this->primary_folder_path << std::endl;
			if ((it->path().parent_path().string() == this->primary_folder_path)) {
				std::string folder_name;
				folder_name = it->path().string();
				std::cerr << folder_name << std::endl;
				for (int a = 0; a < 7; a++) {
					std::stringstream ss;
					ss << a;
					std::string num_string;
					num_string = ss.str();
					std::cerr << "We are looking for this: " << num_string
							<< std::endl;
					if (folder_name.find(num_string) != std::string::npos) {
						this->label = a;
						std::cerr << "TYPE OF CATEGORY: " << label << std::endl;
						break;
					}
				}
			}
			if (boost::filesystem::is_directory(it->status()))
				this->TraverseVFHDirectory(it->path());
			if (boost::filesystem::is_regular_file(it->status())
					&& boost::filesystem::extension(it->path())
							== this->extension_to_read) {
				vfh_model model;
				this->loadHist(it->path(), model);
				this->vfh_files_found++;

				this->model_label_values.reserve(this->vfh_files_found);
				this->model_label_values.push_back(this->label);

				this->models.reserve(this->vfh_files_found);
				this->models.push_back(model);

			}

		}
	}

	return 0;

}

int VFHTest::TraversePCDDirectory(
		const boost::filesystem::path &base_directory) {

	boost::filesystem::path directory_path;

	if (!boost::filesystem::exists(base_directory)) {
		std::cerr << "The path specified does not exist!" << std::endl;
		return (-1);
	}
	for (boost::filesystem::directory_iterator it(base_directory);
			it != boost::filesystem::directory_iterator(); ++it) {

		std::string exact_path_string;

//		std::cerr << it->path().string() << std::endl;
		std::cerr << it->path().filename() << std::endl;

		if (boost::filesystem::is_directory(it->status())) {
			std::cerr << "Has directories?" << this->HasDirectories(it->path())
					<< std::endl;
			if (!this->HasDirectories(it->path())) {
				std::cerr << "Creating VFH Files!" << std::endl;
				exact_path_string.append(this->primary_folder_path);
				exact_path_string.append("/");
				exact_path_string.append(
						it->path().parent_path().filename().string());
				exact_path_string.append("/");
				exact_path_string.append(it->path().filename().string());
//				exact_path_string.append("/");
				std::cerr << "Going to create at this path: "
						<< exact_path_string << std::endl;
				directory_path = exact_path_string;
				boost::filesystem::create_directories(directory_path);
				this->CreateVFHFiles(it->path(), exact_path_string,
						this->extension_to_read);
			} else {
				this->TraversePCDDirectory(it->path());
			}
		}
	}
}

void VFHTest::CreateVFHFiles(const boost::filesystem::path &directory,
		const std::string &path_to_create, std::string extension) {

	std::cerr << "Inside CreateVFHFiles" << std::endl;

	std::stringstream ss;
	boost::filesystem::path directory_to_create;
	pcl::PointCloud<PointType> pcd_cloud;
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(
			new pcl::PointCloud<pcl::VFHSignature308>());
	pcl::PCDReader reader;
//	ss << this->primary_folder_path;
	ss << path_to_create;
	directory_to_create = ss.str();

	std::cerr << "Directory to create inside: " << directory_to_create.string()
			<< std::endl;
	boost::filesystem::create_directory(directory_to_create);

	std::cerr << "Traversing PCD files!" << std::endl;

	for (boost::filesystem::directory_iterator it(directory);
			it != boost::filesystem::directory_iterator(); ++it) {
		if (boost::filesystem::is_regular_file(it->status())
				&& boost::filesystem::extension(it->path()) == extension) {
			std::stringstream string_stream;
			string_stream << ss.str() << "/";
			string_stream << it->path().filename().string();
			reader.read(it->path().string(), pcd_cloud);
			vfhs = this->ComputeVFH(pcd_cloud);
			std::cerr << "Saving VFH to path: " << string_stream.str()
					<< std::endl;
			this->SaveVFHS(string_stream.str(), vfhs);
		}
	}

}

pcl::PointCloud<pcl::Normal>::Ptr VFHTest::normalEstimation(
		const pcl::PointCloud<PointType>::ConstPtr my_cloud) {

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
			new pcl::PointCloud<pcl::Normal>);

	pcl::NormalEstimation<PointType, pcl::Normal> ne;
	pcl::search::KdTree<PointType>::Ptr tree(
			new pcl::search::KdTree<PointType>());

	ne.setInputCloud(my_cloud);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.03); //#################### IF NOT 0.03 BEHAVES STRANGELY ####################
	ne.compute(*cloud_normals);

	return cloud_normals;

}

void VFHTest::DisplayPoints(pcl::PointCloud<PointType>& cloud_to_display) {

	PointType point;

	std::cerr << "Printing!" << std::endl;
	std::cerr << "Size of cloud: " << cloud_to_display.size() << std::endl;

	for (int a = 0; a < cloud_to_display.size(); a++) {
		point = cloud_to_display[a];
		if ((!isnan(point.x)) && (!isnan(point.y)) && (!isnan(point.z))) {
			std::cerr << "x: " << point.x << " y: " << point.y << " z: "
					<< point.z << std::endl;
		}

	}
}

void VFHTest::SaveClouds(const sensor_msgs::PointCloud2::ConstPtr &message) {

//	std::cout << "In SaveClouds Callback!" << std::endl;
//    this->mut.lock();
	pcl::fromROSMsg(*message, this->cloud);
//    this->mut.unlock();

}

void VFHTest::DisplayResults() {
	for (int i = 0; i < this->result_vect.size(); i++) {
		std::cerr << this->result_vect[i] << " ";
	}
	std::cerr << std::endl;
}

void VFHTest::ProcessResults() {
	std::cerr << "result_labels size: " << this->result_labels.size()
			<< std::endl;
	for (int i = 0; i < this->result_labels.size(); i++) {
		std::cerr << "i: " << i << " volt: " << this->data_points_labels[i]
				<< " most: " << this->result_labels[i] << std::endl;
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "vfh_test");

	if (argc != 5) {
		return 0;
	}

	VFHTest* processer = new VFHTest();

	pcl::PointCloud<PointType>::Ptr initial_cloud(
			new pcl::PointCloud<PointType>);

	boost::filesystem::path main_training_path_to_read_PCD_from;
	boost::filesystem::path main_training_path_to_read_VFH_from;

	boost::filesystem::path main_testing_path_to_read_PCD_from;
	boost::filesystem::path main_testing_path_to_read_VFH_from;

	main_training_path_to_read_PCD_from = argv[1];
	main_training_path_to_read_VFH_from = argv[2];
	main_testing_path_to_read_PCD_from = argv[3];
	main_testing_path_to_read_VFH_from = argv[4];

	ros::AsyncSpinner *spinner = new ros::AsyncSpinner(1);
	spinner->start();

	if (!boost::filesystem::exists(main_training_path_to_read_VFH_from)) {

//		std::cerr << main_training_path_to_read_VFH_from << "Deleted"
//				<< std::endl;
//		boost::filesystem::remove_all(main_training_path_to_read_VFH_from);

		std::cerr << main_training_path_to_read_VFH_from << "JUST Created"
				<< std::endl;
		boost::filesystem::create_directory(
				main_training_path_to_read_VFH_from);
	} else {
		std::cerr << main_training_path_to_read_VFH_from << "ALREADY Created"
				<< std::endl;
		boost::filesystem::create_directory(
				main_training_path_to_read_VFH_from);
	}

	if (!boost::filesystem::exists(main_testing_path_to_read_VFH_from)) {

//		std::cerr << main_testing_path_to_read_VFH_from << "Deleted"
//				<< std::endl;
//		boost::filesystem::remove_all(main_testing_path_to_read_VFH_from);

		std::cerr << main_testing_path_to_read_VFH_from << "JUST Created"
				<< std::endl;
		boost::filesystem::create_directory(main_testing_path_to_read_VFH_from);
	} else {
		std::cerr << main_testing_path_to_read_VFH_from << "ALREADY Created"
				<< std::endl;
		boost::filesystem::create_directory(main_testing_path_to_read_VFH_from);
	}

//	processer->CreateFileForLibSVM(processer->csv_training_file_name, processer->libSVM_training_file_name);
//	processer->CreateFileForLibSVM(processer->csv_testing_file_name, processer->libSVM_testing_file_name);

	processer->GenerateLibSVMCommand(processer->libSVM_svm_train_exe_name,
			processer->libSVM_training_file_name,
			processer->libSVM_model_file_name);

	processer->GenerateLibSVMCommand(processer->libSVM_svm_predict_exe_name,
			processer->libSVM_testing_file_name,
			processer->libSVM_model_file_name,
			processer->libSVM_results_file_name);

//	processer->primary_folder_path =
//			main_training_path_to_read_VFH_from.string();
//	processer->TraversePCDDirectory(main_training_path_to_read_PCD_from);
//	processer->TraverseVFHDirectory(main_training_path_to_read_VFH_from);
//	std::cerr << "Models found: " << processer->models.size() << std::endl;
//	processer->CreateSCVFile(processer->csv_training_file_name);

//	processer->LoadSCVFile(processer->csv_training_file_name);
//
//	processer->CreateSVMClassifier();
//
//	processer->AddSVMClassData();
//
//	processer->TrainSVMData();

//	processer->primary_folder_path =
//			main_testing_path_to_read_VFH_from.string();
//	processer->TraversePCDDirectory(main_testing_path_to_read_PCD_from);
//	processer->TraverseVFHDirectory(main_testing_path_to_read_VFH_from);
//	std::cerr << "Models found: " << processer->models.size() << std::endl;
//	processer->CreateSCVFile(processer->csv_testing_file_name);

//	vfh_model mod;
//	processer->loadHist(
//			"/home/furdek/VFH_training_VFH_data_test/1_SPHERE/obj000_bowl1/bowl1_0000-ascii-smooth.pcd",
//			mod);

//	processer->LoadSCVFile(processer->csv_testing_file_name);
//	processer->ClassifySeparateSVMData();

//	processer->ClassifySVMData();
//	processer->ProcessResults();

	delete processer;

//	pcl::PCDReader reader;
//	reader.read(pcd_path, *initial_cloud);
//	processer.ComputeVFH(*initial_cloud);

	return 0;
}
