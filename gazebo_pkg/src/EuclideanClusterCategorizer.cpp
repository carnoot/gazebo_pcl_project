#include <gazebo_pkg/EuclideanClusterCategorizer.h>
//#include <EuclideanClusterCategorizer.h>
#include "/home/furdek/catkin_ws/src/gazebo_pkg/include/stlWriter.h"

void save_proba(const pcl::PointCloud<PointT>::Ptr cloud) {

	std::cout << "Start estimating normals, cloud size:" << cloud->size()
			<< std::endl;

	pcl::search::KdTree<PointT>::Ptr tree_mls(new pcl::search::KdTree<PointT>);

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	normals = normalEstimation(cloud);

	pcl::PointCloud<PointTNormal>::Ptr cloud_with_normals(
			new pcl::PointCloud<PointTNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	std::cout << "Start creating mesh:" << std::endl;

	pcl::search::KdTree<PointTNormal>::Ptr tree2(
			new pcl::search::KdTree<PointTNormal>);
	tree2->setInputCloud(cloud_with_normals);

	pcl::GreedyProjectionTriangulation<PointTNormal> gp3;
	pcl::PolygonMesh triangles;

	gp3.setSearchRadius(0.003);
	gp3.setMu(1.0);
	gp3.setMaximumNearestNeighbors(100);
//       gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
//       gp3.setMinimumAngle(M_PI/18); // 10 degrees
//       gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//       gp3.setNormalConsistency(false);

	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	std::cerr << triangles.cloud.data.size() << std::endl;

	pcl::io::savePolygonFile("/home/furdek/catkin_ws/save_proba_mesh.obj",
			triangles);
	pcl::io::saveVTKFile("/home/furdek/catkin_ws/save_proba.vtk", triangles);
//    pcl::io::savePolygonFileVTK("/home/furdek/catkin_ws/save_proba.vtk", triangles);
	pcl::io::savePolygonFileSTL("/home/furdek/catkin_ws/save_proba.stl",
			triangles);

	vtkSmartPointer<vtkPolyData> output = vtkSmartPointer<vtkPolyData>::New();
	pcl::VTKUtils::convertToVTK(triangles, output);
	output->Update();

	vtkSmartPointer<vtkPolyData> poly_data_1 =
			vtkSmartPointer<vtkPolyData>::New();

	poly_data_1->DeepCopy(output);

	vtkSmartPointer<vtkSTLWriter> poly_writer =
			vtkSmartPointer<vtkSTLWriter>::New();
	poly_writer->SetFileTypeToASCII();
	poly_writer->SetInput(poly_data_1);
	poly_writer->SetFileName("/home/furdek/catkin_ws/data.stl");
	poly_writer->Write();

}

void save_pcl_VTK(const pcl::PointCloud<PointT>::Ptr cloud,
		const std::string str) {

	std::cout << "Start estimating normals, cloud size:" << cloud->size()
			<< std::endl;
	// Create a KD-Tree
	pcl::search::KdTree<PointT>::Ptr tree_mls(new pcl::search::KdTree<PointT>);

	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<PointTNormal>::Ptr mls_points(
			new pcl::PointCloud<PointTNormal>);

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	normals = normalEstimation(cloud);

	pcl::PointCloud<PointTNormal>::Ptr cloud_with_normals(
			new pcl::PointCloud<PointTNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	std::cout << "Start creating mesh:" << std::endl;

	pcl::search::KdTree<PointTNormal>::Ptr tree2(
			new pcl::search::KdTree<PointTNormal>);
	tree2->setInputCloud(cloud_with_normals);

	pcl::GreedyProjectionTriangulation<PointTNormal> gp3;
	pcl::PolygonMesh triangles;

	std::cerr << "Initialized GreedyProjectionTriangulation!" << std::endl;

	gp3.setSearchRadius(0.003);

	gp3.setMu(1.0);
	gp3.setMaximumNearestNeighbors(100);
//       gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
//       gp3.setMinimumAngle(M_PI/18); // 10 degrees
//       gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//       gp3.setNormalConsistency(false);

	std::cerr << "SET parameters for Greedy!" << std::endl;

	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);

	std::cerr << "BEFORE RECONSTRUCT!" << std::endl;

	gp3.reconstruct(triangles);

	std::cerr << "RESULTS FROM GP3!" << std::endl;
	std::cerr << "BEFORE additional VERTEX information!" << std::endl;

	std::cerr << triangles.cloud.data.size() << std::endl;

	vtkSmartPointer<vtkPolyData> poly_data =
			vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> poly_data_1 =
			vtkSmartPointer<vtkPolyData>::New();

	std::string file_name;
	file_name = "/home/furdek/catkin_ws/RaduSTL.stl";

	pcl::io::mesh2vtk(triangles, poly_data);
//       poly_data->Update();
	poly_data_1->DeepCopy(poly_data);

	vtkSmartPointer<vtkSTLWriter> poly_writer =
			vtkSmartPointer<vtkSTLWriter>::New();
	poly_writer->SetFileTypeToASCII();
	poly_writer->SetInput(poly_data_1);
	poly_writer->SetFileName(file_name.c_str());
	poly_writer->Write();

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
	pcl::io::saveVTKFile("mesh.vtk", triangles);

	pcl::PolygonMesh mesh_vtk;
	pcl::io::loadPolygonFileVTK("mesh.vtk", mesh_vtk);
	pcl::io::savePolygonFileSTL("smth.stl", mesh_vtk);

	std::cerr << "STEFAN" << std::endl;

	pcl::PCLPointCloud2 cloud_to_write;
	pcl::toPCLPointCloud2(*cloud, cloud_to_write);
	std::cout << "Saving VTK File to " << str << std::endl;
	pcl::io::saveVTKFile(str, cloud_to_write);
	pcl::PolygonMesh my_mesh;
	pcl::io::loadPolygonFileVTK(str, my_mesh);
	std::cout << "MESH POINTS READ FROM VTK FILE: " << my_mesh.cloud.data.size()
			<< std::endl;
	pcl::io::savePolygonFileSTL("/home/furdek/MYSTL.stl", my_mesh);
}

void save_VTK_to_binary_STL(const std::string path_to_read_from,
		const std::string path_to_write_to) {

	vtkSmartPointer<vtkPolyData> output = vtkSmartPointer<vtkPolyData>::New();

	vtkSmartPointer<vtkInformation> vtk_info =
			vtkSmartPointer<vtkInformation>::New();
	vtkSmartPointer<vtkPolyData> poly_data =
			vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkDataObject> data_object =
			vtkSmartPointer<vtkDataObject>::New();

//    vtkOBJReader* vtk_reader = vtkSmartPointer<vtkOBJReader>::New();
//    vtk_reader->SetFileName(path_to_read_from.c_str());
//    vtk_reader->Update();

//    vtkTriangleFilter *triangles = vtkTriangleFilter::New();
//    triangles->SetInputConnection(vtk_reader->GetOutputPort());
//    triangles->Update();

// ########################### READS VTK FILE WELL ########################## //
	vtkSmartPointer<vtkGenericDataObjectReader> vtk_reader = vtkSmartPointer<
			vtkGenericDataObjectReader>::New(); //vtkSTLReader
	vtk_reader->SetFileName(path_to_read_from.c_str());
	vtk_reader->Update();
	if (vtk_reader->IsFilePolyData()) {
		std::cerr << "output IS a polydata" << std::endl;
		output = vtk_reader->GetPolyDataOutput();
		std::cout << "output has " << output->GetNumberOfPoints() << " points."
				<< std::endl;
	}
	// ########################### READS VTK FILE WELL ########################## //

	pcl::PolygonMesh mesh;
	pcl::VTKUtils::vtk2mesh(output, mesh);
	std::cerr << "MESH SIZE: " << mesh.cloud.data.size() << std::endl;
//    pcl::io::saveVTKFile(path_to_write_to, mesh);
	pcl::io::savePolygonFileSTL(path_to_write_to, mesh);

	std::cerr << "Transformed PolyData to PolygonMesh" << std::endl;

//    vtk_info = vtk_reader->GetInformation();
//    std::cerr << "MEMORY BEFORE: " << poly_data->GetActualMemorySize() << std::endl;
//    vtkPolyData* polyData = vtkPolyData::SafeDownCast(vtk_info->Get(vtkDataObject::DATA_OBJECT()));
//    std::cerr << "NUMBER OF POINTS IN POLY DATA: " << polyData->GetNumberOfPoints() << std::endl;
//    std::cerr << "MEMORY AFTER: " << poly_data->GetActualMemorySize() << std::endl;
//    poly_data = vtkPolyData::SafeDownCast(vtk_info->Get(vtkDataObject::DATA_OBJECT()));
//    std::cerr << poly_data->GetPolys()->GetNumberOfCells() << std::endl;

	mySTLWriter* my_writer = mySTLWriter::New();
	my_writer->SetFileName(path_to_write_to.c_str());
	my_writer->WriteAsciiSTL(output->GetPoints(), output->GetPolys());

//    vtkSmartPointer<vtkSTLWriter> stl_writer = vtkSmartPointer<vtkSTLWriter>::New();
//    stl_writer->SetFileName(path_to_write_to.c_str());
//    stl_writer->SetFileTypeToBinary();
//    stl_writer->SetInput(output);
//    stl_writer->SetInput(vtk_reader->GetOutputDataObject(0));
//    stl_writer->SetInputConnection(vtk_reader->GetOutputPort());
//    stl_writer->SetInput(vtk_reader->GetOutputDataObject(0));

	std::cout << "Written STL file to " << path_to_write_to << std::endl;
}

std::vector<pcl::PointCloud<PointT> > create_separate_planes(
		const pcl::PointCloud<PointT>::Ptr cloud) {

	std::cerr << "CREATING SEPARATE PLANES" << std::endl;

	std::vector<pcl::PointCloud<PointT> > plane_vector;

	plane_vector.resize(regions.size());

	for (size_t i = 0; i < regions.size(); i++) {
		pcl::PointIndices myInd;
		myInd = inlier_indices[i];
		std::cerr << "INSIDE FOR INDICE SIZE: " << myInd.indices.size()
				<< std::endl;
		pcl::copyPointCloud<PointT>(*cloud, myInd.indices, plane_vector[i]);
		pcl::PCDWriter writer;
		std::stringstream ss;
		std::string initstring = "/home/furdek/SEPARATE_PLANES";
		ss << initstring << i << ".pcd";
		std::cerr << "Writing SEPARATE PLANE PCD's to: " << ss.str()
				<< std::endl;
		writer.write(ss.str(), plane_vector[i]);
	}

	std::cerr << "PLANE SIZES CREATE SEPARATE PLANES!" << std::endl;

	for (int j = 0; j < plane_vector.size(); j++) {
		pcl::PointCloud<PointT> my_cloud;
		my_cloud = plane_vector[j];
		std::cerr << my_cloud.size() << std::endl;
	}

	return plane_vector;

}

void choose_plane_with_objects(
		std::vector<pcl::PointCloud<PointT> > plane_vector) {

	double hole_min_treshold = 0.08; // DISCUTABIL
	double hole_max_treshold = 0.23; // DISCUTABIL
	double holes = 0;
	std::vector<double> hole_vector;
	hole_vector.resize(plane_vector.size());

	for (size_t m = 0; m < plane_vector.size(); m++) {
		pcl::PointCloud<PointT> cloud_plane;
		cloud_plane = plane_vector[m];
		for (size_t n = 1; n < cloud_plane.size(); n++) {
			double n_x;
			double n1_x;
			n_x = cloud_plane.at(n).x;
			n1_x = cloud_plane.at(n - 1).x;

			if ((fabs(n_x) - fabs(n1_x) > hole_min_treshold)
					&& (fabs(n_x) - fabs(n1_x) < hole_max_treshold)) {
				holes++;
			}
		}
		hole_vector[m] = holes;
		std::cerr << "Nr. of HOLES in PLANE " << m << ": " << holes
				<< std::endl;
		holes = 0;
	}

	std::vector<double>::iterator it = std::max_element(hole_vector.begin(),
			hole_vector.end());
	std::cerr << "MAXIMUM OF HOLE VECTOR: " << *it << std::endl;
	std::cerr << "POSITION OF MIN HOLE IN VECTOR: " << it - hole_vector.begin()
			<< std::endl;

	main_plane_index = it - hole_vector.begin();
	pcl::copyPointCloud(plane_vector[it - hole_vector.begin()], *plane_cloud);

	pcl::PCDWriter writer;
	std::cerr << "Writing main Plane to /home/furdek/MYPLANE.pcd" << std::endl;
	writer.write("/home/furdek/MYPLANE.pcd", *plane_cloud);

	outlier_removal_from_plane(plane_cloud);

}

void extractPlaneFromProjectedCloud(
		const pcl::PointCloud<PointT>::ConstPtr cloud) {

	pcl::PointCloud<PointT>::Ptr cloud_p(new pcl::PointCloud<PointT>());

	find3DPlaneMinMax(plane_cloud);
//    find3DPlaneMinMax(cloud_projected);

	int points_altered = 0;

	float plus_minus_z = 0.05;
	float plus_minus_x = 0.2;

	std::cout
			<< "Writing Cloud From Which to Extract to /home/furdek/CloudFromWhichToExtract.pcd"
			<< std::endl;

	pcl::io::savePCDFile("/home/furdek/CloudFromWhichToExtract.pcd", *cloud);
	pcl::ExtractIndices<PointT> extractor;

	PointT loc_point;

	std::cerr << "EXTRACTOR BEGIN" << std::endl;

	extractor.setInputCloud(cloud);
	extractor.setIndices(basePlaneIndices);
	extractor.setNegative(true);
	extractor.filter(*cloud_p);

	std::cerr << "EXTRACTOR END" << std::endl;

	std::cerr << "Extracted Plane from Projected Cloud points size: "
			<< cloud_p->points.size() << std::endl;
	std::cerr << "Extracted Plane from Projected Cloud Organized? "
			<< cloud_p->isOrganized() << std::endl;

	for (size_t m = 0; m < cloud_p->points.size(); m++) {
		loc_point = cloud_p->points[m];

//		if (!isnan(loc_point.x)) {
//            if ((loc_point.z < table_z_min
//                    || loc_point.z > table_z_max)
//                    && (loc_point.x < table_x_min
//                    || loc_point.x > table_x_max)
//                    && (loc_point.y < table_y_min
//                    || loc_point.y > table_y_max)) {

		if (loc_point.z < table_z_min + plus_minus_z
				|| loc_point.z > table_z_max - plus_minus_z
				|| loc_point.x < table_x_min + plus_minus_x
				|| loc_point.x > table_x_max - plus_minus_x
				|| loc_point.y < table_y_min || loc_point.y > table_y_max) {

			points_altered++;

			cloud_p->points[m].x = NAN;
			cloud_p->points[m].y = NAN;
			cloud_p->points[m].z = NAN;

		}
	}
//	}

	std::cerr << "Points altered: " << points_altered << std::endl;

	std::cout
			<< "Writing PCD Projection Without Plane to /home/furdek/ProjectionProbaWithoutPlane.pcd"
			<< std::endl;

	pcl::io::savePCDFile("/home/furdek/ProjectionProbaWithoutPlane.pcd",
			*cloud_p);

}

std::vector<pcl::PointCloud<PointT> > separate_2d_clusters(
		const pcl::PointCloud<PointT>::Ptr cloud) {

//    outlier_removal_from_projections(cloud);

//    std::cout
//            << "Writing Final_2D_Cluster PCD to /home/furdek/clusters_2d_final.pcd"
//            << std::endl;
//    pcl::io::savePCDFile("/home/furdek/clusters_2d_final.pcd", *cloud);

	std::string cluster_2d;

	cluster_2d = "/home/furdek/clusters_2d_";

	std::vector<pcl::PointCloud<PointT> > clusters_2d;
	pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<
			pcl::search::Search<PointT> >(new pcl::search::KdTree<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_2d_cluster_normals(
			new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr fasz_cluster(new pcl::PointCloud<PointT>);

	cloud_2d_cluster_normals = normalEstimation(cloud);

	pcl::RegionGrowing<PointT, pcl::Normal> reg;

	reg.setMinClusterSize(500);
	reg.setMaxClusterSize(5000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(50);
	reg.setInputCloud(cloud);
	reg.setInputNormals(cloud_2d_cluster_normals);
//    reg.setResidualThreshold(0.05);
//    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
//    reg.setCurvatureThreshold(1.0);

	std::vector<pcl::PointIndices> clusters;
	reg.extract(clusters);

	clusters_2d.resize(clusters.size() - 1);

	std::cout << "Number of clusters is equal to " << clusters.size()
			<< std::endl;
	for (int i = 0; i < clusters.size() - 1; i++) {
		pcl::PointIndices::Ptr fasz(new pcl::PointIndices);
		*fasz = clusters[i];
		std::cout << fasz->indices.size() << std::endl;

		pcl::ExtractIndices<PointT> extractor;
		extractor.setInputCloud(cloud);
		extractor.setIndices(fasz);
		//    extractor.setNegative(true);
		extractor.filter(*fasz_cluster);
//        clusters_2d.push_back(*fasz_cluster);
		clusters_2d[i] = *fasz_cluster;
		std::stringstream ss;
		ss << cluster_2d << i << ".pcd";
		std::cerr << "Writing 2D_CLUSTER PCD's to " << cluster_2d << std::endl;
		pcl::io::savePCDFile(ss.str(), *fasz_cluster);

	}

	return clusters_2d;

}

void outlier_removal_from_plane(const pcl::PointCloud<PointT>::Ptr cloud) {

	pcl::StatisticalOutlierRemoval<PointT> sor;
	pcl::PCDWriter writer;

	sor.setInputCloud(cloud);
	sor.setMeanK(1000); //250 eleg jo
	sor.setStddevMulThresh(2);
	sor.filter(*cloud);

	std::cout
			<< "Writing PCD after Statistical Outlier Removal to /home/furdek/PLANEAfterOutlier.pcd"
			<< std::endl;
	writer.write("/home/furdek/PLANEAfterOutlier.pcd", *cloud);
}

void outlier_removal_from_projections(
		const pcl::PointCloud<PointT>::Ptr cloud) {

	pcl::StatisticalOutlierRemoval<PointT> sor;
	pcl::PCDWriter writer;

	sor.setInputCloud(cloud);
	sor.setMeanK(500);
	sor.setStddevMulThresh(0.1);
	sor.filter(*cloud);

	std::cout << "Writing PCD after Statistical Outlier Removal!" << std::endl;
	writer.write("/home/furdek/AfterOutlier.pcd", *cloud);
}

void ransac_circle_fitting(std::vector<pcl::PointCloud<PointT> > cluster_vect) {

	for (int a = 0; a < cluster_vect.size(); a++) {

		std::cerr << "RANSAC CIRCLE FITTING: a = " << a << std::endl;

		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

		*cloud = cluster_vect[a];

		std::cerr << "Original Cloud Point Size: " << cloud->points.size()
				<< std::endl;

		outlier_removal_from_projections(cloud);

		std::cerr << "Points remaining after outlier removal: "
				<< cloud->points.size() << std::endl;

		pcl::SampleConsensusModelCircle2D<PointT>::Ptr circle_model(
				new pcl::SampleConsensusModelCircle2D<PointT>(cloud));

		std::cout << "MIN_RADIUS: " << min_radius << std::endl;
		circle_model->setRadiusLimits(min_radius, max_radius);

		pcl::RandomSampleConsensus<PointT> ransac(circle_model);

		pcl::PointCloud<PointT>::Ptr circle_cloud(
				new pcl::PointCloud<PointT>());
		std::vector<int> ransac_circle_inliers;

		std::cout << "Circle Inlier Size BEFORE: "
				<< ransac_circle_inliers.size() << std::endl;

		ransac.setDistanceThreshold(0.001);
		ransac.computeModel();
		ransac.getInliers(ransac_circle_inliers);

		std::cout << "Circle Inlier Size AFTER: "
				<< ransac_circle_inliers.size() << std::endl;

		pcl::copyPointCloud<PointT>(*cloud, ransac_circle_inliers,
				*circle_cloud);

		std::stringstream ss;
		ss << "/home/furdek/CIRCLES" << a << ".pcd";

		if (ransac_circle_inliers.size()) {
			std::cout << "Writing CIRCLES Cloud to:" << ss.str() << std::endl;
			pcl::PCDWriter writer;
			writer.write(ss.str(), *circle_cloud);
		} else {
			std::cerr << "Could not find any CIRCLES!" << std::endl;
		}
	}
}

void ransac_line_fitting(std::vector<pcl::PointCloud<PointT> > cluster_vect) {

	for (int a = 0; a < cluster_vect.size(); a++) {

		std::cerr << "RANSAC LINE FITTING: a = " << a << std::endl;

		pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

		*cloud = cluster_vect[a];

		std::cerr << "Original Cloud Point Size: " << cloud->points.size()
				<< std::endl;

		outlier_removal_from_projections(cloud);

		std::cerr << "Points remaining after outlier removal: "
				<< cloud->points.size() << std::endl;

		pcl::SampleConsensusModelLine<PointT>::Ptr line_model(
				new pcl::SampleConsensusModelLine<PointT>(cloud));

		pcl::RandomSampleConsensus<PointT> ransac(line_model);
		pcl::PointCloud<PointT>::Ptr line_cloud(new pcl::PointCloud<PointT>());

		std::vector<int> ransac_line_inliers;

		ransac.setDistanceThreshold(0.01);
		ransac.computeModel();
		ransac.getInliers(ransac_line_inliers);

		std::cout << "LINE Inlier Size: " << ransac_line_inliers.size()
				<< std::endl;

		pcl::copyPointCloud<PointT>(*cloud, ransac_line_inliers, *line_cloud);

		std::stringstream ss;
		ss << "/home/furdek/LINES" << a << ".pcd";

		if (ransac_line_inliers.size()) {
			std::cout << "Writing LINES Cloud to:" << ss.str() << std::endl;
			pcl::PCDWriter writer;
			writer.write(ss.str(), *line_cloud);
		} else {
			std::cerr << "Could not find any LINES!" << std::endl;

		}
	}
}

void openViewer(pcl::PointCloud<PointT>::Ptr cloud,
		const pcl::PointCloud<pcl::Normal>::ConstPtr cloud_norm) {

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(
			cloud);
	viewer->addPointCloudNormals<PointT, pcl::Normal>(cloud,
			cloud_norm);
	viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0);
	viewer->initCameraParameters();

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

pcl::PointCloud<pcl::Normal>::Ptr normalEstimation(
		const pcl::PointCloud<PointT>::ConstPtr my_cloud) {

	std::string normal_cloud_path;
	normal_cloud_path = "/home/furdek/Normals.pcd";

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
			new pcl::PointCloud<pcl::Normal>);

	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	ne.setInputCloud(my_cloud);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.03); //#################### IF NOT 0.03 BEHAVES STRANGELY ####################
	ne.compute(*cloud_normals);

	openViewer(pcl_current, cloud_normals);

	std::cerr << "Saving Normal Cloud to: " << normal_cloud_path << std::endl;
	pcl::io::savePCDFile(normal_cloud_path, *cloud_normals);

	return (cloud_normals);

}

void display_point_indices_vector_size(std::vector<pcl::PointIndices>& vect_to_display){

	for (size_t i = 0; i < vect_to_display.size(); i++){
		std::cerr << "LABEL:  " << vect_to_display[i].indices.size() << std::endl;
	}

}

void display_point_indices_vector_size_greater_than(std::vector<pcl::PointIndices>& vect_to_display, int lower_limit){

	for (size_t i = 0; i < vect_to_display.size(); i++){
		if (vect_to_display[i].indices.size() > lower_limit)
		std::cerr << "LABEL:  " << vect_to_display[i].indices.size() << std::endl;
	}

}

int point_indices_vector_useful_average_value(std::vector<pcl::PointIndices>& vect_to_display){

	double sum = 0;
	double correct_size = 0;

	for (size_t i = 0; i < vect_to_display.size(); i++){
		if (vect_to_display[i].indices.size() > 3000){
		sum += vect_to_display[i].indices.size();
		correct_size++;
		}
	}

	return ((int)(sum / correct_size));

}

void segmentPlanes(const pcl::PointCloud<PointT>::Ptr my_cloud,
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals) {

	pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> plane_segmenter;
	std::vector<pcl::PointCloud<PointT> > plane_vector;
	pcl::PointIndices loc_indices;
	pcl::PointIndices local_point_indices;
	pcl::ModelCoefficients my_coef;
	pcl::PCDWriter writer;

	plane_segmenter.setAngularThreshold(pcl::deg2rad(2.0));
	plane_segmenter.setDistanceThreshold(0.02);
	plane_segmenter.setInputNormals(cloud_normals);
	plane_segmenter.setInputCloud(my_cloud);
	plane_segmenter.setMinInliers(20000);
	plane_segmenter.segmentAndRefine(regions, model_coefficients,
			inlier_indices, labels, label_indices, boundary_indices);


	std::cerr << "Inlier_indices length: " << inlier_indices.size()
			<< std::endl;
	std::cerr << "Label_indices length: " << label_indices.size() << std::endl;

	std::cerr << "Average of label_indices sizes: " << point_indices_vector_useful_average_value(label_indices) << std::endl;

	for (int a = 0; a < inlier_indices.size(); a++) {
		loc_indices = inlier_indices[a];
		std::cerr << "Number of inliers of the planes: "
				<< loc_indices.indices.size() << std::endl;
	}

	std::cerr << "Model Coeff SIZE: " << model_coefficients.size() << std::endl;

	for (int l = 0; l < model_coefficients.size(); l++) {
		my_coef = model_coefficients[l];
		std::cerr << "Plane Model Coefficients: " << my_coef.values[0] << " "
				<< my_coef.values[1] << " " << my_coef.values[2] << " "
				<< my_coef.values[3] << std::endl;
	}

	plane_vector = create_separate_planes(my_cloud);
	choose_plane_with_objects(plane_vector);

}

bool inlierPointOrNot(size_t in_point_indice,
		const pcl::PointIndices plane_indices) {

	bool present = false;

	for (size_t x = 0; x < plane_indices.indices.size(); x++)
		if (in_point_indice == plane_indices.indices[x])
			present = true;

	return present;
}

void find3DPlaneMinMax(const pcl::PointCloud<PointT>::Ptr cloud) {

	Eigen::Vector4f minPoint;
	Eigen::Vector4f maxPoint;

	pcl::PCLPointCloud2 myPointCloud2;

	pcl::toPCLPointCloud2(*cloud, myPointCloud2);

	const pcl::PCLPointCloud2::Ptr myConstPointCloud2(new pcl::PCLPointCloud2);

	myConstPointCloud2->data = myPointCloud2.data;
	myConstPointCloud2->fields = myPointCloud2.fields;
	myConstPointCloud2->header = myPointCloud2.header;
	myConstPointCloud2->height = myPointCloud2.height;
	myConstPointCloud2->width = myPointCloud2.width;
	myConstPointCloud2->point_step = myPointCloud2.point_step;
	myConstPointCloud2->row_step = myPointCloud2.row_step;

	pcl::getMinMax3D(myConstPointCloud2, 0, 1, 2, minPoint, maxPoint);

	std::cerr << "MIN: " << minPoint[0] << " " << minPoint[1] << " "
			<< minPoint[2] << std::endl;
	std::cerr << "MAX: " << maxPoint[0] << " " << maxPoint[1] << " "
			<< maxPoint[2] << std::endl;

	table_x_min = minPoint[0];
	table_y_min = minPoint[1];
	table_z_min = minPoint[2];

	table_x_max = maxPoint[0];
	table_y_max = maxPoint[1];
	table_z_max = maxPoint[2];

	std::cerr << "table Min: " << table_x_min << " " << table_y_min << " "
			<< table_z_min << std::endl;
	std::cerr << "table Max: " << table_x_max << " " << table_y_max << " "
			<< table_z_max << std::endl;
}

void findPlaneMinMax() {

	PointT myPoint;
	initTableMinMaxValues();

	for (size_t j = 0; j < cloud_projected->size(); j++) {
		if (inlierPointOrNot(j, *basePlaneIndices)) {
			myPoint = cloud_projected->points[j];

			if (myPoint.x > table_x_max)
				table_x_max = myPoint.x;
			if (myPoint.x < table_x_min)
				table_x_min = myPoint.x;

			if (myPoint.y > table_y_max)
				table_y_max = myPoint.y;
			if (myPoint.y < table_y_min)
				table_y_min = myPoint.y;

			if (myPoint.z > table_z_max)
				table_z_max = myPoint.z;
			if (myPoint.z < table_z_min)
				table_z_min = myPoint.z;
		}

	}
	std::cerr << "table Min: " << table_x_min << " " << table_y_min << " "
			<< table_z_min << std::endl;
	std::cerr << "table Max: " << table_x_max << " " << table_y_max << " "
			<< table_z_max << std::endl;
}

void trimPlane() {

	std::cerr << "TRIMPLANE BEGIN" << std::endl;

	float x_min;
	float x_max;
	float tolerance = 0.2;

	find3DPlaneMinMax(plane_cloud);

	x_min = table_x_min + tolerance;
	x_max = table_x_max - tolerance;

	for (size_t a = 0; a < plane_cloud->points.size(); a++) {

		if ((plane_cloud->points[a].x < x_min)
				|| (plane_cloud->points[a].x > x_max)) {
			plane_cloud->points[a].x = NAN;
			plane_cloud->points[a].y = NAN;
			plane_cloud->points[a].z = NAN;
		}

	}

	std::cerr << "TRIMPLANE END" << std::endl;

}

pcl::PointCloud<PointT>::CloudVectorType segmentObjects() {

	int k = 0;
	int valid_clusters = 0;

	std::cerr << "Segmentation" << std::endl;

	pcl::PointCloud<pcl::Label> euclidean_labels;

	pcl::PointCloud<PointT> cluster_cloud;

	pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr comparator(
			new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>());
	pcl::OrganizedConnectedComponentSegmentation<PointT, pcl::Label> segmenter(
			comparator);

	std::vector<pcl::PointIndices> euclidean_label_indices;

	std::cerr << "REGIONS SIZE: " << regions.size() << std::endl;

	if (regions.size() > 0) {
		std::vector<bool> plane_labels;
		plane_labels.resize(label_indices.size(), false);

		std::cout << "PLANE LABEL INDICES SIZE: " << label_indices.size()
				<< std::endl;

		for (size_t i = 0; i < label_indices.size(); i++) {
			if (label_indices[i].indices.size() > point_indices_vector_useful_average_value(label_indices)) {
				plane_labels[i] = true;
			}
		}

		comparator->setInputCloud(pcl_current);
		comparator->setLabels(labels);
		comparator->setExcludeLabels(plane_labels);
		comparator->setDistanceThreshold(0.01f, false);

		segmenter.setInputCloud(pcl_current);
		segmenter.segment(euclidean_labels, euclidean_label_indices);

		for (size_t i = 0; i < euclidean_label_indices.size(); i++) {

			if (euclidean_label_indices[i].indices.size() > 1000) {
//				valid_clusters;
				pcl::PointCloud<PointT> cluster;
				pcl::copyPointCloud(*pcl_current,
						euclidean_label_indices[i].indices, cluster);

				cluster_indices.resize(valid_clusters + 1);
				cluster_indices[valid_clusters] = euclidean_label_indices[i];
				valid_clusters++;

				std::stringstream ss;
				ss << clusterPCDPath << k << ".pcd";
				std::cerr << "Writing CLUSTER PCD's to: " << ss.str()
						<< std::endl;
				pcl::io::savePCDFile(ss.str(), cluster);
				clusters.push_back(cluster);
				k++;
			}
		}

		std::cerr << "Nr of clusters: " << clusters.size() << std::endl;
		for (size_t i = 0; i < clusters.size(); i++) {
			cluster_cloud = clusters[i];
			std::cerr << "Nr of points in clouds: "
					<< cluster_cloud.height * cluster_cloud.width << std::endl;
		}

	}

	return clusters;

}

void createAllInlierClusterIndices(std::vector<float> vect) {

	std::cerr << "CREATE INLIER CLUSTER INDICES" << std::endl;

	float total_cluster_indices_size;
	size_t indices_counter = 0;
	total_cluster_indices_size = vect[vect.size() - 1];

	std::cerr << "Total Cluster Indices Size: " << total_cluster_indices_size
			<< std::endl;

	pcl::PointCloud<PointT>::Ptr inlier_clusters_cloud(
			new pcl::PointCloud<PointT>());
	pcl::PointIndices::Ptr myInd(new pcl::PointIndices());
	pcl::PointIndices aux;
	myInd->indices.resize(total_cluster_indices_size);

	for (size_t a = 0; a < vect.size() - 1; a++) {
		aux = cluster_indices[vect[a]];
		std::cerr << "CLUSTER INDICES NR of: " << aux.indices.size()
				<< std::endl;
		for (size_t b = 0; b < aux.indices.size(); b++) {
//             std::cerr << "itt van a hiba?" << std::endl;
			myInd->indices.at(indices_counter) = aux.indices[b];
			indices_counter++;
		}
		std::cerr << "Indices counter so far: " << indices_counter << std::endl;
	}

	pcl::copyPointCloud<PointT>(*pcl_current, *myInd, *inlier_clusters_cloud);
	std::cerr
			<< "Saving Inlier Clusters Cloud to /home/furdek/InlierClusters.pcd"
			<< std::endl;
	pcl::io::savePCDFile("/home/furdek/InlierClusters.pcd",
			*inlier_clusters_cloud);

}

std::vector<float> pointsIn3DRegion(
		const pcl::PointCloud<PointT>::Ptr polygon) {

	pcl::PointCloud<PointT> cluster_cloud;
	pcl::PointCloud<PointT>::Ptr proba_cloud(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cluster_cloud_ptr(
			new pcl::PointCloud<PointT>());

	std::vector<float> my_vect;
	PointT my_point;
	size_t contor;
	int vect_contor = 0;
	float total_points = 0;

	for (size_t i = 0; i < clusters.size(); i++) {
		cluster_cloud = clusters[i];

		cluster_cloud_ptr->header = cluster_cloud.header;
		cluster_cloud_ptr->height = cluster_cloud.height;
		cluster_cloud_ptr->points = cluster_cloud.points;
		cluster_cloud_ptr->width = cluster_cloud.width;

		contor = 0;
		for (size_t j = 0; j < cluster_cloud.size(); j++) {
			my_point = cluster_cloud[j];
			if (pcl::isPointIn2DPolygon(my_point, *polygon)) {
				contor++;
			}
		}
		std::cerr << "Number of points inside the 3D Region: " << contor
				<< std::endl;
		std::cerr << "Number of points inside cloud: " << cluster_cloud.size()
				<< std::endl;
		if (contor == cluster_cloud.size()) {
			pcl::io::savePCDFile("/home/furdek/catkin_ws/cloud_4_stefan.pcd",
					cluster_cloud);
			my_vect.resize(vect_contor + 1);
			std::cerr << "Cluster cloud NR." << i << " is on the Main Plane!"
					<< std::endl;
			std::stringstream ss;
			std::stringstream ss1;
			ss << VTKPath << i << ".vtk";
			pcl::io::loadPCDFile("/home/furdek/catkin_ws/cloud_4_stefan.pcd",
					*proba_cloud);
			save_pcl_VTK(proba_cloud, ss.str());
//            save_pcl_VTK(cluster_cloud_ptr,ss.str());
			ss1 << STLPath << i << ".stl";
			save_VTK_to_binary_STL(ss.str(), ss1.str());
			my_vect[vect_contor] = i;
			total_points += cluster_cloud.size();
			vect_contor++;
		}

	}

	std::cerr << "Total size: " << total_points << std::endl;

	my_vect.resize(vect_contor + 1);
	my_vect[vect_contor] = total_points;

	std::cerr << "Finish pointsIn3DRegion" << std::endl;

	std::cerr << my_vect.size() << std::endl;

	return my_vect;

}

void projectPointsOnPlane(const pcl::PointCloud<PointT>::ConstPtr cloud) {

	pcl::ModelCoefficients::Ptr model_pointer(new pcl::ModelCoefficients);
	pcl::ProjectInliers<PointT> proj;

	std::cerr << "PROJECT POINTS ON PLANE" << std::endl;
	std::cerr << "Main Plane Index: " << main_plane_index << std::endl;

	model_pointer->header = model_coefficients[main_plane_index].header;
	model_pointer->values = model_coefficients[main_plane_index].values;

	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(model_pointer);

	proj.filter(*cloud_projected);
	std::cerr
			<< "Writing Projection Of Clusters on Main Plane to /home/furdek/ProjectionProba.pcd"
			<< std::endl;
	pcl::io::savePCDFile("/home/furdek/ProjectionProba.pcd", *cloud_projected);

}

void manualProjection() {

}

pcl::PointCloud<PointT>::Ptr createConvexHull(
		const pcl::PointCloud<PointT>::ConstPtr my_cloud) {

	pcl::PointCloud<PointT>::Ptr hull_vis_cloud(new pcl::PointCloud<PointT>());

	pcl::ConvexHull<PointT> chull;

	chull.setInputCloud(my_cloud);
	chull.reconstruct(*cloud_hull);

	std::cerr << "CLOUD HULL SIZE: " << cloud_hull->size() << std::endl;

	hull_vis_cloud->resize(cloud_hull->points.size());
	for (size_t a = 0; a < cloud_hull->points.size(); a++) {
		hull_vis_cloud->points[a] = cloud_hull->points[a];
	}

	std::cerr << "Writing Convex Hull to /home/furdek/Hull.pcd" << std::endl;
	pcl::io::savePCDFile("/home/furdek/Hull.pcd", *hull_vis_cloud);

	return hull_vis_cloud;

}

pcl::PointIndices::Ptr ExtractObjectsWithinHull(
		const pcl::PointCloud<PointT>::ConstPtr my_cloud,
		const pcl::ConvexHull<PointT>::PointCloud::ConstPtr my_hull) {

	pcl::PointIndices::Ptr hull_output(new pcl::PointIndices);
	pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices());
	pcl::ExtractPolygonalPrismData<PointT> ex;
	pcl::PointIndices my_Indices;

	std::cerr << "PRISM" << std::endl;

	my_Indices = inlier_indices[main_plane_index];

	plane_indices->header = my_Indices.header;
	plane_indices->indices = my_Indices.indices;

	std::cerr << "Set InPUt Cloud!" << std::endl;

	ex.setInputCloud(my_cloud);

	ex.setInputPlanarHull(my_hull);

	std::cerr << "Set Indices!" << std::endl;

	ex.setHeightLimits(0, 1);

//    ex.setIndices(plane_indices);

	std::cerr << "Hull Segment" << std::endl;

	ex.segment(*hull_output);

	std::cerr << "HULL OUTPUT SIZE: " << hull_output->indices.size()
			<< std::endl;

	return hull_output;

}

void extractHullOutliersFromProjectedCloud(
		const pcl::PointCloud<PointT>::ConstPtr cloud,
		pcl::PointIndices::Ptr indices) {

	pcl::ExtractIndices<PointT> extractor;
	extractor.setInputCloud(cloud);
	extractor.setIndices(indices);
//    extractor.setNegative(true);
	extractor.filter(*projected_obj_cloud);
	std::cout
			<< "Writing Final PROJECTION PCD to /home/furdek/ProjectionFinal.pcd"
			<< std::endl;
	pcl::io::savePCDFile("/home/furdek/ProjectionFinal.pcd",
			*projected_obj_cloud);

	std::cerr << "FINAL PROJECTION READY" << std::endl;

}

pcl::PointCloud<PointT>::Ptr RadiusOutlierRemoval(
		const pcl::PointCloud<PointT>::ConstPtr cloud) {

	pcl::PointCloud<PointT>::Ptr clusters_2D_cloud(
			new pcl::PointCloud<PointT>());
	pcl::RadiusOutlierRemoval<PointT> outrem;

	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(0.005);
	outrem.setMinNeighborsInRadius(50);
	outrem.filter(*clusters_2D_cloud);

	std::cerr
			<< "Writing Radius Outlier Removal to Projected Points to /home/furdek/Clusters2D.pcd"
			<< std::endl;
	pcl::io::savePCDFile("/home/furdek/Clusters2D.pcd", *clusters_2D_cloud);

	return clusters_2D_cloud;

}

void extractPlanePointsFromCloud(const pcl::PointCloud<PointT>::ConstPtr cloud,
		const pcl::PointIndices::ConstPtr indices) {

	pcl::ExtractIndices<PointT> extractor;
	extractor.setInputCloud(cloud);
	extractor.setIndices(indices);
	extractor.setNegative(true);
	extractor.filter(*projected_obj_cloud);

	std::cerr
			<< "Writing Extract Plane From Projected Cloud to /home/furdek/FINISH.pcd"
			<< std::endl;
	pcl::io::savePCDFile("/home/furdek/FINISH.pcd", *projected_obj_cloud);

}

pcl::PointCloud<PointT>::Ptr createPlaneAndInlierCloud(
		std::vector<float> myvect, float size) {

	std::cerr << "Create PLANE and INLIER cloud" << std::endl;

	pcl::PointCloud<PointT>::Ptr plane_with_clusters(
			new pcl::PointCloud<PointT>());

	int new_cloud_contor = 0;

	float total_size = 0;
	std::cerr << "Plane indices: " << plane_cloud->size() << std::endl;
	total_size = size + plane_cloud->size();
	pcl::PointCloud<PointT> aux;

	std::cerr << "Total size: " << total_size << std::endl;
	plane_with_clusters->resize(total_size);
	plane_with_cluster_plane_indices->indices.resize(plane_cloud->size());

	for (size_t a = 0; a < plane_cloud->size(); a++) {
		plane_with_clusters->points[new_cloud_contor] = plane_cloud->points[a];
		plane_with_cluster_plane_indices->indices.at(new_cloud_contor) = a;
		new_cloud_contor++;
	}

	for (size_t b = 0; b < myvect.size() - 1; b++) {
		aux = clusters[myvect[b]];
		std::cerr << aux.size() << std::endl;
		for (size_t c = 0; c < aux.size() - 1; c++) {
			plane_with_clusters->points[new_cloud_contor] = aux.points[c];
			new_cloud_contor++;
		}
	}

	std::cerr << "new Cloud Nr of Points: " << plane_with_clusters->size()
			<< std::endl;
	std::cerr
			<< "Saving Plane With Clusters Cloud to /home/furdek/PlaneWithClusters.pcd"
			<< std::endl;
	pcl::io::savePCDFile("/home/furdek/PlaneWithClusters.pcd",
			*plane_with_clusters);

	return plane_with_clusters;

}

void final_filter() {

	int counter = 0;
	pcl::PointCloud<PointT>::Ptr my_final(new pcl::PointCloud<PointT>());

	my_final->resize(projected_obj_cloud->size());

	for (size_t b = 0; b < projected_obj_cloud->size(); b++) {
		PointT my_point;
		my_point = projected_obj_cloud->points[b];
		if (pcl::isXYPointIn2DXYPolygon(my_point, *cloud_hull)) {
			std::cerr << counter << std::endl;
			my_final->points[b] = my_point;
			counter++;
		}
	}

	std::cout << "Writing PCD after final_filter to /home/furdek/final.pcd"
			<< std::endl;

	pcl::io::savePCDFile("/home/furdek/final.pcd", *my_final);

}

pcl::PointIndices::Ptr segment_final_plane(
		const pcl::PointCloud<PointT>::Ptr my_cloud) {

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<PointT> seg;

	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.007);

	seg.setInputCloud(my_cloud);
	seg.segment(*inliers, *coefficients);

	std::cerr << inliers->indices.size() << std::endl;

	return inliers;

}

void clustering() {

	std::cout << "Clustering" << std::endl;

	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
	pcl::PointCloud<PointT>::Ptr hull_cloud(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr clusters_2D_cloud(
			new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr plane_with_clusters(
			new pcl::PointCloud<PointT>());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
			new pcl::PointCloud<pcl::Normal>);
	pcl::PointIndices::Ptr objects_in_hull_indices(new pcl::PointIndices);

	pcl::PCDReader reader;
	reader.read(probaString, *cloud_blob);
	pcl::fromPCLPointCloud2(*cloud_blob, *pcl_current);

	std::vector<float> myVect;
	std::vector<pcl::PointCloud<PointT> > final_clusters;
	float sizze = 0;

	cloud_normals = normalEstimation(pcl_current);

	segmentPlanes(pcl_current, cloud_normals);

	segmentObjects();

	hull_cloud = createConvexHull(plane_cloud);

	myVect = pointsIn3DRegion(hull_cloud);

	std::cerr << "SIZZE BEF" << std::endl;

	std::cerr << myVect.size() << std::endl;

	sizze = myVect[myVect.size() - 1];
	std::cerr << "SIZZE AFT" << std::endl;

	plane_with_clusters = createPlaneAndInlierCloud(myVect, sizze);
	createAllInlierClusterIndices(myVect);

	projectPointsOnPlane(plane_with_clusters);
	extractPlanePointsFromCloud(cloud_projected,
			plane_with_cluster_plane_indices);
	clusters_2D_cloud = RadiusOutlierRemoval(cloud_projected);
	final_clusters = separate_2d_clusters(clusters_2D_cloud);

	std::cerr << final_clusters.size() << std::endl;
//    ransac_line_fitting(final_clusters); //GOOD
	ransac_circle_fitting(final_clusters);

//    pcl::io::savePCDFile("/home/furdek/ConvexHullBefore.pcd",
//            *plane_cloud);

//    trimPlane();

//    find3DPlaneMinMax(plane_cloud);

//    std::cerr << "SIZE before PASS: " << plane_cloud->size() << std::endl;

//    pcl::PassThrough<pcl::PointXYZ> pass;
//    pass.setInputCloud(plane_cloud);
//      pass.setFilterFieldName ("x");
//      std::cerr << pass.getFilterFieldName() << std::endl;
//      pass.setFilterLimits (table_x_min + 0.2, table_x_max - 0.2);
//      pass.filter (*plane_cloud);

//      std::cerr << "SIZE after PASS: " << plane_cloud->size() << std::endl;

//      pcl::io::savePCDFile("/home/furdek/PlaneAfterPass.pcd",
//              *plane_cloud);

//    createConvexHull(plane_cloud);

//	objects_in_hull_indices = ExtractObjectsWithinHull(cloud_projected,
//			cloud_hull);
//	extractHullOutliersFromProjectedCloud(cloud_projected,
//			objects_in_hull_indices);
//    final_filter();
//    pcl::PointIndices::Ptr myInd (new pcl::PointIndices());
//    myInd = segment_final_plane(my_final);
//    std::cerr << my_final->size() << std::endl;

//    extractPlanePointsFromCloud(my_final,myInd);
//    extractPlaneFromProjectedCloud(cloud_projected);
//	ransac_circle_fitting(cloud_p);
//    ransac_line_fitting(cloud_p);
}

void initTableMinMaxValues() {
	table_x_min = 1000000;
	table_x_max = -1000000;
	table_y_min = 1000000;
	table_y_max = -1000000;
	table_z_min = 1000000;
	table_z_max = -1000000;
}

int main(int argc, char** argv) {

	probaString = "/home/furdek/PCDs/position0_with_objects.pcd";
	clusterPCDPath = "/home/furdek/PCDs/CLUSTER_0";
	VTKPath = "/home/furdek/VTK_file";
	STLPath = "/home/furdek/STL_file";

	clustering();

//    pcl::PointCloud<PointT>::Ptr faszom(new pcl::PointCloud<PointT>());
//    pcl::io::loadPCDFile("/home/furdek/catkin_ws/cloud_4_stefan.pcd", *faszom);
//    save_proba(faszom);

	return (0);
}
