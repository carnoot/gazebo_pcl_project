#include <iostream>
#include <eigen_stl_containers/eigen_stl_containers.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>
#include <pcl/segmentation/planar_region.h>

std::string initialPCDPath;

void openViewer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		const pcl::PointCloud<pcl::Normal>::ConstPtr cloud_norm) {

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
			cloud);
//    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, initialPCDPath);
//	viewer->setPointCloudRenderingProperties(
//            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, initialPCDPath);
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud,
			cloud_norm);
	viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0);
	viewer->initCameraParameters();

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void openViewer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
			cloud);
//    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, initialPCDPath);
	viewer->addPointCloud(cloud);
	viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
	viewer->initCameraParameters();

	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void multiPlaneExtraction(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
		const pcl::PointCloud<pcl::Normal>::ConstPtr normals) {

	pcl::PCDWriter writer;
	writer.write("/home/furdek/MultiCloudAtStart.pcd", *cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(
			new pcl::PointCloud<pcl::PointXYZ>);

	pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label> plane_segmenter;
	std::vector<pcl::ModelCoefficients> model_coef;
	std::vector<pcl::PointIndices> point_inliers;
	pcl::PointIndices myPointIndices;

	pcl::IndicesPtr myIndicePtr;
	pcl::PointIndices::Ptr myConstPtr(new pcl::PointIndices);
	boost::shared_ptr<pcl::PointIndices> shared_proba_pt;

	plane_segmenter.setInputCloud(cloud);
	plane_segmenter.setInputNormals(normals);
//    plane_segmenter.setMinInliers(10000);
	plane_segmenter.setAngularThreshold(0.017453 * 2.0);
	plane_segmenter.setDistanceThreshold(0.02);
	std::vector<pcl::PlanarRegion<pcl::PointXYZ> > regions;

	plane_segmenter.segment(model_coef, point_inliers);

//    plane_segmenter.segmentAndRefine(regions,what);/*
//	for (size_t i = 0; i < regions.size(); i++) {

//		Eigen::Vector3f centroid = regions[i].getCentroid();
//		Eigen::Vector4f model = regions[i].getCoefficients();
//		pcl::PointCloud<pcl::PointXYZ> boundary_cloud;
//		boundary_cloud.points = regions[i].getContour();

//		printf(
//				"Centroid: (%f, %f, %f)\n Coefficients: (%f, %f, %f, %f)\n Inliers: %d\n",
//				centroid[0], centroid[1], centroid[2], model[0], model[1],
//				model[2], model[3], boundary_cloud.points.size());

//    }

	std::cerr << "PLANE SEGMENT READY" << std::endl;
	std::cerr << model_coef.size() << std::endl;
	std::cerr << point_inliers.size() << std::endl;

	for (int j = 0; j < point_inliers.size(); j++) {

		std::cerr << "Nr of inlier points: " << point_inliers[j].indices.size()
				<< std::endl;

		myPointIndices = point_inliers[j];

		myConstPtr->header = myPointIndices.header;
		myConstPtr->indices = myPointIndices.indices;

		std::cerr << "EXTRACTOR BEGIN" << std::endl;
		pcl::ExtractIndices<pcl::PointXYZ> extractor;
		extractor.setInputCloud(cloud);
		extractor.setIndices(myConstPtr);
//    extractor.setNegative(true);
		extractor.filter(*cloud_p);
		std::cerr << "EXTRACTOR END" << std::endl;

		std::stringstream ss;
		ss << "/home/furdek/ExtractedPlane" << j << ".pcd";

		std::cerr << "Writing" << std::endl;
		writer.write(ss.str(), *cloud_p);

	}

//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_view(new pcl::PointCloud<pcl::PointXYZRGB>);
//    cloud_view->resize(cloud->size());
//    for (size_t i = 0; i < cloud->size(); i++){
//        cloud_view->points[i].x = cloud->points[i].x;
//        cloud_view->points[i].y = cloud->points[i].y;
//        cloud_view->points[i].z = cloud->points[i].z;
//    }

//    writer.write("/home/furdek/MultiCloudAtEnd.pcd", *cloud);

//    openViewer(cloud_view);

}

pcl::PointCloud<pcl::Normal>::Ptr normalEstimation(
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr my_cloud) {

//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
//			new pcl::PointCloud<pcl::PointXYZ>);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(my_cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
			new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
			new pcl::PointCloud<pcl::Normal>);

	ne.setRadiusSearch(0.03);

	ne.compute(*cloud_normals);

	std::cerr << "NORMALS READY" << std::endl;

	return cloud_normals;

}

int main(int argc, char** argv) {

	initialPCDPath = "/home/furdek/PCDs/position1_with_objects";

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2),
			cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_current(
			new pcl::PointCloud<pcl::PointXYZ>), cloud_current_copy(
			new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_norm(
			new pcl::PointCloud<pcl::Normal>);

	pcl::PCDWriter writer;

	pcl::PCDReader reader;
	reader.read(initialPCDPath + ".pcd", *cloud_blob);
	pcl::io::loadPCDFile<pcl::PointXYZRGB>(initialPCDPath + ".pcd", *cloud);
	pcl::fromPCLPointCloud2(*cloud_blob, *cloud_current);

	cloud_norm = normalEstimation(cloud_current);
	writer.write("/home/furdek/NORMALS.pcd", *cloud_norm);

	multiPlaneExtraction(cloud_current, cloud_norm);

//    openViewer(cloud, cloud_norm);

	return (0);
}
