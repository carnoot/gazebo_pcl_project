#include <iostream>
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
#include <pcl/conversions.h>

float table_x_min;
float table_x_max;
float table_y_min;
float table_y_max;

void clustering() {

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_current(
			new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZRGB>::CloudVectorType clusters;

//	pcl::fromPCLPointCloud2(*cloud_blob, *pcl_current);

	boost::shared_ptr<std::vector<pcl::PointIndices> > labelsIndices(
			new std::vector<pcl::PointIndices>());
	pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>());

	pcl::EuclideanClusterComparator<pcl::PointXYZ, pcl::Normal, pcl::Label>::Ptr comparator(
			new pcl::EuclideanClusterComparator<pcl::PointXYZ, pcl::Normal,
					pcl::Label>());

	comparator->setDistanceThreshold(0.1, false); //even if i set it to 10 its still the same
	comparator->setInputCloud(pcl_current);
	comparator->setLabels(labels);

	pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZ, pcl::Label> segmenter(
			comparator);
	segmenter.setInputCloud(pcl_current);
	segmenter.segment(*labels, *labelsIndices);

	for (size_t i = 0; i < labelsIndices->size(); i++) {

		std::cerr << i << std::endl;
		if (labelsIndices->at(i).indices.size() > 1) {
			pcl::PointCloud<pcl::PointXYZRGB> cluster;
			pcl::copyPointCloud(*pcl_current, labelsIndices->at(i).indices,
					cluster);
			clusters.push_back(cluster);
		}
	}

	std::cerr << clusters.size() << std::endl;
}

void initTableMinMaxValues() {
	table_x_min = 1000000;
	table_x_max = -1000000;
	table_y_min = 1000000;
	table_y_max = -1000000;
}

int main(int argc, char** argv) {
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2),
			cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_current(
			new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZ>), cloud_p(
			new pcl::PointCloud<pcl::PointXYZ>), cloud_f(
			new pcl::PointCloud<pcl::PointXYZ>);

	std::string initialPCDPath;
	std::string downsampledPCDPath;
	std::string filteredPCDPath;
	std::string planePCDPath;
	std::string planePCDInliersPath;
	std::string planePCDOutliersPath;

	initialPCDPath = "/home/furdek/PCDs/position0_with_objects";
	planePCDPath = initialPCDPath + "_plane";
	planePCDInliersPath = initialPCDPath + "_inliers";
	planePCDOutliersPath = initialPCDPath + "_outliers";

	pcl::PCDReader reader;
	reader.read(initialPCDPath + ".pcd", *cloud_blob);
    pcl::fromPCLPointCloud2(*cloud_blob, *cloud_current);

    std::cerr << "IS CLOUD_BLOB=CLOUD_CURRENT ORGANIZED?? " << cloud_current->isOrganized() << " height: " << cloud_current->height << std::endl;

	std::cerr << "PointCloud before filtering: "
			<< cloud_blob->width * cloud_blob->height << " data points."
			<< std::endl;

	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*cloud_filtered_blob);

	// Convert to the templated PointCloud
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

    std::cerr << "PointCloud width & height after filtering: "
              << cloud_filtered->width << " " << cloud_filtered->height << " data points."
			<< std::endl;

    std::cerr << "IS CLOUD_FILTERED ORGANIZED?? " << cloud_filtered->isOrganized() << std::endl;

	pcl::PCDWriter writer;
	downsampledPCDPath = initialPCDPath + "_downsampled" + ".pcd";
	writer.write<pcl::PointXYZ>(downsampledPCDPath, *cloud_filtered, false);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	pcl::SACSegmentation<pcl::PointXYZ> seg;

	seg.setOptimizeCoefficients(true);

	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.01);

	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	int i = 0, nr_points = (int) cloud_filtered->points.size();
	// While 30% of the original cloud is still there
    while (cloud_filtered->points.size() > 0.1 * nr_points) {
		std::cerr << "WHILE LOOP + i " << i << std::endl;
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);

//        for (int a = 0; a < inliers->indices.size(); a++){
//            std::cerr << inliers->indices[a] << std::endl;
//        }

		if (inliers->indices.size() == 0) {
			std::cerr
					<< "Could not estimate a planar model for the given dataset."
					<< std::endl;
			break;
		}

		// Extract the inliers
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);

        std::cerr << "REMAINING POINTS:" << cloud_filtered->points.size() << std::endl;

		std::cerr << "PointCloud representing the planar component: "
				<< cloud_p->width * cloud_p->height << " data points."
				<< std::endl;

		std::stringstream ss;
		ss << planePCDPath << i << ".pcd";
		writer.write<pcl::PointXYZ>(ss.str(), *cloud_filtered, false);

            std::cerr << "Inlier indices zise: " << inliers->indices.size() << std::endl;
            std::cerr << "Last element of inliers: " << inliers->indices[inliers->indices.size() + 10] << std::endl;
            std::cerr << "CLOUD CURRENT SIZE: " << cloud_filtered->points.size() << std::endl;
            std::cerr << "POINTTTTTTTTT: " << cloud_filtered->at(inliers->indices[inliers->indices.size()]) << std::endl;

		if (i == 1) {
			pcl::PointXYZ myPoint;
			initTableMinMaxValues();

            std::cerr << "after table values init" << std::endl;
            std::cerr << cloud_current->width << std::endl;
            std::cerr << cloud_current->height << std::endl;

            for (size_t j = 0; j < cloud_current->width; j++)
                for (size_t k = 0; k < cloud_current->height; k++) {

//					std::cerr << "at ELOTT" << std::endl;
                    myPoint = cloud_current->at(j,k);
//                    std::cerr << "x y z" << myPoint.x << " " << myPoint.y << " " << myPoint.z << std::endl;
//					std::cerr << "at UTAN" << std::endl;

					if (myPoint.x > table_x_max)
						table_x_max = myPoint.x;
					if (myPoint.x < table_x_min)
						table_x_min = myPoint.x;

					if (myPoint.y > table_y_max)
						table_y_max = myPoint.y;
					if (myPoint.y < table_y_min)
						table_y_min = myPoint.y;
				}
			std::cerr << "table Min: " << table_x_min << " " << table_y_min
					<< std::endl;
			std::cerr << "table Max: " << table_x_max << " " << table_y_max
					<< std::endl;
		}

		// Create the filtering object
		extract.setNegative(true);
		extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f);

		filteredPCDPath = initialPCDPath + "_filtered";

		writer.write<pcl::PointXYZ>(filteredPCDPath + ".pcd", *cloud_p, false);
		i++;

//	// Create the filtering object
//	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//	sor.setInputCloud(cloud_p);
//	sor.setMeanK(50);
//	sor.setStddevMulThresh(1);
//	sor.filter(*cloud_p);

//	std::stringstream ss1;
//	ss1 << planePCDInliersPath << i << ".pcd";

//	pcl::PCDWriter writer;
//	writer.write<pcl::PointXYZ>(ss1.str(), *cloud_p, false);

//        std::stringstream ss2;
//        ss2 << planePCDOutliersPath << i << ".pcd";

//		sor.setNegative(true);
//		sor.filter(*cloud_p);
//		writer.write<pcl::PointXYZ>("table_scene_lms400_outliers.pcd",
//				*cloud_filtered, false);

	}

	return (0);
}
