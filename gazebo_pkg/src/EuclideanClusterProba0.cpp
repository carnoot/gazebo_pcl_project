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
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/passthrough.h>
#include <pcl/pcl_base.h>

typedef pcl::PointXYZ PointT;

float table_x_min;
float table_x_max;
float table_y_min;
float table_y_max;
float table_z_min;
float table_z_max;
float mask_true;
float mask_vector_counter;
std::vector<bool> mask_vector(380000);

std::string initialPCDPath;
std::string probaString;
std::string clusterPCDPath;

void initTableMinMaxValues();
void findPlaneMinMax();
void outlier_removal_from_plane(const pcl::PointCloud<PointT>::Ptr);
void find3DPlaneMinMax(const pcl::PointCloud<PointT>::Ptr cloud);

pcl::ConvexHull<PointT> chull;
pcl::ExtractPolygonalPrismData<PointT> ex;

pcl::PointIndices::Ptr hull_output (new pcl::PointIndices);

pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);


pcl::PointCloud<PointT>::Ptr my_final (new pcl::PointCloud<PointT>());


pcl::ConvexHull<PointT>::PointCloud::Ptr cloud_hull (new pcl::ConvexHull<PointT>::PointCloud());
pcl::PointCloud<PointT>::Ptr projected_obj_cloud(new pcl::PointCloud<PointT>());

pcl::PointCloud<PointT>::Ptr pcl_current(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr cloud_p(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>());
const pcl::PointCloud<PointT>::Ptr plane_cloud_final(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr cloud_projected(new pcl::PointCloud<PointT>());

pcl::PointCloud<PointT>::CloudVectorType clusters;

pcl::PointCloud<pcl::Normal>::Ptr cloud_norm(new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
		new pcl::PointCloud<pcl::Normal>);

pcl::PointIndices::Ptr basePlaneIndices(new pcl::PointIndices);

std::vector<pcl::PointIndices> label_indices;
std::vector<pcl::PointIndices> boundary_indices;
std::vector<pcl::PointIndices> inlier_indices;

pcl::PointIndices::Ptr plane_indices (new pcl::PointIndices());

boost::shared_ptr<std::vector<pcl::PointIndices> > labelsIndices(
		new std::vector<pcl::PointIndices>());

pcl::ModelCoefficients::Ptr model_pointer(new pcl::ModelCoefficients);
std::vector<pcl::ModelCoefficients> model_coefficients;

pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);

std::vector<pcl::PlanarRegion<PointT>,
		Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;

pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> plane_segmenter;

std::vector<pcl::PointCloud<PointT> > plane_vector;

void create_separate_planes(const pcl::PointCloud<PointT>::Ptr cloud){

    std::cerr << "CREATING SEPARATE PLANES" << std::endl;

    plane_vector.resize(regions.size());

    for (size_t i = 0; i < regions.size(); i++){
        std::cerr << "Inside FOR" << std::endl;
        pcl::PointIndices myInd;
        myInd = inlier_indices[i];
        std::cerr << "INSIDE FOR INDICE SIZE: " << myInd.indices.size() << std::endl;
        pcl::copyPointCloud<PointT>(*cloud, myInd.indices, plane_vector[i]);
        pcl::PCDWriter writer;
        std::stringstream ss;
        std::string initstring = "/home/furdek/SEPARATE_PLANES";
        ss << initstring << i << ".pcd";
        writer.write(ss.str(), plane_vector[i]);
    }

    std::cerr << "PLANE SIZES CREATE SEPARATE PLANES!" << std::endl;

    for (int j = 0; j < plane_vector.size(); j++){
        pcl::PointCloud<PointT> my_cloud;
        my_cloud = plane_vector[j];
        std::cerr << my_cloud.size() << std::endl;
    }

}

void choose_plane_with_objects(){

    double hole_treshold = 0.05; // DISCUTABIL
    double holes = 0;
    std::vector<double> hole_vector;
    hole_vector.resize(plane_vector.size());

    for (size_t m = 0; m < plane_vector.size(); m++){
        pcl::PointCloud<PointT> cloud_plane;
        cloud_plane = plane_vector[m];
        for (size_t n = 1; n < cloud_plane.size(); n++){
            double n_x;
            double n1_x;
            n_x = cloud_plane.at(n).x;
            n1_x = cloud_plane.at(n-1).x;

//            std::cerr << "bemai" << std::endl;
//            if (m == 1){
//            std::cerr << "x: " << n_x << std::endl;
//            }

            if (fabs(n_x) - fabs(n1_x) > hole_treshold){
//            if (fabs((double)(plane_cloud[n].x)) - fabs((double)(plane_cloud[n-1])) > hole_treshold){
                holes++;
            }
        }
        hole_vector[m] = holes;
        std::cerr << "Nr. of HOLES in PLANE " << m << ": " << holes << std::endl;
        holes = 0;
    }

    std::vector<double>::iterator it = std::max_element(hole_vector.begin(), hole_vector.end());
    std::cerr << "MINIMUM OF HOLE VECTOR: " << *it << std::endl;
//    std::distance(hole_vector.begin(), it);
    std::cerr << "POSITION OF MIN HOLE IN VECTOR: " << it - hole_vector.begin() << std::endl;

    pcl::copyPointCloud(plane_vector[it - hole_vector.begin()], *plane_cloud);
    pcl::PCDWriter writer;
    writer.write("/home/furdek/MYPLANE.pcd", *plane_cloud);

    outlier_removal_from_plane(plane_cloud);

}

void extractPlaneFromProjectedCloud(
        const pcl::PointCloud<PointT>::ConstPtr cloud) {

    find3DPlaneMinMax(plane_cloud);
//    find3DPlaneMinMax(cloud_projected);

    int points_altered = 0;

    float plus_minus_z = 0.05;
    float plus_minus_x = 0.2;

    pcl::io::savePCDFile("/home/furdek/CloudFromWhichToExtract.pcd", *cloud);
    pcl::ExtractIndices<pcl::PointXYZ> extractor;

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
                    || loc_point.y < table_y_min
                    || loc_point.y > table_y_max) {

                points_altered++;

                cloud_p->points[m].x = NAN;
                cloud_p->points[m].y = NAN;
                cloud_p->points[m].z = NAN;

            }
        }
//	}

    std::cerr << "Points altered: " << points_altered << std::endl;

    pcl::io::savePCDFile("/home/furdek/ProjectionProbaWithoutPlane.pcd",
            *cloud_p);

}

void outlier_removal_from_plane(
        const pcl::PointCloud<PointT>::Ptr cloud) {

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::PCDWriter writer;

    sor.setInputCloud(cloud);
    sor.setMeanK(1000); //250 eleg jo
    sor.setStddevMulThresh(2);
    sor.filter(*cloud);

//	std::cout << "Writing PCD after Statistical Outlier Removal!" << std::endl;
    writer.write("/home/furdek/PLANEAfterOutlier.pcd", *cloud);
}

void outlier_removal_from_projections(
		const pcl::PointCloud<PointT>::Ptr cloud) {

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	pcl::PCDWriter writer;

	sor.setInputCloud(cloud);
	sor.setMeanK(500);
    sor.setStddevMulThresh(0.1);
	sor.filter(*cloud);

	std::cout << "Writing PCD after Statistical Outlier Removal!" << std::endl;
	writer.write("/home/furdek/AfterOutlier.pcd", *cloud);
}

void ransac_circle_fitting(const pcl::PointCloud<PointT>::Ptr cloud) {

	pcl::SampleConsensusModelCircle2D<PointT>::Ptr circle_model(
			new pcl::SampleConsensusModelCircle2D<PointT>(cloud));
	pcl::RandomSampleConsensus<PointT> ransac(circle_model);

	pcl::PointCloud<PointT>::Ptr circle_cloud(new pcl::PointCloud<PointT>());
	std::vector<int> ransac_circle_inliers;

	std::cerr << "Original Cloud Size: " << cloud->points.size() << std::endl;

	std::cerr << "Applying outlier removal!" << std::endl;

	outlier_removal_from_projections(cloud);

	std::cerr << "Points remaining after outlier removal: "
			<< cloud->points.size() << std::endl;

	std::cout << "Circle Inlier Size BEFORE: " << ransac_circle_inliers.size()
			<< std::endl;

	ransac.setDistanceThreshold(0.01);
	ransac.computeModel();
	ransac.getInliers(ransac_circle_inliers);

	std::cout << "Circle Inlier Size AFTER: " << ransac_circle_inliers.size()
			<< std::endl;

	pcl::copyPointCloud<PointT>(*cloud, ransac_circle_inliers, *circle_cloud);

	if (ransac_circle_inliers.size()) {
		std::cout << "Writing CIRCLES Cloud" << std::endl;
		pcl::PCDWriter writer;
		writer.write("/home/furdek/CIRCLES.pcd", *circle_cloud);
	} else {
		std::cerr << "Could not find any CIRCLES!" << std::endl;
	}
}

void ransac_line_fitting(const pcl::PointCloud<PointT>::Ptr cloud) {

	pcl::SampleConsensusModelLine<PointT>::Ptr line_model(
			new pcl::SampleConsensusModelLine<PointT>(cloud));
	pcl::RandomSampleConsensus<PointT> ransac(line_model);
	pcl::PointCloud<PointT>::Ptr line_cloud(new pcl::PointCloud<PointT>());

	std::vector<int> ransac_line_inliers;

	std::cerr << "Original Cloud Point Size: " << cloud->points.size()
			<< std::endl;

	std::cerr << "Applying outlier removal!" << std::endl;

	outlier_removal_from_projections(cloud);

	std::cerr << "Points remaining after outlier removal: "
			<< cloud->points.size() << std::endl;

	ransac.setDistanceThreshold(0.01);
	ransac.computeModel();
	ransac.getInliers(ransac_line_inliers);

	std::cout << "LINE Inlier Size: " << ransac_line_inliers.size()
			<< std::endl;

	pcl::copyPointCloud<PointT>(*cloud, ransac_line_inliers, *line_cloud);

	if (ransac_line_inliers.size()) {
		std::cout << "Writing LINES Cloud" << std::endl;
		pcl::PCDWriter writer;
		writer.write("/home/furdek/LINES.pcd", *line_cloud);
	} else {
		std::cerr << "Could not find any LINES!" << std::endl;

	}
}

void normalEstimation(const pcl::PointCloud<PointT>::ConstPtr my_cloud) {

	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	ne.setInputCloud(my_cloud);
	ne.setSearchMethod(tree);
	ne.setRadiusSearch(0.03); //#################### IF NOT 0.03 BEHAVES STRANGELY ####################
	ne.compute(*cloud_normals);

}

void segmentPlanes(const pcl::PointCloud<PointT>::Ptr my_cloud) {

	pcl::PointIndices loc_indices;
	pcl::PointIndices local_point_indices;
	pcl::ModelCoefficients my_coef;
	pcl::PCDWriter writer;

	plane_segmenter.setAngularThreshold(pcl::deg2rad(2.0));
	plane_segmenter.setDistanceThreshold(0.02);
	plane_segmenter.setInputNormals(cloud_normals);
    plane_segmenter.setInputCloud(my_cloud);
    plane_segmenter.setMinInliers(40000);
	plane_segmenter.segmentAndRefine(regions, model_coefficients,
			inlier_indices, labels, label_indices, boundary_indices);

	std::cerr << "Inlier_indices length: " << inlier_indices.size()
            << std::endl;

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

	//##################### HARD-CODED ###################

	local_point_indices = inlier_indices[1];

	basePlaneIndices->header = local_point_indices.header;
	basePlaneIndices->indices = local_point_indices.indices;

	//##################### HARD-CODED ###################

	const pcl::PointCloud<PointT>::Ptr aux_cloud(new pcl::PointCloud<PointT>());
    aux_cloud->header = my_cloud->header;
    aux_cloud->points = my_cloud->points;
    aux_cloud->width = my_cloud->width;
    aux_cloud->height = my_cloud->height;

	pcl::ExtractIndices<PointT> extractor;
	extractor.setInputCloud(aux_cloud);
	extractor.setIndices(basePlaneIndices);
//    extractor.setNegative(true);
	extractor.filter(*plane_cloud);

//    PointT myy_point;
//    int init_number = 0;
//    int max_number = 1000;
//    std::string point_test_string = "/home/furdek/PointTest";

//    for (int b = 0; b < 10; b++){
//    pcl::PointCloud<PointT>::Ptr point_test_cloud(new pcl::PointCloud<PointT>());
//    point_test_cloud->resize(plane_cloud->size());

////    std::cerr << "NEEEEEEEEEEEWWWWWWWWWWWW PAAAAAAAAAAAAARRRRRRRRTTTTTTTTTTTT" << std::endl;
//    for (size_t a = init_number; a < init_number + max_number ; a++){
//        myy_point = plane_cloud->at(a);
//        point_test_cloud->at(a) = myy_point;
//        std::cerr << "x: " << myy_point.x << " y: " << myy_point.y << " z: " << myy_point.z << std::endl;
//    }

//	std::stringstream ss;
//	ss << "/home/furdek/FINALEXTRACTEDPLANE.pcd";

//	std::cerr << "Writing" << std::endl;
//	writer.write(ss.str(), *plane_cloud);

//    std::stringstream ss1;
//    ss1 << point_test_string << b << ".pcd";
//    writer.write(ss1.str(), *point_test_cloud);

//    init_number += 1000;

//    }

    create_separate_planes(my_cloud);
    choose_plane_with_objects();

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

void trimPlane(){

    std::cerr << "TRIMPLANE BEGIN" << std::endl;

    float x_min;
    float x_max;
    float tolerance = 0.2;

    find3DPlaneMinMax(plane_cloud);

    x_min = table_x_min + tolerance;
    x_max = table_x_max - tolerance;

    for (size_t a = 0; a < plane_cloud->points.size(); a++){

        if ((plane_cloud->points[a].x < x_min) || (plane_cloud->points[a].x > x_max)){
            plane_cloud->points[a].x = NAN;
            plane_cloud->points[a].y = NAN;
            plane_cloud->points[a].z = NAN;
        }

    }

    std::cerr << "TRIMPLANE END" << std::endl;

}

void segmentObjects() {

	int k = 0;

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
            if (label_indices[i].indices.size() > 40000) {
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
				pcl::PointCloud<PointT> cluster;
				pcl::copyPointCloud(*pcl_current,
						euclidean_label_indices[i].indices, cluster);
				std::stringstream ss;
				std::cerr << "value of i for saving PCD: " << k << std::endl;
				ss << clusterPCDPath << k << ".pcd";
				pcl::io::savePCDFile(ss.str(), cluster);
				clusters.push_back(cluster);
				k++;
			}
		}

		std::cerr << "Nr of clusters: " << clusters.size() << std::endl;
		for (size_t i = 0; i < clusters.size(); i++) {
			cluster_cloud = clusters[i];
			std::cerr << "Nr of points in clouds:"
					<< cluster_cloud.height * cluster_cloud.width << std::endl;
		}

	}
}

void projectPointsOnPlane() {

	pcl::ProjectInliers<pcl::PointXYZ> proj;

	//################### TO CHANGE -- THIS IS HARD-CODED ##################
	model_pointer->header = model_coefficients[1].header;
	model_pointer->values = model_coefficients[1].values;
	//################### TO CHANGE -- THIS IS HARD-CODED ##################

	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(pcl_current);
	proj.setModelCoefficients(model_pointer);
	proj.filter(*cloud_projected);
	pcl::io::savePCDFile("/home/furdek/ProjectionProba.pcd", *cloud_projected);

//    findPlaneMinMax();
//	find3DPlaneMinMax();
}

void createConvexHull(const pcl::PointCloud<PointT>::ConstPtr my_cloud){

    pcl::PointCloud<PointT>::Ptr hull_vis_cloud(new pcl::PointCloud<PointT>());

//    plane_cloud_final->header = plane_cloud->header;
//    plane_cloud_final->points = plane_cloud->points;
//    plane_cloud_final->height = plane_cloud->height;
//    plane_cloud_final->width = plane_cloud->width;

    chull.setInputCloud(my_cloud);
//    chull.setInputCloud(*plane_cloud);
    chull.reconstruct(*cloud_hull);

    std::cerr << "CLOUD HULL SIZE: " << cloud_hull->size() << std::endl;


    hull_vis_cloud->resize(cloud_hull->points.size());
    for (size_t a = 0; a < cloud_hull->points.size(); a++){
        std::cerr << cloud_hull->points[a] << std::endl;
        hull_vis_cloud->points[a] = cloud_hull->points[a];
    }

    pcl::io::savePCDFile("/home/furdek/Hull.pcd", *hull_vis_cloud);
    std::cerr << "Hull cloud saved!" << std::endl;

}

void ExtractObjectsWithinHull(const pcl::PointCloud<PointT>::ConstPtr my_cloud, const pcl::ConvexHull<PointT>::PointCloud::ConstPtr my_hull){

    pcl::PointIndices my_Indices;

    std::cerr << "PRISM" << std::endl;

    my_Indices = inlier_indices[1];

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

    std::cerr << "HULL OUTPUT SIZE: " << hull_output->indices.size() << std::endl;

}

void extractHullOutliersFromProjectedCloud(const pcl::PointCloud<PointT>::ConstPtr cloud){

    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(cloud);
    extractor.setIndices(hull_output);
//    extractor.setNegative(true);
    extractor.filter(*projected_obj_cloud);
    pcl::io::savePCDFile("/home/furdek/ProjectionFinal.pcd", *projected_obj_cloud);

    std::cerr << "FINAL PROJECTION READY" << std::endl;


}

void extractPlanePointsFromCloud(const pcl::PointCloud<PointT>::ConstPtr cloud, const pcl::PointIndices::ConstPtr indices){

	 pcl::ExtractIndices<PointT> extractor;
	    extractor.setInputCloud(cloud);
        extractor.setIndices(indices);
	//    extractor.setNegative(true);
	    extractor.filter(*projected_obj_cloud);
        pcl::io::savePCDFile("/home/furdek/FINISH.pcd", *projected_obj_cloud);

        std::cerr << "Extracted Plane From Cloud Given!" << std::endl;

}

void final_filter(){

    int counter = 0;
    my_final->resize(projected_obj_cloud->size());

    for (size_t b = 0; b < projected_obj_cloud->size(); b++){
        PointT my_point;
        my_point = projected_obj_cloud->points[b];
        if (pcl::isXYPointIn2DXYPolygon(my_point, *cloud_hull)){
            std::cerr << counter << std::endl;
            my_final->points[b] = my_point;
            counter++;
        }
    }

    pcl::io::savePCDFile("/home/furdek/final.pcd", *my_final);


}

pcl::PointIndices::Ptr segment_final_plane(const pcl::PointCloud<PointT>::Ptr my_cloud){

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

      pcl::SACSegmentation<pcl::PointXYZ> seg;

      seg.setOptimizeCoefficients (true);

      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.007);

      seg.setInputCloud(my_cloud);
      seg.segment (*inliers, *coefficients);

      std::cerr << inliers->indices.size() << std::endl;

      return inliers;

}

void clustering() {

	std::cout << "Clustering" << std::endl;

	pcl::PCDReader reader;
	reader.read(probaString, *cloud_blob);
	pcl::fromPCLPointCloud2(*cloud_blob, *pcl_current);

	normalEstimation(pcl_current);

    segmentPlanes(pcl_current);

    segmentObjects();
    projectPointsOnPlane();

    pcl::io::savePCDFile("/home/furdek/ConvexHullBefore.pcd",
            *plane_cloud);

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

    createConvexHull(plane_cloud);
    ExtractObjectsWithinHull(cloud_projected, cloud_hull);
    extractHullOutliersFromProjectedCloud(cloud_projected);
    final_filter();
//    extractPlanePointsFromCloud(my_final);
    pcl::PointIndices::Ptr myInd (new pcl::PointIndices());
    myInd = segment_final_plane(my_final);
    std::cerr << my_final->size() << std::endl;
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

	clustering();

	return (0);
}
