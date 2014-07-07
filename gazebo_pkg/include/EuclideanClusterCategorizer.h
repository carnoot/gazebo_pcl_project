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
#include <pcl/filters/radius_outlier_removal.h>
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

std::string initialPCDPath;
std::string probaString;
std::string clusterPCDPath;

void initTableMinMaxValues();
void findPlaneMinMax();
void outlier_removal_from_plane(const pcl::PointCloud<PointT>::Ptr);
void find3DPlaneMinMax(const pcl::PointCloud<PointT>::Ptr cloud);

struct pclStruct {

	int plane_position;
//    std::vector<pcl::PointIndices>::Ptr my_indices;
	pcl::PointCloud<PointT>::CloudVectorType my_clusters;

};

// ################# ABSOLUTELY NECESSARRY #################
pcl::PointIndices::Ptr plane_with_cluster_plane_indices(new pcl::PointIndices);
pcl::ConvexHull<PointT>::PointCloud::Ptr cloud_hull(
		new pcl::ConvexHull<PointT>::PointCloud());
pcl::PointCloud<PointT>::Ptr projected_obj_cloud(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr pcl_current(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::Ptr cloud_projected(new pcl::PointCloud<PointT>());
pcl::PointCloud<PointT>::CloudVectorType clusters;
pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);

std::vector<pcl::PointIndices> label_indices;
std::vector<pcl::PointIndices> inlier_indices;
std::vector<pcl::PlanarRegion<PointT>,
		Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
std::vector<pcl::ModelCoefficients> model_coefficients;
std::vector<pcl::PointIndices> cluster_indices;
int main_plane_index = 0;
// ################# ABSOLUTELY NECESSARRY #################

// ################# NOT USED AT THE MOMENT #################
pcl::PointIndices::Ptr basePlaneIndices(new pcl::PointIndices);
std::vector<pcl::PointIndices> boundary_indices;
// ################# NOT USED AT THE MOMENT #################
