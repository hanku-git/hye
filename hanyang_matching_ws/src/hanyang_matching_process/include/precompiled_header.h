
#include <pcl/common/eigen.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/parse.h>

// #include <pcl/io/io.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>

// loadPolygonPLY
#include <pcl/type_traits.h>
#include <pcl/pcl_exports.h>
#include <pcl/io/vtk_lib_io.h>
#include <vtkPolyLine.h>
#include <vtkPLYReader.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
// loadPolygonPLY

#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>

//// PCL feature
#include <pcl/features/normal_3d.h> // normal estimation
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/pfh.h> // PFH
#include <pcl/features/fpfh.h> // FPFH
#include <pcl/features/spin_image.h> // SpinImage
#include <pcl/features/3dsc.h> // shape context
#include <pcl/features/impl/3dsc.hpp> // shape context

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/surface/gp3.h>
#include <pcl/surface/concave_hull.h>


#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/default_convergence_criteria.h>