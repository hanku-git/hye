/**
 * @file pcl_processing.h
 * @brief 점군 데이터의 연산을 수행하기 위한 헤더파일
 */
#ifdef max
#undef max
#undef min
#endif 

#pragma once

#include <iostream>
#include <string>
#include <algorithm>
#include <stdio.h>

// #include <pcl/io/io.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/common/transforms.h>
// #include <pcl/common/centroid.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/console/time.h>
// #include <pcl/console/print.h>
// #include <pcl/console/parse.h>
// #include <pcl/console/parse.h>
// #include <pcl/segmentation/supervoxel_clustering.h>
// #include <pcl/search/search.h>
// #include <pcl/search/kdtree.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/features/principal_curvatures.h>
// #include <pcl/features/moment_of_inertia_estimation.h>
// #include <pcl/features/fpfh.h>
// #include <pcl/segmentation/region_growing.h>
// #include <pcl/segmentation/region_growing_rgb.h>
// #include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/passthrough.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/filters/project_inliers.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/segmentation/extract_clusters.h>

// #include <pcl/segmentation/conditional_euclidean_clustering.h>
// #include <pcl/surface/gp3.h>
// #include <pcl/surface/concave_hull.h>
// #include <pcl/pcl_exports.h>
// #include <pcl/io/vtk_lib_io.h>
// #include <pcl/io/obj_io.h>
// #include <vtkPolyLine.h>

// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/registration/correspondence_estimation.h>
// #include <pcl/registration/correspondence_rejection_sample_consensus.h>
// #include <pcl/registration/ia_ransac.h>
// #include <pcl/registration/icp.h>
// #include <pcl/registration/default_convergence_criteria.h>

#include "precompiled_header.h"
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTreeStamped.h>
#include <octomap/math/Utils.h>
#include <octomap/MapCollection.h>

#include <boost/thread.hpp>

#include "module/KUPCLMath.h"
#include "module/data_class/object_data.h"

#include <filesystem>  // C++17

using json = nlohmann::json;
namespace fs = std::filesystem;

/**
 * @class PCLPROCESSING
 * @brief 점군 데이터 처리 모듈 클래스
 * @ingroup PCL_PROCESSING
 */
class PCLPROCESSING
{
public:
	PCLPROCESSING();
	~PCLPROCESSING();

public:
	KUPCLMATH m_math;
	int m_ExplorationNum;
	int m_OptCapturePosNum;
	int m_rescanNum;
	void initCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr);
	void initCloudPtr(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_ptr);
	void initCloudPtr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr);
	void initCloudPtr(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud_ptr);
    void initCloudPtr(pcl::PointIndices::Ptr &cloud_ptr);
	void initOctomapPtr(octomap::OcTree* &map_ptr);
    void initOctomapPtrWithSize(octomap::OcTree* &map_ptr, double octo_size);
    void genOctomapCollection(octomap::OcTree* &map_1, octomap::OcTree* &map_2, octomap::MapCollection<octomap::MapNode<octomap::OcTree>> &collection);
    void genOctomapCollection2(octomap::OcTree* &map_1, octomap::OcTree* &map_2, octomap::MapCollection<octomap::MapNode<octomap::OcTree>> &collection, const std::vector<double> &pose1, const std::vector<double> &pose2);
    void initOctomapCollection(octomap::MapCollection<octomap::MapNode<octomap::OcTree>>* &collection);
	void genCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr, pcl::PointCloud<pcl::PointXYZ> &cloud_input);
	void genCloudPtr2(std::string cloudname, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr);
	void genCloudPtr2(std::string cloudname, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_ptr);
	void genCloudPtr2(std::string cloudname, pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud_ptr);
	void genPLY2CloudPtr(std::string cloudname, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr);
	void genPLY2CloudPtr(std::string cloudname, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_ptr);
	void genPLY2CloudPtr(std::string cloudname, pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud_ptr);
	void genMergedCloud(std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ>> &cloud_input, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr);
	void genCloudSet(std::string cloudname, std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ>> &output);
	//void genOctomapPtr(std::string cloudname, octomap::OcTree* map_ptr);
	void genOctomapPtr(std::string cloudname, octomap::OcTree* &map_ptr);
	void loadCloud(pcl::PointCloud<pcl::PointXYZ> &cloud_out, std::string file_path);
	void copyCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out);


    ////////////////////////////
    //// VIEWPLANNING
	void normalEstimation(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::Normal> &output, int paramK, double paramR, int method);
	void normalEstimation(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src, pcl::PointCloud<pcl::Normal> &output, int paramK, double paramR, int method);
    void cloudNormalEst2Measured(const pcl::PointCloud<pcl::PointXYZ> &cloud_src, pcl::PointCloud<pcl::PointXYZRGBNormal> &output, int paramK, const Eigen::Vector3d &sensor_origin, double thre_angle);
    void cloudNormalEst2Measured(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_src, pcl::PointCloud<pcl::PointXYZRGBNormal> &output, int paramK, const Eigen::Vector3d &sensor_origin, double thre_angle);
    void cloudNormalCompareAndFilp(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_ref, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_proc, int paramK, double thre_angle);
    void cloudNormalCompareAndFilpRadiusSearch(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_ref, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_proc, double radius, double thre_angle);
    void segmentationRGS(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out, std::vector<pcl::PointIndices> &indices_out, CSegParameters &param, bool b_plot);
    void segmentationRGSv2(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out, std::vector<pcl::PointIndices> &indices_out, CSegParameters &param, bool b_plot);
	void segmentationColorRGS(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out, std::vector<pcl::PointIndices> &indices_out, CSegParameters &param, bool b_plot);
	void segmentationVCCS(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out, std::vector<pcl::PointIndices> &indices_out, CSegParameters &param, bool b_plot);
    void segmentationEuclideanClusterExtraction(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out, std::vector<pcl::PointIndices> &indices_out, bool do_plane_removal, CSegParameters &param, bool b_plot);
    void segmentationConditionalEuclideanClustering(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out, std::vector<pcl::PointIndices> &indices_out, CSegParameters &param, bool b_plot);
    void extractClusterList(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, const std::vector<pcl::PointIndices> &indices_in, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PointXYZRGBA> > &output, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_merge);
    void extractClusterList(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, const std::vector<pcl::PointIndices> &indices_in, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>, Eigen::aligned_allocator<pcl::PointXYZRGBNormal> > &output, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_merge);
	void extractCenter(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Vector3d &output, int &idx);
	void extractCenter(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Vector3f &output, int &idx);
    void extractCenterWithNormal(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, Eigen::Vector3f &center, Eigen::Vector3f &normal, int &idx);
    void extractNearestPointWithNormal(const Eigen::Vector3f &pt_query, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, Eigen::Vector3f &pt_nearest, Eigen::Vector3f &normal_nearest, int &idx);
    void genOctoMapCAD(CPLANNINGDATA &plan_data);
    void genOctoMapCAD2(CPLANNINGDATA &plan_data, double maxRange);
	octomap::OcTree* genTree(float resolution);
    bool checkSinglePointOccupancy(const Eigen::Vector3d& pt, octomap::OcTree* OctoMap_CAD, double thre_map);
    int checkSinglePointOccupancyUsingMapCollection(const octomap::point3d& q, octomap::MapCollection<octomap::MapNode<octomap::OcTree>> &collection);
    void extractOBBLength(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, std::vector<double> &ascending_order_sorted_list, bool b_plot);
    void extractOBBTransform(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Matrix4f &output, bool b_plot);
    ////////////////////////////
    //// ALIGNMENT
    void modelMatchingBottomPlaneShapedPreProcessing(const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot);
    void modelMatchingPreProcessingFlipCheckAndCentroid(CTARGET_OBJECT_DATA* &target_object_data, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot);

    void modelMatchingComplexShapedPreProcessing(const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot);
    void matchingMajorAxisTranslation(const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot);
    void matchingMajorAxisTranslationMinMax(const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot);
    
    void matchingPlaneRotationUsingOBB(const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot);
    void modelMatchingReg2CADUsingQuickAlignBinPicking(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot);
    void modelMatchingReg2CADUsingQuickAlignBinPickingUsingJson(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, const std::string &quick_align_json_path, Eigen::Matrix4f &T_quick_out, Eigen::Matrix4f &T_out, bool b_plot);
    void modelMatchingReg2CADUsingQuickAlignBinPickingWithGICP(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot);
    void modelMatchingReg2CADUsingQuickAlignBinPickingWithNormalVer1(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_reg, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot);
    void modelMatchingReg2CADUsingQuickAlignBinPickingWithNormalVer2(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_reg, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD, const Eigen::Matrix4f &T_in, const std::string &quick_align_json_path, Eigen::Matrix4f &T_quick_out, Eigen::Matrix4f &T_out, bool b_plot);
    void modelMatchingReg2CADUsingQuickAlignBinPickingWithNormalVer2OnlyQuickAlign(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_reg, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot);

    Eigen::Matrix4f jsonToMatrix4f(const json& jmat);
    void printMatrixAsJson(const Eigen::Matrix4f& mat, const std::string& name);
    json eigenMat4ToJson(const Eigen::Matrix4f& M);
    Eigen::Matrix4f jsonToMat4(const json& jmat);
    std::vector<Eigen::Matrix4f> loadTransformsFromJson(
        const std::string& path,
        const std::string& key_prefix = "T");
    std::vector<Eigen::Matrix4f> loadTransformsFromJsonWithCount(
        const std::string& path, int* out_count = nullptr, const std::string& prefix = "T");
    std::string appendTransformAndUpdateCount(
            const std::string& path, const Eigen::Matrix4f& T, const std::string& prefix = "T");

    //// ICP
    void matchingReg2CADUsingICP4(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, Eigen::Matrix4f &T_out, bool b_plot);
    void matchingReg2CADUsingICP4NoEstNormal(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_reg, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD, Eigen::Matrix4f &T_out, bool b_plot);
    void matchingReg2CADUsingICP5NoEstNormal(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_reg, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD, Eigen::Matrix4f &T_out, bool b_plot);
    
    //// GICP (Generalized ICP)
    void matchingReg2CADUsingGeneralizedICP1(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, Eigen::Matrix4f &T_out, bool b_plot);
    void matchingReg2CADUsingGeneralizedICP1NoEstNormal(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_reg, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD, Eigen::Matrix4f &T_out, bool b_plot);


    void matchingSurfaceCheckAndICPVer1(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZ> &cloud_major_segment, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_reg, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD, pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &md_KdTree, const Eigen::Matrix4f &T_in, const Eigen::Matrix4f &T_B2S, Eigen::Matrix4f &T_out, bool b_plot);
    void matchingSurfaceCheckAndICPVer2(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_src, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD, pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &md_KdTree, const Eigen::Matrix4f &T_in, const Eigen::Matrix4f &T_B2S, Eigen::Matrix4f &T_out, bool b_plot);

    bool checkObjectCloudFlip(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_src, const Eigen::Vector3f &vec_reference, double max_cad_z, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out, Eigen::Vector3f &vec_flip_check, Eigen::Matrix4f &T_now);
    ////////////////////////////
    //// PLOT
    void plotCloud(pcl::PointCloud<pcl::PointXYZ> &cloud_in, float view_scale, std::string plot_name);
    void plotCloudWithFrame(pcl::PointCloud<pcl::PointXYZ> &cloud_in, float view_scale, float line_length, std::string plot_name);
    void plotCloudWithFrame(pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, float view_scale, float line_length, std::string plot_name);
	void plotCloudWithNormal(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, float view_scale, std::string plot_name);
    void plotRGBCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, float view_scale, double pt_size, std::string plot_name);
    void plotProcessedCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_ref, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_meas_before, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_meas_after, const std::string &fig_title);
	void plotCloudCompare(const pcl::PointCloud<pcl::PointXYZ> &cloud_in1, const pcl::PointCloud<pcl::PointXYZ> &cloud_in2, const pcl::PointCloud<pcl::PointXYZ> &cloud_in3, float view_scale, std::string plot_name);
    bool plotEnvironmentWithRobotBase(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const Eigen::Matrix4f &T_base, std::vector<Eigen::Vector3f> &robot_joint_set);
    bool plotEnvironmentWithRobotBase2(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_base, std::vector<Eigen::Vector3f> &robot_joint_set, const pcl::PointCloud<pcl::PointXYZRGB> &cloud_query_base);
    bool plotEnvironmentWithRobotBase3(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const Eigen::Matrix4f &T_base, std::vector<Eigen::Vector3f> &robot_joint_set, const pcl::PointCloud<pcl::PointXYZRGB> &cloud_query_base, const Eigen::Vector4f &bin_center_wrt_optimal_base_frame);
    bool plotCalibrationPoseWithBin(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const Eigen::Matrix4f &T_base, std::vector<Eigen::Vector3f> &robot_joint_set, bool b_plot);
    bool plotFinalPoseResults(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const size_t &threshold, std::vector<size_t> &collision_count_set, bool b_plot);
    bool plotFinalPoseResultsMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, const Eigen::Matrix4f &T_tgt, const size_t &threshold, std::vector<size_t> &collision_count_set, bool b_plot);
    
    bool plotFinalPoseResultsWithKDTree(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const size_t &threshold, std::vector<size_t> &collision_count_set, std::string plot_name, bool b_plot);
    bool plotFinalPoseResultsNoKDTree(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const size_t &threshold, std::vector<size_t> &collision_count_set, std::string plot_name, bool b_plot);
    bool plotFinalPoseResultsNoKDTreeForEscape(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const Eigen::Matrix4f &T_proc, const size_t &threshold, std::vector<size_t> &collision_count_set, std::string plot_name, bool b_plot);



    //// PRINT
    void printFPFHCompare(const pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_meas, const pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_ref, const std::string &str, bool is_compare);

//// RVBUST Scanner
    void removeNanPoints(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out);
    void removeNanPoints(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out);
    void removeNanPoints(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out);
    void removeNanPoints(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out);

    void filterCloudUsingViewFrustum(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, const std::vector<double> view_frustum, pcl::PointCloud<pcl::PointXYZ> &cloud_out);
    void filterCloudUsingViewFrustum(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, const std::vector<double> view_frustum, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out);
    void filterCloudUsingViewFrustum(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, const std::vector<double> view_frustum, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out);
    void filterCloudUsingViewFrustum(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, const std::vector<double> view_frustum, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out);

};