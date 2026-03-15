#include <iostream>
#include <array>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <boost/thread.hpp>
#include <time.h>
#include <ament_index_cpp/get_package_share_directory.hpp>


// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "module/data_class/object_data.h"
#include "module/path_planning.h"
#include "module/pcl_processing.h"
#include "module/KUPCLMath.h"
#include "module/image2cloud_processing.h"
#include "module/optimization_module.h"
#include "state_estimation/contact_state_estimator.h"
#include <nlohmann/json.hpp> // JSON

// C++ Plot MIT license, https://github.com/InductiveComputerScience/pbPlots
#include "pbPlots/pbPlots.hpp" 
#include "pbPlots/supportLib.hpp"

#include <octomap/OcTree.h>

#include <mutex>



class TEMPLATE_MATCHING
{
public: 
	TEMPLATE_MATCHING();
	~TEMPLATE_MATCHING();
public:

    std::mutex mtx;

	KUPCLMATH m_math;
    IMAGE2CLOUDPROCESSING m_image2cloud_proc;
    PCLPROCESSING m_pcl_proc;
    PATHPLANNING m_pathplanning;
	CRobotModel model_;
    COPTIMODULE m_opt_module;
    CContactStateEstimator m_state_evaluation;


	//// Assembly state estimation
    std::vector<CTARGET_OBJECT_DATA, Eigen::aligned_allocator<CTARGET_OBJECT_DATA>> m_template_list;
    std::vector<CTARGET_OBJECT_DATA*> m_ptr_template_list;
    // std::vector<CTARGET_OBJECT_DATA> m_template_tool_list; // 아래와 같이 선언해야 함.
    std::vector<CTARGET_OBJECT_DATA, Eigen::aligned_allocator<CTARGET_OBJECT_DATA>> m_template_tool_list;
    std::vector<CTARGET_OBJECT_DATA*> m_ptr_template_tool_list;
    std::string m_package_path;
    std::string m_home_folder_path;

    GRIPPER_DATA m_grp_data;
    BIN_DATA m_bin_data;
    COPTCONDITIONLIST m_optimization_input;

    void icp_matchingInitProcess(bool do_measured_cloud_process, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_scan, const int target_id, const std::string target_name, const Eigen::VectorXf q_scan, bool do_scan_sampling, size_t sampling_num, bool b_plot);
    bool icp_matchingSingleProcessVer3(std::shared_ptr<bool> &ptr_is_pose_assigned, std::vector<size_t> &mask_confident_idx_set, std::vector<size_t> &pre_selected_idx_set, const size_t thread_id, CTARGET_OBJECT_DATA &object_data, bool b_plot);

    bool gicp_matchingSingleProcessVer1(std::shared_ptr<bool> &ptr_is_pose_assigned, std::vector<size_t> &mask_confident_idx_set, std::vector<size_t> &pre_selected_idx_set, const size_t thread_id, CTARGET_OBJECT_DATA &object_data, bool b_plot);

	void icp_matchingBinPicking(bool do_measured_cloud_process, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_scan, const int target_id, const std::string target_name, const Eigen::VectorXf q_scan, std::vector<double> &vec_out, std::vector<std::vector<double>> &vec_out_sub_pose_set, std::vector<double> &zig_pose, double &matching_accuracy, bool do_scan_sampling, size_t sampling_num, bool b_plot);
	void icp_matchingMultipleBinPicking(bool do_measured_cloud_process, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_scan, const int target_id, const std::string target_name, const Eigen::VectorXf q_scan, std::vector<double> &vec_out, std::vector<std::vector<double>> &vec_out_sub_pose_set, std::vector<double> &zig_pose, double &matching_accuracy, bool do_scan_sampling, size_t sampling_num, bool b_plot);
    void icp_matchingGlobalToolCalibration(bool do_measured_cloud_process, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_scan, const int target_id, const std::string target_name, const Eigen::VectorXf q_scan, std::vector<double> &vec_out, std::vector<std::vector<double>> &vec_out_sub_pose_set, double &matching_accuracy, bool do_scan_sampling, size_t sampling_num, bool b_plot);

    std::vector<double> transformSensorPose2RobotPose(const Eigen::Matrix4f &T_in, const Eigen::VectorXf q_scan, const int target_id, const std::string target_name);

    // Load initial data
    void importGripperJSON(GRIPPER_DATA &gripper_data);
    void importBaseZigJSON(CTARGET_OBJECT_DATA* &target_object_data);
    void importBinJSON(BIN_DATA &bin_data);
    void importOptimizationJSON(COPTCONDITIONLIST &opt_data);
    void importGlobalCalibrationToolJSON(CTARGET_OBJECT_DATA* &global_calibration_tool_data, GRIPPER_DATA &gripper_data, BIN_DATA &bin_data);

    void setGripperBinData(CTARGET_OBJECT_DATA* &target_object_data, GRIPPER_DATA &gripper_data, BIN_DATA &bin_data);

    void importTargetJSONFiles(CTARGET_OBJECT_DATA* &target_object_data, GRIPPER_DATA &gripper_data, BIN_DATA &bin_data);

    void importRobotConfigJSON(CTARGET_OBJECT_DATA* &target_object_data, std::string path);
    void importSensorConfigJSON(CTARGET_OBJECT_DATA* &target_object_data, std::string path);
    void importTargetObjectJSON(CTARGET_OBJECT_DATA* &target_object_data, std::string path);
    void importBinConfigJSON(CTARGET_OBJECT_DATA* &target_object_data, std::string path);
    void importTargetObjectCADModel(CTARGET_OBJECT_DATA* &target_object_data);
    
    void doMeasuredCloudProcessing(CTARGET_OBJECT_DATA* &target_object_data);
    void doAlignmentScan2RobotBaseFrame(CTARGET_OBJECT_DATA* &target_object_data);
    Eigen::Matrix4f doAlignmentScan2RobotBaseFrame2(CTARGET_OBJECT_DATA* &target_object_data, const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out);
    Eigen::Matrix4f doAlignmentScan2RobotBaseFrame2MultiThreaded(CTARGET_OBJECT_DATA &target_object_data, const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out);

    void doMeasuredCloudFilteringUsingBinBoundingBox(CTARGET_OBJECT_DATA* &target_object_data);
    void doMeasuredCloudFilteringUsingBinBoundingBoxWithMargin(CTARGET_OBJECT_DATA* &target_object_data, double margin);
    void doMeasuredCloudFilteringUsingBinBoundingBoxForCollisionCheck(CTARGET_OBJECT_DATA* &target_object_data, const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out);
    void doPreMatchingProcess(CTARGET_OBJECT_DATA* &target_object_data);
    void calMeasuredCloudFeatureForQuickAlign(CTARGET_OBJECT_DATA* &target_object_data);
    void doTemplateMatchingProcess(CTARGET_OBJECT_DATA* &target_object_data);

    void doPreMatchingProcessMultiThreaded(CTARGET_OBJECT_DATA &target_object_data);
    void calMeasuredCloudFeatureForQuickAlignMultiThreaded(CTARGET_OBJECT_DATA &target_object_data);
    void doTemplateMatchingProcessMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, const size_t thread_id);
    void doTemplateMatchingProcessUsingGICPMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, const size_t thread_id);
    void doTemplateMatchingProcessUsingGICPWithoutQuickAlignMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, const size_t thread_id);

    
    void doTemplateMatchingProcessBefore(CTARGET_OBJECT_DATA* &target_object_data);

    void doBinMatching(CTARGET_OBJECT_DATA* &target_object_data);
    void doMeasuredBinCloudProcessing(CTARGET_OBJECT_DATA* &target_object_data);

    void doBaseZigMatching(CTARGET_OBJECT_DATA* &target_object_data);
    void doMeasuredBaseZigProcessing(CTARGET_OBJECT_DATA* &target_object_data);

    void doMeasuredEyeInHandZigProcessing(CTARGET_OBJECT_DATA* &target_object_data);
    void doEyeInHandZigMatching(CTARGET_OBJECT_DATA* &target_object_data);

    void getCalibrationRobotJSPosition(CTARGET_OBJECT_DATA* &target_object_data);
    void setCalibrationRobotJSPosition(CTARGET_OBJECT_DATA* &target_object_data);
    void doRobotCameraCalibrationMatching(CTARGET_OBJECT_DATA* &target_object_data);
    void doMeasuredCalibrationToolCloudProcessing(CTARGET_OBJECT_DATA* &target_object_data);
    void doMeasuredCloudProcessingForEvaluatingGraspingPose(CTARGET_OBJECT_DATA* &target_object_data);

    bool calGraspingPose(CTARGET_OBJECT_DATA* &target_object_data);
    bool calGraspingPoseMultiThreadedVer2(std::shared_ptr<bool> &ptr_is_pose_assigned, CTARGET_OBJECT_DATA &target_object_data, pcl::PointCloud<pcl::PointXYZ> &cloud_env_check_collision);
    
    
    bool calCandidateGraspingPoseForOptimization(CTARGET_OBJECT_DATA* &target_object_data);
    
    Eigen::Matrix4f transformGraspingPoseForComplexShape(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_O2E);
    Eigen::Matrix4f transformGraspingPoseForComplexShapeMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, const Eigen::Matrix4f &T_O2E);
    
    bool modify2UserDefinedOrientationForComplexShape(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_O2E, Eigen::Matrix4f &T_out);
    bool modify2UserDefinedOrientationForComplexShapeMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, const Eigen::Matrix4f &T_O2E, Eigen::Matrix4f &T_out);
    bool modify2UserDefinedFixedOrientationForComplexShapeMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, const Eigen::Matrix4f &T_O2E, Eigen::Matrix4f &T_out);
    Eigen::Matrix4f modify2UserDefinedOrientationForCylinderShape(CTARGET_OBJECT_DATA* &target_object_data, int direction);
    Eigen::Matrix4f modify2UserDefinedOrientationForCylinderShapeMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, int direction);
    
    Eigen::Matrix4f generateSubGraspingPose(CTARGET_OBJECT_DATA* &target_object_data, int direction);
    Eigen::Matrix4f generateSubGraspingPoseMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, int direction);



    Eigen::Matrix4f genCollisionFreeGraspingPoseForCylinderShape(CTARGET_OBJECT_DATA* &target_object_data, Eigen::Matrix4f &T_tgt);
    Eigen::Matrix4f genCollisionFreeGraspingPoseForCylinderShapeMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, Eigen::Matrix4f &T_tgt, pcl::PointCloud<pcl::PointXYZ> &cloud_env_check_collision);
    Eigen::Matrix4f genCollisionFreeGraspingPoseForComplexShape(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt);
    Eigen::Matrix4f genCollisionFreeGraspingPoseForComplexShapeMultiThreadedVer3(std::shared_ptr<bool> &ptr_is_pose_assigned, CTARGET_OBJECT_DATA &target_object_data, pcl::PointCloud<pcl::PointXYZ> &cloud_env_check_collision);

    // new
    void genCandidateGraspingPoseForOptimizationForCylinder(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &candidate_approach_pose_set, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &candidate_grasping_pose_set);
    Eigen::Matrix4f genCandidateGraspingPoseForOptimizationForComplexShape(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt);

    bool checkGripperCollisionUsingBoundingBox(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, double candidate_angle, Eigen::Matrix4f &T_out);
    bool checkGripperCollisionUsingVoxelMap(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const size_t &threshold, std::vector<size_t> &collision_count_set, bool b_plot);
    bool checkGripperCollisionUsingVoxelMapWithRvizCloud(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const size_t &threshold, std::vector<size_t> &collision_count_set, bool b_plot);
    bool checkGripperCollisionUsingVoxelMapWithoutObject(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const size_t &threshold, std::vector<size_t> &collision_count_set, bool b_plot);
    bool checkGripperCollisionUsingVoxelMapWithObject(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const Eigen::Matrix4f &T_proc, const size_t &threshold, std::vector<size_t> &collision_count_set, bool b_plot);
    bool checkCollisionApproach2Escape(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt);

    bool checkGripperCollisionUsingBoundingBoxMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, const Eigen::Matrix4f &T_tgt, double candidate_angle, Eigen::Matrix4f &T_out);
    bool checkGripperCollisionUsingVoxelMapMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, const Eigen::Matrix4f &T_tgt, pcl::PointCloud<pcl::PointXYZ> &cloud_env_check_collision, const size_t &threshold, std::vector<size_t> &collision_count_set, bool b_plot);
    bool generateRvizCloudGripperCollisionUsingVoxelMapMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, const Eigen::Matrix4f &T_tgt, const size_t &threshold, std::vector<size_t> &collision_count_set, bool b_plot);
    bool checkGripperCollisionUsingVoxelMapWithoutObjectMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, const Eigen::Matrix4f &T_tgt, pcl::PointCloud<pcl::PointXYZ> &cloud_env_check_collision, const size_t &threshold, std::vector<size_t> &collision_count_set, bool b_plot);
    bool checkGripperCollisionUsingVoxelMapWithObjectMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, const Eigen::Matrix4f &T_tgt, const Eigen::Matrix4f &T_proc, pcl::PointCloud<pcl::PointXYZ> &cloud_env_check_collision, const size_t &threshold, std::vector<size_t> &collision_count_set, bool b_plot);
    bool checkCollisionApproach2EscapeMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, const Eigen::Matrix4f &T_tgt, pcl::PointCloud<pcl::PointXYZ> &cloud_env_check_collision);

    bool estimatePoseWithoutCollisionForCylinderShape(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const size_t &threshold, std::vector<size_t> &collision_count_set, Eigen::Matrix4f &T_out);
    bool estimatePoseWithoutCollisionForCylinderShapeMultiThreadedVer2(CTARGET_OBJECT_DATA &target_object_data, const Eigen::Matrix4f &T_tgt, pcl::PointCloud<pcl::PointXYZ> &cloud_env_check_collision, const size_t &threshold, std::vector<size_t> &collision_count_set, Eigen::Matrix4f &T_out);
    bool estimatePoseWithoutCollisionForComplexShape(CTARGET_OBJECT_DATA* &target_object_data, const size_t &threshold, std::vector<size_t> &collision_count_set, Eigen::Matrix4f &T_out);
    bool estimatePoseWithoutCollisionForComplexShapeMultiThreadedVer3(std::shared_ptr<bool> &ptr_is_pose_assigned, CTARGET_OBJECT_DATA &target_object_data, pcl::PointCloud<pcl::PointXYZ> &cloud_env_check_collision, const size_t &threshold, std::vector<size_t> &collision_count_set, Eigen::Matrix4f &T_out);
    


    bool genWorkspaceGripperViewCloud(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const size_t &threshold, std::vector<size_t> &collision_count_set, bool b_plot);
    
    bool genWorkspaceEnvViewCloudBefore(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const size_t &threshold, std::vector<size_t> &collision_count_set);
    void genWorkspaceEnvViewCloud(CTARGET_OBJECT_DATA* &target_object_data);
    
    bool doScanCloudProcessingForCollisionCheck(CTARGET_OBJECT_DATA* &target_object_data);
    bool generateEnvCloudFromDatasetForCollisionCheck(CTARGET_OBJECT_DATA* &target_object_data);
    void mergeMatchingCAD2ScanResults(CTARGET_OBJECT_DATA* &target_object_data);
    void removeObjectMeasuredCloudUsingMaskForCollisionCheck(CTARGET_OBJECT_DATA* &target_object_data);
    void removeObjectMeasuredCloudUsingOBBForCollisionCheck(CTARGET_OBJECT_DATA* &target_object_data);
    void mergeObjectCADCloudForCollisionCheck(CTARGET_OBJECT_DATA* &target_object_data, bool b_color);

    void removeObjectMeasuredCloudUsingMaskForCollisionCheckMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, pcl::PointCloud<pcl::PointXYZ> &cloud_out);
    void removeObjectMeasuredCloudUsingOBBForCollisionCheckMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, pcl::PointCloud<pcl::PointXYZ> &cloud_out);
    void mergeObjectCADCloudForCollisionCheckMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, bool b_color, pcl::PointCloud<pcl::PointXYZ> &cloud_out);


    Eigen::Matrix4f rotatePose(const double &angle, CTARGET_OBJECT_DATA* &target_object_data, Eigen::Matrix4f &pose, int direction);
    Eigen::Matrix4f rotatePoseMultiThreaded(const double &angle, CTARGET_OBJECT_DATA &target_object_data, Eigen::Matrix4f &pose, int direction);
    
    void generateVoxelMap(CTARGET_OBJECT_DATA* &target_object_data); // Voxel map using Octomap
    void generateVoxelMap2(CTARGET_OBJECT_DATA* &target_object_data, const pcl::PointCloud<pcl::PointXYZ> &cloud_in, octomap::OcTree* OctoMap_CAD, pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &md_KdTree, double map_voxel_length);
    void generateVoxelMap3(std::string file_path, const pcl::PointCloud<pcl::PointXYZ> &cloud_in, octomap::OcTree* &OctoMap_CAD, pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &md_KdTree, double map_voxel_length);
    double evaluateMatchingResults(const std::string &color, pcl::PointCloud<pcl::PointXYZRGB> &cloud_matching_results, octomap::OcTree* OctoMap_CAD);
    double evaluateMatchingResultsWithKDTree(const std::string &color, pcl::PointCloud<pcl::PointXYZRGB> &cloud_matching_results, octomap::OcTree* OctoMap_CAD, pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &md_KdTree, double thre_dist);
    double extractOutlierCloudFromMatchingResults(const std::string &color, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_matching_results, octomap::OcTree* OctoMap_CAD, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out);



	// Load CAD model
	void importCADNormalModel(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out_downsampled, const Eigen::Matrix4f &T_Base2CAD, const std::string &file_path, const std::string &object_name, float downsample_size);
    void importCADNormalModel2(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out, const Eigen::Matrix4f &T_Base2CAD, const std::string &file_input_path, const std::string &file_output_path);
    void importCADNormalModel3(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out, const Eigen::Matrix4f &T_Base2CAD, const std::string &file_input_path);
    
	std::vector<std::vector<double>> curvatureEstimation3(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, double limit_angle, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointCloud<pcl::PointXYZRGBNormal> &output, int paramK, double paramR);
	void genSegmentNormalFromCAD(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD_mesh, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD, pcl::KdTreeFLANN<pcl::PointXYZRGBNormal, flann::L2_Simple<float>> &model);

	// Feature extraction
	void extractFeaturePFH(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, pcl::PointCloud<pcl::PFHSignature125> &output);
	void extractFeatureFPFH(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, pcl::PointCloud<pcl::FPFHSignature33> &output);
	void extractFeatureSpinImage(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, pcl::PointCloud<pcl::Histogram<153>> &output);
	void extractFeatureShapeContext(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, pcl::PointCloud<pcl::ShapeContext1980> &output);

    void saveBin2SensorFrameData2JSON(const std::vector<double> &pose_bin_to_sensor_frame);
    void saveNominalBin2RobotBaseFrameData2JSON(const std::vector<double> &pose_bin_to_base_frame);
    void saveNominalRobotBase2SensorFrameData2JSON(const Eigen::Matrix4f &T_in);
    void saveCalibratedRobotBase2SensorFrameData2JSON(const Eigen::Matrix4f &T_in);
    void saveBase2SensorFrameData2JSON(const Eigen::Matrix4f &T_in);
    void saveEndEffector2SensorFrameData2JSON(const Eigen::Matrix4f &T_in);
    void saveBinBoundingBoxForBaseFrameData2JSON(CTARGET_OBJECT_DATA* &target_object_data, const std::vector<double> &vec_in);
    void saveBin2RobotBaseFrameData2JSON(const std::vector<double> &pose_bin_to_base_frame);

    void saveVectorData2JSON(std::vector<std::vector<double>> &data, std::string file_path);
    bool loadVectorData2JSON(std::vector<std::vector<double>> &data, std::string file_path);
    void saveEigenData2CSV(std::vector<Eigen::VectorXf> &data, std::string file_path);
    void saveVectorData2CSV(std::vector<std::vector<double>> &data, std::string file_path);
    bool loadEigenData2CSV(std::vector<Eigen::VectorXf> &data, std::string file_path);
    void saveIndexData2CSV(std::vector<size_t> &data, std::string file_path, std::string file_name);
    bool loadIndexData2CSV(std::vector<size_t> &data, std::string file_path, std::string file_name);
    void selectMaskRCNN(CTARGET_OBJECT_DATA* &target_object_data, pcl::PointCloud<pcl::PointXYZ> &cloud_mask, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_mask_rgbn, pcl::PointCloud<pcl::PointXYZ> &cloud_mask_processed, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_mask_processed_rgbn);
    void selectMaskSAM(CTARGET_OBJECT_DATA* &target_object_data, pcl::PointCloud<pcl::PointXYZ> &cloud_mask, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_mask_rgbn, pcl::PointCloud<pcl::PointXYZ> &cloud_mask_processed, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_mask_processed_rgbn);
    void selectMaskSAMVer2(CTARGET_OBJECT_DATA* &target_object_data);
    void selectMaskSAMVer3(const std::vector<size_t> &mask_confident_idx_set_in, std::vector<size_t> &pre_selected_idx_set, CTARGET_OBJECT_DATA* &target_object_data);
    void selectMaskSAMVer4(const std::vector<size_t> &mask_confident_idx_set_in, std::vector<size_t> &pre_selected_idx_set, CTARGET_OBJECT_DATA &target_object_data);
    void mergeMasks(CTARGET_OBJECT_DATA* &target_object_data, pcl::PointCloud<pcl::PointXYZ> &cloud_mask_processed);
    bool filterUsingEuclideanClustering(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_merged, double ec_tol, size_t ec_min_size, size_t ec_max_size, bool b_plot);
    bool filterUsingEuclideanClusteringAndBoundingBox(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_merged, double ec_tol, size_t ec_min_size, size_t ec_max_size, bool b_plot);
    bool extractMajorSegmentUsingEuclideanClustering(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_merged, double ec_tol, size_t ec_min_size, size_t ec_max_size, bool b_plot);
    bool extractMajorSegmentUsingEuclideanClusteringWithNormals(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_merged, double ec_tol, size_t ec_min_size, size_t ec_max_size, bool b_plot);


    void optimizeBinWorkspace(CTARGET_OBJECT_DATA* &target_object_data, bool do_measured_cloud_process, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_scan, const int target_id, const std::string target_name, const Eigen::VectorXf q_scan, std::vector<double> &vec_out, std::vector<std::vector<double>> &vec_out_sub_pose_set, double &matching_accuracy, bool do_scan_sampling, size_t sampling_num, bool b_plot);
    void genQueryBaseFramePoints(CTARGET_OBJECT_DATA* &target_object_data, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out);
	bool checkPath2TargetPose(CTARGET_OBJECT_DATA* &target_object_data, CPATHDATA &path_data, const Eigen::VectorXf &pose_in, const Eigen::VectorXf q_init, Eigen::VectorXf &q_sol);
    bool checkRobotSingularPose(CTARGET_OBJECT_DATA* &target_object_data, CVIEWPOINTDATA &vp_data, CPATHDATA &path_data, const std::vector<double> pose_in, Eigen::VectorXf &q_prev);
	bool checkJacobianCondNum(const Eigen::MatrixXf &matJ, double thre_cond, double &output);



    // initialize template
    void initializeTemplate();
    void setTargetObjectTemplate(const size_t &target_idx, bool do_overwrite);
    void setTargetObjectRobotParameters(const size_t &target_idx, const std::vector<double> &dh, const std::vector<double> &tcp_default, const std::vector<double> &tcp);
    void initializeGlobalCalibrationToolTemplate(const std::vector<double> &dh, const std::vector<double> &tcp_default, const std::vector<double> &tcp);
    void setTemplateParameters();
    void initializeTargetObjectData(CTARGET_OBJECT_DATA &target_object_data);

    // UI
    void UIFuncDoMeasuredCloudFilteringUsingBinBoundingBox(const int target_id, const Eigen::VectorXf &q_scan, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out);
    void UIFuncDoMeasuredCloudFilteringUsingBinBoundingBox2(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::VectorXf &q_scan, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out);


    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// NEW FUNCTION ///////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    void initialMaskProcessingVer1(MASK_DATA &mask_set_in, const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_scan);
    void initialMaskProcessingVer2(MASK_DATA &mask_set_in, const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_scan);
    void doMatchingForEvaluationGraspingPose(CTARGET_OBJECT_DATA* &target_object_data);
    void doMatchingForEvaluationGraspingPoseVer2(CTARGET_OBJECT_DATA* &target_object_data);
    bool evaluateGraspingPose(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const size_t &threshold, std::vector<size_t> &collision_count_set, bool b_plot);

    void doZigPoseInitialTeaching(CTARGET_OBJECT_DATA* &target_object_data);

    bool checkWorkspace(const std::vector<double> pose, const std::vector<double> &workspace);
    bool checkWorkspacePosition(const std::vector<double> pose, const std::vector<double> &workspace);


    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////// SIMULATION /////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////

    bool extractCollisionCloudResults(CTARGET_OBJECT_DATA* &target_object_data, const std::string &color, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_matching_results);
    double extractCollisionCloudResultsWithKDTree(CTARGET_OBJECT_DATA* &target_object_data, const std::string &color, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_matching_results);

    void doCustomPlacingObject2Bin(CTARGET_OBJECT_DATA* &target_object_data);
    void doSimulatePlacingObject2Bin(CTARGET_OBJECT_DATA* &target_object_data);
    bool manipulateObjectsAndEvaluateVoxelMap(CTARGET_OBJECT_DATA* &target_object_data, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_process, const Eigen::Matrix4f &T_process);

    void doCustomPlacingObject2Bin_ver2(CTARGET_OBJECT_DATA* &target_object_data);
    void doSimulatePlacingObject2Bin_ver2(CTARGET_OBJECT_DATA* &target_object_data);
    bool manipulateObjectsAndEvaluateVoxelMap_ver2(bool do_step, CTARGET_OBJECT_DATA* &target_object_data, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_process);
    
    bool findMultipleContactPoint(CTARGET_OBJECT_DATA* &target_object_data);
    bool findMultipleContactPoint2(CTARGET_OBJECT_DATA* &target_object_data);
    bool isContactStable(CTARGET_OBJECT_DATA* &target_object_data);
    bool isPointInsideOBB(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, const Eigen::Vector3f &query_point, bool b_plot);
    void calNetMoment(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, const Eigen::Vector3f &center_of_mass, Eigen::Vector3f &net_moment);


    void initializeSimulationOctoMap(CTARGET_OBJECT_DATA* &target_object_data);
    void insertPointSimulationOctoMap(CTARGET_OBJECT_DATA* &target_object_data);
    void setSimulationParameters(CTARGET_OBJECT_DATA* &target_object_data);

    float calculateMomentOfInertia(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, const Eigen::Vector3f &current_axis, const float &point_mass);

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////// SIMILARITY EVALUATION ///////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
	bool featureSimilarityEvaluation(const pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_meas, const pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_ref);
    void selectClusterUsingFeatureSimilarity(const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PointXYZRGBA> > &cluster_set, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out);

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////// TEST FUNCTION //////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
	void test_function();
    void saveBin2BaseHTM(CTARGET_OBJECT_DATA* &target_object_data);


    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////// LEARNING WORKSPACE /////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    void calConditionsBinWorkspaceForLearning(CTARGET_OBJECT_DATA* &target_object_data, const int target_id, const std::string target_name, const Eigen::VectorXf q_scan, std::vector<double> &vec_out, std::vector<std::vector<double>> &vec_out_sub_pose_set, double &matching_accuracy, bool do_scan_sampling, size_t sampling_num, bool b_plot);
    void pushbackDatasetForLearning(CTARGET_OBJECT_DATA* &target_object_data);

    
    int load_sampling_num_from_json(const std::string &config_path,
        const std::string &object_name,
        int current_sampling_num, // fallback
        const rclcpp::Logger &logger);

    std::vector<double> load_parameter_view_frustum_from_json(
        const std::string &config_path,
        const std::string &object_name,
        const std::vector<double> &current_params, // fallback
        const rclcpp::Logger &logger);

    void saveHtmToJson(
            const std::string& json_path,
            const std::string& tag,
            const Eigen::Matrix4f& T_in);

    Eigen::Matrix4f loadHtmFromJson(
        const std::string& json_path,
        const std::string& tag);
};