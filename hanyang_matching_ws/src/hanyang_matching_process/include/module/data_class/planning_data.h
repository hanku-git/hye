#pragma once


// #include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/parse.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/fpfh.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/pcl_exports.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>
#include <vtkPolyLine.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/default_convergence_criteria.h>

#include <octomap/OcTree.h>
#include <Eigen/Eigen>

class CSegParameters
{
public:
	CSegParameters() {};
	~CSegParameters() {};

public:

    // Region growing segmenation (RGS)
    int rgs_normal_K;
    int rgs_min_size;
    int rgs_max_size; 
    double rgs_thre_angle; 
    double rgs_thre_curvature; 
    int rgs_neighbor_K;

    // Color-based Region growing segmenation (CRGS)
    double color_rgs_neighbor_R; // ex) 10mm
    double color_rgs_thre_1st_color; // ex) 6, (normal과 유사) This method specifies the threshold value for color test between the points. This kind of testing is made at the first stage of the algorithm(region growing). If the difference between points color is less than threshold value, then they are considered to be in the same region.
    double color_rgs_thre_2nd_color; // ex) 5, (curvature과 유사) threshold value for color test between the regions. This kind of testing is made at the second stage of the algorithm(region merging). If the difference between segments color is less than threshold value, then they are merged together.
    size_t color_rgs_min_cluster_size; // ex) 600, minimum cluster size, 이 값보다 작으면 근처 cluster에 병합됨.
    
    // Euclidean cluster segmentation
    double euclidean_cluster_tol;
    size_t euclidean_min_cluster_size;
    size_t euclidean_max_cluster_size;
};

/**
 * @class FeatureCloud
 * @brief point cloud representation
 * @ingroup ETC
 */
class FeatureCloud
{
  public:
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
	  normal_radius_(0.02f),
	  feature_radius_(0.02f)
    {}

    ~FeatureCloud () {}

    void
    setInputCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    void
    setInputCloudNoProcessing (pcl::PointCloud<pcl::PointXYZ>::Ptr xyz)
    {
      xyz_ = xyz;
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
		xyz_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
      processInput ();
    }

    // Get a pointer to the cloud 3D points
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

    void
    setLocalFeatures (LocalFeatures::Ptr features)
    {
      features_ = features;
    }
    

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

/**
 * @class TemplateAlignment
 * @brief point cloud representation
 * @ingroup ETC
 */
class TemplateAlignment
{
  public:

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      PCL_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment () :
      min_sample_distance_ (0.05f),
      max_correspondence_distance_ (0.01f*0.01f),
      nr_iterations_ (500),
      sample_num_ (3),
      corr_rnd_k_ (10)
    {
      // Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);
      sac_ia_.setNumberOfSamples (sample_num_);
      sac_ia_.setCorrespondenceRandomness (corr_rnd_k_);
    }

    ~TemplateAlignment () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }

    void
    setParameters(float min_sample_dist, float max_corr_dist, int iter, int sample_num, int corr_rnd_k)
    {
        min_sample_distance_ = min_sample_dist;
        max_correspondence_distance_ = max_corr_dist*max_corr_dist;
        nr_iterations_ = iter;
        sample_num_ = sample_num;
        corr_rnd_k_ = corr_rnd_k;
        sac_ia_.setMinSampleDistance (min_sample_distance_);
        sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
        sac_ia_.setMaximumIterations (nr_iterations_);
        sac_ia_.setNumberOfSamples (sample_num_);
        sac_ia_.setCorrespondenceRandomness (corr_rnd_k_);
        std::cout << "min_sample_distance_: " << sac_ia_.getMinSampleDistance()  << std::endl;
        std::cout << "max_correspondence_distance_(squared): " << sac_ia_.getMaxCorrespondenceDistance()  << std::endl;
        std::cout << "nr_iterations_: " << sac_ia_.getMaximumIterations()  << std::endl;
        std::cout << "NumberOfSamples: " << sac_ia_.getNumberOfSamples()  << std::endl;
        std::cout << "CorrespondenceRandomness: " << sac_ia_.getCorrespondenceRandomness()  << std::endl;
        
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
      templates_.push_back (template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
      sac_ia_.setInputSource (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

      pcl::PointCloud<pcl::PointXYZ> registration_output;
      sac_ia_.align (registration_output);

      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.final_transformation = sac_ia_.getFinalTransformation ();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());
      for (std::size_t i = 0; i < templates_.size (); ++i)
      {
        align (templates_[i], results[i]);
      }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
    {
      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (results);

      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (std::size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.fitness_score < lowest_score)
        {
          lowest_score = r.fitness_score;
          best_template = (int) i;
        }
      }

      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }

  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
    int sample_num_;
    int corr_rnd_k_;
};

/**
 * @class FeatureCloud
 * @brief point cloud representation
 * @ingroup ETC
 */
class FeatureCloudNormal
{
  public:
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloudNormal () :
      search_method_xyz_ (new SearchMethod),
	  normal_radius_(0.02f),
	  feature_radius_(0.02f)
    {}

    ~FeatureCloudNormal () {}

    void
    setInputCloud (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr xyzrgbn)
    {
      xyzrgbn_ = xyzrgbn;
      xyz_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*xyzrgbn_, *xyz_);
      processInput ();
    }

    void
    setInputCloudNoProcessing (pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr xyzrgbn)
    {
      xyzrgbn_ = xyzrgbn;
      xyz_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*xyzrgbn_, *xyz_);
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyzrgbn_ = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcl::io::loadPCDFile (pcd_file, *xyzrgbn_);
      processInput ();
    }

    // Get a pointer to the cloud 3D points
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

    void
    setLocalFeatures (LocalFeatures::Ptr features)
    {
      features_ = features;
    }
    

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);
      pcl::copyPointCloud(*xyzrgbn_, *normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr xyzrgbn_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

/**
 * @class TemplateAlignment
 * @brief point cloud representation
 * @ingroup ETC
 */
class TemplateAlignmentNormal
{
  public:

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      PCL_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignmentNormal () :
      min_sample_distance_ (0.05f),
      max_correspondence_distance_ (0.01f*0.01f),
      nr_iterations_ (500),
      sample_num_ (3),
      corr_rnd_k_ (10)
    {
      // Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);
      sac_ia_.setNumberOfSamples (sample_num_);
      sac_ia_.setCorrespondenceRandomness (corr_rnd_k_);
    }

    ~TemplateAlignmentNormal () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloudNormal &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }

    void
    setParameters(float min_sample_dist, float max_corr_dist, int iter, int sample_num, int corr_rnd_k)
    {
        min_sample_distance_ = min_sample_dist;
        max_correspondence_distance_ = max_corr_dist*max_corr_dist;
        nr_iterations_ = iter;
        sample_num_ = sample_num;
        corr_rnd_k_ = corr_rnd_k;
        sac_ia_.setMinSampleDistance (min_sample_distance_);
        sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
        sac_ia_.setMaximumIterations (nr_iterations_);
        sac_ia_.setNumberOfSamples (sample_num_);
        sac_ia_.setCorrespondenceRandomness (corr_rnd_k_);

        std::cout << "min_sample_distance_: " << sac_ia_.getMinSampleDistance()  << std::endl;
        std::cout << "max_correspondence_distance_(squared): " << sac_ia_.getMaxCorrespondenceDistance()  << std::endl;
        std::cout << "nr_iterations_: " << sac_ia_.getMaximumIterations()  << std::endl;
        std::cout << "NumberOfSamples: " << sac_ia_.getNumberOfSamples()  << std::endl;
        std::cout << "CorrespondenceRandomness: " << sac_ia_.getCorrespondenceRandomness()  << std::endl;
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloudNormal &template_cloud)
    {
      templates_.push_back (template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloudNormal &template_cloud, TemplateAlignmentNormal::Result &result)
    {
      sac_ia_.setInputSource (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

      pcl::PointCloud<pcl::PointXYZ> registration_output;
      sac_ia_.align (registration_output);

      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.final_transformation = sac_ia_.getFinalTransformation ();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignmentNormal::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());
      for (std::size_t i = 0; i < templates_.size (); ++i)
      {
        align (templates_[i], results[i]);
      }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignmentNormal::Result &result)
    {
      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (results);

      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (std::size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.fitness_score < lowest_score)
        {
          lowest_score = r.fitness_score;
          best_template = (int) i;
        }
      }

      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }

  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloudNormal> templates_;
    FeatureCloudNormal target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
    int sample_num_;
    int corr_rnd_k_;
};

/**
* @class CTARGET_OBJECT_DATA
* @brief 파지 대상 물체의 데이터 클래스
* @ingroup BIN_PICKING_DATA_CLASS
*/
class CALIGN_PARAMETER
{
public:
	CALIGN_PARAMETER() {};
	~CALIGN_PARAMETER() {};
public:
    bool do_quick_align = false;
    bool quick_align_use_cloud_with_mesh_normal = false;
    double quick_align_z_threshold;
    float quick_align_min_sample_distance;
    float quick_align_max_correspondence_distance;
    int quick_align_iteration;
    int quick_align_sample_num;
    int quick_align_corr_rnd_k;

    double icp_corr_threshold;
    double icp_euclidean_fitness_eps; // the maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the algorithm is considered to have converged. The error is estimated as the sum of the differences between correspondences in an Euclidean sense, divided by the number of correspondences.
    double icp_transformation_eps; // the transformation epsilon (maximum allowable translation squared difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution. 
    int icp_max_iter_inner;
    int icp_max_iter_outer;

    FeatureCloud target_ft_cloud;
    FeatureCloudNormal target_ft_cloud_with_normal;

    FeatureCloud measured_ft_cloud;
    FeatureCloudNormal measured_ft_cloud_with_normal;
    
};


/**
* @class CPLANNINGDATA
* @brief 계획 단계의 입출력 데이터 클래스
* @ingroup DATA_CLASS
*/
class CPLANNINGDATA
{
public:
	CPLANNINGDATA() {};
	~CPLANNINGDATA() {};

public:
	// Voxel map
	double map_voxel_length;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr m_cloud_CAD;
	octomap::OcTree* m_OctoMap_CAD;
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> md_KdTree_octoMap_CAD;
};

/**
* @class CVIEWPOINTDATA
* @brief 시야점 데이터 클래스
* @ingroup DATA_CLASS
*/
class CVIEWPOINTDATA
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
	CVIEWPOINTDATA() {};
	~CVIEWPOINTDATA() {};

public:
	bool is_path_feasible;
	//// input
	Eigen::Vector3d VP;
	Eigen::Vector3d center;
	Eigen::Vector3d normal;

	//// output
	// robot
	Eigen::VectorXf target_CS_pose;
	Eigen::VectorXf target_JS_position;
	Eigen::VectorXf prev_JS_position;
	// rotation stage
	double target_rs_angle; 
	double prev_rs_angle;

	// path time
	double path_time_cost;
};

/**
* @class CPATHDATA
* @brief 검사 측정 단계의 데이터 클래스
* @ingroup DATA_CLASS
*/
class CPATHDATA
{
public:
	CPATHDATA() {};
	~CPATHDATA() {};

public:
	//// User input parameters
	std::vector<CVIEWPOINTDATA> viewpoint_data;
//    std::vector<Eigen::Matrix4f> target_sensor_pose;
    std::vector<std::vector<double>> target_sensor_pose;
	std::vector<std::vector<int>> registration_idx_input;
	std::vector<double> initial_JS_angle;

	//// parameters
	double angle_rotation_stage2Base; // rotation stage frame to base frame
	Eigen::MatrixXf T_T2S;
	Eigen::MatrixXf T_S2T;
	Eigen::MatrixXf T_B2RS;
	Eigen::Vector3d cam_rx_vector; // 카메라의 x방향 벡터가 Base frame 기준의 방향을 향하도록 

    // inverse kinematics
    double kInverseKineLambda; 
    int kInverseKineIterLimit;
    double kInverseKineTransTol;
    double kInverseKineOriTol;
    // path planning
    double control_period;
    std::vector<double> dAcc;
    std::vector<double> dVel;

	//// Output
	std::vector<Eigen::VectorXf> target_pose;
	std::vector<std::vector<int>> registration_idx_output;
	bool is_path_planning_finished = false;
};