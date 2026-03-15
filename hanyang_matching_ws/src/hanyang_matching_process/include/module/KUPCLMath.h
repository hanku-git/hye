/**
 * @file KUPCLMath.h
 * @brief 
 */
#ifdef max
#undef max
#undef min
#endif

#pragma once

#include <iostream>
#include <string>
#include <numeric>

// #include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/pcl_exports.h>
#include <pcl/search/kdtree.h>


/**
 * @class KUPCLMATH
 * @brief 연산 모듈 클래스
 * @ingroup MATH
 */
class KUPCLMATH
{
public: 
	KUPCLMATH();
	~KUPCLMATH();
public:
	void removeDuplicates(std::vector<Eigen::Vector3d> &con);
	void removeDuplicates_2D(std::vector<Eigen::Vector2i> &con);
	void removeDuplicates(std::vector<int> &con);
	void removeDuplicates(std::vector<size_t> &con);
    void removeDuplicates(std::vector<unsigned int> &con);
	void removeDuplicatesCloud(const pcl::PointCloud<pcl::PointXYZ> &input, pcl::PointCloud<pcl::PointXYZ> &output);
	void removeDuplicatesCloud(const pcl::PointCloud<pcl::PointXYZRGBA> &input, pcl::PointCloud<pcl::PointXYZRGBA> &output);
	void removeDuplicatesCloud(const pcl::PointCloud<pcl::PointXYZLNormal> &input, pcl::PointCloud<pcl::PointXYZLNormal> &output);
	std::vector<int> intersection(std::vector<int> &v1, std::vector<int> &v2);
	void vec2cloud(const std::vector<Eigen::Vector3d>& input, pcl::PointCloud<pcl::PointXYZ> &output);
	void vec2cloud(const std::vector<Eigen::Vector3d>& input, pcl::PointCloud<pcl::PointXYZRGBNormal> &output);
	void vec2cloud(const std::vector<Eigen::Vector3d>& input, pcl::PointCloud<pcl::PointXYZLNormal> &output);
	void vec2cloud(const std::vector<Eigen::Vector3d>& input, pcl::PointCloud<pcl::PointXYZRGBA> &output);
	void pt2cloud(const Eigen::Vector3d& input, pcl::PointCloud<pcl::PointXYZ> &output);
    Eigen::Vector3d cloud2pt(const pcl::PointCloud<pcl::PointXYZ> &input, const size_t &idx);
    Eigen::Vector3d cloud2pt(const pcl::PointCloud<pcl::PointXYZRGB> &input, const size_t &idx);
	Eigen::Vector3d cloud2pt(const pcl::PointCloud<pcl::PointXYZRGBNormal> &input, const size_t &idx);
	Eigen::Vector3d cloud2pt(const pcl::PointCloud<pcl::PointXYZLNormal> &input, const size_t &idx);
	void cloud2ptNormal(const pcl::PointCloud<pcl::PointXYZRGBNormal> &input, const size_t &idx, Eigen::Vector3d &pt, Eigen::Vector3d &normal);
	void cloud2ptNormal(const pcl::PointCloud<pcl::PointXYZLNormal> &input, const size_t &idx, Eigen::Vector3d &pt, Eigen::Vector3d &normal);
    void extractCloud(const pcl::PointCloud<pcl::PointXYZ> &input, const pcl::PointIndices &idx, pcl::PointCloud<pcl::PointXYZ> &output);
	void extractCloud(const pcl::PointCloud <pcl::Normal> &input, const pcl::PointIndices &idx, pcl::PointCloud <pcl::Normal> &output);
	void extractCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> &input, const pcl::PointIndices &idx, pcl::PointCloud<pcl::PointXYZ> &output);
	void extractCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> &input, const pcl::PointIndices &idx, pcl::PointCloud<pcl::PointXYZRGBNormal> &output);
	void extractSegmentIdx(const std::vector<int> &input, const pcl::PointCloud<pcl::PointXYZLNormal> &label, std::vector<int> &output);
    void extractSegmentIdx(const std::vector<size_t> &input, const pcl::PointCloud<pcl::PointXYZLNormal> &label, std::vector<size_t> &output);
	void extractDataFromIdx(const std::vector<size_t> &input, const pcl::PointIndices &idx, std::vector<size_t> &output);
	void extractDataFromIdx(const std::vector<size_t> &input, const std::vector<size_t> &idx, std::vector<size_t> &output);
    void reverseVecOrder(std::vector<std::vector<int>>& list);
    void reverseVecOrder(std::vector<std::vector<size_t>>& list);
	std::vector<std::vector<int>> combnk(const std::vector<int>& input, int k);
	std::vector<std::vector<size_t>> combnk(const std::vector<size_t>& input, size_t k);	
    void vec2meanStd(const std::vector<double>& input, double& m, double& std);
	Eigen::Vector3d calMeanPoint(const std::vector<Eigen::Vector3d>& input);
	bool dotCheck(double dot, double angle);
	bool dotCheck2(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, double angle);
	bool dotCheck2(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, double angle);
    float getVecSign(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2);
	double sgn(double val);
	void cloudScaling(pcl::PointCloud<pcl::PointXYZ> &input, int b_scale);
	void cloudScaling(pcl::PointCloud<pcl::PointXYZRGBNormal> &input, int b_scale);
	void cloudScaling(pcl::PointCloud<pcl::PointXYZLNormal> &input, int b_scale);
	void cloudScaling(pcl::PointCloud<pcl::PointXYZRGBA> &input, int b_scale);
	void cloudScaling(pcl::PointCloud<pcl::PointXYZRGB> &input, int b_scale);
	void cloudPointScaling(pcl::PointCloud<pcl::PointXYZ> &input, float scale);
	void vec2Indices(const std::vector<int>& input, pcl::PointIndices &output);
	void vec2Indices(const std::vector<size_t>& input, pcl::PointIndices &output);
	void label2Indices(const pcl::PointCloud<pcl::PointXYZLNormal> &input, pcl::PointIndices &output);
	void genVecMinMax(const std::vector<int>& input, int &idx_out, int &dis_out, int b_select);
	void genVecMinMax(const std::vector<size_t>& input, size_t &idx_out, size_t &dis_out, int b_select);
	void genVecMinMax(const std::vector<double>& input, int &idx_out, double &dis_out, int b_select);
	void genVecMinMax(const std::vector<double>& input, size_t &idx_out, double &dis_out, int b_select);
	void genVecMinMax(const std::vector<float>& input, int &idx_out, float &dis_out, int b_select);
	void genAscendingOrder(const std::vector<int>& input, std::vector<int>& output_sort, std::vector<int>& output_idx);
	void genAscendingOrder(const std::vector<double>& input, std::vector<double>& output_sort, std::vector<int>& output_idx);
	void genAscendingOrder(const std::vector<double>& input, std::vector<double>& output_sort, std::vector<size_t>& output_idx);
	void genDescendingOrder(const std::vector<int>& input, std::vector<int>& output_sort, std::vector<int>& output_idx);
	void genDescendingOrder(const std::vector<double>& input, std::vector<double>& output_sort, std::vector<int>& output_idx);
	void genDescendingOrder(const std::vector<double>& input, std::vector<double>& output_sort, std::vector<size_t>& output_idx);
	void eraseIdxValue(std::vector<int>& input, int idx); 
    void eraseIdxValue(std::vector<size_t>& input, size_t idx);
	void eraseIdxValue(std::vector<float>& input, int idx);
	void eraseIdxValue(std::vector<double>& input, int idx);
	void eraseIdxValue(std::vector<Eigen::Vector3d>& input, int idx);
	void genDiffElements(const std::vector<int>& input1, const std::vector<int>& input2, std::vector<int>& output);
	void genRotM(double angle, Eigen::Matrix3d &rotM, Eigen::Matrix4d &HTM, int direction);
    void genRotM(double angle, Eigen::Matrix3f &rotM, Eigen::Matrix4f &HTM, int direction);
	Eigen::Vector3d rotm2ZYX(Eigen::Matrix3d &RotMat);
    Eigen::Vector3f rotm2ZYX(Eigen::Matrix3f &RotMat);
	void forwardKinematics(const double DH[6][4], const Eigen::VectorXd &q, Eigen::Matrix4d &T_out);
    void forwardKinematics(const double DH[6][4], const Eigen::VectorXf &q, Eigen::Matrix4f &T_out);
	void tr2pose(const Eigen::Matrix4d &T_in, Eigen::VectorXd &vec_out);
    void tr2pose(const Eigen::Matrix4f &T_in, Eigen::VectorXf &vec_out);
	Eigen::Matrix4f genTranslationHTM(double x, double y, double z);
	Eigen::Matrix4f genRxRotationHTM(double Roll);
	Eigen::Matrix4f genRyRotationHTM(double Pitch);
	Eigen::Matrix4f genRzRotationHTM(double Yaw);
	Eigen::Vector3f toRotVec(double Roll, double Pitch, double Yaw);
	Eigen::Vector3f crossProduct(double V1[3], double V2[3]);
	Eigen::Vector3f toRPY(Eigen::Matrix4f RotMat);
	Eigen::Matrix4f genHTM(double rot, double trans_x, double trans_y, double trans_z, int method);
	Eigen::Matrix4f genPose2HTM(const Eigen::VectorXd &pose_in);

	//// ver. 2
	double calDistPt2Plane(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector3d &normal);
	Eigen::Vector3d multipleUnitVectorSummation(const std::vector<Eigen::Vector3d> &vec_set);
	Eigen::Vector3d findKNearestPoint(const std::vector<Eigen::Vector3d> &vec_set, const Eigen::Vector3d &pt_query);
	Eigen::Vector3d findKNearestPointWithIndex(const std::vector<Eigen::Vector3d> &vec_set, const Eigen::Vector3d &pt_query, size_t &nearest_index);
	Eigen::Vector3d findKNearestPointWithIndex(const pcl::PointCloud<pcl::PointXYZ> &input, const Eigen::Vector3d &pt_query, size_t &nearest_index);
	Eigen::Vector3d findKNearestPointWithIndex(const pcl::PointCloud<pcl::PointXYZRGBNormal> &input, const Eigen::Vector3d &pt_query, size_t &nearest_index);
	Eigen::Vector3d findKNearestPointWithIndex(const pcl::PointCloud<pcl::PointXYZLNormal> &input, const Eigen::Vector3d &pt_query, size_t &nearest_index);
	Eigen::Vector3d findKNearestPoint2(pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &model, const pcl::PointCloud<pcl::PointXYZ> &cloud_in, const Eigen::Vector3d &pt_query);
	Eigen::Vector3d findKNearestPoint2(pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &model, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, const Eigen::Vector3d &pt_query);
	void findKNearestIndex(pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &model, const Eigen::Vector3d &pt_query, std::vector<int> &output_idx, std::vector<float> &output_dists, int K);
	void findKNearestIndex(pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &model, const Eigen::Vector3f &pt_query, std::vector<int> &output_idx, std::vector<float> &output_dists, int K);
    Eigen::Vector3f findKNearestIndex2(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, const Eigen::Vector3f &pt_query, size_t &nearest_index);
    Eigen::VectorXf findKNearestIndex2(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, const Eigen::Vector3f &pt_query, size_t &nearest_index);
    void findKNearestIndex3(pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &model, const Eigen::Vector3f &pt_query, size_t &nearest_index);
	void findRadiusNearestIndex(pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &model, const Eigen::Vector3d &pt_query, std::vector<int> &output_idx, std::vector<float> &output_dists, double radius);
    void findRadiusNearestIndex(pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &model, const Eigen::Vector3f &pt_query, std::vector<int> &output_idx, std::vector<float> &output_dists, double radius);

	Eigen::Vector3d transformPoint2A(Eigen::Vector3d &vec_in, const Eigen::Matrix4d &T_A2B);
	Eigen::Vector3f transformPoint2A(Eigen::Vector3f &vec_in, const Eigen::Matrix4f &T_A2B);
	std::vector<Eigen::Vector3d> genSortedVectorSet(const std::vector<Eigen::Vector3d> &vec_set_in, const std::vector<size_t> &sort_idx);
	std::vector<Eigen::Matrix4d> genSortedVectorSet(const std::vector<Eigen::Matrix4d> &vec_set_in, const std::vector<size_t> &sort_idx);
	std::vector<Eigen::Matrix4f> genSortedVectorSet(const std::vector<Eigen::Matrix4f> &vec_set_in, const std::vector<size_t> &sort_idx);
	std::vector<double> genSortedVectorSet(const std::vector<double> &vec_set_in, const std::vector<size_t> &sort_idx);

	std::vector<double> genSameWidthValueSet(double x_min, double x_max, double scale);
	std::vector<size_t> genSameWidthValueSet(size_t x_min, size_t x_max, size_t scale);

	double calRotationAngleFromNormal(const Eigen::Vector3d normal, const Eigen::Vector3d center, double angle_stage2base, int direction);
	Eigen::Matrix4d transformViewpoint2SensorPose(const Eigen::Vector3d &VP, const Eigen::Vector3d &center, const Eigen::Vector3d sensor_orientaion_rx);

	Eigen::VectorXd vec2Eigen(const std::vector<double> &vec_in);
	std::vector<double> eigen2Vec(const Eigen::VectorXd &vec_in);

	void transformCloudPose(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, const Eigen::VectorXf &pose, Eigen::Matrix4f &T_out);
	void transformCloudPose(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, const Eigen::VectorXd &pose, Eigen::Matrix4f &T_out);
	void transformCloudHTM(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, const Eigen::Matrix4f &T);
    void transformCloudHTM2(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out, const Eigen::Matrix4f &T);
	void genCloudMinMax(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointXYZ &minPt, pcl::PointXYZ &maxPt);

    void RPY2Rot(double dRPY[3], double dRot[3][3]);
    void SetTcp(std::vector<double> tcp);


    std::vector<double> htm2pose_mm2m(const Eigen::Matrix4f &T, std::string str);
    std::vector<double> htm2pose(const Eigen::Matrix4f &T, std::string str);
    Eigen::Matrix4f pose2htm(const std::vector<double> &pose_in);
    Eigen::Matrix3f genRotationAroundVector(const Eigen::Vector3f &vec_in, double beta);


    //////////////////////// PRINT
	void printCloudMinMax(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::string &str);
	void printCloudMinMax(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::string &str);
	void printCloudMinMax(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, const std::string &str);
	void printCloudMinMax(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, const std::string &str);
    void printCloudPoints(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, size_t point_size);
    void printCloudPoints(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, size_t point_size);
	void printHTM(const Eigen::Matrix4d &T, std::string str);
	void printHTM(const Eigen::Matrix4f &T, std::string str);
    void printVector(const std::vector<double> &vec_in, std::string str);
    void printVector(const std::vector<float> &vec_in, std::string str);
    void printVector(const Eigen::VectorXf &vec_in, std::string str);
    void printVector3f(const Eigen::Vector3f &vec_in, std::string str);

private:
    Eigen::Matrix4d m_TCP;

};