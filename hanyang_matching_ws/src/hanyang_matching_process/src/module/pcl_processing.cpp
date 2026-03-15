/**
 * @file pcl_processing.cpp
 * @brief 점군 데이터의 연산을 수행하기 위한 구현파일
 */
#include "module/pcl_processing.h"
#include <string>
#include <iostream>


PCLPROCESSING::PCLPROCESSING()
{

}

PCLPROCESSING::~PCLPROCESSING()
{

}

const double visualize_pt_size_1 = 2.5; // reference cloud
const double visualize_pt_size_2 = 5.0; // measured cloud
bool next_iter = false;
void print4Matrix(const Eigen::Matrix4d &matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}
void keyEventOccurred(const pcl::visualization::KeyboardEvent &event, void *nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iter = true;
}

/** @brief Korean: 점군 데이터 포인터의 초기화를 수행한다.
 * @param[in] cloud_ptr : 점군 데이터의 포인터
 */
void PCLPROCESSING::initCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_ptr = cloud_tmp;
}

/** @brief Korean: 점군 데이터 포인터의 초기화를 수행한다.
* @param[in] cloud_ptr : 점군 데이터의 포인터
*/
void PCLPROCESSING::initCloudPtr(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_ptr)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	cloud_ptr = cloud_tmp;
}

/** @brief Korean: 점군 데이터 포인터의 초기화를 수행한다.
* @param[in] cloud_ptr : 점군 데이터의 포인터
*/
void PCLPROCESSING::initCloudPtr(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_ptr)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud_ptr = cloud_tmp;
}

/** @brief Korean: 점군 데이터 포인터의 초기화를 수행한다.
* @param[in] cloud_ptr : 점군 데이터의 포인터
*/
void PCLPROCESSING::initCloudPtr(pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud_ptr)
{
	pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZLNormal>);
	cloud_ptr = cloud_tmp;
}

/** @brief Korean: 점군 데이터 포인터의 초기화를 수행한다.
* @param[in] cloud_ptr : 점군 데이터의 포인터
*/
void PCLPROCESSING::initCloudPtr(pcl::PointIndices::Ptr &cloud_ptr)
{
	pcl::PointIndices::Ptr cloud_tmp(new pcl::PointIndices());
	cloud_ptr = cloud_tmp;
}

/** @brief Korean: OctoMap 포인터의 초기화를 수행한다.
* @param[in] cloud_ptr : 점군 데이터의 포인터
*/
void PCLPROCESSING::initOctomapPtr(octomap::OcTree* &map_ptr)
{
	octomap::OcTree* OctoMap_CAD = new octomap::OcTree(0.8);
	map_ptr = OctoMap_CAD;
}

/** @brief Korean: OctoMap 포인터의 초기화를 수행한다.
* @param[in] cloud_ptr : 점군 데이터의 포인터
*/
void PCLPROCESSING::initOctomapPtrWithSize(octomap::OcTree* &map_ptr, double octo_size)
{
	octomap::OcTree* OctoMap_CAD = new octomap::OcTree(octo_size);
	map_ptr = OctoMap_CAD;
}

void PCLPROCESSING::initOctomapCollection(octomap::MapCollection<octomap::MapNode<octomap::OcTree>>* &collection)
{
    octomap::MapCollection<octomap::MapNode<octomap::OcTree>>* OctoMap_collection;
    collection = OctoMap_collection;
}

void PCLPROCESSING::genOctomapCollection(octomap::OcTree* &map_1, octomap::OcTree* &map_2, octomap::MapCollection<octomap::MapNode<octomap::OcTree>> &collection)
{
    octomap::MapCollection<octomap::MapNode<octomap::OcTree>> OctoMap_collection;
    octomap::MapNode<octomap::OcTree>* mn1 = new octomap::MapNode<octomap::OcTree>(map_1, octomap::pose6d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    octomap::MapNode<octomap::OcTree>* mn2 = new octomap::MapNode<octomap::OcTree>(map_2, octomap::pose6d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    mn1->setId("CAD");
    mn2->setId("Object");
    OctoMap_collection.addNode(mn1);
    OctoMap_collection.addNode(mn2);
    map_1->setOccupancyThres(0.6); // 점유 파라미터(thre_map) 이상이면 점유(occupied)로 판정
    map_2->setOccupancyThres(0.6); // 점유 파라미터(thre_map) 이상이면 점유(occupied)로 판정
    collection = OctoMap_collection;
}

void PCLPROCESSING::genOctomapCollection2(octomap::OcTree* &map_1, octomap::OcTree* &map_2, octomap::MapCollection<octomap::MapNode<octomap::OcTree>> &collection, const std::vector<double> &pose1, const std::vector<double> &pose2)
{
    octomap::MapCollection<octomap::MapNode<octomap::OcTree>> OctoMap_collection;
    octomap::MapNode<octomap::OcTree>* mn1 = new octomap::MapNode<octomap::OcTree>(map_1, octomap::pose6d(pose1[0], pose1[1], pose1[2], pose1[3], pose1[4], pose1[5]));
    octomap::MapNode<octomap::OcTree>* mn2 = new octomap::MapNode<octomap::OcTree>(map_2, octomap::pose6d(pose2[0], pose2[1], pose2[2], pose2[3], pose2[4], pose2[5]));
    mn1->setId("CAD");
    mn2->setId("Object");
    OctoMap_collection.addNode(mn1);
    OctoMap_collection.addNode(mn2);
    map_1->setOccupancyThres(0.6); // 점유 파라미터(thre_map) 이상이면 점유(occupied)로 판정
    map_2->setOccupancyThres(0.6); // 점유 파라미터(thre_map) 이상이면 점유(occupied)로 판정
    collection = OctoMap_collection;
}

/** @brief Korean: 점군 데이터 포인터에 입력 점군 데이터를 복사한다.
 * @param[in] cloud_ptr : 점군 데이터의 포인터
 * @param[in] cloud_input : 입력 점군 데이터
 */
void PCLPROCESSING::genCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr, pcl::PointCloud<pcl::PointXYZ> &cloud_input)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_tmp = cloud_input;
	cloud_ptr = cloud_tmp;
}

/** @brief Korean: 입력된 파일명의 점군 데이터를 불러온 후, 점군 데이터 포인터에 입력 점군 데이터를 복사한다.
 * @param[in] cloudname : 점군 데이터의 파일명
 * @param[in] cloud_ptr : 점군 데이터의 포인터
 */
void PCLPROCESSING::genCloudPtr2(std::string cloudname, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(cloudname, *cloud_tmp);
	cloud_ptr = cloud_tmp;
}

/** @brief Korean: 입력된 파일명의 점군 데이터를 불러온 후, 점군 데이터 포인터에 입력 점군 데이터를 복사한다.
 * @param[in] cloudname : 점군 데이터의 파일명
 * @param[in] cloud_ptr : 점군 데이터의 포인터
 */
void PCLPROCESSING::genCloudPtr2(std::string cloudname, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_ptr)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::io::loadPCDFile(cloudname, *cloud_tmp);
	cloud_ptr = cloud_tmp;
}

/** @brief Korean: 입력된 파일명의 점군 데이터를 불러온 후, 점군 데이터 포인터에 입력 점군 데이터를 복사한다.
 * @param[in] cloudname : 점군 데이터의 파일명
 * @param[in] cloud_ptr : 점군 데이터의 포인터
 */
void PCLPROCESSING::genCloudPtr2(std::string cloudname, pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud_ptr)
{
	pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZLNormal>);
	pcl::io::loadPCDFile(cloudname, *cloud_tmp);
	cloud_ptr = cloud_tmp;
}

/** @brief Korean: 입력된 파일명의 점군 데이터를 불러온 후, 점군 데이터 포인터에 입력 점군 데이터를 복사한다.
* @param[in] cloudname : 점군 데이터의 파일명
* @param[in] cloud_ptr : 점군 데이터의 포인터
*/
void PCLPROCESSING::genPLY2CloudPtr(std::string cloudname, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile(cloudname, *cloud_tmp);
	cloud_ptr = cloud_tmp;
}

/** @brief Korean: 입력된 파일명의 점군 데이터를 불러온 후, 점군 데이터 포인터에 입력 점군 데이터를 복사한다.
* @param[in] cloudname : 점군 데이터의 파일명
* @param[in] cloud_ptr : 점군 데이터의 포인터
*/
void PCLPROCESSING::genPLY2CloudPtr(std::string cloudname, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_ptr)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::io::loadPLYFile(cloudname, *cloud_tmp);
	cloud_ptr = cloud_tmp;
}

/** @brief Korean: 입력된 파일명의 점군 데이터를 불러온 후, 점군 데이터 포인터에 입력 점군 데이터를 복사한다.
* @param[in] cloudname : 점군 데이터의 파일명
* @param[in] cloud_ptr : 점군 데이터의 포인터
*/
void PCLPROCESSING::genPLY2CloudPtr(std::string cloudname, pcl::PointCloud<pcl::PointXYZLNormal>::Ptr &cloud_ptr)
{
	pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZLNormal>);
	pcl::io::loadPLYFile(cloudname, *cloud_tmp);
	cloud_ptr = cloud_tmp;
}

/** @brief Korean: 입력된 점군 데이터 set을 모두 병합하여 점군 데이터 포인터에 복사한다.
 * @param[in] cloud_input : 점군 데이터 set
 * @param[in] cloud_ptr : 점군 데이터의 포인터
 */
void PCLPROCESSING::genMergedCloud(std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ>> &cloud_input, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ptr)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_merge(new pcl::PointCloud<pcl::PointXYZ>);

	for (int i = 0; i < cloud_input.size(); i++)
	{
		*cloud_tmp = cloud_input[i];
		*cloud_merge += *cloud_tmp;
	}
	cloud_ptr = cloud_merge;
}

/** @brief Korean: 입력된 파일명의 점군 데이터를 불러온 후, 출력 점군 데이터 set에 순서대로 저장한다.
 * @param[in] cloudname : 점군 데이터의 파일명
 * @param[out] output : 점군 데이터의 포인터
 */
void PCLPROCESSING::genCloudSet(std::string cloudname, std::vector<pcl::PointCloud<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PointXYZ>> &output)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(cloudname, *cloud_tmp);
	output.push_back(*cloud_tmp);
}

/** @brief Korean: 입력된 파일명의 octomap을 불러온 후, 출력 octomap 포인터에 복사한다.
 * @param[in] cloudname : 저장된 octomap의 파일명
 * @param[out] map_ptr : 출력 octomap의 포인터
 */
void PCLPROCESSING::genOctomapPtr(std::string cloudname, octomap::OcTree* &map_ptr)
{
	octomap::OcTree* OctoMap_CAD = new octomap::OcTree(0.8);
	OctoMap_CAD->readBinary(cloudname);
	map_ptr = OctoMap_CAD;
}

/** @brief Korean: 입력된 점군 데이터를 출력 점군 데이터에 복사한다.
 * @param[in] cloud_in : 입력 점군 데이터
 * @param[out] cloud_out : 출력 점군 데이터
 */
void PCLPROCESSING::copyCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
	pcl::copyPointCloud(cloud_in, cloud_out);
}

void PCLPROCESSING::loadCloud(pcl::PointCloud<pcl::PointXYZ> &cloud_out, std::string file_path) 
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile(file_path, *cloud);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////// SEGMENTATION /////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief Korean: 입력 점군 데이터의 법선(normal)을 추출한다.
 * @param[in] cloud_src : 입력 점군 데이터
 * @param[out] output : 입력 법선
 * @param[in] paramK : kNN search의 K 파라미터
 * @param[in] paramR : radius search의 반경 파라미터
 * @param[in] method : 1: kNN search, 2: radius search
 */
void PCLPROCESSING::normalEstimation(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::Normal> &output, int paramK, double paramR, int method)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	
	ne.setInputCloud(cloud_src);
	ne.setSearchMethod(tree);
	//output->resize(cloud_src->points.size());
	if (method == 0) { ne.setRadiusSearch(paramR); } // radius search
	else { ne.setKSearch(paramK); } // K nearest search
	ne.compute(output);
}

/** @brief Korean: 입력 점군 데이터의 법선(normal)을 추출한다.
 * @param[in] cloud_src : 입력 점군 데이터
 * @param[out] output : 입력 법선
 * @param[in] paramK : kNN search의 K 파라미터
 * @param[in] paramR : radius search의 반경 파라미터
 * @param[in] method : 1: kNN search, 2: radius search
 */
void PCLPROCESSING::normalEstimation(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src, pcl::PointCloud<pcl::Normal> &output, int paramK, double paramR, int method)
{
	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
	
	ne.setInputCloud(cloud_src);
	ne.setSearchMethod(tree);
	//output->resize(cloud_src->points.size());
	if (method == 0) { ne.setRadiusSearch(paramR); } // radius search
	else { ne.setKSearch(paramK); } // K nearest search
	ne.compute(output);
}

/** @brief Korean: 입력 점군 데이터를 구성하는 각 점에 대해 센서좌표계 중심을 향하도록 flip된 법선을 추정하고, 법선이 포함된 점군 데이터를 추출한다.
 * @details 센서 좌표계 기준의 측정 점군의 법선 추정을 위한 방법 - 센서좌표계의 중심 방향에서 visible한 normal을 갖는 점군만 측정된다는 점에 착안
 * @param[in] cloud_src : 입력 점군 데이터
 * @param[out] output : 추출된 법선이 포함된 점군 데이터
 * @param[in] paramK : 법선 추정 시, K-nearest search에 사용되는 파라미터 K
 * @param[in] sensor_origin : 센서좌표계의 중심
 * @param[in] thre_angle : 센서의 허용 임계 각도
 */
void PCLPROCESSING::cloudNormalEst2Measured(const pcl::PointCloud<pcl::PointXYZ> &cloud_src, pcl::PointCloud<pcl::PointXYZRGBNormal> &output, int paramK, const Eigen::Vector3d &sensor_origin, double thre_angle)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normal_CAD(new pcl::PointCloud<pcl::Normal>);
	pcl::copyPointCloud(cloud_src, *cloud_input);

	// normal estimation
    double paramR = 0.0;
	normalEstimation(cloud_input, *normal_CAD, paramK, paramR, 1); // Using K

	// Viewpoint
	Eigen::Vector3d v1;	Eigen::Vector3d v2;
	Eigen::Vector3d pt_query;
	double tmp1 = 0.0;
	double tmp2 = 0.0;
	double angle = 0.0;
	for (size_t i = 0; i < cloud_input->points.size(); i++)
	{
        // selection normal of nearest mesh center 
        v2[0] = normal_CAD->points[i].normal_x;
        v2[1] = normal_CAD->points[i].normal_y;
        v2[2] = normal_CAD->points[i].normal_z;

        pt_query[0] = cloud_input->points[i].x;
        pt_query[1] = cloud_input->points[i].y;
        pt_query[2] = cloud_input->points[i].z;

        v1 = (sensor_origin-pt_query); // 현재 점에서 sensor origin을 향하는 방향벡터
        v1.normalize();

        if (!m_math.dotCheck2(v1, v2, thre_angle)) // 센서를 향하는 방향벡터와의 사잇각이 80deg보다 크면 flip
        {
            normal_CAD->points[i].normal_x  *= -1;
            normal_CAD->points[i].normal_y  *= -1;
            normal_CAD->points[i].normal_z *= -1;
        }
	}
    // update
    pcl::copyPointCloud(*cloud_input, output);
    pcl::copyPointCloud(*normal_CAD, output);
}

/** @brief Korean: 입력 점군 데이터를 구성하는 각 점에 대해 센서좌표계 중심을 향하도록 flip된 법선을 추정하고, 법선이 포함된 점군 데이터를 추출한다.
 * @details 센서 좌표계 기준의 측정 점군의 법선 추정을 위한 방법 - 센서좌표계의 중심 방향에서 visible한 normal을 갖는 점군만 측정된다는 점에 착안
 * @param[in] cloud_src : 입력 점군 데이터
 * @param[out] output : 추출된 법선이 포함된 점군 데이터
 * @param[in] paramK : 법선 추정 시, K-nearest search에 사용되는 파라미터 K
 * @param[in] sensor_origin : 센서좌표계의 중심
 * @param[in] thre_angle : 센서의 허용 임계 각도
 */
void PCLPROCESSING::cloudNormalEst2Measured(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_src, pcl::PointCloud<pcl::PointXYZRGBNormal> &output, int paramK, const Eigen::Vector3d &sensor_origin, double thre_angle)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::Normal>::Ptr normal_CAD(new pcl::PointCloud<pcl::Normal>);
	pcl::copyPointCloud(cloud_src, *cloud_input);

	// normal estimation
    double paramR = 0.0;
	normalEstimation(cloud_input, *normal_CAD, paramK, paramR, 1); // Using K

	// Viewpoint
	Eigen::Vector3d v1;	Eigen::Vector3d v2;
	Eigen::Vector3d pt_query;
	double tmp1 = 0.0;
	double tmp2 = 0.0;
	double angle = 0.0;
	for (size_t i = 0; i < cloud_input->points.size(); i++)
	{
        // selection normal of nearest mesh center 
        v2[0] = normal_CAD->points[i].normal_x;
        v2[1] = normal_CAD->points[i].normal_y;
        v2[2] = normal_CAD->points[i].normal_z;

        pt_query[0] = cloud_input->points[i].x;
        pt_query[1] = cloud_input->points[i].y;
        pt_query[2] = cloud_input->points[i].z;

        v1 = (sensor_origin-pt_query); // 현재 점에서 sensor origin을 향하는 방향벡터
        v1.normalize();

        if (!m_math.dotCheck2(v1, v2, thre_angle)) // 센서를 향하는 방향벡터와의 사잇각이 80deg보다 크면 flip
        {
            normal_CAD->points[i].normal_x  *= -1;
            normal_CAD->points[i].normal_y  *= -1;
            normal_CAD->points[i].normal_z *= -1;
        }
	}
    // update
    pcl::copyPointCloud(*cloud_input, output);
    pcl::copyPointCloud(*normal_CAD, output);
}

/** @brief Korean: 입력 점군 데이터를 구성하는 각 점에 대해 센서좌표계 중심을 향하도록 flip된 법선을 추정하고, 법선이 포함된 점군 데이터를 추출한다.
 * @details 기준 법선과의 각도 비교를 통해 추정된 법선의 방향을 flip, 두 점군은 점 개수 및 인덱스가 같아야 한다.
 * @param[in] cloud_ref : 기준 점군 데이터 (mesh normal)
 * @param[out] cloud_flip : 법선 처리된 점군 데이터 (estimated normal)
 * @param[in] paramK : 법선 추정 시, K-nearest search에 사용되는 파라미터 K
 * @param[in] thre_angle : 센서의 허용 임계 각도
 */
void PCLPROCESSING::cloudNormalCompareAndFilp(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_ref, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_proc, int paramK, double thre_angle)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normal_CAD(new pcl::PointCloud<pcl::Normal>);
	pcl::copyPointCloud(cloud_proc, *cloud_input);

	// normal estimation
    double paramR = 0.0;
	normalEstimation(cloud_input, *normal_CAD, paramK, paramR, 1); // Using K

	// Viewpoint
	Eigen::Vector3d v1;	Eigen::Vector3d v2;
	double tmp1 = 0.0;
	double tmp2 = 0.0;
	double angle = 0.0;
	for (size_t i = 0; i < cloud_input->points.size(); i++)
	{
        // selection normal of nearest mesh center 
        v2[0] = normal_CAD->points[i].normal_x;
        v2[1] = normal_CAD->points[i].normal_y;
        v2[2] = normal_CAD->points[i].normal_z;
        
        // Reference normal
        v1[0] = cloud_ref.points[i].normal_x;
        v1[1] = cloud_ref.points[i].normal_y;
        v1[2] = cloud_ref.points[i].normal_z;

        v1.normalize();

        if (!m_math.dotCheck2(v1, v2, thre_angle)) // 센서를 향하는 방향벡터와의 사잇각이 80deg보다 크면 flip
        {
            normal_CAD->points[i].normal_x  *= -1;
            normal_CAD->points[i].normal_y  *= -1;
            normal_CAD->points[i].normal_z *= -1;
        }
	}
    // update
    pcl::copyPointCloud(*cloud_input, cloud_proc);
    pcl::copyPointCloud(*normal_CAD, cloud_proc);
}

/** @brief Korean: 입력 점군 데이터를 구성하는 각 점에 대해 센서좌표계 중심을 향하도록 flip된 법선을 추정하고, 법선이 포함된 점군 데이터를 추출한다.
 * @details 기준 법선과의 각도 비교를 통해 추정된 법선의 방향을 flip, 두 점군은 점 개수 및 인덱스가 같아야 한다.
 * @param[in] cloud_ref : 기준 점군 데이터 (mesh normal)
 * @param[out] cloud_flip : 법선 처리된 점군 데이터 (estimated normal)
 * @param[in] paramK : 법선 추정 시, K-nearest search에 사용되는 파라미터 K
 * @param[in] thre_angle : 센서의 허용 임계 각도
 */
void PCLPROCESSING::cloudNormalCompareAndFilpRadiusSearch(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_ref, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_proc, double radius, double thre_angle)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normal_CAD(new pcl::PointCloud<pcl::Normal>);
	pcl::copyPointCloud(cloud_proc, *cloud_input);

	// normal estimation
    int paramK = 0;
    double paramR = radius;
	normalEstimation(cloud_input, *normal_CAD, paramK, paramR, 0); // Using radius search

	// Viewpoint
	Eigen::Vector3d v1;	Eigen::Vector3d v2;
	double tmp1 = 0.0;
	double tmp2 = 0.0;
	double angle = 0.0;
	for (size_t i = 0; i < cloud_input->points.size(); i++)
	{
        // selection normal of nearest mesh center 
        v2[0] = normal_CAD->points[i].normal_x;
        v2[1] = normal_CAD->points[i].normal_y;
        v2[2] = normal_CAD->points[i].normal_z;
        
        // Reference normal
        v1[0] = cloud_ref.points[i].normal_x;
        v1[1] = cloud_ref.points[i].normal_y;
        v1[2] = cloud_ref.points[i].normal_z;

        v1.normalize();

        if (!m_math.dotCheck2(v1, v2, thre_angle)) // 센서를 향하는 방향벡터와의 사잇각이 80deg보다 크면 flip
        {
            normal_CAD->points[i].normal_x  *= -1;
            normal_CAD->points[i].normal_y  *= -1;
            normal_CAD->points[i].normal_z *= -1;
        }
	}
    // update
    pcl::copyPointCloud(*cloud_input, cloud_proc);
    pcl::copyPointCloud(*normal_CAD, cloud_proc);
}

/** @brief Korean: Region growing segmentation을 수행한다.
 * @param[in] cloud_in : 입력 점군 데이터
 * @param[out] indices_out : 분할 결과의 인덱스 데이터
 * @param[in] param : 분할 파라미터
 * @param[in] b_plot : false: viewer off, true: viewer on
 */
void PCLPROCESSING::segmentationRGS(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out, std::vector<pcl::PointIndices> &indices_out, CSegParameters &param, bool b_plot)
{
    indices_out.clear();
    int normal_K = param.rgs_normal_K;
    int min_size = param.rgs_min_size;
    int max_size = param.rgs_max_size;
    double thre_angle = param.rgs_thre_angle;
    double thre_curv = param.rgs_thre_curvature;
    int neighbor_K = param.rgs_neighbor_K;
    
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(cloud_in, *cloud);
	// The algorithm requires normals
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(normal_K);
	normal_estimator.compute(*normals);

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	std::vector <pcl::PointIndices> clusters;

	reg.setMinClusterSize(min_size);
	reg.setMaxClusterSize(max_size);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(neighbor_K);
	reg.setInputCloud(cloud);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(thre_angle / 180.0 * M_PI);
	reg.setCurvatureThreshold(thre_curv);

	reg.extract(clusters);

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	for (int i = 0; i < clusters.size(); i++)
	{
		inliers->indices = clusters[i].indices;
        indices_out.push_back(*inliers);
	}

	//// Result check
	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
	std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;


    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::copyPointCloud(*colored_cloud, cloud_out);
	if (b_plot)
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cluster viewer"));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5);

		while (!viewer->wasStopped())
		{
			viewer->spinOnce();
		}
	}
}

/** @brief Korean: Region growing segmentation을 수행한다.
 * @param[in] cloud_in : 입력 점군 데이터
 * @param[out] indices_out : 분할 결과의 인덱스 데이터
 * @param[in] param : 분할 파라미터
 * @param[in] b_plot : false: viewer off, true: viewer on
 */
void PCLPROCESSING::segmentationRGSv2(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out, std::vector<pcl::PointIndices> &indices_out, CSegParameters &param, bool b_plot)
{
    indices_out.clear();
    int normal_K = param.rgs_normal_K;
    int min_size = param.rgs_min_size;
    int max_size = param.rgs_max_size;
    double thre_angle = param.rgs_thre_angle;
    double thre_curv = param.rgs_thre_curvature;
    int neighbor_K = param.rgs_neighbor_K;
    
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_total(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(cloud_in, *cloud);
    pcl::copyPointCloud(cloud_in, *cloud_total);
	
    
    // Ver. 2
//      * @param[in] cloud_src : 입력 점군 데이터
//  * @param[out] output : 추출된 법선이 포함된 점군 데이터
//  * @param[in] paramK : 법선 추정 시, K-nearest search에 사용되는 파라미터 K
//  * @param[in] sensor_origin : 센서좌표계의 중심
//  * @param[in] thre_angle : 센서의 허용 임계 각도
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normal(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    Eigen::Vector3d sensor_origin = {0.0, 0.0, 0.0};
    cloudNormalEst2Measured(cloud_in, *cloud_normal, normal_K, sensor_origin, 85.0);
    pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    if(0)
    {
        // The algorithm requires normals
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(cloud);
        normal_estimator.setKSearch(normal_K);
        normal_estimator.compute(*normals);
    }
    else
    {
        pcl::copyPointCloud(*cloud_normal, *normals);
    }





	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	std::vector <pcl::PointIndices> clusters;

	reg.setMinClusterSize(min_size);
	reg.setMaxClusterSize(max_size);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(neighbor_K);
	reg.setInputCloud(cloud);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(thre_angle / 180.0 * M_PI);
	reg.setCurvatureThreshold(thre_curv);

	reg.extract(clusters);


    // For merged cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    size_t count = 0;
	for (int i = 0; i < clusters.size(); i++)
	{
		inliers->indices = clusters[i].indices;
        indices_out.push_back(*inliers);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
        extract.setInputCloud(cloud_total);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_tmp);

        // Generate unique colour
        // Visualize point clouds from clouds vector
        pcl::RGB rgb;
        if (i>0 && i % 256 == 0) { count++; }
        rgb = pcl::GlasbeyLUT::at(i - 256 * count);

        for (size_t j=0; j<cloud_tmp->size(); j++) {
            cloud_tmp->points[j].r = rgb.r;
            cloud_tmp->points[j].g = rgb.g;
            cloud_tmp->points[j].b = rgb.b;
        }
        *merged_cloud += *cloud_tmp; // pcmerge
	}

	//// Result check
	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
	std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;

    // update
    pcl::copyPointCloud(*merged_cloud, cloud_out);


	if (b_plot)
	{
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cluster viewer"));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5);

		while (!viewer->wasStopped())
		{
			viewer->spinOnce();
		}

		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("Cluster viewer2"));
		viewer2->setBackgroundColor(0, 0, 0);
		viewer2->addPointCloud<pcl::PointXYZRGB>(merged_cloud);
		viewer2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5);

		while (!viewer2->wasStopped())
		{
			viewer2->spinOnce();
		}
	}
}

/** @brief Korean: Voxel cloud connectivity segmentation을 수행한다.
 * @details 빈피킹 시스템 기능
 * @param[in] cloud_in : 입력 점군 데이터
 * @param[out] indices_out : 분할 결과의 인덱스 데이터
 * @param[in] param : 분할 파라미터
 * @param[in] b_plot : false: viewer off, true: viewer on
 */
void PCLPROCESSING::segmentationVCCS(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out, std::vector<pcl::PointIndices> &indices_out, CSegParameters &param, bool b_plot)
{

}

/** @brief Korean: Color-based region growing segmentation을 수행한다.
 * @details 빈피킹 시스템 기능
 * @param[in] cloud_in : 입력 점군 데이터, metric: [m]
 * @param[out] indices_out : 분할 결과의 인덱스 데이터
 * @param[in] param : 분할 파라미터
 * @param[in] b_plot : false: viewer off, true: viewer on
 */
void PCLPROCESSING::segmentationColorRGS(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out, std::vector<pcl::PointIndices> &indices_out, CSegParameters &param, bool b_plot)
{
    double color_rgs_neighbor_R = 0.001*param.color_rgs_neighbor_R; // ex) 10mm
    double color_rgs_thre_1st_color = param.color_rgs_thre_1st_color; // ex) 6, (normal과 유사) This method specifies the threshold value for color test between the points. This kind of testing is made at the first stage of the algorithm(region growing). If the difference between points color is less than threshold value, then they are considered to be in the same region.
    double color_rgs_thre_2nd_color = param.color_rgs_thre_2nd_color; // ex) 5, (curvature과 유사) threshold value for color test between the regions. This kind of testing is made at the second stage of the algorithm(region merging). If the difference between segments color is less than threshold value, then they are merged together.
    size_t color_rgs_min_cluster_size = param.color_rgs_min_cluster_size; // ex) 600, minimum cluster size, 이 값보다 작으면 근처 cluster에 병합됨.
       
    indices_out.clear();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(cloud_in, *cloud);

    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud, *indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(cloud);
    reg.setIndices(indices);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(color_rgs_neighbor_R);
    reg.setPointColorThreshold(color_rgs_thre_1st_color);
    reg.setRegionColorThreshold(color_rgs_thre_2nd_color);
    reg.setMinClusterSize(color_rgs_min_cluster_size);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);
   

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	for (int i = 0; i < clusters.size(); i++)
	{
		inliers->indices = clusters[i].indices;
        indices_out.push_back(*inliers);
	}

	//// Result check
	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
	std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;

    // For visualizer
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
    pcl::copyPointCloud(*colored_cloud, cloud_out);

    if (0)
    {
        // AABB
        pcl::PointXYZ minPt, maxPt;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_AABB(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(cloud_in, *cloud_AABB);
        pcl::getMinMax3D(*cloud_AABB, minPt, maxPt);

        std::cout << "cloud x min, max: " << minPt.x << maxPt.x << std::endl;
        std::cout << "color_rgs_neighbor_R: " << color_rgs_neighbor_R << std::endl;
        std::cout << "reg.getDistanceThreshold(): " << reg.getDistanceThreshold() << std::endl;
    }

    if (b_plot)
	{
		
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Cluster viewer"));
		viewer->setBackgroundColor(0, 0, 0);
		viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5);
		while (!viewer->wasStopped())
		{
			viewer->spinOnce();
            // viewer->spinOnce(100);
			// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
	}
}

/** @brief Korean: Euclidean cluster extraction을 수행한다.
 * @details 빈피킹 시스템 기능
 * @param[in] cloud_in : 입력 점군 데이터, metric: [m]
 * @param[out] indices_out : 분할 결과의 인덱스 데이터
 * @param[in] param : 분할 파라미터
 * @param[in] b_plot : false: viewer off, true: viewer on
 */
void PCLPROCESSING::segmentationEuclideanClusterExtraction(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out, std::vector<pcl::PointIndices> &indices_out, bool do_plane_removal, CSegParameters &param, bool b_plot)
{
    double euclidean_cluster_tol = 0.001*param.euclidean_cluster_tol; // ex) 0.02 (2cm)
    size_t euclidean_min_cluster_size = param.euclidean_min_cluster_size; // ex) 100
    size_t euclidean_max_cluster_size = param.euclidean_max_cluster_size; // ex) 25000
           
    indices_out.clear();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(cloud_in, *cloud);
    pcl::copyPointCloud(cloud_in, *cloud_filtered);
    // m_math.cloudScaling(*cloud_filtered, 0); // mm to m

    bool do_voxel_grid_filtering = false;
    if(do_voxel_grid_filtering) {
        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        // std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud (cloud_filtered);
        vg.setLeafSize (0.01f, 0.01f, 0.01f);
        vg.filter (*cloud_filtered);
        // std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*
    }

    if(do_plane_removal) {
        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers_planar (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100); // param.
        seg.setDistanceThreshold (0.001); // param.

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
        int nr_points = (int)cloud_filtered->size();
        while (cloud_filtered->size() > 0.3 * nr_points)
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers_planar, *coefficients);
            if (inliers_planar->indices.size() == 0)
            {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers_planar);
            extract.setNegative(false);

            // Get the points associated with the planar surface
            extract.filter(*cloud_plane);
            // std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;

            // Remove the planar inliers, extract the rest
            extract.setNegative(true);
            extract.filter(*cloud_f);
            *cloud_filtered = *cloud_f;
        }
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(euclidean_cluster_tol);
    ec.setMinClusterSize(euclidean_min_cluster_size);
    ec.setMaxClusterSize(euclidean_max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    for (const auto &cluster : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &idx : cluster.indices)
        {
            cloud_cluster->push_back((*cloud_filtered)[idx]);
        } //*
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
    }


	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	for (int i = 0; i < cluster_indices.size(); i++)
	{
		inliers->indices = cluster_indices[i].indices;
        indices_out.push_back(*inliers);
	}

	//// Result check
	// std::cout << "Number of clusters is equal to " << cluster_indices.size() << std::endl;
    if(cluster_indices.size()==0) { return; }
	// std::cout << "First cluster has " << cluster_indices[0].indices.size() << " points." << endl;
	// std::cout << "These are the indices of the points of the initial" <<
	// 	std::endl << "cloud that belong to the first cluster:" << std::endl;

    // For visualizer
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    size_t count = 0;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    for (size_t i = 0; i < cluster_indices.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plot(new pcl::PointCloud<pcl::PointXYZRGB>);
        inliers->indices = cluster_indices[i].indices;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plot);
        
        // Generate unique colour
        // Visualize point clouds from clouds vector
        pcl::RGB rgb;
        if (i>0 && i % 256 == 0) { count++; }
        rgb = pcl::GlasbeyLUT::at(i - 256 * count);

        for (size_t j=0; j<cloud_plot->size(); j++) {
            cloud_plot->points[j].r = rgb.r;
            cloud_plot->points[j].g = rgb.g;
            cloud_plot->points[j].b = rgb.b;
        }
        *colored_cloud += *cloud_plot; // pcmerge
    }
    // output
    pcl::copyPointCloud(*colored_cloud, cloud_out);

	if (b_plot)
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Euclidean cluster extraction viewer"));
		viewer->setBackgroundColor(0, 0, 0);
        std::stringstream cloud_name, line_name;
        size_t count = 0;
	    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        for (size_t i = 0; i < cluster_indices.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plot(new pcl::PointCloud<pcl::PointXYZRGB>);
    		inliers->indices = cluster_indices[i].indices;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cloud_plot);
            
            // Generate unique colour
            cloud_name.str("");
            cloud_name << "Cloud " << i + 1;

            // Visualize point clouds from clouds vector
            pcl::RGB rgb;
            if (i>0 && i % 256 == 0) { count++; }
            rgb = pcl::GlasbeyLUT::at(i - 256 * count);

            // Create colour handle
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> colour_handle(cloud_plot, rgb.r, rgb.g, rgb.b);

            // Add points to viewer and set parameters
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud_plot, colour_handle, cloud_name.str());
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloud_name.str());
        }
        while (!viewer->wasStopped())
        {
            viewer->spinOnce();
        }
	}
}

/** @brief Korean: Conditional Euclidean clustering을 수행한다.
 * @details 빈피킹 시스템 기능
 * @param[in] cloud_in : 입력 점군 데이터
 * @param[out] indices_out : 분할 결과의 인덱스 데이터
 * @param[in] param : 분할 파라미터
 * @param[in] b_plot : false: viewer off, true: viewer on
 */
void PCLPROCESSING::segmentationConditionalEuclideanClustering(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out, std::vector<pcl::PointIndices> &indices_out, CSegParameters &param, bool b_plot)
{
//   // Data containers used
//   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_process (new pcl::PointCloud<pcl::PointXYZI>);
//   pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
//   pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters), small_clusters (new pcl::IndicesClusters), large_clusters (new pcl::IndicesClusters);
//   pcl::search::KdTree<pcl::PointXYZI>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZI>);
//   pcl::console::TicToc tt;

//     pcl::copyPointCloud(*cloud_in, *cloud_process);
//   // Set up a Normal Estimation class and merge data in cloud_with_normals
//   std::cerr << "Computing normals...\n", tt.tic ();
//   pcl::copyPointCloud (*cloud_out, *cloud_with_normals);
//   pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> ne;
//   ne.setInputCloud (cloud_out);
//  ne.setSearchMethod (search_tree);
//   ne.setRadiusSearch (300.0);
//   ne.compute (*cloud_with_normals);
//   std::cerr << ">> Done: " << tt.toc () << " ms\n";

//   // Set up a Conditional Euclidean Clustering class
//   std::cerr << "Segmenting to clusters...\n", tt.tic ();
//   pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> cec (true);
//   cec.setInputCloud (cloud_with_normals);
//   cec.setConditionFunction (&customRegionGrowing);
//   cec.setClusterTolerance (500.0);
//   cec.setMinClusterSize (cloud_with_normals->size () / 1000);
//   cec.setMaxClusterSize (cloud_with_normals->size () / 5);
//   cec.segment (*clusters);
//   cec.getRemovedClusters (small_clusters, large_clusters);
//   std::cerr << ">> Done: " << tt.toc () << " ms\n";

//   // Using the intensity channel for lazy visualization of the output
//   for (const auto& small_cluster : (*small_clusters))
//     for (const auto& j : small_cluster.indices)
//       (*cloud_out)[j].intensity = -2.0;
//  for (const auto& large_cluster : (*large_clusters))
//    for (const auto& j : large_cluster.indices)
//      (*cloud_out)[j].intensity = +10.0;
//  for (const auto& cluster : (*clusters))
//   {
//     int label = rand () % 8;
//     for (const auto& j : cluster.indices)
//       (*cloud_out)[j].intensity = label;
//   }

//   // Save the output point cloud
//   std::cerr << "Saving...\n", tt.tic ();
//   pcl::io::savePCDFile ("output.pcd", *cloud_out);
//   std::cerr << ">> Done: " << tt.toc () << " ms\n";

}

/** @brief Korean: Segmentation을 통해 추출된 인덱스를 입력 받아 각각의 cluster를 추출한다.
 * @details 빈피킹 시스템 기능
 * @param[in] cloud_in : 입력 점군 데이터, metric: [m]
 * @param[in] indices_in : 분할 결과의 인덱스 데이터
 * @param[out] output : cluster list
 */
void PCLPROCESSING::extractClusterList(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, const std::vector<pcl::PointIndices> &indices_in, std::vector<pcl::PointCloud<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PointXYZRGBA> > &output, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_merge)
{
    output.clear();
    // std::cout << "--- extractClusterList()! --- cluster num: " << indices_in.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(cloud_in, *cloud);

    // if(indices_in.size() < 2)
    if(indices_in.size() == 0)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
        *cloud_cluster = *cloud;
        cloud_cluster->width = cloud->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        output.push_back(*cloud_cluster);
        // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plot(new pcl::PointCloud<pcl::PointXYZRGBA>);
        *cloud_plot = *cloud;
        
        // Generate unique colour
        // Visualize point clouds from clouds vector
        pcl::RGB rgb;
        rgb = pcl::GlasbeyLUT::at(0);
        for (size_t j=0; j<cloud_plot->size(); j++) {
            cloud_plot->points[j].r = rgb.r;
            cloud_plot->points[j].g = rgb.g;
            cloud_plot->points[j].b = rgb.b;
        }
        *colored_cloud += *cloud_plot; // pcmerge
    }
    else
    {
        std::vector<pcl::PointIndices> cluster_indices = indices_in;
        for (const auto &cluster : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
            for (const auto &idx : cluster.indices)
            {
                cloud_cluster->push_back((*cloud)[idx]);
            } //*
            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            output.push_back(*cloud_cluster);

            // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        }

        // For visualizer
        size_t count = 0;
        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        for (size_t i = 0; i < cluster_indices.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plot(new pcl::PointCloud<pcl::PointXYZRGBA>);
            inliers->indices = cluster_indices[i].indices; // 분할영역 각각에 대한 input cloud의 index
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cloud_plot);
            
            // Generate unique colour
            // Visualize point clouds from clouds vector
            pcl::RGB rgb;
            if (i>0 && i % 256 == 0) { count++; }
            rgb = pcl::GlasbeyLUT::at(i - 256 * count);

            for (size_t j=0; j<cloud_plot->size(); j++) {
                cloud_plot->points[j].r = rgb.r;
                cloud_plot->points[j].g = rgb.g;
                cloud_plot->points[j].b = rgb.b;
            }
            *colored_cloud += *cloud_plot; // pcmerge
        }
    }

    // output
    pcl::copyPointCloud(*colored_cloud, cloud_merge);
}

/** @brief Korean: Segmentation을 통해 추출된 인덱스를 입력 받아 각각의 cluster를 추출한다.
 * @details 빈피킹 시스템 기능
 * @param[in] cloud_in : 입력 점군 데이터, metric: [m]
 * @param[in] indices_in : 분할 결과의 인덱스 데이터
 * @param[out] output : cluster list
 */
void PCLPROCESSING::extractClusterList(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, const std::vector<pcl::PointIndices> &indices_in, std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>, Eigen::aligned_allocator<pcl::PointXYZRGBNormal> > &output, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_merge)
{
    output.clear();
    // std::cout << "--- extractClusterList()! --- cluster num: " << indices_in.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::copyPointCloud(cloud_in, *cloud);

    // if(indices_in.size() < 2)
    if(indices_in.size() == 0)
    {
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        *cloud_cluster = *cloud;
        cloud_cluster->width = cloud->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        output.push_back(*cloud_cluster);
        // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_plot(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        *cloud_plot = *cloud;
        
        // Generate unique colour
        // Visualize point clouds from clouds vector
        pcl::RGB rgb;
        rgb = pcl::GlasbeyLUT::at(0);
        for (size_t j=0; j<cloud_plot->size(); j++) {
            cloud_plot->points[j].r = rgb.r;
            cloud_plot->points[j].g = rgb.g;
            cloud_plot->points[j].b = rgb.b;
        }
        *colored_cloud += *cloud_plot; // pcmerge
    }
    else
    {
        std::vector<pcl::PointIndices> cluster_indices = indices_in;
        for (const auto &cluster : cluster_indices)
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            for (const auto &idx : cluster.indices)
            {
                cloud_cluster->push_back((*cloud)[idx]);
            } //*
            cloud_cluster->width = cloud_cluster->size();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            output.push_back(*cloud_cluster);

            // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
        }

        // For visualizer
        size_t count = 0;
        pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        for (size_t i = 0; i < cluster_indices.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_plot(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            inliers->indices = cluster_indices[i].indices; // 분할영역 각각에 대한 input cloud의 index
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cloud_plot);
            
            // Generate unique colour
            // Visualize point clouds from clouds vector
            pcl::RGB rgb;
            if (i>0 && i % 256 == 0) { count++; }
            rgb = pcl::GlasbeyLUT::at(i - 256 * count);

            for (size_t j=0; j<cloud_plot->size(); j++) {
                cloud_plot->points[j].r = rgb.r;
                cloud_plot->points[j].g = rgb.g;
                cloud_plot->points[j].b = rgb.b;
            }
            *colored_cloud += *cloud_plot; // pcmerge
        }
    }

    // output
    pcl::copyPointCloud(*colored_cloud, cloud_merge);
}

/** @brief Korean: 입력 점군 데이터의 중심을 추출한다.
 * @details 
 * @param[in] cloud_in : 입력 점군 데이터
 * @param[out] output : 추출된 중심
 * @param[out] idx : 추출된 중심의 점 인덱스
 */
void PCLPROCESSING::extractCenter(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Vector3d &output, int &idx)
{
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
	*cloud = cloud_in;

	// Estimate the XYZ centroid
	Eigen::Vector4f xyz_centroid;
	pcl::compute3DCentroid(*cloud, xyz_centroid);

	//// Closest point from centroid in cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr center_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	center_tmp->resize(1);

	center_tmp->points[0].x = xyz_centroid[0];
	center_tmp->points[0].y = xyz_centroid[1];
	center_tmp->points[0].z = xyz_centroid[2];

	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float > > modelTree;
	std::vector<int> nn_indices(1);
	std::vector<float> nn_dists(1);
	modelTree.setInputCloud(cloud->makeShared());
	modelTree.nearestKSearch(center_tmp->points[0], 1, nn_indices, nn_dists);

	Eigen::Vector3d center;
	center[0] = cloud->points[nn_indices[0]].x;
	center[1] = cloud->points[nn_indices[0]].y;
	center[2] = cloud->points[nn_indices[0]].z;

	output = center;
	idx = nn_indices[0];
}

/** @brief Korean: 입력 점군 데이터의 중심을 추출한다.
 * @details 
 * @param[in] cloud_in : 입력 점군 데이터
 * @param[out] output : 추출된 중심
 * @param[out] idx : 추출된 중심의 점 인덱스
 */
void PCLPROCESSING::extractCenter(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Vector3f &output, int &idx)
{
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
	*cloud = cloud_in;

	// Estimate the XYZ centroid
	Eigen::Vector4f xyz_centroid;
	pcl::compute3DCentroid(*cloud, xyz_centroid);

	//// Closest point from centroid in cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr center_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	center_tmp->resize(1);

	center_tmp->points[0].x = xyz_centroid[0];
	center_tmp->points[0].y = xyz_centroid[1];
	center_tmp->points[0].z = xyz_centroid[2];

	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float > > modelTree;
	std::vector<int> nn_indices(1);
	std::vector<float> nn_dists(1);
	modelTree.setInputCloud(cloud->makeShared());
	modelTree.nearestKSearch(center_tmp->points[0], 1, nn_indices, nn_dists);

	Eigen::Vector3f center;
	center[0] = cloud->points[nn_indices[0]].x;
	center[1] = cloud->points[nn_indices[0]].y;
	center[2] = cloud->points[nn_indices[0]].z;

	output = center;
	idx = nn_indices[0];
}

/** @brief Korean: 입력 점군 데이터의 중심을 추출한다.
 * @details 
 * @param[in] cloud_in : 입력 점군 데이터
 * @param[out] output : 추출된 중심
 * @param[out] idx : 추출된 중심의 점 인덱스
 */
void PCLPROCESSING::extractCenterWithNormal(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, Eigen::Vector3f &center, Eigen::Vector3f &normal, int &idx)
{
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
	pcl::copyPointCloud(cloud_in, *cloud);

	// Estimate the XYZ centroid
	Eigen::Vector4f xyz_centroid;
	pcl::compute3DCentroid(*cloud, xyz_centroid);

	//// Closest point from centroid in cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr center_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	center_tmp->resize(1);

	center_tmp->points[0].x = xyz_centroid[0];
	center_tmp->points[0].y = xyz_centroid[1];
	center_tmp->points[0].z = xyz_centroid[2];

	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float > > modelTree;
	std::vector<int> nn_indices(1);
	std::vector<float> nn_dists(1);
	modelTree.setInputCloud(cloud->makeShared());
	modelTree.nearestKSearch(center_tmp->points[0], 1, nn_indices, nn_dists);

	center[0] = cloud_in.points[nn_indices[0]].x;
	center[1] = cloud_in.points[nn_indices[0]].y;
	center[2] = cloud_in.points[nn_indices[0]].z;
    normal[0] = cloud_in.points[nn_indices[0]].normal_x;
    normal[1] = cloud_in.points[nn_indices[0]].normal_y;
    normal[2] = cloud_in.points[nn_indices[0]].normal_z;

	idx = nn_indices[0];
}

/** @brief Korean: 입력된 query point에서 가장 가까운 점군 데이터의 점 및 법선을 추출한다.
 */
void PCLPROCESSING::extractNearestPointWithNormal(const Eigen::Vector3f &pt_query, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, Eigen::Vector3f &pt_nearest, Eigen::Vector3f &normal_nearest, int &idx)
{
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
	pcl::copyPointCloud(cloud_in, *cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr pt_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	pt_tmp->resize(1);
	pt_tmp->points[0].x = pt_query[0];
	pt_tmp->points[0].y = pt_query[1];
	pt_tmp->points[0].z = pt_query[2];

	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> modelTree;
	std::vector<int> nn_indices(1);
	std::vector<float> nn_dists(1);
	modelTree.setInputCloud(cloud->makeShared());
	modelTree.nearestKSearch(pt_tmp->points[0], 1, nn_indices, nn_dists);

	pt_nearest[0] = cloud_in.points[nn_indices[0]].x;
	pt_nearest[1] = cloud_in.points[nn_indices[0]].y;
	pt_nearest[2] = cloud_in.points[nn_indices[0]].z;
    normal_nearest[0] = cloud_in.points[nn_indices[0]].normal_x;
    normal_nearest[1] = cloud_in.points[nn_indices[0]].normal_y;
    normal_nearest[2] = cloud_in.points[nn_indices[0]].normal_z;

	idx = nn_indices[0];
}

/** @brief Korean: 대상 물체의 CAD를 이용하여 OctoMap을 생성한다.
 */
void PCLPROCESSING::genOctoMapCAD(CPLANNINGDATA &plan_data)
{
	// plan_data.m_OctoMap_CAD = genTree(0.8f); // scale: 0.8mm
	plan_data.m_OctoMap_CAD = genTree(plan_data.map_voxel_length); // scale: [mm]
    
	octomap::point3d origin(0.0f, 0.0f, 0.0f);
	octomap::Pointcloud pt_cloud_CAD;
	octomap::point3d pt_tmp(0.0f, 0.0f, 0.0f);

	for (unsigned i = 0; i < plan_data.m_cloud_CAD->size(); i++)
	{
		pt_tmp(0) = plan_data.m_cloud_CAD->points[i].x;
		pt_tmp(1) = plan_data.m_cloud_CAD->points[i].y;
		pt_tmp(2) = plan_data.m_cloud_CAD->points[i].z;
		pt_cloud_CAD.push_back(pt_tmp);
	}
	double maxRange = 320; // 320mm, 최대 거리 범위(센서의 주사 거리보다 더 크게 설정)
	plan_data.m_OctoMap_CAD->insertPointCloud(pt_cloud_CAD, origin, maxRange); // input pointcloud, sensor origin 

	// kd tree model
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> md_KdTree_octoMap_CAD;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_CAD(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*plan_data.m_cloud_CAD, *cloud_CAD);
	md_KdTree_octoMap_CAD.setInputCloud(cloud_CAD->makeShared());
	plan_data.md_KdTree_octoMap_CAD = md_KdTree_octoMap_CAD;
}

/** @brief Korean: 대상 물체의 CAD를 이용하여 OctoMap을 생성한다.
* @param[in] fileName : 회전판의 CAD 파일명
 */
void PCLPROCESSING::genOctoMapCAD2(CPLANNINGDATA &plan_data, double maxRange)
{
	// plan_data.m_OctoMap_CAD = genTree(0.8f); // scale: 0.8mm
	plan_data.m_OctoMap_CAD = genTree(plan_data.map_voxel_length); // scale: [mm]
    
	octomap::point3d origin(0.0f, 0.0f, 0.0f);
	octomap::Pointcloud pt_cloud_CAD;
	octomap::point3d pt_tmp(0.0f, 0.0f, 0.0f);

	for (unsigned i = 0; i < plan_data.m_cloud_CAD->size(); i++)
	{
		pt_tmp(0) = plan_data.m_cloud_CAD->points[i].x;
		pt_tmp(1) = plan_data.m_cloud_CAD->points[i].y;
		pt_tmp(2) = plan_data.m_cloud_CAD->points[i].z;
		pt_cloud_CAD.push_back(pt_tmp);
	}
	// double maxRange = 320; // 320mm, 최대 거리 범위(센서의 주사 거리보다 더 크게 설정)
	plan_data.m_OctoMap_CAD->insertPointCloud(pt_cloud_CAD, origin, maxRange); // input pointcloud, sensor origin 

	// kd tree model
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> md_KdTree_octoMap_CAD;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_CAD(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*plan_data.m_cloud_CAD, *cloud_CAD);
	md_KdTree_octoMap_CAD.setInputCloud(cloud_CAD->makeShared());
	plan_data.md_KdTree_octoMap_CAD = md_KdTree_octoMap_CAD;
}

/** @brief Korean: OctoMap 생성을 위한 octomap 포인터를 반환한다.
 * @param[in] resolution : OctoMap을 구성하는 단위 voxel의 resolution(voxel의 한 변의 길이)
 * @return tree : octomap::OcTree* 타입으로 생성된 포인터를 반환
 */
octomap::OcTree* PCLPROCESSING::genTree(float resolution)
{
	octomap::OcTree* tree = new octomap::OcTree(resolution);
	return tree;
}

/** @brief Korean: 입력된 점에 대한 복셀 지도 상의 점유(occupancy) 여부를 반환한다.
 * @details -
 * @param[in] pt1 : 점 1
 * @param[in] pt2 : 점 2
 * @param[in] OctoMap_CAD : 복셀 지도
 * @param[in] cnt_ray : ray-projection에 사용되는 점 개수
 * @param[in] thre_map : 가려짐 판별을 위한 임계 파라미터(0.0~1.0), 보통 0.6 사용
 * @return 가려짐 여부를 반환
 * @retval true : 가려짐 발생
 * @retval false : 가려짐 없음
 */
bool PCLPROCESSING::checkSinglePointOccupancy(const Eigen::Vector3d& pt, octomap::OcTree* OctoMap_CAD, double thre_map)
{
	octomap::MapCollection<octomap::MapNode<octomap::OcTree>> collection;
	octomap::MapNode<octomap::OcTree>* mn1 = new octomap::MapNode<octomap::OcTree>(OctoMap_CAD, octomap::pose6d(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	mn1->setId("CAD");
	collection.addNode(mn1);
	OctoMap_CAD->setOccupancyThres(thre_map); // 점유 파라미터(thre_map) 이상이면 점유(occupied)로 판정

	std::vector<octomap::point3d> query;
	octomap::point3d point_tmp;
	point_tmp(0) = pt[0]; point_tmp(1) = pt[1]; point_tmp(2) = pt[2];
    query.push_back(point_tmp);

	// Occlusion check
	int cnt_occupancy = 0; // 주사된 벡터가 복셀 지도를 통과하지 못하고, 계속 가려짐으로 판별된 횟수
	for (std::vector<octomap::point3d>::iterator it = query.begin(); it != query.end(); ++it)
	{
		octomap::point3d& q = *it;
		if (collection.isOccupied(q)) 
        { 
            cnt_occupancy = cnt_occupancy + 1; 
        }
        else
        {
            // printf("no occu.\n");
        }
	}
	if (cnt_occupancy > 0) { return true; }
	else { return false; }
}

/** @brief Korean: 입력된 점에 대한 복셀 지도 상의 점유(occupancy) 여부를 반환한다.
 * @details -
 * @return 결과 상태를 반환
 * @retval 1 : 가려짐 발생
 * @retval 0 : 가려짐 없음
 * @retval -1 : input points have nan point
 */
int PCLPROCESSING::checkSinglePointOccupancyUsingMapCollection(const octomap::point3d& q, octomap::MapCollection<octomap::MapNode<octomap::OcTree>> &collection)
{
    for (int i = 0; i < 3; i++)
    {
        if(std::isnan(q(i))) 
        {
            printf("Input point is NAN! OctoMap - check the input cloud\n");
            return -1;
        }
    }
	int cnt_occupancy = 0; // 주사된 벡터가 복셀 지도를 통과하지 못하고, 계속 가려짐으로 판별된 횟수
    if (collection.isOccupied(q)) cnt_occupancy = cnt_occupancy + 1; 
	if (cnt_occupancy > 0) { return 1; }
	else { return 0; }
}


void PCLPROCESSING::extractOBBLength(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, std::vector<double> &ascending_order_sorted_list, bool b_plot)
{
    //// OBB (Object-oriented Bounding Box)
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(cloud_in, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(cloud_in, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(cloud_in, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    Eigen::Vector3f box_dim;
    box_dim = maxPoint.getVector3fMap() - minPoint.getVector3fMap();
    // printf("Box dimension: %0.3f, %0.3f, %0.3f\n", box_dim[0], box_dim[1], box_dim[2]);

    std::vector<double> box_dim_list(3);
    for (int k = 0; k < 3; k++) { box_dim_list[k] = box_dim[k]; }
    std::vector<double> box_dim_sorted_list;
    std::vector<size_t> box_dim_sorted_idx_list;
    m_math.genAscendingOrder(box_dim_list, box_dim_sorted_list, box_dim_sorted_idx_list);

    // output
    ascending_order_sorted_list = box_dim_sorted_list;

    if(b_plot) // plot
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(cloud_in, *cloud_plot);
        pcl::visualization::PCLVisualizer viewer;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_1(cloud_plot, 0, 0, 255);	
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_2(cloudPointsProjected, 255, 0, 0);	
        viewer.addPointCloud(cloud_plot, handler_1, "cloud1");
        viewer.addPointCloud(cloudPointsProjected, handler_2, "cloud2");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");
        // const Eigen::Quaternionf bboxQ(keep_Z_Rot);
        // const Eigen::Vector3f    bboxT(transform3.translation()); 
        viewer.addCube(bboxTransform, bboxQuaternion, box_dim(0), box_dim(1), box_dim(2), "bbox");
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox");
        viewer.addCoordinateSystem(1.0);
        viewer.setBackgroundColor(1, 1, 1);
        viewer.spin();
    }
}

void PCLPROCESSING::extractOBBTransform(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, Eigen::Matrix4f &output, bool b_plot)
{
    //// OBB (Object-oriented Bounding Box)
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(cloud_in, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(cloud_in, pcaCentroid, covariance);
    // Eigen::Matrix4f cov_tmp = Eigen::Matrix4f::Identity();
    // cov_tmp.block<3, 3>(0, 0) = covariance;
    // m_math.printHTM(cov_tmp, "covariance");

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but

    Eigen::Matrix4f input2projectionTransform = Eigen::Matrix4f::Identity();
    input2projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA;
    input2projectionTransform.block<3, 1>(0, 3) = pcaCentroid.head<3>();
    // m_math.printHTM(input2projectionTransform, "eigenVectorsPCA");
    output = input2projectionTransform;

    // 추출 순서는 경계박스의 한 변의 길이가 짧은 순
    if(b_plot) // plot
    {
        // Transform the original cloud to the origin where the principal components correspond to the axes.
        Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
        projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
        projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(cloud_in, *cloudPointsProjected, projectionTransform);

        // Get the minimum and maximum points of the transformed cloud.
        pcl::PointXYZ minPoint, maxPoint;
        pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
        const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

        // Final transform
        const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
        const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();


        Eigen::Vector3f box_dim;
        box_dim = maxPoint.getVector3fMap() - minPoint.getVector3fMap();
        printf("Box dimension: %0.3f, %0.3f, %0.3f\n", box_dim[0], box_dim[1], box_dim[2]);
        printf("bboxTransform: %0.3f, %0.3f, %0.3f\n", bboxTransform[0], bboxTransform[1], bboxTransform[2]);
        // printf("Quaternionf: %0.3f, %0.3f, %0.3f\n", bboxTransform[0], bboxTransform[1], bboxTransform[2]);


        std::vector<double> box_dim_list(3);
        for (int k = 0; k < 3; k++) { box_dim_list[k] = box_dim[k]; }
        std::vector<double> box_dim_sorted_list;
        std::vector<size_t> box_dim_sorted_idx_list;
        m_math.genAscendingOrder(box_dim_list, box_dim_sorted_list, box_dim_sorted_idx_list);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(cloud_in, *cloud_plot);
        pcl::visualization::PCLVisualizer viewer;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_1(cloud_plot, 0, 0, 255);	
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> handler_2(cloudPointsProjected, 255, 0, 0);	
        viewer.addPointCloud(cloud_plot, handler_1, "cloud1");
        viewer.addPointCloud(cloudPointsProjected, handler_2, "cloud2");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");
        // const Eigen::Quaternionf bboxQ(keep_Z_Rot);
        // const Eigen::Vector3f    bboxT(transform3.translation()); 
        viewer.addCube(bboxTransform, bboxQuaternion, box_dim(0), box_dim(1), box_dim(2), "bbox");
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox");
        viewer.addCoordinateSystem(1.0);
        viewer.setBackgroundColor(1, 1, 1);
        viewer.spin();
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////// ALIGNMENT /////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/** @brief Korean: Quick align 오차 감소를 위한 처리 함수, 
 * details 바닥면이 평면인 형상의 물체가 바닥에 놓여진 경우에 사용.
 */
void PCLPROCESSING::modelMatchingBottomPlaneShapedPreProcessing(const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot) {
	// printf("\n--------------  Pre processing!  -----------------\n");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(cloud_CAD, *cloud_target); // Reference model (CAD model)
	*cloud_source = cloud_reg;	
    Eigen::Matrix4f T_now = Eigen::Matrix4f::Identity();

    //// initialize the source cloud
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_in);
    T_now = T_in;


    //// 미리 CAD 근처로 translation
    Eigen::Vector4f centroid_aligned;
    Eigen::Vector4f centroid_CAD;
    pcl::compute3DCentroid(*cloud_source, centroid_aligned);
    pcl::compute3DCentroid(*cloud_target, centroid_CAD);

    // printf("centroid_aligned: %f, %f, %f\n", centroid_aligned[0], centroid_aligned[1], centroid_aligned[2]);
    // printf("centroid_CAD: %f, %f, %f\n", centroid_CAD[0], centroid_CAD[1], centroid_CAD[2]);
    Eigen::Matrix4f T_trans_tmp = Eigen::Matrix4f::Identity();
    
    // a) to CAD centroid
    T_trans_tmp(0, 3) = -centroid_aligned[0];
    T_trans_tmp(1, 3) = -centroid_aligned[1];
    T_trans_tmp(2, 3) = -centroid_aligned[2];
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_trans_tmp);
    T_now = T_trans_tmp * T_now;

    // b) z translation
    pcl::PointXYZ minPt_aligned, maxPt_aligned;
    pcl::PointXYZ minPt_CAD, maxPt_CAD;
    pcl::getMinMax3D(*cloud_source, minPt_aligned, maxPt_aligned);
    pcl::getMinMax3D(*cloud_target, minPt_CAD, maxPt_CAD);

    T_trans_tmp(0, 3) = 0.0;
    T_trans_tmp(1, 3) = 0.0;
    T_trans_tmp(2, 3) = maxPt_CAD.z - maxPt_aligned.z;
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_trans_tmp);
    T_now = T_trans_tmp * T_now;

    //// c) centroid 2, 221213
    pcl::compute3DCentroid(*cloud_source, centroid_aligned);
    T_trans_tmp(0, 3) = centroid_CAD[0]-centroid_aligned[0];
    T_trans_tmp(1, 3) = centroid_CAD[1]-centroid_aligned[1];
    T_trans_tmp(2, 3) = 0.0;
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_trans_tmp);
    T_now = T_trans_tmp * T_now;

    // output
    T_out = T_now;

	// Visualization
	if (b_plot)
	{
		pcl::visualization::PCLVisualizer viewer("raw data before quick align");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text


		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*cloud_target, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud_target, 0, 0, 0);
		viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// // raw cloud is green
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(Registered, 10, 10, 255);
		// viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_align_color_h(cloud_source, 10, 10, 255);
		viewer.addPointCloud(cloud_source, cloud_align_color_h, "cloud_source", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
            viewer.spin();
			// viewer.spinOnce(100);
			// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		viewer.close();
	}
}


/** @brief Korean: Quick align 오차 감소를 위한 처리 함수, 
 * details 위아래가 대칭인 경우, 뒤집힘 체크 후, centroid 이동
 */
void PCLPROCESSING::modelMatchingPreProcessingFlipCheckAndCentroid(CTARGET_OBJECT_DATA* &target_object_data, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot) {
	// printf("\n--------------  Pre processing!  -----------------\n");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source_rgbn(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::copyPointCloud(cloud_CAD, *cloud_target); // Reference model (CAD model)
	*cloud_source_rgbn = cloud_reg;	

    //// initialize the source cloud
    pcl::transformPointCloudWithNormals(*cloud_source_rgbn, *cloud_source_rgbn, T_in);
	pcl::copyPointCloud(*cloud_source_rgbn, *cloud_source);

    Eigen::Matrix4f T_now = Eigen::Matrix4f::Identity();
    T_now = T_in;


    // //////////////////////////////////////////////////////////////////////////////////////

    // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_flip_check(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	// pcl::copyPointCloud(*cloud_source_rgbn, *cloud_flip_check);
                
    // //// TODO: JSON과 연동
    // Eigen::Vector3f vec_reference(0.0, 0.0, 1.0);
    // Eigen::Vector3f vec_flip_check;
    // ROS_LOG_WARN("modelMatchingPreProcessingFlipCheckAndCentroid (Filp Check) - Start");
    // target_object_data->is_object_pose_flipped = m_pcl_proc.checkObjectCloudFlip(*cloud_flip_check, vec_reference, target_object_data->cad_max_pt.z, *cloud_flip_check, vec_flip_check, T_result_post_proc);
    // target_object_data->measured_object_normal_for_flip_check = vec_flip_check;

    // for (size_t i = 0; i < cloud_flip_check->size(); i++) {
    //     if(!std::isnan(cloud_flip_check->points[i].normal_x) && !std::isnan(cloud_flip_check->points[i].normal_y) && !std::isnan(cloud_flip_check->points[i].normal_z)) {
    //         Eigen::Vector3f vec_normal_tmp;
    //         vec_normal_tmp[0] = cloud_flip_check->points[i].normal_x;
    //         vec_normal_tmp[1] = cloud_flip_check->points[i].normal_y;
    //         vec_normal_tmp[2] = cloud_flip_check->points[i].normal_z;

    //         vec_flip_check += vec_normal_tmp;
    //         vec_flip_check.normalize();
    //     } else {
    //         printf("Normal has NAN!\n");
    //     }
    // }

    // //// TODO: vec_flip_check: -0.000, -0.000, -0.000이 나오는 경우가 발생하는 데, 이를 방지하도록 처리하기
    // printf("vec_flip_check: %0.3f, %0.3f, %0.3f\n", vec_flip_check[0], vec_flip_check[1], vec_flip_check[2]);
    // target_object_data->measured_object_normal_for_flip_check = vec_flip_check;

    // if(!m_math.dotCheck2(vec_reference, vec_flip_check, 90.0)) { // 사잇각이 90deg보다 크면
    //     printf("\n-------------- Plane-shaped cloud post processing! 180 deg w.r.t {O} Flipped!  -----------------\n");
    //     Eigen::Matrix4f T_trans_flip_1 = Eigen::Matrix4f::Identity();
    //     T_trans_flip_1(2, 3) = -target_object_data->cad_max_pt.z;
    //     // 1) 물체 좌표계의 중심으로 translation
    //     pcl::transformPointCloudWithNormals(*cloud_flip_check, *cloud_flip_check, T_trans_flip_1);
    //     // m_math.printCloudMinMax(*cloud_flip_check, "cloud_flip_check - 1");
    //     // 2) 물체 좌표계의 y축 기준으로 180deg 회전
    //     Eigen::Matrix3f R_rot_flip;
    //     Eigen::Matrix4f T_rot_flip;
    //     m_math.genRotM(180.0, R_rot_flip, T_rot_flip, 1);
    //     pcl::transformPointCloudWithNormals(*cloud_flip_check, *cloud_flip_check, T_rot_flip);
    //     // m_math.printCloudMinMax(*cloud_flip_check, "cloud_flip_check - 2");
    //     // 3) 원래 위치로 translation
    //     Eigen::Matrix4f T_trans_flip_2 = Eigen::Matrix4f::Identity();
    //     T_trans_flip_2(2, 3) = target_object_data->cad_max_pt.z;
    //     pcl::transformPointCloudWithNormals(*cloud_flip_check, *cloud_flip_check, T_trans_flip_2);
    //     // m_math.printCloudMinMax(*cloud_flip_check, "cloud_flip_check - 3");

    //     T_now = T_trans_flip_2*T_rot_flip*T_trans_flip_1*T_now;
    //     target_object_data->is_object_pose_flipped = true;

    // } else {
    //     ROS_LOG_WARN("\n-------------- is_object_pose_flipped: false ---- [doTemplateMatchingProcess]\n");
    //     target_object_data->is_object_pose_flipped = false;
    // }
    // ROS_LOG_WARN("modelMatchingPreProcessingFlipCheckAndCentroid (Filp Check) - End");

    // //////////////////////////////////////////////////////////////////////////////////////


    //// 미리 CAD 근처로 translation
    Eigen::Vector4f centroid_aligned;
    Eigen::Vector4f centroid_CAD;
    pcl::compute3DCentroid(*cloud_source, centroid_aligned);
    pcl::compute3DCentroid(*cloud_target, centroid_CAD);

    // printf("centroid_aligned: %f, %f, %f\n", centroid_aligned[0], centroid_aligned[1], centroid_aligned[2]);
    // printf("centroid_CAD: %f, %f, %f\n", centroid_CAD[0], centroid_CAD[1], centroid_CAD[2]);
    Eigen::Matrix4f T_trans_tmp = Eigen::Matrix4f::Identity();
    
    // a) to CAD centroid
    T_trans_tmp(0, 3) = -centroid_aligned[0];
    T_trans_tmp(1, 3) = -centroid_aligned[1];
    T_trans_tmp(2, 3) = -centroid_aligned[2];
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_trans_tmp);
    T_now = T_trans_tmp * T_now;

    // b) z translation
    pcl::PointXYZ minPt_aligned, maxPt_aligned;
    pcl::PointXYZ minPt_CAD, maxPt_CAD;
    pcl::getMinMax3D(*cloud_source, minPt_aligned, maxPt_aligned);
    pcl::getMinMax3D(*cloud_target, minPt_CAD, maxPt_CAD);

    T_trans_tmp(0, 3) = 0.0;
    T_trans_tmp(1, 3) = 0.0;
    T_trans_tmp(2, 3) = maxPt_CAD.z - maxPt_aligned.z;
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_trans_tmp);
    T_now = T_trans_tmp * T_now;

    //// c) centroid 2, 221213
    pcl::compute3DCentroid(*cloud_source, centroid_aligned);
    T_trans_tmp(0, 3) = centroid_CAD[0]-centroid_aligned[0];
    T_trans_tmp(1, 3) = centroid_CAD[1]-centroid_aligned[1];
    T_trans_tmp(2, 3) = 0.0;
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_trans_tmp);
    T_now = T_trans_tmp * T_now;

    // output
    T_out = T_now;

	// Visualization
	if (b_plot)
	{
		pcl::visualization::PCLVisualizer viewer("raw data before quick align");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text


		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*cloud_target, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud_target, 0, 0, 0);
		viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// // raw cloud is green
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(Registered, 10, 10, 255);
		// viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_align_color_h(cloud_source, 10, 10, 255);
		viewer.addPointCloud(cloud_source, cloud_align_color_h, "cloud_source", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
            viewer.spin();
			// viewer.spinOnce(100);
			// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		viewer.close();
	}
}

/** @brief Korean: Quick align 오차 감소를 위한 처리 함수, 
 * details 특정 축 기준으로 특정 각도로 회전한 후에 무게중심으로 병진 이동.
 */
void PCLPROCESSING::modelMatchingComplexShapedPreProcessing(const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot) {
	// printf("\n--------------  Pre processing!  -----------------\n");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(cloud_CAD, *cloud_target); // Reference model (CAD model)
	*cloud_source = cloud_reg;	
    Eigen::Matrix4f T_now = Eigen::Matrix4f::Identity();

    //// initialize the source cloud
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_in);
    T_now = T_in;

    //// TODO: 회전 행렬
    //// TODO: 일단, y축 90도 회전만 적용
    //// TODO:
    //// TODO:
    //// TODO:
    Eigen::Matrix3f R_Ry;
    Eigen::Matrix4f T_Ry;
    m_math.genRotM(0.0, R_Ry, T_Ry, 1); // Ry
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_Ry);
    T_now = T_Ry * T_now;


    //// 미리 CAD 근처로 translation
    Eigen::Vector4f centroid_aligned;
    Eigen::Vector4f centroid_CAD;
    pcl::compute3DCentroid(*cloud_source, centroid_aligned);
    pcl::compute3DCentroid(*cloud_target, centroid_CAD);

    // printf("centroid_aligned: %f, %f, %f\n", centroid_aligned[0], centroid_aligned[1], centroid_aligned[2]);
    // printf("centroid_CAD: %f, %f, %f\n", centroid_CAD[0], centroid_CAD[1], centroid_CAD[2]);
    Eigen::Matrix4f T_trans_tmp = Eigen::Matrix4f::Identity();
    
    // a) to CAD centroid
    T_trans_tmp(0, 3) = -centroid_aligned[0];
    T_trans_tmp(1, 3) = -centroid_aligned[1];
    T_trans_tmp(2, 3) = -centroid_aligned[2];
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_trans_tmp);
    T_now = T_trans_tmp * T_now;

    // b) z translation
    pcl::PointXYZ minPt_aligned, maxPt_aligned;
    pcl::PointXYZ minPt_CAD, maxPt_CAD;
    pcl::getMinMax3D(*cloud_source, minPt_aligned, maxPt_aligned);
    pcl::getMinMax3D(*cloud_target, minPt_CAD, maxPt_CAD);

    T_trans_tmp(0, 3) = 0.0;
    T_trans_tmp(1, 3) = 0.0;
    T_trans_tmp(2, 3) = maxPt_CAD.z - maxPt_aligned.z;
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_trans_tmp);
    T_now = T_trans_tmp * T_now;

    //// c) centroid 2, 221213
    pcl::compute3DCentroid(*cloud_source, centroid_aligned);
    T_trans_tmp(0, 3) = centroid_CAD[0]-centroid_aligned[0];
    T_trans_tmp(1, 3) = centroid_CAD[1]-centroid_aligned[1];
    T_trans_tmp(2, 3) = 0.0;
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_trans_tmp);
    T_now = T_trans_tmp * T_now;

    // output
    T_out = T_now;

	// Visualization
	if (b_plot)
	{
		pcl::visualization::PCLVisualizer viewer("raw data before quick align");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text


		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*cloud_target, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud_target, 0, 0, 0);
		viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// // raw cloud is green
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(Registered, 10, 10, 255);
		// viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_align_color_h(cloud_source, 10, 10, 255);
		viewer.addPointCloud(cloud_source, cloud_align_color_h, "cloud_source", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			// viewer.spinOnce(100);
			// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            viewer.spin();
		}
		viewer.close();
	}
}

/** @brief Korean: Quick align 오차 감소를 위한 처리 함수, 
 * details 바닥면이 평면인 형상의 물체가 바닥에 놓여진 경우에 사용.
 */
void PCLPROCESSING::matchingMajorAxisTranslation(const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(cloud_CAD, *cloud_target); // Reference model (CAD model)
    *cloud_source = cloud_reg;	
    Eigen::Matrix4f T_now = Eigen::Matrix4f::Identity();

    //// initialize the source cloud
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_in);
    T_now = T_in;


    Eigen::Matrix4f T_trans_tmp = Eigen::Matrix4f::Identity();
    if(0) // min max translation, 기존 5mm씩 틀어지는 경우 발생
    {
        pcl::PointXYZ minPt_aligned, maxPt_aligned;
        pcl::PointXYZ minPt_CAD, maxPt_CAD;
        pcl::getMinMax3D(*cloud_source, minPt_aligned, maxPt_aligned);
        pcl::getMinMax3D(*cloud_target, minPt_CAD, maxPt_CAD);

        T_trans_tmp(0, 3) = maxPt_CAD.x - maxPt_aligned.x;
        T_trans_tmp(1, 3) = 0.0;
        T_trans_tmp(2, 3) = 0.0;
    }
    else // centroid 이용
    {
        //// 미리 CAD 근처로 translation
        Eigen::Vector4f centroid_aligned;
        Eigen::Vector4f centroid_CAD;
        pcl::compute3DCentroid(*cloud_source, centroid_aligned);
        // pcl::compute3DCentroid(*cloud_target, centroid_CAD);

        printf("centroid_aligned: %f, %f, %f\n", centroid_aligned[0], centroid_aligned[1], centroid_aligned[2]);
        // printf("centroid_CAD: %f, %f, %f\n", centroid_CAD[0], centroid_CAD[1], centroid_CAD[2]);
        
        pcl::compute3DCentroid(*cloud_source, centroid_aligned);
        T_trans_tmp(0, 3) = 0.0 - centroid_aligned[0];
        T_trans_tmp(1, 3) = 0.0;
        T_trans_tmp(2, 3) = 0.0;
    }
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_trans_tmp);
    T_now = T_trans_tmp * T_now;

    // output
    T_out = T_now;

	// Visualization
	if (b_plot)
	{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_raw(new pcl::PointCloud<pcl::PointXYZ>);
	    *cloud_source_raw = cloud_reg;
        pcl::transformPointCloud(*cloud_source_raw, *cloud_source_raw, T_in);
		pcl::visualization::PCLVisualizer viewer("raw data before quick align");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text


		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*cloud_target, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud_target, 0, 0, 0);
		viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// // raw cloud is green
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(Registered, 10, 10, 255);
		// viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(cloud_source, 255, 10, 10);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_align_color_h(cloud_source, 10, 10, 255);
		viewer.addPointCloud(cloud_source_raw, cloud_raw_color_h, "cloud_source_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source_v1");
		viewer.addPointCloud(cloud_source, cloud_align_color_h, "cloud_source_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source_v2");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		viewer.close();
	}
}

/** @brief Korean: Quick align 오차 감소를 위한 처리 함수, 
 * details 바닥면이 평면인 형상의 물체가 바닥에 놓여진 경우에 사용.
 */
void PCLPROCESSING::matchingMajorAxisTranslationMinMax(const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(cloud_CAD, *cloud_target); // Reference model (CAD model)
    *cloud_source = cloud_reg;	
    Eigen::Matrix4f T_now = Eigen::Matrix4f::Identity();

    //// initialize the source cloud
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_in);
    T_now = T_in;


    Eigen::Matrix4f T_trans_tmp = Eigen::Matrix4f::Identity();
    if(1) // min max translation, 기존 5mm씩 틀어지는 경우 발생
    {
        pcl::PointXYZ minPt_aligned, maxPt_aligned;
        pcl::PointXYZ minPt_CAD, maxPt_CAD;
        pcl::getMinMax3D(*cloud_source, minPt_aligned, maxPt_aligned);
        pcl::getMinMax3D(*cloud_target, minPt_CAD, maxPt_CAD);

        T_trans_tmp(0, 3) = maxPt_CAD.x - maxPt_aligned.x;
        T_trans_tmp(1, 3) = 0.0;
        T_trans_tmp(2, 3) = 0.0;
    }
    else // centroid 이용
    {
        //// 미리 CAD 근처로 translation
        Eigen::Vector4f centroid_aligned;
        Eigen::Vector4f centroid_CAD;
        pcl::compute3DCentroid(*cloud_source, centroid_aligned);
        // pcl::compute3DCentroid(*cloud_target, centroid_CAD);

        printf("centroid_aligned: %f, %f, %f\n", centroid_aligned[0], centroid_aligned[1], centroid_aligned[2]);
        // printf("centroid_CAD: %f, %f, %f\n", centroid_CAD[0], centroid_CAD[1], centroid_CAD[2]);
        
        pcl::compute3DCentroid(*cloud_source, centroid_aligned);
        T_trans_tmp(0, 3) = 0.0 - centroid_aligned[0];
        T_trans_tmp(1, 3) = 0.0;
        T_trans_tmp(2, 3) = 0.0;
    }
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_trans_tmp);
    T_now = T_trans_tmp * T_now;

    // output
    T_out = T_now;

	// Visualization
	if (b_plot)
	{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_raw(new pcl::PointCloud<pcl::PointXYZ>);
	    *cloud_source_raw = cloud_reg;
        pcl::transformPointCloud(*cloud_source_raw, *cloud_source_raw, T_in);
		pcl::visualization::PCLVisualizer viewer("raw data before quick align");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text


		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*cloud_target, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud_target, 0, 0, 0);
		viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// // raw cloud is green
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(Registered, 10, 10, 255);
		// viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(cloud_source, 255, 10, 10);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_align_color_h(cloud_source, 10, 10, 255);
		viewer.addPointCloud(cloud_source_raw, cloud_raw_color_h, "cloud_source_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source_v1");
		viewer.addPointCloud(cloud_source, cloud_align_color_h, "cloud_source_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source_v2");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			// viewer.spinOnce(100);
			// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
            viewer.spin();
		}
		viewer.close();
	}
}

/** @brief Korean: Quick align 오차 감소를 위한 처리 함수, 
 * details 바닥면이 평면인 형상의 물체가 바닥에 놓여진 경우에 사용.
 */
void PCLPROCESSING::matchingPlaneRotationUsingOBB(const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(cloud_CAD, *cloud_target); // Reference model (CAD model)
    *cloud_source = cloud_reg;	
    Eigen::Matrix4f T_now = Eigen::Matrix4f::Identity();

    //// initialize the source cloud
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_in);
    T_now = T_in;

    ////////////////////////////////////////////////////////////
    //// OBB (Object-oriented Bounding Box)
    Eigen::Matrix4f T_obb_source2projection;
    extractOBBTransform(*cloud_source, T_obb_source2projection, b_plot);
    // m_math.printHTM(T_obb_source2projection, "T_obb_source2projection");
    ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////
    Eigen::Matrix3f eigenVectorsPCA = T_obb_source2projection.block<3, 3>(0, 0);

    // 추출 순서는 경계박스의 한 변의 길이가 짧은 순
    Eigen::Vector3f vec_x_tmp = eigenVectorsPCA*Eigen::Vector3f(1, 0, 0); // 가장 변화가 작은 축
    Eigen::Vector3f vec_y_tmp = eigenVectorsPCA*Eigen::Vector3f(0, 1, 0); // 변화 크기가 중간인 축
    Eigen::Vector3f vec_z_tmp = eigenVectorsPCA*Eigen::Vector3f(0, 0, 1); // 가장 변화가 큰 축
    // printf("vec_x_tmp: %0.3f, %0.3f, %0.3f\n", vec_x_tmp[0], vec_x_tmp[1], vec_x_tmp[2]);
    // printf("vec_y_tmp: %0.3f, %0.3f, %0.3f\n", vec_y_tmp[0], vec_y_tmp[1], vec_y_tmp[2]);
    // printf("vec_z_tmp: %0.3f, %0.3f, %0.3f\n", vec_z_tmp[0], vec_z_tmp[1], vec_z_tmp[2]);

    //// TODO: 경우에 따라 방향이 반대로 추출되는 경우가 없을 지 확인 필요
    //// TODO: 경우에 따라 방향이 반대로 추출되는 경우가 없을 지 확인 필요
    //// TODO: 경우에 따라 방향이 반대로 추출되는 경우가 없을 지 확인 필요
    double rotation_angle = (180.0 / M_PI)*acos(vec_z_tmp.dot(Eigen::Vector3f(1, 0, 0))); // CAD의 장축 방향(+x)
    printf("rotation_angle: %0.3f\n", rotation_angle);
    printf("rotation axis: %0.3f, %0.3f, %0.3f\n", vec_x_tmp[0], vec_x_tmp[1], vec_x_tmp[2]);
    Eigen::Matrix3f R = m_math.genRotationAroundVector(vec_x_tmp, rotation_angle);
    Eigen::Matrix4f T_process = Eigen::Matrix4f::Identity();
    T_process.block<3, 3>(0, 0) = R;
    // m_math.printHTM(T_process, "T_process");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Matrix4f T_trans = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T_trans_inv = Eigen::Matrix4f::Identity();
    T_trans.block<3, 1>(0, 3) = -T_obb_source2projection.block<3, 1>(0, 3);
    T_trans_inv.block<3, 1>(0, 3) = T_obb_source2projection.block<3, 1>(0, 3);

    ////////////////////////////////////////////////////////////
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_trans);
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_process);
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_trans_inv);
    T_now = T_trans_inv * T_process * T_trans * T_now;

    // output
    T_out = T_now;

	// Visualization
	if (b_plot)
	{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_raw(new pcl::PointCloud<pcl::PointXYZ>);
	    *cloud_source_raw = cloud_reg;
        pcl::transformPointCloud(*cloud_source_raw, *cloud_source_raw, T_in);
		pcl::visualization::PCLVisualizer viewer("raw data before quick align");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text


		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*cloud_target, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud_target, 0, 0, 0);
		viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// // raw cloud is green
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(Registered, 10, 10, 255);
		// viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(cloud_source, 255, 10, 10);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_align_color_h(cloud_source, 10, 10, 255);
		viewer.addPointCloud(cloud_source_raw, cloud_raw_color_h, "cloud_source_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source_v1");
		viewer.addPointCloud(cloud_source, cloud_align_color_h, "cloud_source_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source_v2");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		viewer.close();
	}
}

/** @brief Korean: Quick align을 이용하여 모델 매칭을 수행한다.
 * @param[in] cloud_reg : 정합 모델(정합된 점군 데이터)
 * @param[in] cloud_CAD : 기준 모델(CAD 점군 데이터)
 * @param[out] T_out : Quick align을 통해 추출된 두 점군 데이터 사이의 변환행렬, T_O2B
 * @param[in] b_plot : false: viewer off, true: viewer on
 */
void PCLPROCESSING::modelMatchingReg2CADUsingQuickAlignBinPicking(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot)
{
	printf("\n--------------  Quick align - only XYZ (Bin Picking) start!  -----------------\n");
	pcl::PointCloud<pcl::PointXYZ>::Ptr Reference(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Registered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(cloud_CAD, *Reference); // Reference model (CAD model)

	// 1) rough alignment using quick align
	// pcl::copyPointCloud(cloud_CAD, *cloud_target); // Reference model (CAD model)
	*cloud_source = cloud_reg;					   // Registration model
    Eigen::Matrix4f T_now = Eigen::Matrix4f::Identity();

    //// initialize the source cloud
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_in);
    T_now = T_in;

    // a) quick align과 ICP 수행 전에, 임의로 +dz로 margin 추가 --> 내부 면에 붙는 경우를 방지
    Eigen::Matrix4f T_trans_z = Eigen::Matrix4f::Identity();
    T_trans_z(0, 3) = 0.0;
    T_trans_z(1, 3) = 0.0;
    T_trans_z(2, 3) = align_param.quick_align_z_threshold;
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_trans_z);
    T_now = T_trans_z * T_now;

	// Visualization
	if (b_plot)
	{
		pcl::visualization::PCLVisualizer viewer("quick_align_z_threshold");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text

		
		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*Reference, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(Reference, 0, 0, 0);
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// raw cloud is green
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(Registered, 10, 10, 255);
		viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_align_color_h(cloud_source, 10, 10, 255);
		viewer.addPointCloud(cloud_source, cloud_align_color_h, "cloud_source", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		viewer.close();
	}

    //// Quick align
	// Voxel filtering -  too slow
	// 221102, measured cloud - cloud_input을 pixel 기준으로 downsampling / TODO: measured cloud를 filtering 하는게 맞을지? 정합 성능 측면에서 확인
	// 221102, CAD cloud - 입력 초기에만 1회 voxel filtering을 수행한 cloud를 저장하여 사용


    pcl::console::TicToc time;
	if(align_param.do_quick_align) {


		pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud_sc(new pcl::PointCloud<pcl::PointXYZ>);
		// pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);

		// m_math.cloudScaling(*cloud_target, 0);		   // mm to m
		m_math.cloudScaling(*cloud_source, 0);		   // mm to m

		*tempCloud_sc = *cloud_source;
		// *tempCloud = *cloud_target;

        time.tic();

		FeatureCloud source_ft_cloud;
		source_ft_cloud.setInputCloud(tempCloud_sc);

		std::vector<FeatureCloud> object_templates;
		object_templates.resize(0);
		object_templates.push_back(source_ft_cloud);

        // std::cout << "Quick align(without normal)" << " - process 1-1 time: " << time.toc() << " ms" << std::endl;

        time.tic();

        // FeatureCloud target_ft_cloud;
        // target_ft_cloud.setInputCloud(tempCloud);

		TemplateAlignment template_align;
		for (size_t i = 0; i < object_templates.size(); ++i)
		{
			template_align.addTemplateCloud(object_templates[i]);
		}

        // std::cout << "Quick align(without normal)" << " - process 1-2 time: " << time.toc() << " ms" << std::endl;


        time.tic();
		template_align.setTargetCloud(align_param.target_ft_cloud); // target
        // Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
        template_align.setParameters(0.001*align_param.quick_align_min_sample_distance, 0.001*align_param.quick_align_max_correspondence_distance, align_param.quick_align_iteration, align_param.quick_align_sample_num, align_param.quick_align_corr_rnd_k); // min_sample_distance, max_correspondence_distance, iteration, sample_num

		// Find the best template alignment
		TemplateAlignment::Result best_alignment;
		int best_index = template_align.findBestAlignment(best_alignment);
		const FeatureCloud &best_template = object_templates[best_index];
		// Print the alignment fitness score (values less than 0.00002 are good)
		// printf("Best fitness score: %f\n", best_alignment.fitness_score);
        std::cout << "Quick align(without normal)" << " - process 1-3 time: " << time.toc() << " ms" << std::endl;

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
		Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);
		// printf("\n");
		// printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		// printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		// printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		// printf("\n");
		// printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

		// Save the aligned template for visualization
		pcl::transformPointCloud(*cloud_source, *cloud_aligned, best_alignment.final_transformation);
		m_math.cloudScaling(*cloud_aligned, 1); // m to mm
		Eigen::Matrix4f T_quick = best_alignment.final_transformation;
		T_quick(0, 3) = 1000.0 * T_quick(0, 3);
		T_quick(1, 3) = 1000.0 * T_quick(1, 3);
		T_quick(2, 3) = 1000.0 * T_quick(2, 3);

	    T_now = T_quick*T_now;


// // JSON 파일 열기
// std::ifstream file("matrices.json");
// json j;
// file >> j;

// // 특정 키(T1, T2)를 Eigen::Matrix4f로 변환
// Eigen::Matrix4f T1 = jsonToMatrix4f(j["T1"]);
// Eigen::Matrix4f T2 = jsonToMatrix4f(j["T2"]);

// std::cout << "T1:\n" << T1 << "\n\n";
// std::cout << "T2:\n" << T2 << "\n\n";

        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("***************** [%s] Quick Aling Results *****************", __func__);
        ROS_LOG_WARN("T_now = T_quick*T_now;");
        ROS_LOG_WARN("*******************************************************");
        printMatrixAsJson(T_now, "T_now");

        ROS_LOG_WARN("T_quick;");
        ROS_LOG_WARN("*******************************************************");
        
        printMatrixAsJson(T_quick, "T_quick");
        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("*******************************************************");

	}
	else {
		*cloud_aligned = *cloud_source;
	}
	// printf("----------- T_quick ------------\n");
	// print4Matrix(T_quick.cast<double>());

	// Visualization
	if (b_plot)
	{
		pcl::visualization::PCLVisualizer viewer("Rough alignment using quick align");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text

		
		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*Reference, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(Reference, 0, 0, 0);
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// raw cloud is green
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(Registered, 10, 10, 255);
		viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_align_color_h(cloud_aligned, 10, 10, 255);
		viewer.addPointCloud(cloud_aligned, cloud_align_color_h, "cloud_aligned", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_aligned");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		viewer.close();
	}

    //// TODO: 코드 맨 아랫줄에 주석 처리한 Quick align 실패 방지 처리 1, 2 참고

	////////////////////////////////////////////////////////////////////////////////
	// 2) precise alignment using ICP
	Eigen::Matrix4f T_icp;
	// modelMatchingReg2CADUsingICP2(*cloud_aligned, *Reference, T_icp, b_plot);
	// modelMatchingReg2CADUsingICP3(1.0, *cloud_aligned, *Reference, T_icp, b_plot);
	matchingReg2CADUsingICP4(align_param, *cloud_aligned, *Reference, T_icp, b_plot);
    T_now = T_icp*T_now;
	// printf("----------- T_icp ------------\n");
	// print4Matrix(T_icp.cast<double>());

    //// output
	T_out = T_now; // metric: mm
}




/** @brief Korean: Quick align을 이용하여 모델 매칭을 수행한다.
 * @param[in] cloud_reg : 정합 모델(정합된 점군 데이터)
 * @param[in] cloud_CAD : 기준 모델(CAD 점군 데이터)
 * @param[out] T_out : Quick align을 통해 추출된 두 점군 데이터 사이의 변환행렬, T_O2B
 * @param[in] b_plot : false: viewer off, true: viewer on
 */
void PCLPROCESSING::modelMatchingReg2CADUsingQuickAlignBinPickingUsingJson(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, const std::string &quick_align_json_path, Eigen::Matrix4f &T_quick_out, Eigen::Matrix4f &T_out, bool b_plot)
{
	printf("\n[%s]--------------  Quick align - only XYZ (Bin Picking) start!  -----------------\n", __func__);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Reference(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Registered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(cloud_CAD, *Reference); // Reference model (CAD model)

	// 1) rough alignment using quick align
	// pcl::copyPointCloud(cloud_CAD, *cloud_target); // Reference model (CAD model)
	*cloud_source = cloud_reg;					   // Registration model
    Eigen::Matrix4f T_now = Eigen::Matrix4f::Identity();

    //// initialize the source cloud
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_in);
    T_now = T_in;

    // a) quick align과 ICP 수행 전에, 임의로 +dz로 margin 추가 --> 내부 면에 붙는 경우를 방지
    Eigen::Matrix4f T_trans_z = Eigen::Matrix4f::Identity();
    T_trans_z(0, 3) = 0.0;
    T_trans_z(1, 3) = 0.0;
    T_trans_z(2, 3) = align_param.quick_align_z_threshold;
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_trans_z);
    T_now = T_trans_z * T_now;

	// Visualization
	if (b_plot)
	{
		pcl::visualization::PCLVisualizer viewer("quick_align_z_threshold");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text

		
		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*Reference, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(Reference, 0, 0, 0);
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// raw cloud is green
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(Registered, 10, 10, 255);
		viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_align_color_h(cloud_source, 10, 10, 255);
		viewer.addPointCloud(cloud_source, cloud_align_color_h, "cloud_source", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		viewer.close();
	}

    //// Quick align
	// Voxel filtering -  too slow
	// 221102, measured cloud - cloud_input을 pixel 기준으로 downsampling / TODO: measured cloud를 filtering 하는게 맞을지? 정합 성능 측면에서 확인
	// 221102, CAD cloud - 입력 초기에만 1회 voxel filtering을 수행한 cloud를 저장하여 사용


    pcl::console::TicToc time;
    Eigen::Matrix4f T_quick_for_json = Eigen::Matrix4f::Identity();


    auto run_quick_align_proc = [&]() {

		pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud_sc(new pcl::PointCloud<pcl::PointXYZ>);
		// pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);

		// m_math.cloudScaling(*cloud_target, 0);		   // mm to m
		m_math.cloudScaling(*cloud_source, 0);		   // mm to m

		*tempCloud_sc = *cloud_source;
		// *tempCloud = *cloud_target;

        time.tic();

		FeatureCloud source_ft_cloud;
		source_ft_cloud.setInputCloud(tempCloud_sc);

		std::vector<FeatureCloud> object_templates;
		object_templates.resize(0);
		object_templates.push_back(source_ft_cloud);

        // std::cout << "Quick align(without normal)" << " - process 1-1 time: " << time.toc() << " ms" << std::endl;

        time.tic();

        // FeatureCloud target_ft_cloud;
        // target_ft_cloud.setInputCloud(tempCloud);

		TemplateAlignment template_align;
		for (size_t i = 0; i < object_templates.size(); ++i)
		{
			template_align.addTemplateCloud(object_templates[i]);
		}

        // std::cout << "Quick align(without normal)" << " - process 1-2 time: " << time.toc() << " ms" << std::endl;


        time.tic();
		template_align.setTargetCloud(align_param.target_ft_cloud); // target
        // Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
        template_align.setParameters(0.001*align_param.quick_align_min_sample_distance, 0.001*align_param.quick_align_max_correspondence_distance, align_param.quick_align_iteration, align_param.quick_align_sample_num, align_param.quick_align_corr_rnd_k); // min_sample_distance, max_correspondence_distance, iteration, sample_num

		// Find the best template alignment
		TemplateAlignment::Result best_alignment;
		int best_index = template_align.findBestAlignment(best_alignment);
		const FeatureCloud &best_template = object_templates[best_index];
		// Print the alignment fitness score (values less than 0.00002 are good)
		// printf("Best fitness score: %f\n", best_alignment.fitness_score);
        std::cout << "Quick align(without normal)" << " - process 1-3 time: " << time.toc() << " ms" << std::endl;

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
		Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);
		// printf("\n");
		// printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		// printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		// printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		// printf("\n");
		// printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

		// Save the aligned template for visualization
		pcl::transformPointCloud(*cloud_source, *cloud_aligned, best_alignment.final_transformation);
		m_math.cloudScaling(*cloud_aligned, 1); // m to mm
		Eigen::Matrix4f T_quick = best_alignment.final_transformation;
		T_quick(0, 3) = 1000.0 * T_quick(0, 3);
		T_quick(1, 3) = 1000.0 * T_quick(1, 3);
		T_quick(2, 3) = 1000.0 * T_quick(2, 3);

	    T_now = T_quick*T_now;

        //// 251006, Here
        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("***************** [%s] Quick Aling Results *****************", __func__);
        // ROS_LOG_WARN("T_now = T_quick*T_now;");
        ROS_LOG_WARN("*******************************************************");
        // printMatrixAsJson(T_now, "T_now");
        ROS_LOG_WARN("T_quick;");
        ROS_LOG_WARN("*******************************************************");
        printMatrixAsJson(T_quick, "T_quick");
        //// JSON 저장용, 251006
        T_quick_for_json = T_quick;

        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("*******************************************************");

    };

    // decide whether to run quick-align
    bool json_exists = fs::exists(quick_align_json_path);
    std::vector<Eigen::Matrix4f> s_Ts; int s_count = 0;
    if (!align_param.do_quick_align) { // usin Quick Align Table (JSON)
        // try to load T* only when quick-align is off
        if (json_exists) s_Ts = loadTransformsFromJsonWithCount(quick_align_json_path, &s_count);
    }

    //// 아래와 같은 경우가 발생하면 기존대로 quick_align을 수행
    const bool run_quick_align =
    align_param.do_quick_align       // explicit request
    || !json_exists                  // file missing
    || s_Ts.empty();                 // file exists but no usable T’s


    if (run_quick_align) {
        run_quick_align_proc(); // 기존 Quick align 수행
	}
	else {

        if(0) {
            *cloud_aligned = *cloud_source;
        } else {
            // quick-align off, and we have json Ts → sample one
            pcl::copyPointCloud(*cloud_source, *cloud_aligned); // [mm], cloud_source already has T_in
        
            static thread_local std::mt19937 rng{std::random_device{}()};
            std::uniform_int_distribution<size_t> dist(0, s_Ts.size() - 1);
            const size_t idx = dist(rng);
        
            Eigen::Matrix4f T_quick = s_Ts[idx]; // [mm]
            //// 변환 적용. T_now는 함수 초기에 이미 적용됨.
            pcl::transformPointCloud(*cloud_aligned, *cloud_aligned, T_quick);
            /// update
            T_now = T_quick * T_now;
        
            ROS_LOG_WARN("[QuickAlign OFF] Sampled T_quick from JSON (idx=%zu, total=%zu, count=%d).",
                        idx, s_Ts.size(), s_count);
            printMatrixAsJson(T_quick, "T_quick(sampled)");
            T_quick_for_json = T_quick;
        }
	}
	// printf("----------- T_quick ------------\n");
	// print4Matrix(T_quick.cast<double>());

	// Visualization
	if (b_plot)
	{
		pcl::visualization::PCLVisualizer viewer("Rough alignment using quick align");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text

		
		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*Reference, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(Reference, 0, 0, 0);
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// raw cloud is green
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(Registered, 10, 10, 255);
		viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_align_color_h(cloud_aligned, 10, 10, 255);
		viewer.addPointCloud(cloud_aligned, cloud_align_color_h, "cloud_aligned", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_aligned");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		viewer.close();
	}

    //// TODO: 코드 맨 아랫줄에 주석 처리한 Quick align 실패 방지 처리 1, 2 참고

	////////////////////////////////////////////////////////////////////////////////
	// 2) precise alignment using ICP
	Eigen::Matrix4f T_icp;
	// modelMatchingReg2CADUsingICP2(*cloud_aligned, *Reference, T_icp, b_plot);
	// modelMatchingReg2CADUsingICP3(1.0, *cloud_aligned, *Reference, T_icp, b_plot);
	matchingReg2CADUsingICP4(align_param, *cloud_aligned, *Reference, T_icp, b_plot);
    T_now = T_icp*T_now;
	// printf("----------- T_icp ------------\n");
	// print4Matrix(T_icp.cast<double>());

    //// output
	T_out = T_now; // metric: mm
    T_quick_out = T_quick_for_json; // 251006
}


/** @brief Korean: [GICP 적용] Quick align을 이용하여 모델 매칭을 수행한다.
 * @param[in] cloud_reg : 정합 모델(정합된 점군 데이터)
 * @param[in] cloud_CAD : 기준 모델(CAD 점군 데이터)
 * @param[out] T_out : Quick align을 통해 추출된 두 점군 데이터 사이의 변환행렬, T_O2B
 * @param[in] b_plot : false: viewer off, true: viewer on
 */
void PCLPROCESSING::modelMatchingReg2CADUsingQuickAlignBinPickingWithGICP(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot)
{
	printf("\n-------------- [GICP] Quick align - only XYZ (Bin Picking) start!  -----------------\n");
	pcl::PointCloud<pcl::PointXYZ>::Ptr Reference(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Registered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(cloud_CAD, *Reference); // Reference model (CAD model)

	// 1) rough alignment using quick align
	// pcl::copyPointCloud(cloud_CAD, *cloud_target); // Reference model (CAD model)
	*cloud_source = cloud_reg;					   // Registration model
    Eigen::Matrix4f T_now = Eigen::Matrix4f::Identity();

    //// initialize the source cloud
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_in);
    T_now = T_in;

    // a) quick align과 ICP 수행 전에, 임의로 +dz로 margin 추가 --> 내부 면에 붙는 경우를 방지
    Eigen::Matrix4f T_trans_z = Eigen::Matrix4f::Identity();
    T_trans_z(0, 3) = 0.0;
    T_trans_z(1, 3) = 0.0;
    T_trans_z(2, 3) = align_param.quick_align_z_threshold;
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_trans_z);
    T_now = T_trans_z * T_now;

	// Visualization
	if (b_plot)
	{
		pcl::visualization::PCLVisualizer viewer("quick_align_z_threshold");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text

		
		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*Reference, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(Reference, 0, 0, 0);
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// raw cloud is green
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(Registered, 10, 10, 255);
		viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_align_color_h(cloud_source, 10, 10, 255);
		viewer.addPointCloud(cloud_source, cloud_align_color_h, "cloud_source", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		viewer.close();
	}

    //// Quick align
	// Voxel filtering -  too slow
	// 221102, measured cloud - cloud_input을 pixel 기준으로 downsampling / TODO: measured cloud를 filtering 하는게 맞을지? 정합 성능 측면에서 확인
	// 221102, CAD cloud - 입력 초기에만 1회 voxel filtering을 수행한 cloud를 저장하여 사용


    pcl::console::TicToc time;
	if(align_param.do_quick_align) {


		pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud_sc(new pcl::PointCloud<pcl::PointXYZ>);
		// pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);

		// m_math.cloudScaling(*cloud_target, 0);		   // mm to m
		m_math.cloudScaling(*cloud_source, 0);		   // mm to m

		*tempCloud_sc = *cloud_source;
		// *tempCloud = *cloud_target;

        time.tic();

		FeatureCloud source_ft_cloud;
		source_ft_cloud.setInputCloud(tempCloud_sc);

		std::vector<FeatureCloud> object_templates;
		object_templates.resize(0);
		object_templates.push_back(source_ft_cloud);

        // std::cout << "Quick align(without normal)" << " - process 1-1 time: " << time.toc() << " ms" << std::endl;

        time.tic();

        // FeatureCloud target_ft_cloud;
        // target_ft_cloud.setInputCloud(tempCloud);

		TemplateAlignment template_align;
		for (size_t i = 0; i < object_templates.size(); ++i)
		{
			template_align.addTemplateCloud(object_templates[i]);
		}

        // std::cout << "Quick align(without normal)" << " - process 1-2 time: " << time.toc() << " ms" << std::endl;


        time.tic();
		template_align.setTargetCloud(align_param.target_ft_cloud); // target
        // Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
        template_align.setParameters(0.001*align_param.quick_align_min_sample_distance, 0.001*align_param.quick_align_max_correspondence_distance, align_param.quick_align_iteration, align_param.quick_align_sample_num, align_param.quick_align_corr_rnd_k); // min_sample_distance, max_correspondence_distance, iteration, sample_num

		// Find the best template alignment
		TemplateAlignment::Result best_alignment;
		int best_index = template_align.findBestAlignment(best_alignment);
		const FeatureCloud &best_template = object_templates[best_index];
		// Print the alignment fitness score (values less than 0.00002 are good)
		// printf("Best fitness score: %f\n", best_alignment.fitness_score);
        std::cout << "Quick align(without normal)" << " - process 1-3 time: " << time.toc() << " ms" << std::endl;

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
		Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);
		// printf("\n");
		// printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		// printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		// printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		// printf("\n");
		// printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

		// Save the aligned template for visualization
		pcl::transformPointCloud(*cloud_source, *cloud_aligned, best_alignment.final_transformation);
		m_math.cloudScaling(*cloud_aligned, 1); // m to mm
		Eigen::Matrix4f T_quick = best_alignment.final_transformation;
		T_quick(0, 3) = 1000.0 * T_quick(0, 3);
		T_quick(1, 3) = 1000.0 * T_quick(1, 3);
		T_quick(2, 3) = 1000.0 * T_quick(2, 3);

	    T_now = T_quick*T_now;

        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("***************** [%s] Quick Aling Results *****************", __func__);
        ROS_LOG_WARN("T_now = T_quick*T_now;");
        ROS_LOG_WARN("*******************************************************");
        printMatrixAsJson(T_now, "T_now");

        ROS_LOG_WARN("T_quick;");
        ROS_LOG_WARN("*******************************************************");
        
        printMatrixAsJson(T_quick, "T_quick");
        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("*******************************************************");

	}
	else {
		*cloud_aligned = *cloud_source;
	}
	// printf("----------- T_quick ------------\n");
	// print4Matrix(T_quick.cast<double>());

	// Visualization
	if (b_plot)
	{
		pcl::visualization::PCLVisualizer viewer("Rough alignment using quick align");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text

		
		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*Reference, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(Reference, 0, 0, 0);
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// raw cloud is green
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(Registered, 10, 10, 255);
		viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_align_color_h(cloud_aligned, 10, 10, 255);
		viewer.addPointCloud(cloud_aligned, cloud_align_color_h, "cloud_aligned", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_aligned");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		viewer.close();
	}

    //// TODO: 코드 맨 아랫줄에 주석 처리한 Quick align 실패 방지 처리 1, 2 참고

	////////////////////////////////////////////////////////////////////////////////
	// 2) precise alignment using ICP
	Eigen::Matrix4f T_icp;

    //// 기존
	// matchingReg2CADUsingICP4(align_param, *cloud_aligned, *Reference, T_icp, b_plot);
    //// 변경
	matchingReg2CADUsingGeneralizedICP1(align_param, *cloud_aligned, *Reference, T_icp, b_plot);



    T_now = T_icp*T_now;
	// printf("----------- T_icp ------------\n");
	// print4Matrix(T_icp.cast<double>());

    //// output
	T_out = T_now; // metric: mm
}

/** @brief Korean: Quick align을 이용하여 모델 매칭을 수행한다.
 * @param[in] cloud_reg : 정합 모델(정합된 점군 데이터)
 * @param[in] cloud_CAD : 기준 모델(CAD 점군 데이터)
 * @param[out] T_out : Quick align을 통해 추출된 두 점군 데이터 사이의 변환행렬, T_O2B
 * @param[in] b_plot : false: viewer off, true: viewer on
 */
void PCLPROCESSING::modelMatchingReg2CADUsingQuickAlignBinPickingWithNormalVer1(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_reg, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot)
{
    //// 내부 함수에서 법선 추정을 하지 않도록 처리
    //// 1) normal을 할당하여 얻은 fpfh feature를 사용하고,
    //// 2) ICP에서 할당된 normal를 사용함.
	printf("\n[%s]--------------  Quick align with normal assigned (Bin Picking) start!  -----------------\n", __func__);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Reference(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Registered(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::copyPointCloud(cloud_CAD, *Reference); // Reference model (CAD model)

	// 1) rough alignment using quick align
	// pcl::copyPointCloud(cloud_CAD, *cloud_target); // Reference model (CAD model)
	*cloud_source = cloud_reg;					   // Registration model
    Eigen::Matrix4f T_now = Eigen::Matrix4f::Identity();

    //// initialize the source cloud
    pcl::transformPointCloudWithNormals(*cloud_source, *cloud_source, T_in);
    T_now = T_in;

    // a) quick align과 ICP 수행 전에, 임의로 +dz로 margin 추가 --> 내부 면에 붙는 경우를 방지
    Eigen::Matrix4f T_trans_z = Eigen::Matrix4f::Identity();
    T_trans_z(0, 3) = 0.0;
    T_trans_z(1, 3) = 0.0;
    T_trans_z(2, 3) = align_param.quick_align_z_threshold;
    pcl::transformPointCloudWithNormals(*cloud_source, *cloud_source, T_trans_z);
    T_now = T_trans_z * T_now;

	// Visualization
	if (b_plot)
	{
		pcl::visualization::PCLVisualizer viewer("quick_align_z_threshold");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text

		
		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*Reference, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_in_color_h(Reference, 0, 0, 0);
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// raw cloud is green
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_raw_color_h(Registered, 10, 10, 255);
		viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_align_color_h(cloud_source, 10, 10, 255);
		viewer.addPointCloud(cloud_source, cloud_align_color_h, "cloud_source", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		viewer.close();
	}

    //// Quick align
	// Voxel filtering -  too slow
	// 221102, measured cloud - cloud_input을 pixel 기준으로 downsampling / TODO: measured cloud를 filtering 하는게 맞을지? 정합 성능 측면에서 확인
	// 221102, CAD cloud - 입력 초기에만 1회 voxel filtering을 수행한 cloud를 저장하여 사용

    pcl::console::TicToc time;
	if(align_param.do_quick_align) {
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tempCloud_sc(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		// pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

		// m_math.cloudScaling(*cloud_target, 0);		   // mm to m
		m_math.cloudScaling(*cloud_source, 0);		   // mm to m

		*tempCloud_sc = *cloud_source;
		// *tempCloud = *cloud_target;

        time.tic();

		FeatureCloudNormal source_ft_cloud;
		source_ft_cloud.setInputCloud(tempCloud_sc);

		std::vector<FeatureCloudNormal> object_templates;
		object_templates.resize(0);
		object_templates.push_back(source_ft_cloud);

        std::cout << "Quick align(without normal)" << " - process 1-1 time: " << time.toc() << " ms" << std::endl;

        time.tic();

		// FeatureCloudNormal target_ft_cloud;
		// target_ft_cloud.setInputCloud(tempCloud);

		TemplateAlignmentNormal template_align;
		for (size_t i = 0; i < object_templates.size(); ++i)
		{
			template_align.addTemplateCloud(object_templates[i]);
		}
        std::cout << "Quick align(without normal)" << " - process 1-2 time: " << time.toc() << " ms" << std::endl;

        time.tic();
		template_align.setTargetCloud(align_param.target_ft_cloud_with_normal); // target
        template_align.setParameters(0.001*align_param.quick_align_min_sample_distance, 0.001*align_param.quick_align_max_correspondence_distance, align_param.quick_align_iteration, align_param.quick_align_sample_num, align_param.quick_align_corr_rnd_k); // min_sample_distance, max_correspondence_distance, iteration, sample_num

		// Find the best template alignment
		TemplateAlignmentNormal::Result best_alignment;
		int best_index = template_align.findBestAlignment(best_alignment);
		const FeatureCloudNormal &best_template = object_templates[best_index];
		// Print the alignment fitness score (values less than 0.00002 are good)
		// printf("Best fitness score: %f\n", best_alignment.fitness_score);
        std::cout << "Quick align(without normal)" << " - process 1-3 time: " << time.toc() << " ms" << std::endl;

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
		Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);
		// printf("\n");
		// printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		// printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		// printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		// printf("\n");
		// printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

		// Save the aligned template for visualization
		// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::copyPointCloud(*cloud_source, *cloud_source_xyz);
		pcl::transformPointCloudWithNormals(*cloud_source, *cloud_aligned, best_alignment.final_transformation);
		m_math.cloudScaling(*cloud_aligned, 1); // m to mm
		Eigen::Matrix4f T_quick = best_alignment.final_transformation;
		T_quick(0, 3) = 1000.0 * T_quick(0, 3);
		T_quick(1, 3) = 1000.0 * T_quick(1, 3);
		T_quick(2, 3) = 1000.0 * T_quick(2, 3);

	    T_now = T_quick*T_now;

        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("***************** [%s] Quick Aling Results *****************", __func__);
        ROS_LOG_WARN("T_now = T_quick*T_now;");
        ROS_LOG_WARN("*******************************************************");
        printMatrixAsJson(T_now, "T_now");

        ROS_LOG_WARN("T_quick;");
        ROS_LOG_WARN("*******************************************************");
        
        printMatrixAsJson(T_quick, "T_quick");
        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("*******************************************************");
	}
	else {
		// *cloud_aligned = *cloud_source;
        pcl::copyPointCloud(*cloud_source, *cloud_aligned);
	}

	// printf("----------- T_quick ------------\n");
	// print4Matrix(T_quick.cast<double>());

	// Visualization
	if (b_plot)
	{
		pcl::visualization::PCLVisualizer viewer("Rough alignment using quick align");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text

		
		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*Reference, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_in_color_h(Reference, 0, 0, 0);
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// raw cloud is green
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_raw_color_h(Registered, 10, 10, 255);
		viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_align_color_h(cloud_aligned, 10, 10, 255);
		viewer.addPointCloud(cloud_aligned, cloud_align_color_h, "cloud_aligned", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_aligned");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		viewer.close();
	}

    //// TODO: 코드 맨 아랫줄에 주석 처리한 Quick align 실패 방지 처리 1, 2 참고

	////////////////////////////////////////////////////////////////////////////////
	// 2) precise alignment using ICP
	Eigen::Matrix4f T_icp;
    matchingReg2CADUsingICP4NoEstNormal(align_param, *cloud_aligned, *Reference, T_icp, b_plot);
    
    T_now = T_icp*T_now;
    //// output
	T_out = T_now; // metric: mm
}


/** @brief Korean: Quick align을 이용하여 모델 매칭을 수행한다.
 * @param[in] cloud_reg : 정합 모델(정합된 점군 데이터)
 * @param[in] cloud_CAD : 기준 모델(CAD 점군 데이터)
 * @param[out] T_out : Quick align을 통해 추출된 두 점군 데이터 사이의 변환행렬, T_O2B
 * @param[in] b_plot : false: viewer off, true: viewer on
 */
void PCLPROCESSING::modelMatchingReg2CADUsingQuickAlignBinPickingWithNormalVer2(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_reg, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD, const Eigen::Matrix4f &T_in, const std::string &quick_align_json_path, Eigen::Matrix4f &T_quick_out, Eigen::Matrix4f &T_out, bool b_plot)
{
    //// 내부 함수에서 법선 추정을 하지 않도록 처리
    //// 1) normal을 할당하여 얻은 fpfh feature를 사용하고,
    //// 2) ICP에서 할당된 normal를 사용함.
	printf("\n[%s]--------------  Quick align with normal assigned (Bin Picking) start!  -----------------\n", __func__);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Reference(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Registered(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::copyPointCloud(cloud_CAD, *Reference); // Reference model (CAD model)

	// 1) rough alignment using quick align
	// pcl::copyPointCloud(cloud_CAD, *cloud_target); // Reference model (CAD model)
	*cloud_source = cloud_reg;					   // Registration model
    Eigen::Matrix4f T_now = Eigen::Matrix4f::Identity();

    //// initialize the source cloud
    pcl::transformPointCloudWithNormals(*cloud_source, *cloud_source, T_in);
    T_now = T_in;

    // // a) quick align과 ICP 수행 전에, 임의로 +dz로 margin 추가 --> 내부 면에 붙는 경우를 방지
    // Eigen::Matrix4f T_trans_z = Eigen::Matrix4f::Identity();
    // T_trans_z(0, 3) = 0.0;
    // T_trans_z(1, 3) = 0.0;
    // T_trans_z(2, 3) = align_param.quick_align_z_threshold;
    // pcl::transformPointCloudWithNormals(*cloud_source, *cloud_source, T_trans_z);
    // T_now = T_trans_z * T_now;

	// Visualization
	if (b_plot)
	{
		pcl::visualization::PCLVisualizer viewer("quick_align_z_threshold");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text

		
		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*Reference, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_in_color_h(Reference, 0, 0, 0);
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// raw cloud is green
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_raw_color_h(Registered, 10, 10, 255);
		viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_align_color_h(cloud_source, 10, 10, 255);
		viewer.addPointCloud(cloud_source, cloud_align_color_h, "cloud_source", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			// viewer.spinOnce(100);
			// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			viewer.spin();
		}
		viewer.close();
	}

    //// Quick align
	// Voxel filtering -  too slow
	// 221102, measured cloud - cloud_input을 pixel 기준으로 downsampling / TODO: measured cloud를 filtering 하는게 맞을지? 정합 성능 측면에서 확인
	// 221102, CAD cloud - 입력 초기에만 1회 voxel filtering을 수행한 cloud를 저장하여 사용

    pcl::console::TicToc time;
    Eigen::Matrix4f T_quick_for_json = Eigen::Matrix4f::Identity();
    

    auto run_quick_align_proc = [&]() {
		m_math.cloudScaling(*cloud_source, 0);		   // mm to m

        time.tic();

        //// 231129 추가
		// FeatureCloudNormal source_ft_cloud;
        // if(!align_param.is_measured_cloud_feature_assigned) {
		//     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tempCloud_sc(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		//     *tempCloud_sc = *cloud_source;
		//     source_ft_cloud.setInputCloud(tempCloud_sc);
        //     align_param.measured_ft_cloud_with_normal = source_ft_cloud;
        // } else {
        //     source_ft_cloud = align_param.measured_ft_cloud_with_normal;
        // }
		std::vector<FeatureCloudNormal> object_templates;
		object_templates.resize(0);
		object_templates.push_back(align_param.measured_ft_cloud_with_normal);

        std::cout << "Quick align(with normal)" << " - process 1-1 time: " << time.toc() << " ms" << std::endl;

        time.tic();

		// FeatureCloudNormal target_ft_cloud;
		// target_ft_cloud.setInputCloud(tempCloud);

		TemplateAlignmentNormal template_align;
		for (size_t i = 0; i < object_templates.size(); ++i)
		{
			template_align.addTemplateCloud(object_templates[i]);
		}
        std::cout << "Quick align(with normal)" << " - process 1-2 time: " << time.toc() << " ms" << std::endl;

        time.tic();
		template_align.setTargetCloud(align_param.target_ft_cloud_with_normal); // target
        template_align.setParameters(0.001*align_param.quick_align_min_sample_distance, 0.001*align_param.quick_align_max_correspondence_distance, align_param.quick_align_iteration, align_param.quick_align_sample_num, align_param.quick_align_corr_rnd_k); // min_sample_distance, max_correspondence_distance, iteration, sample_num

		// Find the best template alignment
		TemplateAlignmentNormal::Result best_alignment;
		int best_index = template_align.findBestAlignment(best_alignment);
		const FeatureCloudNormal &best_template = object_templates[best_index];
		// Print the alignment fitness score (values less than 0.00002 are good)
		printf("[%s] Best fitness score: %f\n", __func__, best_alignment.fitness_score);
        std::cout << "Quick align(with normal)" << " - process 1-3 time: " << time.toc() << " ms" << std::endl;

        // std::cout << "NumberOfSamples: " << template_align.getNumberOfSamples()  << std::endl;

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
		Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);
		// printf("\n");
		// printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		// printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		// printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		// printf("\n");
		// printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

		// Save the aligned template for visualization
		// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::copyPointCloud(*cloud_source, *cloud_source_xyz);
		pcl::transformPointCloudWithNormals(*cloud_source, *cloud_aligned, best_alignment.final_transformation);
		m_math.cloudScaling(*cloud_aligned, 1); // m to mm
		Eigen::Matrix4f T_quick = best_alignment.final_transformation;
		T_quick(0, 3) = 1000.0 * T_quick(0, 3);
		T_quick(1, 3) = 1000.0 * T_quick(1, 3);
		T_quick(2, 3) = 1000.0 * T_quick(2, 3);

	    T_now = T_quick*T_now;

        //// 251006, Here
        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("***************** [%s] Quick Aling Results *****************", __func__);
        // ROS_LOG_WARN("T_now = T_quick*T_now;");
        ROS_LOG_WARN("*******************************************************");
        // printMatrixAsJson(T_now, "T_now");
        ROS_LOG_WARN("T_quick;");
        ROS_LOG_WARN("*******************************************************");
        printMatrixAsJson(T_quick, "T_quick");
        //// JSON 저장용, 251006
        T_quick_for_json = T_quick;

        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("*******************************************************");

    };
    
    // decide whether to run quick-align
    bool json_exists = fs::exists(quick_align_json_path);
    std::vector<Eigen::Matrix4f> s_Ts; int s_count = 0;
    if (!align_param.do_quick_align) { // usin Quick Align Table (JSON)
        // try to load T* only when quick-align is off
        if (json_exists) s_Ts = loadTransformsFromJsonWithCount(quick_align_json_path, &s_count);
    }

    //// 아래와 같은 경우가 발생하면 기존대로 quick_align을 수행
    const bool run_quick_align =
    align_param.do_quick_align       // explicit request
    || !json_exists                  // file missing
    || s_Ts.empty();                 // file exists but no usable T’s

    if (run_quick_align) {
        run_quick_align_proc(); // 기존 Quick align 수행
    } else { // 251007
        // quick-align off, and we have json Ts → sample one
        pcl::copyPointCloud(*cloud_source, *cloud_aligned); // [mm], cloud_source already has T_in
    
        static thread_local std::mt19937 rng{std::random_device{}()};
        std::uniform_int_distribution<size_t> dist(0, s_Ts.size() - 1);
        const size_t idx = dist(rng);
    
        Eigen::Matrix4f T_quick = s_Ts[idx]; // [mm]
        //// 변환 적용. T_now는 함수 초기에 이미 적용됨.
        pcl::transformPointCloudWithNormals(*cloud_aligned, *cloud_aligned, T_quick);
        /// update
        T_now = T_quick * T_now;
    
        ROS_LOG_WARN("[QuickAlign OFF] Sampled T_quick from JSON (idx=%zu, total=%zu, count=%d).",
                     idx, s_Ts.size(), s_count);
        printMatrixAsJson(T_quick, "T_quick(sampled)");
        T_quick_for_json = T_quick;
    }

	// Visualization
	if (b_plot)
	{
		pcl::visualization::PCLVisualizer viewer("modelMatchingReg2CADUsingQuickAlignBinPickingWithNormalVer2");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text

		
		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*Reference, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_in_color_h(Reference, 0, 0, 0);
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// raw cloud is green
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_raw_color_h(Registered, 10, 10, 255);
		viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_align_color_h(cloud_aligned, 10, 10, 255);
		viewer.addPointCloud(cloud_aligned, cloud_align_color_h, "cloud_aligned", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_aligned");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			viewer.spin();
			// viewer.spinOnce(100);
			// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		// viewer.close();
	}

    //// TODO: 코드 맨 아랫줄에 주석 처리한 Quick align 실패 방지 처리 1, 2 참고

	////////////////////////////////////////////////////////////////////////////////
	// 2) precise alignment using ICP
	Eigen::Matrix4f T_icp;
    matchingReg2CADUsingICP4NoEstNormal(align_param, *cloud_aligned, *Reference, T_icp, b_plot);
    // matchingReg2CADUsingICP5NoEstNormal(align_param, *cloud_aligned, *Reference, T_icp, true);
    
    T_now = T_icp*T_now;
    //// output
	T_out = T_now; // metric: mm
    T_quick_out = T_quick_for_json; // 251006
}

// JSON 배열을 Eigen::Matrix4f로 변환
Eigen::Matrix4f PCLPROCESSING::jsonToMatrix4f(const json& jmat) {
    Eigen::Matrix4f mat;
    for (int i = 0; i < 4; ++i) {
        for (int j2 = 0; j2 < 4; ++j2) {
            mat(i, j2) = jmat[i][j2].get<float>();
        }
    }
    return mat;
}

void PCLPROCESSING::printMatrixAsJson(const Eigen::Matrix4f& mat, const std::string& name) {
    json jmat = eigenMat4ToJson(mat);
    
    ROS_LOG_WARN("%s = %s", name.c_str(), jmat.dump().c_str());
}


json PCLPROCESSING::eigenMat4ToJson(const Eigen::Matrix4f& M) {
    json arr = json::array();
    for (int i = 0; i < 4; ++i) {
        json row = json::array();
        for (int j = 0; j < 4; ++j) row.push_back(static_cast<double>(M(i,j)));
        arr.push_back(row);
    }
    return arr;
}

Eigen::Matrix4f PCLPROCESSING::jsonToMat4(const json& jmat) {
    Eigen::Matrix4f M = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            M(i, j) = jmat.at(i).at(j).get<float>();
    return M;
}


// load all keys starting with "T" (e.g., T1, T2, ...) into a vector
std::vector<Eigen::Matrix4f> PCLPROCESSING::loadTransformsFromJson(
    const std::string& path,
    const std::string& key_prefix)
{
    std::ifstream ifs(path);
    if (!ifs) {
        ROS_LOG_WARN("Failed to open JSON file: %s", path.c_str());
        return {};
    }
    json j; ifs >> j;

    std::vector<Eigen::Matrix4f> out;
    if (!j.is_object()) {
        ROS_LOG_WARN("Root of JSON is not an object.");
        return out;
    }
    for (auto it = j.begin(); it != j.end(); ++it) {
        const std::string& k = it.key();
        if (k.size() >= key_prefix.size() &&
            k.compare(0, key_prefix.size(), key_prefix) == 0 &&
            it.value().is_array() && it.value().size() == 4)
        {
            try {
                out.push_back(jsonToMatrix4f(it.value()));
            } catch (...) {
                ROS_LOG_WARN("Skipping key %s due to parse error.", k.c_str());
            }
        }
    }
    ROS_LOG_WARN("Loaded %zu transforms from %s", out.size(), path.c_str());
    return out;
}

// Load all T* matrices and the count
std::vector<Eigen::Matrix4f> PCLPROCESSING::loadTransformsFromJsonWithCount(
        const std::string& path, int* out_count, const std::string& prefix)
{
    std::ifstream ifs(path);
    if (!ifs) {
        ROS_LOG_WARN("Failed to open JSON file: %s", path.c_str());
        if (out_count) *out_count = 0;
        return {};
    }
    json j; ifs >> j;
    if (out_count) *out_count = j.value("count", 0);

    std::vector<Eigen::Matrix4f> Ts;
    if (!j.is_object()) return Ts;

    for (auto it = j.begin(); it != j.end(); ++it) {
        const std::string& k = it.key();
        if (k.rfind(prefix, 0) == 0 && it.value().is_array() && it.value().size() == 4)
            Ts.push_back(jsonToMat4(it.value()));
    }
    return Ts;
}

// Append T_quick as "T{count+1}" and update count; create file if missing
std::string PCLPROCESSING::appendTransformAndUpdateCount(
        const std::string& path, const Eigen::Matrix4f& T, const std::string& prefix)
{
    static std::mutex mtx;
    std::lock_guard<std::mutex> lk(mtx);

    if (!fs::exists(fs::path(path).parent_path())) {
        fs::create_directories(fs::path(path).parent_path());
    }

    json j;
    {
        std::ifstream ifs(path);
        if (ifs) {
            try { ifs >> j; }
            catch (...) { j = json::object(); }
        }
    }
    if (!j.is_object()) j = json::object();

    int count = j.value("count", 0);

    // If external edits created gaps (e.g., missing T3), still use count+1
    const int next_idx = count + 1;
    const std::string key = prefix + std::to_string(next_idx);

    // Ensure last row is [0 0 0 1]
    Eigen::Matrix4f T_fixed = T;
    T_fixed.row(3) << 0.f, 0.f, 0.f, 1.f;

    j[key] = eigenMat4ToJson(T_fixed);
    j["count"] = next_idx;

    // Pretty print with stable precision
    std::ofstream ofs(path, std::ios::trunc);
    ofs << std::setw(2) << j; // indentation = 2
    ofs.flush();

    ROS_LOG_WARN("Appended %s and updated count to %d in %s", key.c_str(), next_idx, path.c_str());
    return key;
}


/** @brief Korean: Quick align을 이용하여 모델 매칭을 수행한다.
 * @param[in] cloud_reg : 정합 모델(정합된 점군 데이터)
 * @param[in] cloud_CAD : 기준 모델(CAD 점군 데이터)
 * @param[out] T_out : Quick align을 통해 추출된 두 점군 데이터 사이의 변환행렬, T_O2B
 * @param[in] b_plot : false: viewer off, true: viewer on
 */
void PCLPROCESSING::modelMatchingReg2CADUsingQuickAlignBinPickingWithNormalVer2OnlyQuickAlign(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_reg, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD, const Eigen::Matrix4f &T_in, Eigen::Matrix4f &T_out, bool b_plot)
{
    //// 내부 함수에서 법선 추정을 하지 않도록 처리
    //// 1) normal을 할당하여 얻은 fpfh feature를 사용하고,
    //// 2) ICP에서 할당된 normal를 사용함.
	printf("\n[%s]--------------  Quick align with normal assigned (Bin Picking) start!  -----------------\n", __func__);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Reference(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Registered(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::copyPointCloud(cloud_CAD, *Reference); // Reference model (CAD model)

	// 1) rough alignment using quick align
	*cloud_source = cloud_reg;					   // Registration model
    Eigen::Matrix4f T_now = Eigen::Matrix4f::Identity();

    //// initialize the source cloud
    pcl::transformPointCloudWithNormals(*cloud_source, *cloud_source, T_in);
    T_now = T_in;

    //// Quick align
    pcl::console::TicToc time;
	if(align_param.do_quick_align) {
		m_math.cloudScaling(*cloud_source, 0);		   // mm to m

        time.tic();

        //// 231129 추가
		// FeatureCloudNormal source_ft_cloud;
        // if(!align_param.is_measured_cloud_feature_assigned) {
		//     pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tempCloud_sc(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		//     *tempCloud_sc = *cloud_source;
		//     source_ft_cloud.setInputCloud(tempCloud_sc);
        //     align_param.measured_ft_cloud_with_normal = source_ft_cloud;
        // } else {
        //     source_ft_cloud = align_param.measured_ft_cloud_with_normal;
        // }
		std::vector<FeatureCloudNormal> object_templates;
		object_templates.resize(0);
		object_templates.push_back(align_param.measured_ft_cloud_with_normal);

        std::cout << "Quick align(with normal)" << " - process 1-1 time: " << time.toc() << " ms" << std::endl;

        time.tic();

		// FeatureCloudNormal target_ft_cloud;
		// target_ft_cloud.setInputCloud(tempCloud);

		TemplateAlignmentNormal template_align;
		for (size_t i = 0; i < object_templates.size(); ++i)
		{
			template_align.addTemplateCloud(object_templates[i]);
		}
        std::cout << "Quick align(with normal)" << " - process 1-2 time: " << time.toc() << " ms" << std::endl;

        time.tic();
		template_align.setTargetCloud(align_param.target_ft_cloud_with_normal); // target
        template_align.setParameters(0.001*align_param.quick_align_min_sample_distance, 0.001*align_param.quick_align_max_correspondence_distance, align_param.quick_align_iteration, align_param.quick_align_sample_num, align_param.quick_align_corr_rnd_k); // min_sample_distance, max_correspondence_distance, iteration, sample_num

		// Find the best template alignment
		TemplateAlignmentNormal::Result best_alignment;
		int best_index = template_align.findBestAlignment(best_alignment);
		const FeatureCloudNormal &best_template = object_templates[best_index];
		// Print the alignment fitness score (values less than 0.00002 are good)
		printf("[%s] Best fitness score: %f\n", __func__, best_alignment.fitness_score);
        std::cout << "Quick align(with normal)" << " - process 1-3 time: " << time.toc() << " ms" << std::endl;

        // std::cout << "NumberOfSamples: " << template_align.getNumberOfSamples()  << std::endl;

		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
		Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);
		// printf("\n");
		// printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		// printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		// printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		// printf("\n");
		// printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

		// Save the aligned template for visualization
		// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::copyPointCloud(*cloud_source, *cloud_source_xyz);
		pcl::transformPointCloudWithNormals(*cloud_source, *cloud_aligned, best_alignment.final_transformation);
		m_math.cloudScaling(*cloud_aligned, 1); // m to mm
		Eigen::Matrix4f T_quick = best_alignment.final_transformation;
		T_quick(0, 3) = 1000.0 * T_quick(0, 3);
		T_quick(1, 3) = 1000.0 * T_quick(1, 3);
		T_quick(2, 3) = 1000.0 * T_quick(2, 3);

	    

        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("***************** [%s] Quick Aling Results *****************", __func__);
        ROS_LOG_WARN("T_now = T_quick*T_now;");
        ROS_LOG_WARN("*******************************************************");
        printMatrixAsJson(T_now, "T_now");

        ROS_LOG_WARN("T_quick;");
        ROS_LOG_WARN("*******************************************************");
        
        printMatrixAsJson(T_quick, "T_quick");
        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("*******************************************************");
        ROS_LOG_WARN("*******************************************************");
	}
	else {
		// *cloud_aligned = *cloud_source;
        pcl::copyPointCloud(*cloud_source, *cloud_aligned);
	}

	////////////////////////////////////////////////////////////////////////////////
	// // 2) precise alignment using ICP
	// Eigen::Matrix4f T_icp;
    // matchingReg2CADUsingICP4NoEstNormal(align_param, *cloud_aligned, *Reference, T_icp, b_plot);
    // // matchingReg2CADUsingICP5NoEstNormal(align_param, *cloud_aligned, *Reference, T_icp, true);
    
    // T_now = T_icp*T_now;
    //// output
	T_out = T_now; // metric: mm
}

/** @brief Korean: (precise alignment) ICP을 이용하여 모델 매칭을 수행한다.
 * @param[in] corr_thre : correspondence 사이 거리 임계값
 * @param[in] cloud_reg : 정합 모델(정합된 점군 데이터)
 * @param[in] cloud_CAD : 기준 모델(CAD 점군 데이터)
 * @param[out] T_out : Quick align을 통해 추출된 두 점군 데이터 사이의 변환행렬
 */
void PCLPROCESSING::matchingReg2CADUsingICP4(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, Eigen::Matrix4f &T_out, bool b_plot)
{
	printf("\n--------------  ICP precise alignment start!  -----------------\n");
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);

	*cloud_target = cloud_CAD; // Reference (CAD model)
	*cloud_source = cloud_reg; // Registration model
	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_Reference(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_Registered(new pcl::PointCloud<pcl::PointNormal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(20);
	norm_est.setInputCloud(cloud_target);
	norm_est.compute(*points_with_normals_Reference);
	pcl::copyPointCloud(*cloud_target, *points_with_normals_Reference);
	// printf("Reference cloud K search finished\n");

    m_math.printCloudMinMax(*cloud_source, "cloud_source");
	norm_est.setInputCloud(cloud_source);
	norm_est.compute(*points_with_normals_Registered);
	pcl::copyPointCloud(*cloud_source, *points_with_normals_Registered);
	// printf("Registration cloud K search finished\n");

	setVerbosityLevel(pcl::console::L_DEBUG);
	pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
	icp.setInputSource(points_with_normals_Registered); // Rotary table to CAD coordinate
	icp.setInputTarget(points_with_normals_Reference);
	icp.setMaximumIterations(align_param.icp_max_iter_inner);
	icp.setMaxCorrespondenceDistance(align_param.icp_corr_threshold);
	icp.setEuclideanFitnessEpsilon(align_param.icp_euclidean_fitness_eps); // [mm]의 경우, 0.001
	icp.setTransformationEpsilon(align_param.icp_transformation_eps); // [mm]의 경우, 0.001

	// printf("align start\n");
	icp.align(*points_with_normals_Registered);

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

	if (icp.hasConverged())
	{
		// std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		// std::cout << "\nICP transformation " << icp_iterations << " : cloud_icp -> cloud_in" << std::endl;
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		// print4Matrix(transformation_matrix);
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
	}

	if (1)
	{
		double corres = 5.0;
		double iter = 1;
		int iter_cnt = 0;
		while (iter_cnt <= 2)
		{
			iter_cnt++;
			icp.setMaxCorrespondenceDistance(corres - iter);
			icp.align(*points_with_normals_Registered);

			if (icp.hasConverged())
			{
				// printf("\033[11A");
				// printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
				// std::cout << "Max correspondence distance: " << icp.getMaxCorrespondenceDistance() << std::endl;
				// std::cout << "\nICP transformation " << ++icp_iterations << " : cloud_icp -> cloud_in" << std::endl;
				transformation_matrix *= icp.getFinalTransformation().cast<double>();
				// print4Matrix(transformation_matrix);
				iter = iter + 0.5;
			}
			else
			{
				PCL_ERROR("\nICP has not converged.\n");
			}
		}

		if (b_plot)
		{
			pcl::visualization::PCLVisualizer viewer("ICP demo");
			// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("ICP demo"));
			int v1(0);
			int v2(1);
			viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
			viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

			float bckgr_gray_level = 1.0; // white
			float txt_gray_lvl = 0.0;	  // black text


            ///////////////////////////////////////////
            float view_scale = 800.0;
            Eigen::Vector4f centroid_CAD;
            pcl::compute3DCentroid(*points_with_normals_Reference, centroid_CAD);
            viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
            std::vector<pcl::visualization::Camera> cams;
            viewer.getCameras(cams);
            for (auto &&camera : cams)
            {
                for (int i = 0; i < 3; i++)
                {
                    camera.focal[i] = centroid_CAD[i];
                }
            }
            viewer.setCameraParameters(cams[0]);
            ///////////////////////////////////////////

			// Original point cloud is black
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_in_color_h(points_with_normals_Reference, 0, 0, 0);
			viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v1", v1);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
			viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v2", v2);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

			// Transformed point cloud is green
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h(cloud_source, 10, 10, 255);
			viewer.addPointCloud(cloud_source, cloud_tr_color_h, "cloud_tr_v1", v1);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_tr_v1");

			// ICP aligned point cloud is red
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_icp_color_h(points_with_normals_Registered, 10, 10, 255);
			viewer.addPointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2", v2);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_icp_v2");

			// Adding text descriptions in each viewport
			viewer.addText("Black: Original point cloud\nBlue: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
			viewer.addText("Black: Original point cloud\nBlue: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

			std::stringstream ss0;
			ss0 << align_param.icp_max_iter_inner;
			std::string iterations_cnt = "ICP iterations = " + ss0.str();
			viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
			viewer.setBackgroundColor(1, 1, 1, v1);
			viewer.setBackgroundColor(1, 1, 1, v2);

			// while (!viewer.wasStopped())
			// {
			// 	// viewer.spinOnce();
			// 	viewer.spinOnce(100);
			// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			// }
			viewer.spin();
			viewer.close();
		}

        ///////////// 논문용
		if (b_plot)
		{
			pcl::visualization::PCLVisualizer viewer("ICP demo2");
			// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("ICP demo"));
			int v1(0);
			int v2(1);
			viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
			viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

			float bckgr_gray_level = 1.0; // white
			float txt_gray_lvl = 0.0;	  // black text


            ///////////////////////////////////////////
            float view_scale = 800.0;
            Eigen::Vector4f centroid_CAD;
            pcl::compute3DCentroid(*points_with_normals_Reference, centroid_CAD);
            viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
            std::vector<pcl::visualization::Camera> cams;
            viewer.getCameras(cams);
            for (auto &&camera : cams)
            {
                for (int i = 0; i < 3; i++)
                {
                    camera.focal[i] = centroid_CAD[i];
                }
            }
            viewer.setCameraParameters(cams[0]);
            ///////////////////////////////////////////

			// Original point cloud is black
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_in_color_h(points_with_normals_Reference, 0, 0, 0);
			viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v1", v1);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");

			// Transformed point cloud is Blue
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_icp_color_h(points_with_normals_Registered, 10, 10, 255);
			viewer.addPointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2", v2);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_icp_v2");

			// Adding text descriptions in each viewport
			viewer.addText("Black: Original point cloud\nBlue: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
			viewer.addText("Black: Original point cloud\nBlue: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

			std::stringstream ss0;
			ss0 << align_param.icp_max_iter_inner;
			std::string iterations_cnt = "ICP iterations = " + ss0.str();
			viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
			viewer.setBackgroundColor(1, 1, 1, v1);
			viewer.setBackgroundColor(1, 1, 1, v2);

			// while (!viewer.wasStopped())
			// {
			// 	// viewer.spinOnce();
			// 	viewer.spinOnce(100);
			// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			// }
			viewer.spin();
			viewer.close();
		}



	}
	else // figure checking - each stage of icp
	{

		pcl::visualization::PCLVisualizer viewer("ICP demo");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

		float bckgr_gray_level = 1.0; // white
		float txt_gray_lvl = 0.0;	  // black text

		// Original point cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_in_color_h(points_with_normals_Reference, 0, 0, 0);
		viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v2", v2);

		// Transformed point cloud is green
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_tr_color_h(points_with_normals_Registered, 10, 10, 255);
		viewer.addPointCloud(points_with_normals_Registered, cloud_tr_color_h, "cloud_tr_v1", v1);

		// ICP aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_icp_color_h(points_with_normals_Registered, 10, 10, 255);
		viewer.addPointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2", v2);

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

		std::stringstream ss0;
		ss0 << align_param.icp_max_iter_inner;
		std::string iterations_cnt = "ICP iterations = " + ss0.str();
		viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);

		double corres = 5.0;
		double iter = 1;
		viewer.registerKeyboardCallback(&keyEventOccurred, (void *)NULL);
		// Display the visualiser
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
			if (next_iter)
			{
				icp.setMaxCorrespondenceDistance(corres - iter);
				icp.align(*points_with_normals_Registered);

				if (icp.hasConverged())
				{
					// printf("\033[11A");
					// printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
					// std::cout << "Max correspondence distance: " << icp.getMaxCorrespondenceDistance() << std::endl;
					// std::cout << "\nICP transformation " << ++icp_iterations << " : cloud_icp -> cloud_in" << std::endl;
					transformation_matrix *= icp.getFinalTransformation().cast<double>();
					// print4Matrix(transformation_matrix);

					ss0.str("");
					ss0 << align_param.icp_max_iter_inner;
					std::string iterations_cnt = "ICP iterations = " + ss0.str();
					viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
					viewer.updatePointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2");

					printf("press 'space'!!!");
					// iter = iter + 2;
					iter = iter + 0.5;
				}
				else
				{
					PCL_ERROR("\nICP has not converged.\n");
				}
			}
			next_iter = false;
		}
		viewer.close();
	}

	T_out = transformation_matrix.cast<float>();
}

/** @brief Korean: (precise alignment) ICP을 이용하여 모델 매칭을 수행한다.
 * @param[in] corr_thre : correspondence 사이 거리 임계값
 * @param[in] cloud_reg : 정합 모델(정합된 점군 데이터)
 * @param[in] cloud_CAD : 기준 모델(CAD 점군 데이터)
 * @param[out] T_out : Quick align을 통해 추출된 두 점군 데이터 사이의 변환행렬
 */
void PCLPROCESSING::matchingReg2CADUsingICP4NoEstNormal(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_reg, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD, Eigen::Matrix4f &T_out, bool b_plot)
{
	printf("\n[%s]----------- (Using assigned normal) ICP precise alignment start!  -------------\n", __func__);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(cloud_CAD, *cloud_target); // Reference (CAD model)
    pcl::copyPointCloud(cloud_reg, *cloud_source); // Registration model
	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_Reference(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_Registered(new pcl::PointCloud<pcl::PointNormal>);

    if(0) { //// CAD 파일의 법선 추정 및 flip은 초기 단계에만 계산하도록 변경 완료
        pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_est;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        norm_est.setSearchMethod(tree);
        norm_est.setKSearch(20);
        norm_est.setInputCloud(cloud_target);
        norm_est.compute(*points_with_normals_Reference);
        pcl::copyPointCloud(*cloud_target, *points_with_normals_Reference);
    } else {
	    pcl::copyPointCloud(cloud_CAD, *points_with_normals_Reference);
    }

    //// NOTICE: CAD의 경우, CAD normal을 직접 사용하면 ICP에서 NAN값이 입력되는 버그 발생 --> 일단 기존대로 추정하는 방식으로 사용
    //// NOTICE: CAD의 경우, CAD normal을 직접 사용하면 ICP에서 NAN값이 입력되는 버그 발생

    //// TODO: matchingReg2CADUsingICP4를 matchingReg2CADUsingICP4NoEstNormal로 변경하기
    //// TODO: matchingReg2CADUsingICP4를 matchingReg2CADUsingICP4NoEstNormal로 변경하기
    //// TODO: matchingReg2CADUsingICP4를 matchingReg2CADUsingICP4NoEstNormal로 변경하기
    //// TODO: matchingReg2CADUsingICP4를 matchingReg2CADUsingICP4NoEstNormal로 변경하기


    //// 측정 점군
	pcl::copyPointCloud(cloud_reg, *points_with_normals_Registered);

    // printf("***********************************************************\n");
    // printf("************************** CAD ****************************\n");
    // printf("***********************************************************\n");
    // // Test
    // // for (size_t i = 0; i < points_with_normals_Reference->size(); i++) {
    // //     printf("%0.3f, %0.3f, %0.3f // ", points_with_normals_Reference->points[i].x, points_with_normals_Reference->points[i].y, points_with_normals_Reference->points[i].z);
    // // }
    // for (size_t i = 0; i < points_with_normals_Reference->size(); i++) {
    //     printf("%0.3f, %0.3f, %0.3f // ", points_with_normals_Reference->points[i].normal_x, points_with_normals_Reference->points[i].normal_y, points_with_normals_Reference->points[i].normal_z);
    // }
    // printf("***********************************************************\n");
    // printf("***********************************************************\n");
    // printf("***********************************************************\n");

    // printf("***********************************************************\n");
    // printf("********************** Measured ***************************\n");
    // printf("***********************************************************\n");
    // // Test
    // for (size_t i = 0; i < points_with_normals_Registered->size(); i++) {
    //     printf("%0.3f, %0.3f, %0.3f // ", points_with_normals_Registered->points[i].x, points_with_normals_Registered->points[i].y, points_with_normals_Registered->points[i].z);
    // }
    // // for (size_t i = 0; i < points_with_normals_Registered->size(); i++) {
    // //     printf("%0.3f, %0.3f, %0.3f // ", points_with_normals_Registered->points[i].normal_x, points_with_normals_Registered->points[i].normal_y, points_with_normals_Registered->points[i].normal_z);
    // // }
    // printf("***********************************************************\n");
    // printf("***********************************************************\n");
    // printf("***********************************************************\n");

    // m_math.printCloudMinMax(*points_with_normals_Reference, "points_with_normals_Reference");
    // m_math.printCloudMinMax(*points_with_normals_Registered, "points_with_normals_Registered");


    // for (size_t i = 0; i < points_with_normals_Registered->size(); i++) {
    //     printf("%0.3f, %0.3f, %0.3f, ", points_with_normals_Registered->points[i].x, points_with_normals_Registered->points[i].y, points_with_normals_Registered->points[i].z);
    // }

    // for (size_t i = 0; i < points_with_normals_Registered->size(); i++) {
    //     printf("%0.3f, %0.3f, %0.3f, ", points_with_normals_Registered->points[i].normal_x, points_with_normals_Registered->points[i].normal_y, points_with_normals_Registered->points[i].normal_z);
    // }
    
    // for (size_t i = 0; i < points_with_normals_Registered->size(); i++) {
    //     printf("%0.3f, %0.3f, %0.3f, ", points_with_normals_Registered->points[i].normal_x, points_with_normals_Registered->points[i].normal_y, points_with_normals_Registered->points[i].normal_z);
    // }

	// printf("Registration cloud K search finished\n");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot_3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(cloud_reg, *cloud_plot_1);
	pcl::copyPointCloud(cloud_reg, *cloud_plot_2);
	pcl::copyPointCloud(cloud_CAD, *cloud_plot_3);
    // plotCloudCompare(*cloud_plot_1, *cloud_plot_2, *cloud_plot_3, 800.0, "-");

	printf("\n----------- 1  -------------\n");
	setVerbosityLevel(pcl::console::L_DEBUG);
	printf("\n----------- 1-1  -------------\n");
	pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
	icp.setInputSource(points_with_normals_Registered); // Rotary table to CAD coordinate
	icp.setInputTarget(points_with_normals_Reference);
	icp.setMaximumIterations(align_param.icp_max_iter_inner);
	icp.setMaxCorrespondenceDistance(align_param.icp_corr_threshold);
	icp.setEuclideanFitnessEpsilon(align_param.icp_euclidean_fitness_eps); // [mm]의 경우, 0.001
	icp.setTransformationEpsilon(align_param.icp_transformation_eps); // [mm]의 경우, 0.001

	icp.align(*points_with_normals_Registered);

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	printf("\n----------- 2  -------------\n");

	if (icp.hasConverged())
	{
		// std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		// std::cout << "\nICP transformation " << icp_iterations << " : cloud_icp -> cloud_in" << std::endl;
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		// print4Matrix(transformation_matrix);
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
	}
	printf("\n----------- 3  -------------\n");

	if (1)
	{
		double corres = 5.0;
		double iter = 1;
		int iter_cnt = 0;
		while (iter_cnt <= 2)
		{
			iter_cnt++;
			icp.setMaxCorrespondenceDistance(corres - iter);
			icp.align(*points_with_normals_Registered);

			if (icp.hasConverged())
			{
				// printf("\033[11A");
				// printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
				// std::cout << "Max correspondence distance: " << icp.getMaxCorrespondenceDistance() << std::endl;
				// std::cout << "\nICP transformation " << ++icp_iterations << " : cloud_icp -> cloud_in" << std::endl;
				transformation_matrix *= icp.getFinalTransformation().cast<double>();
				// print4Matrix(transformation_matrix);
				iter = iter + 0.5;
			}
			else
			{
				PCL_ERROR("\nICP has not converged.\n");
			}
		}
	    printf("\n----------- 4  -------------\n");

		if (b_plot)
		{
			pcl::visualization::PCLVisualizer viewer("ICP demo");
			// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("ICP demo"));
			int v1(0);
			int v2(1);
			viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
			viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

			float bckgr_gray_level = 1.0; // white
			float txt_gray_lvl = 0.0;	  // black text


            ///////////////////////////////////////////
            float view_scale = 800.0;
            Eigen::Vector4f centroid_CAD;
            pcl::compute3DCentroid(*points_with_normals_Reference, centroid_CAD);
            viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
            std::vector<pcl::visualization::Camera> cams;
            viewer.getCameras(cams);
            for (auto &&camera : cams)
            {
                for (int i = 0; i < 3; i++)
                {
                    camera.focal[i] = centroid_CAD[i];
                }
            }
            viewer.setCameraParameters(cams[0]);
            ///////////////////////////////////////////

			// Original point cloud is black
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_in_color_h(points_with_normals_Reference, 0, 0, 0);
			viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v1", v1);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
			viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v2", v2);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

			// Transformed point cloud is green
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h(cloud_source, 10, 10, 255);
			viewer.addPointCloud(cloud_source, cloud_tr_color_h, "cloud_tr_v1", v1);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_tr_v1");

			// ICP aligned point cloud is red
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_icp_color_h(points_with_normals_Registered, 10, 10, 255);
			viewer.addPointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2", v2);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_icp_v2");

			// Adding text descriptions in each viewport
			viewer.addText("Black: Original point cloud\nBlue: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
			viewer.addText("Black: Original point cloud\nBlue: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

			std::stringstream ss0;
			ss0 << align_param.icp_max_iter_inner;
			std::string iterations_cnt = "ICP iterations = " + ss0.str();
			viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
			viewer.setBackgroundColor(1, 1, 1, v1);
			viewer.setBackgroundColor(1, 1, 1, v2);

			// while (!viewer.wasStopped())
			// {
			// 	// viewer.spinOnce();
			// 	viewer.spinOnce(100);
			// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			// }
			viewer.spin();
			viewer.close();
		}

        ///////////// 논문용
		if (b_plot)
		{
			pcl::visualization::PCLVisualizer viewer("ICP demo2");
			// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("ICP demo"));
			int v1(0);
			int v2(1);
			viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
			viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

			float bckgr_gray_level = 1.0; // white
			float txt_gray_lvl = 0.0;	  // black text


            ///////////////////////////////////////////
            float view_scale = 800.0;
            Eigen::Vector4f centroid_CAD;
            pcl::compute3DCentroid(*points_with_normals_Reference, centroid_CAD);
            viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
            std::vector<pcl::visualization::Camera> cams;
            viewer.getCameras(cams);
            for (auto &&camera : cams)
            {
                for (int i = 0; i < 3; i++)
                {
                    camera.focal[i] = centroid_CAD[i];
                }
            }
            viewer.setCameraParameters(cams[0]);
            ///////////////////////////////////////////

			// Original point cloud is black
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_in_color_h(points_with_normals_Reference, 0, 0, 0);
			viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v1", v1);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");

			// Transformed point cloud is Blue
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_icp_color_h(points_with_normals_Registered, 10, 10, 255);
			viewer.addPointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2", v2);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_icp_v2");

			// Adding text descriptions in each viewport
			viewer.addText("Black: Original point cloud\nBlue: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
			viewer.addText("Black: Original point cloud\nBlue: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

			std::stringstream ss0;
			ss0 << align_param.icp_max_iter_inner;
			std::string iterations_cnt = "ICP iterations = " + ss0.str();
			viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
			viewer.setBackgroundColor(1, 1, 1, v1);
			viewer.setBackgroundColor(1, 1, 1, v2);

			// while (!viewer.wasStopped())
			// {
			// 	// viewer.spinOnce();
			// 	viewer.spinOnce(100);
			// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			// }
			viewer.spin();
			viewer.close();
		}



	}
	else // figure checking - each stage of icp
	{

		pcl::visualization::PCLVisualizer viewer("ICP demo");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

		float bckgr_gray_level = 1.0; // white
		float txt_gray_lvl = 0.0;	  // black text

		// Original point cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_in_color_h(points_with_normals_Reference, 0, 0, 0);
		viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v2", v2);

		// Transformed point cloud is green
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_tr_color_h(points_with_normals_Registered, 10, 10, 255);
		viewer.addPointCloud(points_with_normals_Registered, cloud_tr_color_h, "cloud_tr_v1", v1);

		// ICP aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_icp_color_h(points_with_normals_Registered, 10, 10, 255);
		viewer.addPointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2", v2);

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

		std::stringstream ss0;
		ss0 << align_param.icp_max_iter_inner;
		std::string iterations_cnt = "ICP iterations = " + ss0.str();
		viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);

		double corres = 5.0;
		double iter = 1;
		viewer.registerKeyboardCallback(&keyEventOccurred, (void *)NULL);
		// Display the visualiser
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
			if (next_iter)
			{
				icp.setMaxCorrespondenceDistance(corres - iter);
				icp.align(*points_with_normals_Registered);

				if (icp.hasConverged())
				{
					// printf("\033[11A");
					// printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
					// std::cout << "Max correspondence distance: " << icp.getMaxCorrespondenceDistance() << std::endl;
					// std::cout << "\nICP transformation " << ++icp_iterations << " : cloud_icp -> cloud_in" << std::endl;
					transformation_matrix *= icp.getFinalTransformation().cast<double>();
					// print4Matrix(transformation_matrix);

					ss0.str("");
					ss0 << align_param.icp_max_iter_inner;
					std::string iterations_cnt = "ICP iterations = " + ss0.str();
					viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
					viewer.updatePointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2");

					printf("press 'space'!!!");
					// iter = iter + 2;
					iter = iter + 0.5;
				}
				else
				{
					PCL_ERROR("\nICP has not converged.\n");
				}
			}
			next_iter = false;
		}
		viewer.close();
	}

	T_out = transformation_matrix.cast<float>();
}

/** @brief Korean: (precise alignment) ICP을 이용하여 모델 매칭을 수행한다.
 * @param[in] corr_thre : correspondence 사이 거리 임계값
 * @param[in] cloud_reg : 정합 모델(정합된 점군 데이터)
 * @param[in] cloud_CAD : 기준 모델(CAD 점군 데이터)
 * @param[out] T_out : Quick align을 통해 추출된 두 점군 데이터 사이의 변환행렬
 */
void PCLPROCESSING::matchingReg2CADUsingICP5NoEstNormal(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_reg, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD, Eigen::Matrix4f &T_out, bool b_plot)
{
	printf("\n[%s]----------- (Using assigned normal) ICP precise alignment start!  -------------\n", __func__);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(cloud_CAD, *cloud_target); // Reference (CAD model)
    pcl::copyPointCloud(cloud_reg, *cloud_source); // Registration model
	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_Reference(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_Registered(new pcl::PointCloud<pcl::PointNormal>);

    if(0) { //// CAD 파일의 법선 추정 및 flip은 초기 단계에만 계산하도록 변경 완료
        pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_est;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        norm_est.setSearchMethod(tree);
        norm_est.setKSearch(20);
        norm_est.setInputCloud(cloud_target);
        norm_est.compute(*points_with_normals_Reference);
        pcl::copyPointCloud(*cloud_target, *points_with_normals_Reference);
    } else {
	    pcl::copyPointCloud(cloud_CAD, *points_with_normals_Reference);
    }

    //// NOTICE: CAD의 경우, CAD normal을 직접 사용하면 ICP에서 NAN값이 입력되는 버그 발생 --> 일단 기존대로 추정하는 방식으로 사용
    //// NOTICE: CAD의 경우, CAD normal을 직접 사용하면 ICP에서 NAN값이 입력되는 버그 발생

    //// TODO: matchingReg2CADUsingICP4를 matchingReg2CADUsingICP4NoEstNormal로 변경하기
    //// TODO: matchingReg2CADUsingICP4를 matchingReg2CADUsingICP4NoEstNormal로 변경하기
    //// TODO: matchingReg2CADUsingICP4를 matchingReg2CADUsingICP4NoEstNormal로 변경하기
    //// TODO: matchingReg2CADUsingICP4를 matchingReg2CADUsingICP4NoEstNormal로 변경하기


    //// 측정 점군
	pcl::copyPointCloud(cloud_reg, *points_with_normals_Registered);

    // printf("***********************************************************\n");
    // printf("************************** CAD ****************************\n");
    // printf("***********************************************************\n");
    // // Test
    // // for (size_t i = 0; i < points_with_normals_Reference->size(); i++) {
    // //     printf("%0.3f, %0.3f, %0.3f // ", points_with_normals_Reference->points[i].x, points_with_normals_Reference->points[i].y, points_with_normals_Reference->points[i].z);
    // // }
    // for (size_t i = 0; i < points_with_normals_Reference->size(); i++) {
    //     printf("%0.3f, %0.3f, %0.3f // ", points_with_normals_Reference->points[i].normal_x, points_with_normals_Reference->points[i].normal_y, points_with_normals_Reference->points[i].normal_z);
    // }
    // printf("***********************************************************\n");
    // printf("***********************************************************\n");
    // printf("***********************************************************\n");

    // printf("***********************************************************\n");
    // printf("********************** Measured ***************************\n");
    // printf("***********************************************************\n");
    // // Test
    // for (size_t i = 0; i < points_with_normals_Registered->size(); i++) {
    //     printf("%0.3f, %0.3f, %0.3f // ", points_with_normals_Registered->points[i].x, points_with_normals_Registered->points[i].y, points_with_normals_Registered->points[i].z);
    // }
    // // for (size_t i = 0; i < points_with_normals_Registered->size(); i++) {
    // //     printf("%0.3f, %0.3f, %0.3f // ", points_with_normals_Registered->points[i].normal_x, points_with_normals_Registered->points[i].normal_y, points_with_normals_Registered->points[i].normal_z);
    // // }
    // printf("***********************************************************\n");
    // printf("***********************************************************\n");
    // printf("***********************************************************\n");

    // m_math.printCloudMinMax(*points_with_normals_Reference, "points_with_normals_Reference");
    // m_math.printCloudMinMax(*points_with_normals_Registered, "points_with_normals_Registered");


    // for (size_t i = 0; i < points_with_normals_Registered->size(); i++) {
    //     printf("%0.3f, %0.3f, %0.3f, ", points_with_normals_Registered->points[i].x, points_with_normals_Registered->points[i].y, points_with_normals_Registered->points[i].z);
    // }

    // for (size_t i = 0; i < points_with_normals_Registered->size(); i++) {
    //     printf("%0.3f, %0.3f, %0.3f, ", points_with_normals_Registered->points[i].normal_x, points_with_normals_Registered->points[i].normal_y, points_with_normals_Registered->points[i].normal_z);
    // }
    
    // for (size_t i = 0; i < points_with_normals_Registered->size(); i++) {
    //     printf("%0.3f, %0.3f, %0.3f, ", points_with_normals_Registered->points[i].normal_x, points_with_normals_Registered->points[i].normal_y, points_with_normals_Registered->points[i].normal_z);
    // }

	// printf("Registration cloud K search finished\n");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot_3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(cloud_reg, *cloud_plot_1);
	pcl::copyPointCloud(cloud_reg, *cloud_plot_2);
	pcl::copyPointCloud(cloud_CAD, *cloud_plot_3);
    // plotCloudCompare(*cloud_plot_1, *cloud_plot_2, *cloud_plot_3, 800.0, "-");

	printf("\n----------- 1  -------------\n");
	setVerbosityLevel(pcl::console::L_DEBUG);
	printf("\n----------- 1-1  -------------\n");
	pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
	icp.setInputSource(points_with_normals_Registered); // Rotary table to CAD coordinate
	icp.setInputTarget(points_with_normals_Reference);
	icp.setMaximumIterations(align_param.icp_max_iter_inner);
	icp.setMaxCorrespondenceDistance(align_param.icp_corr_threshold);
	icp.setEuclideanFitnessEpsilon(align_param.icp_euclidean_fitness_eps); // [mm]의 경우, 0.001
	icp.setTransformationEpsilon(align_param.icp_transformation_eps); // [mm]의 경우, 0.001

	icp.align(*points_with_normals_Registered);

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	printf("\n----------- 2  -------------\n");

	if (icp.hasConverged())
	{
		// std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		// std::cout << "\nICP transformation " << icp_iterations << " : cloud_icp -> cloud_in" << std::endl;
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		// print4Matrix(transformation_matrix);
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
	}
	printf("\n----------- 3  -------------\n");

	if (1)
	{
		double corres = align_param.icp_corr_threshold;
		double iter = 1.0;
		int iter_cnt = 0;
		while (iter_cnt <= align_param.icp_max_iter_inner)
		{
        	printf("\n----------- ICP ...  -------------\n");

			iter_cnt++;
			icp.setMaxCorrespondenceDistance(corres - iter);
			icp.align(*points_with_normals_Registered);

			if (icp.hasConverged())
			{
				// printf("\033[11A");
				// printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
				// std::cout << "Max correspondence distance: " << icp.getMaxCorrespondenceDistance() << std::endl;
				// std::cout << "\nICP transformation " << ++icp_iterations << " : cloud_icp -> cloud_in" << std::endl;
				transformation_matrix *= icp.getFinalTransformation().cast<double>();
				// print4Matrix(transformation_matrix);
				iter = iter + 0.5;
			}
			else
			{
				PCL_ERROR("\nICP has not converged.\n");
			}
		}
	    printf("\n----------- 4  -------------\n");

		if (b_plot)
		{
			pcl::visualization::PCLVisualizer viewer("ICP demo");
			// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("ICP demo"));
			int v1(0);
			int v2(1);
			viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
			viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

			float bckgr_gray_level = 1.0; // white
			float txt_gray_lvl = 0.0;	  // black text


            ///////////////////////////////////////////
            float view_scale = 800.0;
            Eigen::Vector4f centroid_CAD;
            pcl::compute3DCentroid(*points_with_normals_Reference, centroid_CAD);
            viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
            std::vector<pcl::visualization::Camera> cams;
            viewer.getCameras(cams);
            for (auto &&camera : cams)
            {
                for (int i = 0; i < 3; i++)
                {
                    camera.focal[i] = centroid_CAD[i];
                }
            }
            viewer.setCameraParameters(cams[0]);
            ///////////////////////////////////////////

			// Original point cloud is black
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_in_color_h(points_with_normals_Reference, 0, 0, 0);
			viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v1", v1);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
			viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v2", v2);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

			// Transformed point cloud is green
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h(cloud_source, 10, 10, 255);
			viewer.addPointCloud(cloud_source, cloud_tr_color_h, "cloud_tr_v1", v1);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_tr_v1");

			// ICP aligned point cloud is red
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_icp_color_h(points_with_normals_Registered, 10, 10, 255);
			viewer.addPointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2", v2);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_icp_v2");

			// Adding text descriptions in each viewport
			viewer.addText("Black: Original point cloud\nBlue: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
			viewer.addText("Black: Original point cloud\nBlue: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

			std::stringstream ss0;
			ss0 << align_param.icp_max_iter_inner;
			std::string iterations_cnt = "ICP iterations = " + ss0.str();
			viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
			viewer.setBackgroundColor(1, 1, 1, v1);
			viewer.setBackgroundColor(1, 1, 1, v2);

			// while (!viewer.wasStopped())
			// {
			// 	// viewer.spinOnce();
			// 	viewer.spinOnce(100);
			// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			// }
			viewer.spin();
			viewer.close();
		}

        ///////////// 논문용
		if (b_plot)
		{
			pcl::visualization::PCLVisualizer viewer("ICP demo2");
			// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("ICP demo"));
			int v1(0);
			int v2(1);
			viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
			viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

			float bckgr_gray_level = 1.0; // white
			float txt_gray_lvl = 0.0;	  // black text


            ///////////////////////////////////////////
            float view_scale = 800.0;
            Eigen::Vector4f centroid_CAD;
            pcl::compute3DCentroid(*points_with_normals_Reference, centroid_CAD);
            viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
            std::vector<pcl::visualization::Camera> cams;
            viewer.getCameras(cams);
            for (auto &&camera : cams)
            {
                for (int i = 0; i < 3; i++)
                {
                    camera.focal[i] = centroid_CAD[i];
                }
            }
            viewer.setCameraParameters(cams[0]);
            ///////////////////////////////////////////

			// Original point cloud is black
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_in_color_h(points_with_normals_Reference, 0, 0, 0);
			viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v1", v1);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");

			// Transformed point cloud is Blue
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_icp_color_h(points_with_normals_Registered, 10, 10, 255);
			viewer.addPointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2", v2);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_icp_v2");

			// Adding text descriptions in each viewport
			viewer.addText("Black: Original point cloud\nBlue: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
			viewer.addText("Black: Original point cloud\nBlue: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

			std::stringstream ss0;
			ss0 << align_param.icp_max_iter_inner;
			std::string iterations_cnt = "ICP iterations = " + ss0.str();
			viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
			viewer.setBackgroundColor(1, 1, 1, v1);
			viewer.setBackgroundColor(1, 1, 1, v2);

			// while (!viewer.wasStopped())
			// {
			// 	// viewer.spinOnce();
			// 	viewer.spinOnce(100);
			// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			// }
			viewer.spin();
			viewer.close();
		}



	}
	else // figure checking - each stage of icp
	{

		pcl::visualization::PCLVisualizer viewer("ICP demo");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

		float bckgr_gray_level = 1.0; // white
		float txt_gray_lvl = 0.0;	  // black text

		// Original point cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_in_color_h(points_with_normals_Reference, 0, 0, 0);
		viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v2", v2);

		// Transformed point cloud is green
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_tr_color_h(points_with_normals_Registered, 10, 10, 255);
		viewer.addPointCloud(points_with_normals_Registered, cloud_tr_color_h, "cloud_tr_v1", v1);

		// ICP aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_icp_color_h(points_with_normals_Registered, 10, 10, 255);
		viewer.addPointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2", v2);

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

		std::stringstream ss0;
		ss0 << align_param.icp_max_iter_inner;
		std::string iterations_cnt = "ICP iterations = " + ss0.str();
		viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);

		double corres = 5.0;
		double iter = 1;
		viewer.registerKeyboardCallback(&keyEventOccurred, (void *)NULL);
		// Display the visualiser
		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
			if (next_iter)
			{
				icp.setMaxCorrespondenceDistance(corres - iter);
				icp.align(*points_with_normals_Registered);

				if (icp.hasConverged())
				{
					// printf("\033[11A");
					// printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
					// std::cout << "Max correspondence distance: " << icp.getMaxCorrespondenceDistance() << std::endl;
					// std::cout << "\nICP transformation " << ++icp_iterations << " : cloud_icp -> cloud_in" << std::endl;
					transformation_matrix *= icp.getFinalTransformation().cast<double>();
					// print4Matrix(transformation_matrix);

					ss0.str("");
					ss0 << align_param.icp_max_iter_inner;
					std::string iterations_cnt = "ICP iterations = " + ss0.str();
					viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
					viewer.updatePointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2");

					printf("press 'space'!!!");
					// iter = iter + 2;
					iter = iter + 0.5;
				}
				else
				{
					PCL_ERROR("\nICP has not converged.\n");
				}
			}
			next_iter = false;
		}
		viewer.close();
	}

	T_out = transformation_matrix.cast<float>();
}



/** @brief Korean: (precise alignment) GICP을 이용하여 모델 매칭을 수행한다.
 * @param[in] corr_thre : correspondence 사이 거리 임계값
 * @param[in] cloud_reg : 정합 모델(정합된 점군 데이터)
 * @param[in] cloud_CAD : 기준 모델(CAD 점군 데이터)
 * @param[out] T_out : Quick align을 통해 추출된 두 점군 데이터 사이의 변환행렬
 */
void PCLPROCESSING::matchingReg2CADUsingGeneralizedICP1(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZ> &cloud_reg, const pcl::PointCloud<pcl::PointXYZ> &cloud_CAD, Eigen::Matrix4f &T_out, bool b_plot)
{
	printf("[matchingReg2CADUsingGeneralizedICP1]\n");
	printf("\n----------- (-) Generalized ICP precise alignment start!  -------------\n");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(cloud_CAD, *cloud_target); // Reference (CAD model)
    pcl::copyPointCloud(cloud_reg, *cloud_source); // Registration model
	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_Reference(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_Registered(new pcl::PointCloud<pcl::PointNormal>);

    if(0) { //// CAD 파일의 법선 추정 및 flip은 초기 단계에만 계산하도록 변경 완료
        pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_est;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        norm_est.setSearchMethod(tree);
        norm_est.setKSearch(20);
        norm_est.setInputCloud(cloud_target);
        norm_est.compute(*points_with_normals_Reference);
        pcl::copyPointCloud(*cloud_target, *points_with_normals_Reference);
    } else {
	    pcl::copyPointCloud(cloud_CAD, *points_with_normals_Reference);
    }

    //// NOTICE: CAD의 경우, CAD normal을 직접 사용하면 ICP에서 NAN값이 입력되는 버그 발생 --> 일단 기존대로 추정하는 방식으로 사용
    //// NOTICE: CAD의 경우, CAD normal을 직접 사용하면 ICP에서 NAN값이 입력되는 버그 발생

    //// 측정 점군
	pcl::copyPointCloud(cloud_reg, *points_with_normals_Registered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot_3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(cloud_reg, *cloud_plot_1);
	pcl::copyPointCloud(cloud_reg, *cloud_plot_2);
	pcl::copyPointCloud(cloud_CAD, *cloud_plot_3);
    // plotCloudCompare(*cloud_plot_1, *cloud_plot_2, *cloud_plot_3, 800.0, "-");

	printf("\n----------- 1  -------------\n");
	setVerbosityLevel(pcl::console::L_DEBUG);
	printf("\n----------- 1-1  -------------\n");

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// 기존 ICP
	// pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
	// icp.setInputSource(points_with_normals_Registered); // Rotary table to CAD coordinate
	// icp.setInputTarget(points_with_normals_Reference);
	// icp.setMaximumIterations(align_param.icp_max_iter_inner);
	// icp.setMaxCorrespondenceDistance(align_param.icp_corr_threshold);
	// icp.setEuclideanFitnessEpsilon(align_param.icp_euclidean_fitness_eps); // [mm]의 경우, 0.001
	// icp.setTransformationEpsilon(align_param.icp_transformation_eps); // [mm]의 경우, 0.001

	// icp.align(*points_with_normals_Registered);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// GICP
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaxCorrespondenceDistance(align_param.icp_corr_threshold);
    gicp.setTransformationEpsilon(align_param.icp_transformation_eps); // [mm]의 경우, 0.001
    gicp.setMaximumIterations(align_param.icp_max_iter_inner);

    // 걸리는 시간 측정
    // chrono::system_clock::time_point t_start = chrono::system_clock::now();

    // Registration 시행
    gicp.setInputSource(cloud_source);
    gicp.setInputTarget(cloud_target);
    gicp.align(*cloud_source_aligned);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	printf("\n----------- 2  -------------\n");

	if (gicp.hasConverged())
	{
		// std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		// std::cout << "\nICP transformation " << icp_iterations << " : cloud_icp -> cloud_in" << std::endl;
		transformation_matrix = gicp.getFinalTransformation().cast<double>();
		// print4Matrix(transformation_matrix);
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
	}
	printf("\n----------- 3  -------------\n");

	if (1)
	{
		double corres = 5.0;
		double iter = 1;
		int iter_cnt = 0;
		while (iter_cnt <= 2)
		{
			iter_cnt++;
			gicp.setMaxCorrespondenceDistance(corres - iter);
			gicp.align(*cloud_source_aligned);

			if (gicp.hasConverged())
			{
				// printf("\033[11A");
				// printf("\nICP has converged, score is %+.0e\n", gicp.getFitnessScore());
				// std::cout << "Max correspondence distance: " << gicp.getMaxCorrespondenceDistance() << std::endl;
				// std::cout << "\nICP transformation " << ++icp_iterations << " : cloud_icp -> cloud_in" << std::endl;
				transformation_matrix *= gicp.getFinalTransformation().cast<double>();
				// print4Matrix(transformation_matrix);
				iter = iter + 0.5;
			}
			else
			{
				PCL_ERROR("\nICP has not converged.\n");
			}
		}
	    printf("\n----------- 4  -------------\n");

		if (b_plot)
		{
			pcl::visualization::PCLVisualizer viewer("ICP demo");
			// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("ICP demo"));
			int v1(0);
			int v2(1);
			viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
			viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

			float bckgr_gray_level = 1.0; // white
			float txt_gray_lvl = 0.0;	  // black text


            ///////////////////////////////////////////
            float view_scale = 800.0;
            Eigen::Vector4f centroid_CAD;
            pcl::compute3DCentroid(*points_with_normals_Reference, centroid_CAD);
            viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
            std::vector<pcl::visualization::Camera> cams;
            viewer.getCameras(cams);
            for (auto &&camera : cams)
            {
                for (int i = 0; i < 3; i++)
                {
                    camera.focal[i] = centroid_CAD[i];
                }
            }
            viewer.setCameraParameters(cams[0]);
            ///////////////////////////////////////////

			// Original point cloud is black
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_in_color_h(points_with_normals_Reference, 0, 0, 0);
			viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v1", v1);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
			viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v2", v2);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

			// Transformed point cloud is green
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h(cloud_source, 10, 10, 255);
			viewer.addPointCloud(cloud_source, cloud_tr_color_h, "cloud_tr_v1", v1);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_tr_v1");

			// ICP aligned point cloud is red
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_icp_color_h(points_with_normals_Registered, 10, 10, 255);
			viewer.addPointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2", v2);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_icp_v2");

			// Adding text descriptions in each viewport
			viewer.addText("Black: Original point cloud\nBlue: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
			viewer.addText("Black: Original point cloud\nBlue: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

			std::stringstream ss0;
			ss0 << align_param.icp_max_iter_inner;
			std::string iterations_cnt = "ICP iterations = " + ss0.str();
			viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
			viewer.setBackgroundColor(1, 1, 1, v1);
			viewer.setBackgroundColor(1, 1, 1, v2);

			// while (!viewer.wasStopped())
			// {
			// 	// viewer.spinOnce();
			// 	viewer.spinOnce(100);
			// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			// }
			viewer.spin();
			viewer.close();
		}

        ///////////// 논문용
		if (b_plot)
		{
			pcl::visualization::PCLVisualizer viewer("ICP demo2");
			// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("ICP demo"));
			int v1(0);
			int v2(1);
			viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
			viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

			float bckgr_gray_level = 1.0; // white
			float txt_gray_lvl = 0.0;	  // black text


            ///////////////////////////////////////////
            float view_scale = 800.0;
            Eigen::Vector4f centroid_CAD;
            pcl::compute3DCentroid(*points_with_normals_Reference, centroid_CAD);
            viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
            std::vector<pcl::visualization::Camera> cams;
            viewer.getCameras(cams);
            for (auto &&camera : cams)
            {
                for (int i = 0; i < 3; i++)
                {
                    camera.focal[i] = centroid_CAD[i];
                }
            }
            viewer.setCameraParameters(cams[0]);
            ///////////////////////////////////////////

			// Original point cloud is black
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_in_color_h(points_with_normals_Reference, 0, 0, 0);
			viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v1", v1);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");

			// Transformed point cloud is Blue
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_icp_color_h(points_with_normals_Registered, 10, 10, 255);
			viewer.addPointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2", v2);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_icp_v2");

			// Adding text descriptions in each viewport
			viewer.addText("Black: Original point cloud\nBlue: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
			viewer.addText("Black: Original point cloud\nBlue: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

			std::stringstream ss0;
			ss0 << align_param.icp_max_iter_inner;
			std::string iterations_cnt = "ICP iterations = " + ss0.str();
			viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
			viewer.setBackgroundColor(1, 1, 1, v1);
			viewer.setBackgroundColor(1, 1, 1, v2);

			// while (!viewer.wasStopped())
			// {
			// 	// viewer.spinOnce();
			// 	viewer.spinOnce(100);
			// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			// }
			viewer.spin();
			viewer.close();
		}



	}
	else // figure checking - each stage of icp
	{

		// pcl::visualization::PCLVisualizer viewer("ICP demo");
		// int v1(0);
		// int v2(1);
		// viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		// viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

		// float bckgr_gray_level = 1.0; // white
		// float txt_gray_lvl = 0.0;	  // black text

		// // Original point cloud is black
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_in_color_h(points_with_normals_Reference, 0, 0, 0);
		// viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v1", v1);
		// viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v2", v2);

		// // Transformed point cloud is green
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_tr_color_h(points_with_normals_Registered, 10, 10, 255);
		// viewer.addPointCloud(points_with_normals_Registered, cloud_tr_color_h, "cloud_tr_v1", v1);

		// // ICP aligned point cloud is red
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_icp_color_h(points_with_normals_Registered, 10, 10, 255);
		// viewer.addPointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2", v2);

		// // Adding text descriptions in each viewport
		// viewer.addText("Black: Original point cloud\nBlue: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
		// viewer.addText("Black: Original point cloud\nBlue: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

		// std::stringstream ss0;
		// ss0 << align_param.icp_max_iter_inner;
		// std::string iterations_cnt = "ICP iterations = " + ss0.str();
		// viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
		// viewer.setBackgroundColor(1, 1, 1, v1);
		// viewer.setBackgroundColor(1, 1, 1, v2);

		// double corres = 5.0;
		// double iter = 1;
		// viewer.registerKeyboardCallback(&keyEventOccurred, (void *)NULL);
		// // Display the visualiser
		// while (!viewer.wasStopped())
		// {
		// 	viewer.spinOnce();
		// 	if (next_iter)
		// 	{
		// 		gicp.setMaxCorrespondenceDistance(corres - iter);
		// 		gicp.align(*cloud_source_aligned);

		// 		if (gicp.hasConverged())
		// 		{
		// 			// printf("\033[11A");
		// 			// printf("\nICP has converged, score is %+.0e\n", gicp.getFitnessScore());
		// 			// std::cout << "Max correspondence distance: " << gicp.getMaxCorrespondenceDistance() << std::endl;
		// 			// std::cout << "\nICP transformation " << ++icp_iterations << " : cloud_icp -> cloud_in" << std::endl;
		// 			transformation_matrix *= gicp.getFinalTransformation().cast<double>();
		// 			// print4Matrix(transformation_matrix);

		// 			ss0.str("");
		// 			ss0 << align_param.icp_max_iter_inner;
		// 			std::string iterations_cnt = "ICP iterations = " + ss0.str();
		// 			viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
		// 			viewer.updatePointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2");

		// 			printf("press 'space'!!!");
		// 			// iter = iter + 2;
		// 			iter = iter + 0.5;
		// 		}
		// 		else
		// 		{
		// 			PCL_ERROR("\nICP has not converged.\n");
		// 		}
		// 	}
		// 	next_iter = false;
		// }
		// viewer.close();
	}

	T_out = transformation_matrix.cast<float>();
}

/** @brief Korean: (precise alignment) GICP을 이용하여 모델 매칭을 수행한다.
 * @param[in] corr_thre : correspondence 사이 거리 임계값
 * @param[in] cloud_reg : 정합 모델(정합된 점군 데이터)
 * @param[in] cloud_CAD : 기준 모델(CAD 점군 데이터)
 * @param[out] T_out : Quick align을 통해 추출된 두 점군 데이터 사이의 변환행렬
 */
void PCLPROCESSING::matchingReg2CADUsingGeneralizedICP1NoEstNormal(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_reg, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD, Eigen::Matrix4f &T_out, bool b_plot)
{
	printf("[matchingReg2CADUsingGeneralizedICP1NoEstNormal]\n");
	printf("\n----------- (Using assigned normal) Generalized ICP precise alignment start!  -------------\n");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(cloud_CAD, *cloud_target); // Reference (CAD model)
    pcl::copyPointCloud(cloud_reg, *cloud_source); // Registration model
	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_Reference(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_Registered(new pcl::PointCloud<pcl::PointNormal>);

    if(0) { //// CAD 파일의 법선 추정 및 flip은 초기 단계에만 계산하도록 변경 완료
        pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_est;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        norm_est.setSearchMethod(tree);
        norm_est.setKSearch(20);
        norm_est.setInputCloud(cloud_target);
        norm_est.compute(*points_with_normals_Reference);
        pcl::copyPointCloud(*cloud_target, *points_with_normals_Reference);
    } else {
	    pcl::copyPointCloud(cloud_CAD, *points_with_normals_Reference);
    }

    //// NOTICE: CAD의 경우, CAD normal을 직접 사용하면 ICP에서 NAN값이 입력되는 버그 발생 --> 일단 기존대로 추정하는 방식으로 사용
    //// NOTICE: CAD의 경우, CAD normal을 직접 사용하면 ICP에서 NAN값이 입력되는 버그 발생

    //// 측정 점군
	pcl::copyPointCloud(cloud_reg, *points_with_normals_Registered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot_3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(cloud_reg, *cloud_plot_1);
	pcl::copyPointCloud(cloud_reg, *cloud_plot_2);
	pcl::copyPointCloud(cloud_CAD, *cloud_plot_3);
    // plotCloudCompare(*cloud_plot_1, *cloud_plot_2, *cloud_plot_3, 800.0, "-");

	printf("\n----------- 1  -------------\n");
	setVerbosityLevel(pcl::console::L_DEBUG);
	printf("\n----------- 1-1  -------------\n");

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// 기존 ICP
	// pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
	// icp.setInputSource(points_with_normals_Registered); // Rotary table to CAD coordinate
	// icp.setInputTarget(points_with_normals_Reference);
	// icp.setMaximumIterations(align_param.icp_max_iter_inner);
	// icp.setMaxCorrespondenceDistance(align_param.icp_corr_threshold);
	// icp.setEuclideanFitnessEpsilon(align_param.icp_euclidean_fitness_eps); // [mm]의 경우, 0.001
	// icp.setTransformationEpsilon(align_param.icp_transformation_eps); // [mm]의 경우, 0.001

	// icp.align(*points_with_normals_Registered);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// GICP
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaxCorrespondenceDistance(align_param.icp_corr_threshold);
    gicp.setTransformationEpsilon(align_param.icp_transformation_eps); // [mm]의 경우, 0.001
    gicp.setMaximumIterations(align_param.icp_max_iter_inner);

    // 걸리는 시간 측정
    // chrono::system_clock::time_point t_start = chrono::system_clock::now();

    // Registration 시행
    gicp.setInputSource(cloud_source);
    gicp.setInputTarget(cloud_target);
    gicp.align(*cloud_source_aligned);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	printf("\n----------- 2  -------------\n");

	if (gicp.hasConverged())
	{
		// std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		// std::cout << "\nICP transformation " << icp_iterations << " : cloud_icp -> cloud_in" << std::endl;
		transformation_matrix = gicp.getFinalTransformation().cast<double>();
		// print4Matrix(transformation_matrix);
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
	}
	printf("\n----------- 3  -------------\n");

	if (1)
	{
		double corres = 5.0;
		double iter = 1;
		int iter_cnt = 0;
		while (iter_cnt <= 2)
		{
			iter_cnt++;
			gicp.setMaxCorrespondenceDistance(corres - iter);
			gicp.align(*cloud_source_aligned);

			if (gicp.hasConverged())
			{
				// printf("\033[11A");
				// printf("\nICP has converged, score is %+.0e\n", gicp.getFitnessScore());
				// std::cout << "Max correspondence distance: " << gicp.getMaxCorrespondenceDistance() << std::endl;
				// std::cout << "\nICP transformation " << ++icp_iterations << " : cloud_icp -> cloud_in" << std::endl;
				transformation_matrix *= gicp.getFinalTransformation().cast<double>();
				// print4Matrix(transformation_matrix);
				iter = iter + 0.5;
			}
			else
			{
				PCL_ERROR("\nICP has not converged.\n");
			}
		}
	    printf("\n----------- 4  -------------\n");

		if (b_plot)
		{
			pcl::visualization::PCLVisualizer viewer("ICP demo");
			// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("ICP demo"));
			int v1(0);
			int v2(1);
			viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
			viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

			float bckgr_gray_level = 1.0; // white
			float txt_gray_lvl = 0.0;	  // black text


            ///////////////////////////////////////////
            float view_scale = 800.0;
            Eigen::Vector4f centroid_CAD;
            pcl::compute3DCentroid(*points_with_normals_Reference, centroid_CAD);
            viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
            std::vector<pcl::visualization::Camera> cams;
            viewer.getCameras(cams);
            for (auto &&camera : cams)
            {
                for (int i = 0; i < 3; i++)
                {
                    camera.focal[i] = centroid_CAD[i];
                }
            }
            viewer.setCameraParameters(cams[0]);
            ///////////////////////////////////////////

			// Original point cloud is black
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_in_color_h(points_with_normals_Reference, 0, 0, 0);
			viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v1", v1);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
			viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v2", v2);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

			// Transformed point cloud is green
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h(cloud_source, 10, 10, 255);
			viewer.addPointCloud(cloud_source, cloud_tr_color_h, "cloud_tr_v1", v1);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_tr_v1");

			// ICP aligned point cloud is red
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_icp_color_h(points_with_normals_Registered, 10, 10, 255);
			viewer.addPointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2", v2);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_icp_v2");

			// Adding text descriptions in each viewport
			viewer.addText("Black: Original point cloud\nBlue: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
			viewer.addText("Black: Original point cloud\nBlue: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

			std::stringstream ss0;
			ss0 << align_param.icp_max_iter_inner;
			std::string iterations_cnt = "ICP iterations = " + ss0.str();
			viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
			viewer.setBackgroundColor(1, 1, 1, v1);
			viewer.setBackgroundColor(1, 1, 1, v2);

			// while (!viewer.wasStopped())
			// {
			// 	// viewer.spinOnce();
			// 	viewer.spinOnce(100);
			// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			// }
			viewer.spin();
			viewer.close();
		}

        ///////////// 논문용
		if (b_plot)
		{
			pcl::visualization::PCLVisualizer viewer("ICP demo2");
			// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("ICP demo"));
			int v1(0);
			int v2(1);
			viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
			viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

			float bckgr_gray_level = 1.0; // white
			float txt_gray_lvl = 0.0;	  // black text


            ///////////////////////////////////////////
            float view_scale = 800.0;
            Eigen::Vector4f centroid_CAD;
            pcl::compute3DCentroid(*points_with_normals_Reference, centroid_CAD);
            viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
            std::vector<pcl::visualization::Camera> cams;
            viewer.getCameras(cams);
            for (auto &&camera : cams)
            {
                for (int i = 0; i < 3; i++)
                {
                    camera.focal[i] = centroid_CAD[i];
                }
            }
            viewer.setCameraParameters(cams[0]);
            ///////////////////////////////////////////

			// Original point cloud is black
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_in_color_h(points_with_normals_Reference, 0, 0, 0);
			viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v1", v1);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");

			// Transformed point cloud is Blue
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_icp_color_h(points_with_normals_Registered, 10, 10, 255);
			viewer.addPointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2", v2);
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_icp_v2");

			// Adding text descriptions in each viewport
			viewer.addText("Black: Original point cloud\nBlue: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
			viewer.addText("Black: Original point cloud\nBlue: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

			std::stringstream ss0;
			ss0 << align_param.icp_max_iter_inner;
			std::string iterations_cnt = "ICP iterations = " + ss0.str();
			viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
			viewer.setBackgroundColor(1, 1, 1, v1);
			viewer.setBackgroundColor(1, 1, 1, v2);

			// while (!viewer.wasStopped())
			// {
			// 	// viewer.spinOnce();
			// 	viewer.spinOnce(100);
			// 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
			// }
			viewer.spin();
			viewer.close();
		}



	}
	else // figure checking - each stage of icp
	{

		// pcl::visualization::PCLVisualizer viewer("ICP demo");
		// int v1(0);
		// int v2(1);
		// viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		// viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

		// float bckgr_gray_level = 1.0; // white
		// float txt_gray_lvl = 0.0;	  // black text

		// // Original point cloud is black
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_in_color_h(points_with_normals_Reference, 0, 0, 0);
		// viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v1", v1);
		// viewer.addPointCloud(points_with_normals_Reference, cloud_in_color_h, "cloud_in_v2", v2);

		// // Transformed point cloud is green
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_tr_color_h(points_with_normals_Registered, 10, 10, 255);
		// viewer.addPointCloud(points_with_normals_Registered, cloud_tr_color_h, "cloud_tr_v1", v1);

		// // ICP aligned point cloud is red
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> cloud_icp_color_h(points_with_normals_Registered, 10, 10, 255);
		// viewer.addPointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2", v2);

		// // Adding text descriptions in each viewport
		// viewer.addText("Black: Original point cloud\nBlue: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
		// viewer.addText("Black: Original point cloud\nBlue: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

		// std::stringstream ss0;
		// ss0 << align_param.icp_max_iter_inner;
		// std::string iterations_cnt = "ICP iterations = " + ss0.str();
		// viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);
		// viewer.setBackgroundColor(1, 1, 1, v1);
		// viewer.setBackgroundColor(1, 1, 1, v2);

		// double corres = 5.0;
		// double iter = 1;
		// viewer.registerKeyboardCallback(&keyEventOccurred, (void *)NULL);
		// // Display the visualiser
		// while (!viewer.wasStopped())
		// {
		// 	viewer.spinOnce();
		// 	if (next_iter)
		// 	{
		// 		gicp.setMaxCorrespondenceDistance(corres - iter);
		// 		gicp.align(*cloud_source_aligned);

		// 		if (gicp.hasConverged())
		// 		{
		// 			// printf("\033[11A");
		// 			// printf("\nICP has converged, score is %+.0e\n", gicp.getFitnessScore());
		// 			// std::cout << "Max correspondence distance: " << gicp.getMaxCorrespondenceDistance() << std::endl;
		// 			// std::cout << "\nICP transformation " << ++icp_iterations << " : cloud_icp -> cloud_in" << std::endl;
		// 			transformation_matrix *= gicp.getFinalTransformation().cast<double>();
		// 			// print4Matrix(transformation_matrix);

		// 			ss0.str("");
		// 			ss0 << align_param.icp_max_iter_inner;
		// 			std::string iterations_cnt = "ICP iterations = " + ss0.str();
		// 			viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
		// 			viewer.updatePointCloud(points_with_normals_Registered, cloud_icp_color_h, "cloud_icp_v2");

		// 			printf("press 'space'!!!");
		// 			// iter = iter + 2;
		// 			iter = iter + 0.5;
		// 		}
		// 		else
		// 		{
		// 			PCL_ERROR("\nICP has not converged.\n");
		// 		}
		// 	}
		// 	next_iter = false;
		// }
		// viewer.close();
	}

	T_out = transformation_matrix.cast<float>();
}



/** @brief Korean: 내/외부 면 사이의 두께가 얇은 경우, 매칭 결과가 잘못된 면에 붙는 경우가 발생함. 이를 법선 정보를 이용하여 처리
 * details 
 */
void PCLPROCESSING::matchingSurfaceCheckAndICPVer1(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZ> &cloud_major_segment, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_reg, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD, pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &md_KdTree, const Eigen::Matrix4f &T_in, const Eigen::Matrix4f &T_B2S, Eigen::Matrix4f &T_out, bool b_plot) {
    printf("\n--------------  Post processing - surface check and ICP!  -----------------\n");
    pcl::PointCloud<pcl::PointXYZ>::Ptr Reference(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Reference_inner(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Reference_outer(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_wrt_sensor_frame(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_aligned_rgbn(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::copyPointCloud(cloud_CAD, *cloud_target_xyz); // Reference model (CAD model)
	pcl::copyPointCloud(cloud_CAD, *cloud_target); // Reference model (CAD model)
	pcl::copyPointCloud(cloud_reg, *cloud_source); // Measured model
    Eigen::Matrix4f T_now = Eigen::Matrix4f::Identity();

    //// initialize the source cloud
    pcl::transformPointCloud(*cloud_source, *cloud_source, T_in);
    T_now = T_in;

    // Visualization - before post processing
    bool b_plot_tmp = false;
	if (b_plot_tmp)
	{
		pcl::visualization::PCLVisualizer viewer("raw data before post processing");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text
		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*cloud_target, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_in_color_h(cloud_target, 0, 0, 0);
		viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// // raw cloud is green
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(Registered, 10, 10, 255);
		// viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_align_color_h(cloud_source, 10, 10, 255);
		viewer.addPointCloud(cloud_source, cloud_align_color_h, "cloud_source", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		viewer.close();
	}
    
    //// 1) Surface check using normal - 최종: 내부면을 제거하고, Euclidean filter를 통해 지배적인 면만 남긴 후, ICP 재수행
    if(1)
    {
        // double dot_tmp = 0.0;
        // Eigen::Vector3d query_point, vec_cad, vec_meas;
        // for (size_t i = 0; i < cloud_source->size(); i++)
        // {
        //     m_math.cloud2ptNormal(*cloud_source, i, query_point, vec_meas);
        //     if(std::isnan(query_point[0]) || std::isnan(query_point[1]) || std::isnan(query_point[2])) 
        //     {
        //         // printf("nan point- query_point!\n");
        //         continue;
        //     }
        //     // KdTree
        //     std::vector<int> nearest_idx;
        //     std::vector<float> nearest_dists;
        //     m_math.findKNearestIndex(md_KdTree, query_point, nearest_idx, nearest_dists, 1);
        //     vec_cad[0] = cloud_CAD.points[nearest_idx[0]].normal_x;
        //     vec_cad[1] = cloud_CAD.points[nearest_idx[0]].normal_y;
        //     vec_cad[2] = cloud_CAD.points[nearest_idx[0]].normal_z;

        //     // normal check
        //     if (m_math.dotCheck2(vec_cad, vec_meas, 90.0)) { Reference->points.push_back(cloud_target_xyz->points[nearest_idx[0]]); }
        // }

        ///////////////////////////////////////////////
        //// 방법 2
        //// 해당 idx 제거
        // double dot_tmp = 0.0;
        // Eigen::Vector3d query_point, vec_cad, vec_meas;
        // std::vector<int> idx_out;
        // for (size_t i = 0; i < cloud_source->size(); i++)
        // {
        //     m_math.cloud2ptNormal(*cloud_source, i, query_point, vec_meas);
        //     if(std::isnan(query_point[0]) || std::isnan(query_point[1]) || std::isnan(query_point[2])) 
        //     {
        //         // printf("nan point- query_point!\n");
        //         continue;
        //     }
        //     // KdTree
        //     // Radius 이내의 점 중에 법선이 반대방향이면 해당 인덱스의 점을 제거
        //     std::vector<int> nearest_idx;
        //     std::vector<float> nearest_dists;
        //     // m_math.findKNearestIndex(md_KdTree, query_point, nearest_idx, nearest_dists, 1);
        //     m_math.findRadiusNearestIndex(md_KdTree, query_point, nearest_idx, nearest_dists, 10.0);
        //     for (size_t j = 0; j < nearest_idx.size(); j++)
        //     {
        //         vec_cad[0] = cloud_CAD.points[nearest_idx[j]].normal_x;
        //         vec_cad[1] = cloud_CAD.points[nearest_idx[j]].normal_y;
        //         vec_cad[2] = cloud_CAD.points[nearest_idx[j]].normal_z;
        //         // normal check
        //         if (!m_math.dotCheck2(vec_cad, vec_meas, 90.0)) { idx_out.push_back(nearest_idx[j]); }
        //     }
        // }
        // m_math.removeDuplicates(idx_out);

        // for (size_t i = 0; i < cloud_target_xyz->size(); i++)
        // {
        //     bool do_exclude_pt = false;
        //     for (size_t j = 0; j < idx_out.size(); j++)
        //     {
        //         if(i == idx_out[j]) 
        //         { 
        //             do_exclude_pt = true; 
        //             break;
        //         }
        //     }
        //     if(do_exclude_pt) continue;
        //     Reference->points.push_back(cloud_target_xyz->points[i]);
        // }

        ///////////////////////////////////////////////
        //// 방법 3 내부면 제거
        // Eigen::Vector3d com_pt = {-33.33, 33.33, 0.0};
        Eigen::Vector3d com_pt = {0.0, 0.0, 0.0};
        for (size_t i = 0; i < cloud_target->size(); i++)
        {
            Eigen::Vector3d vec_inner_check;
            vec_inner_check[0] = cloud_target->points[i].x - com_pt[0];
            vec_inner_check[1] = cloud_target->points[i].y - com_pt[1];
            vec_inner_check[2] = cloud_target->points[i].z - com_pt[2];
            vec_inner_check.normalize();

            Eigen::Vector3d query_point, vec_cad;
            m_math.cloud2ptNormal(*cloud_target, i, query_point, vec_cad);

            if (m_math.dotCheck2(vec_inner_check, vec_cad, 90.0)) { Reference_outer->points.push_back(cloud_target_xyz->points[i]); }
            else { Reference_inner->points.push_back(cloud_target_xyz->points[i]); }
        }
        pcl::copyPointCloud(*Reference_outer, *Reference);

    }
    else
    {
        // // CAD 모델을 Sensor frame 기준으로 변환
        // Eigen::Matrix4f T_B2O = T_in.inverse();
        // Eigen::Matrix4f T_S2B = T_B2S.inverse();
        // Eigen::Matrix4f T_S2O = T_S2B*T_B2O;
        // pcl::transformPointCloud(*cloud_target, *cloud_target_wrt_sensor_frame, T_S2O);
        // double dot_tmp = 0.0;
        // Eigen::Vector3d query_point, vec_cad, tmp, vec_meas;
        // Eigen::Vector3d sensor_origin = {0.0, 0.0, 0.0};
        // for (size_t i = 0; i < cloud_target_wrt_sensor_frame->size(); i++)
        // {
        //     // 센서 좌표계 기준의 CAD normal 벡터
        //     vec_cad[0] = cloud_target_wrt_sensor_frame->points[i].normal_x;
        //     vec_cad[1] = cloud_target_wrt_sensor_frame->points[i].normal_y;
        //     vec_cad[2] = cloud_target_wrt_sensor_frame->points[i].normal_z;

        //     // CAD point로부터 센서 중심을 향하는 방향 벡터
        //     vec_meas[0] = sensor_origin[0] - cloud_target_wrt_sensor_frame->points[i].x;
        //     vec_meas[1] = sensor_origin[1] - cloud_target_wrt_sensor_frame->points[i].y;
        //     vec_meas[2] = sensor_origin[2] - cloud_target_wrt_sensor_frame->points[i].z;
        //     vec_meas.normalize();

        //     // normal check
        //     if (m_math.dotCheck2(vec_cad, vec_meas, 90.0)) { Reference->points.push_back(cloud_target_xyz->points[i]); }
        // }


        //// 매칭이 잘 이루어져야 아래 방법이 유효함 --> 사용하기 어려움
        // Eigen::Matrix4f T_B2O = T_in.inverse();
        // Eigen::Matrix4f T_S2B = T_B2S.inverse();
        // Eigen::Matrix4f T_S2O = T_S2B*T_B2O;
        // pcl::transformPointCloud(*cloud_target, *cloud_target_wrt_sensor_frame, T_S2O);
        // double dot_tmp = 0.0;
        // Eigen::Vector3d query_point, vec_cad, tmp, vec_meas;
        // Eigen::Vector3d sensor_origin = {0.0, 0.0, 0.0};
        // for (size_t i = 0; i < cloud_target_wrt_sensor_frame->size(); i++)
        // {
        //     // 센서 좌표계 기준의 CAD normal 벡터
        //     vec_cad[0] = cloud_target_wrt_sensor_frame->points[i].normal_x;
        //     vec_cad[1] = cloud_target_wrt_sensor_frame->points[i].normal_y;
        //     vec_cad[2] = cloud_target_wrt_sensor_frame->points[i].normal_z;

        //     // CAD point로부터 센서 중심을 향하는 방향 벡터
        //     vec_meas[0] = sensor_origin[0] - cloud_target_wrt_sensor_frame->points[i].x;
        //     vec_meas[1] = sensor_origin[1] - cloud_target_wrt_sensor_frame->points[i].y;
        //     vec_meas[2] = sensor_origin[2] - cloud_target_wrt_sensor_frame->points[i].z;
        //     vec_meas.normalize();

        //     // normal check
        //     if (m_math.dotCheck2(vec_cad, vec_meas, 90.0)) { Reference->points.push_back(cloud_target_xyz->points[i]); }
        // }
    }


    if(0)
    {
        pcl::copyPointCloud(*cloud_source, *cloud_aligned);
    }
    else // euclidean cluster 중에 가장 점 개수가 많은 cloud
    {
        pcl::copyPointCloud(cloud_major_segment, *cloud_aligned);
        pcl::copyPointCloud(cloud_reg, *cloud_aligned_rgbn);
        pcl::transformPointCloud(*cloud_aligned, *cloud_aligned, T_in);
    }

	////////////////////////////////////////////////////////////////////////////////
	// 2) precise alignment using ICP
    m_math.printCloudMinMax(*cloud_aligned, "cloud_aligned");
    m_math.printCloudMinMax(*cloud_aligned_rgbn, "cloud_aligned_rgbn");
    m_math.printCloudMinMax(*Reference, "Reference");
    
	Eigen::Matrix4f T_icp;
    matchingReg2CADUsingICP4(align_param, *cloud_aligned, *Reference, T_icp, b_plot); // w/ normal est.
    T_now = T_icp*T_now;
	// printf("----------- T_icp ------------\n");
	// print4Matrix(T_icp.cast<double>());

    //// output
	T_out = T_now; // metric: mm

	// Visualization
	if (b_plot_tmp)
	{
        pcl::transformPointCloud(cloud_reg, *cloud_source, T_now);
		pcl::visualization::PCLVisualizer viewer("raw data after post processing");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text
		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*Reference, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(Reference, 0, 0, 0);
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(Reference, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// // raw cloud is green
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(Registered, 10, 10, 255);
		// viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_align_color_h(cloud_source, 10, 10, 255);
		viewer.addPointCloud(cloud_source, cloud_align_color_h, "cloud_source", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		viewer.close();
	}
}

/** @brief Korean: 내/외부 면 사이의 두께가 얇은 경우, 매칭 결과가 잘못된 면에 붙는 경우가 발생함. 이를 법선 정보를 이용하여 처리
 * details 
 */
void PCLPROCESSING::matchingSurfaceCheckAndICPVer2(CALIGN_PARAMETER &align_param, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_src, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_CAD, pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &md_KdTree, const Eigen::Matrix4f &T_in, const Eigen::Matrix4f &T_B2S, Eigen::Matrix4f &T_out, bool b_plot) {
    printf("\n--------------  Post processing - surface check and ICP!  -----------------\n");
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Reference_rgbn(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Reference_inner(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Reference_outer(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_target_rgbn(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_aligned_rgbn(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::copyPointCloud(cloud_CAD, *cloud_target_rgbn); // Reference model (CAD model)
    Eigen::Matrix4f T_now = Eigen::Matrix4f::Identity();

    //// initialize the source cloud
    pcl::copyPointCloud(cloud_src, *cloud_aligned_rgbn);
    pcl::transformPointCloudWithNormals(*cloud_aligned_rgbn, *cloud_aligned_rgbn, T_in);

    T_now = T_in;

    // Visualization - before post processing
    bool b_plot_tmp = false;
	if (b_plot_tmp)
	{
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::copyPointCloud(cloud_src, *cloud_source); // Measured model
        pcl::transformPointCloudWithNormals(*cloud_source, *cloud_source, T_in);
        
		pcl::visualization::PCLVisualizer viewer("raw data before post processing");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text
		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*cloud_target_rgbn, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_in_color_h(cloud_target_rgbn, 0, 0, 0);
		viewer.addPointCloud(cloud_target_rgbn, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(cloud_target_rgbn, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// // raw cloud is green
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(Registered, 10, 10, 255);
		// viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_align_color_h(cloud_source, 10, 10, 255);
		viewer.addPointCloud(cloud_source, cloud_align_color_h, "cloud_source", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		viewer.close();
	}
    
    // printf("***********************************************************\n");
    // printf("************************** CAD original ****************************\n");
    // printf("***********************************************************\n");
    // // Test
    // for (size_t i = 0; i < cloud_target_rgbn->size(); i++) {
    //     printf("%0.3f, %0.3f, %0.3f // ", cloud_target_rgbn->points[i].x, cloud_target_rgbn->points[i].y, cloud_target_rgbn->points[i].z);
    // }
    // for (size_t i = 0; i < cloud_target_rgbn->size(); i++) {
    //     printf("%0.3f, %0.3f, %0.3f // ", cloud_target_rgbn->points[i].normal_x, cloud_target_rgbn->points[i].normal_y, cloud_target_rgbn->points[i].normal_z);
    // }
    // printf("***********************************************************\n");
    // printf("***********************************************************\n");
    // printf("***********************************************************\n");


    //// 1) Surface check using normal - 최종: 내부면을 제거하고, Euclidean filter를 통해 지배적인 면만 남긴 후, ICP 재수행
    ///////////////////////////////////////////////
    //// 방법 3 내부면 제거
    // Eigen::Vector3d com_pt = {-33.33, 33.33, 0.0};
    Eigen::Vector3d com_pt = {0.0, 0.0, 0.0};
    for (size_t i = 0; i < cloud_target_rgbn->size(); i++)
    {
        Eigen::Vector3d vec_inner_check;
        vec_inner_check[0] = cloud_target_rgbn->points[i].x - com_pt[0];
        vec_inner_check[1] = cloud_target_rgbn->points[i].y - com_pt[1];
        vec_inner_check[2] = cloud_target_rgbn->points[i].z - com_pt[2];
        vec_inner_check.normalize();

        Eigen::Vector3d query_point, vec_cad;
        m_math.cloud2ptNormal(*cloud_target_rgbn, i, query_point, vec_cad);

        if (m_math.dotCheck2(vec_inner_check, vec_cad, 90.0)) { Reference_outer->points.push_back(cloud_target_rgbn->points[i]); }
        else { Reference_inner->points.push_back(cloud_target_rgbn->points[i]); }
    }
    pcl::copyPointCloud(*Reference_outer, *Reference_rgbn);

    // euclidean cluster 중에 가장 점 개수가 많은 cloud

	////////////////////////////////////////////////////////////////////////////////
	// 2) precise alignment using ICP
	Eigen::Matrix4f T_icp;

    m_math.printCloudMinMax(*cloud_target_rgbn, "cloud_target_rgbn");
    m_math.printCloudMinMax(*cloud_aligned_rgbn, "cloud_aligned_rgbn");
    m_math.printCloudMinMax(*Reference_rgbn, "Reference_rgbn");

    matchingReg2CADUsingICP4NoEstNormal(align_param, *cloud_aligned_rgbn, *Reference_rgbn, T_icp, b_plot); // w/o normal est. --> 할당된 normal 사용
    T_now = T_icp*T_now;
    //// output
	T_out = T_now; // metric: mm

	// Visualization
	if (b_plot_tmp)
	{
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::transformPointCloud(cloud_src, *cloud_source, T_now);
		pcl::visualization::PCLVisualizer viewer("raw data after post processing");
		int v1(0);
		int v2(1);
		viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
		viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
		viewer.setBackgroundColor(1, 1, 1, v1);
		viewer.setBackgroundColor(1, 1, 1, v2);
		float txt_color = 0.0; // black text
		// Cal. centroid 
		Eigen::Vector4f centroid_CAD;
		pcl::compute3DCentroid(*Reference_rgbn, centroid_CAD);
		viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
		std::vector<pcl::visualization::Camera> cams;
		viewer.getCameras(cams);
		for (auto &&camera : cams)  { for(int i=0; i<3; i++) { camera.focal[i] = centroid_CAD[i]; } }
		viewer.setCameraParameters(cams[0]);

		// CAD cloud is black
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(Reference, 0, 0, 0);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_in_color_h(Reference_rgbn, 0, 0, 0);
		viewer.addPointCloud(Reference_rgbn, cloud_in_color_h, "cloud_in_v1", v1);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
		viewer.addPointCloud(Reference_rgbn, cloud_in_color_h, "cloud_in_v2", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

		// // raw cloud is green
		// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_raw_color_h(Registered, 10, 10, 255);
		// viewer.addPointCloud(Registered, cloud_raw_color_h, "cloud_raw_v1", v1);
		// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_raw_v1");

		// aligned point cloud is red
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_align_color_h(cloud_source, 10, 10, 255);
		viewer.addPointCloud(cloud_source, cloud_align_color_h, "cloud_source", v2);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_source");

		// Adding text descriptions in each viewport
		viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
		viewer.addText("Black: Original point cloud\nBlue: aligned point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
			boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		viewer.close();
	}
}



bool PCLPROCESSING::checkObjectCloudFlip(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_src, const Eigen::Vector3f &vec_reference, double max_cad_z, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out, Eigen::Vector3f &vec_flip_check, Eigen::Matrix4f &T_now)
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_flip_check(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::copyPointCloud(cloud_src, *cloud_flip_check);
    for (size_t i = 0; i < cloud_src.size(); i++) {
        if(!std::isnan(cloud_src.points[i].normal_x) && !std::isnan(cloud_src.points[i].normal_y) && !std::isnan(cloud_src.points[i].normal_z)) {
            Eigen::Vector3f vec_normal_tmp;
            vec_normal_tmp[0] = cloud_src.points[i].normal_x;
            vec_normal_tmp[1] = cloud_src.points[i].normal_y;
            vec_normal_tmp[2] = cloud_src.points[i].normal_z;

            vec_flip_check += vec_normal_tmp;
            vec_flip_check.normalize();
        } else {
            printf("Normal has NAN!\n");
        }
    }

    //// TODO: vec_flip_check: -0.000, -0.000, -0.000이 나오는 경우가 발생하는 데, 이를 방지하도록 처리하기
    printf("vec_flip_check(checkObjectCloudFlip): %0.3f, %0.3f, %0.3f\n", vec_flip_check[0], vec_flip_check[1], vec_flip_check[2]);

    if(!m_math.dotCheck2(vec_reference, vec_flip_check, 90.0)) { // 사잇각이 90deg보다 크면
        printf("\n-------------- Plane-shaped cloud post processing! 180 deg w.r.t {O} Flipped!  -----------------\n");
        Eigen::Matrix4f T_trans_flip_1 = Eigen::Matrix4f::Identity();
        // T_trans_flip_1(2, 3) = -target_object_data->cad_max_pt.z;
        T_trans_flip_1(2, 3) = -max_cad_z;
        
        // 1) 물체 좌표계의 중심으로 translation
        pcl::transformPointCloudWithNormals(*cloud_flip_check, *cloud_flip_check, T_trans_flip_1);
        // m_math.printCloudMinMax(*cloud_flip_check, "cloud_flip_check - 1");
        // 2) 물체 좌표계의 y축 기준으로 180deg 회전
        Eigen::Matrix3f R_rot_flip;
        Eigen::Matrix4f T_rot_flip;
        m_math.genRotM(180.0, R_rot_flip, T_rot_flip, 1);
        pcl::transformPointCloudWithNormals(*cloud_flip_check, *cloud_flip_check, T_rot_flip);
        // m_math.printCloudMinMax(*cloud_flip_check, "cloud_flip_check - 2");
        // 3) 원래 위치로 translation
        Eigen::Matrix4f T_trans_flip_2 = Eigen::Matrix4f::Identity();
        // T_trans_flip_2(2, 3) = target_object_data->cad_max_pt.z;
        T_trans_flip_2(2, 3) = max_cad_z;
        pcl::transformPointCloudWithNormals(*cloud_flip_check, *cloud_flip_check, T_trans_flip_2);
        // m_math.printCloudMinMax(*cloud_flip_check, "cloud_flip_check - 3");

        T_now = T_trans_flip_2*T_rot_flip*T_trans_flip_1*T_now;
        cloud_out = *cloud_flip_check;
        return true;
        // target_object_data->is_object_pose_flipped = true;

    } else {
        // target_object_data->is_object_pose_flipped = false;
        cloud_out = *cloud_flip_check;
        return false;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////// PLOT /////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** @brief Korean: 입력된 점군 데이터의 plot 함수
* @param[in] cloud_in : 입력 점군 데이터
*/
void PCLPROCESSING::plotCloud(pcl::PointCloud<pcl::PointXYZ> &cloud_in, float view_scale, std::string plot_name)
{
	pcl::visualization::PCLVisualizer viewer;
	viewer.setBackgroundColor(1, 1, 1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot(new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_plot = cloud_in;

    // Cal. centroid
    Eigen::Vector4f centroid_CAD;
    pcl::compute3DCentroid(*cloud_plot, centroid_CAD);
    viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
    std::vector<pcl::visualization::Camera> cams;
    viewer.getCameras(cams);
    for (auto &&camera : cams)
    {
        for (int i = 0; i < 3; i++)
        {
            camera.focal[i] = centroid_CAD[i];
        }
    }
    viewer.setCameraParameters(cams[0]);


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_plot, 0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud_plot, single_color, plot_name.c_str());
	float txt_gray_lvl = 0.0; // black text
	std::string name = plot_name;
	viewer.addText(plot_name.c_str(), 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, name);

	while (!viewer.wasStopped())
	{
        viewer.spin();
        // viewer.spinOnce(100);
	}
}

/** @brief Korean: 입력된 점군 데이터의 plot 함수
* @param[in] cloud_in : 입력 점군 데이터 [m]
*/
void PCLPROCESSING::plotCloudWithFrame(pcl::PointCloud<pcl::PointXYZ> &cloud_in, float view_scale, float line_length, std::string plot_name)
{
	pcl::visualization::PCLVisualizer viewer;
	viewer.setBackgroundColor(1, 1, 1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot(new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_plot = cloud_in;


    //// Orientation
    Eigen::Vector4f grp_p0(0.0, 0.0, 0.0, 1.0);
    Eigen::Vector4f grp_p_base = grp_p0;

    Eigen::Vector3f grp_rx0(1.0, 0.0, 0.0);
    Eigen::Vector3f grp_ry0(0.0, 1.0, 0.0);
    Eigen::Vector3f grp_rz0(0.0, 0.0, 1.0);
    Eigen::Vector3f grp_rx_base = grp_rx0;
    Eigen::Vector3f grp_ry_base = grp_ry0;
    Eigen::Vector3f grp_rz_base = grp_rz0;

    // float line_length = 0.075;

    Eigen::Vector3f grp_ppx = grp_p_base.block<3,1>(0,0) + line_length*grp_rx_base;
    Eigen::Vector3f grp_ppy = grp_p_base.block<3,1>(0,0) + line_length*grp_ry_base;
    Eigen::Vector3f grp_ppz = grp_p_base.block<3,1>(0,0) + line_length*grp_rz_base;

    // Cal. centroid
    Eigen::Vector4f centroid_CAD;
    pcl::compute3DCentroid(*cloud_plot, centroid_CAD);
    viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
    std::vector<pcl::visualization::Camera> cams;
    viewer.getCameras(cams);
    for (auto &&camera : cams)
    {
        for (int i = 0; i < 3; i++)
        {
            camera.focal[i] = centroid_CAD[i];
        }
    }
    viewer.setCameraParameters(cams[0]);


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_plot, 0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud_plot, single_color, plot_name.c_str());

    pcl::PointXYZ grp_ppt0(grp_p_base(0), grp_p_base(1), grp_p_base(2));
    pcl::PointXYZ grp_pptx(grp_ppx(0), grp_ppx(1), grp_ppx(2));
    pcl::PointXYZ grp_ppty(grp_ppy(0), grp_ppy(1), grp_ppy(2));
    pcl::PointXYZ grp_pptz(grp_ppz(0), grp_ppz(1), grp_ppz(2));
    viewer.addLine(grp_ppt0, grp_pptx, 1.0, 0.0, 0.0, "x");
    viewer.addLine(grp_ppt0, grp_ppty, 0.0, 1.0, 0.0, "y");
    viewer.addLine(grp_ppt0, grp_pptz, 0.0, 0.0, 1.0, "z");

	float txt_gray_lvl = 0.0; // black text
	std::string name = plot_name;
	viewer.addText(plot_name.c_str(), 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, name);

	while (!viewer.wasStopped())
	{
        viewer.spin();
        // viewer.spinOnce(100);
	}
}

/** @brief Korean: 입력된 점군 데이터의 plot 함수
* @param[in] cloud_in : 입력 점군 데이터 [m]
*/
void PCLPROCESSING::plotCloudWithFrame(pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, float view_scale, float line_length, std::string plot_name)
{
	pcl::visualization::PCLVisualizer viewer;
	viewer.setBackgroundColor(1, 1, 1);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plot(new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud_plot = cloud_in;


    //// Orientation
    Eigen::Vector4f grp_p0(0.0, 0.0, 0.0, 1.0);
    Eigen::Vector4f grp_p_base = grp_p0;

    Eigen::Vector3f grp_rx0(1.0, 0.0, 0.0);
    Eigen::Vector3f grp_ry0(0.0, 1.0, 0.0);
    Eigen::Vector3f grp_rz0(0.0, 0.0, 1.0);
    Eigen::Vector3f grp_rx_base = grp_rx0;
    Eigen::Vector3f grp_ry_base = grp_ry0;
    Eigen::Vector3f grp_rz_base = grp_rz0;

    // float line_length = 0.075;

    Eigen::Vector3f grp_ppx = grp_p_base.block<3,1>(0,0) + line_length*grp_rx_base;
    Eigen::Vector3f grp_ppy = grp_p_base.block<3,1>(0,0) + line_length*grp_ry_base;
    Eigen::Vector3f grp_ppz = grp_p_base.block<3,1>(0,0) + line_length*grp_rz_base;

    // Cal. centroid
    Eigen::Vector4f centroid_CAD;
    pcl::compute3DCentroid(*cloud_plot, centroid_CAD);
    viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
    std::vector<pcl::visualization::Camera> cams;
    viewer.getCameras(cams);
    for (auto &&camera : cams)
    {
        for (int i = 0; i < 3; i++)
        {
            camera.focal[i] = centroid_CAD[i];
        }
    }
    viewer.setCameraParameters(cams[0]);


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud_plot, 0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZRGB>(cloud_plot, single_color, plot_name.c_str());

    pcl::PointXYZ grp_ppt0(grp_p_base(0), grp_p_base(1), grp_p_base(2));
    pcl::PointXYZ grp_pptx(grp_ppx(0), grp_ppx(1), grp_ppx(2));
    pcl::PointXYZ grp_ppty(grp_ppy(0), grp_ppy(1), grp_ppy(2));
    pcl::PointXYZ grp_pptz(grp_ppz(0), grp_ppz(1), grp_ppz(2));
    viewer.addLine(grp_ppt0, grp_pptx, 1.0, 0.0, 0.0, "x");
    viewer.addLine(grp_ppt0, grp_ppty, 0.0, 1.0, 0.0, "y");
    viewer.addLine(grp_ppt0, grp_pptz, 0.0, 0.0, 1.0, "z");

	float txt_gray_lvl = 0.0; // black text
	std::string name = plot_name;
	viewer.addText(plot_name.c_str(), 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, name);

	while (!viewer.wasStopped())
	{
        viewer.spin();
        // viewer.spinOnce(100);
	}
}

/** @brief Korean: 입력된 점군 데이터의 plot 함수
* @param[in] cloud_in : 입력 점군 데이터 [m]
*/
void PCLPROCESSING::plotCloudWithNormal(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, float view_scale, std::string plot_name)
{
	pcl::visualization::PCLVisualizer viewer;
	viewer.setBackgroundColor(1, 1, 1);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_plot(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	*cloud_plot = cloud_in;

    //// Orientation
    Eigen::Vector4f grp_p0(0.0, 0.0, 0.0, 1.0);
    Eigen::Vector4f grp_p_base = grp_p0;

    Eigen::Vector3f grp_rx0(1.0, 0.0, 0.0);
    Eigen::Vector3f grp_ry0(0.0, 1.0, 0.0);
    Eigen::Vector3f grp_rz0(0.0, 0.0, 1.0);
    Eigen::Vector3f grp_rx_base = grp_rx0;
    Eigen::Vector3f grp_ry_base = grp_ry0;
    Eigen::Vector3f grp_rz_base = grp_rz0;

    float line_length = 75.0;
    Eigen::Vector3f grp_ppx = grp_p_base.block<3,1>(0,0) + line_length*grp_rx_base;
    Eigen::Vector3f grp_ppy = grp_p_base.block<3,1>(0,0) + line_length*grp_ry_base;
    Eigen::Vector3f grp_ppz = grp_p_base.block<3,1>(0,0) + line_length*grp_rz_base;

    // Cal. centroid
    Eigen::Vector4f centroid_CAD;
    pcl::compute3DCentroid(*cloud_plot, centroid_CAD);
    viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
    std::vector<pcl::visualization::Camera> cams;
    viewer.getCameras(cams);
    for (auto &&camera : cams)
    {
        for (int i = 0; i < 3; i++)
        {
            camera.focal[i] = centroid_CAD[i];
        }
    }
    viewer.setCameraParameters(cams[0]);


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> single_color(cloud_plot, 0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZRGBNormal>(cloud_plot, single_color, plot_name.c_str());

    pcl::PointXYZ grp_ppt0(grp_p_base(0), grp_p_base(1), grp_p_base(2));
    pcl::PointXYZ grp_pptx(grp_ppx(0), grp_ppx(1), grp_ppx(2));
    pcl::PointXYZ grp_ppty(grp_ppy(0), grp_ppy(1), grp_ppy(2));
    pcl::PointXYZ grp_pptz(grp_ppz(0), grp_ppz(1), grp_ppz(2));
    viewer.addLine(grp_ppt0, grp_pptx, 1.0, 0.0, 0.0, "x");
    viewer.addLine(grp_ppt0, grp_ppty, 0.0, 1.0, 0.0, "y");
    viewer.addLine(grp_ppt0, grp_pptz, 0.0, 0.0, 1.0, "z");

	float txt_gray_lvl = 0.0; // black text
	std::string name = plot_name;
	viewer.addText(plot_name.c_str(), 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, name);


    std::stringstream ss;
    float normal_line_length = 5.0;

    for(size_t i=0; i<cloud_plot->size(); i++)
    {
        ss.str("");
        ss << "n" << i+1;

        Eigen::Vector3f n_tmp = {cloud_plot->points[i].normal_x, cloud_plot->points[i].normal_y, cloud_plot->points[i].normal_z};
        Eigen::Vector3f n_end = cloud_plot->points[i].getVector3fMap() + normal_line_length*n_tmp;
        pcl::PointXYZ pt_end(n_end(0), n_end(1), n_end(2));
        viewer.addLine(cloud_plot->points[i], pt_end, 1.0, 1.0, 0.0, ss.str());
    }


	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
}

void PCLPROCESSING::plotRGBCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, float view_scale, double pt_size, std::string plot_name)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plot(new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud_plot = cloud_in;
    pcl::visualization::PCLVisualizer viewer("Query points - the centor of base frame");
    
    viewer.setBackgroundColor(1, 1, 1);
    // CAD cloud is black
    viewer.addPointCloud(cloud_plot, "cloud_in_v1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, pt_size, "cloud_in_v1");

    // Set camera position
    viewer.setCameraPosition(view_scale, view_scale, view_scale, 0, 0, 1);
    std::vector<pcl::visualization::Camera> cams;
    viewer.getCameras(cams);
    for (auto &&camera : cams)
    {
        for (int i = 0; i < 3; i++)
        {
            camera.focal[i] = 0.0;
        }
    }
    viewer.setCameraParameters(cams[0]);

    //// Bin frame

    //// Orientation
    Eigen::Vector3f grp_rx0(1.0, 0.0, 0.0);
    Eigen::Vector3f grp_ry0(0.0, 1.0, 0.0);
    Eigen::Vector3f grp_rz0(0.0, 0.0, 1.0);
    float line_length = 75.0;
    Eigen::Vector3f p0(0.0, 0.0, 0.0);
    Eigen::Vector3f ppx0 = p0 + line_length*grp_rx0;
    Eigen::Vector3f ppy0 = p0 + line_length*grp_ry0;
    Eigen::Vector3f ppz0 = p0 + line_length*grp_rz0;
    pcl::PointXYZ ppt0(p0(0), p0(1), p0(2));
    pcl::PointXYZ pptx(ppx0(0), ppx0(1), ppx0(2));
    pcl::PointXYZ ppty(ppy0(0), ppy0(1), ppy0(2));
    pcl::PointXYZ pptz(ppz0(0), ppz0(1), ppz0(2));
    viewer.addLine(ppt0, pptx, 1.0, 0.0, 0.0, "x0");
    viewer.addLine(ppt0, ppty, 0.0, 1.0, 0.0, "y0");
    viewer.addLine(ppt0, pptz, 0.0, 0.0, 1.0, "z0");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

/** @brief Korean: 입력된 점군 데이터의 plot 함수
* @param[in] cloud_ref : CAD 점군 데이터
* @param[in] cloud_meas_before : 프로세싱 전 측정 데이터
* @param[in] cloud_meas_after : 프로세싱 후 측정 데이터
*/
void PCLPROCESSING::plotProcessedCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_ref, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_meas_before, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_meas_after, const std::string &fig_title)
{
    const double visualize_pt_size_1 = 2.5; // reference cloud
    const double visualize_pt_size_2 = 5.0; // measured cloud
    pcl::visualization::PCLVisualizer viewer(fig_title.c_str());
    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer.setBackgroundColor(1, 1, 1, v1);
    viewer.setBackgroundColor(1, 1, 1, v2);
    float txt_color = 0.0; // black text
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_plot_ref(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_plot_meas_before(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_plot_meas_after(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    *cloud_plot_ref = cloud_ref;
    *cloud_plot_meas_before = cloud_meas_before;
    *cloud_plot_meas_after = cloud_meas_after;

    // Cal. centroid
    Eigen::Vector4f centroid_CAD;
    pcl::compute3DCentroid(*cloud_plot_ref, centroid_CAD);
    viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + 500.0, centroid_CAD[2] + 500.0, 0, 0, 1);
    std::vector<pcl::visualization::Camera> cams;
    viewer.getCameras(cams);
    for (auto &&camera : cams)
    {
        for (int i = 0; i < 3; i++)
        {
            camera.focal[i] = centroid_CAD[i];
        }
    }
    viewer.setCameraParameters(cams[0]);

    // Reference cloud is black
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_ref_color(cloud_plot_ref, 0, 0, 0);
    viewer.addPointCloud(cloud_plot_ref, cloud_ref_color, "cloud_ref_v1", v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_ref_v1");
    viewer.addPointCloud(cloud_plot_ref, cloud_ref_color, "cloud_ref_v2", v2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_ref_v2");

    // Measured cloud
    // (Before processing) cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_meas_color_before(cloud_plot_meas_before, 10, 255, 10);
    viewer.addPointCloud(cloud_plot_meas_before, cloud_meas_color_before, "cloud_meas_v1", v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_meas_v1");

    // (After processing) cloud is blue
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_meas_color_after(cloud_plot_meas_after, 10, 10, 255);
    viewer.addPointCloud(cloud_plot_meas_after, cloud_meas_color_after, "cloud_meas_v2", v2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_meas_v2");

    // Adding text descriptions in each viewport
    viewer.addText("Black: Original point cloud\nGreen: before processing", 10, 15, 16, txt_color, txt_color, txt_color, "Before", v1);
    viewer.addText("Black: Original point cloud\nBlue: after processing", 10, 15, 16, txt_color, txt_color, txt_color, "After", v2);

    while (!viewer.wasStopped())
    {
        // viewer.spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        viewer.spin();
    }
    viewer.close();
}

void PCLPROCESSING::plotCloudCompare(const pcl::PointCloud<pcl::PointXYZ> &cloud_in1, const pcl::PointCloud<pcl::PointXYZ> &cloud_in2, const pcl::PointCloud<pcl::PointXYZ> &cloud_in3, float view_scale, std::string plot_name)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ref(new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_plot1 = cloud_in1;
	*cloud_plot2 = cloud_in2;
    *cloud_ref = cloud_in3;

    pcl::visualization::PCLVisualizer viewer("Cloud compare");
    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    float bckgr_gray_level = 1.0; // white
    float txt_gray_lvl = 0.0;	  // black text
    double visualize_pt_size_1 = 2.5; // reference cloud
    double visualize_pt_size_2 = 5.0; // measured cloud

    // Original point cloud is black
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud_ref, 0, 0, 0);
    viewer.addPointCloud(cloud_ref, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
    viewer.addPointCloud(cloud_ref, cloud_in_color_h, "cloud_in_v2", v2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_tr_color_h(cloud_plot1, 10, 10, 255);
    viewer.addPointCloud(cloud_plot1, cloud_tr_color_h, "cloud_tr_v1", v1);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_tr_v1");

    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_icp_color_h(cloud_plot2, 10, 10, 255);
    viewer.addPointCloud(cloud_plot2, cloud_icp_color_h, "cloud_icp_v2", v2);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_icp_v2");

    // Adding text descriptions in each viewport
    viewer.addText("Black: Original point cloud\nBlue: Processed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText("Black: Original point cloud\nBlue: Processed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    std::string txt_v1 = "Quick align";
    std::string txt_v2 = "Quick align & ICP";
    viewer.addText(txt_v1, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "txt_v1", v1);
    viewer.addText(txt_v2, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "txt_v2", v2);
    viewer.setBackgroundColor(1, 1, 1, v1);
    viewer.setBackgroundColor(1, 1, 1, v2);



    // Set camera position
    viewer.setCameraPosition(0.0 + view_scale, 0.0 + view_scale, 0.0 + view_scale + 0.5, 0, 0, 1);
    std::vector<pcl::visualization::Camera> cams;
    viewer.getCameras(cams);
    for (auto &&camera : cams)
    {
        for (int i = 0; i < 3; i++)
        {
            camera.focal[i] = 0.0;
        }
    }
    viewer.setCameraParameters(cams[0]);

    // while (!viewer.wasStopped())
    // {
    // 	// viewer.spinOnce();
    // 	viewer.spinOnce(100);
    // 	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    // }
    viewer.spin();
    viewer.close();
}

/** @brief Korean: 
 */
bool PCLPROCESSING::plotEnvironmentWithRobotBase(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const Eigen::Matrix4f &T_base, std::vector<Eigen::Vector3f> &robot_joint_set)
{
    //// Gripper CAD idx
    for (size_t i = 0; i < target_object_data->set_gripper_CAD_open_length.size(); i++)
    {
        if(target_object_data->set_gripper_CAD_open_length[i] == target_object_data->gripper_open_length)
        {
            target_object_data->gripper_idx_now = i;
            break;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Scanned cloud merged with bin(CAD) for collision check
    // 그리퍼 CAD 좌표변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gripper_CAD_gripper_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gripper_CAD_base_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_gripper_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(target_object_data->set_cloud_gripper_CAD_uniformly_downsampled[target_object_data->gripper_idx_now], *cloud_gripper_CAD_gripper_frame);
    Eigen::Matrix4f T_B2E = T_tgt;
    // Eigen::Matrix4f T_B2E = T_base*T_tgt;
    pcl::transformPointCloud(target_object_data->set_cloud_gripper_CAD_uniformly_downsampled[target_object_data->gripper_idx_now], *cloud_gripper_CAD_base_frame, T_B2E); // transform the scanned cloud from base to object frame
    pcl::copyPointCloud(*cloud_gripper_CAD_base_frame, *cloud_gripper_CAD_rgb);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Base frame transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_base_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_env_base_frame = *target_object_data->cloud_scan_base_aligned_with_bin_collision_check_process;
    pcl::transformPointCloud(*cloud_env_base_frame, *cloud_env_base_frame, T_base);
    pcl::copyPointCloud(*cloud_env_base_frame, *cloud_env_base_frame_rgb);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::vector<size_t> gripper_tip_idx = target_object_data->set_gripper_tip_surface_idx[target_object_data->gripper_idx_now];
    for (size_t i = 0; i < cloud_gripper_CAD_rgb->size(); i++)
    {
        cloud_gripper_CAD_rgb->points[i].r = 1;
        cloud_gripper_CAD_rgb->points[i].g = 255;
        cloud_gripper_CAD_rgb->points[i].b = 1;
    }

    for (size_t i = 0; i < gripper_tip_idx.size(); i++)
    {
        size_t idx_tmp = gripper_tip_idx[i]; 
        cloud_gripper_CAD_rgb->points[idx_tmp].r = 255;
        cloud_gripper_CAD_rgb->points[idx_tmp].g = 1;
        cloud_gripper_CAD_rgb->points[idx_tmp].b = 255;
    }

    if (1)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb_with_gripper(new pcl::PointCloud<pcl::PointXYZRGB>);

        double visualize_pt_size_1 = 2.5;
        double visualize_pt_size_2 = 2.5;
        pcl::visualization::PCLVisualizer viewer("plotEnvironmentWithRobotBase");
        
        m_math.cloudScaling(*cloud_gripper_CAD_rgb, 0); // mm to m
        m_math.cloudScaling(*cloud_env_base_frame_rgb, 0); // mm to m
        *cloud_env_base_frame_rgb_with_gripper = *cloud_env_base_frame_rgb;
        *cloud_env_base_frame_rgb_with_gripper += *cloud_gripper_CAD_rgb;

        //// Object CAD cloud

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_object_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::copyPointCloud(*target_object_data->cloud_CAD_uniformly_downsampled, *cloud_object_CAD_rgb);
        Eigen::Matrix4f T_B2O = target_object_data->T_matching_B2O;
        pcl::transformPointCloud(*cloud_object_CAD_rgb, *cloud_object_CAD_rgb, T_B2O);
        pcl::transformPointCloud(*cloud_object_CAD_rgb, *cloud_object_CAD_rgb, T_base);
        for (size_t i = 0; i < cloud_object_CAD_rgb->size(); i++)
        {
            cloud_object_CAD_rgb->points[i].r = 1;
            cloud_object_CAD_rgb->points[i].g = 1;
            cloud_object_CAD_rgb->points[i].b = 255;
        }

        m_math.cloudScaling(*cloud_object_CAD_rgb, 0); // mm to m
        *cloud_env_base_frame_rgb_with_gripper += *cloud_object_CAD_rgb;

        int v1(0);
        int v2(1);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(1, 1, 1, v1);
        viewer.setBackgroundColor(1, 1, 1, v2);


        // CAD cloud is black
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_in_color_h(cloud_env_base_frame_rgb, 0, 0, 0);
        viewer.addPointCloud(cloud_env_base_frame_rgb, "cloud_in_v1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
        viewer.addPointCloud(cloud_env_base_frame_rgb_with_gripper, "cloud_in_v2", v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

        //// Orientation
        Eigen::Vector4f grp_p0(0.0, 0.0, 0.0, 1.0);
        Eigen::Vector4f grp_p_base = T_B2E*grp_p0;

        Eigen::Vector3f grp_rx0(1.0, 0.0, 0.0);
        Eigen::Vector3f grp_ry0(0.0, 1.0, 0.0);
        Eigen::Vector3f grp_rz0(0.0, 0.0, 1.0);
        Eigen::Vector3f grp_rx_base = T_B2E.block<3,3>(0,0)*grp_rx0;
        Eigen::Vector3f grp_ry_base = T_B2E.block<3,3>(0,0)*grp_ry0;
        Eigen::Vector3f grp_rz_base = T_B2E.block<3,3>(0,0)*grp_rz0;

        float line_length = 75.0;
        Eigen::Vector3f grp_ppx = grp_p_base.block<3,1>(0,0) + line_length*grp_rx_base;
        Eigen::Vector3f grp_ppy = grp_p_base.block<3,1>(0,0) + line_length*grp_ry_base;
        Eigen::Vector3f grp_ppz = grp_p_base.block<3,1>(0,0) + line_length*grp_rz_base;

        // std::cout << "grp_p_base" << std::endl;
        // std::cout << grp_p_base << std::endl;
        // std::cout << "ppx ppy ppz" << std::endl;
        // std::cout << ppx << std::endl;
        // std::cout << ppy << std::endl;
        // std::cout << ppz << std::endl;

        pcl::PointXYZ grp_ppt0(0.001*grp_p_base(0), 0.001*grp_p_base(1), 0.001*grp_p_base(2));
        pcl::PointXYZ grp_pptx(0.001*grp_ppx(0), 0.001*grp_ppx(1), 0.001*grp_ppx(2));
        pcl::PointXYZ grp_ppty(0.001*grp_ppy(0), 0.001*grp_ppy(1), 0.001*grp_ppy(2));
        pcl::PointXYZ grp_pptz(0.001*grp_ppz(0), 0.001*grp_ppz(1), 0.001*grp_ppz(2));
        viewer.addLine(grp_ppt0, grp_pptx, 1.0, 0.0, 0.0, "x");
        viewer.addLine(grp_ppt0, grp_ppty, 0.0, 1.0, 0.0, "y");
        viewer.addLine(grp_ppt0, grp_pptz, 0.0, 0.0, 1.0, "z");

        // Set camera position
        float view_scale = 1.0;
        viewer.setCameraPosition(0.001*grp_p_base[0] + view_scale, 0.001*grp_p_base[1] + view_scale, 0.001*grp_p_base[2] + view_scale + 0.5, 0, 0, 1);
        std::vector<pcl::visualization::Camera> cams;
        viewer.getCameras(cams);
        for (auto &&camera : cams)
        {
            for (int i = 0; i < 3; i++)
            {
                camera.focal[i] = 0.001*grp_p_base[i];
            }
        }
        viewer.setCameraParameters(cams[0]);

        //// Base frame
        Eigen::Vector3f p0(0.0, 0.0, 0.0);
        Eigen::Vector3f ppx0 = p0 + line_length*grp_rx0;
        Eigen::Vector3f ppy0 = p0 + line_length*grp_ry0;
        Eigen::Vector3f ppz0 = p0 + line_length*grp_rz0;
        pcl::PointXYZ ppt0(0.001*p0(0), 0.001*p0(1), 0.001*p0(2));
        pcl::PointXYZ pptx(0.001*ppx0(0), 0.001*ppx0(1), 0.001*ppx0(2));
        pcl::PointXYZ ppty(0.001*ppy0(0), 0.001*ppy0(1), 0.001*ppy0(2));
        pcl::PointXYZ pptz(0.001*ppz0(0), 0.001*ppz0(1), 0.001*ppz0(2));
        viewer.addLine(ppt0, pptx, 1.0, 0.0, 0.0, "x0");
        viewer.addLine(ppt0, ppty, 0.0, 1.0, 0.0, "y0");
        viewer.addLine(ppt0, pptz, 0.0, 0.0, 1.0, "z0");


        //// Robot configuration
        // metric: [m]
        Eigen::Vector3f joint_pt_s = p0;
        std::stringstream joint_name;
        Eigen::MatrixXf color_map_tmp(7,3);
        color_map_tmp(0,0) = 0.0; color_map_tmp(0,1) = 0.5; color_map_tmp(0,2) = 1.0; 
        color_map_tmp(1,0) = 1.0; color_map_tmp(1,1) = 0.5; color_map_tmp(1,2) = 0.0; 
        color_map_tmp(2,0) = 0.3; color_map_tmp(2,1) = 1.0; color_map_tmp(2,2) = 0.2; 
        color_map_tmp(3,0) = 1.0; color_map_tmp(3,1) = 0.0; color_map_tmp(3,2) = 1.0; 
        color_map_tmp(4,0) = 0.0; color_map_tmp(4,1) = 1.0; color_map_tmp(4,2) = 1.0; 
        color_map_tmp(5,0) = 0.0; color_map_tmp(5,1) = 0.8; color_map_tmp(5,2) = 1.0; 
        color_map_tmp(6,0) = 1.0; color_map_tmp(6,1) = 0.2; color_map_tmp(6,2) = 0.0; 

        for (size_t i = 0; i < robot_joint_set.size(); i++)
        {
			joint_name.str("");
			joint_name << "Joint " << i + 1;
            Eigen::Vector3f joint_pt_e = robot_joint_set[i];
            Eigen::Vector3f joint_rot_vec = joint_pt_e - joint_pt_s;
            float joint_length = joint_rot_vec.norm();
          

			// Create colour handle
            pcl::PointXYZ ppt_s(joint_pt_s(0), joint_pt_s(1), joint_pt_s(2));
            pcl::PointXYZ ppt_e(joint_pt_e(0), joint_pt_e(1), joint_pt_e(2));
            viewer.addLine(ppt_s, ppt_e, color_map_tmp(i,0), color_map_tmp(i,1), color_map_tmp(i,2), joint_name.str());

            joint_pt_s = robot_joint_set[i];
        }
        


        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    return true;
}

/** @brief Korean: 
 */
bool PCLPROCESSING::plotEnvironmentWithRobotBase2(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_base, std::vector<Eigen::Vector3f> &robot_joint_set, const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in)
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Base frame transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_base_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_env_base_frame = *target_object_data->cloud_scan_base_aligned_with_bin_collision_check_process;
    // *cloud_env_base_frame = *target_object_data->cloud_bin_CAD_uniformly_downsampled_bin_frame;

    pcl::transformPointCloud(*cloud_env_base_frame, *cloud_env_base_frame, T_base);
    pcl::copyPointCloud(*cloud_env_base_frame, *cloud_env_base_frame_rgb);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_query_base(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_query_base = cloud_in;

    if (1)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb_with_gripper(new pcl::PointCloud<pcl::PointXYZRGB>);

        double visualize_pt_size_1 = 2.5;
        double visualize_pt_size_2 = 10.0;
        pcl::visualization::PCLVisualizer viewer("plotEnvironmentWithRobotBase2");
        
        m_math.cloudScaling(*cloud_env_base_frame_rgb, 0); // mm to m
        m_math.cloudScaling(*cloud_query_base, 0); // mm to m
        *cloud_env_base_frame_rgb_with_gripper = *cloud_env_base_frame_rgb;

        int v1(0);
        int v2(1);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(1, 1, 1, v1);
        viewer.setBackgroundColor(1, 1, 1, v2);


        // CAD cloud is black
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_in_color_h(cloud_env_base_frame_rgb, 0, 0, 0);
        viewer.addPointCloud(cloud_env_base_frame_rgb, "cloud_in_v1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
        viewer.addPointCloud(cloud_env_base_frame_rgb_with_gripper, "cloud_in_v2", v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

        viewer.addPointCloud(cloud_query_base, "cloud_query_base_v2", v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_query_base_v2");

        //// Orientation
        Eigen::Vector4f grp_p0(0.0, 0.0, 0.0, 1.0);
        Eigen::Vector3f grp_rx0(1.0, 0.0, 0.0);
        Eigen::Vector3f grp_ry0(0.0, 1.0, 0.0);
        Eigen::Vector3f grp_rz0(0.0, 0.0, 1.0);


        // Set camera position
        float view_scale = 1.0;
        viewer.setCameraPosition(view_scale, view_scale, view_scale + 0.5, 0, 0, 1);
        std::vector<pcl::visualization::Camera> cams;
        viewer.getCameras(cams);
        for (auto &&camera : cams)
        {
            for (int i = 0; i < 3; i++)
            {
                camera.focal[i] = 0.0;
            }
        }
        viewer.setCameraParameters(cams[0]);

        float line_length = 75.0;
        //// Base frame
        Eigen::Vector3f p0(0.0, 0.0, 0.0);
        Eigen::Vector3f ppx0 = p0 + line_length*grp_rx0;
        Eigen::Vector3f ppy0 = p0 + line_length*grp_ry0;
        Eigen::Vector3f ppz0 = p0 + line_length*grp_rz0;
        pcl::PointXYZ ppt0(0.001*p0(0), 0.001*p0(1), 0.001*p0(2));
        pcl::PointXYZ pptx(0.001*ppx0(0), 0.001*ppx0(1), 0.001*ppx0(2));
        pcl::PointXYZ ppty(0.001*ppy0(0), 0.001*ppy0(1), 0.001*ppy0(2));
        pcl::PointXYZ pptz(0.001*ppz0(0), 0.001*ppz0(1), 0.001*ppz0(2));
        viewer.addLine(ppt0, pptx, 1.0, 0.0, 0.0, "x0");
        viewer.addLine(ppt0, ppty, 0.0, 1.0, 0.0, "y0");
        viewer.addLine(ppt0, pptz, 0.0, 0.0, 1.0, "z0");


        //// Robot configuration
        // metric: [m]
        Eigen::Vector3f joint_pt_s = p0;
        std::stringstream joint_name;
        Eigen::MatrixXf color_map_tmp(7,3);
        color_map_tmp(0,0) = 0.0; color_map_tmp(0,1) = 0.5; color_map_tmp(0,2) = 1.0; 
        color_map_tmp(1,0) = 1.0; color_map_tmp(1,1) = 0.5; color_map_tmp(1,2) = 0.0; 
        color_map_tmp(2,0) = 0.3; color_map_tmp(2,1) = 1.0; color_map_tmp(2,2) = 0.2; 
        color_map_tmp(3,0) = 1.0; color_map_tmp(3,1) = 0.0; color_map_tmp(3,2) = 1.0; 
        color_map_tmp(4,0) = 0.0; color_map_tmp(4,1) = 1.0; color_map_tmp(4,2) = 1.0; 
        color_map_tmp(5,0) = 0.0; color_map_tmp(5,1) = 0.8; color_map_tmp(5,2) = 1.0; 
        color_map_tmp(6,0) = 1.0; color_map_tmp(6,1) = 0.2; color_map_tmp(6,2) = 0.0; 

        for (size_t i = 0; i < robot_joint_set.size(); i++)
        {
			joint_name.str("");
			joint_name << "Joint " << i + 1;
            Eigen::Vector3f joint_pt_e = robot_joint_set[i];
            Eigen::Vector3f joint_rot_vec = joint_pt_e - joint_pt_s;
            float joint_length = joint_rot_vec.norm();
          

			// Create colour handle
            pcl::PointXYZ ppt_s(joint_pt_s(0), joint_pt_s(1), joint_pt_s(2));
            pcl::PointXYZ ppt_e(joint_pt_e(0), joint_pt_e(1), joint_pt_e(2));
            viewer.addLine(ppt_s, ppt_e, color_map_tmp(i,0), color_map_tmp(i,1), color_map_tmp(i,2), joint_name.str());

            joint_pt_s = robot_joint_set[i];
            // printf("joint_pt_e: %0.5f, %0.5f, %0.5f\n", joint_pt_e[0], joint_pt_e[1], joint_pt_e[2]);
        }

        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    return true;
}

/** @brief Korean: 
 */
bool PCLPROCESSING::plotEnvironmentWithRobotBase3(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const Eigen::Matrix4f &T_base, std::vector<Eigen::Vector3f> &robot_joint_set, const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, const Eigen::Vector4f &bin_center_wrt_optimal_base_frame)
{
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Base frame transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_base_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_env_base_frame = *target_object_data->cloud_scan_base_aligned_with_bin_collision_check_process;
    // *cloud_env_base_frame = *target_object_data->cloud_bin_CAD_uniformly_downsampled_bin_frame;

    pcl::transformPointCloud(*cloud_env_base_frame, *cloud_env_base_frame, T_base);
    pcl::copyPointCloud(*cloud_env_base_frame, *cloud_env_base_frame_rgb);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_query_base(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_query_base = cloud_in;

    if (1)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb_with_gripper(new pcl::PointCloud<pcl::PointXYZRGB>);

        double visualize_pt_size_1 = 2.5;
        double visualize_pt_size_2 = 10.0;
        pcl::visualization::PCLVisualizer viewer("plotEnvironmentWithRobotBase3");
        
        m_math.cloudScaling(*cloud_env_base_frame_rgb, 0); // mm to m
        m_math.cloudScaling(*cloud_query_base, 0); // mm to m
        *cloud_env_base_frame_rgb_with_gripper = *cloud_env_base_frame_rgb;

        int v1(0);
        int v2(1);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(1, 1, 1, v1);
        viewer.setBackgroundColor(1, 1, 1, v2);


        // CAD cloud is black
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_in_color_h(cloud_env_base_frame_rgb, 0, 0, 0);
        viewer.addPointCloud(cloud_env_base_frame_rgb, "cloud_in_v1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
        viewer.addPointCloud(cloud_env_base_frame_rgb_with_gripper, "cloud_in_v2", v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

        viewer.addPointCloud(cloud_query_base, "cloud_query_base_v2", v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_2, "cloud_query_base_v2");

        //// Orientation
        Eigen::Vector4f grp_p0(0.0, 0.0, 0.0, 1.0);
        Eigen::Vector3f grp_rx0(1.0, 0.0, 0.0);
        Eigen::Vector3f grp_ry0(0.0, 1.0, 0.0);
        Eigen::Vector3f grp_rz0(0.0, 0.0, 1.0);


        // Set camera position
        float view_scale = 1.0;
        viewer.setCameraPosition(view_scale, view_scale, view_scale + 0.5, 0, 0, 1);
        std::vector<pcl::visualization::Camera> cams;
        viewer.getCameras(cams);
        for (auto &&camera : cams)
        {
            for (int i = 0; i < 3; i++)
            {
                camera.focal[i] = 0.0;
            }
        }
        viewer.setCameraParameters(cams[0]);

        float line_length = 75.0;
        //// Base frame
        Eigen::Vector3f p0(0.0, 0.0, 0.0);
        Eigen::Vector3f ppx0 = p0 + line_length*grp_rx0;
        Eigen::Vector3f ppy0 = p0 + line_length*grp_ry0;
        Eigen::Vector3f ppz0 = p0 + line_length*grp_rz0;
        pcl::PointXYZ ppt0(0.001*p0(0), 0.001*p0(1), 0.001*p0(2));
        pcl::PointXYZ pptx(0.001*ppx0(0), 0.001*ppx0(1), 0.001*ppx0(2));
        pcl::PointXYZ ppty(0.001*ppy0(0), 0.001*ppy0(1), 0.001*ppy0(2));
        pcl::PointXYZ pptz(0.001*ppz0(0), 0.001*ppz0(1), 0.001*ppz0(2));
        viewer.addLine(ppt0, pptx, 1.0, 0.0, 0.0, "x0");
        viewer.addLine(ppt0, ppty, 0.0, 1.0, 0.0, "y0");
        viewer.addLine(ppt0, pptz, 0.0, 0.0, 1.0, "z0");


        //// Robot configuration
        // metric: [m]
        Eigen::Vector3f joint_pt_s = p0;
        std::stringstream joint_name;
        Eigen::MatrixXf color_map_tmp(7,3);
        color_map_tmp(0,0) = 0.0; color_map_tmp(0,1) = 0.5; color_map_tmp(0,2) = 1.0; 
        color_map_tmp(1,0) = 1.0; color_map_tmp(1,1) = 0.5; color_map_tmp(1,2) = 0.0; 
        color_map_tmp(2,0) = 0.3; color_map_tmp(2,1) = 1.0; color_map_tmp(2,2) = 0.2; 
        color_map_tmp(3,0) = 1.0; color_map_tmp(3,1) = 0.0; color_map_tmp(3,2) = 1.0; 
        color_map_tmp(4,0) = 0.0; color_map_tmp(4,1) = 1.0; color_map_tmp(4,2) = 1.0; 
        color_map_tmp(5,0) = 0.0; color_map_tmp(5,1) = 0.8; color_map_tmp(5,2) = 1.0; 
        color_map_tmp(6,0) = 1.0; color_map_tmp(6,1) = 0.2; color_map_tmp(6,2) = 0.0; 

        for (size_t i = 0; i < robot_joint_set.size(); i++)
        {
			joint_name.str("");
			joint_name << "Joint " << i + 1;
            Eigen::Vector3f joint_pt_e = robot_joint_set[i];
            Eigen::Vector3f joint_rot_vec = joint_pt_e - joint_pt_s;
            float joint_length = joint_rot_vec.norm();
          
			// Create colour handle
            pcl::PointXYZ ppt_s(joint_pt_s(0), joint_pt_s(1), joint_pt_s(2));
            pcl::PointXYZ ppt_e(joint_pt_e(0), joint_pt_e(1), joint_pt_e(2));
            viewer.addLine(ppt_s, ppt_e, color_map_tmp(i,0), color_map_tmp(i,1), color_map_tmp(i,2), joint_name.str());

            joint_pt_s = robot_joint_set[i];
            // printf("joint_pt_e: %0.5f, %0.5f, %0.5f\n", joint_pt_e[0], joint_pt_e[1], joint_pt_e[2]);
        }

        // Adding text descriptions in each viewport
        float txt_gray_lvl = 0.0; // black text
        std::stringstream ss;
        ss << "Bin position from the origin of the optimal base frame [mm]: " << bin_center_wrt_optimal_base_frame[0] << ", " << bin_center_wrt_optimal_base_frame[1] << ", " << bin_center_wrt_optimal_base_frame[2];
        viewer.addText(ss.str(), 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "info_1", v1);
        viewer.addText("Blue: optimal\nPupple: sub-optimal\nYellow: feasible\nRed: infeasible", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "info_2", v2);




        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    return true;
}

/** @brief Korean: 
 */
bool PCLPROCESSING::plotCalibrationPoseWithBin(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const Eigen::Matrix4f &T_base, std::vector<Eigen::Vector3f> &robot_joint_set, bool b_plot)
{
    //// Gripper CAD idx
    for (size_t i = 0; i < target_object_data->set_gripper_CAD_open_length.size(); i++)
    {
        if(target_object_data->set_gripper_CAD_open_length[i] == target_object_data->gripper_open_length)
        {
            target_object_data->gripper_idx_now = i;
            break;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Scanned cloud merged with bin(CAD) for collision check
    // 그리퍼 CAD 좌표변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gripper_CAD_gripper_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gripper_CAD_base_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_gripper_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(target_object_data->set_cloud_gripper_CAD_uniformly_downsampled[target_object_data->gripper_idx_now], *cloud_gripper_CAD_gripper_frame);
    Eigen::Matrix4f T_B2E = T_tgt;
    // pcl::transformPointCloud(target_object_data->set_cloud_gripper_CAD_uniformly_downsampled[target_object_data->gripper_idx_now], *cloud_gripper_CAD_base_frame, T_B2E); // transform the scanned cloud from base to object frame
    pcl::transformPointCloud(*target_object_data->cloud_calibration_tool_CAD_uniformly_downsampled_object_frame, *cloud_gripper_CAD_base_frame, T_B2E); // transform the scanned cloud from base to object frame
    pcl::copyPointCloud(*cloud_gripper_CAD_base_frame, *cloud_gripper_CAD_rgb);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Base frame transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_base_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_env_base_frame = *target_object_data->cloud_bin_CAD_uniformly_downsampled_bin_frame_raw;
    pcl::transformPointCloud(*cloud_env_base_frame, *cloud_env_base_frame, T_base);
    pcl::copyPointCloud(*cloud_env_base_frame, *cloud_env_base_frame_rgb);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::vector<size_t> gripper_tip_idx = target_object_data->set_gripper_tip_surface_idx[target_object_data->gripper_idx_now];
    for (size_t i = 0; i < cloud_gripper_CAD_rgb->size(); i++)
    {
        cloud_gripper_CAD_rgb->points[i].r = 1;
        cloud_gripper_CAD_rgb->points[i].g = 255;
        cloud_gripper_CAD_rgb->points[i].b = 1;
    }

    for (size_t i = 0; i < gripper_tip_idx.size(); i++)
    {
        size_t idx_tmp = gripper_tip_idx[i]; 
        cloud_gripper_CAD_rgb->points[idx_tmp].r = 255;
        cloud_gripper_CAD_rgb->points[idx_tmp].g = 1;
        cloud_gripper_CAD_rgb->points[idx_tmp].b = 255;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb_with_gripper(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_env_base_frame_rgb_with_gripper = *cloud_env_base_frame_rgb;
    *cloud_env_base_frame_rgb_with_gripper += *cloud_gripper_CAD_rgb;
    pcl::copyPointCloud(*cloud_env_base_frame_rgb_with_gripper, *target_object_data->cloud_final_results_for_collision_check);

    if (b_plot)
    {
        double visualize_pt_size_1 = 2.5;
        double visualize_pt_size_2 = 2.5;
        pcl::visualization::PCLVisualizer viewer("plotCalibrationPoseWithBin");
        
        m_math.cloudScaling(*cloud_env_base_frame_rgb_with_gripper, 0); // mm to m

        //// Object CAD cloud
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_object_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        // pcl::copyPointCloud(*target_object_data->cloud_CAD_uniformly_downsampled, *cloud_object_CAD_rgb);
        // Eigen::Matrix4f T_B2O = target_object_data->T_matching_B2O;
        // pcl::transformPointCloud(*cloud_object_CAD_rgb, *cloud_object_CAD_rgb, T_B2O);
        // pcl::transformPointCloud(*cloud_object_CAD_rgb, *cloud_object_CAD_rgb, T_base);
        // for (size_t i = 0; i < cloud_object_CAD_rgb->size(); i++)
        // {
        //     cloud_object_CAD_rgb->points[i].r = 1;
        //     cloud_object_CAD_rgb->points[i].g = 1;
        //     cloud_object_CAD_rgb->points[i].b = 255;
        // }

        // m_math.cloudScaling(*cloud_object_CAD_rgb, 0); // mm to m
        // *cloud_env_base_frame_rgb_with_gripper += *cloud_object_CAD_rgb;

        int v1(0);
        int v2(1);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(1, 1, 1, v1);
        viewer.setBackgroundColor(1, 1, 1, v2);


        // CAD cloud is black
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_in_color_h(cloud_env_base_frame_rgb, 0, 0, 0);
        viewer.addPointCloud(cloud_env_base_frame_rgb, "cloud_in_v1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
        viewer.addPointCloud(cloud_env_base_frame_rgb_with_gripper, "cloud_in_v2", v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

        //// Orientation
        Eigen::Vector4f grp_p0(0.0, 0.0, 0.0, 1.0);
        Eigen::Vector4f grp_p_base = T_B2E*grp_p0;

        Eigen::Vector3f grp_rx0(1.0, 0.0, 0.0);
        Eigen::Vector3f grp_ry0(0.0, 1.0, 0.0);
        Eigen::Vector3f grp_rz0(0.0, 0.0, 1.0);
        Eigen::Vector3f grp_rx_base = T_B2E.block<3,3>(0,0)*grp_rx0;
        Eigen::Vector3f grp_ry_base = T_B2E.block<3,3>(0,0)*grp_ry0;
        Eigen::Vector3f grp_rz_base = T_B2E.block<3,3>(0,0)*grp_rz0;

        float line_length = 75.0;
        Eigen::Vector3f grp_ppx = grp_p_base.block<3,1>(0,0) + line_length*grp_rx_base;
        Eigen::Vector3f grp_ppy = grp_p_base.block<3,1>(0,0) + line_length*grp_ry_base;
        Eigen::Vector3f grp_ppz = grp_p_base.block<3,1>(0,0) + line_length*grp_rz_base;

        // std::cout << "grp_p_base" << std::endl;
        // std::cout << grp_p_base << std::endl;
        // std::cout << "ppx ppy ppz" << std::endl;
        // std::cout << ppx << std::endl;
        // std::cout << ppy << std::endl;
        // std::cout << ppz << std::endl;

        pcl::PointXYZ grp_ppt0(0.001*grp_p_base(0), 0.001*grp_p_base(1), 0.001*grp_p_base(2));
        pcl::PointXYZ grp_pptx(0.001*grp_ppx(0), 0.001*grp_ppx(1), 0.001*grp_ppx(2));
        pcl::PointXYZ grp_ppty(0.001*grp_ppy(0), 0.001*grp_ppy(1), 0.001*grp_ppy(2));
        pcl::PointXYZ grp_pptz(0.001*grp_ppz(0), 0.001*grp_ppz(1), 0.001*grp_ppz(2));
        viewer.addLine(grp_ppt0, grp_pptx, 1.0, 0.0, 0.0, "x");
        viewer.addLine(grp_ppt0, grp_ppty, 0.0, 1.0, 0.0, "y");
        viewer.addLine(grp_ppt0, grp_pptz, 0.0, 0.0, 1.0, "z");

        // Set camera position
        float view_scale = 1.0;
        viewer.setCameraPosition(0.001*grp_p_base[0] + view_scale, 0.001*grp_p_base[1] + view_scale, 0.001*grp_p_base[2] + view_scale + 0.5, 0, 0, 1);
        std::vector<pcl::visualization::Camera> cams;
        viewer.getCameras(cams);
        for (auto &&camera : cams)
        {
            for (int i = 0; i < 3; i++)
            {
                camera.focal[i] = 0.001*grp_p_base[i];
            }
        }
        viewer.setCameraParameters(cams[0]);

        //// Base frame
        Eigen::Vector3f p0(0.0, 0.0, 0.0);
        Eigen::Vector3f ppx0 = p0 + line_length*grp_rx0;
        Eigen::Vector3f ppy0 = p0 + line_length*grp_ry0;
        Eigen::Vector3f ppz0 = p0 + line_length*grp_rz0;
        pcl::PointXYZ ppt0(0.001*p0(0), 0.001*p0(1), 0.001*p0(2));
        pcl::PointXYZ pptx(0.001*ppx0(0), 0.001*ppx0(1), 0.001*ppx0(2));
        pcl::PointXYZ ppty(0.001*ppy0(0), 0.001*ppy0(1), 0.001*ppy0(2));
        pcl::PointXYZ pptz(0.001*ppz0(0), 0.001*ppz0(1), 0.001*ppz0(2));
        viewer.addLine(ppt0, pptx, 1.0, 0.0, 0.0, "x0");
        viewer.addLine(ppt0, ppty, 0.0, 1.0, 0.0, "y0");
        viewer.addLine(ppt0, pptz, 0.0, 0.0, 1.0, "z0");


        //// Robot configuration
        // metric: [m]
        Eigen::Vector3f joint_pt_s = p0;
        std::stringstream joint_name;
        Eigen::MatrixXf color_map_tmp(7,3);
        color_map_tmp(0,0) = 0.0; color_map_tmp(0,1) = 0.5; color_map_tmp(0,2) = 1.0; 
        color_map_tmp(1,0) = 1.0; color_map_tmp(1,1) = 0.5; color_map_tmp(1,2) = 0.0; 
        color_map_tmp(2,0) = 0.3; color_map_tmp(2,1) = 1.0; color_map_tmp(2,2) = 0.2; 
        color_map_tmp(3,0) = 1.0; color_map_tmp(3,1) = 0.0; color_map_tmp(3,2) = 1.0; 
        color_map_tmp(4,0) = 0.0; color_map_tmp(4,1) = 1.0; color_map_tmp(4,2) = 1.0; 
        color_map_tmp(5,0) = 0.0; color_map_tmp(5,1) = 0.8; color_map_tmp(5,2) = 1.0; 
        color_map_tmp(6,0) = 1.0; color_map_tmp(6,1) = 0.2; color_map_tmp(6,2) = 0.0; 

        for (size_t i = 0; i < robot_joint_set.size(); i++)
        {
			joint_name.str("");
			joint_name << "Joint " << i + 1;
            Eigen::Vector3f joint_pt_e = robot_joint_set[i];
            Eigen::Vector3f joint_rot_vec = joint_pt_e - joint_pt_s;
            float joint_length = joint_rot_vec.norm();
          

			// Create colour handle
            pcl::PointXYZ ppt_s(joint_pt_s(0), joint_pt_s(1), joint_pt_s(2));
            pcl::PointXYZ ppt_e(joint_pt_e(0), joint_pt_e(1), joint_pt_e(2));
            viewer.addLine(ppt_s, ppt_e, color_map_tmp(i,0), color_map_tmp(i,1), color_map_tmp(i,2), joint_name.str());

            joint_pt_s = robot_joint_set[i];
        }
        


        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }

    return true;
}

/** @brief Korean: 
 */
bool PCLPROCESSING::plotFinalPoseResults(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const size_t &threshold, std::vector<size_t> &collision_count_set, bool b_plot)
{
    bool is_collision = false;
    size_t cnt_occupied = 0;
    size_t cnt_gripper_tip_left_occupied = 0;
    size_t cnt_gripper_tip_right_occupied = 0;

    //// Gripper CAD idx
    for (size_t i = 0; i < target_object_data->set_gripper_CAD_open_length.size(); i++)
    {
        if(target_object_data->set_gripper_CAD_open_length[i] == target_object_data->gripper_open_length)
        {
            target_object_data->gripper_idx_now = i;
            break;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Scanned cloud merged with bin(CAD) for collision check
    // 그리퍼 CAD 좌표변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gripper_CAD_gripper_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(target_object_data->set_cloud_gripper_CAD_uniformly_downsampled[target_object_data->gripper_idx_now], *cloud_gripper_CAD_gripper_frame);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gripper_CAD_base_frame(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f T_B2E = T_tgt;
    pcl::transformPointCloud(target_object_data->set_cloud_gripper_CAD_uniformly_downsampled[target_object_data->gripper_idx_now], *cloud_gripper_CAD_base_frame, T_B2E); // transform the scanned cloud from base to object frame

    // sensor_origin - measurement origin in global reference frame
    // {E} 기준의 그리퍼 CAD의 Octomap을 {B} 기준의 Octomap으로 변환하여 사용하기 위해 origin_pose 입력(T_B2E에 해당하는 pose)
    std::vector<double> origin_pose = m_math.htm2pose_mm2m(T_B2E, "origin_pose"); // [mm] to [m]
    for (int j = 0; j < 3; j++) 
    { 
        origin_pose[j] = 1000.0*origin_pose[j];
        origin_pose[j+3] = DEGTORAD*origin_pose[j+3];
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_base_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_gripper_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_env_base_frame = *target_object_data->cloud_scan_base_aligned_with_bin_collision_check_process;
    pcl::copyPointCloud(*cloud_env_base_frame, *cloud_env_base_frame_rgb);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_gripper_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_gripper_CAD_base_frame, *cloud_gripper_CAD_rgb);

    std::vector<size_t> gripper_tip_idx = target_object_data->set_gripper_tip_surface_idx[target_object_data->gripper_idx_now];
    for (size_t i = 0; i < cloud_gripper_CAD_rgb->size(); i++)
    {
        cloud_gripper_CAD_rgb->points[i].r = 1;
        cloud_gripper_CAD_rgb->points[i].g = 255;
        cloud_gripper_CAD_rgb->points[i].b = 1;
    }

    for (size_t i = 0; i < gripper_tip_idx.size(); i++)
    {
        size_t idx_tmp = gripper_tip_idx[i]; 
        cloud_gripper_CAD_rgb->points[idx_tmp].r = 255;
        cloud_gripper_CAD_rgb->points[idx_tmp].g = 1;
        cloud_gripper_CAD_rgb->points[idx_tmp].b = 255;
    }

    //// Check collision using voxel map w.r.t nearest points from the gripper CAD cloud
    // kdTree search -  center에 가장 가까운 CAD 점 선택
    Eigen::Matrix4f T_E2B = T_B2E.inverse();
    pcl::transformPointCloud(*cloud_env_base_frame, *cloud_env_gripper_frame, T_E2B);

    ////////////////////////////////////////////////////////////////////////////////////////////
    //// Check cloud occupancy
    octomap::MapCollection<octomap::MapNode<octomap::OcTree>> collection;
    octomap::MapNode<octomap::OcTree>* mn1 = new octomap::MapNode<octomap::OcTree>(target_object_data->m_set_OctoMap_gripper_CAD[target_object_data->gripper_idx_now], octomap::pose6d(origin_pose[0], origin_pose[1], origin_pose[2], origin_pose[3], origin_pose[4], origin_pose[5]));
    mn1->setId("CAD");
    collection.addNode(mn1);
    target_object_data->m_set_OctoMap_gripper_CAD[target_object_data->gripper_idx_now]->setOccupancyThres(0.6); // 점유 파라미터(thre_map) 이상이면 점유(occupied)로 판정
    // Occlusion check
    for (size_t i = 0; i < cloud_env_base_frame->size(); i++)
    {
        octomap::point3d q;
        q(0) = cloud_env_base_frame->points[i].x;
        q(1) = cloud_env_base_frame->points[i].y;
        q(2) = cloud_env_base_frame->points[i].z;
        if (collection.isOccupied(q)) 
        { 
            ////////////////////////////////////////////////
            //// 230214
            //// 인식된 CAD에 속한 점이 충돌로 감지된 경우는 제외하도록 처리
            //// CAD 점군으로부터 threshold 내의 존재하는 점을 제거
            // {O} 기준의 KDTree Model 이용
            if(1) 
            {
                Eigen::Matrix4f T_O2B = target_object_data->T_matching_O2B;
                // ROS_LOG_INFO("idx_now: %zu", idx_now);
                Eigen::Vector3d query_point = m_math.cloud2pt(*cloud_env_base_frame, i);
                Eigen::Vector4f query_tmp;
                for(int j=0; j<3; j++) { query_tmp[j] = query_point[j]; }
                query_tmp[3] = 1.0;
                query_tmp = T_O2B*query_tmp;
                Eigen::Vector3f query_wrt_object = query_tmp.block<3,1>(0,0);
                float thre_dist = 1.0; // [mm]
                std::vector<int> nearest_idx;
                std::vector<float> nearest_dists;
                // if(query_wrt_object.norm() > 10e3) { continue; }
                m_math.findKNearestIndex(target_object_data->md_KdTree_octoMap_CAD, query_wrt_object, nearest_idx, nearest_dists, 1);
                // 선택된 Mask에 근접한 점은 삭제 (thre_dist 이내의 점들을 제외)
                if(nearest_dists[0] < thre_dist) { continue; }
            }

            cloud_env_base_frame_rgb->points[i].r = 255;
            cloud_env_base_frame_rgb->points[i].g = 1;
            cloud_env_base_frame_rgb->points[i].b = 1;

            ////////////////////////////////////////////////

            // cloud_gripper_CAD_rgb->points[nearest_idx[0]].r = 255;
            // cloud_gripper_CAD_rgb->points[nearest_idx[0]].g = 1;
            // cloud_gripper_CAD_rgb->points[nearest_idx[0]].b = 1;

            // float grp_cad_pt_y = cloud_gripper_CAD_gripper_frame->points[nearest_idx[0]].y;
            // KDTree 사용하지 않기 위해, 점유로 판별된 그리퍼 좌표계 기준의 측정점을 이용(점유되었으므로, 그리퍼 근처 값을 가질 것)
            float grp_cad_pt_y = cloud_env_gripper_frame->points[i].y; // cloud_gripper_CAD_gripper_frame, pt_y

            // ROS_LOG_INFO("grp_cad_pt_y(occupied): %0.1f", grp_cad_pt_y);
            // ZIVID가 장착된 방향이 정면, 정면을 향하도록 바라봤을 때를 기준으로 좌(+y)우(-y)를 의미
            if(grp_cad_pt_y > 0.0) cnt_gripper_tip_left_occupied++; // left (+y)
            else cnt_gripper_tip_right_occupied++; // right (-y)
            // total
            cnt_occupied++;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////
    //// TODO: Collision Threshold 정의
    if(cnt_occupied > threshold) is_collision = true;
    else is_collision = false;
    collision_count_set.resize(3);
    collision_count_set[0] = cnt_occupied; // total
    collision_count_set[1] = cnt_gripper_tip_left_occupied; // left
    collision_count_set[2] = cnt_gripper_tip_right_occupied; // right

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb_with_gripper(new pcl::PointCloud<pcl::PointXYZRGB>);
    double visualize_pt_size_1 = 2.5;
    double visualize_pt_size_2 = 2.5;
    m_math.cloudScaling(*cloud_gripper_CAD_rgb, 0); // mm to m
    m_math.cloudScaling(*cloud_env_base_frame_rgb, 0); // mm to m

    *cloud_env_base_frame_rgb_with_gripper = *cloud_env_base_frame_rgb;
    *cloud_env_base_frame_rgb_with_gripper += *cloud_gripper_CAD_rgb;
    //// Object CAD cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_object_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*target_object_data->cloud_CAD_uniformly_downsampled, *cloud_object_CAD_rgb);
    Eigen::Matrix4f T_B2O = target_object_data->T_matching_B2O;
    pcl::transformPointCloud(*cloud_object_CAD_rgb, *cloud_object_CAD_rgb, T_B2O);
    for (size_t i = 0; i < cloud_object_CAD_rgb->size(); i++)
    {
        cloud_object_CAD_rgb->points[i].r = 1;
        cloud_object_CAD_rgb->points[i].g = 1;
        cloud_object_CAD_rgb->points[i].b = 255;
    }
    m_math.cloudScaling(*cloud_object_CAD_rgb, 0); // mm to m
    *cloud_env_base_frame_rgb_with_gripper += *cloud_object_CAD_rgb; // final results

    if (b_plot)
    {
        pcl::visualization::PCLVisualizer viewer("plotFinalPoseResults");
        int v1(0);
        int v2(1);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(1, 1, 1, v1);
        viewer.setBackgroundColor(1, 1, 1, v2);
        // CAD cloud is black
        viewer.addPointCloud(cloud_env_base_frame_rgb, "cloud_in_v1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
        viewer.addPointCloud(cloud_env_base_frame_rgb_with_gripper, "cloud_in_v2", v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

        //// Orientation
        Eigen::Vector4f grp_p0(0.0, 0.0, 0.0, 1.0);
        Eigen::Vector4f grp_p_base = T_B2E*grp_p0;

        Eigen::Vector3f grp_rx0(1.0, 0.0, 0.0);
        Eigen::Vector3f grp_ry0(0.0, 1.0, 0.0);
        Eigen::Vector3f grp_rz0(0.0, 0.0, 1.0);
        Eigen::Vector3f grp_rx_base = T_B2E.block<3,3>(0,0)*grp_rx0;
        Eigen::Vector3f grp_ry_base = T_B2E.block<3,3>(0,0)*grp_ry0;
        Eigen::Vector3f grp_rz_base = T_B2E.block<3,3>(0,0)*grp_rz0;

        float line_length = 75.0;
        Eigen::Vector3f grp_ppx = grp_p_base.block<3,1>(0,0) + line_length*grp_rx_base;
        Eigen::Vector3f grp_ppy = grp_p_base.block<3,1>(0,0) + line_length*grp_ry_base;
        Eigen::Vector3f grp_ppz = grp_p_base.block<3,1>(0,0) + line_length*grp_rz_base;

        // std::cout << "grp_p_base" << std::endl;
        // std::cout << grp_p_base << std::endl;
        // std::cout << "ppx ppy ppz" << std::endl;
        // std::cout << ppx << std::endl;
        // std::cout << ppy << std::endl;
        // std::cout << ppz << std::endl;

        pcl::PointXYZ grp_ppt0(0.001*grp_p_base(0), 0.001*grp_p_base(1), 0.001*grp_p_base(2));
        pcl::PointXYZ grp_pptx(0.001*grp_ppx(0), 0.001*grp_ppx(1), 0.001*grp_ppx(2));
        pcl::PointXYZ grp_ppty(0.001*grp_ppy(0), 0.001*grp_ppy(1), 0.001*grp_ppy(2));
        pcl::PointXYZ grp_pptz(0.001*grp_ppz(0), 0.001*grp_ppz(1), 0.001*grp_ppz(2));
        viewer.addLine(grp_ppt0, grp_pptx, 1.0, 0.0, 0.0, "x");
        viewer.addLine(grp_ppt0, grp_ppty, 0.0, 1.0, 0.0, "y");
        viewer.addLine(grp_ppt0, grp_pptz, 0.0, 0.0, 1.0, "z");

        // Set camera position
        float view_scale = 0.8;
        viewer.setCameraPosition(0.001*grp_p_base[0] + view_scale, 0.001*grp_p_base[1] + view_scale, 0.001*grp_p_base[2] + view_scale, 0, 0, 1);
        std::vector<pcl::visualization::Camera> cams;
        viewer.getCameras(cams);
        for (auto &&camera : cams)
        {
            for (int i = 0; i < 3; i++)
            {
                camera.focal[i] = 0.001*grp_p_base[i];
            }
        }
        viewer.setCameraParameters(cams[0]);

        //// Base frame
        Eigen::Vector3f p0(0.0, 0.0, 0.0);
        Eigen::Vector3f ppx0 = p0 + line_length*grp_rx0;
        Eigen::Vector3f ppy0 = p0 + line_length*grp_ry0;
        Eigen::Vector3f ppz0 = p0 + line_length*grp_rz0;
        pcl::PointXYZ ppt0(0.001*p0(0), 0.001*p0(1), 0.001*p0(2));
        pcl::PointXYZ pptx(0.001*ppx0(0), 0.001*ppx0(1), 0.001*ppx0(2));
        pcl::PointXYZ ppty(0.001*ppy0(0), 0.001*ppy0(1), 0.001*ppy0(2));
        pcl::PointXYZ pptz(0.001*ppz0(0), 0.001*ppz0(1), 0.001*ppz0(2));
        viewer.addLine(ppt0, pptx, 1.0, 0.0, 0.0, "x0");
        viewer.addLine(ppt0, ppty, 0.0, 1.0, 0.0, "y0");
        viewer.addLine(ppt0, pptz, 0.0, 0.0, 1.0, "z0");

        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }
    m_math.cloudScaling(*cloud_env_base_frame_rgb_with_gripper, 1); // m to mm
    *target_object_data->cloud_final_results_for_collision_check = *cloud_env_base_frame_rgb_with_gripper;
    return is_collision;
}



/** @brief Korean: 
 */
bool PCLPROCESSING::plotFinalPoseResultsMultiThreaded(CTARGET_OBJECT_DATA &target_object_data, const Eigen::Matrix4f &T_tgt, const size_t &threshold, std::vector<size_t> &collision_count_set, bool b_plot)
{
    bool is_collision = false;
    size_t cnt_occupied = 0;
    size_t cnt_gripper_tip_left_occupied = 0;
    size_t cnt_gripper_tip_right_occupied = 0;

    //// Gripper CAD idx
    for (size_t i = 0; i < target_object_data.set_gripper_CAD_open_length.size(); i++)
    {
        if(target_object_data.set_gripper_CAD_open_length[i] == target_object_data.gripper_open_length)
        {
            target_object_data.gripper_idx_now = i;
            break;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Scanned cloud merged with bin(CAD) for collision check
    // 그리퍼 CAD 좌표변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gripper_CAD_gripper_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(target_object_data.set_cloud_gripper_CAD_uniformly_downsampled[target_object_data.gripper_idx_now], *cloud_gripper_CAD_gripper_frame);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gripper_CAD_base_frame(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f T_B2E = T_tgt;
    pcl::transformPointCloud(target_object_data.set_cloud_gripper_CAD_uniformly_downsampled[target_object_data.gripper_idx_now], *cloud_gripper_CAD_base_frame, T_B2E); // transform the scanned cloud from base to object frame

    // sensor_origin - measurement origin in global reference frame
    // {E} 기준의 그리퍼 CAD의 Octomap을 {B} 기준의 Octomap으로 변환하여 사용하기 위해 origin_pose 입력(T_B2E에 해당하는 pose)
    std::vector<double> origin_pose = m_math.htm2pose_mm2m(T_B2E, "origin_pose"); // [mm] to [m]
    for (int j = 0; j < 3; j++) 
    { 
        origin_pose[j] = 1000.0*origin_pose[j];
        origin_pose[j+3] = DEGTORAD*origin_pose[j+3];
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_base_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_gripper_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_env_base_frame = *target_object_data.cloud_scan_base_aligned_with_bin_collision_check_process;
    pcl::copyPointCloud(*cloud_env_base_frame, *cloud_env_base_frame_rgb);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_gripper_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_gripper_CAD_base_frame, *cloud_gripper_CAD_rgb);

    std::vector<size_t> gripper_tip_idx = target_object_data.set_gripper_tip_surface_idx[target_object_data.gripper_idx_now];
    for (size_t i = 0; i < cloud_gripper_CAD_rgb->size(); i++)
    {
        cloud_gripper_CAD_rgb->points[i].r = 1;
        cloud_gripper_CAD_rgb->points[i].g = 255;
        cloud_gripper_CAD_rgb->points[i].b = 1;
    }

    for (size_t i = 0; i < gripper_tip_idx.size(); i++)
    {
        size_t idx_tmp = gripper_tip_idx[i]; 
        cloud_gripper_CAD_rgb->points[idx_tmp].r = 255;
        cloud_gripper_CAD_rgb->points[idx_tmp].g = 1;
        cloud_gripper_CAD_rgb->points[idx_tmp].b = 255;
    }

    //// Check collision using voxel map w.r.t nearest points from the gripper CAD cloud
    // kdTree search -  center에 가장 가까운 CAD 점 선택
    Eigen::Matrix4f T_E2B = T_B2E.inverse();
    pcl::transformPointCloud(*cloud_env_base_frame, *cloud_env_gripper_frame, T_E2B);

    ////////////////////////////////////////////////////////////////////////////////////////////
    //// Check cloud occupancy
    octomap::MapCollection<octomap::MapNode<octomap::OcTree>> collection;
    octomap::MapNode<octomap::OcTree>* mn1 = new octomap::MapNode<octomap::OcTree>(target_object_data.m_set_OctoMap_gripper_CAD[target_object_data.gripper_idx_now], octomap::pose6d(origin_pose[0], origin_pose[1], origin_pose[2], origin_pose[3], origin_pose[4], origin_pose[5]));
    mn1->setId("CAD");
    collection.addNode(mn1);
    target_object_data.m_set_OctoMap_gripper_CAD[target_object_data.gripper_idx_now]->setOccupancyThres(0.6); // 점유 파라미터(thre_map) 이상이면 점유(occupied)로 판정
    // Occlusion check
    for (size_t i = 0; i < cloud_env_base_frame->size(); i++)
    {
        octomap::point3d q;
        q(0) = cloud_env_base_frame->points[i].x;
        q(1) = cloud_env_base_frame->points[i].y;
        q(2) = cloud_env_base_frame->points[i].z;
        if (collection.isOccupied(q)) 
        { 
            ////////////////////////////////////////////////
            //// 230214
            //// 인식된 CAD에 속한 점이 충돌로 감지된 경우는 제외하도록 처리
            //// CAD 점군으로부터 threshold 내의 존재하는 점을 제거
            // {O} 기준의 KDTree Model 이용
            if(1) 
            {
                Eigen::Matrix4f T_O2B = target_object_data.T_matching_O2B;
                // ROS_LOG_INFO("idx_now: %zu", idx_now);
                Eigen::Vector3d query_point = m_math.cloud2pt(*cloud_env_base_frame, i);
                Eigen::Vector4f query_tmp;
                for(int j=0; j<3; j++) { query_tmp[j] = query_point[j]; }
                query_tmp[3] = 1.0;
                query_tmp = T_O2B*query_tmp;
                Eigen::Vector3f query_wrt_object = query_tmp.block<3,1>(0,0);
                float thre_dist = 1.0; // [mm]
                std::vector<int> nearest_idx;
                std::vector<float> nearest_dists;
                // if(query_wrt_object.norm() > 10e3) { continue; }
                m_math.findKNearestIndex(target_object_data.md_KdTree_octoMap_CAD, query_wrt_object, nearest_idx, nearest_dists, 1);
                // 선택된 Mask에 근접한 점은 삭제 (thre_dist 이내의 점들을 제외)
                if(nearest_dists[0] < thre_dist) { continue; }
            }

            cloud_env_base_frame_rgb->points[i].r = 255;
            cloud_env_base_frame_rgb->points[i].g = 1;
            cloud_env_base_frame_rgb->points[i].b = 1;

            ////////////////////////////////////////////////

            // cloud_gripper_CAD_rgb->points[nearest_idx[0]].r = 255;
            // cloud_gripper_CAD_rgb->points[nearest_idx[0]].g = 1;
            // cloud_gripper_CAD_rgb->points[nearest_idx[0]].b = 1;

            // float grp_cad_pt_y = cloud_gripper_CAD_gripper_frame->points[nearest_idx[0]].y;
            // KDTree 사용하지 않기 위해, 점유로 판별된 그리퍼 좌표계 기준의 측정점을 이용(점유되었으므로, 그리퍼 근처 값을 가질 것)
            float grp_cad_pt_y = cloud_env_gripper_frame->points[i].y; // cloud_gripper_CAD_gripper_frame, pt_y

            // ROS_LOG_INFO("grp_cad_pt_y(occupied): %0.1f", grp_cad_pt_y);
            // ZIVID가 장착된 방향이 정면, 정면을 향하도록 바라봤을 때를 기준으로 좌(+y)우(-y)를 의미
            if(grp_cad_pt_y > 0.0) cnt_gripper_tip_left_occupied++; // left (+y)
            else cnt_gripper_tip_right_occupied++; // right (-y)
            // total
            cnt_occupied++;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////
    //// TODO: Collision Threshold 정의
    if(cnt_occupied > threshold) is_collision = true;
    else is_collision = false;
    collision_count_set.resize(3);
    collision_count_set[0] = cnt_occupied; // total
    collision_count_set[1] = cnt_gripper_tip_left_occupied; // left
    collision_count_set[2] = cnt_gripper_tip_right_occupied; // right

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb_with_gripper(new pcl::PointCloud<pcl::PointXYZRGB>);
    double visualize_pt_size_1 = 2.5;
    double visualize_pt_size_2 = 2.5;
    m_math.cloudScaling(*cloud_gripper_CAD_rgb, 0); // mm to m
    m_math.cloudScaling(*cloud_env_base_frame_rgb, 0); // mm to m

    *cloud_env_base_frame_rgb_with_gripper = *cloud_env_base_frame_rgb;
    *cloud_env_base_frame_rgb_with_gripper += *cloud_gripper_CAD_rgb;
    //// Object CAD cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_object_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*target_object_data.cloud_CAD_uniformly_downsampled, *cloud_object_CAD_rgb);
    Eigen::Matrix4f T_B2O = target_object_data.T_matching_B2O;
    pcl::transformPointCloud(*cloud_object_CAD_rgb, *cloud_object_CAD_rgb, T_B2O);
    for (size_t i = 0; i < cloud_object_CAD_rgb->size(); i++)
    {
        cloud_object_CAD_rgb->points[i].r = 1;
        cloud_object_CAD_rgb->points[i].g = 1;
        cloud_object_CAD_rgb->points[i].b = 255;
    }
    m_math.cloudScaling(*cloud_object_CAD_rgb, 0); // mm to m
    *cloud_env_base_frame_rgb_with_gripper += *cloud_object_CAD_rgb; // final results

    if (b_plot)
    {
        pcl::visualization::PCLVisualizer viewer("plotFinalPoseResults");
        int v1(0);
        int v2(1);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(1, 1, 1, v1);
        viewer.setBackgroundColor(1, 1, 1, v2);
        // CAD cloud is black
        viewer.addPointCloud(cloud_env_base_frame_rgb, "cloud_in_v1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
        viewer.addPointCloud(cloud_env_base_frame_rgb_with_gripper, "cloud_in_v2", v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

        //// Orientation
        Eigen::Vector4f grp_p0(0.0, 0.0, 0.0, 1.0);
        Eigen::Vector4f grp_p_base = T_B2E*grp_p0;

        Eigen::Vector3f grp_rx0(1.0, 0.0, 0.0);
        Eigen::Vector3f grp_ry0(0.0, 1.0, 0.0);
        Eigen::Vector3f grp_rz0(0.0, 0.0, 1.0);
        Eigen::Vector3f grp_rx_base = T_B2E.block<3,3>(0,0)*grp_rx0;
        Eigen::Vector3f grp_ry_base = T_B2E.block<3,3>(0,0)*grp_ry0;
        Eigen::Vector3f grp_rz_base = T_B2E.block<3,3>(0,0)*grp_rz0;

        float line_length = 75.0;
        Eigen::Vector3f grp_ppx = grp_p_base.block<3,1>(0,0) + line_length*grp_rx_base;
        Eigen::Vector3f grp_ppy = grp_p_base.block<3,1>(0,0) + line_length*grp_ry_base;
        Eigen::Vector3f grp_ppz = grp_p_base.block<3,1>(0,0) + line_length*grp_rz_base;

        // std::cout << "grp_p_base" << std::endl;
        // std::cout << grp_p_base << std::endl;
        // std::cout << "ppx ppy ppz" << std::endl;
        // std::cout << ppx << std::endl;
        // std::cout << ppy << std::endl;
        // std::cout << ppz << std::endl;

        pcl::PointXYZ grp_ppt0(0.001*grp_p_base(0), 0.001*grp_p_base(1), 0.001*grp_p_base(2));
        pcl::PointXYZ grp_pptx(0.001*grp_ppx(0), 0.001*grp_ppx(1), 0.001*grp_ppx(2));
        pcl::PointXYZ grp_ppty(0.001*grp_ppy(0), 0.001*grp_ppy(1), 0.001*grp_ppy(2));
        pcl::PointXYZ grp_pptz(0.001*grp_ppz(0), 0.001*grp_ppz(1), 0.001*grp_ppz(2));
        viewer.addLine(grp_ppt0, grp_pptx, 1.0, 0.0, 0.0, "x");
        viewer.addLine(grp_ppt0, grp_ppty, 0.0, 1.0, 0.0, "y");
        viewer.addLine(grp_ppt0, grp_pptz, 0.0, 0.0, 1.0, "z");

        // Set camera position
        float view_scale = 0.8;
        viewer.setCameraPosition(0.001*grp_p_base[0] + view_scale, 0.001*grp_p_base[1] + view_scale, 0.001*grp_p_base[2] + view_scale, 0, 0, 1);
        std::vector<pcl::visualization::Camera> cams;
        viewer.getCameras(cams);
        for (auto &&camera : cams)
        {
            for (int i = 0; i < 3; i++)
            {
                camera.focal[i] = 0.001*grp_p_base[i];
            }
        }
        viewer.setCameraParameters(cams[0]);

        //// Base frame
        Eigen::Vector3f p0(0.0, 0.0, 0.0);
        Eigen::Vector3f ppx0 = p0 + line_length*grp_rx0;
        Eigen::Vector3f ppy0 = p0 + line_length*grp_ry0;
        Eigen::Vector3f ppz0 = p0 + line_length*grp_rz0;
        pcl::PointXYZ ppt0(0.001*p0(0), 0.001*p0(1), 0.001*p0(2));
        pcl::PointXYZ pptx(0.001*ppx0(0), 0.001*ppx0(1), 0.001*ppx0(2));
        pcl::PointXYZ ppty(0.001*ppy0(0), 0.001*ppy0(1), 0.001*ppy0(2));
        pcl::PointXYZ pptz(0.001*ppz0(0), 0.001*ppz0(1), 0.001*ppz0(2));
        viewer.addLine(ppt0, pptx, 1.0, 0.0, 0.0, "x0");
        viewer.addLine(ppt0, ppty, 0.0, 1.0, 0.0, "y0");
        viewer.addLine(ppt0, pptz, 0.0, 0.0, 1.0, "z0");

        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }
    m_math.cloudScaling(*cloud_env_base_frame_rgb_with_gripper, 1); // m to mm
    *target_object_data.cloud_final_results_for_collision_check = *cloud_env_base_frame_rgb_with_gripper;
    return is_collision;
}

/** @brief Korean: 
 */
bool PCLPROCESSING::plotFinalPoseResultsWithKDTree(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const size_t &threshold, std::vector<size_t> &collision_count_set, std::string plot_name, bool b_plot)
{
    bool is_collision = false;
    size_t cnt_occupied = 0;
    size_t cnt_gripper_tip_left_occupied = 0;
    size_t cnt_gripper_tip_right_occupied = 0;

    //// Gripper CAD idx
    for (size_t i = 0; i < target_object_data->set_gripper_CAD_open_length.size(); i++)
    {
        if(target_object_data->set_gripper_CAD_open_length[i] == target_object_data->gripper_open_length)
        {
            target_object_data->gripper_idx_now = i;
            break;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Scanned cloud merged with bin(CAD) for collision check
    // 그리퍼 CAD 좌표변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gripper_CAD_gripper_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(target_object_data->set_cloud_gripper_CAD_uniformly_downsampled[target_object_data->gripper_idx_now], *cloud_gripper_CAD_gripper_frame);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gripper_CAD_base_frame(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f T_B2E = T_tgt;
    pcl::transformPointCloud(target_object_data->set_cloud_gripper_CAD_uniformly_downsampled[target_object_data->gripper_idx_now], *cloud_gripper_CAD_base_frame, T_B2E); // transform the scanned cloud from base to object frame

    // sensor_origin - measurement origin in global reference frame
    // {E} 기준의 그리퍼 CAD의 Octomap을 {B} 기준의 Octomap으로 변환하여 사용하기 위해 origin_pose 입력(T_B2E에 해당하는 pose)
    std::vector<double> origin_pose = m_math.htm2pose_mm2m(T_B2E, "origin_pose"); // [mm] to [m]
    for (int j = 0; j < 3; j++) 
    { 
        origin_pose[j] = 1000.0*origin_pose[j];
        origin_pose[j+3] = DEGTORAD*origin_pose[j+3];
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_base_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_gripper_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_env_base_frame = *target_object_data->cloud_scan_base_aligned_with_bin_collision_check_process;
    pcl::copyPointCloud(*cloud_env_base_frame, *cloud_env_base_frame_rgb);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_gripper_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_gripper_CAD_base_frame, *cloud_gripper_CAD_rgb);

    std::vector<size_t> gripper_tip_idx = target_object_data->set_gripper_tip_surface_idx[target_object_data->gripper_idx_now];
    for (size_t i = 0; i < cloud_gripper_CAD_rgb->size(); i++)
    {
        cloud_gripper_CAD_rgb->points[i].r = 1;
        cloud_gripper_CAD_rgb->points[i].g = 255;
        cloud_gripper_CAD_rgb->points[i].b = 1;
    }

    for (size_t i = 0; i < gripper_tip_idx.size(); i++)
    {
        size_t idx_tmp = gripper_tip_idx[i]; 
        cloud_gripper_CAD_rgb->points[idx_tmp].r = 255;
        cloud_gripper_CAD_rgb->points[idx_tmp].g = 1;
        cloud_gripper_CAD_rgb->points[idx_tmp].b = 255;
    }

    //// Check collision using voxel map w.r.t nearest points from the gripper CAD cloud
    // kdTree search -  center에 가장 가까운 CAD 점 선택
    Eigen::Matrix4f T_E2B = T_B2E.inverse();
    pcl::transformPointCloud(*cloud_env_base_frame, *cloud_env_gripper_frame, T_E2B);

    ////////////////////////////////////////////////////////////////////////////////////////////
    //// Check cloud occupancy
    octomap::MapCollection<octomap::MapNode<octomap::OcTree>> collection;
    octomap::MapNode<octomap::OcTree>* mn1 = new octomap::MapNode<octomap::OcTree>(target_object_data->m_set_OctoMap_gripper_CAD[target_object_data->gripper_idx_now], octomap::pose6d(origin_pose[0], origin_pose[1], origin_pose[2], origin_pose[3], origin_pose[4], origin_pose[5]));
    mn1->setId("CAD");
    collection.addNode(mn1);
    target_object_data->m_set_OctoMap_gripper_CAD[target_object_data->gripper_idx_now]->setOccupancyThres(0.6); // 점유 파라미터(thre_map) 이상이면 점유(occupied)로 판정
    // Occlusion check
    for (size_t i = 0; i < cloud_env_base_frame->size(); i++)
    {
        octomap::point3d q;
        q(0) = cloud_env_base_frame->points[i].x;
        q(1) = cloud_env_base_frame->points[i].y;
        q(2) = cloud_env_base_frame->points[i].z;
        if (collection.isOccupied(q)) 
        { 
            ////////////////////////////////////////////////
            //// 230214
            //// 인식된 CAD에 속한 점이 충돌로 감지된 경우는 제외하도록 처리
            //// CAD 점군으로부터 threshold 내의 존재하는 점을 제거
            // {O} 기준의 KDTree Model 이용
            if(1) 
            {
                Eigen::Matrix4f T_O2B = target_object_data->T_matching_O2B;
                // ROS_LOG_INFO("idx_now: %zu", idx_now);
                Eigen::Vector3d query_point = m_math.cloud2pt(*cloud_env_base_frame, i);
                Eigen::Vector4f query_tmp;
                for(int j=0; j<3; j++) { query_tmp[j] = query_point[j]; }
                query_tmp[3] = 1.0;
                query_tmp = T_O2B*query_tmp;
                Eigen::Vector3f query_wrt_object = query_tmp.block<3,1>(0,0);
                float thre_dist = 1.0; // [mm]
                std::vector<int> nearest_idx;
                std::vector<float> nearest_dists;
                // if(query_wrt_object.norm() > 10e3) { continue; }
                m_math.findKNearestIndex(target_object_data->md_KdTree_octoMap_CAD, query_wrt_object, nearest_idx, nearest_dists, 1);
                // 선택된 Mask에 근접한 점은 삭제 (thre_dist 이내의 점들을 제외)
                if(nearest_dists[0] < thre_dist) { continue; }
            }

            cloud_env_base_frame_rgb->points[i].r = 255;
            cloud_env_base_frame_rgb->points[i].g = 1;
            cloud_env_base_frame_rgb->points[i].b = 1;

            ////////////////////////////////////////////////

            // cloud_gripper_CAD_rgb->points[nearest_idx[0]].r = 255;
            // cloud_gripper_CAD_rgb->points[nearest_idx[0]].g = 1;
            // cloud_gripper_CAD_rgb->points[nearest_idx[0]].b = 1;

            // float grp_cad_pt_y = cloud_gripper_CAD_gripper_frame->points[nearest_idx[0]].y;
            // KDTree 사용하지 않기 위해, 점유로 판별된 그리퍼 좌표계 기준의 측정점을 이용(점유되었으므로, 그리퍼 근처 값을 가질 것)
            float grp_cad_pt_y = cloud_env_gripper_frame->points[i].y; // cloud_gripper_CAD_gripper_frame, pt_y

            // ROS_LOG_INFO("grp_cad_pt_y(occupied): %0.1f", grp_cad_pt_y);
            // ZIVID가 장착된 방향이 정면, 정면을 향하도록 바라봤을 때를 기준으로 좌(+y)우(-y)를 의미
            if(grp_cad_pt_y > 0.0) cnt_gripper_tip_left_occupied++; // left (+y)
            else cnt_gripper_tip_right_occupied++; // right (-y)
            // total
            cnt_occupied++;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////
    //// TODO: Collision Threshold 정의
    if(cnt_occupied > threshold) is_collision = true;
    else is_collision = false;
    collision_count_set.resize(3);
    collision_count_set[0] = cnt_occupied; // total
    collision_count_set[1] = cnt_gripper_tip_left_occupied; // left
    collision_count_set[2] = cnt_gripper_tip_right_occupied; // right

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb_with_gripper(new pcl::PointCloud<pcl::PointXYZRGB>);
    double visualize_pt_size_1 = 2.5;
    double visualize_pt_size_2 = 2.5;
    m_math.cloudScaling(*cloud_gripper_CAD_rgb, 0); // mm to m
    m_math.cloudScaling(*cloud_env_base_frame_rgb, 0); // mm to m

    *cloud_env_base_frame_rgb_with_gripper = *cloud_env_base_frame_rgb;
    *cloud_env_base_frame_rgb_with_gripper += *cloud_gripper_CAD_rgb;
    //// Object CAD cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_object_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*target_object_data->cloud_CAD_uniformly_downsampled, *cloud_object_CAD_rgb);
    Eigen::Matrix4f T_B2O = target_object_data->T_matching_B2O;
    pcl::transformPointCloud(*cloud_object_CAD_rgb, *cloud_object_CAD_rgb, T_B2O);
    for (size_t i = 0; i < cloud_object_CAD_rgb->size(); i++)
    {
        cloud_object_CAD_rgb->points[i].r = 1;
        cloud_object_CAD_rgb->points[i].g = 1;
        cloud_object_CAD_rgb->points[i].b = 255;
    }
    m_math.cloudScaling(*cloud_object_CAD_rgb, 0); // mm to m
    *cloud_env_base_frame_rgb_with_gripper += *cloud_object_CAD_rgb; // final results

    if (b_plot)
    {
        pcl::visualization::PCLVisualizer viewer(plot_name.c_str());
        int v1(0);
        int v2(1);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(1, 1, 1, v1);
        viewer.setBackgroundColor(1, 1, 1, v2);
        // CAD cloud is black
        viewer.addPointCloud(cloud_env_base_frame_rgb, "cloud_in_v1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
        viewer.addPointCloud(cloud_env_base_frame_rgb_with_gripper, "cloud_in_v2", v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

        //// Orientation
        Eigen::Vector4f grp_p0(0.0, 0.0, 0.0, 1.0);
        Eigen::Vector4f grp_p_base = T_B2E*grp_p0;

        Eigen::Vector3f grp_rx0(1.0, 0.0, 0.0);
        Eigen::Vector3f grp_ry0(0.0, 1.0, 0.0);
        Eigen::Vector3f grp_rz0(0.0, 0.0, 1.0);
        Eigen::Vector3f grp_rx_base = T_B2E.block<3,3>(0,0)*grp_rx0;
        Eigen::Vector3f grp_ry_base = T_B2E.block<3,3>(0,0)*grp_ry0;
        Eigen::Vector3f grp_rz_base = T_B2E.block<3,3>(0,0)*grp_rz0;

        float line_length = 75.0;
        Eigen::Vector3f grp_ppx = grp_p_base.block<3,1>(0,0) + line_length*grp_rx_base;
        Eigen::Vector3f grp_ppy = grp_p_base.block<3,1>(0,0) + line_length*grp_ry_base;
        Eigen::Vector3f grp_ppz = grp_p_base.block<3,1>(0,0) + line_length*grp_rz_base;

        // std::cout << "grp_p_base" << std::endl;
        // std::cout << grp_p_base << std::endl;
        // std::cout << "ppx ppy ppz" << std::endl;
        // std::cout << ppx << std::endl;
        // std::cout << ppy << std::endl;
        // std::cout << ppz << std::endl;

        pcl::PointXYZ grp_ppt0(0.001*grp_p_base(0), 0.001*grp_p_base(1), 0.001*grp_p_base(2));
        pcl::PointXYZ grp_pptx(0.001*grp_ppx(0), 0.001*grp_ppx(1), 0.001*grp_ppx(2));
        pcl::PointXYZ grp_ppty(0.001*grp_ppy(0), 0.001*grp_ppy(1), 0.001*grp_ppy(2));
        pcl::PointXYZ grp_pptz(0.001*grp_ppz(0), 0.001*grp_ppz(1), 0.001*grp_ppz(2));
        viewer.addLine(grp_ppt0, grp_pptx, 1.0, 0.0, 0.0, "x");
        viewer.addLine(grp_ppt0, grp_ppty, 0.0, 1.0, 0.0, "y");
        viewer.addLine(grp_ppt0, grp_pptz, 0.0, 0.0, 1.0, "z");

        // Set camera position
        float view_scale = 0.8;
        viewer.setCameraPosition(0.001*grp_p_base[0] + view_scale, 0.001*grp_p_base[1] + view_scale, 0.001*grp_p_base[2] + view_scale, 0, 0, 1);
        std::vector<pcl::visualization::Camera> cams;
        viewer.getCameras(cams);
        for (auto &&camera : cams)
        {
            for (int i = 0; i < 3; i++)
            {
                camera.focal[i] = 0.001*grp_p_base[i];
            }
        }
        viewer.setCameraParameters(cams[0]);

        //// Base frame
        Eigen::Vector3f p0(0.0, 0.0, 0.0);
        Eigen::Vector3f ppx0 = p0 + line_length*grp_rx0;
        Eigen::Vector3f ppy0 = p0 + line_length*grp_ry0;
        Eigen::Vector3f ppz0 = p0 + line_length*grp_rz0;
        pcl::PointXYZ ppt0(0.001*p0(0), 0.001*p0(1), 0.001*p0(2));
        pcl::PointXYZ pptx(0.001*ppx0(0), 0.001*ppx0(1), 0.001*ppx0(2));
        pcl::PointXYZ ppty(0.001*ppy0(0), 0.001*ppy0(1), 0.001*ppy0(2));
        pcl::PointXYZ pptz(0.001*ppz0(0), 0.001*ppz0(1), 0.001*ppz0(2));
        viewer.addLine(ppt0, pptx, 1.0, 0.0, 0.0, "x0");
        viewer.addLine(ppt0, ppty, 0.0, 1.0, 0.0, "y0");
        viewer.addLine(ppt0, pptz, 0.0, 0.0, 1.0, "z0");

        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }
    m_math.cloudScaling(*cloud_env_base_frame_rgb_with_gripper, 1); // m to mm
    *target_object_data->cloud_final_results_for_collision_check = *cloud_env_base_frame_rgb_with_gripper;
    return is_collision;
}

/** @brief Korean: 
 */
bool PCLPROCESSING::plotFinalPoseResultsNoKDTree(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const size_t &threshold, std::vector<size_t> &collision_count_set, std::string plot_name, bool b_plot)
{
    bool is_collision = false;
    size_t cnt_occupied = 0;
    size_t cnt_gripper_tip_left_occupied = 0;
    size_t cnt_gripper_tip_right_occupied = 0;

    //// Gripper CAD idx
    for (size_t i = 0; i < target_object_data->set_gripper_CAD_open_length.size(); i++)
    {
        if(target_object_data->set_gripper_CAD_open_length[i] == target_object_data->gripper_open_length)
        {
            target_object_data->gripper_idx_now = i;
            break;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Scanned cloud merged with bin(CAD) for collision check
    // 그리퍼 CAD 좌표변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gripper_CAD_gripper_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(target_object_data->set_cloud_gripper_CAD_uniformly_downsampled[target_object_data->gripper_idx_now], *cloud_gripper_CAD_gripper_frame);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gripper_CAD_base_frame(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f T_B2E = T_tgt;
    pcl::transformPointCloud(target_object_data->set_cloud_gripper_CAD_uniformly_downsampled[target_object_data->gripper_idx_now], *cloud_gripper_CAD_base_frame, T_B2E); // transform the scanned cloud from base to object frame

    // sensor_origin - measurement origin in global reference frame
    // {E} 기준의 그리퍼 CAD의 Octomap을 {B} 기준의 Octomap으로 변환하여 사용하기 위해 origin_pose 입력(T_B2E에 해당하는 pose)
    std::vector<double> origin_pose = m_math.htm2pose_mm2m(T_B2E, "origin_pose"); // [mm] to [m]
    for (int j = 0; j < 3; j++) 
    { 
        origin_pose[j] = 1000.0*origin_pose[j];
        origin_pose[j+3] = DEGTORAD*origin_pose[j+3];
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_base_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_gripper_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_env_base_frame = *target_object_data->cloud_scan_base_aligned_with_bin_collision_check_process;
    pcl::copyPointCloud(*cloud_env_base_frame, *cloud_env_base_frame_rgb);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_gripper_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_gripper_CAD_base_frame, *cloud_gripper_CAD_rgb);

    std::vector<size_t> gripper_tip_idx = target_object_data->set_gripper_tip_surface_idx[target_object_data->gripper_idx_now];
    for (size_t i = 0; i < cloud_gripper_CAD_rgb->size(); i++)
    {
        cloud_gripper_CAD_rgb->points[i].r = 1;
        cloud_gripper_CAD_rgb->points[i].g = 255;
        cloud_gripper_CAD_rgb->points[i].b = 1;
    }

    for (size_t i = 0; i < gripper_tip_idx.size(); i++)
    {
        size_t idx_tmp = gripper_tip_idx[i]; 
        cloud_gripper_CAD_rgb->points[idx_tmp].r = 255;
        cloud_gripper_CAD_rgb->points[idx_tmp].g = 1;
        cloud_gripper_CAD_rgb->points[idx_tmp].b = 255;
    }

    //// Check collision using voxel map w.r.t nearest points from the gripper CAD cloud
    // kdTree search -  center에 가장 가까운 CAD 점 선택
    Eigen::Matrix4f T_E2B = T_B2E.inverse();
    pcl::transformPointCloud(*cloud_env_base_frame, *cloud_env_gripper_frame, T_E2B);

    ////////////////////////////////////////////////////////////////////////////////////////////
    //// Check cloud occupancy
    octomap::MapCollection<octomap::MapNode<octomap::OcTree>> collection;
    octomap::MapNode<octomap::OcTree>* mn1 = new octomap::MapNode<octomap::OcTree>(target_object_data->m_set_OctoMap_gripper_CAD[target_object_data->gripper_idx_now], octomap::pose6d(origin_pose[0], origin_pose[1], origin_pose[2], origin_pose[3], origin_pose[4], origin_pose[5]));
    mn1->setId("CAD");
    collection.addNode(mn1);
    target_object_data->m_set_OctoMap_gripper_CAD[target_object_data->gripper_idx_now]->setOccupancyThres(0.6); // 점유 파라미터(thre_map) 이상이면 점유(occupied)로 판정
    // Occlusion check
    for (size_t i = 0; i < cloud_env_base_frame->size(); i++)
    {
        octomap::point3d q;
        q(0) = cloud_env_base_frame->points[i].x;
        q(1) = cloud_env_base_frame->points[i].y;
        q(2) = cloud_env_base_frame->points[i].z;
        if (collection.isOccupied(q)) 
        { 
            cloud_env_base_frame_rgb->points[i].r = 255;
            cloud_env_base_frame_rgb->points[i].g = 1;
            cloud_env_base_frame_rgb->points[i].b = 1;

            // float grp_cad_pt_y = cloud_gripper_CAD_gripper_frame->points[nearest_idx[0]].y;
            // KDTree 사용하지 않기 위해, 점유로 판별된 그리퍼 좌표계 기준의 측정점을 이용(점유되었으므로, 그리퍼 근처 값을 가질 것)
            float grp_cad_pt_y = cloud_env_gripper_frame->points[i].y; // cloud_gripper_CAD_gripper_frame, pt_y

            // ROS_LOG_INFO("grp_cad_pt_y(occupied): %0.1f", grp_cad_pt_y);
            // ZIVID가 장착된 방향이 정면, 정면을 향하도록 바라봤을 때를 기준으로 좌(+y)우(-y)를 의미
            if(grp_cad_pt_y > 0.0) cnt_gripper_tip_left_occupied++; // left (+y)
            else cnt_gripper_tip_right_occupied++; // right (-y)
            // total
            cnt_occupied++;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////
    //// TODO: Collision Threshold 정의
    if(cnt_occupied > threshold) is_collision = true;
    else is_collision = false;
    collision_count_set.resize(3);
    collision_count_set[0] = cnt_occupied; // total
    collision_count_set[1] = cnt_gripper_tip_left_occupied; // left
    collision_count_set[2] = cnt_gripper_tip_right_occupied; // right

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb_with_gripper(new pcl::PointCloud<pcl::PointXYZRGB>);
    double visualize_pt_size_1 = 2.5;
    double visualize_pt_size_2 = 2.5;
    m_math.cloudScaling(*cloud_gripper_CAD_rgb, 0); // mm to m
    m_math.cloudScaling(*cloud_env_base_frame_rgb, 0); // mm to m

    *cloud_env_base_frame_rgb_with_gripper = *cloud_env_base_frame_rgb;
    *cloud_env_base_frame_rgb_with_gripper += *cloud_gripper_CAD_rgb;
    //// Object CAD cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_object_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*target_object_data->cloud_CAD_uniformly_downsampled, *cloud_object_CAD_rgb);
    Eigen::Matrix4f T_B2O = target_object_data->T_matching_B2O;
    pcl::transformPointCloud(*cloud_object_CAD_rgb, *cloud_object_CAD_rgb, T_B2O);
    for (size_t i = 0; i < cloud_object_CAD_rgb->size(); i++)
    {
        cloud_object_CAD_rgb->points[i].r = 1;
        cloud_object_CAD_rgb->points[i].g = 1;
        cloud_object_CAD_rgb->points[i].b = 255;
    }
    m_math.cloudScaling(*cloud_object_CAD_rgb, 0); // mm to m
    *cloud_env_base_frame_rgb_with_gripper += *cloud_object_CAD_rgb; // final results

    if (b_plot)
    {
        pcl::visualization::PCLVisualizer viewer(plot_name.c_str());
        int v1(0);
        int v2(1);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(1, 1, 1, v1);
        viewer.setBackgroundColor(1, 1, 1, v2);
        // CAD cloud is black
        viewer.addPointCloud(cloud_env_base_frame_rgb, "cloud_in_v1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
        viewer.addPointCloud(cloud_env_base_frame_rgb_with_gripper, "cloud_in_v2", v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

        //// Orientation
        Eigen::Vector4f grp_p0(0.0, 0.0, 0.0, 1.0);
        Eigen::Vector4f grp_p_base = T_B2E*grp_p0;

        Eigen::Vector3f grp_rx0(1.0, 0.0, 0.0);
        Eigen::Vector3f grp_ry0(0.0, 1.0, 0.0);
        Eigen::Vector3f grp_rz0(0.0, 0.0, 1.0);
        Eigen::Vector3f grp_rx_base = T_B2E.block<3,3>(0,0)*grp_rx0;
        Eigen::Vector3f grp_ry_base = T_B2E.block<3,3>(0,0)*grp_ry0;
        Eigen::Vector3f grp_rz_base = T_B2E.block<3,3>(0,0)*grp_rz0;

        float line_length = 75.0;
        Eigen::Vector3f grp_ppx = grp_p_base.block<3,1>(0,0) + line_length*grp_rx_base;
        Eigen::Vector3f grp_ppy = grp_p_base.block<3,1>(0,0) + line_length*grp_ry_base;
        Eigen::Vector3f grp_ppz = grp_p_base.block<3,1>(0,0) + line_length*grp_rz_base;

        // std::cout << "grp_p_base" << std::endl;
        // std::cout << grp_p_base << std::endl;
        // std::cout << "ppx ppy ppz" << std::endl;
        // std::cout << ppx << std::endl;
        // std::cout << ppy << std::endl;
        // std::cout << ppz << std::endl;

        pcl::PointXYZ grp_ppt0(0.001*grp_p_base(0), 0.001*grp_p_base(1), 0.001*grp_p_base(2));
        pcl::PointXYZ grp_pptx(0.001*grp_ppx(0), 0.001*grp_ppx(1), 0.001*grp_ppx(2));
        pcl::PointXYZ grp_ppty(0.001*grp_ppy(0), 0.001*grp_ppy(1), 0.001*grp_ppy(2));
        pcl::PointXYZ grp_pptz(0.001*grp_ppz(0), 0.001*grp_ppz(1), 0.001*grp_ppz(2));
        viewer.addLine(grp_ppt0, grp_pptx, 1.0, 0.0, 0.0, "x");
        viewer.addLine(grp_ppt0, grp_ppty, 0.0, 1.0, 0.0, "y");
        viewer.addLine(grp_ppt0, grp_pptz, 0.0, 0.0, 1.0, "z");

        // Set camera position
        float view_scale = 0.8;
        viewer.setCameraPosition(0.001*grp_p_base[0] + view_scale, 0.001*grp_p_base[1] + view_scale, 0.001*grp_p_base[2] + view_scale, 0, 0, 1);
        std::vector<pcl::visualization::Camera> cams;
        viewer.getCameras(cams);
        for (auto &&camera : cams)
        {
            for (int i = 0; i < 3; i++)
            {
                camera.focal[i] = 0.001*grp_p_base[i];
            }
        }
        viewer.setCameraParameters(cams[0]);

        //// Base frame
        Eigen::Vector3f p0(0.0, 0.0, 0.0);
        Eigen::Vector3f ppx0 = p0 + line_length*grp_rx0;
        Eigen::Vector3f ppy0 = p0 + line_length*grp_ry0;
        Eigen::Vector3f ppz0 = p0 + line_length*grp_rz0;
        pcl::PointXYZ ppt0(0.001*p0(0), 0.001*p0(1), 0.001*p0(2));
        pcl::PointXYZ pptx(0.001*ppx0(0), 0.001*ppx0(1), 0.001*ppx0(2));
        pcl::PointXYZ ppty(0.001*ppy0(0), 0.001*ppy0(1), 0.001*ppy0(2));
        pcl::PointXYZ pptz(0.001*ppz0(0), 0.001*ppz0(1), 0.001*ppz0(2));
        viewer.addLine(ppt0, pptx, 1.0, 0.0, 0.0, "x0");
        viewer.addLine(ppt0, ppty, 0.0, 1.0, 0.0, "y0");
        viewer.addLine(ppt0, pptz, 0.0, 0.0, 1.0, "z0");

        while (!viewer.wasStopped())
        {
            viewer.spin();
            // viewer.spinOnce(100);
            // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }
    m_math.cloudScaling(*cloud_env_base_frame_rgb_with_gripper, 1); // m to mm
    *target_object_data->cloud_final_results_for_collision_check = *cloud_env_base_frame_rgb_with_gripper;
    return is_collision;
}

/** @brief Korean: 
 */
bool PCLPROCESSING::plotFinalPoseResultsNoKDTreeForEscape(CTARGET_OBJECT_DATA* &target_object_data, const Eigen::Matrix4f &T_tgt, const Eigen::Matrix4f &T_proc, const size_t &threshold, std::vector<size_t> &collision_count_set, std::string plot_name, bool b_plot)
{
    bool is_collision = false;
    size_t cnt_occupied = 0;
    size_t cnt_gripper_tip_left_occupied = 0;
    size_t cnt_gripper_tip_right_occupied = 0;

    //// Gripper CAD idx
    for (size_t i = 0; i < target_object_data->set_gripper_CAD_open_length.size(); i++)
    {
        if(target_object_data->set_gripper_CAD_open_length[i] == target_object_data->gripper_open_length)
        {
            target_object_data->gripper_idx_now = i;
            break;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    //// Scanned cloud merged with bin(CAD) for collision check
    // 그리퍼 CAD 좌표변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gripper_CAD_gripper_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(target_object_data->set_cloud_gripper_CAD_uniformly_downsampled[target_object_data->gripper_idx_now], *cloud_gripper_CAD_gripper_frame);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gripper_CAD_base_frame(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f T_B2E = T_proc*T_tgt;
    pcl::transformPointCloud(target_object_data->set_cloud_gripper_CAD_uniformly_downsampled[target_object_data->gripper_idx_now], *cloud_gripper_CAD_base_frame, T_B2E); // transform the scanned cloud from base to object frame

    // sensor_origin - measurement origin in global reference frame
    // {E} 기준의 그리퍼 CAD의 Octomap을 {B} 기준의 Octomap으로 변환하여 사용하기 위해 origin_pose 입력(T_B2E에 해당하는 pose)
    std::vector<double> origin_pose = m_math.htm2pose_mm2m(T_B2E, "origin_pose"); // [mm] to [m]
    for (int j = 0; j < 3; j++) 
    { 
        origin_pose[j] = 1000.0*origin_pose[j];
        origin_pose[j+3] = DEGTORAD*origin_pose[j+3];
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_base_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_env_gripper_frame(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_env_base_frame = *target_object_data->cloud_scan_base_aligned_with_bin_collision_check_process;
    pcl::copyPointCloud(*cloud_env_base_frame, *cloud_env_base_frame_rgb);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_gripper_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_gripper_CAD_base_frame, *cloud_gripper_CAD_rgb);

    std::vector<size_t> gripper_tip_idx = target_object_data->set_gripper_tip_surface_idx[target_object_data->gripper_idx_now];
    for (size_t i = 0; i < cloud_gripper_CAD_rgb->size(); i++)
    {
        cloud_gripper_CAD_rgb->points[i].r = 1;
        cloud_gripper_CAD_rgb->points[i].g = 255;
        cloud_gripper_CAD_rgb->points[i].b = 1;
    }

    for (size_t i = 0; i < gripper_tip_idx.size(); i++)
    {
        size_t idx_tmp = gripper_tip_idx[i]; 
        cloud_gripper_CAD_rgb->points[idx_tmp].r = 255;
        cloud_gripper_CAD_rgb->points[idx_tmp].g = 1;
        cloud_gripper_CAD_rgb->points[idx_tmp].b = 255;
    }

    //// Check collision using voxel map w.r.t nearest points from the gripper CAD cloud
    // kdTree search -  center에 가장 가까운 CAD 점 선택
    Eigen::Matrix4f T_E2B = T_B2E.inverse();
    pcl::transformPointCloud(*cloud_env_base_frame, *cloud_env_gripper_frame, T_E2B);

    ////////////////////////////////////////////////////////////////////////////////////////////
    //// Check cloud occupancy

    octomap::MapCollection<octomap::MapNode<octomap::OcTree>> collection;
    // sensor_origin - measurement origin in global reference frame
    // {E} 기준의 그리퍼 CAD의 Octomap을 {B} 기준의 Octomap으로 변환하여 사용하기 위해 origin_pose 입력(T_B2E에 해당하는 pose)
    std::vector<double> pose1 = m_math.htm2pose(T_B2E, "origin_pose"); // [mm] to [m]
    for (int j = 0; j < 3; j++) { 
        pose1[j+3] = DEGTORAD*pose1[j+3];
    }
    // Eigen::Matrix4f T_E2GP = target_object_data->T_E2GP;
    // Eigen::Matrix4f T_GP2O = target_object_data->T_O2GP;
    // Eigen::Matrix4f T_E2O = T_E2GP*T_GP2O;
    // // m_math.printHTM(T_E2O, "T_E2O(Evaluation grasping pose)");

    Eigen::Matrix4f T_B2O = target_object_data->T_matching_B2O;
    T_B2O = T_proc*T_B2O;
    std::vector<double> pose2 = m_math.htm2pose(T_B2O, "pose2"); // T_E2O에 해당하는 pose
    for (int j = 0; j < 3; j++) { 
        pose2[j+3] = DEGTORAD*pose2[j+3];
    }
    genOctomapCollection2(target_object_data->m_set_OctoMap_gripper_CAD[target_object_data->gripper_idx_now], target_object_data->m_OctoMap_CAD, collection, pose1, pose2);


    // Occlusion check
    for (size_t i = 0; i < cloud_env_base_frame->size(); i++)
    {
        octomap::point3d q;
        q(0) = cloud_env_base_frame->points[i].x;
        q(1) = cloud_env_base_frame->points[i].y;
        q(2) = cloud_env_base_frame->points[i].z;
        if (collection.isOccupied(q)) 
        { 

            ////////////////////////////////////////////////
            //// 230214
            //// 인식된 CAD에 속한 점이 충돌로 감지된 경우는 제외하도록 처리
            //// CAD 점군으로부터 threshold 내의 존재하는 점을 제거
            // {O} 기준의 KDTree Model 이용
            if(1) 
            {
                Eigen::Matrix4f T_O2B = target_object_data->T_matching_O2B;
                // ROS_LOG_INFO("idx_now: %zu", idx_now);
                Eigen::Vector3d query_point = m_math.cloud2pt(*cloud_env_base_frame, i);
                Eigen::Vector4f query_tmp;
                for(int j=0; j<3; j++) { query_tmp[j] = query_point[j]; }
                query_tmp[3] = 1.0;
                query_tmp = T_O2B*query_tmp;
                Eigen::Vector3f query_wrt_object = query_tmp.block<3,1>(0,0);
                float thre_dist = 1.0; // [mm]
                std::vector<int> nearest_idx;
                std::vector<float> nearest_dists;
                // if(query_wrt_object.norm() > 10e3) { continue; }
                m_math.findKNearestIndex(target_object_data->md_KdTree_octoMap_CAD, query_wrt_object, nearest_idx, nearest_dists, 1);
                // 선택된 Mask에 근접한 점은 삭제 (thre_dist 이내의 점들을 제외)
                if(nearest_dists[0] < thre_dist) { continue; }
            }

            cloud_env_base_frame_rgb->points[i].r = 255;
            cloud_env_base_frame_rgb->points[i].g = 1;
            cloud_env_base_frame_rgb->points[i].b = 1;

            // float grp_cad_pt_y = cloud_gripper_CAD_gripper_frame->points[nearest_idx[0]].y;
            // KDTree 사용하지 않기 위해, 점유로 판별된 그리퍼 좌표계 기준의 측정점을 이용(점유되었으므로, 그리퍼 근처 값을 가질 것)
            float grp_cad_pt_y = cloud_env_gripper_frame->points[i].y; // cloud_gripper_CAD_gripper_frame, pt_y

            // ROS_LOG_INFO("grp_cad_pt_y(occupied): %0.1f", grp_cad_pt_y);
            // ZIVID가 장착된 방향이 정면, 정면을 향하도록 바라봤을 때를 기준으로 좌(+y)우(-y)를 의미
            if(grp_cad_pt_y > 0.0) cnt_gripper_tip_left_occupied++; // left (+y)
            else cnt_gripper_tip_right_occupied++; // right (-y)
            // total
            cnt_occupied++;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////
    //// TODO: Collision Threshold 정의
    if(cnt_occupied > threshold) {
        // ROS_LOG_INFO("Collision counts: %zu", cnt_occupied);
        // ROS_LOG_INFO("Collision counts: %zu", cnt_occupied);
        // ROS_LOG_INFO("Collision counts: %zu", cnt_occupied);
        is_collision = true;
    } else {
        is_collision = false;
    }
    collision_count_set.resize(3);
    collision_count_set[0] = cnt_occupied; // total
    collision_count_set[1] = cnt_gripper_tip_left_occupied; // left
    collision_count_set[2] = cnt_gripper_tip_right_occupied; // right

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_env_base_frame_rgb_with_gripper(new pcl::PointCloud<pcl::PointXYZRGB>);
    double visualize_pt_size_1 = 2.5;
    double visualize_pt_size_2 = 2.5;
    m_math.cloudScaling(*cloud_gripper_CAD_rgb, 0); // mm to m
    m_math.cloudScaling(*cloud_env_base_frame_rgb, 0); // mm to m

    *cloud_env_base_frame_rgb_with_gripper = *cloud_env_base_frame_rgb;
    *cloud_env_base_frame_rgb_with_gripper += *cloud_gripper_CAD_rgb;
    //// Object CAD cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_object_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*target_object_data->cloud_CAD_uniformly_downsampled, *cloud_object_CAD_rgb);
    // Eigen::Matrix4f T_E2GP = target_object_data->T_E2GP;
    // Eigen::Matrix4f T_O2GP = target_object_data->T_O2GP;
    // Eigen::Matrix4f T_GP2O = T_O2GP.inverse();
    // Eigen::Matrix4f T_E2O = T_E2GP*T_GP2O;
    // m_math.printHTM(T_E2GP, "T_E2GP");
    // m_math.printHTM(T_O2GP, "T_O2GP");
    // m_math.printHTM(T_GP2O, "T_GP2O");
    // m_math.printHTM(T_E2O, "T_E2O");
    // m_math.printHTM(T_B2E, "T_B2E");
    // Eigen::Matrix4f T_B2O = T_B2E*T_E2O;
    // m_math.printHTM(T_B2O, "T_proc*T_B2O");

    pcl::transformPointCloud(*cloud_object_CAD_rgb, *cloud_object_CAD_rgb, T_B2O);
    for (size_t i = 0; i < cloud_object_CAD_rgb->size(); i++)
    {
        cloud_object_CAD_rgb->points[i].r = 1;
        cloud_object_CAD_rgb->points[i].g = 1;
        cloud_object_CAD_rgb->points[i].b = 255;
    }
    m_math.cloudScaling(*cloud_object_CAD_rgb, 0); // mm to m
    *cloud_env_base_frame_rgb_with_gripper += *cloud_object_CAD_rgb; // final results

    if (b_plot)
    {
        pcl::visualization::PCLVisualizer viewer(plot_name.c_str());
        int v1(0);
        int v2(1);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(1, 1, 1, v1);
        viewer.setBackgroundColor(1, 1, 1, v2);
        // CAD cloud is black
        viewer.addPointCloud(cloud_env_base_frame_rgb, "cloud_in_v1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
        viewer.addPointCloud(cloud_env_base_frame_rgb_with_gripper, "cloud_in_v2", v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

        //// Orientation
        Eigen::Vector4f grp_p0(0.0, 0.0, 0.0, 1.0);
        Eigen::Vector4f grp_p_base = T_B2E*grp_p0;

        Eigen::Vector3f grp_rx0(1.0, 0.0, 0.0);
        Eigen::Vector3f grp_ry0(0.0, 1.0, 0.0);
        Eigen::Vector3f grp_rz0(0.0, 0.0, 1.0);
        Eigen::Vector3f grp_rx_base = T_B2E.block<3,3>(0,0)*grp_rx0;
        Eigen::Vector3f grp_ry_base = T_B2E.block<3,3>(0,0)*grp_ry0;
        Eigen::Vector3f grp_rz_base = T_B2E.block<3,3>(0,0)*grp_rz0;

        float line_length = 75.0;
        Eigen::Vector3f grp_ppx = grp_p_base.block<3,1>(0,0) + line_length*grp_rx_base;
        Eigen::Vector3f grp_ppy = grp_p_base.block<3,1>(0,0) + line_length*grp_ry_base;
        Eigen::Vector3f grp_ppz = grp_p_base.block<3,1>(0,0) + line_length*grp_rz_base;

        // std::cout << "grp_p_base" << std::endl;
        // std::cout << grp_p_base << std::endl;
        // std::cout << "ppx ppy ppz" << std::endl;
        // std::cout << ppx << std::endl;
        // std::cout << ppy << std::endl;
        // std::cout << ppz << std::endl;

        pcl::PointXYZ grp_ppt0(0.001*grp_p_base(0), 0.001*grp_p_base(1), 0.001*grp_p_base(2));
        pcl::PointXYZ grp_pptx(0.001*grp_ppx(0), 0.001*grp_ppx(1), 0.001*grp_ppx(2));
        pcl::PointXYZ grp_ppty(0.001*grp_ppy(0), 0.001*grp_ppy(1), 0.001*grp_ppy(2));
        pcl::PointXYZ grp_pptz(0.001*grp_ppz(0), 0.001*grp_ppz(1), 0.001*grp_ppz(2));
        viewer.addLine(grp_ppt0, grp_pptx, 1.0, 0.0, 0.0, "x");
        viewer.addLine(grp_ppt0, grp_ppty, 0.0, 1.0, 0.0, "y");
        viewer.addLine(grp_ppt0, grp_pptz, 0.0, 0.0, 1.0, "z");

        // Set camera position
        float view_scale = 0.8;
        viewer.setCameraPosition(0.001*grp_p_base[0] + view_scale, 0.001*grp_p_base[1] + view_scale, 0.001*grp_p_base[2] + view_scale, 0, 0, 1);
        std::vector<pcl::visualization::Camera> cams;
        viewer.getCameras(cams);
        for (auto &&camera : cams)
        {
            for (int i = 0; i < 3; i++)
            {
                camera.focal[i] = 0.001*grp_p_base[i];
            }
        }
        viewer.setCameraParameters(cams[0]);

        //// Base frame
        Eigen::Vector3f p0(0.0, 0.0, 0.0);
        Eigen::Vector3f ppx0 = p0 + line_length*grp_rx0;
        Eigen::Vector3f ppy0 = p0 + line_length*grp_ry0;
        Eigen::Vector3f ppz0 = p0 + line_length*grp_rz0;
        pcl::PointXYZ ppt0(0.001*p0(0), 0.001*p0(1), 0.001*p0(2));
        pcl::PointXYZ pptx(0.001*ppx0(0), 0.001*ppx0(1), 0.001*ppx0(2));
        pcl::PointXYZ ppty(0.001*ppy0(0), 0.001*ppy0(1), 0.001*ppy0(2));
        pcl::PointXYZ pptz(0.001*ppz0(0), 0.001*ppz0(1), 0.001*ppz0(2));
        viewer.addLine(ppt0, pptx, 1.0, 0.0, 0.0, "x0");
        viewer.addLine(ppt0, ppty, 0.0, 1.0, 0.0, "y0");
        viewer.addLine(ppt0, pptz, 0.0, 0.0, 1.0, "z0");

        while (!viewer.wasStopped())
        {
            viewer.spin();
            // viewer.spinOnce(100);
            // boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }
    m_math.cloudScaling(*cloud_env_base_frame_rgb_with_gripper, 1); // m to mm
    *target_object_data->cloud_final_results_for_collision_check = *cloud_env_base_frame_rgb_with_gripper;
    return is_collision;
}

void PCLPROCESSING::printFPFHCompare(const pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_meas, const pcl::PointCloud<pcl::FPFHSignature33> &fpfhs_ref, const std::string &str, bool is_compare) {
  printf("------------------------------------------------\n");
  printf("%s\n", str.c_str());
  std::cout << std::fixed;
  std::cout.precision(1);
  for (size_t i = 0; i < 10; i++)
  {
    for (size_t j = 0; j < 33; j++)
    {
      if(is_compare) {
        std::cout << fabs(fpfhs_meas.points[i].histogram[j] - fpfhs_ref.points[i].histogram[j]) << " ";
      }
      else {
        std::cout << fabs(fpfhs_meas.points[i].histogram[j]) << " ";
      }
    }
    std::cout << std::endl;
  }
  printf("------------------------------------------------\n");
}

///////////////////////////////////////////////////////////////////////////////

void PCLPROCESSING::removeNanPoints(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
    ROS_LOG_INFO("Remove Nan points");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(cloud_in, *cloud_src);
    m_math.printCloudMinMax(*cloud_src, "raw_scan_cloud - before removing Nan points");

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud_src, *indices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_src);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cloud_filtered);
    m_math.printCloudMinMax(*cloud_filtered, "raw_scan_cloud - after removing Nan points");
    ROS_LOG_INFO("raw_scan_cloud size (after removing Nan points): %zu", cloud_filtered->size());
    
    // update
    cloud_out = *cloud_filtered;
}

void PCLPROCESSING::removeNanPoints(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out)
{
    ROS_LOG_INFO("Remove Nan points");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(cloud_in, *cloud_src);
    m_math.printCloudMinMax(*cloud_src, "raw_scan_cloud - before removing Nan points");

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud_src, *indices);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_src);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cloud_filtered);
    m_math.printCloudMinMax(*cloud_filtered, "raw_scan_cloud - after removing Nan points");
    ROS_LOG_INFO("raw_scan_cloud size (after removing Nan points): %zu", cloud_filtered->size());
    
    // update
    cloud_out = *cloud_filtered;
}

void PCLPROCESSING::removeNanPoints(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out)
{
    ROS_LOG_INFO("Remove Nan points");
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::copyPointCloud(cloud_in, *cloud_src);
    m_math.printCloudMinMax(*cloud_src, "raw_scan_cloud - before removing Nan points");

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud_src, *indices);
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
    extract.setInputCloud(cloud_src);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cloud_filtered);
    m_math.printCloudMinMax(*cloud_filtered, "raw_scan_cloud - after removing Nan points");
    ROS_LOG_INFO("raw_scan_cloud size (after removing Nan points): %zu", cloud_filtered->size());
    
    // update
    cloud_out = *cloud_filtered;
}

void PCLPROCESSING::removeNanPoints(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out)
{
    ROS_LOG_INFO("Remove Nan points");
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(cloud_in, *cloud_src);
    m_math.printCloudMinMax(*cloud_src, "raw_scan_cloud - before removing Nan points");

    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud_src, *indices);
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud(cloud_src);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cloud_filtered);
    m_math.printCloudMinMax(*cloud_filtered, "raw_scan_cloud - after removing Nan points");
    ROS_LOG_INFO("raw_scan_cloud size (after removing Nan points): %zu", cloud_filtered->size());
    
    // update
    cloud_out = *cloud_filtered;
}



void PCLPROCESSING::filterCloudUsingViewFrustum(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, const std::vector<double> view_frustum, pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
    //// TODO: Base frame이 아닌, Bin의 object frame으로 측정 점군을 변환한 뒤에, bin bounding box를 적용하기
    //// Alignment - measured cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(cloud_in, *cloud_src);
    // m_math.cloudScaling(*cloud_src, 0); // mm to m

    double bin_min_x = view_frustum[0];
    double bin_max_x = view_frustum[1];
    double bin_min_y = view_frustum[2];
    double bin_max_y = view_frustum[3];
    double bin_min_z = view_frustum[4];
    double bin_max_z = view_frustum[5];
    if (1)
    {
        printf("Pre-defined view frustum: (min,max): x: (%f, %f), y: (%f, %f), z: (%f, %f)\n", bin_min_x, bin_max_x, bin_min_y, bin_max_y, bin_min_z, bin_max_z);
        printf("\ncloud_base_aligned after_pass_through - bounding box size\n");

        for (size_t i = 0; i < view_frustum.size(); i++)
        {
            printf("%f ", view_frustum[i]);
        }
        printf("\n");
        // m_math.printCloudMinMax(*cloud_src, "cloud_filtered before pass through removal");
    }
    ///////////////////////////////////////////////////////
    //// plane removal filter - using bin bounding box info.
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud_src);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(bin_min_x, bin_max_x);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    filter.setInputCloud(cloud_filtered);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(bin_min_y, bin_max_y);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    filter.setInputCloud(cloud_filtered);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(bin_min_z, bin_max_z);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    // m_math.cloudScaling(*cloud_filtered, 1); // m to mm
    // update
    cloud_out = *cloud_filtered;

    m_math.printCloudMinMax(*cloud_filtered, "raw_scan_cloud - after filter using view frustum");
    ROS_LOG_INFO("raw_scan_cloud size (after filter using view frustum): %zu", cloud_filtered->size());

    if (0)
    {
        printf("code here 3!\n");
        double visualize_pt_size_1 = 2.5;
        double visualize_pt_size_2 = 5.0;
        pcl::visualization::PCLVisualizer viewer("Cloud before/after Bin Bounding Box Filtering");
        // m_math.cloudScaling(*cloud_filtered, 0); // mm to m

        ///////////////////////////////////////////
        printf("code here 4!\n");
        float view_scale = 0.8;
        Eigen::Vector4f centroid_CAD;
        pcl::compute3DCentroid(*cloud_filtered, centroid_CAD);
        viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
        std::vector<pcl::visualization::Camera> cams;
        viewer.getCameras(cams);
        for (auto &&camera : cams)
        {
            for (int i = 0; i < 3; i++)
            {
                camera.focal[i] = centroid_CAD[i];
            }
        }
        viewer.setCameraParameters(cams[0]);
        ///////////////////////////////////////////
        printf("code here 5!\n");

        int v1(0);
        int v2(1);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(1, 1, 1, v1);
        viewer.setBackgroundColor(1, 1, 1, v2);
        // CAD cloud is black
        printf("code here 6!\n");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud_src, 0, 0, 0);
        viewer.addPointCloud(cloud_src, cloud_in_color_h, "cloud_in_v1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
        viewer.addPointCloud(cloud_filtered, cloud_in_color_h, "cloud_in_v2", v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

        printf("code here 7!\n");
        //// Bounding box plot
        Eigen::Vector3f pp1(bin_max_x, bin_max_y, bin_min_z);
        Eigen::Vector3f pp2(bin_min_x, bin_max_y, bin_min_z);
        Eigen::Vector3f pp3(bin_min_x, bin_min_y, bin_min_z);
        Eigen::Vector3f pp4(bin_max_x, bin_min_y, bin_min_z);
        Eigen::Vector3f pp5(bin_max_x, bin_max_y, bin_max_z);
        Eigen::Vector3f pp6(bin_min_x, bin_max_y, bin_max_z);
        Eigen::Vector3f pp7(bin_min_x, bin_min_y, bin_max_z);
        Eigen::Vector3f pp8(bin_max_x, bin_min_y, bin_max_z);
        pcl::PointXYZ ppt1(pp1(0), pp1(1), pp1(2));
        pcl::PointXYZ ppt2(pp2(0), pp2(1), pp2(2));
        pcl::PointXYZ ppt3(pp3(0), pp3(1), pp3(2));
        pcl::PointXYZ ppt4(pp4(0), pp4(1), pp4(2));
        pcl::PointXYZ ppt5(pp5(0), pp5(1), pp5(2));
        pcl::PointXYZ ppt6(pp6(0), pp6(1), pp6(2));
        pcl::PointXYZ ppt7(pp7(0), pp7(1), pp7(2));
        pcl::PointXYZ ppt8(pp8(0), pp8(1), pp8(2));
        viewer.addLine(ppt1, ppt2, 1.0, 0.0, 0.0, "1p edge");
        viewer.addLine(ppt2, ppt3, 1.0, 0.0, 0.0, "2p edge");
        viewer.addLine(ppt3, ppt4, 1.0, 0.0, 0.0, "3p edge");
        viewer.addLine(ppt4, ppt1, 1.0, 0.0, 0.0, "4p edge");
        viewer.addLine(ppt5, ppt6, 1.0, 0.0, 0.0, "5p edge");
        viewer.addLine(ppt6, ppt7, 1.0, 0.0, 0.0, "6p edge");
        viewer.addLine(ppt7, ppt8, 1.0, 0.0, 0.0, "7p edge");
        viewer.addLine(ppt8, ppt5, 1.0, 0.0, 0.0, "8p edge");
        viewer.addLine(ppt1, ppt5, 1.0, 0.0, 0.0, "9p edge");
        viewer.addLine(ppt2, ppt6, 1.0, 0.0, 0.0, "10p edge");
        viewer.addLine(ppt3, ppt7, 1.0, 0.0, 0.0, "11p edge");
        viewer.addLine(ppt4, ppt8, 1.0, 0.0, 0.0, "12p edge");

        printf("code here 8!\n");

        while (!viewer.wasStopped())
        {
            // viewer.spinOnce(100);
            viewer.spin();
            // boost::this_thread::sleep(boost::posix_time::microseconds(10000));
        }
        viewer.close();
        printf("code here 9\n");

    }
}

void PCLPROCESSING::filterCloudUsingViewFrustum(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, const std::vector<double> view_frustum, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out)
{
    //// TODO: Base frame이 아닌, Bin의 object frame으로 측정 점군을 변환한 뒤에, bin bounding box를 적용하기
    //// Alignment - measured cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(cloud_in, *cloud_src);
    // m_math.cloudScaling(*cloud_src, 0); // mm to m

    double bin_min_x = view_frustum[0];
    double bin_max_x = view_frustum[1];
    double bin_min_y = view_frustum[2];
    double bin_max_y = view_frustum[3];
    double bin_min_z = view_frustum[4];
    double bin_max_z = view_frustum[5];
    if (1)
    {
        printf("Pre-defined view frustum: (min,max): x: (%f, %f), y: (%f, %f), z: (%f, %f)\n", bin_min_x, bin_max_x, bin_min_y, bin_max_y, bin_min_z, bin_max_z);
        printf("\ncloud_base_aligned after_pass_through - bounding box size\n");

        for (size_t i = 0; i < view_frustum.size(); i++)
        {
            printf("%f ", view_frustum[i]);
        }
        printf("\n");
        // m_math.printCloudMinMax(*cloud_src, "cloud_filtered before pass through removal");
    }
    ///////////////////////////////////////////////////////
    //// plane removal filter - using bin bounding box info.
    pcl::PassThrough<pcl::PointXYZRGB> filter;
    filter.setInputCloud(cloud_src);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(bin_min_x, bin_max_x);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    filter.setInputCloud(cloud_filtered);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(bin_min_y, bin_max_y);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    filter.setInputCloud(cloud_filtered);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(bin_min_z, bin_max_z);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    // m_math.cloudScaling(*cloud_filtered, 1); // m to mm

    // update
    cloud_out = *cloud_filtered;
 
    m_math.printCloudMinMax(*cloud_filtered, "raw_scan_cloud - after filter using view frustum");
    ROS_LOG_INFO("raw_scan_cloud size (after filter using view frustum): %zu", cloud_filtered->size());

    if (0)
    {
        printf("code here 3!\n");
        double visualize_pt_size_1 = 2.5;
        double visualize_pt_size_2 = 5.0;
        pcl::visualization::PCLVisualizer viewer("Cloud before/after Bin Bounding Box Filtering");
        // m_math.cloudScaling(*cloud_filtered, 0); // mm to m

        ///////////////////////////////////////////
        printf("code here 4!\n");
        float view_scale = 0.8;
        Eigen::Vector4f centroid_CAD;
        pcl::compute3DCentroid(*cloud_filtered, centroid_CAD);
        viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
        std::vector<pcl::visualization::Camera> cams;
        viewer.getCameras(cams);
        for (auto &&camera : cams)
        {
            for (int i = 0; i < 3; i++)
            {
                camera.focal[i] = centroid_CAD[i];
            }
        }
        viewer.setCameraParameters(cams[0]);
        ///////////////////////////////////////////
        printf("code here 5!\n");

        int v1(0);
        int v2(1);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(1, 1, 1, v1);
        viewer.setBackgroundColor(1, 1, 1, v2);
        // CAD cloud is black
        printf("code here 6!\n");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_in_color_h(cloud_src, 0, 0, 0);
        viewer.addPointCloud(cloud_src, cloud_in_color_h, "cloud_in_v1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
        viewer.addPointCloud(cloud_filtered, cloud_in_color_h, "cloud_in_v2", v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

        printf("code here 7!\n");
        //// Bounding box plot
        Eigen::Vector3f pp1(bin_max_x, bin_max_y, bin_min_z);
        Eigen::Vector3f pp2(bin_min_x, bin_max_y, bin_min_z);
        Eigen::Vector3f pp3(bin_min_x, bin_min_y, bin_min_z);
        Eigen::Vector3f pp4(bin_max_x, bin_min_y, bin_min_z);
        Eigen::Vector3f pp5(bin_max_x, bin_max_y, bin_max_z);
        Eigen::Vector3f pp6(bin_min_x, bin_max_y, bin_max_z);
        Eigen::Vector3f pp7(bin_min_x, bin_min_y, bin_max_z);
        Eigen::Vector3f pp8(bin_max_x, bin_min_y, bin_max_z);
        pcl::PointXYZ ppt1(pp1(0), pp1(1), pp1(2));
        pcl::PointXYZ ppt2(pp2(0), pp2(1), pp2(2));
        pcl::PointXYZ ppt3(pp3(0), pp3(1), pp3(2));
        pcl::PointXYZ ppt4(pp4(0), pp4(1), pp4(2));
        pcl::PointXYZ ppt5(pp5(0), pp5(1), pp5(2));
        pcl::PointXYZ ppt6(pp6(0), pp6(1), pp6(2));
        pcl::PointXYZ ppt7(pp7(0), pp7(1), pp7(2));
        pcl::PointXYZ ppt8(pp8(0), pp8(1), pp8(2));
        viewer.addLine(ppt1, ppt2, 1.0, 0.0, 0.0, "1p edge");
        viewer.addLine(ppt2, ppt3, 1.0, 0.0, 0.0, "2p edge");
        viewer.addLine(ppt3, ppt4, 1.0, 0.0, 0.0, "3p edge");
        viewer.addLine(ppt4, ppt1, 1.0, 0.0, 0.0, "4p edge");
        viewer.addLine(ppt5, ppt6, 1.0, 0.0, 0.0, "5p edge");
        viewer.addLine(ppt6, ppt7, 1.0, 0.0, 0.0, "6p edge");
        viewer.addLine(ppt7, ppt8, 1.0, 0.0, 0.0, "7p edge");
        viewer.addLine(ppt8, ppt5, 1.0, 0.0, 0.0, "8p edge");
        viewer.addLine(ppt1, ppt5, 1.0, 0.0, 0.0, "9p edge");
        viewer.addLine(ppt2, ppt6, 1.0, 0.0, 0.0, "10p edge");
        viewer.addLine(ppt3, ppt7, 1.0, 0.0, 0.0, "11p edge");
        viewer.addLine(ppt4, ppt8, 1.0, 0.0, 0.0, "12p edge");

        printf("code here 8!\n");

        while (!viewer.wasStopped())
        {
            // viewer.spinOnce(100);
            viewer.spin();
            // boost::this_thread::sleep(boost::posix_time::microseconds(10000));
        }
        viewer.close();
        printf("code here 9\n");

    }
}

void PCLPROCESSING::filterCloudUsingViewFrustum(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, const std::vector<double> view_frustum, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out)
{
    //// TODO: Base frame이 아닌, Bin의 object frame으로 측정 점군을 변환한 뒤에, bin bounding box를 적용하기
    //// Alignment - measured cloud
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::copyPointCloud(cloud_in, *cloud_src);
    // m_math.cloudScaling(*cloud_src, 0); // mm to m

    double bin_min_x = view_frustum[0];
    double bin_max_x = view_frustum[1];
    double bin_min_y = view_frustum[2];
    double bin_max_y = view_frustum[3];
    double bin_min_z = view_frustum[4];
    double bin_max_z = view_frustum[5];
    if (1)
    {
        printf("Pre-defined view frustum: (min,max): x: (%f, %f), y: (%f, %f), z: (%f, %f)\n", bin_min_x, bin_max_x, bin_min_y, bin_max_y, bin_min_z, bin_max_z);
        printf("\ncloud_base_aligned after_pass_through - bounding box size\n");

        for (size_t i = 0; i < view_frustum.size(); i++)
        {
            printf("%f ", view_frustum[i]);
        }
        printf("\n");
        // m_math.printCloudMinMax(*cloud_src, "cloud_filtered before pass through removal");
    }
    ///////////////////////////////////////////////////////
    //// plane removal filter - using bin bounding box info.
    pcl::PassThrough<pcl::PointXYZRGBNormal> filter;
    filter.setInputCloud(cloud_src);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(bin_min_x, bin_max_x);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    filter.setInputCloud(cloud_filtered);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(bin_min_y, bin_max_y);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    filter.setInputCloud(cloud_filtered);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(bin_min_z, bin_max_z);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    // m_math.cloudScaling(*cloud_filtered, 1); // m to mm

    // update
    cloud_out = *cloud_filtered;
 
    m_math.printCloudMinMax(*cloud_filtered, "raw_scan_cloud - after filter using view frustum");
    ROS_LOG_INFO("raw_scan_cloud size (after filter using view frustum): %zu", cloud_filtered->size());

    if (0)
    {
        printf("code here 3!\n");
        double visualize_pt_size_1 = 2.5;
        double visualize_pt_size_2 = 5.0;
        pcl::visualization::PCLVisualizer viewer("Cloud before/after Bin Bounding Box Filtering");
        // m_math.cloudScaling(*cloud_filtered, 0); // mm to m

        ///////////////////////////////////////////
        printf("code here 4!\n");
        float view_scale = 0.8;
        Eigen::Vector4f centroid_CAD;
        pcl::compute3DCentroid(*cloud_filtered, centroid_CAD);
        viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
        std::vector<pcl::visualization::Camera> cams;
        viewer.getCameras(cams);
        for (auto &&camera : cams)
        {
            for (int i = 0; i < 3; i++)
            {
                camera.focal[i] = centroid_CAD[i];
            }
        }
        viewer.setCameraParameters(cams[0]);
        ///////////////////////////////////////////
        printf("code here 5!\n");

        int v1(0);
        int v2(1);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(1, 1, 1, v1);
        viewer.setBackgroundColor(1, 1, 1, v2);
        // CAD cloud is black
        printf("code here 6!\n");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> cloud_in_color_h(cloud_src, 0, 0, 0);
        viewer.addPointCloud(cloud_src, cloud_in_color_h, "cloud_in_v1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
        viewer.addPointCloud(cloud_filtered, cloud_in_color_h, "cloud_in_v2", v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

        printf("code here 7!\n");
        //// Bounding box plot
        Eigen::Vector3f pp1(bin_max_x, bin_max_y, bin_min_z);
        Eigen::Vector3f pp2(bin_min_x, bin_max_y, bin_min_z);
        Eigen::Vector3f pp3(bin_min_x, bin_min_y, bin_min_z);
        Eigen::Vector3f pp4(bin_max_x, bin_min_y, bin_min_z);
        Eigen::Vector3f pp5(bin_max_x, bin_max_y, bin_max_z);
        Eigen::Vector3f pp6(bin_min_x, bin_max_y, bin_max_z);
        Eigen::Vector3f pp7(bin_min_x, bin_min_y, bin_max_z);
        Eigen::Vector3f pp8(bin_max_x, bin_min_y, bin_max_z);
        pcl::PointXYZ ppt1(pp1(0), pp1(1), pp1(2));
        pcl::PointXYZ ppt2(pp2(0), pp2(1), pp2(2));
        pcl::PointXYZ ppt3(pp3(0), pp3(1), pp3(2));
        pcl::PointXYZ ppt4(pp4(0), pp4(1), pp4(2));
        pcl::PointXYZ ppt5(pp5(0), pp5(1), pp5(2));
        pcl::PointXYZ ppt6(pp6(0), pp6(1), pp6(2));
        pcl::PointXYZ ppt7(pp7(0), pp7(1), pp7(2));
        pcl::PointXYZ ppt8(pp8(0), pp8(1), pp8(2));
        viewer.addLine(ppt1, ppt2, 1.0, 0.0, 0.0, "1p edge");
        viewer.addLine(ppt2, ppt3, 1.0, 0.0, 0.0, "2p edge");
        viewer.addLine(ppt3, ppt4, 1.0, 0.0, 0.0, "3p edge");
        viewer.addLine(ppt4, ppt1, 1.0, 0.0, 0.0, "4p edge");
        viewer.addLine(ppt5, ppt6, 1.0, 0.0, 0.0, "5p edge");
        viewer.addLine(ppt6, ppt7, 1.0, 0.0, 0.0, "6p edge");
        viewer.addLine(ppt7, ppt8, 1.0, 0.0, 0.0, "7p edge");
        viewer.addLine(ppt8, ppt5, 1.0, 0.0, 0.0, "8p edge");
        viewer.addLine(ppt1, ppt5, 1.0, 0.0, 0.0, "9p edge");
        viewer.addLine(ppt2, ppt6, 1.0, 0.0, 0.0, "10p edge");
        viewer.addLine(ppt3, ppt7, 1.0, 0.0, 0.0, "11p edge");
        viewer.addLine(ppt4, ppt8, 1.0, 0.0, 0.0, "12p edge");

        printf("code here 8!\n");

        while (!viewer.wasStopped())
        {
            // viewer.spinOnce(100);
            viewer.spin();
            // boost::this_thread::sleep(boost::posix_time::microseconds(10000));
        }
        viewer.close();
        printf("code here 9\n");

    }
}

void PCLPROCESSING::filterCloudUsingViewFrustum(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, const std::vector<double> view_frustum, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out)
{
    //// TODO: Base frame이 아닌, Bin의 object frame으로 측정 점군을 변환한 뒤에, bin bounding box를 적용하기
    //// Alignment - measured cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(cloud_in, *cloud_src);
    // m_math.cloudScaling(*cloud_src, 0); // mm to m

    double bin_min_x = view_frustum[0];
    double bin_max_x = view_frustum[1];
    double bin_min_y = view_frustum[2];
    double bin_max_y = view_frustum[3];
    double bin_min_z = view_frustum[4];
    double bin_max_z = view_frustum[5];
    if (1)
    {
        printf("Pre-defined view frustum: (min,max): x: (%f, %f), y: (%f, %f), z: (%f, %f)\n", bin_min_x, bin_max_x, bin_min_y, bin_max_y, bin_min_z, bin_max_z);
        printf("\ncloud_base_aligned after_pass_through - bounding box size\n");

        for (size_t i = 0; i < view_frustum.size(); i++)
        {
            printf("%f ", view_frustum[i]);
        }
        printf("\n");
        // m_math.printCloudMinMax(*cloud_src, "cloud_filtered before pass through removal");
    }
    ///////////////////////////////////////////////////////
    //// plane removal filter - using bin bounding box info.
    pcl::PassThrough<pcl::PointXYZRGBA> filter;
    filter.setInputCloud(cloud_src);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(bin_min_x, bin_max_x);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    filter.setInputCloud(cloud_filtered);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(bin_min_y, bin_max_y);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    filter.setInputCloud(cloud_filtered);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(bin_min_z, bin_max_z);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    // m_math.cloudScaling(*cloud_filtered, 1); // m to mm

    // update
    cloud_out = *cloud_filtered;
 
    m_math.printCloudMinMax(*cloud_filtered, "raw_scan_cloud - after filter using view frustum");
    ROS_LOG_INFO("raw_scan_cloud size (after filter using view frustum): %zu", cloud_filtered->size());

    if (0)
    {
        printf("code here 3!\n");
        double visualize_pt_size_1 = 2.5;
        double visualize_pt_size_2 = 5.0;
        pcl::visualization::PCLVisualizer viewer("Cloud before/after Bin Bounding Box Filtering");
        // m_math.cloudScaling(*cloud_filtered, 0); // mm to m

        ///////////////////////////////////////////
        printf("code here 4!\n");
        float view_scale = 0.8;
        Eigen::Vector4f centroid_CAD;
        pcl::compute3DCentroid(*cloud_filtered, centroid_CAD);
        viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
        std::vector<pcl::visualization::Camera> cams;
        viewer.getCameras(cams);
        for (auto &&camera : cams)
        {
            for (int i = 0; i < 3; i++)
            {
                camera.focal[i] = centroid_CAD[i];
            }
        }
        viewer.setCameraParameters(cams[0]);
        ///////////////////////////////////////////
        printf("code here 5!\n");

        int v1(0);
        int v2(1);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(1, 1, 1, v1);
        viewer.setBackgroundColor(1, 1, 1, v2);
        // CAD cloud is black
        printf("code here 6!\n");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> cloud_in_color_h(cloud_src, 0, 0, 0);
        viewer.addPointCloud(cloud_src, cloud_in_color_h, "cloud_in_v1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
        viewer.addPointCloud(cloud_filtered, cloud_in_color_h, "cloud_in_v2", v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

        printf("code here 7!\n");
        //// Bounding box plot
        Eigen::Vector3f pp1(bin_max_x, bin_max_y, bin_min_z);
        Eigen::Vector3f pp2(bin_min_x, bin_max_y, bin_min_z);
        Eigen::Vector3f pp3(bin_min_x, bin_min_y, bin_min_z);
        Eigen::Vector3f pp4(bin_max_x, bin_min_y, bin_min_z);
        Eigen::Vector3f pp5(bin_max_x, bin_max_y, bin_max_z);
        Eigen::Vector3f pp6(bin_min_x, bin_max_y, bin_max_z);
        Eigen::Vector3f pp7(bin_min_x, bin_min_y, bin_max_z);
        Eigen::Vector3f pp8(bin_max_x, bin_min_y, bin_max_z);
        pcl::PointXYZ ppt1(pp1(0), pp1(1), pp1(2));
        pcl::PointXYZ ppt2(pp2(0), pp2(1), pp2(2));
        pcl::PointXYZ ppt3(pp3(0), pp3(1), pp3(2));
        pcl::PointXYZ ppt4(pp4(0), pp4(1), pp4(2));
        pcl::PointXYZ ppt5(pp5(0), pp5(1), pp5(2));
        pcl::PointXYZ ppt6(pp6(0), pp6(1), pp6(2));
        pcl::PointXYZ ppt7(pp7(0), pp7(1), pp7(2));
        pcl::PointXYZ ppt8(pp8(0), pp8(1), pp8(2));
        viewer.addLine(ppt1, ppt2, 1.0, 0.0, 0.0, "1p edge");
        viewer.addLine(ppt2, ppt3, 1.0, 0.0, 0.0, "2p edge");
        viewer.addLine(ppt3, ppt4, 1.0, 0.0, 0.0, "3p edge");
        viewer.addLine(ppt4, ppt1, 1.0, 0.0, 0.0, "4p edge");
        viewer.addLine(ppt5, ppt6, 1.0, 0.0, 0.0, "5p edge");
        viewer.addLine(ppt6, ppt7, 1.0, 0.0, 0.0, "6p edge");
        viewer.addLine(ppt7, ppt8, 1.0, 0.0, 0.0, "7p edge");
        viewer.addLine(ppt8, ppt5, 1.0, 0.0, 0.0, "8p edge");
        viewer.addLine(ppt1, ppt5, 1.0, 0.0, 0.0, "9p edge");
        viewer.addLine(ppt2, ppt6, 1.0, 0.0, 0.0, "10p edge");
        viewer.addLine(ppt3, ppt7, 1.0, 0.0, 0.0, "11p edge");
        viewer.addLine(ppt4, ppt8, 1.0, 0.0, 0.0, "12p edge");

        printf("code here 8!\n");

        while (!viewer.wasStopped())
        {
            // viewer.spinOnce(100);
            viewer.spin();
            // boost::this_thread::sleep(boost::posix_time::microseconds(10000));
        }
        viewer.close();
        printf("code here 9\n");

    }
}
