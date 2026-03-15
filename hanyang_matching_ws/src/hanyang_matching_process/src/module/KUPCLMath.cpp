/**
 * @file KUPCLMATH.cpp
 * @brief 검사 자동화 시스템의 연산을 위한 구현파일
 */
#include "module/KUPCLMath.h"
#include <string>
#include <iostream>
#include <math.h>

#define DEG2RAD(A) (A)*3.141592 / 180.0
#define RAD2DEG(A) (A)*180.0 / 3.141592

KUPCLMATH::KUPCLMATH()
{
    m_TCP = Eigen::Matrix4d::Identity();
}

KUPCLMATH::~KUPCLMATH()
{

}


inline bool sortVec(const Eigen::Vector3d &lhs, const Eigen::Vector3d &rhs) {
	if (lhs.x() < rhs.x()) return true;
	if (rhs.x() < lhs.x()) return false;
	if (lhs.y() < rhs.y()) return true;
	if (rhs.y() < lhs.y())  return false;
	return lhs.z() < rhs.z();
}

inline bool cmpVec(const Eigen::Vector3d &lhs, const Eigen::Vector3d &rhs) {
	return lhs.x() == rhs.x() && lhs.y() == rhs.y() && lhs.z() == rhs.z();
}

inline bool sortVec_2D(const Eigen::Vector2i &lhs, const Eigen::Vector2i &rhs) {
	if (lhs.x() < rhs.x()) return true;
	if (rhs.x() < lhs.x()) return false;
	return lhs.y() < rhs.y();
}

inline bool cmpVec_2D(const Eigen::Vector2i &lhs, const Eigen::Vector2i &rhs) {
	return lhs.x() == rhs.x() && lhs.y() == rhs.y();
}

/** @brief Korean: 입력된 배열에서 중복 값을 제거한다. 
 * @param[in] input : std::vector<Eigen::Vector2i> 형식의 배열
 */
void KUPCLMATH::removeDuplicates_2D(std::vector<Eigen::Vector2i> &con)
{
	std::sort(con.data(), con.data() + con.size(), sortVec_2D);
	con.erase(std::unique(con.begin(), con.end(), cmpVec_2D), con.end());
}

/** @brief Korean: 입력된 배열에서 중복 값을 제거한다. 
 * @param[in] input : std::vector 형식의 배열
 */
void KUPCLMATH::removeDuplicates(std::vector<Eigen::Vector3d> &con)
{
	std::sort(con.data(), con.data() + con.size(), sortVec);
	con.erase(std::unique(con.begin(), con.end(), cmpVec), con.end());
}

/** @brief Korean: 입력된 배열에서 중복 값을 제거한다. 
 * @param[in] input : std::vector 형식의 배열
 */
void KUPCLMATH::removeDuplicates(std::vector<int> &con)
{
	std::sort(con.data(), con.data() + con.size());
	con.erase(std::unique(con.begin(), con.end()), con.end());
}

/** @brief Korean: 입력된 배열에서 중복 값을 제거한다. 
 * @param[in] input : std::vector 형식의 배열
 */
void KUPCLMATH::removeDuplicates(std::vector<unsigned int> &con)
{
	std::sort(con.data(), con.data() + con.size());
	con.erase(std::unique(con.begin(), con.end()), con.end());
}

/** @brief Korean: 입력된 배열에서 중복 값을 제거한다.
* @param[in] input : std::vector 형식의 배열
*/
void KUPCLMATH::removeDuplicates(std::vector<size_t> &con)
{
	std::sort(con.data(), con.data() + con.size());
	con.erase(std::unique(con.begin(), con.end()), con.end());
}

/** @brief Korean: 입력된 점군 데이터에서 중복 값을 제거하여 점군 데이터를 추출한다. 
 * @param[in] input : 입력 점군 데이터
 * @param[out] output : 중복 값이 제거된 점군 데이터
 */
void KUPCLMATH::removeDuplicatesCloud(const pcl::PointCloud<pcl::PointXYZ> &input, pcl::PointCloud<pcl::PointXYZ> &output)
{
	std::vector<Eigen::Vector3d> pt_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
	for (int i = 0; i < input.size(); i++)
	{
		Eigen::Vector3d pt_tmp = { input.points[i].x, input.points[i].y, input.points[i].z };
		pt_cloud.push_back(pt_tmp);
	}

	removeDuplicates(pt_cloud); // duplicates removal 
	vec2cloud(pt_cloud, *cloud_tmp);

	pcl::copyPointCloud(*cloud_tmp, output);
}

/** @brief Korean: 입력된 점군 데이터에서 중복 값을 제거하여 점군 데이터를 추출한다. 
 * @param[in] input : 입력 점군 데이터
 * @param[out] output : 중복 값이 제거된 점군 데이터
 */
void KUPCLMATH::removeDuplicatesCloud(const pcl::PointCloud<pcl::PointXYZLNormal> &input, pcl::PointCloud<pcl::PointXYZLNormal> &output)
{
	std::vector<Eigen::Vector3d> pt_cloud;
	pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZLNormal>);
	for (int i = 0; i < input.size(); i++)
	{
		Eigen::Vector3d pt_tmp = { input.points[i].x, input.points[i].y, input.points[i].z };
		pt_cloud.push_back(pt_tmp);
	}

	removeDuplicates(pt_cloud); // duplicates removal 
	vec2cloud(pt_cloud, *cloud_tmp);

	pcl::copyPointCloud(*cloud_tmp, output);
}

/** @brief Korean: 입력된 점군 데이터에서 중복 값을 제거하여 점군 데이터를 추출한다. 
 * @param[in] input : 입력 점군 데이터
 * @param[out] output : 중복 값이 제거된 점군 데이터
 */
void KUPCLMATH::removeDuplicatesCloud(const pcl::PointCloud<pcl::PointXYZRGBA> &input, pcl::PointCloud<pcl::PointXYZRGBA> &output)
{
	std::vector<Eigen::Vector3d> pt_cloud;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
	for (int i = 0; i < input.size(); i++)
	{
		Eigen::Vector3d pt_tmp = { input.points[i].x, input.points[i].y, input.points[i].z };
		pt_cloud.push_back(pt_tmp);
	}

	removeDuplicates(pt_cloud); // duplicates removal 
	vec2cloud(pt_cloud, *cloud_tmp);

	pcl::copyPointCloud(*cloud_tmp, output);
}

/** @brief Korean: 입력된 인덱스 사이의 중복된 인덱스를 추출한다.
* @param[in] v1 : 입력 인덱스 1
* @param[in] v2 : 입력 인덱스 2
*/
std::vector<int> KUPCLMATH::intersection(std::vector<int> &v1, std::vector<int> &v2)
{
	std::vector<int> v3;

	std::sort(v1.begin(), v1.end());
	std::sort(v2.begin(), v2.end());

	std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), back_inserter(v3));
	return v3;
}

/** @brief Korean: std::vector 형식의 점군 데이터를 pcl::PointCloud 형식으로 변환한다.
 * @param[in] input : std::vector 형식의 점군 데이터
 * @param[out] output : pcl::PointCloud 형식의 점군 데이터
 */
void KUPCLMATH::vec2cloud(const std::vector<Eigen::Vector3d>& input, pcl::PointCloud<pcl::PointXYZ> &output)
{
	output.resize(input.size());
	for (int i = 0; i < input.size(); i++)
	{
		Eigen::Vector3d vec_tmp = input[i];
		output.points[i].x = vec_tmp[0];
		output.points[i].y = vec_tmp[1];
		output.points[i].z = vec_tmp[2];
	}
}

/** @brief Korean: std::vector 형식의 점군 데이터를 pcl::PointCloud 형식으로 변환한다.
 * @param[in] input : std::vector 형식의 점군 데이터
 * @param[out] output : pcl::PointCloud 형식의 점군 데이터
 */
void KUPCLMATH::vec2cloud(const std::vector<Eigen::Vector3d>& input, pcl::PointCloud<pcl::PointXYZRGBNormal> &output)
{
	output.resize(input.size());
	for (int i = 0; i < input.size(); i++)
	{
		Eigen::Vector3d vec_tmp = input[i];
		output.points[i].x = vec_tmp[0];
		output.points[i].y = vec_tmp[1];
		output.points[i].z = vec_tmp[2];
	}
}

/** @brief Korean: std::vector 형식의 점군 데이터를 pcl::PointCloud 형식으로 변환한다.
 * @param[in] input : std::vector 형식의 점군 데이터
 * @param[out] output : pcl::PointCloud 형식의 점군 데이터
 */
void KUPCLMATH::vec2cloud(const std::vector<Eigen::Vector3d>& input, pcl::PointCloud<pcl::PointXYZLNormal> &output)
{
	output.resize(input.size());
	for (int i = 0; i < input.size(); i++)
	{
		Eigen::Vector3d vec_tmp = input[i];
		output.points[i].x = vec_tmp[0];
		output.points[i].y = vec_tmp[1];
		output.points[i].z = vec_tmp[2];
	}
}

/** @brief Korean: std::vector 형식의 점군 데이터를 pcl::PointCloud 형식으로 변환한다.
 * @param[in] input : std::vector 형식의 점군 데이터
 * @param[out] output : pcl::PointCloud 형식의 점군 데이터
 */
void KUPCLMATH::vec2cloud(const std::vector<Eigen::Vector3d>& input, pcl::PointCloud<pcl::PointXYZRGBA> &output)
{
	output.resize(input.size());
	for (int i = 0; i < input.size(); i++)
	{
		Eigen::Vector3d vec_tmp = input[i];
		output.points[i].x = vec_tmp[0];
		output.points[i].y = vec_tmp[1];
		output.points[i].z = vec_tmp[2];
	}
}

/** @brief Korean: Eigen::Vector3d 형식의 점군 데이터를 pcl::PointCloud 형식으로 변환한다.
 * @param[in] input : Eigen::Vector3d 형식의 점군 데이터
 * @param[out] output : pcl::PointCloud 형식의 점군 데이터
 */
void KUPCLMATH::pt2cloud(const Eigen::Vector3d& input, pcl::PointCloud<pcl::PointXYZ> &output)
{
	output.resize(1);
	output.points[0].x = input[0];
	output.points[0].y = input[1];
	output.points[0].z = input[2];
}

/** @brief Korean: pcl::PointCloud 형식의 점군의 입력 인덱스의 점을 Eigen::Vector3d 형식으로 변환한다.
* @param[in] input : pcl::PointCloud 형식의 점군 데이터
* @return output: Eigen::Vector3d 형식의 점군 데이터
*/
Eigen::Vector3d KUPCLMATH::cloud2pt(const pcl::PointCloud<pcl::PointXYZ> &input, const size_t &idx)
{
	Eigen::Vector3d output;
	output[0] = input.points[idx].x;
	output[1] = input.points[idx].y;
	output[2] = input.points[idx].z;
	return output;
}

/** @brief Korean: pcl::PointCloud 형식의 점군의 입력 인덱스의 점을 Eigen::Vector3d 형식으로 변환한다.
* @param[in] input : pcl::PointCloud 형식의 점군 데이터
* @return output: Eigen::Vector3d 형식의 점군 데이터
*/
Eigen::Vector3d KUPCLMATH::cloud2pt(const pcl::PointCloud<pcl::PointXYZRGB> &input, const size_t &idx)
{
	Eigen::Vector3d output;
	output[0] = input.points[idx].x;
	output[1] = input.points[idx].y;
	output[2] = input.points[idx].z;
	return output;
}

/** @brief Korean: pcl::PointCloud 형식의 점군의 입력 인덱스의 점을 Eigen::Vector3d 형식으로 변환한다.
* @param[in] input : pcl::PointCloud 형식의 점군 데이터
* @return output: Eigen::Vector3d 형식의 점군 데이터
*/
Eigen::Vector3d KUPCLMATH::cloud2pt(const pcl::PointCloud<pcl::PointXYZRGBNormal> &input, const size_t &idx)
{
	Eigen::Vector3d output;
	output[0] = input.points[idx].x;
	output[1] = input.points[idx].y;
	output[2] = input.points[idx].z;
	return output;
}

/** @brief Korean: pcl::PointCloud 형식의 점군의 입력 인덱스의 점을 Eigen::Vector3d 형식으로 변환한다.
* @param[in] input : pcl::PointCloud 형식의 점군 데이터
* @return output: Eigen::Vector3d 형식의 점군 데이터
*/
Eigen::Vector3d KUPCLMATH::cloud2pt(const pcl::PointCloud<pcl::PointXYZLNormal> &input, const size_t &idx)
{
	Eigen::Vector3d output;
	output[0] = input.points[idx].x;
	output[1] = input.points[idx].y;
	output[2] = input.points[idx].z;
	return output;
}

/** @brief Korean: pcl::PointCloud 형식의 점군의 입력 인덱스의 점에 대한 Eigen::Vector3d 형식의 점과 법선을 추출한다.
* @param[in] input : pcl::PointCloud 형식의 점군 데이터
* @return output: Eigen::Vector3d 형식의 점군 데이터
*/
void KUPCLMATH::cloud2ptNormal(const pcl::PointCloud<pcl::PointXYZRGBNormal> &input, const size_t &idx, Eigen::Vector3d &pt, Eigen::Vector3d &normal)
{
	pt[0] = input.points[idx].x;
	pt[1] = input.points[idx].y;
	pt[2] = input.points[idx].z;
	normal[0] = input.points[idx].normal_x;
	normal[1] = input.points[idx].normal_y;
	normal[2] = input.points[idx].normal_z;
}

/** @brief Korean: pcl::PointCloud 형식의 점군의 입력 인덱스의 점에 대한 Eigen::Vector3d 형식의 점과 법선을 추출한다.
* @param[in] input : pcl::PointCloud 형식의 점군 데이터
* @return output: Eigen::Vector3d 형식의 점군 데이터
*/
void KUPCLMATH::cloud2ptNormal(const pcl::PointCloud<pcl::PointXYZLNormal> &input, const size_t &idx, Eigen::Vector3d &pt, Eigen::Vector3d &normal)
{
	pt[0] = input.points[idx].x;
	pt[1] = input.points[idx].y;
	pt[2] = input.points[idx].z;
	normal[0] = input.points[idx].normal_x;
	normal[1] = input.points[idx].normal_y;
	normal[2] = input.points[idx].normal_z;
}

/** @brief Korean: 입력 점군 데이터에서 인덱스에 해당하는 점군 데이터를 추출한다.
 * @param[in] input : 입력 점군 데이터
 * @param[in] idx : 입력 점군 데이터에 해당하는 인덱스
 * @param[out] output : 출력 점군 데이터
 */
void KUPCLMATH::extractCloud(const pcl::PointCloud<pcl::PointXYZ> &input, const pcl::PointIndices &idx, pcl::PointCloud<pcl::PointXYZ> &output)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_query(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(input, *cloud_query);
	pcl::PointIndices::Ptr idx_query(new pcl::PointIndices);
	*idx_query = idx;

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud <pcl::PointXYZ>);
	extract.setInputCloud(cloud_query);
	extract.setIndices(idx_query);
	extract.setNegative(false);
	extract.filter(*cloud_output); // boundary point cloud

	output = *cloud_output;
}

/** @brief Korean: 입력 점군 데이터에서 인덱스에 해당하는 점군 데이터를 추출한다.
 * @param[in] input : 입력 점군 데이터
 * @param[in] idx : 입력 점군 데이터에 해당하는 인덱스
 * @param[out] output : 출력 점군 데이터
 */
void KUPCLMATH::extractCloud(const pcl::PointCloud <pcl::Normal> &input, const pcl::PointIndices &idx, pcl::PointCloud <pcl::Normal> &output)
{
	pcl::PointCloud <pcl::Normal>::Ptr cloud_query(new pcl::PointCloud <pcl::Normal>);
	pcl::copyPointCloud(input, *cloud_query);
	pcl::PointIndices::Ptr idx_query(new pcl::PointIndices);
	*idx_query = idx;

	pcl::ExtractIndices<pcl::Normal> extract;
	pcl::PointCloud <pcl::Normal>::Ptr cloud_output(new pcl::PointCloud <pcl::Normal>);
	extract.setInputCloud(cloud_query);
	extract.setIndices(idx_query);
	extract.setNegative(false);
	extract.filter(*cloud_output); // boundary point cloud

	output = *cloud_output;
}

/** @brief Korean: 입력 점군 데이터에서 인덱스에 해당하는 점군 데이터를 추출한다.
 * @param[in] input : 입력 점군 데이터
 * @param[in] idx : 입력 점군 데이터에 해당하는 인덱스
 * @param[out] output : 출력 점군 데이터
 */
void KUPCLMATH::extractCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> &input, const pcl::PointIndices &idx, pcl::PointCloud<pcl::PointXYZ> &output)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_query(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(input, *cloud_query);
	pcl::PointIndices::Ptr idx_query(new pcl::PointIndices);
	*idx_query = idx;

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_output(new pcl::PointCloud <pcl::PointXYZ>);
	extract.setInputCloud(cloud_query);
	extract.setIndices(idx_query);
	extract.setNegative(false);
	extract.filter(*cloud_output); // boundary point cloud

	output = *cloud_output;
}

/** @brief Korean: 입력 점군 데이터에서 인덱스에 해당하는 점군 데이터를 추출한다.
 * @param[in] input : 입력 점군 데이터
 * @param[in] idx : 점군 데이터의 인덱스
 * @param[out] output : 출력 점군 데이터
 */
void KUPCLMATH::extractCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal> &input, const pcl::PointIndices &idx, pcl::PointCloud<pcl::PointXYZRGBNormal> &output)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_query(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::copyPointCloud(input, *cloud_query);
	pcl::PointIndices::Ptr idx_query(new pcl::PointIndices);
	*idx_query = idx;

	pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
	pcl::PointCloud <pcl::PointXYZRGBNormal>::Ptr cloud_output(new pcl::PointCloud <pcl::PointXYZRGBNormal>);
	extract.setInputCloud(cloud_query);
	extract.setIndices(idx_query);
	extract.setNegative(false);
	extract.filter(*cloud_output); // boundary point cloud

	output = *cloud_output;
}

/** @brief Korean: 입력된 인덱스에서 라벨에 해당하는 인덱스를 추출한다.
 * @param[in] input : 입력 인덱스
 * @param[in] label : label 형태의 인덱스
 * @param[out] output : 출력 인덱스
 */
void KUPCLMATH::extractSegmentIdx(const std::vector<int> &input, const pcl::PointCloud<pcl::PointXYZLNormal> &label, std::vector<int> &output)
{
	for (int i = 0; i < label.size(); i++)
	{
		int idx_tmp = label.points[i].label;
		output.push_back(input[idx_tmp]);
	}
}

/** @brief Korean: 입력된 인덱스에서 라벨에 해당하는 인덱스를 추출한다.
* @param[in] input : 입력 인덱스
* @param[in] label : label 형태의 인덱스
* @param[out] output : 출력 인덱스
*/
void KUPCLMATH::extractSegmentIdx(const std::vector<size_t> &input, const pcl::PointCloud<pcl::PointXYZLNormal> &label, std::vector<size_t> &output)
{
	for (size_t i = 0; i < label.size(); i++)
	{
		size_t idx_tmp = label.points[i].label;
		output.push_back(input[idx_tmp]);
	}
}

void KUPCLMATH::extractDataFromIdx(const std::vector<size_t> &input, const pcl::PointIndices &idx, std::vector<size_t> &output)
{
	output.resize(idx.indices.size());
	for (size_t i = 0; i < idx.indices.size(); i++)
	{
		output[i] = input[idx.indices[i]];
	}
}

void KUPCLMATH::extractDataFromIdx(const std::vector<size_t> &input, const std::vector<size_t> &idx, std::vector<size_t> &output)
{
	output.resize(idx.size());
	for (size_t i = 0; i < idx.size(); i++)
	{
		output[i] = input[idx[i]];
	}
}

/** @brief Korean: 입력된 배열 요소의 순서를 역순으로 변경한다.
 * @param[in] list : std::vector형태의 입력
 */
void KUPCLMATH::reverseVecOrder(std::vector<std::vector<int>>& list)
{
	std::vector<std::vector<int>>::iterator first = list.begin();
	std::vector<std::vector<int>>::iterator last = list.end();

	while ((first != last) && (first != --last))
	{
		std::iter_swap(first, last);
		++first;
	}
}

/** @brief Korean: 입력된 배열 요소의 순서를 역순으로 변경한다.
* @param[in] list : std::vector형태의 입력
*/
void KUPCLMATH::reverseVecOrder(std::vector<std::vector<size_t>>& list)
{
	std::vector<std::vector<size_t>>::iterator first = list.begin();
	std::vector<std::vector<size_t>>::iterator last = list.end();

	while ((first != last) && (first != --last))
	{
		std::iter_swap(first, last);
		++first;
	}
}

/** @brief Korean: 입력된 배열 요소 중에서 한 번에 k개를 선택할 때 가능한 모든 조합이 포함된 std::vector형태의 결과를 반환한다.
 * @param[in] input : std::vector형태의 입력
 * @param[in] k : 선택 개수
 * @return totVec : 모든 조합의 경우의 수를 std::vector의 형태로 반환
 */
std::vector<std::vector<int>> KUPCLMATH::combnk(const std::vector<int>& input, int k)
{
	std::vector<std::vector<int> >totVec;
	int n = input.size();
    if(n == k) 
    {
        totVec.push_back(input);
        return totVec;
    }
	if (n < k) return{};
	if (k < 0) return{};

	std::vector<int> ind;
	Eigen::VectorXd test(k);
	for (int i = 0; i < k; i++)
	{
		ind.push_back(1);
	}
	std::vector<int> tempVec(k);
	for (int i = 0; i < input.size() - k; i++){	ind.push_back(0); }

	sort(ind.begin(), ind.end());
	int idx = 0;
	do{
		idx = 0;
		for (int i = 0; i < ind.size(); i++)
		{
			if (ind[i] == 1) { tempVec[idx++] = *(input.begin() + i); }
		}
		totVec.push_back(tempVec);
	} while (next_permutation(ind.begin(), ind.end()));
	reverseVecOrder(totVec);

	return totVec;
}

/** @brief Korean: 입력된 배열 요소 중에서 한 번에 k개를 선택할 때 가능한 모든 조합이 포함된 std::vector형태의 결과를 반환한다.
* @param[in] input : std::vector형태의 입력
* @param[in] k : 선택 개수
* @return totVec : 모든 조합의 경우의 수를 std::vector의 형태로 반환
*/
std::vector<std::vector<size_t>> KUPCLMATH::combnk(const std::vector<size_t>& input, size_t k)
{
	std::vector<std::vector<size_t>>totVec;
	size_t n = input.size();
    if(n == k) 
    {
        totVec.push_back(input);
        return totVec;
    }
	if (n < k) return{};
	if (k < 0) return{};

	std::vector<size_t> ind;
	Eigen::VectorXd test(k);
	for (size_t i = 0; i < k; i++)
	{
		ind.push_back(1);
	}
	std::vector<size_t> tempVec(k);
	for (size_t i = 0; i < input.size() - k; i++){ ind.push_back(0); }

	sort(ind.begin(), ind.end());
	size_t idx = 0;
	do{
		idx = 0;
		for (size_t i = 0; i < ind.size(); i++)
		{
			if (ind[i] == 1) { tempVec[idx++] = *(input.begin() + i); }
		}
		totVec.push_back(tempVec);
	} while (next_permutation(ind.begin(), ind.end()));
	reverseVecOrder(totVec);

	return totVec;
}

/** @brief Korean: 입력된 배열 요소에 대한 평균과 표준편차를 계산한다.
 * @param[in] input : std::vector형태의 입력
 * @param[out] m : mean
 * @param[out] std : standard deviation
 */
void KUPCLMATH::vec2meanStd(const std::vector<double>& input, double& m, double& std)
{
	double sum = std::accumulate(input.begin(), input.end(), 0.0);
	double mean = sum / input.size();

	std::vector<double> diff(input.size());
	std::transform(input.begin(), input.end(), diff.begin(),
		std::bind2nd(std::minus<double>(), mean));
	double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
	double stdev = std::sqrt(sq_sum / (input.size() - 1));

	m = mean;
	std = stdev;
}

/** @brief Korean: 입력된 배열 요소에 대한 평균을 산출한다.
* @param[in] input : std::vector형태의 입력
*/
Eigen::Vector3d KUPCLMATH::calMeanPoint(const std::vector<Eigen::Vector3d>& input)
{
	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> z;
	double mean_x, mean_y, mean_z;
	double std_x, std_y, std_z;

	for (size_t i = 0; i < input.size(); i++)
	{
		x.push_back(input[i][0]);
		y.push_back(input[i][1]);
		z.push_back(input[i][2]);
	}
	vec2meanStd(x, mean_x, std_x);
	vec2meanStd(y, mean_y, std_y);
	vec2meanStd(z, mean_z, std_z);

	Eigen::Vector3d output;
	output[0] = mean_x;
	output[1] = mean_y;
	output[2] = mean_z;

	return output;
}

/** @brief Korean: 내적 결과로부터 사잇각의 임계값 초과 여부를 반환한다.
 * @param[in] dot : 두 벡터 사이의 내적
 * @param[in] angle : 사잇각 임계값 [deg]
 * @return 임계값 초과 여부를 반환
 * @retval true : 사잇각이 임계값보다 작음
 * @retval false : 사잇각이 임계값보다 큼
 */
bool KUPCLMATH::dotCheck(double dot, double angle)
{
	if (dot >= cos(angle*(M_PI / 180.0))) return true;
	else return false;
}

/** @brief Korean: 두 입력 벡터 사이의 내적 결과로부터 사잇각의 임계값 초과 여부를 반환한다.
 * @param[in] v1 : 입력 벡터 1
 * @param[in] v2 : 입력 벡터 2
 * @param[in] angle : 사잇각 임계값 [deg]
 * @return 임계값 초과 여부를 반환
 * @retval true : 사잇각이 임계값보다 작음
 * @retval false : 사잇각이 임계값보다 큼
 */
bool KUPCLMATH::dotCheck2(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, double angle)
{
    double dot = v1.dot(v2);
	if (dot >= cos(angle*(M_PI / 180.0))) return true;
	else return false;
}

/** @brief Korean: 두 입력 벡터 사이의 내적 결과로부터 사잇각의 임계값 초과 여부를 반환한다.
 * @param[in] v1 : 입력 벡터 1
 * @param[in] v2 : 입력 벡터 2
 * @param[in] angle : 사잇각 임계값 [deg]
 * @return 임계값 초과 여부를 반환
 * @retval true : 사잇각이 임계값보다 작음
 * @retval false : 사잇각이 임계값보다 큼
 */
bool KUPCLMATH::dotCheck2(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2, double angle)
{
    double dot = v1.dot(v2);
	if (dot >= cos(angle*(M_PI / 180.0))) return true;
	else return false;
}

/** @brief Korean: 두 입력 벡터 사이의 내적 결과로부터 사잇각의 임계값 초과 여부를 반환한다.
 * @param[in] v1 : 입력 벡터 1
 * @param[in] v2 : 입력 벡터 2
 * @param[in] angle : 사잇각 임계값 [deg]
 * @return 임계값 초과 여부를 반환
 * @retval true : 사잇각이 임계값보다 작음
 * @retval false : 사잇각이 임계값보다 큼
 */
float KUPCLMATH::getVecSign(const Eigen::Vector3f &v1, const Eigen::Vector3f &v2)
{
    float vec_sign = 0.0;
    double dot = v1.dot(v2);
	if (dot > cos(90.0*(M_PI / 180.0))) vec_sign = 1.0;
	else if (dot < cos(90.0*(M_PI / 180.0))) vec_sign = -1.0;
    return vec_sign;
}

/** @brief Korean: 입력된 값의 부호를 산출한다.
* @param[in] val : 입력 값
* @return 부호를 반환
* @retval 1 : 양수
* @retval 0 : 0
* @retval -1 : 음수
*/
double KUPCLMATH::sgn(double val)
{
	if (val > 0.0) return 1.0;
	else if (val < 0.0) return -1.0;
	else return 0.0;
}

/** @brief Korean: 입력된 점군 데이터의 scale을 변경한다.
* @param[in] input : 입력 점군 데이터
* @param[in] b_scale : 1(m to mm), 0(mm to m)
* @param[out] std : standard deviation
*/
void KUPCLMATH::cloudScaling(pcl::PointCloud<pcl::PointXYZ> &input, int b_scale)
{
	// mm to m
	Eigen::Matrix4f transform_metric_mm2m = Eigen::Matrix4f::Identity();
	transform_metric_mm2m(0, 0) = 0.001; transform_metric_mm2m(1, 1) = 0.001; transform_metric_mm2m(2, 2) = 0.001;

	// m to mm
	Eigen::Matrix4f transform_metric_m2mm = Eigen::Matrix4f::Identity();
	transform_metric_m2mm(0, 0) = 1000.0; transform_metric_m2mm(1, 1) = 1000.0;	transform_metric_m2mm(2, 2) = 1000.0;

	if (b_scale) { pcl::transformPointCloud(input, input, transform_metric_m2mm); } // m to mm
	else { pcl::transformPointCloud(input, input, transform_metric_mm2m); } // mm to m
}

/** @brief Korean: 입력된 점군 데이터의 scale을 변경한다.
* @param[in] input : 입력 점군 데이터
* @param[in] b_scale : 1(m to mm), 0(mm to m)
* @param[out] std : standard deviation
*/
void KUPCLMATH::cloudScaling(pcl::PointCloud<pcl::PointXYZRGBNormal> &input, int b_scale)
{
	// mm to m
	Eigen::Matrix4f transform_metric_mm2m = Eigen::Matrix4f::Identity();
	transform_metric_mm2m(0, 0) = 0.001; transform_metric_mm2m(1, 1) = 0.001; transform_metric_mm2m(2, 2) = 0.001;

	// m to mm
	Eigen::Matrix4f transform_metric_m2mm = Eigen::Matrix4f::Identity();
	transform_metric_m2mm(0, 0) = 1000.0; transform_metric_m2mm(1, 1) = 1000.0;	transform_metric_m2mm(2, 2) = 1000.0;

	if (b_scale) { pcl::transformPointCloud(input, input, transform_metric_m2mm); }// m to mm
	else { pcl::transformPointCloud(input, input, transform_metric_mm2m); } // mm to m
}

/** @brief Korean: 입력된 점군 데이터의 scale을 변경한다.
* @param[in] input : 입력 점군 데이터
* @param[in] b_scale : 1(m to mm), 0(mm to m)
* @param[out] std : standard deviation
*/
void KUPCLMATH::cloudScaling(pcl::PointCloud<pcl::PointXYZLNormal> &input, int b_scale)
{
	// mm to m
	Eigen::Matrix4f transform_metric_mm2m = Eigen::Matrix4f::Identity();
	transform_metric_mm2m(0, 0) = 0.001; transform_metric_mm2m(1, 1) = 0.001; transform_metric_mm2m(2, 2) = 0.001;

	// m to mm
	Eigen::Matrix4f transform_metric_m2mm = Eigen::Matrix4f::Identity();
	transform_metric_m2mm(0, 0) = 1000.0; transform_metric_m2mm(1, 1) = 1000.0;	transform_metric_m2mm(2, 2) = 1000.0;

	if (b_scale) { pcl::transformPointCloud(input, input, transform_metric_m2mm); } // m to mm
	else { pcl::transformPointCloud(input, input, transform_metric_mm2m); } // mm to m
}

/** @brief Korean: 입력된 점군 데이터의 scale을 변경한다.
* @param[in] input : 입력 점군 데이터
* @param[in] b_scale : 1(m to mm), 0(mm to m)
*/
void KUPCLMATH::cloudScaling(pcl::PointCloud<pcl::PointXYZRGBA> &input, int b_scale)
{
	// mm to m
	Eigen::Matrix4f transform_metric_mm2m = Eigen::Matrix4f::Identity();
	transform_metric_mm2m(0, 0) = 0.001; transform_metric_mm2m(1, 1) = 0.001; transform_metric_mm2m(2, 2) = 0.001;

	// m to mm
	Eigen::Matrix4f transform_metric_m2mm = Eigen::Matrix4f::Identity();
	transform_metric_m2mm(0, 0) = 1000.0; transform_metric_m2mm(1, 1) = 1000.0;	transform_metric_m2mm(2, 2) = 1000.0;

	if (b_scale) { pcl::transformPointCloud(input, input, transform_metric_m2mm); } // m to mm
	else { pcl::transformPointCloud(input, input, transform_metric_mm2m); } // mm to m
}

/** @brief Korean: 입력된 점군 데이터의 scale을 변경한다.
* @param[in] input : 입력 점군 데이터
* @param[in] b_scale : 1(m to mm), 0(mm to m)
*/
void KUPCLMATH::cloudScaling(pcl::PointCloud<pcl::PointXYZRGB> &input, int b_scale)
{
	// mm to m
	Eigen::Matrix4f transform_metric_mm2m = Eigen::Matrix4f::Identity();
	transform_metric_mm2m(0, 0) = 0.001; transform_metric_mm2m(1, 1) = 0.001; transform_metric_mm2m(2, 2) = 0.001;

	// m to mm
	Eigen::Matrix4f transform_metric_m2mm = Eigen::Matrix4f::Identity();
	transform_metric_m2mm(0, 0) = 1000.0; transform_metric_m2mm(1, 1) = 1000.0;	transform_metric_m2mm(2, 2) = 1000.0;

	if (b_scale) { pcl::transformPointCloud(input, input, transform_metric_m2mm); } // m to mm
	else { pcl::transformPointCloud(input, input, transform_metric_mm2m); } // mm to m
}

/** @brief Korean: 입력된 점군 데이터의 scale을 변경한다.
* @param[in] input : 입력 점군 데이터
* @param[in] scale : 사용자가 입력한 scale
*/
void KUPCLMATH::cloudPointScaling(pcl::PointCloud<pcl::PointXYZ> &input, float scale)
{
	Eigen::Matrix4f transform_metric_scaling = Eigen::Matrix4f::Identity();
	transform_metric_scaling(0, 0) = scale; transform_metric_scaling(1, 1) = scale; transform_metric_scaling(2, 2) = scale;
	pcl::transformPointCloud(input, input, transform_metric_scaling);
}

/** @brief Korean: 입력된 std::vector 형식의 인덱스를 pcl::PointIndices 형식의 인덱스로 변경한다.
* @param[in] input : 입력된 std::vector 형식의 인덱스
* @param[out] output : pcl::PointIndices 형식의 인덱스
*/
void KUPCLMATH::vec2Indices(const std::vector<int>& input, pcl::PointIndices &output)
{
	output.indices.resize(input.size());
	for (int i = 0; i < input.size(); i++) { output.indices[i] = input[i]; }
}

/** @brief Korean: 입력된 std::vector 형식의 인덱스를 pcl::PointIndices 형식의 인덱스로 변경한다.
* @param[in] input : 입력된 std::vector 형식의 인덱스
* @param[out] output : pcl::PointIndices 형식의 인덱스
*/
void KUPCLMATH::vec2Indices(const std::vector<size_t>& input, pcl::PointIndices &output)
{
	output.indices.resize(input.size());
	for (size_t i = 0; i < input.size(); i++) { output.indices[i] = input[i]; }
}

/** @brief Korean: 입력된 점군 데이터의 라벨을 pcl::PointIndices 형식의 인덱스로 변경한다.
* @param[in] input : 입력된 점군 데이터(라벨 포함)
* @param[out] output : pcl::PointIndices 형식의 인덱스
*/
void KUPCLMATH::label2Indices(const pcl::PointCloud<pcl::PointXYZLNormal> &input, pcl::PointIndices &output)
{
	output.indices.resize(input.size());
	for (int i = 0; i < input.size(); i++) { output.indices[i] = input.points[i].label; }
}

/** @brief Korean: 입력된 배열 중에서 최소 또는 최대 요소와 인덱스를 추출한다.
 * @param[in] input : std::vector 형식의 입력
 * @param[out] idx_out : 최소 또는 최대 요소의 인덱스
 * @param[out] dis_out : 최소 또는 최대 요소 값
 * @param[in] b_select : 0: min. element, 1: max. element
 */
void KUPCLMATH::genVecMinMax(const std::vector<int>& input, int &idx_out, int &dis_out, int b_select) // 0: min., 1: max.
{
	std::vector<int> vec_in = input;
	if (b_select) // max
	{
		int dis = *std::max_element(vec_in.begin(), vec_in.end());
		std::vector<int>::iterator it_dis = std::max_element(vec_in.begin(), vec_in.end());
		int idx = std::distance(vec_in.begin(), it_dis);
		idx_out = idx;
		dis_out = dis;
	}
	else // min
	{
		int dis = *std::min_element(vec_in.begin(), vec_in.end());
		std::vector<int>::iterator it_dis = std::min_element(vec_in.begin(), vec_in.end());
		int idx = std::distance(vec_in.begin(), it_dis);
		idx_out = idx;
		dis_out = dis;
	}
}

/** @brief Korean: 입력된 배열 중에서 최소 또는 최대 요소와 인덱스를 추출한다.
* @param[in] input : std::vector 형식의 입력
* @param[out] idx_out : 최소 또는 최대 요소의 인덱스
* @param[out] dis_out : 최소 또는 최대 요소 값
* @param[in] b_select : 0: min. element, 1: max. element
*/
void KUPCLMATH::genVecMinMax(const std::vector<size_t>& input, size_t &idx_out, size_t &dis_out, int b_select)
{
	std::vector<size_t> vec_in = input;
	if (b_select) // max
	{
		size_t dis = *std::max_element(vec_in.begin(), vec_in.end());
		std::vector<size_t>::iterator it_dis = std::max_element(vec_in.begin(), vec_in.end());
		size_t idx = std::distance(vec_in.begin(), it_dis);
		idx_out = idx;
		dis_out = dis;
	}
	else // min
	{
		size_t dis = *std::min_element(vec_in.begin(), vec_in.end());
		std::vector<size_t>::iterator it_dis = std::min_element(vec_in.begin(), vec_in.end());
		size_t idx = std::distance(vec_in.begin(), it_dis);
		idx_out = idx;
		dis_out = dis;
	}
}

/** @brief Korean: 입력된 배열 중에서 최소 또는 최대 요소와 인덱스를 추출한다.
 * @param[in] input : std::vector 형식의 입력
 * @param[out] idx_out : 최소 또는 최대 요소의 인덱스
 * @param[out] dis_out : 최소 또는 최대 요소 값
 * @param[in] b_select : 0: min. element, 1: max. element
 */
void KUPCLMATH::genVecMinMax(const std::vector<double>& input, int &idx_out, double &dis_out, int b_select) // 0: min., 1: max.
{
	std::vector<double> vec_in = input;
	if (b_select) // max
	{
		double dis = *std::max_element(vec_in.begin(), vec_in.end());
		std::vector<double>::iterator it_dis = std::max_element(vec_in.begin(), vec_in.end());
		int idx = std::distance(vec_in.begin(), it_dis);
		idx_out = idx;
		dis_out = dis;
	}
	else // min
	{
		double dis = *std::min_element(vec_in.begin(), vec_in.end());
		std::vector<double>::iterator it_dis = std::min_element(vec_in.begin(), vec_in.end());
		int idx = std::distance(vec_in.begin(), it_dis);
		idx_out = idx;
		dis_out = dis;
	}
}

/** @brief Korean: 입력된 배열 중에서 최소 또는 최대 요소와 인덱스를 추출한다.
* @param[in] input : std::vector 형식의 입력
* @param[out] idx_out : 최소 또는 최대 요소의 인덱스
* @param[out] dis_out : 최소 또는 최대 요소 값
* @param[in] b_select : 0: min. element, 1: max. element
*/
void KUPCLMATH::genVecMinMax(const std::vector<double>& input, size_t &idx_out, double &dis_out, int b_select) // 0: min., 1: max.
{
	std::vector<double> vec_in = input;
	if (b_select) // max
	{
		double dis = *std::max_element(vec_in.begin(), vec_in.end());
		std::vector<double>::iterator it_dis = std::max_element(vec_in.begin(), vec_in.end());
		size_t idx = std::distance(vec_in.begin(), it_dis);
		idx_out = idx;
		dis_out = dis;
	}
	else // min
	{
		double dis = *std::min_element(vec_in.begin(), vec_in.end());
		std::vector<double>::iterator it_dis = std::min_element(vec_in.begin(), vec_in.end());
		size_t idx = std::distance(vec_in.begin(), it_dis);
		idx_out = idx;
		dis_out = dis;
	}
}

/** @brief Korean: 입력된 배열 중에서 최소 또는 최대 요소와 인덱스를 추출한다.
 * @param[in] input : std::vector 형식의 입력
 * @param[out] idx_out : 최소 또는 최대 요소의 인덱스
 * @param[out] dis_out : 최소 또는 최대 요소 값
 * @param[in] b_select : 0: min. element, 1: max. element
 */
void KUPCLMATH::genVecMinMax(const std::vector<float>& input, int &idx_out, float &dis_out, int b_select) // 0: min., 1: max.
{
	std::vector<float> vec_in = input;
	if (b_select) // max
	{
		float dis = *std::max_element(vec_in.begin(), vec_in.end());
		std::vector<float>::iterator it_dis = std::max_element(vec_in.begin(), vec_in.end());
		int idx = std::distance(vec_in.begin(), it_dis);
		idx_out = idx;
		dis_out = dis;
	}
	else // min
	{
		float dis = *std::min_element(vec_in.begin(), vec_in.end());
		std::vector<float>::iterator it_dis = std::min_element(vec_in.begin(), vec_in.end());
		int idx = std::distance(vec_in.begin(), it_dis);
		idx_out = idx;
		dis_out = dis;
	}
}

/** @brief Korean: 입력된 배열의 요소를 내림차순으로 정렬한다.
 * @param[in] input : std::vector 형식의 입력
 * @param[out] output_sort : 내림차순으로 정렬된 배열
 * @param[out] output_idx : 정렬된 인덱스
 */
void KUPCLMATH::genDescendingOrder(const std::vector<int>& input, std::vector<int>& output_sort, std::vector<int>& output_idx)
{
	// sorted vector
	output_sort = input;
	std::sort(output_sort.begin(), output_sort.end(), std::greater<int>());

	// index output
	output_idx.resize(input.size());
	std::iota(output_idx.begin(), output_idx.end(), 0);
	std::sort(output_idx.begin(), output_idx.end(),
		[&](const int& a, const int& b) {
		return (input[a] > input[b]);
	}
	);
}

/** @brief Korean: 입력된 배열의 요소를 내림차순으로 정렬한다.
 * @param[in] input : std::vector 형식의 입력
 * @param[out] output_sort : 내림차순으로 정렬된 배열
 * @param[out] output_idx : 정렬된 인덱스
 */
void KUPCLMATH::genDescendingOrder(const std::vector<double>& input, std::vector<double>& output_sort, std::vector<int>& output_idx) // descending order
{
	// sorted vector
	output_sort = input;
	std::sort(output_sort.begin(), output_sort.end(), std::greater<double>());

	// index output
	output_idx.resize(input.size());
	std::iota(output_idx.begin(), output_idx.end(), 0);
	std::sort(output_idx.begin(), output_idx.end(),
		[&](const int& a, const int& b) {
		return (input[a] > input[b]);
	}
	);
}

/** @brief Korean: 입력된 배열의 요소를 내림차순으로 정렬한다.
* @param[in] input : std::vector 형식의 입력
* @param[out] output_sort : 내림차순으로 정렬된 배열
* @param[out] output_idx : 정렬된 인덱스
*/
void KUPCLMATH::genDescendingOrder(const std::vector<double>& input, std::vector<double>& output_sort, std::vector<size_t>& output_idx) // descending order
{
	// sorted vector
	output_sort = input;
	std::sort(output_sort.begin(), output_sort.end(), std::greater<double>());

	// index output
	output_idx.resize(input.size());
	std::iota(output_idx.begin(), output_idx.end(), 0);
	std::sort(output_idx.begin(), output_idx.end(),
		[&](const size_t& a, const size_t& b) {
		return (input[a] > input[b]);
	}
	);
}

/** @brief Korean: 입력된 배열의 요소를 오름차순으로 정렬한다.
* @param[in] input : std::vector 형식의 입력
* @param[out] output_sort : 오름차순으로 정렬된 배열
* @param[out] output_idx : 정렬된 인덱스
*/
void KUPCLMATH::genAscendingOrder(const std::vector<int>& input, std::vector<int>& output_sort, std::vector<int>& output_idx) // ascending order
{
	// sorted vector
	output_sort = input;
	std::sort(output_sort.begin(), output_sort.end(), std::less<int>());

	// index output
	output_idx.resize(input.size());
	std::iota(output_idx.begin(), output_idx.end(), 0);
	std::sort(output_idx.begin(), output_idx.end(),
		[&](const int& a, const int& b) {
		return (input[a] < input[b]);
	}
	);
}

/** @brief Korean: 입력된 배열의 요소를 오름차순으로 정렬한다.
 * @param[in] input : std::vector 형식의 입력
 * @param[out] output_sort : 오름차순으로 정렬된 배열
 * @param[out] output_idx : 정렬된 인덱스
 */
void KUPCLMATH::genAscendingOrder(const std::vector<double>& input, std::vector<double>& output_sort, std::vector<int>& output_idx)
{
	// sorted vector
	output_sort = input;
	std::sort(output_sort.begin(), output_sort.end(), std::less<double>());

	// index output
	output_idx.resize(input.size());
	std::iota(output_idx.begin(), output_idx.end(), 0);
	std::sort(output_idx.begin(), output_idx.end(),
		[&](const int& a, const int& b) {
		return (input[a] < input[b]);
	}
	);
}

/** @brief Korean: 입력된 배열의 요소를 오름차순으로 정렬한다.
* @param[in] input : std::vector 형식의 입력
* @param[out] output_sort : 오름차순으로 정렬된 배열
* @param[out] output_idx : 정렬된 인덱스
*/
void KUPCLMATH::genAscendingOrder(const std::vector<double>& input, std::vector<double>& output_sort, std::vector<size_t>& output_idx)
{
	// sorted vector
	output_sort = input;
	std::sort(output_sort.begin(), output_sort.end(), std::less<double>());

	// index output
	output_idx.resize(input.size());
	std::iota(output_idx.begin(), output_idx.end(), 0);
	std::sort(output_idx.begin(), output_idx.end(),
		[&](const size_t& a, const size_t& b) {
		return (input[a] < input[b]);
	}
	);
}

/** @brief Korean: 입력된 배열의 요소 중에서 해당 인덱스의 요소를 제거한다.
 * @param[in] input : std::vector 형식의 입력
 * @param[in] idx : 제거할 요소의 인덱스
 */
void KUPCLMATH::eraseIdxValue(std::vector<int>& input, int idx)
{
	input.erase(input.begin() + idx);
}

/** @brief Korean: 입력된 배열의 요소 중에서 해당 인덱스의 요소를 제거한다.
 * @param[in] input : std::vector 형식의 입력
 * @param[in] idx : 제거할 요소의 인덱스
 */
void KUPCLMATH::eraseIdxValue(std::vector<size_t>& input, size_t idx)
{
	input.erase(input.begin() + idx);
}

/** @brief Korean: 입력된 배열의 요소 중에서 해당 인덱스의 요소를 제거한다.
* @param[in] input : std::vector 형식의 입력
* @param[in] idx : 제거할 요소의 인덱스
*/
void KUPCLMATH::eraseIdxValue(std::vector<float>& input, int idx)
{
	input.erase(input.begin() + idx);
}

/** @brief Korean: 입력된 배열의 요소 중에서 해당 인덱스의 요소를 제거한다.
* @param[in] input : std::vector 형식의 입력
* @param[in] idx : 제거할 요소의 인덱스
*/
void KUPCLMATH::eraseIdxValue(std::vector<double>& input, int idx)
{
	input.erase(input.begin() + idx);
}

/** @brief Korean: 입력된 배열의 요소 중에서 해당 인덱스의 요소를 제거한다.
 * @param[in] input : std::vector 형식의 입력
 * @param[in] idx : 제거할 요소의 인덱스
 */
void KUPCLMATH::eraseIdxValue(std::vector<Eigen::Vector3d>& input, int idx)
{
	input.erase(input.begin() + idx);
}

/** @brief Korean: 입력된 두 배열 사이의 요소 중에서 서로 다른 요소를 추출한다.
 * @param[in] input1 : std::vector 형식의 입력
 * @param[in] input2 : std::vector 형식의 입력
 * @param[out] output : 서로 다른 요소만 추출된 배열
 */
void KUPCLMATH::genDiffElements(const std::vector<int>& input1, const std::vector<int>& input2, std::vector<int>& output)
{
	for (int i = 0; i < input1.size(); i++)
	{
		bool b_check_diff = true;
		for (int j = 0; j < input2.size(); j++)
		{
			if (input1[i] == input2[j])
			{
				b_check_diff = false;
				break;
			}
		}
		if (b_check_diff) { output.push_back(input1[i]); }
	}
}

/** @brief Korean: 입력된 회전축, 회전각도를 이용하여 회전 및 변환행렬을 계산한다.
 * @param[in] angle : 회전각도
 * @param[out] rotM : 회전축을 중심으로 회전각도만큼 회전한 회전행렬
 * @param[out] HTM : 회전축을 중심으로 회전각도만큼 회전한 변환행렬
 * @param[in] direction : 회전 축 설정 (0: Rx, 1: Ry, 2: Rz)
 */
void KUPCLMATH::genRotM(double angle, Eigen::Matrix3d &rotM, Eigen::Matrix4d &HTM, int direction)
{
	rotM = Eigen::Matrix3d::Identity();
  	HTM = Eigen::Matrix4d::Identity();
	// input RotM, HTM --> identity matrix
	double angle_rad = angle*(M_PI / 180.0);
	switch (direction)
	{
	case 0: // in x direction
		rotM(1, 1) = cos(angle_rad);	rotM(1, 2) = -sin(angle_rad);
		rotM(2, 1) = sin(angle_rad);	rotM(2, 2) = cos(angle_rad);

		HTM(1, 1) = cos(angle_rad);	HTM(1, 2) = -sin(angle_rad);
		HTM(2, 1) = sin(angle_rad);	HTM(2, 2) = cos(angle_rad);
		break;

	case 1: // in y direction
		rotM(0, 0) = cos(angle_rad);	rotM(0, 2) = sin(angle_rad);
		rotM(2, 0) = -sin(angle_rad);	rotM(2, 2) = cos(angle_rad);

		HTM(0, 0) = cos(angle_rad);		HTM(0, 2) = sin(angle_rad);
		HTM(2, 0) = -sin(angle_rad);	HTM(2, 2) = cos(angle_rad);
		break;

	case 2: // in z direction
		rotM(0, 0) = cos(angle_rad);	rotM(0, 1) = -sin(angle_rad);
		rotM(1, 0) = sin(angle_rad);	rotM(1, 1) = cos(angle_rad);

		HTM(0, 0) = cos(angle_rad);		HTM(0, 1) = -sin(angle_rad);
		HTM(1, 0) = sin(angle_rad);		HTM(1, 1) = cos(angle_rad);
		break;
	}
}

/** @brief Korean: 입력된 회전축, 회전각도를 이용하여 회전 및 변환행렬을 계산한다.
 * @param[in] angle : 회전각도
 * @param[out] rotM : 회전축을 중심으로 회전각도만큼 회전한 회전행렬
 * @param[out] HTM : 회전축을 중심으로 회전각도만큼 회전한 변환행렬
 * @param[in] direction : 회전 축 설정 (0: Rx, 1: Ry, 2: Rz)
 */
void KUPCLMATH::genRotM(double angle, Eigen::Matrix3f &rotM, Eigen::Matrix4f &HTM, int direction)
{
	rotM = Eigen::Matrix3f::Identity();
  	HTM = Eigen::Matrix4f::Identity();
	// input RotM, HTM --> identity matrix
	double angle_rad = angle*(M_PI / 180.0);
	switch (direction)
	{
	case 0: // in x direction
		rotM(1, 1) = cos(angle_rad);	rotM(1, 2) = -sin(angle_rad);
		rotM(2, 1) = sin(angle_rad);	rotM(2, 2) = cos(angle_rad);

		HTM(1, 1) = cos(angle_rad);	HTM(1, 2) = -sin(angle_rad);
		HTM(2, 1) = sin(angle_rad);	HTM(2, 2) = cos(angle_rad);
		break;

	case 1: // in y direction
		rotM(0, 0) = cos(angle_rad);	rotM(0, 2) = sin(angle_rad);
		rotM(2, 0) = -sin(angle_rad);	rotM(2, 2) = cos(angle_rad);

		HTM(0, 0) = cos(angle_rad);		HTM(0, 2) = sin(angle_rad);
		HTM(2, 0) = -sin(angle_rad);	HTM(2, 2) = cos(angle_rad);
		break;

	case 2: // in z direction
		rotM(0, 0) = cos(angle_rad);	rotM(0, 1) = -sin(angle_rad);
		rotM(1, 0) = sin(angle_rad);	rotM(1, 1) = cos(angle_rad);

		HTM(0, 0) = cos(angle_rad);		HTM(0, 1) = -sin(angle_rad);
		HTM(1, 0) = sin(angle_rad);		HTM(1, 1) = cos(angle_rad);
		break;
	}
}

/** @brief Korean: 입력된 회전행렬을 ZYX euler angle (roll, pitch, yaw)로 반환한다.
 * @param[in] RotMat : 입력 회전행렬
 */
Eigen::Vector3d KUPCLMATH::rotm2ZYX(Eigen::Matrix3d &RotMat)
{
	// Rotation matrix to ZYX euler angle (roll, pitch, yaw)
	Eigen::Vector3d RPY;
	double Roll, Pitch, Yaw;
	double r11 = RotMat(0, 0);
	double r12 = RotMat(0, 1);
	double r13 = RotMat(0, 2);

	double r21 = RotMat(1, 0);
	double r22 = RotMat(1, 1);
	double r23 = RotMat(1, 2);

	double r31 = RotMat(2, 0);
	double r32 = RotMat(2, 1);
	double r33 = RotMat(2, 2);

	if (r11*r11 + r21*r21 != 0)
	{
		Roll = atan2(r32, r33);
		Pitch = atan2(-r31, sqrt(r32*r32 + r33*r33));
		Yaw = atan2(r21, r11);
	}
	else
	{
		Roll = 0;
		Pitch = 0;
		Yaw = 0;
	}
	RPY[0] = Roll; // Rx
	RPY[1] = Pitch; // Ry
	RPY[2] = Yaw; // Rz
	return RPY;
}

/** @brief Korean: 입력된 회전행렬을 ZYX euler angle (roll, pitch, yaw)로 반환한다.
 * @param[in] RotMat : 입력 회전행렬
 */
Eigen::Vector3f KUPCLMATH::rotm2ZYX(Eigen::Matrix3f &RotMat)
{
	// Rotation matrix to ZYX euler angle (roll, pitch, yaw)
	Eigen::Vector3f RPY;
	double Roll, Pitch, Yaw;
	double r11 = RotMat(0, 0);
	double r12 = RotMat(0, 1);
	double r13 = RotMat(0, 2);

	double r21 = RotMat(1, 0);
	double r22 = RotMat(1, 1);
	double r23 = RotMat(1, 2);

	double r31 = RotMat(2, 0);
	double r32 = RotMat(2, 1);
	double r33 = RotMat(2, 2);

	if (r11*r11 + r21*r21 != 0)
	{
		Roll = atan2(r32, r33);
		Pitch = atan2(-r31, sqrt(r32*r32 + r33*r33));
		Yaw = atan2(r21, r11);
	}
	else
	{
		Roll = 0;
		Pitch = 0;
		Yaw = 0;
	}
	RPY[0] = Roll; // Rx
	RPY[1] = Pitch; // Ry
	RPY[2] = Yaw; // Rz
	return RPY;
}

/** @brief Korean: 순기구학을 수행한다.
 * @param[in] DH : 로봇의 DH 파라미터
 * @param[in] q : 로봇의 관절 각도 [deg]
 * @param[out] T_out : 출력 변환행렬
 */
void KUPCLMATH::forwardKinematics(const double DH[6][4], const Eigen::VectorXd &q, Eigen::Matrix4d &T_out)
{
	T_out = Eigen::Matrix4d::Identity();

	for (int i = 0; i < 6; i++)
	{
		Eigen::Matrix4d T_tmp = Eigen::Matrix4d::Identity();
		double q_des = q[i]*(M_PI/180.0);
		T_tmp(0, 0) = cos(q_des);
		T_tmp(0, 1) = -sin(q_des)*cos(DH[i][1]);
		T_tmp(0, 2) = sin(q_des)*sin(DH[i][1]);
		T_tmp(0, 3) = DH[i][0] * cos(q_des);
		T_tmp(1, 0) = sin(q_des);
		T_tmp(1, 1) = cos(q_des)*cos(DH[i][1]);
		T_tmp(1, 2) = -cos(q_des)*sin(DH[i][1]);
		T_tmp(1, 3) = DH[i][0] * sin(q_des);
		T_tmp(2, 0) = 0.0;
		T_tmp(2, 1) = sin(DH[i][1]);
		T_tmp(2, 2) = cos(DH[i][1]);
		T_tmp(2, 3) = DH[i][2];

		T_out = T_out*T_tmp;
	}

    // TCP 적용
    T_out = T_out*m_TCP;
}

/** @brief Korean: 순기구학을 수행한다.
 * @param[in] DH : 로봇의 DH 파라미터
 * @param[in] q : 로봇의 관절 각도 [deg]
 * @param[out] T_out : 출력 변환행렬
 */
void KUPCLMATH::forwardKinematics(const double DH[6][4], const Eigen::VectorXf &q, Eigen::Matrix4f &T_out)
{
	T_out = Eigen::Matrix4f::Identity();

	for (int i = 0; i < 6; i++)
	{
		Eigen::Matrix4f T_tmp = Eigen::Matrix4f::Identity();
		double q_des = q[i]*(M_PI/180.0);
		T_tmp(0, 0) = cos(q_des);
		T_tmp(0, 1) = -sin(q_des)*cos(DH[i][1]);
		T_tmp(0, 2) = sin(q_des)*sin(DH[i][1]);
		T_tmp(0, 3) = DH[i][0] * cos(q_des);
		T_tmp(1, 0) = sin(q_des);
		T_tmp(1, 1) = cos(q_des)*cos(DH[i][1]);
		T_tmp(1, 2) = -cos(q_des)*sin(DH[i][1]);
		T_tmp(1, 3) = DH[i][0] * sin(q_des);
		T_tmp(2, 0) = 0.0;
		T_tmp(2, 1) = sin(DH[i][1]);
		T_tmp(2, 2) = cos(DH[i][1]);
		T_tmp(2, 3) = DH[i][2];

		T_out = T_out*T_tmp;
	}

    // TCP 적용
    T_out = T_out*m_TCP.cast<float>();
}

/** @brief Korean: 입력된 postion(x, y, z)을 변환행렬 형식으로 반환한다.
 * @param[in] T_in : 입력 변환행렬
 * @param[out] vec_out : 출력 자세 (x,y,z,R,P,Y)
 */
void KUPCLMATH::tr2pose(const Eigen::Matrix4d &T_in, Eigen::VectorXd &vec_out)
{
	Eigen::Matrix3d rotM = T_in.block<3,3>(0,0);
	Eigen::Vector3d RPY = rotm2ZYX(rotM);

	vec_out[0] = T_in(0, 3); // x
	vec_out[1] = T_in(1, 3); // y
	vec_out[2] = T_in(2, 3); // z
	vec_out[3] = RPY[0]; // Rx
	vec_out[4] = RPY[1]; // Ry
	vec_out[5] = RPY[2]; // Rz
}

/** @brief Korean: 입력된 postion(x, y, z)을 변환행렬 형식으로 반환한다.
 * @param[in] T_in : 입력 변환행렬
 * @param[out] vec_out : 출력 자세 (x,y,z,R,P,Y)
 */
void KUPCLMATH::tr2pose(const Eigen::Matrix4f &T_in, Eigen::VectorXf &vec_out)
{
	Eigen::Matrix3f rotM = T_in.block<3,3>(0,0);
	Eigen::Vector3f RPY = rotm2ZYX(rotM);

	vec_out[0] = T_in(0, 3); // x
	vec_out[1] = T_in(1, 3); // y
	vec_out[2] = T_in(2, 3); // z
	vec_out[3] = RPY[0]; // Rx
	vec_out[4] = RPY[1]; // Ry
	vec_out[5] = RPY[2]; // Rz
}

/** @brief Korean: 입력된 postion(x, y, z)을 변환행렬 형식으로 반환한다.
 */
Eigen::Matrix4f KUPCLMATH::genTranslationHTM(double x, double y, double z)
{
	// Only translation vector to 4x4 Homogeneous Transform Matrix
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	transformation_matrix(0, 3) = x;
	transformation_matrix(1, 3) = y;
	transformation_matrix(2, 3) = z;
	return transformation_matrix;
}

/** @brief Korean: Roll 회전을 수행한 변환행렬을 반환한다.
 */
Eigen::Matrix4f KUPCLMATH::genRxRotationHTM(double Roll)
{
	// Roll [rad] to 4x4 Homogeneous Transform Matrix
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	transformation_matrix(1, 1) = cos(Roll);
	transformation_matrix(1, 2) = -sin(Roll);
	transformation_matrix(2, 1) = sin(Roll);
	transformation_matrix(2, 2) = cos(Roll);
	return transformation_matrix;
}

/** @brief Korean: Pitch 회전을 수행한 변환행렬을 반환한다.
 */
Eigen::Matrix4f KUPCLMATH::genRyRotationHTM(double Pitch)
{
	// Pitch [rad] to 4x4 Homogeneous Transform Matrix
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	transformation_matrix(0, 0) = cos(Pitch);
	transformation_matrix(0, 2) = sin(Pitch);
	transformation_matrix(2, 0) = -sin(Pitch);
	transformation_matrix(2, 2) = cos(Pitch);
	return transformation_matrix;
}

/** @brief Korean: Yaw 회전을 수행한 변환행렬을 반환한다.
 */
Eigen::Matrix4f KUPCLMATH::genRzRotationHTM(double Yaw)
{
	// Yaw [rad] to 4x4 Homogeneous Transform Matrix
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	transformation_matrix(0, 0) = cos(Yaw);
	transformation_matrix(0, 1) = -sin(Yaw);
	transformation_matrix(1, 0) = sin(Yaw);
	transformation_matrix(1, 1) = cos(Yaw);
	return transformation_matrix;
}

/** @brief Korean: 두 벡터의 외적을 반환한다.
 */
Eigen::Vector3f KUPCLMATH::crossProduct(double V1[3], double V2[3])
{
	// Cross product
	Eigen::Vector3f V3;
	V3[0] = V1[1] * V2[2] - V1[2] * V2[1];
	V3[1] = V1[2] * V2[0] - V1[0] * V2[2];
	V3[2] = V1[0] * V2[1] - V1[1] * V2[0];
	return V3;
}

/** @brief Korean: 입력된 회전행렬을 RPY로 변환하여 반환한다.
 * @param[in] RotMat : 회전행렬
 */
Eigen::Vector3f KUPCLMATH::toRPY(Eigen::Matrix4f RotMat)
{
	Eigen::Vector3f RPY;
	double Roll, Pitch, Yaw;
	double r11 = RotMat(0, 0);
	double r12 = RotMat(0, 1);
	double r13 = RotMat(0, 2);

	double r21 = RotMat(1, 0);
	double r22 = RotMat(1, 1);
	double r23 = RotMat(1, 2);

	double r31 = RotMat(2, 0);
	double r32 = RotMat(2, 1);
	double r33 = RotMat(2, 2);

	if (r11*r11 + r21*r21 != 0)
	{
		Roll = atan2(r32, r33);
		Pitch = atan2(-r31, sqrt(r32*r32 + r33*r33));
		Yaw = atan2(r21, r11);
	}
	else
	{
		Roll = 0;
		Pitch = 0;
		Yaw = 0;
	}
	RPY[0] = Roll; // Rx
	RPY[1] = Pitch; // Ry
	RPY[2] = Yaw; // Rz
	return RPY;
}

/** @brief Korean: 입력된 RPY를 rotation vector로 변환하여 반환한다.
 * @param[in] Roll : RPY의 Roll
 * @param[in] Pitch : RPY의 Pitch
 * @param[in] Yaw : RPY의 Yaw
 */
Eigen::Vector3f KUPCLMATH::toRotVec(double Roll, double Pitch, double Yaw)
{
	Eigen::Vector3f RotationVec;
	if (Roll == 0 && Pitch == 0 && Yaw == 0) { return RotationVec; }

	Eigen::Matrix3f RollM;
	RollM(0, 0) = 1; RollM(0, 1) = 0;		  RollM(0, 2) = 0;
	RollM(1, 0) = 0; RollM(1, 1) = cos(Roll); RollM(1, 2) = -sin(Roll);
	RollM(2, 0) = 0; RollM(2, 1) = sin(Roll); RollM(2, 2) = cos(Roll);

	Eigen::Matrix3f PitchM;
	PitchM(0, 0) = cos(Pitch);  PitchM(0, 1) = 0; PitchM(0, 2) = sin(Pitch);
	PitchM(1, 0) = 0;		    PitchM(1, 1) = 1; PitchM(1, 2) = 0;
	PitchM(2, 0) = -sin(Pitch); PitchM(2, 1) = 0; PitchM(2, 2) = cos(Pitch);

	Eigen::Matrix3f YawM;
	YawM(0, 0) = cos(Yaw); YawM(0, 1) = -sin(Yaw); YawM(0, 2) = 0;
	YawM(1, 0) = sin(Yaw); YawM(1, 1) = cos(Yaw);  YawM(1, 2) = 0;
	YawM(2, 0) = 0;		   YawM(2, 1) = 0;		   YawM(2, 2) = 1;

	Eigen::Matrix3f Rot = YawM*PitchM*RollM;

	double rotSum = Rot(0, 0) + Rot(1, 1) + Rot(2, 2) - 1;
	double alpha = acos((rotSum) / 2.0);
	double theta = 0;
	if (Roll >= 0) { theta = alpha; }
	else { theta = 2 * M_PI - alpha; }

	double my = 1 / (2 * sin(theta));
	double rx = my * (Rot(2, 1) - Rot(1, 2)) * theta;
	double ry = my * (Rot(0, 2) - Rot(2, 0)) * theta;
	double rz = my * (Rot(1, 0) - Rot(0, 1)) * theta;

	//// Singularity 처리
	if (abs(theta) < 0.001 || abs(theta) - M_PI < 0.001)
	{
		double tmp_rx = (Rot(0, 0) + 1.0) / 2.0;
		if (Rot(0, 0) > 0) rx = sqrt(tmp_rx);
		else rx = -sqrt(tmp_rx);
		rx = M_PI*rx;

		double tmp_ry = (Rot(1, 1) + 1.0) / 2.0;
		if (Rot(1, 1) > 0) ry = sqrt(tmp_ry);
		else ry = -sqrt(tmp_ry);
		ry = M_PI*ry;

		double tmp_rz = (Rot(2, 2) + 1.0) / 2.0;
		if (Rot(2, 2) > 0) rz = sqrt(tmp_rz);
		else rz = -sqrt(tmp_rz);
		rz = M_PI*rz;
	}

	////// Rx = +-3.14, Ry = 0.0, Rz = 0.0인 경우, singularity 방지
	//if (abs(theta) - M_PI < 0.001 && abs(Pitch) < 0.001 && abs(Yaw) < 0.001)
	//{
	//	rx = Roll;
	//	ry = 0.0;
	//	rz = 0.0;
	//}

	////// Rx = +3.14, Ry = 0.7854, Rz = 0.0인 경우, singularity 방지
	//if (abs(theta) - M_PI < 0.001 && abs(Pitch)-0.7854 < 0.001 && abs(Yaw) < 0.001)
	//{
	//	rx = 2.9024583;
	//	ry = -0.0000044;
	//	rz = -1.2022407;
	//}

	RotationVec[0] = rx;
	RotationVec[1] = ry;
	RotationVec[2] = rz;
	return RotationVec;
}

/** @brief Korean: 회전변환과 병진이동을 포함하는 변환행렬을 반환한다.
 * @param[in] rot : 입력 회전행렬
 * @param[in] trans_x : translation x
 * @param[in] trans_y : translation y
 * @param[in] trans_z : translation z
 * @param[in] method : 회전 축 선택 - (0: x-axis, 1: y-axis, 2: z-axis)
 */
Eigen::Matrix4f KUPCLMATH::genHTM(double rot, double trans_x, double trans_y, double trans_z, int method)
{
	// Generating 4x4 Homogeneous Transform Matrix from rotation about an axis and translation
	Eigen::Matrix4f mat_HTM = Eigen::Matrix4f::Identity();
	double Rotation_rad = DEG2RAD(rot);

	Eigen::Matrix4f mat_trans = genTranslationHTM(trans_x, trans_y, trans_z);
	Eigen::Matrix4f mat_rot;

	if (method == 0) mat_rot = genRxRotationHTM(Rotation_rad); // about x-axis
	else if (method == 1) mat_rot = genRyRotationHTM(Rotation_rad); // about y-axis
	else if (method == 2) mat_rot = genRzRotationHTM(Rotation_rad); // about z-axis

	mat_HTM = mat_trans*mat_rot;
	return mat_HTM;
}

/** @brief Korean: RPY 형식의 자세를 변환행렬로 반환한다.
 * @param[in] rot : 입력 회전행렬
 * @param[in] trans_x : translation x
 * @param[in] trans_y : translation y
 * @param[in] trans_z : translation z
 * @param[in] method : 회전 축 선택 - (0: x-axis, 1: y-axis, 2: z-axis)
 */
Eigen::Matrix4f KUPCLMATH::genPose2HTM(const Eigen::VectorXd &pose_in)
{
	//// Defining a rotation matrix and translation vector
	//// xyz, roll, pitch, yaw to 4x4 Homogeneous Transform Matrix

	// pose input
	double x = pose_in[0];
	double y = pose_in[1];
	double z = pose_in[2];
	double roll = pose_in[3];
	double pitch = pose_in[4];
	double yaw = pose_in[5];

	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f translation_matrix = genTranslationHTM(x, y, z);
	Eigen::Matrix4f Rotation_matrix = genRzRotationHTM(yaw) * genRyRotationHTM(pitch) * genRxRotationHTM(roll);

	transformation_matrix = translation_matrix*Rotation_matrix;
	return transformation_matrix;
    
}

/** @brief Korean: 점과 평면 사이의 거리를 산출한다.
*/
double KUPCLMATH::calDistPt2Plane(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector3d &normal)
{
	double distance = 0.0;
	distance = (pt2 - pt1).dot(normal);
	return distance;
}

/** @brief Korean: 벡터 합을 산출한다.
*/
Eigen::Vector3d KUPCLMATH::multipleUnitVectorSummation(const std::vector<Eigen::Vector3d> &vec_set)
{
	// 1) Vector summation
	Eigen::Vector3d vec_sum = {0.0, 0.0, 0.0};
	for (size_t i = 0; i < vec_set.size(); i++)
	{
		Eigen::Vector3d vec_tmp = vec_set[i];
		vec_sum = vec_sum + vec_tmp;
		vec_sum.normalize();
	}
	return vec_sum;
}

/** @brief Korean: KdTree를 이용하여 가장 근접한 점을 산출한다.
*/
Eigen::Vector3d KUPCLMATH::findKNearestPoint(const std::vector<Eigen::Vector3d> &vec_set, const Eigen::Vector3d &pt_query)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	vec2cloud(vec_set, *cloud_in);

	// KdTree search
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> md_KdTree;
	md_KdTree.setInputCloud(cloud_in->makeShared());

	std::vector<int> kd_indices(1);
	std::vector<float> kd_dists(1);
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_pt(new pcl::PointCloud <pcl::PointXYZ>);
	cloud_pt->resize(1);
	cloud_pt->points[0].x = pt_query[0];
	cloud_pt->points[0].y = pt_query[1];
	cloud_pt->points[0].z = pt_query[2];
	md_KdTree.nearestKSearch(cloud_pt->points[0], 1, kd_indices, kd_dists); // 2D vision center

	Eigen::Vector3d output;
	output[0] = cloud_in->points[kd_indices[0]].x;
	output[1] = cloud_in->points[kd_indices[0]].y;
	output[2] = cloud_in->points[kd_indices[0]].z;

	return output;
}

/** @brief Korean: KdTree를 이용하여 가장 근접한 점과 인덱스를 산출한다.
*/
Eigen::Vector3d KUPCLMATH::findKNearestPointWithIndex(const std::vector<Eigen::Vector3d> &vec_set, const Eigen::Vector3d &pt_query, size_t &nearest_index)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	vec2cloud(vec_set, *cloud_in);

	// KdTree search
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> md_KdTree;
	md_KdTree.setInputCloud(cloud_in->makeShared());

	std::vector<int> kd_indices(1);
	std::vector<float> kd_dists(1);
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_pt(new pcl::PointCloud <pcl::PointXYZ>);
	cloud_pt->resize(1);
	cloud_pt->points[0].x = pt_query[0];
	cloud_pt->points[0].y = pt_query[1];
	cloud_pt->points[0].z = pt_query[2];
	md_KdTree.nearestKSearch(cloud_pt->points[0], 1, kd_indices, kd_dists); // 2D vision center

	nearest_index = kd_indices[0];
	Eigen::Vector3d output;
	output[0] = cloud_in->points[kd_indices[0]].x;
	output[1] = cloud_in->points[kd_indices[0]].y;
	output[2] = cloud_in->points[kd_indices[0]].z;

	return output;
}

/** @brief Korean: KdTree를 이용하여 가장 근접한 점과 인덱스를 산출한다.
*/
Eigen::Vector3d KUPCLMATH::findKNearestPointWithIndex(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, const Eigen::Vector3d &pt_query, size_t &nearest_index)
{
	// KdTree search
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> md_KdTree;
	md_KdTree.setInputCloud(cloud_in.makeShared());

	std::vector<int> kd_indices(1);
	std::vector<float> kd_dists(1);
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_pt(new pcl::PointCloud <pcl::PointXYZ>);
	cloud_pt->resize(1);
	cloud_pt->points[0].x = pt_query[0];
	cloud_pt->points[0].y = pt_query[1];
	cloud_pt->points[0].z = pt_query[2];
	md_KdTree.nearestKSearch(cloud_pt->points[0], 1, kd_indices, kd_dists); // 2D vision center

	nearest_index = kd_indices[0];
	Eigen::Vector3d output;
	output[0] = cloud_in.points[kd_indices[0]].x;
	output[1] = cloud_in.points[kd_indices[0]].y;
	output[2] = cloud_in.points[kd_indices[0]].z;

	return output;
}

/** @brief Korean: KdTree를 이용하여 가장 근접한 점과 인덱스를 산출한다.
*/
Eigen::Vector3d KUPCLMATH::findKNearestPointWithIndex(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, const Eigen::Vector3d &pt_query, size_t &nearest_index)
{
	// KdTree search
	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal, flann::L2_Simple<float>> md_KdTree;
	md_KdTree.setInputCloud(cloud_in.makeShared());

	std::vector<int> kd_indices(1);
	std::vector<float> kd_dists(1);
	pcl::PointCloud <pcl::PointXYZRGBNormal>::Ptr cloud_pt(new pcl::PointCloud <pcl::PointXYZRGBNormal>);
	cloud_pt->resize(1);
	cloud_pt->points[0].x = pt_query[0];
	cloud_pt->points[0].y = pt_query[1];
	cloud_pt->points[0].z = pt_query[2];
	md_KdTree.nearestKSearch(cloud_pt->points[0], 1, kd_indices, kd_dists); // 2D vision center

	nearest_index = kd_indices[0];
	Eigen::Vector3d output;
	output[0] = cloud_in.points[kd_indices[0]].x;
	output[1] = cloud_in.points[kd_indices[0]].y;
	output[2] = cloud_in.points[kd_indices[0]].z;

	return output;
}

/** @brief Korean: KdTree를 이용하여 가장 근접한 점과 인덱스를 산출한다.
*/
Eigen::Vector3d KUPCLMATH::findKNearestPointWithIndex(const pcl::PointCloud<pcl::PointXYZLNormal> &cloud_in, const Eigen::Vector3d &pt_query, size_t &nearest_index)
{
	// KdTree search
	pcl::KdTreeFLANN<pcl::PointXYZLNormal, flann::L2_Simple<float>> md_KdTree;
	md_KdTree.setInputCloud(cloud_in.makeShared());

	std::vector<int> kd_indices(1);
	std::vector<float> kd_dists(1);
	pcl::PointCloud <pcl::PointXYZLNormal>::Ptr cloud_pt(new pcl::PointCloud <pcl::PointXYZLNormal>);
	cloud_pt->resize(1);
	cloud_pt->points[0].x = pt_query[0];
	cloud_pt->points[0].y = pt_query[1];
	cloud_pt->points[0].z = pt_query[2];
	md_KdTree.nearestKSearch(cloud_pt->points[0], 1, kd_indices, kd_dists); // 2D vision center

	nearest_index = kd_indices[0];
	Eigen::Vector3d output;
	output[0] = cloud_in.points[kd_indices[0]].x;
	output[1] = cloud_in.points[kd_indices[0]].y;
	output[2] = cloud_in.points[kd_indices[0]].z;

	return output;
}

/** @brief Korean: KdTree를 이용하여 가장 근접한 점을 산출한다.
*/
Eigen::Vector3d KUPCLMATH::findKNearestPoint2(pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &model, const pcl::PointCloud<pcl::PointXYZ> &cloud_in, const Eigen::Vector3d &pt_query)
{
	// KdTree search
	std::vector<int> kd_indices(1);
	std::vector<float> kd_dists(1);
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_pt(new pcl::PointCloud <pcl::PointXYZ>);
	cloud_pt->resize(1);
	cloud_pt->points[0].x = pt_query[0];
	cloud_pt->points[0].y = pt_query[1];
	cloud_pt->points[0].z = pt_query[2];
	model.nearestKSearch(cloud_pt->points[0], 1, kd_indices, kd_dists); // 2D vision center

	Eigen::Vector3d output;
	output[0] = cloud_in.points[kd_indices[0]].x;
	output[1] = cloud_in.points[kd_indices[0]].y;
	output[2] = cloud_in.points[kd_indices[0]].z;

	return output;
}

/** @brief Korean: KdTree를 이용하여 가장 근접한 점을 산출한다.
*/
Eigen::Vector3d KUPCLMATH::findKNearestPoint2(pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &model, const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, const Eigen::Vector3d &pt_query)
{
	// KdTree search
	std::vector<int> kd_indices(1);
	std::vector<float> kd_dists(1);
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_pt(new pcl::PointCloud <pcl::PointXYZ>);
	cloud_pt->resize(1);
	cloud_pt->points[0].x = pt_query[0];
	cloud_pt->points[0].y = pt_query[1];
	cloud_pt->points[0].z = pt_query[2];
	model.nearestKSearch(cloud_pt->points[0], 1, kd_indices, kd_dists); // 2D vision center

	Eigen::Vector3d output;
	output[0] = cloud_in.points[kd_indices[0]].x;
	output[1] = cloud_in.points[kd_indices[0]].y;
	output[2] = cloud_in.points[kd_indices[0]].z;

	return output;
}

/** @brief Korean: KdTree를 이용하여 가장 근접한 점의 인덱스를 산출한다.
*/
void KUPCLMATH::findKNearestIndex(pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &model, const Eigen::Vector3d &pt_query, std::vector<int> &output_idx, std::vector<float> &output_dists, int K)
{
	// KdTree search
	std::vector<int> kd_indices(K);
	std::vector<float> kd_dists(K);
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_pt(new pcl::PointCloud <pcl::PointXYZ>);
	cloud_pt->resize(1);
	cloud_pt->points[0].x = pt_query[0];
	cloud_pt->points[0].y = pt_query[1];
	cloud_pt->points[0].z = pt_query[2];
	model.nearestKSearch(cloud_pt->points[0], K, kd_indices, kd_dists); // 2D vision center
	output_idx = kd_indices;
	output_dists = kd_dists;
}

/** @brief Korean: KdTree를 이용하여 가장 근접한 점의 인덱스를 산출한다.
*/
void KUPCLMATH::findKNearestIndex(pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &model, const Eigen::Vector3f &pt_query, std::vector<int> &output_idx, std::vector<float> &output_dists, int K)
{
	// KdTree search
	std::vector<int> kd_indices(K);
	std::vector<float> kd_dists(K);
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_pt(new pcl::PointCloud <pcl::PointXYZ>);
	cloud_pt->resize(1);
	cloud_pt->points[0].x = pt_query[0];
	cloud_pt->points[0].y = pt_query[1];
	cloud_pt->points[0].z = pt_query[2];
	model.nearestKSearch(cloud_pt->points[0], K, kd_indices, kd_dists); // 2D vision center
	output_idx = kd_indices;
	output_dists = kd_dists;
}

/** @brief Korean: KdTree를 이용하여 가장 근접한 점과 인덱스를 산출한다.
*/
Eigen::Vector3f KUPCLMATH::findKNearestIndex2(const pcl::PointCloud<pcl::PointXYZ> &cloud_in, const Eigen::Vector3f &pt_query, size_t &nearest_index)
{
	// KdTree search
	pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> md_KdTree;
	md_KdTree.setInputCloud(cloud_in.makeShared());

	std::vector<int> kd_indices(1);
	std::vector<float> kd_dists(1);
    pcl::PointXYZ pt_in;
    pt_in.getVector3fMap() = pt_query;
	md_KdTree.nearestKSearch(pt_in, 1, kd_indices, kd_dists); // 2D vision center

	nearest_index = kd_indices[0];
    Eigen::Vector3f output;
	output = cloud_in.points[nearest_index].getVector3fMap();

	return output;
}

/** @brief Korean: KdTree를 이용하여 가장 근접한 점과 인덱스를 산출한다.
*/
Eigen::VectorXf KUPCLMATH::findKNearestIndex2(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, const Eigen::Vector3f &pt_query, size_t &nearest_index)
{
	// KdTree search
	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal, flann::L2_Simple<float>> md_KdTree;
	md_KdTree.setInputCloud(cloud_in.makeShared());

	std::vector<int> kd_indices(1);
	std::vector<float> kd_dists(1);
    pcl::PointXYZRGBNormal pt_in;
    pt_in.getVector3fMap() = pt_query;
	md_KdTree.nearestKSearch(pt_in, 1, kd_indices, kd_dists); // 2D vision center

	nearest_index = kd_indices[0];
    Eigen::VectorXf output(6);
	output.segment<3>(0) = cloud_in.points[nearest_index].getVector3fMap();
	output.segment<3>(3) = cloud_in.points[nearest_index].getNormalVector3fMap();

	return output;
}

/** @brief Korean: KdTree를 이용하여 가장 근접한 점과 인덱스를 산출한다.
*/
void KUPCLMATH::findKNearestIndex3(pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &model, const Eigen::Vector3f &pt_query, size_t &nearest_index)
{
	std::vector<int> kd_indices(1);
	std::vector<float> kd_dists(1);
    pcl::PointXYZ pt_in;
    pt_in.getVector3fMap() = pt_query;
	model.nearestKSearch(pt_in, 1, kd_indices, kd_dists); // 2D vision center

	nearest_index = kd_indices[0];
}

/** @brief Korean: KdTree(radius search)를 이용하여 가장 근접한 점의 인덱스를 산출한다.
*/
void KUPCLMATH::findRadiusNearestIndex(pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &model, const Eigen::Vector3d &pt_query, std::vector<int> &output_idx, std::vector<float> &output_dists, double radius)
{
	// KdTree search - radius search
	std::vector<int> kd_indices;
	std::vector<float> kd_dists;
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_pt(new pcl::PointCloud <pcl::PointXYZ>);
	cloud_pt->resize(1);
	cloud_pt->points[0].x = pt_query[0];
	cloud_pt->points[0].y = pt_query[1];
	cloud_pt->points[0].z = pt_query[2];
	model.radiusSearch(cloud_pt->points[0], radius, kd_indices, kd_dists); 
	output_idx = kd_indices;
	output_dists = kd_dists;
}

/** @brief Korean: KdTree(radius search)를 이용하여 가장 근접한 점의 인덱스를 산출한다.
*/
void KUPCLMATH::findRadiusNearestIndex(pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>> &model, const Eigen::Vector3f &pt_query, std::vector<int> &output_idx, std::vector<float> &output_dists, double radius)
{
	// KdTree search - radius search
	std::vector<int> kd_indices;
	std::vector<float> kd_dists;
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloud_pt(new pcl::PointCloud <pcl::PointXYZ>);
	cloud_pt->resize(1);
	cloud_pt->points[0].x = pt_query[0];
	cloud_pt->points[0].y = pt_query[1];
	cloud_pt->points[0].z = pt_query[2];
	model.radiusSearch(cloud_pt->points[0], radius, kd_indices, kd_dists); 
	output_idx = kd_indices;
	output_dists = kd_dists;
}

/** @brief Korean: 입력된 점의 좌표변환을 수행한다.
*/
Eigen::Vector3d KUPCLMATH::transformPoint2A(Eigen::Vector3d &vec_in, const Eigen::Matrix4d &T_A2B)
{
	Eigen::Vector4d vec_in_B = { vec_in[0], vec_in[1], vec_in[2], 1.0 };
	Eigen::Vector4d tmp = T_A2B*vec_in_B;
	Eigen::Vector3d vec_out_A = { tmp[0], tmp[1], tmp[2] };
	return vec_out_A;
}

/** @brief Korean: 입력된 점의 좌표변환을 수행한다.
*/
Eigen::Vector3f KUPCLMATH::transformPoint2A(Eigen::Vector3f &vec_in, const Eigen::Matrix4f &T_A2B)
{
	Eigen::Vector4f vec_in_B = { vec_in[0], vec_in[1], vec_in[2], 1.0 };
	Eigen::Vector4f tmp = T_A2B*vec_in_B;
	Eigen::Vector3f vec_out_A = { tmp[0], tmp[1], tmp[2] };
	return vec_out_A;
}

/** @brief Korean: 입력된 벡터를 인덱스에 맞게 정렬한다.
*/
std::vector<Eigen::Vector3d> KUPCLMATH::genSortedVectorSet(const std::vector<Eigen::Vector3d> &vec_set_in, const std::vector<size_t> &sort_idx)
{
	std::vector<Eigen::Vector3d> vec_set_out;
	vec_set_out.resize(vec_set_in.size());
	for (size_t i = 0; i < sort_idx.size(); i++) { vec_set_out[i] = vec_set_in[sort_idx[i]]; }
	return vec_set_out;
}

/** @brief Korean: 입력된 벡터를 인덱스에 맞게 정렬한다.
*/
std::vector<Eigen::Matrix4d> KUPCLMATH::genSortedVectorSet(const std::vector<Eigen::Matrix4d> &vec_set_in, const std::vector<size_t> &sort_idx)
{
	std::vector<Eigen::Matrix4d> vec_set_out;
	vec_set_out.resize(vec_set_in.size());
	for (size_t i = 0; i < sort_idx.size(); i++) { vec_set_out[i] = vec_set_in[sort_idx[i]]; }
	return vec_set_out;
}

/** @brief Korean: 입력된 벡터를 인덱스에 맞게 정렬한다.
*/
std::vector<Eigen::Matrix4f> KUPCLMATH::genSortedVectorSet(const std::vector<Eigen::Matrix4f> &vec_set_in, const std::vector<size_t> &sort_idx)
{
	std::vector<Eigen::Matrix4f> vec_set_out;
	vec_set_out.resize(vec_set_in.size());
	for (size_t i = 0; i < sort_idx.size(); i++) { vec_set_out[i] = vec_set_in[sort_idx[i]]; }
	return vec_set_out;
}

/** @brief Korean: 입력된 벡터를 인덱스에 맞게 정렬한다.
*/
std::vector<double> KUPCLMATH::genSortedVectorSet(const std::vector<double> &vec_set_in, const std::vector<size_t> &sort_idx)
{
	std::vector<double> vec_set_out;
	vec_set_out.resize(vec_set_in.size());
	for (size_t i = 0; i < sort_idx.size(); i++) { vec_set_out[i] = vec_set_in[sort_idx[i]]; }
	return vec_set_out;
}

/** @brief Korean: 입력된 최소, 최대값 사이를 일정한 간격으로 나눈 벡터를 산출한다.
*/
std::vector<double> KUPCLMATH::genSameWidthValueSet(double x_min, double x_max, double scale) {
	
	std::vector<double> vec_out;
	if (scale < 0.000001)
	{
		vec_out.push_back(0.0);
		return vec_out;
	}

    if(fabs(x_max-x_min) < 1e-9)
    {
		vec_out.push_back(x_min);
		return vec_out;
    }

	int n = floor((x_max - x_min) / scale)+1;
	vec_out.reserve(n);
	double element = x_min;
	for (int i = 0; i < n; i++)
	{
		vec_out.push_back(element);
		element += scale;
	}
	return vec_out;
}

/** @brief Korean: 입력된 최소, 최대값 사이를 일정한 간격으로 나눈 벡터를 산출한다.
*/
std::vector<size_t> KUPCLMATH::genSameWidthValueSet(size_t x_min, size_t x_max, size_t scale) {

	std::vector<size_t> vec_out;
	if (scale == 0)
	{
		vec_out.push_back(0);
		return vec_out;
	}

    if(x_max-x_min == 0)
    {
		vec_out.push_back(x_min);
		return vec_out;
    }

	size_t n = floor((x_max - x_min) / scale) + 1;
	vec_out.reserve(n);
	size_t element = x_min;
	for (int i = 0; i < n; i++)
	{
		vec_out.push_back(element);
		element += scale;
	}
	return vec_out;
}

double KUPCLMATH::calRotationAngleFromNormal(const Eigen::Vector3d normal, const Eigen::Vector3d center, double angle_stage2base, int direction)
{
	double rotation_angle = 0.0;
	Eigen::Vector2d normal_2d_proj;
	switch (direction)
	{
	case 0: // in x direction

		break;

	case 1: // in y direction

		break;

	case 2: // in z direction
		Eigen::Vector3d rot_axis = {0.0, 0.0, 1.0};
		if (!dotCheck(normal.dot(rot_axis), 1.0)) 
		{
			normal_2d_proj[0] = normal[0];
			normal_2d_proj[1] = normal[1];
		}
		else // < 1.0[deg]
		{
			Eigen::Vector3d prj_normal_vector = { 0.0, 0.0, center[2] };
			Eigen::Vector3d prj_plane_vector = center - prj_normal_vector;
			if (prj_plane_vector.norm() > 1.0) // 1.0[mm]
			{
				prj_plane_vector.normalize();
				normal_2d_proj[0] = prj_plane_vector[0];
				normal_2d_proj[1] = prj_plane_vector[1];
			}
			else // x axis
			{
				normal_2d_proj[0] = 1.0;
				normal_2d_proj[1] = 0.0;
			}
		}
		normal_2d_proj.normalize();
		rotation_angle = atan2(normal_2d_proj[1], normal_2d_proj[0]);
		rotation_angle = rotation_angle * (180.0 / M_PI) + angle_stage2base;
		break;
	}
	return rotation_angle;
}

Eigen::Matrix4d KUPCLMATH::transformViewpoint2SensorPose(const Eigen::Vector3d &VP, const Eigen::Vector3d &center, const Eigen::Vector3d sensor_orientaion_rx)
{
	Eigen::Vector3d normal = VP - center;
	normal.normalize();

	Eigen::Vector3d cam_Rx = sensor_orientaion_rx;
	Eigen::Vector3d cam_Rz = normal;
	Eigen::Vector3d cam_Ry = cam_Rz.cross(cam_Rx);
	cam_Ry.normalize();
	Eigen::Matrix3d R_B2C_Pose = Eigen::Matrix3d::Identity();
	R_B2C_Pose.col(0) = cam_Rx;
	R_B2C_Pose.col(1) = cam_Ry;
	R_B2C_Pose.col(2) = cam_Rz;

	Eigen::Matrix4d T_B2C_Pose = Eigen::Matrix4d::Identity();
	T_B2C_Pose.block<3, 3>(0, 0) = R_B2C_Pose;
	T_B2C_Pose.block<3, 1>(0, 3) = VP;

	return T_B2C_Pose;
}

/** @brief Korean: std::vector 형식의 vector를 eigen 형식으로 변환한다.
*/
Eigen::VectorXd KUPCLMATH::vec2Eigen(const std::vector<double> &vec_in)
{
	std::vector<double> v = vec_in;
	double* ptr_data = &v[0];
	Eigen::VectorXd vec_out = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(v.data(), v.size());
	return vec_out;
}

/** @brief Korean: eigen 형식의 vector를 std::vector 형식으로 변환한다.
*/
std::vector<double> KUPCLMATH::eigen2Vec(const Eigen::VectorXd &vec_in)
{
	Eigen::VectorXd v = vec_in;
	std::vector<double> vec_out(&v[0], v.data() + v.cols()*v.rows());
	return vec_out;
}

void KUPCLMATH::transformCloudPose(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, const Eigen::VectorXf &pose, Eigen::Matrix4f &T_out) {
  Eigen::Matrix3d R_Rx, R_Ry, R_Rz;
  Eigen::Matrix4d T_Rx, T_Ry, T_Rz;
  genRotM(pose[3], R_Rx, T_Rx, 0); // rx
  genRotM(pose[4], R_Ry, T_Ry, 1); // ry
  genRotM(pose[5], R_Rz, T_Rz, 2); // rz

  Eigen::Matrix4d T_trans = Eigen::Matrix4d::Identity();
  T_trans(0, 3) = pose[0];
  T_trans(1, 3) = pose[1];
  T_trans(2, 3) = pose[2];

  Eigen::Matrix4d T = T_trans*T_Rz*T_Ry*T_Rx;
  T_out = T.cast<float>();

  //// points
  pcl::transformPointCloud(cloud, cloud, T.cast<float>()); // T_after2before

  //// normals
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  for(size_t i=0; i<cloud.size(); i++) {
    Eigen::Vector3d normal_tmp = {cloud.points[i].normal_x, cloud.points[i].normal_y, cloud.points[i].normal_z};

    normal_tmp = R*normal_tmp;
    cloud.points[i].normal_x = normal_tmp[0];
    cloud.points[i].normal_y = normal_tmp[1];
    cloud.points[i].normal_z = normal_tmp[2];
  }
}

void KUPCLMATH::transformCloudPose(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, const Eigen::VectorXd &pose, Eigen::Matrix4f &T_out) {
  Eigen::Matrix3d R_Rx, R_Ry, R_Rz;
  Eigen::Matrix4d T_Rx, T_Ry, T_Rz;
  genRotM(pose[3], R_Rx, T_Rx, 0); // rx
  genRotM(pose[4], R_Ry, T_Ry, 1); // ry
  genRotM(pose[5], R_Rz, T_Rz, 2); // rz

  Eigen::Matrix4d T_trans = Eigen::Matrix4d::Identity();
  T_trans(0, 3) = pose[0];
  T_trans(1, 3) = pose[1];
  T_trans(2, 3) = pose[2];

  Eigen::Matrix4d T = T_trans*T_Rz*T_Ry*T_Rx;
  T_out = T.cast<float>();

  //// points
  pcl::transformPointCloud(cloud, cloud, T.cast<float>()); // T_after2before

  //// normals
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  for(size_t i=0; i<cloud.size(); i++) {
    Eigen::Vector3d normal_tmp = {cloud.points[i].normal_x, cloud.points[i].normal_y, cloud.points[i].normal_z};

    normal_tmp = R*normal_tmp;
    cloud.points[i].normal_x = normal_tmp[0];
    cloud.points[i].normal_y = normal_tmp[1];
    cloud.points[i].normal_z = normal_tmp[2];
  }
}

//// FIXME: pcl::transformPointCloudWithNormals 함수를 사용할 것, 아래 함수를 사용할 필요없음
void KUPCLMATH::transformCloudHTM(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, const Eigen::Matrix4f &T) {
  //// points
  pcl::transformPointCloud(cloud, cloud, T); // T_after2before
  //// normals
  Eigen::Matrix3f R = T.block<3, 3>(0, 0);
  for(size_t i=0; i<cloud.size(); i++) {
    Eigen::Vector3f normal_tmp = {cloud.points[i].normal_x, cloud.points[i].normal_y, cloud.points[i].normal_z};

    normal_tmp = R*normal_tmp;
    cloud.points[i].normal_x = normal_tmp[0];
    cloud.points[i].normal_y = normal_tmp[1];
    cloud.points[i].normal_z = normal_tmp[2];
  }
}

//// FIXME: pcl::transformPointCloudWithNormals 함수를 사용할 것, 아래 함수를 사용할 필요없음
void KUPCLMATH::transformCloudHTM2(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_in, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_out, const Eigen::Matrix4f &T) {
  //// points
  pcl::transformPointCloud(cloud_in, cloud_out, T); // T_after2before
  //// normals
  Eigen::Matrix3f R = T.block<3, 3>(0, 0);
  for(size_t i=0; i<cloud_in.size(); i++) {
    Eigen::Vector3f normal_tmp = {cloud_in.points[i].normal_x, cloud_in.points[i].normal_y, cloud_in.points[i].normal_z};

    normal_tmp = R*normal_tmp;
    cloud_out.points[i].normal_x = normal_tmp[0];
    cloud_out.points[i].normal_y = normal_tmp[1];
    cloud_out.points[i].normal_z = normal_tmp[2];
  }
}

void KUPCLMATH::genCloudMinMax(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointXYZ &minPt, pcl::PointXYZ &maxPt) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(cloud, *cloud_tmp);
//   pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cloud_tmp, minPt, maxPt);
}

/** @brief Korean: RPY의 형태의 방위를 회전행렬(rotation matrix)로 변환
 * @param[in] dRPY : 입력 Roll-Pitch-Yaw (Rx, Ry, Rz)
 * @param[out] dRot : 출력 회전행렬
 */
void KUPCLMATH::RPY2Rot(double dRPY[3], double dRot[3][3]) {
    dRot[0][0] =  cos(dRPY[2]) * cos(dRPY[1]);
    dRot[0][1] =  cos(dRPY[2]) * sin(dRPY[1]) * sin(dRPY[0]) - sin(dRPY[2]) * cos(dRPY[0]);
    dRot[0][2] =  cos(dRPY[2]) * sin(dRPY[1]) * cos(dRPY[0]) + sin(dRPY[2]) * sin(dRPY[0]);

    dRot[1][0] =  sin(dRPY[2]) * cos(dRPY[1]);
    dRot[1][1] =  sin(dRPY[2]) * sin(dRPY[1]) * sin(dRPY[0]) + cos(dRPY[2]) * cos(dRPY[0]);
    dRot[1][2] =  sin(dRPY[2]) * sin(dRPY[1]) * cos(dRPY[0]) - cos(dRPY[2]) * sin(dRPY[0]);

    dRot[2][0] = -sin(dRPY[1]);
    dRot[2][1] =  cos(dRPY[1]) * sin(dRPY[0]);
    dRot[2][2] =  cos(dRPY[1]) * cos(dRPY[0]);
}

/** @brief Korean: 로봇의 TCP 정보를 입력한다.
 * @param[in] tcp : 로봇의 TCP 정보, [m], [rad]
 */
void KUPCLMATH::SetTcp(std::vector<double> tcp) {

    double rot[3][3] = {{0.0,},};
    double rpyofTcp[3] = {tcp[3], tcp[4], tcp[5]};

    RPY2Rot(rpyofTcp, rot);
    for (std::size_t i = 0; i < 3; i++) {
        for (std::size_t  j = 0; j < 3; j++) {
            m_TCP(i,j) = rot[i][j];
        }
        m_TCP(i,3) = tcp[i];
    }
}

/////////////////////////////////////
std::vector<double> KUPCLMATH::htm2pose_mm2m(const Eigen::Matrix4f &T, std::string str)
{
    Eigen::VectorXf P_tgt(6);
    Eigen::Matrix4f T_tgt = T;
    tr2pose(T_tgt, P_tgt);
    for (int j = 3; j < 6; j++)
    {
        P_tgt[j] = (180.0 / M_PI) * P_tgt[j];
    } // deg
    // printf("target (%s) pose: %f %f %f %f %f %f\n", str.c_str(), P_tgt(0), P_tgt(1), P_tgt(2), P_tgt(3), P_tgt(4), P_tgt(5));

    std::vector<double> grasping_pose;
    grasping_pose.resize(6);
    for (int i = 0; i < 6; i++)
    {
        if (i < 3)
        {
            grasping_pose[i] = 0.001 * P_tgt[i]; // [m]
        }
        else
        {
            grasping_pose[i] = P_tgt[i]; // [deg]
        }
    }

    return grasping_pose; // [m], [deg]
}

std::vector<double> KUPCLMATH::htm2pose(const Eigen::Matrix4f &T, std::string str)
{
    Eigen::VectorXf P_tgt(6);
    Eigen::Matrix4f T_tgt = T;
    tr2pose(T_tgt, P_tgt);
    for (int j = 3; j < 6; j++)
    {
        P_tgt[j] = (180.0 / M_PI) * P_tgt[j];
    } // deg
    // printf("target (%s) pose: %f %f %f %f %f %f\n", str.c_str(), P_tgt(0), P_tgt(1), P_tgt(2), P_tgt(3), P_tgt(4), P_tgt(5));

    std::vector<double> grasping_pose;
    grasping_pose.resize(6);
    for (int i = 0; i < 6; i++)
    {
        if (i < 3)
        {
            grasping_pose[i] = P_tgt[i];
        }
        else
        {
            grasping_pose[i] = P_tgt[i]; // [deg]
        }
    }

    return grasping_pose; // [deg]
}

/** @brief Korean: RPY 형식의 자세를 변환행렬로 반환한다.
 * @param[in] rot : 입력 회전행렬
 * @param[in] trans_x : translation x
 * @param[in] trans_y : translation y
 * @param[in] trans_z : translation z
 * @param[in] method : 회전 축 선택 - (0: x-axis, 1: y-axis, 2: z-axis)
 */
Eigen::Matrix4f KUPCLMATH::pose2htm(const std::vector<double> &pose_in)
{
	//// Defining a rotation matrix and translation vector
	//// xyz, roll, pitch, yaw to 4x4 Homogeneous Transform Matrix
	// pose input
	double x = pose_in[0];
	double y = pose_in[1];
	double z = pose_in[2];
	double roll = pose_in[3]; // [rad]
	double pitch = pose_in[4]; // [rad]
	double yaw = pose_in[5]; // [rad]

	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f translation_matrix = genTranslationHTM(x, y, z);
	Eigen::Matrix4f Rotation_matrix = genRzRotationHTM(yaw) * genRyRotationHTM(pitch) * genRxRotationHTM(roll);

	transformation_matrix = translation_matrix*Rotation_matrix;
	return transformation_matrix;
    
}

/** @brief Korean: 입력 벡터를 기준으로 beta만큼 회전하는 회전행렬 반환한다.
 * @param[in] vec_in : 입력 벡터
 * @param[in] beta : 회전 각도 [deg]
 * @return R : 회전행렬을 반환
 */
Eigen::Matrix3f KUPCLMATH::genRotationAroundVector(const Eigen::Vector3f &vec_in, double beta)
{
	Eigen::Matrix3f M;
	M(0, 0) = 0;			M(0, 1) = -vec_in[2];	M(0, 2) = vec_in[1];
	M(1, 0) = vec_in[2];	M(1, 1) = 0;			M(1, 2) = -vec_in[0];
	M(2, 0) = -vec_in[1];	M(2, 1) = vec_in[0];	M(2, 2) = 0;

	Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
	Eigen::Matrix3f R = vec_in*vec_in.transpose() + cos(beta * M_PI / 180.0)*(I - vec_in*vec_in.transpose()) + sin(beta * M_PI / 180.0)*M;

	return R;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////// Print function //////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void KUPCLMATH::printCloudMinMax(const pcl::PointCloud<pcl::PointXYZ> &cloud, const std::string &str) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(cloud, *cloud_tmp);
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cloud_tmp, minPt, maxPt);
  if(fabs(minPt.x) > 1.0 &&
  	fabs(minPt.y) > 1.0 &&
  	fabs(minPt.z) > 1.0 &&
	fabs(maxPt.x) > 1.0 &&
	fabs(maxPt.y) > 1.0 &&
	fabs(maxPt.z) > 1.0) {
		printf("cloud (%s): (min,max): x: (%0.1f, %0.1f), y: (%0.1f, %0.1f), z: (%0.1f, %0.1f)\n", str.c_str(), minPt.x, maxPt.x, minPt.y, maxPt.y, minPt.z, maxPt.z);
  } else {
	  printf("cloud (%s): (min,max): x: (%0.4f, %0.4f), y: (%0.4f, %0.4f), z: (%0.4f, %0.4f)\n", str.c_str(), minPt.x, maxPt.x, minPt.y, maxPt.y, minPt.z, maxPt.z);
  }
}

void KUPCLMATH::printCloudMinMax(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const std::string &str) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(cloud, *cloud_tmp);
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cloud_tmp, minPt, maxPt);
  if(fabs(minPt.x) > 1.0 &&
  	fabs(minPt.y) > 1.0 &&
  	fabs(minPt.z) > 1.0 &&
	fabs(maxPt.x) > 1.0 &&
	fabs(maxPt.y) > 1.0 &&
	fabs(maxPt.z) > 1.0) {
		printf("cloud (%s): (min,max): x: (%0.1f, %0.1f), y: (%0.1f, %0.1f), z: (%0.1f, %0.1f)\n", str.c_str(), minPt.x, maxPt.x, minPt.y, maxPt.y, minPt.z, maxPt.z);
  } else {
	  printf("cloud (%s): (min,max): x: (%0.4f, %0.4f), y: (%0.4f, %0.4f), z: (%0.4f, %0.4f)\n", str.c_str(), minPt.x, maxPt.x, minPt.y, maxPt.y, minPt.z, maxPt.z);
  }
}

void KUPCLMATH::printCloudMinMax(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, const std::string &str) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(cloud, *cloud_tmp);
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cloud_tmp, minPt, maxPt);
  if(fabs(minPt.x) > 1.0 &&
  	fabs(minPt.y) > 1.0 &&
  	fabs(minPt.z) > 1.0 &&
	fabs(maxPt.x) > 1.0 &&
	fabs(maxPt.y) > 1.0 &&
	fabs(maxPt.z) > 1.0) {
		printf("cloud (%s): (min,max): x: (%0.1f, %0.1f), y: (%0.1f, %0.1f), z: (%0.1f, %0.1f)\n", str.c_str(), minPt.x, maxPt.x, minPt.y, maxPt.y, minPt.z, maxPt.z);
  } else {
	  printf("cloud (%s): (min,max): x: (%0.4f, %0.4f), y: (%0.4f, %0.4f), z: (%0.4f, %0.4f)\n", str.c_str(), minPt.x, maxPt.x, minPt.y, maxPt.y, minPt.z, maxPt.z);
  }
}

void KUPCLMATH::printCloudMinMax(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, const std::string &str) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(cloud, *cloud_tmp);
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cloud_tmp, minPt, maxPt);
  if(fabs(minPt.x) > 1.0 &&
  	fabs(minPt.y) > 1.0 &&
  	fabs(minPt.z) > 1.0 &&
	fabs(maxPt.x) > 1.0 &&
	fabs(maxPt.y) > 1.0 &&
	fabs(maxPt.z) > 1.0) {
		printf("cloud (%s): (min,max): x: (%0.1f, %0.1f), y: (%0.1f, %0.1f), z: (%0.1f, %0.1f)\n", str.c_str(), minPt.x, maxPt.x, minPt.y, maxPt.y, minPt.z, maxPt.z);
  } else {
	  printf("cloud (%s): (min,max): x: (%0.4f, %0.4f), y: (%0.4f, %0.4f), z: (%0.4f, %0.4f)\n", str.c_str(), minPt.x, maxPt.x, minPt.y, maxPt.y, minPt.z, maxPt.z);
  }
}

void KUPCLMATH::printCloudPoints(const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, size_t point_size) {
    for (size_t i = 0; i < point_size; i++)
    {
        printf("pt #%zu: %f %f %f\n", i+1, cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
        printf("normal #%zu: %f %f %f\n", i+1, cloud.points[i].normal_x, cloud.points[i].normal_y, cloud.points[i].normal_z);
        printf("rgb #%zu: %i %i %i\n", i+1, cloud.points[i].r, cloud.points[i].g, cloud.points[i].b);
    }
}

void KUPCLMATH::printCloudPoints(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, size_t point_size) {
    for (size_t i = 0; i < point_size; i++)
    {
        printf("pt #%zu: %f %f %f\n", i+1, cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
        // printf("normal #%zu: %f %f %f\n", i+1, cloud.points[i].normal_x, cloud.points[i].normal_y, cloud.points[i].normal_z);
        printf("rgb #%zu: %i %i %i\n", i+1, cloud.points[i].r, cloud.points[i].g, cloud.points[i].b);
    }
}

void KUPCLMATH::printVector(const std::vector<double> &vec_in, std::string str)
{
    printf("%s: ", str.c_str());
    for (size_t i = 0; i < vec_in.size(); i++)
    {
        if(i != vec_in.size()-1) printf("%0.5f, ", vec_in[i]);
        else printf("%0.5f\n", vec_in[i]);
    }
}

void KUPCLMATH::printVector(const std::vector<float> &vec_in, std::string str)
{
    printf("%s: ", str.c_str());
    for (size_t i = 0; i < vec_in.size(); i++)
    {
        if(i != vec_in.size()-1) printf("%0.5f, ", vec_in[i]);
        else printf("%0.5f\n", vec_in[i]);
    }
}

void KUPCLMATH::printVector(const Eigen::VectorXf &vec_in, std::string str)
{
    printf("%s: ", str.c_str());
    for (size_t i = 0; i < vec_in.size(); i++)
    {
        if(i != vec_in.size()-1) printf("%0.5f, ", vec_in[i]);
        else printf("%0.5f\n", vec_in[i]);
    }
}

void KUPCLMATH::printVector3f(const Eigen::Vector3f &vec_in, std::string str)
{
    printf("%s: ", str.c_str());
    for (size_t i = 0; i < vec_in.size(); i++)
    {
        if(i != vec_in.size()-1) printf("%0.5f, ", vec_in[i]);
        else printf("%0.5f\n", vec_in[i]);
    }
}

void KUPCLMATH::printHTM(const Eigen::Matrix4d &T, std::string str) {
  printf("%s\n", str.c_str());
  printf("    | %6.3f %6.3f %6.3f | \n", T(0, 0), T(0, 1), T(0, 2));
  printf("R = | %6.3f %6.3f %6.3f | \n", T(1, 0), T(1, 1), T(1, 2));
  printf("    | %6.3f %6.3f %6.3f | \n", T(2, 0), T(2, 1), T(2, 2));
  printf("\n");
  printf("t = < %0.3f, %0.3f, %0.3f >\n", T(0,3), T(1,3), T(2,3));
}

void KUPCLMATH::printHTM(const Eigen::Matrix4f &T, std::string str) {
  printf("%s\n", str.c_str());
  printf("    | %6.6f %6.6f %6.6f | \n", T(0, 0), T(0, 1), T(0, 2));
  printf("R = | %6.6f %6.6f %6.6f | \n", T(1, 0), T(1, 1), T(1, 2));
  printf("    | %6.6f %6.6f %6.6f | \n", T(2, 0), T(2, 1), T(2, 2));
  printf("\n");
  printf("t = < %0.3f, %0.3f, %0.3f >\n", T(0,3), T(1,3), T(2,3));
}
