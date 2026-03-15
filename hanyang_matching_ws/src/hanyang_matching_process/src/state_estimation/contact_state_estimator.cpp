/**
 * @file contact_state_estimator.cpp
 * @brief 접촉상태 판별을 위한 헤더파일
 */

#include "state_estimation/contact_state_estimator.h"
#include <iostream>
#include <cmath>
#include <string.h>

CContactStateEstimator::CContactStateEstimator(void) {

}

CContactStateEstimator::~CContactStateEstimator(void) {
}

/** @brief Korean: 벡터 거리(VD, vector distance) 기반의 유사도를 산출한다.
 * @param[in] dRobotPose : 로봇의 측정 자세
 * @param[in] dFTsensorData : 측정된 힘/토크 데이터
 */
void CContactStateEstimator::CalSimilarityVD(double dRobotPose[], double dFTsensorData[])
{
    // 1. 실시간 측정 데이터 처리
	Eigen::VectorXf dMeasuredData(m_data_dim);
	if(m_data_dim > 6) 
	{
		for(int i=0;i<6;i++) 
		{ 
			dMeasuredData[i] = dFTsensorData[i];
			dMeasuredData[i+6] = dRobotPose[i]; 
		}
	}
	else 
	{
		for(int i=0;i<m_data_dim;i++) { dMeasuredData[i] = dFTsensorData[i]; }
	}
    // // rx, rz 각도 변환
    // if(dMeasuredData[9]<=-90.0) dMeasuredData[9] = dMeasuredData[9] + 360.0; // Pose Rx
    // if(dMeasuredData[11]<=-90.0) dMeasuredData[11] = dMeasuredData[11] + 360.0; // Pose Ry

    // 3차, 실시간 데이터 모델행렬의 Max,Min으로 정규화
    for(int i=0;i<m_data_dim;i++) dMeasuredData[i] = (dMeasuredData[i]-m_model_normalize_min_list[i])/(m_model_normalize_max_list[i]-m_model_normalize_min_list[i]);

	// 유사도지수 계산
	double vector_distance = 0.0;
	Eigen::VectorXf similarity_vector = Eigen::VectorXf::Zero(m_state_num);
	Eigen::VectorXf input_vector;
	for(size_t i = 0; i<m_state_num; i++) // state_num
	{
		for(size_t j = 0; j <m_model_matrix_VD[i].rows() ; j++)
		{
			input_vector = m_model_matrix_VD[i].row(j)-dMeasuredData;
			input_vector = m_model_weight[i].cwiseSqrt()*input_vector; // m_model_weight는 각 항의 제곱에 곱해지는 계수
			vector_distance = input_vector.norm();
			similarity_vector[i] += 1 / vector_distance; 
		}
		// update
		m_dSimilarity_VD[i] = similarity_vector[i];
	}

	// // // // // // // if(m_AssemblyDenso.GetPointContact()||m_AssemblyDenso.GetLineContact()||m_AssemblyDenso.GetSurfaceContact())
	// // // // // // // {
	// // // // // // // 	m_VecDis[i] = sqrt(
	// // // // // // // 		powf((m_dModelMatrix[i][0] - m_dfx),2) +
	// // // // // // // 		powf((m_dModelMatrix[i][1] - m_dfy),2) +
	// // // // // // // 		powf((m_dModelMatrix[i][2] - m_dfz),2) + 
	// // // // // // // 		powf((m_dModelMatrix[i][3] - m_dtx),2) + 
	// // // // // // // 		powf((m_dModelMatrix[i][4] - m_dty),2) + 
	// // // // // // // 		powf((m_dModelMatrix[i][5] - m_dtz),2));
	// // // // // // // }
	// // // // // // // else
	// // // // // // // {
	// // // // // // // 	m_VecDis[i] = sqrt(
	// // // // // // // 		m_dWeightSim[0][0]*powf((m_dModelMatrix[i][0] - m_dfx),2) +
	// // // // // // // 		m_dWeightSim[0][1]*powf((m_dModelMatrix[i][1] - m_dfy),2) +
	// // // // // // // 		m_dWeightSim[0][2]*powf((m_dModelMatrix[i][2] - m_dfz),2) + 
	// // // // // // // 		m_dWeightSim[0][3]*powf((m_dModelMatrix[i][3] - m_dtx),2) + 
	// // // // // // // 		m_dWeightSim[0][4]*powf((m_dModelMatrix[i][4] - m_dty),2) + 
	// // // // // // // 		m_dWeightSim[0][5]*powf((m_dModelMatrix[i][5] - m_dtz),2) +
	// // // // // // // 		m_dWeightSim[0][6]*powf((m_dModelMatrix[i][6] - m_px),2) + 
	// // // // // // // 		m_dWeightSim[0][7]*powf((m_dModelMatrix[i][7] - m_py),2) + 
	// // // // // // // 		m_dWeightSim[0][8]*powf((m_dModelMatrix[i][8] - m_pz),2) + 
	// // // // // // // 		m_dWeightSim[0][9]*powf((m_dModelMatrix[i][9] - m_rx),2) + 
	// // // // // // // 		m_dWeightSim[0][10]*powf((m_dModelMatrix[i][10] - m_ry),2) +
	// // // // // // // 		m_dWeightSim[0][11]*powf((m_dModelMatrix[i][11] - m_rz),2));

	// // // // // // // 	//*유사도 지수 구하기*/
	// // // // // // // 	m_SimIndex[i] = 1 / m_VecDis[i]; 

	// // // // // // // }


}

/** @brief Korean: ()벡터 거리(VD, vector distance) 기반의 유사도를 산출한다.
 * @param[in] dRobotPose : 로봇의 측정 자세
 * @param[in] dFTsensorData : 측정된 힘/토크 데이터
 */
Eigen::VectorXf CContactStateEstimator::CalSimilarityVD2(const std::vector<Eigen::MatrixXf> &model_matrix, const Eigen::VectorXf &vec_in, const Eigen::VectorXf &model_weight)
{
	double vector_distance = 0.0;
	Eigen::VectorXf similarity_vector = Eigen::VectorXf::Zero(1);
	Eigen::VectorXf input_vector;
	Eigen::VectorXf weight = model_weight.cwiseSqrt(); // m_model_weight는 각 항의 제곱에 곱해지는 계수
	for (size_t i = 0; i < model_matrix.size(); i++) // state_num
	{
		for (size_t j = 0; j < model_matrix[i].rows(); j++)
		{
			Eigen::VectorXf input_tmp = model_matrix[i].row(j);
			input_vector = input_tmp-vec_in;
			for (int k=0; k<input_vector.size(); k++) { input_vector[k] = weight[k]*input_vector[k]; }
			vector_distance = input_vector.norm();

			if(vector_distance > 1e-6) { similarity_vector[i] += 1 / vector_distance; }
			else { similarity_vector[i] = 1e6; }
			
		}
	}
	return similarity_vector;
}

/** @brief Korean: 특이값 분해(SVD, singular value decomposition) 기반의 유사도를 산출한다.
 * @param[in] dRobotPose : 로봇의 측정 자세
 * @param[in] dFTsensorData : 측정된 힘/토크 데이터
 */
void CContactStateEstimator::CalSimilaritySVD(double dRobotPose[], double dFTsensorData[])
{
    // 1. 실시간 측정 데이터 처리
	Eigen::VectorXf dMeasuredData(m_data_dim);
	if(m_data_dim > 6) 
	{
		for(int i=0;i<6;i++) 
		{ 
			dMeasuredData[i] = dFTsensorData[i];
			dMeasuredData[i+6] = dRobotPose[i]; 
		}
	}
	else 
	{
		for(int i=0;i<m_data_dim;i++) { dMeasuredData[i] = dFTsensorData[i]; }
	}
    // // rx, rz 각도 변환
    // if(dMeasuredData[9]<=-90.0) dMeasuredData[9] = dMeasuredData[9] + 360.0; // Pose Rx
    // if(dMeasuredData[11]<=-90.0) dMeasuredData[11] = dMeasuredData[11] + 360.0; // Pose Ry

    // 3차, 실시간 데이터 모델행렬의 Max,Min으로 정규화
    for(int i=0;i<m_data_dim;i++) dMeasuredData[i] = (dMeasuredData[i]-m_model_normalize_min_list[i])/(m_model_normalize_max_list[i]-m_model_normalize_min_list[i]);

    //// 2. Measured data processing
    Eigen::VectorXf dMeasVec_SVD(6);
    double dMeasured_SVD[6] = {0.0};
    for(int i=0;i<6;i++) dMeasured_SVD[i] = dMeasuredData[i];
    for(int i=0;i<6;i++) dMeasVec_SVD(i) = dMeasured_SVD[i];
    Eigen::MatrixXf M_MeasQ_SVD = dMeasVec_SVD*dMeasVec_SVD.transpose(); // (12by1) * (1by12)
    Eigen::JacobiSVD<Eigen::MatrixXf> svd_MeasQ(M_MeasQ_SVD, Eigen::ComputeFullU | Eigen::ComputeFullV);
    //JacobiSVD<Eigen::MatrixXf> svd_MeasQ(M_MeasQ_SVD, ComputeThinU | ComputeThinV);
    Eigen::MatrixXf M_UofMeasQ_SVD = svd_MeasQ.matrixU(); 	
    Eigen::MatrixXf M_SofMeasQ_SVD = svd_MeasQ.singularValues(); 	

    //// 3. Similarity metric (like euclidean distance)
	std::vector<double> sum_SVD_first(m_state_num);
	std::vector<double> sum_SVD_second(m_state_num);
	double sum_vec_S, sum_vec_S_meas;
	for(size_t i = 0; i<m_state_num; i++) // state_num
	{
		for(int j=0; j<m_data_dim; j++)
    	{
        	sum_SVD_first[i] += m_model_matrix_S_SVD[i](j,0)*(m_model_matrix_U_SVD[i].col(j)).dot(M_UofMeasQ_SVD.col(j));
        	sum_SVD_second[i] += M_SofMeasQ_SVD(j,0)*(m_model_matrix_U_SVD[i].col(j)).dot(M_UofMeasQ_SVD.col(j));
		}
		sum_vec_S = m_model_matrix_S_SVD[i].col(0).sum();
		sum_vec_S_meas = M_SofMeasQ_SVD.col(0).sum();

		sum_SVD_first[i] = sum_SVD_first[i]/fabs(sum_vec_S);
		sum_SVD_second[i] = sum_SVD_second[i]/fabs(sum_vec_S_meas);

		// 유사도 결과
		if(sum_SVD_first[i] > sum_SVD_second[i]) { m_dSimilarity_SVD[i] = sum_SVD_first[i]; }
		else { m_dSimilarity_SVD[i] = sum_SVD_second[i]; }
	}
}

/** @brief Korean: 마할라노비스 거리(MD, mahalanobis distance) 기반의 유사도를 산출한다.
 * @param[in] dRobotPose : 로봇의 측정 자세
 * @param[in] dFTsensorData : 측정된 힘/토크 데이터
 */
void CContactStateEstimator::CalSimilarityMD(double dRobotPose[], double dFTsensorData[])
{
	Eigen::VectorXf dMeasuredData(m_data_dim);
	if(m_data_dim > 6) 
	{
		for(int i=0;i<6;i++) 
		{ 
			dMeasuredData[i] = dFTsensorData[i];
			dMeasuredData[i+6] = dRobotPose[i]; 
		}
	}
	else 
	{
		for(int i=0;i<m_data_dim;i++) { dMeasuredData[i] = dFTsensorData[i]; }
	}

	// Mahalanobis distance
	double mahalanobis_dist, mahalanobis_similarity = 0.0;
	for(size_t i = 0; i<m_state_num; i++) // state_num
	{
		Eigen::VectorXf dMeasVec_MD = dMeasuredData-m_mean_vector_MD[i]; // (Measured - Mean value)
		Eigen::MatrixXf M_Cov_inv = m_model_matrix_MD[i];
		mahalanobis_dist = dMeasVec_MD.transpose()*M_Cov_inv*dMeasVec_MD;
		mahalanobis_similarity = 1/sqrt(mahalanobis_dist);
		m_dSimilarity_MD[i] = mahalanobis_similarity;
	}
}

/** @brief Korean: 마할라노비스 거리(MD, mahalanobis distance) 기반의 유사도를 산출한다.
 * @details 예시) 0.02s의 제어주기로 0.8초 동안 수집한 조립 수행 1회에 대한 접촉 상태의 데이터는 40개, 이때 중간에서 데이터 10개(model_size)만 추출하여 조립모델 생성 --> 조립 10회 수행 시, 총 100x6(F/T) 또는 100x12(F/T+Pose)의 크기를 갖는 조립모델 행렬 산출 
 * @param[in] measured_data : 접촉상태별로 수집한 측정 데이터 (6차원: F/T 데이터, 12차원: F/T 데이터 + 로봇 자세)
 * @param[in] model_size : 모델의 샘플링 데이터 크기
 */
void CContactStateEstimator::GenContactStateModelMatrix(std::vector<std::vector<Eigen::MatrixXf>> &measured_data, size_t model_size)
{
	//// 전처리과정, 1. 모델 매트릭스 생성, 2. 실시간 데이터 정규화를 위한 Min Max 값 계산, 3. 바타차야거리를 이용한 둔감성분 제거 , 4. 유사도 임계값 계산
	printf("\n[전처리과정 시작]\n");

	//// 1. 실시간 데이터 정규화를 위한 Min Max 값 계산
	Eigen::MatrixXf input_data;
	int idx_min, idx_max = 0; double min_val, max_val = 0.0;

	std::vector<double> vec_tmp(model_size); // ex) 수집된 데이터의 중간값으로부터 10개
	size_t state_num = measured_data.size();
	size_t assembly_num = measured_data[0].size();
	size_t data_num = static_cast<size_t>(measured_data[0][0].rows()); // data 수집 횟수
	size_t data_dim = static_cast<size_t>(measured_data[0][0].cols()); // data 차원
	size_t sampling_start = static_cast<size_t>(measured_data[0][0].rows()/2);

	// update
	m_state_num = state_num;
	m_data_num = data_num;
	m_data_dim = data_dim;
	m_model_size = model_size;
	m_sampling_start = sampling_start;

	std::vector<Eigen::VectorXf> min_val_weight(state_num);
	std::vector<Eigen::VectorXf> max_val_weight(state_num);
	Eigen::VectorXf min_tmp, max_tmp, sum_tmp;

	m_model_normalize_min_list = Eigen::VectorXf::Zero(12);
	m_model_normalize_max_list = Eigen::VectorXf::Zero(12);
	Eigen::MatrixXf model_normalize_min(state_num,data_dim);
	Eigen::MatrixXf model_normalize_max(state_num,data_dim);
	for(size_t i=0; i<state_num; i++) // num_state 
	{
		Eigen::MatrixXf model_min_list(assembly_num,data_dim);
		Eigen::MatrixXf model_max_list(assembly_num,data_dim);

		for(size_t j=0; j<assembly_num; j++) // assembly count, 10
		{
			input_data = measured_data[i][j];
			// Block of size (p,q), starting at (i,j), matrix.block(i,j,p,q);, matrix.block<p,q>(i,j);
			// Eigen::MatrixXf sampling_data = input_data.block<model_size,data_dim>(sampling_start,0);
			Eigen::MatrixXf sampling_data = input_data.block(sampling_start,0,model_size,data_dim); 
			Eigen::VectorXf minVal = sampling_data.colwise().minCoeff();
			Eigen::VectorXf maxVal = sampling_data.colwise().maxCoeff();
			// std::cout << "maxVal " << maxVal << endl;
			model_min_list.row(j) = minVal;
			model_max_list.row(j) = maxVal;
		}
		min_tmp = model_min_list.colwise().minCoeff();
		max_tmp = model_min_list.colwise().maxCoeff();
		sum_tmp = model_min_list.colwise().sum();
		model_normalize_min.row(i) = sum_tmp-min_tmp-max_tmp;
		model_normalize_min.row(i) = model_normalize_min.row(i)/(data_num-2);
		min_val_weight[i] = min_tmp; // weight 산출 시 사용

		min_tmp = model_max_list.colwise().minCoeff();
		max_tmp = model_max_list.colwise().maxCoeff();
		sum_tmp = model_max_list.colwise().sum();
		model_normalize_max.row(i) = sum_tmp-min_tmp-max_tmp;
		model_normalize_max.row(i) = model_normalize_max.row(i)/(data_num-2);
		max_val_weight[i] = max_tmp; // weight 산출 시 사용
	}
	m_model_normalize_min_list = model_normalize_min.colwise().minCoeff();
	m_model_normalize_max_list = model_normalize_max.colwise().maxCoeff();
	printf("min, max 값 계산완료\n");
	std::cout << " min_normalization: " << m_model_normalize_min_list << std::endl; 
	std::cout << " max_normalization: " << m_model_normalize_max_list << std::endl; 

	//// 2. 모델 매트릭스 생성
	// a) VD(vector distance)-based similarity assembly model
	m_model_matrix_VD.clear();
	m_model_matrix_VD.resize(state_num);
	for(size_t i=0; i<state_num; i++) // num_state 
	{
		Eigen::MatrixXf model_matrix(assembly_num*model_size, data_dim); // 100 by 12
		for(size_t j=0; j<assembly_num; j++) // assembly count, 10
		{
			input_data = measured_data[i][j];
			// Eigen::MatrixXf sampling_data = input_data.block<model_size,data_dim>(sampling_start,0);
			Eigen::MatrixXf sampling_data = input_data.block(sampling_start,0,model_size,data_dim); 
			for(size_t p=0; p<model_size; p++)
			{
				// normalization
				for(size_t k=0; k<data_dim; k++) { model_matrix(model_size*j+p, k) = (sampling_data(p, k)-m_model_normalize_min_list[k])/(m_model_normalize_max_list[k]-m_model_normalize_min_list[k]); } // 데이터 정규화
			}
		}
		// update
		m_model_matrix_VD[i] = model_matrix;
	}

	// b) SVD(singular value decomposition)-based similarity assembly model
	m_model_matrix_U_SVD.clear();
	m_model_matrix_S_SVD.clear();
	m_model_matrix_U_SVD.resize(state_num);
	m_model_matrix_S_SVD.resize(state_num);

	Eigen::MatrixXf input_SVD, M_Q_SVD;
	for(size_t i=0; i<state_num; i++) 
	{ 
		// 1) input from VD model matrix
		input_SVD = m_model_matrix_VD[i]; 
		// 2) Q matrix calculation
		M_Q_SVD = input_SVD.transpose()*input_SVD; // 12 by 12 
		// 3) SVD  [U,S,V] = svd(Q)
		Eigen::JacobiSVD<Eigen::MatrixXf> svd_AS( M_Q_SVD, Eigen::ComputeFullU | Eigen::ComputeFullV);
		// SVD 결과 오차 확인 부분
		//MatrixXd Cp = svd.matrixU() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();
		//MatrixXd diff = Cp - M_QofA1_SVD;
		//cout << "diff:\n" << diff.array().abs().sum() << "\n";// SVD 결과 오차

		m_model_matrix_U_SVD[i] = svd_AS.matrixU();
		m_model_matrix_S_SVD[i] = svd_AS.singularValues();
	}

	// c) MD(Mahalanobis distance)-based similarity assembly model
	m_model_matrix_MD.clear();
	m_mean_vector_MD.clear();
	m_model_matrix_MD.resize(state_num);
	m_mean_vector_MD.resize(state_num);
	// input data
	double sum_MD;
	for(size_t i=0; i<state_num; i++) // num_state 
	{
		// mean vector
		Eigen::MatrixXf model_matrix(assembly_num*model_size, data_dim); // 100 by 12
		for(size_t j=0; j<assembly_num; j++) // assembly count, 10
		{
			input_data = measured_data[i][j];
			// Eigen::MatrixXf sampling_data = input_data.block<model_size,data_dim>(sampling_start,0);
			Eigen::MatrixXf sampling_data = input_data.block(sampling_start,0,model_size,data_dim); 
			for(size_t p=0; p<model_size; p++)
			{   // raw data for MD
				for(size_t k=0; k<data_dim; k++) { model_matrix(model_size*j+p, k) = sampling_data(p, k); } 
			}
		}
		m_mean_vector_MD[i] = model_matrix.colwise().mean();
		Eigen::MatrixXf centered = model_matrix.rowwise() - model_matrix.colwise().mean(); // data centering
		std::cout << "Centering matrix : " << centered << std::endl;
		// update
		// Eigen::MatrixXf M_Cov = (centered.transpose() * centered) / double(model_matrix.cols() - 1); // Covariance
		// Eigen::MatrixXf M_Cov = (centered.adjoint() * centered) / double(model_matrix.cols() - 1); // Covariance
		Eigen::MatrixXf M_Cov = (centered.adjoint() * centered) / double(model_matrix.rows() - 1); // Covariance
		m_model_matrix_MD[i] = M_Cov.inverse(); // inverse
		std::cout << "Covariance inverse : " << m_model_matrix_MD[i] << std::endl;
	}
	printf("모델매트릭스 생성 완료\n");

	//// 3-1. 바타차야거리를 이용한 둔감성분 제거
	// 바타차야 거리 계산 후 가중치 업데이트 - 0 또는 1의 값(해당 변수의 사용 여부만을 선택)
	// 총 11회 데이터 수집
	// 1~10th 데이터: 모델행렬 산출, 11th 데이터: 바타차야 거리 및 임계값 산출에 1회 사용
	if(1) { checkCorrelationUsingBhattacharyya(measured_data); }
	else { for(size_t k=0; k<state_num; k++) { for(size_t i=0; i<12; i++) { m_model_weight[k][i] = 1.0; } } }

	//// 3-2. weight 산출
	calWeight(measured_data, min_val_weight, max_val_weight, sampling_start);

	//// 
	m_dSimilarity_VD.clear();
	m_dSimilarity_SVD.clear();
	m_dSimilarity_MD.clear();
	m_dSimilarity_VD.resize(state_num);
	m_dSimilarity_SVD.resize(state_num);
	m_dSimilarity_MD.resize(state_num);
}

/** @brief Korean: 조립 데이터를 이용하여 상태 판별 알고리즘의 임계값을 산출한다.
 * @param[in] process_data : 조립공정 1회의 전체 데이터 (6차원: F/T 데이터, 12차원: F/T 데이터 + 로봇 자세)
 */
void CContactStateEstimator::calThreshold(std::vector<Eigen::VectorXf> &process_data, std::vector<std::vector<size_t>> &contact_index, double margin, double ratio)
{
	printf("\n<4. 유사도 임계값 계산 과정>\n");
	size_t state_num = m_model_matrix_VD.size();
	size_t data_num = process_data.size(); // data 개수
	size_t data_dim = process_data[0].size(); // data 차원
	//// 4. 유사도 임계값 계산
	Eigen::MatrixXf input_data(data_num,data_dim);
	for(size_t i=0; i<data_num; i++)
	{
		input_data.row(i) = process_data[i];
		// normalization
		for(size_t j=0; j<data_dim; j++)
		{
			input_data(i,j) =(input_data(i,j)-m_model_normalize_min_list[j])/(m_model_normalize_max_list[j]-m_model_normalize_min_list[j]);
		}
	}

	// 유사도지수 계산
	double vector_distance = 0.0;
	Eigen::VectorXf input_vector;
	Eigen::MatrixXf model_matrix;
	Eigen::VectorXf similarity_vector = Eigen::VectorXf::Zero(state_num);
	Eigen::MatrixXf similarity_matrix(data_num,state_num);
	for(size_t i=0; i<data_num; i++)
	{
		input_vector = input_data.row(i);
		for(size_t j = 0; j <state_num ; j++) // state_num
		{
			model_matrix = m_model_matrix_VD[j];
			for(size_t k = 0; k <model_matrix.rows() ; k++)
			{
				vector_distance = (model_matrix.row(k)-input_vector).norm();
				similarity_vector[j] += 1 / vector_distance; 
			}
		}
		similarity_matrix.row(i) = similarity_vector;
		similarity_vector = Eigen::VectorXf::Zero(state_num);
	}


	//2) 임계값 계산을 위한 유사도min, 유사도max 계산과정
	std::vector<double> min_sim(state_num, 0.0); // 유사도 min : 각 접촉 구간에서의 유사도 최소값
	std::vector<double> max_sim(state_num, 0.0); // 유사도 max : 접촉 이외 구간에서의 유사도 값들의 평균
	std::vector<double> sum_sim_non_contact(state_num, 0.0); // 비접촉구간
	for(size_t i=0; i<data_num; i++)
	{
		for(size_t j=0; j<state_num; j++)
		{
			if(i==contact_index[j][0]-1+margin) {min_sim[j] = similarity_vector(i,j);} // initial value
			if(i>=contact_index[j][0]-1+margin && i<contact_index[j][1]-margin)
			{
				if(similarity_matrix(i,j)<min_sim[j]) min_sim[j] = similarity_matrix(i,j);
			}
			else
			{
				sum_sim_non_contact[j] += similarity_matrix(i,j);
			}
		}
	}

	// 최종 임계값 계산
	m_similarity_threshold.clear();
	m_similarity_threshold.resize(state_num);
	// 접촉 이외 구간의 유사도를 모두 더한후 평균 (접촉구간 앞뒤로 마진구간 때문에 최대값이 아닌 평균을 취함)
	// 앞뒤 마진 뺀 경우 - 2*iMargin
	printf("<유사도 임계값 계산 결과>\n");
	for(size_t i=0; i<state_num; i++)
	{
		max_sim[i] = sum_sim_non_contact[i] / (data_num-(contact_index[i][1]-contact_index[i][0]+2*margin)); // 데이터 개수 - 접촉구간 개수 ,앞뒤로 8개씩 빼줌
		m_similarity_threshold[i] = ratio*(min_sim[i]+max_sim[i]);
		// printf("AS_%i 임계값: %f\n", i,  m_similarity_threshold[i]);
	}
}

/** @brief Korean: 바타차야 거리(Bhattacharyya distance) 기반의 상관관계 분석을 수행한다.
 * @details 예시) 접촉상태가 1개인 경우에는 수행하지 않고, 2개 이상인 경우에 수행, 오직 자세 정보에 대해서만 적용
 * @param[in] measured_data : 접촉상태별로 수집한 측정 데이터 (6차원: F/T 데이터, 12차원: F/T 데이터 + 로봇 자세)
 */
void CContactStateEstimator::checkCorrelationUsingBhattacharyya(std::vector<std::vector<Eigen::MatrixXf>> &measured_data)
{
	size_t state_num = measured_data.size();
	size_t data_num = measured_data[0][measured_data.size()-1].rows(); // data 차원
	size_t data_dim = measured_data[0][measured_data.size()-1].cols(); // data 차원
	m_model_weight.clear();
	m_model_weight.resize(state_num);
	for(int k=0; k<state_num; k++)	{ m_model_weight[k] = Eigen::VectorXf::Ones(data_dim); }
	if(state_num==1)
	{
		for(size_t i=0; i<data_dim; i++) { m_model_weight[0][i] = 1.0; }
		return;
	}
	std::vector<Eigen::VectorXf> mean_list(state_num);
	std::vector<Eigen::VectorXf> std_list(state_num);
	for(size_t i=0; i<state_num; i++)
	{
		calMeanStd(measured_data[i][measured_data[i].size()-1], mean_list[i], std_list[i]);
	}

	//// combination
	std::vector<int> comb_in(state_num);
	for(size_t i=0; i<state_num; i++) { comb_in[i] = i; }
	std::vector<std::vector<int>> comb_set = m_math.combnk(comb_in,2);
	Eigen::VectorXf mean_a, std_a, mean_b, std_b; 
	std::vector<Eigen::VectorXf> distance_list(comb_set.size());

	for(size_t i=0; i<comb_set.size(); i++)
	{
		mean_a = mean_list[comb_set[i][0]];
		std_a = std_list[comb_set[i][0]];
		mean_b = mean_list[comb_set[i][1]];
		std_b = std_list[comb_set[i][1]];
		distance_list[i] = calBhattacharyyaDistance(mean_a, std_a, mean_b, std_b);
	}

	std::vector<std::vector<bool>> is_correlated(comb_set.size());
	bool b_correlation = false;
	for(size_t i=0; i<comb_set.size(); i++)
	{
		b_correlation = false;
		std::vector<bool> tmp(12, false);
		is_correlated[i] = tmp;
		for(int j=0; j<data_dim; j++) 
		{
			if(j<6) // F/T 데이터는 1
			{
				is_correlated[i][j] = false;
				continue;
			}

			// pose 데이터만
			if(distance_list[i][j] < 100.0)  // 모든 경우에 대해 true이면 0
			{
				b_correlation = true; 
			}
			else 
			{
				b_correlation = false;
			}

			// update
			if(b_correlation) {is_correlated[i][j] = true;}
			else {is_correlated[i][j] = false;}
		}
	}

	for(size_t k=0; k<state_num; k++)
	{
		for(size_t i=0; i<comb_set.size(); i++)
		{
			if(k!=comb_set[i][0] && k!=comb_set[i][1]) { continue; }
			for(int j=0; j<data_dim; j++) 
			{
				if(is_correlated[i][j]) { m_model_weight[k][j] = 0.0; } // 다른 상태와 연관된 차원은 0.0으로 설정
			}
		}
		std::cout << m_model_weight[k] << std::endl << std::endl;
	}
	printf("<바타차야거리를 통한 가중치 계산 결과>\n");
}

/** @brief Korean: 입력된 데이터의 평균과 표준편차를 산출한다.
 * @param[in] data : 입력 데이터
 * @param[out] mean : 평균
 * @param[out] std : 표준편차
 */
void CContactStateEstimator::calMeanStd(const Eigen::MatrixXf &data, Eigen::VectorXf& mean, Eigen::VectorXf& std)
{
		//// 바타차야 거리 공식을 이용한 가중치 계산
		printf("\n< 바타차야 거리 공식을 이용한 가중치 계산 >\n");
		double powerOfBhatta, varianceOfBhatta = 0.0;
		Eigen::MatrixXf x_of_Bhattacharyya = data;
		// 바타차야 거리 계산을 위한 변수
		size_t data_num = data.rows();
		size_t data_dim = data.cols();
		Eigen::VectorXf mean_of_Bhattacharyya(data_dim);
		Eigen::VectorXf std_of_Bhattacharyya(data_dim);
		Eigen::VectorXf vec_in;
		Eigen::VectorXf unit_vec = Eigen::VectorXf::Ones(data_num);
		mean_of_Bhattacharyya = x_of_Bhattacharyya.colwise().mean();
		for(size_t i=0; i<data_dim; i++) // FT + pose
		{ 
			vec_in = x_of_Bhattacharyya.col(i); // std
			vec_in = vec_in - vec_in.mean()*unit_vec;
			std_of_Bhattacharyya[i] = sqrt(vec_in.cwiseSqrt().sum()/(vec_in.size()-1));
		}
		mean = mean_of_Bhattacharyya;
		std = std_of_Bhattacharyya;
}

/** @brief Korean: 바타차야 거리(Bhattacharyya distance)를 산출한다.
 * @param[in] mean_a : 데이터 a의 평균
 * @param[in] std_a : 데이터 a의 표준편차
 * @param[in] mean_b : 데이터 b의 평균
 * @param[in] std_b : 데이터 b의 표준편차
 * @return 성분별 바타차야 거리
 */
Eigen::VectorXf CContactStateEstimator::calBhattacharyyaDistance(const Eigen::VectorXf& mean_a, const Eigen::VectorXf& std_a, const Eigen::VectorXf& mean_b, const Eigen::VectorXf& std_b)
{
		Eigen::VectorXf BhattacharyyaDistance(12); // 0: (AS1,AS2), 1: (AS2,AS3), 2: (AS1,AS3)

		// 바타차야 거리 공식
		for(int j=0; j<12; j++) 
		{
			BhattacharyyaDistance[j] = 0.25*log(0.25*(powf(std_a[j], 2)/powf(std_b[j], 2)+
				powf(std_b[j], 2)/powf(std_a[j], 2)+2.0))+
				0.25*(powf(mean_a[j]-mean_b[j], 2)/(powf(std_a[j],2)+powf(std_b[j],2)));
		}
		return BhattacharyyaDistance;
}

/** @brief Korean: 조립상태 데이터 샘플을 이용하여 유사도 판별 알고리즘의 가중치를 산출한다.
 * @details F/T 데이터 항에 대한 가중치(0~1 사이값)를 산출하며, pose 데이터의 경우 바타차야 거리 기반으로 산출된 가중치(0 또는 1의 값)를 사용한다.
 * @param[in] measured_data : 접촉상태별로 수집한 측정 데이터 (6차원: F/T 데이터, 12차원: F/T 데이터 + 로봇 자세)
 * @param[in] min_val_list : 유사도의 최소값 리스트
 * @param[in] max_val_list : 유사도의 최대값 리스트
 * @param[in] sampling_start : 데이터 샘플링 시작점(제어주기 count)
 */
void CContactStateEstimator::calWeight(std::vector<std::vector<Eigen::MatrixXf>> &measured_data, std::vector<Eigen::VectorXf> &min_val_list, std::vector<Eigen::VectorXf> &max_val_list, size_t sampling_start)
{
	size_t state_num = measured_data.size();
	size_t last_idx = measured_data.size()-1;
	size_t data_num = measured_data[0][last_idx].rows(); // data 차원
	size_t data_dim = measured_data[0][last_idx].cols(); // data 차원
	Eigen::MatrixXf input_data;

	double dMagParameter = 1.0;
	double dWtThreshold = 1000.0;
	double dTmpValue = 0.0;
	double dWeight[6] = {0.0};	

	Eigen::MatrixXf diff_matrix(state_num, data_dim);
	for(int k=0; k<state_num; k++)	
	{
		Eigen::VectorXf diff_val = max_val_list[k]-min_val_list[k];
		diff_matrix.row(k) = diff_val;
	}
	Eigen::VectorXf diff_val_max = diff_matrix.rowwise().maxCoeff();

	for(int k=0; k<state_num; k++)	
	{ 
		input_data = measured_data[k][last_idx];
		Eigen::VectorXf dMeasuredForWt(12);
		Eigen::VectorXf dDataForWt_Normal(12);
		for(int i=0; i<data_dim; i++) { dMeasuredForWt[i] = input_data(sampling_start,i); }

		// weight Initialize
		for(int i=0; i<6; i++) // only F/T
		{
			for(int j=0; j<6; j++) // only F/T
			{
				if(i==j) dWeight[j] = 1.0;
				else dWeight[j] = 0.0;

				if(i==j) dTmpValue = diff_val_max[i];
				else dTmpValue = 0.0;
				dDataForWt_Normal[j] = ((dMeasuredForWt[j]+dTmpValue*dMagParameter)-m_model_normalize_min_list[j])/(m_model_normalize_max_list[j]-m_model_normalize_min_list[j]);
			}

			// Similarity calculation for weight
			double vector_distance = 0.0;
			Eigen::VectorXf similarity_vector = Eigen::VectorXf::Zero(state_num);
			Eigen::VectorXf input_vector;
			for(size_t j = 0; j<state_num; j++) // state_num
			{
				for(size_t k = 0; k <m_model_matrix_VD[j].rows() ; k++)
				{
					input_vector = m_model_matrix_VD[j].row(k)-dDataForWt_Normal;
					input_vector = m_model_weight[j].cwiseSqrt()*input_vector; // m_model_weight는 각 항의 제곱에 곱해지는 계수
					vector_distance = input_vector.norm();
					similarity_vector[j] += 1 / vector_distance; 
				}
			}

			for(int j = 0; j < 6; j++)
			{
				if(i==j) m_model_weight[k][j] = similarity_vector[j]/dWtThreshold;
			}
			printf("\nWeight\n");
			for(int j = 0; j < 6; j++) printf("%f ", m_model_weight[k][j]);
			printf("\n");
		}
	}
}

/** @brief Korean: 조립상태 추정 알고리즘을 수행한다.
 */
// void CContactStateEstimator::estimateContactState()
// {
// 	// 상태추정알고리즘 - Ver.2 : 유사도지수 기반
// 	// 유사도 지수 실시간 평가(20ms)
// 		// 실패상태 Flag 초기화 180813
// 		m_AssemblyDenso.m_bFlagPointFSE = false;

// 		// 상태추정 2차, 임계값 3차, m_dSimilarityThreshold[]
// 		// Estimate the contact state, 여기가 판별 기준, 임계값 정의할 때 추가

// 		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 		// 1. Point contact 

// 		if(m_AssemblyDenso.GetPointContact()) // 점접촉구간에서
// 		{			
// 			printf("%f %f %f \n", m_dSimilarity_VD[0], m_dSimilarity_VD[1], m_dSimilarity_VD[2]);

// 			if((m_dSimilarity_VD[1] < m_dSimilarityThreshold[1]) && (m_dSimilarity_VD[2] < m_dSimilarityThreshold[2]))
// 			{
// 				if(m_dSimilarity_VD[0] > m_dSimilarityThreshold[0])
// 				{				
// 					m_bIsPointSuccess = true;
// 					CString str1;
// 					str1.Format("Point Contact");
// 					GetDlgItem(IDC_AS1)->SetWindowText(str1);

// 					m_iCountPointFail = 0;
// 					printf("\n");
// 					printf("Ver.2 : 점접촉 추정 성공 / ");

// 				}

// 				else // 점접촉 실패의 경우
// 				{
// 					m_iCountPointFail ++;

// 					if(m_iCountPointFail >= m_iMonitoringCount) // m_iMonitoringCount 변수 조정하여 모니터링 카운트 조절, 181002 
// 					{
// 						m_bIsPointSuccess = false; // 전체 조립 성공 판별 용
// 						//m_AssemblyDenso.m_bFlagPointFSE = true; // 실패상태판별 수행 여부 Flag, 180813
// 						CString str1;
// 						str1.Format("점접촉 실패");
// 						GetDlgItem(IDC_AS1)->SetWindowText(str1);

// 						printf("Ver.2 : 점접촉 추정 실패 / ");


// 					}

// 				}
// 			}

// 			else// 접접촉 구간에서 나머지도 높은경우
// 			{
// 				if(m_dSimilarity_VD[0] > m_dSimilarityThreshold[0])
// 				{
// 					CString str1;
// 					str1.Format("추정 실패");
// 					GetDlgItem(IDC_AS1)->SetWindowText(str1);

// 					if((m_dSimilarity_VD[1] > m_dSimilarityThreshold[1]) && (m_dSimilarity_VD[2] > m_dSimilarityThreshold[2]))
// 					{
// 						cout << "점접촉은 성공했으나, 점접촉 구간을 점접촉, 선접촉, 면접촉이라 동시 추정" << endl;
// 					}
// 					else if(m_dSimilarity_VD[1] > m_dSimilarityThreshold[1])
// 					{
// 						cout << "점접촉은 성공했으나, 점접촉 구간을 점접촉, 선접촉이라 동시 추정" << endl;
// 					}
// 					else if(m_dSimilarity_VD[2] > m_dSimilarityThreshold[2])
// 					{
// 						cout << "점접촉은 성공했으나, 점접촉 구간을 점접촉, 면접촉이라 동시 추정" << endl;
// 					}
// 					else
// 					{

// 					}

// 				}
// 				else // 접접촉 구간에서 접접촉은 낮은데, 다른게 높은경우(다른 상태로 추정된 경우)
// 				{
// 					m_iCountPointFail ++;

// 					if(m_iCountPointFail >= m_iMonitoringCount)
// 					{
// 						m_bIsPointSuccess = false; // 전체 조립 성공 판별 용

// 						CString str1;
// 						str1.Format("점접촉 실패");
// 						GetDlgItem(IDC_AS1)->SetWindowText(str1);
// 						printf("Ver.2 : 점접촉 추정 실패 / ");

// 						if((m_dSimilarity_VD[1] > m_dSimilarityThreshold[1]) && (m_dSimilarity_VD[2] > m_dSimilarityThreshold[2]))
// 						{
// 							cout << "점접촉은 실패했고, 점접촉 구간을 선접촉, 면접촉이라 동시 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[1] > m_dSimilarityThreshold[1])
// 						{
// 							cout << "점접촉은 실패했고, 점접촉 구간을 선접촉이라 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[2] > m_dSimilarityThreshold[2])
// 						{
// 							cout << "점접촉은 실패했고, 점접촉 구간을 면접촉이라 추정" << endl;
// 						}
// 						else
// 						{

// 						}

// 					}

// 					if(!m_bIsPointSuccess)
// 					{
// 						if((m_dSimilarity_VD[1] > m_dSimilarityThreshold[1]) && (m_dSimilarity_VD[2] > m_dSimilarityThreshold[2]))
// 						{
// 							cout << "점접촉은 실패했고, 점접촉 구간을 선접촉, 면접촉이라 동시 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[1] > m_dSimilarityThreshold[1])
// 						{
// 							cout << "점접촉은 실패했고, 점접촉 구간을 선접촉이라 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[2] > m_dSimilarityThreshold[2])
// 						{
// 							cout << "점접촉은 실패했고, 점접촉 구간을 면접촉이라 추정" << endl;
// 						}
// 						else
// 						{

// 						}
// 					}
// 					else // 3개 연속해야 점접촉 성공으로 판별하므로, 1,2개는 성공으로 간주
// 					{
// 						if((m_dSimilarity_VD[1] > m_dSimilarityThreshold[1]) && (m_dSimilarity_VD[2] > m_dSimilarityThreshold[2]))
// 						{
// 							cout << "점접촉은 성공했으나 점접촉 구간을 점접촉, 선접촉, 면접촉이라 동시 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[1] > m_dSimilarityThreshold[1])
// 						{
// 							cout << "점접촉은 성공했으나 점접촉 구간을 선접촉이라 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[2] > m_dSimilarityThreshold[2])
// 						{
// 							cout << "점접촉은 성공했으나, 점접촉 구간을 면접촉이라 추정" << endl;
// 						}
// 						else
// 						{

// 						}
// 					}

// 				}// 접접촉 구간에서 접접촉은 낮은데, 다른게 높은경우 끝

// 			}// 접접촉 구간에서 나머지도 높은경우 끝

// 		}// 점접촉구간 끝


// 		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 		// 2. Line contact 

// 		if(m_AssemblyDenso.GetLineContact()) // 선접촉구간에서
// 		{			
// 			if((m_dSimilarity_VD[0] < m_dSimilarityThreshold[0]) && (m_dSimilarity_VD[2] < m_dSimilarityThreshold[2]))
// 			{
// 				if(m_dSimilarity_VD[1] > m_dSimilarityThreshold[1])
// 				{				
// 					m_bIsLineSuccess = true;
// 					CString str2;
// 					str2.Format("Line Contact");
// 					GetDlgItem(IDC_AS2)->SetWindowText(str2);

// 					m_iCountLineFail = 0;
// 					printf("\n");
// 					printf("Ver.2 : 선접촉 추정 성공 / ");

// 				}

// 				else // 선접촉 실패의 경우
// 				{
// 					m_iCountLineFail ++;

// 					if(m_iCountLineFail >= m_iMonitoringCount)
// 					{
// 						m_bIsLineSuccess = false; // 전체 조립 성공 판별 용
// 						//m_AssemblyDenso.m_bFlagPointFSE = true; // 실패상태판별 수행 여부 Flag, 180813

// 						CString str2;
// 						str2.Format("선접촉 실패");
// 						GetDlgItem(IDC_AS2)->SetWindowText(str2);

// 						printf("Ver.2 : 선접촉 추정 실패 / ");
// 					}

// 				}
// 			}

// 			else// 선접촉 구간에서 나머지도 높은경우
// 			{
// 				if(m_dSimilarity_VD[1] > m_dSimilarityThreshold[1])
// 				{
// 					CString str2;
// 					str2.Format("추정 실패");
// 					GetDlgItem(IDC_AS2)->SetWindowText(str2);

// 					if((m_dSimilarity_VD[0] > m_dSimilarityThreshold[0]) && (m_dSimilarity_VD[2] > m_dSimilarityThreshold[2]))
// 					{
// 						cout << "선접촉은 성공했으나, 선접촉 구간을 점접촉, 선접촉, 면접촉이라 동시 추정" << endl;
// 					}
// 					else if(m_dSimilarity_VD[2] > m_dSimilarityThreshold[2])
// 					{
// 						cout << "선접촉은 성공했으나, 선접촉 구간을 선접촉, 면접촉이라 동시 추정" << endl;
// 					}
// 					else if(m_dSimilarity_VD[0] > m_dSimilarityThreshold[0])
// 					{
// 						cout << "선접촉은 성공했으나, 선접촉 구간을 선접촉, 점접촉이라 동시 추정" << endl;
// 					}
// 					else
// 					{

// 					}

// 				}
// 				else // 선접촉 구간에서 선접촉은 낮은데, 다른게 높은경우
// 				{
// 					m_iCountLineFail ++;

// 					if(m_iCountLineFail >= m_iMonitoringCount) 
// 					{
// 						m_bIsLineSuccess = false; // 전체 조립 성공 판별 용

// 						CString str2;
// 						str2.Format("선접촉 실패");
// 						GetDlgItem(IDC_AS2)->SetWindowText(str2);
// 						printf("Ver.2 : 선접촉 추정 실패 / ");

// 						if((m_dSimilarity_VD[0] > m_dSimilarityThreshold[0]) && (m_dSimilarity_VD[2] > m_dSimilarityThreshold[2]))
// 						{
// 							cout << "선접촉은 실패했고, 선접촉 구간을 점접촉, 면접촉이라 동시 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[2] > m_dSimilarityThreshold[2])
// 						{
// 							cout << "선접촉은 실패했고, 선접촉 구간을 면접촉이라 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[0] > m_dSimilarityThreshold[0])
// 						{
// 							cout << "선접촉은 실패했고, 선접촉 구간을 점접촉이라 추정" << endl;
// 						}
// 						else
// 						{

// 						}
// 					}

// 					if(!m_bIsLineSuccess)
// 					{
// 						if((m_dSimilarity_VD[0] > m_dSimilarityThreshold[0]) && (m_dSimilarity_VD[2] > m_dSimilarityThreshold[2]))
// 						{
// 							cout << "선접촉은 실패했고, 선접촉 구간을 점접촉, 면접촉이라 동시 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[2] > m_dSimilarityThreshold[2])
// 						{
// 							cout << "선접촉은 실패했고, 선접촉 구간을 면접촉이라 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[0] > m_dSimilarityThreshold[0])
// 						{
// 							cout << "선접촉은 실패했고, 선접촉 구간을 점접촉이라 추정" << endl;
// 						}
// 						else
// 						{

// 						}

// 					}

// 					else
// 					{
// 						if((m_dSimilarity_VD[0] > m_dSimilarityThreshold[0]) && (m_dSimilarity_VD[2] > m_dSimilarityThreshold[2]))
// 						{
// 							cout << "선접촉은 성공했으나 선접촉 구간을 점접촉, 선접촉, 면접촉이라 동시 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[2] > m_dSimilarityThreshold[2])
// 						{
// 							cout << "선접촉은 성공했으나 선접촉 구간을 면접촉이라 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[0] > m_dSimilarityThreshold[0])
// 						{
// 							cout << "선접촉은 성공했으나 선접촉 구간을 점접촉이라 추정" << endl;
// 						}
// 						else
// 						{

// 						}
// 					}

// 				}// 선접촉 구간에서 선접촉은 낮은데, 다른게 높은경우 끝

// 			}// 선접촉 구간에서 나머지도 높은경우 끝

// 		}// 선접촉구간 끝

// 		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 		// 3. Surface contact 

// 		if(m_AssemblyDenso.GetSurfaceContact()) // 면접촉구간에서
// 		{			
// 			if((m_dSimilarity_VD[0] < m_dSimilarityThreshold[0]) && (m_dSimilarity_VD[1] < m_dSimilarityThreshold[1]))
// 			{
// 				if(m_dSimilarity_VD[2] > m_dSimilarityThreshold[2])
// 				{				
// 					m_bIsSurfaceSuccess = true;
// 					CString str3;
// 					str3.Format("Surface Contact");
// 					GetDlgItem(IDC_AS3)->SetWindowText(str3);

// 					m_iCountSurfaceFail = 0;
// 					printf("\n");
// 					printf("Ver.2 : 면접촉 추정 성공 / ");

// 				}

// 				else // 면접촉 실패의 경우
// 				{
// 					m_iCountSurfaceFail ++;

// 					if(m_iCountSurfaceFail >= m_iMonitoringCount) 
// 					{
// 						m_bIsSurfaceSuccess = false; // 전체 조립 성공 판별 용
// 						//m_AssemblyDenso.m_bFlagPointFSE = true; // 실패상태판별 수행 여부 Flag, 180813

// 						CString str3;
// 						str3.Format("면접촉 실패");
// 						GetDlgItem(IDC_AS3)->SetWindowText(str3);

// 						printf("Ver.2 : 면접촉 추정 실패 / ");
// 					}

// 				}
// 			}

// 			else// 면접촉 구간에서 나머지도 높은경우
// 			{
// 				if(m_dSimilarity_VD[2] > m_dSimilarityThreshold[2])
// 				{
// 					CString str3;
// 					str3.Format("추정 실패");
// 					GetDlgItem(IDC_AS3)->SetWindowText(str3);

// 					if((m_dSimilarity_VD[0] > m_dSimilarityThreshold[0]) && (m_dSimilarity_VD[1] > m_dSimilarityThreshold[1]))
// 					{
// 						cout << "면접촉은 성공했으나, 면접촉 구간을 면접촉, 선접촉, 점접촉이라 동시 추정" << endl;
// 					}
// 					else if(m_dSimilarity_VD[0] > m_dSimilarityThreshold[0])
// 					{
// 						cout << "면접촉은 성공했으나, 면접촉 구간을 면접촉, 점접촉이라 동시 추정" << endl;
// 					}
// 					else if(m_dSimilarity_VD[1] > m_dSimilarityThreshold[1])
// 					{
// 						cout << "면접촉은 성공했으나, 면접촉 구간을 면접촉, 선접촉이라 동시 추정" << endl;
// 					}
// 					else
// 					{

// 					}

// 				}
// 				else // 면접촉 구간에서 선접촉은 낮은데, 다른게 높은경우
// 				{
// 					m_iCountSurfaceFail ++;

// 					if(m_iCountSurfaceFail >= m_iMonitoringCount)
// 					{
// 						m_bIsSurfaceSuccess = false; // 전체 조립 성공 판별 용

// 						CString str3;
// 						str3.Format("면접촉 실패");
// 						GetDlgItem(IDC_AS3)->SetWindowText(str3);
// 						printf("Ver.2 : 면접촉 추정 실패 / ");

// 						if((m_dSimilarity_VD[0] > m_dSimilarityThreshold[0]) && (m_dSimilarity_VD[1] > m_dSimilarityThreshold[1]))
// 						{
// 							cout << "면접촉은 실패했고, 면접촉 구간을 점접촉, 선접촉이라 동시 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[0] > m_dSimilarityThreshold[0])
// 						{
// 							cout << "면접촉은 실패했고, 면접촉 구간을 점접촉이라 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[1] > m_dSimilarityThreshold[1])
// 						{
// 							cout << "면접촉은 실패했고, 면접촉 구간을 선접촉이라 추정" << endl;
// 						}
// 						else
// 						{

// 						}

// 					}

// 					if(!m_bIsSurfaceSuccess)
// 					{
// 						if((m_dSimilarity_VD[0] > m_dSimilarityThreshold[0]) && (m_dSimilarity_VD[2] > m_dSimilarityThreshold[2]))
// 						{
// 							cout << "면접촉은 실패했고, 면접촉 구간을 점접촉, 선접촉이라 동시 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[0] > m_dSimilarityThreshold[0])
// 						{
// 							cout << "면접촉은 실패했고, 면접촉 구간을 점접촉이라 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[1] > m_dSimilarityThreshold[1])
// 						{
// 							cout << "면접촉은 실패했고, 면접촉 구간을 선접촉이라 추정" << endl;
// 						}
// 						else
// 						{

// 						}
// 					}

// 					else
// 					{
// 						if((m_dSimilarity_VD[0] > m_dSimilarityThreshold[0]) && (m_dSimilarity_VD[2] > m_dSimilarityThreshold[2]))
// 						{
// 							cout << "면접촉은 성공했으나 면접촉 구간을 점접촉, 선접촉, 면접촉이라 동시 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[0] > m_dSimilarityThreshold[0])
// 						{
// 							cout << "면접촉은 성공했으나 면접촉 구간을 점접촉이라 추정" << endl;
// 						}
// 						else if(m_dSimilarity_VD[1] > m_dSimilarityThreshold[1])
// 						{
// 							cout << "면접촉은 성공했으나 면접촉 구간을 선접촉이라 추정" << endl;
// 						}
// 						else
// 						{

// 						}
// 					}

// 				}// 면접촉 구간에서 면접촉은 낮은데, 다른게 높은경우 끝

// 			}// 면접촉 구간에서 나머지도 높은경우 끝

// 		}// 면접촉구간 끝


// 		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 		// 4. 비접촉구간
// 		//if(!m_AssemblyDenso.GetSurfaceContact() && !m_AssemblyDenso.GetSurfaceContact() && !m_AssemblyDenso.GetSurfaceContact())
// 		//{
// 		//	// 비접촉구간에서 다 낮으면
// 		//	if((m_dSimilarity_VD[0] < m_dSimilarityThreshold[0]) && (m_dSimilarity_VD[1] < m_dSimilarityThreshold[1]) && (m_dSimilarity_VD[2] < m_dSimilarityThreshold[2]))
// 		//	{
// 		//		CString str1;
// 		//		str1.Format("-");
// 		//		GetDlgItem(IDC_AS1)->SetWindowText(str1);

// 		//		CString str2;
// 		//		str2.Format("-");
// 		//		GetDlgItem(IDC_AS2)->SetWindowText(str2);

// 		//		CString str3;
// 		//		str3.Format("-");
// 		//		GetDlgItem(IDC_AS3)->SetWindowText(str3);
// 		//	}
// 		//	// 어느 경우라도 높은게 있으면
// 		//	else  
// 		//	{
// 		//		if(m_dSimilarity_VD[0] > m_dSimilarityThreshold[0])
// 		//		{
// 		//			CString str1;
// 		//			str1.Format("비접촉구간 추정");
// 		//			GetDlgItem(IDC_AS1)->SetWindowText(str1);
// 		//			cout << "비접촉구간에서 점접촉이라 추정" << endl;
// 		//		}

// 		//		if(m_dSimilarity_VD[1] > m_dSimilarityThreshold[1])
// 		//		{
// 		//			CString str2;
// 		//			str2.Format("비접촉구간 추정");
// 		//			GetDlgItem(IDC_AS2)->SetWindowText(str2);
// 		//			cout << "비접촉구간에서 선접촉이라 추정" << endl;
// 		//		}

// 		//		if(m_dSimilarity_VD[2] > m_dSimilarityThreshold[2])
// 		//		{
// 		//			CString str3;
// 		//			str3.Format("비접촉구간 추정");
// 		//			GetDlgItem(IDC_AS3)->SetWindowText(str3);
// 		//			cout << "비접촉구간에서 면접촉이라 추정" << endl;
// 		//		}
// 		//	}// 어느 경우라도 높은 경우 끝

// 		//}// 비접촉 구간에서 다 낮은 경우 끝




// 		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 		// 5. 조립모델 재생성 판정 부분, 181002
// 		if(m_AssemblyDenso.GetPointContact()) // A1
// 		{		

// 			//if(m_dSimilarity_VD[0] > m_dRatioOfReModel*(m_dSimilarityThreshold[0]/m_dRatioOfSimilarityThreshold)) {m_iCountA1ReModel = 0;}
// 			if(m_dSimilarity_VD[0] > 300) {m_iCountA1ReModel = 0;}
// 			else 
// 			{
// 				m_iCountA1ReModel ++;
// 				if(m_iCountA1ReModel >= m_iMonitoringCount+10) // m_iMonitoringCount 변수 조정하여 모니터링 카운트 조절, 181002 
// 				{
// 					m_bCheckReModelA1 = true; //A1 조립모델 재생성 판별 용
// 				}
// 			}
// 		}

// 		if(m_AssemblyDenso.GetLineContact()) // A2
// 		{		

// 			//if(m_dSimilarity_VD[1] > m_dRatioOfReModel*(m_dSimilarityThreshold[1]/m_dRatioOfSimilarityThreshold)) {m_iCountA2ReModel = 0;}
// 			if(m_dSimilarity_VD[1] > 300) {m_iCountA2ReModel = 0;}
// 			else 
// 			{
// 				m_iCountA2ReModel ++;
// 				if(m_iCountA2ReModel >= m_iMonitoringCount+10) // m_iMonitoringCount 변수 조정하여 모니터링 카운트 조절, 181002 
// 				{
// 					m_bCheckReModelA2 = true; // A2 조립모델 재생성 판별 용
// 				}
// 			}
// 		}

// 		if(m_AssemblyDenso.GetSurfaceContact()) // A3
// 		{		

// 			//if(m_dSimilarity_VD[2] > m_dRatioOfReModel*(m_dSimilarityThreshold[2]/m_dRatioOfSimilarityThreshold)) {m_iCountA3ReModel = 0;}
// 			if(m_dSimilarity_VD[2] > 300) {m_iCountA3ReModel = 0;}
// 			else 
// 			{
// 				m_iCountA3ReModel ++;
// 				if(m_iCountA3ReModel >= m_iMonitoringCount+10) // m_iMonitoringCount 변수 조정하여 모니터링 카운트 조절, 181002 
// 				{
// 					m_bCheckReModelA3 = true; // A3 조립모델 재생성 판별 용
// 				}
// 			}
// 		}



// 		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 		// 6.최종 조립 성패 판별
// 		if(m_bIsPointSuccess && m_bIsLineSuccess && m_bIsSurfaceSuccess)
// 		{
// 			CString str4;
// 			str4.Format("조립 성공");
// 			GetDlgItem(IDC_ESTIMATED_STATE)->SetWindowText(str4);

// 		}
// 		else
// 		{
// 			CString str4;
// 			str4.Format("조립 실패");
// 			GetDlgItem(IDC_ESTIMATED_STATE)->SetWindowText(str4);

// 		}
// }

