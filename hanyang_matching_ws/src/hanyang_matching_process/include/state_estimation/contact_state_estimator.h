/**
 * @file contact_state_estimator.h
 * @brief 접촉상태 판별을 위한 헤더파일
 */

#ifndef CONTACT_STATE_ESTIMATOR_H
#define CONTACT_STATE_ESTIMATOR_H

#include "define.h"
#include "control/RobotModel.hpp"
#include "module/KUPCLMath.h"

/**
 * @class CContactStateEstimator
 * @brief 접촉상태 판별 알고리즘 클래스
 * @ingroup STATE_ESTIMATION
 */
class CContactStateEstimator {
public:
    CContactStateEstimator(void);
    ~CContactStateEstimator(void);           

	KUPCLMATH m_math; 

    void CalSimilarityVD(double dRobotPose[], double dFTsensorData[]);
	Eigen::VectorXf CalSimilarityVD2(const std::vector<Eigen::MatrixXf> &model_matrix, const Eigen::VectorXf &vec_in, const Eigen::VectorXf &model_weight);
    void CalSimilaritySVD(double dRobotPose[], double dFTsensorData[]);
    void CalSimilarityMD(double dRobotPose[], double dFTsensorData[]);
	

	void GenContactStateModelMatrix(std::vector<std::vector<Eigen::MatrixXf>> &measured_data, size_t model_size);
	void calThreshold(std::vector<Eigen::VectorXf> &process_data, std::vector<std::vector<size_t>> &contact_index, double margin, double ratio);


    void GenContactStateModelMatrixMD(); // Only 힘/토크
    void GenContactStateModelMatrixMD2(); // 힘/토크 + 로봇 자세

    void estimateContactState();

private:
	void calMeanStd(const Eigen::MatrixXf &data, Eigen::VectorXf& mean, Eigen::VectorXf& std);
	Eigen::VectorXf calBhattacharyyaDistance(const Eigen::VectorXf& mean_a, const Eigen::VectorXf& std_a, const Eigen::VectorXf& mean_b, const Eigen::VectorXf& std_b);
	void calWeight(std::vector<std::vector<Eigen::MatrixXf>> &measured_data, std::vector<Eigen::VectorXf> &min_val_list, std::vector<Eigen::VectorXf> &max_val_list, size_t sampling_start);
	void checkCorrelationUsingBhattacharyya(std::vector<std::vector<Eigen::MatrixXf>> &measured_data);
	
	std::vector<Eigen::MatrixXf> m_model_matrix_VD;
	std::vector<Eigen::MatrixXf> m_model_matrix_U_SVD;
	std::vector<Eigen::MatrixXf> m_model_matrix_S_SVD;
	std::vector<Eigen::MatrixXf> m_model_matrix_MD;
	std::vector<Eigen::VectorXf> m_mean_vector_MD;
	Eigen::VectorXf m_model_normalize_min_list; // 실시간 정규화에 적용할 min
	Eigen::VectorXf m_model_normalize_max_list; // 실시간 정규화에 적용할 max
	std::vector<Eigen::VectorXf> m_model_weight; // 유사도 평가를 위한 가중치
	std::vector<double> m_similarity_threshold; // 유사도 평가를 위한 임계값

	std::vector<double> m_dSimilarity_VD; // SVD 기반 유사도
	std::vector<double> m_dSimilarity_SVD; // SVD 기반 유사도
	std::vector<double> m_dSimilarity_MD; // 마할라노비스 기반 유사도

	size_t m_state_num;
	size_t m_data_num;
	size_t m_data_dim;
	size_t m_model_size;
	size_t m_sampling_start;


	double m_dModelMatrix[300][12]; // 모델 행렬
	double m_dDataBhattacharyya[865][12];
	double x_of_BhattacharyyaAS[40][12];
	double m_dWeightSim[3][12]; // 바타차야 거리 계산에 의한 가중치


	double m_dNormalizeMaxOfTest[12]; // 실시간에 적용할 Max
	double m_dNormalizeMinOfTest[12]; // 실시간에 적용할 Min

	double m_dSimilarityThreshold[3]; // 유사도지수 임계값(맨 처음 1번 수행)

	int m_iContactCount;// 접촉구간 시작시간 계산 시에 사용
	bool m_bIsPointStart;
	bool m_bIsLineStart;	
	bool m_bIsSurfaceStart;
	bool m_bIsPointEnd;
	bool m_bIsLineEnd;
	bool m_bIsSurfaceEnd;
	double m_dPointStartTime;
	double m_dLineStartTime;
	double m_dSurfaceStartTime;
	double m_dPointEndTime;
	double m_dLineEndTime;
	double m_dSurfaceEndTime;

	int m_iMarginOfSimilarityThreshold;
	double m_dRatioOfSimilarityThreshold;
	bool isSetParameterOfThreshold;


	// 181002, ED, SVD, MD 
	double m_dMeanVector_MD[3][6];
	double m_dCov_A1[6][6];
	double m_dCov_A2[6][6];
	double m_dCov_A3[6][6];

	double m_dMeanVector_MD_FTPos[3][12];
	double m_dCov_A1_FTPos[12][12];
	double m_dCov_A2_FTPos[12][12];
	double m_dCov_A3_FTPos[12][12];
	// 181002, 조립모델 재생성 판별기준

	bool m_bCheckReModelA1; 
	bool m_bCheckReModelA2;
	bool m_bCheckReModelA3; 

	int m_iMonitoringCount;
	double m_dRatioOfReModel;

	int m_iCountA1ReModel;
	int m_iCountA2ReModel;
	int m_iCountA3ReModel;



	//181005 SVD
	double m_dU_A1[12][12];
	double m_dU_A2[12][12];
	double m_dU_A3[12][12];

	double m_dS_A1[12];
	double m_dS_A2[12];
	double m_dS_A3[12];

	void CalSimilaritySVD(double dFTsensorData[]);


	// 181121
	double m_dModelMatrix_MD[300][12]; // 모델 행렬    

};


#endif // PATHPLANNER_H
