/**
* @file optimazation_module.h
* @brief 최적화를 위한 헤더파일
*/
#ifdef max
#undef max
#undef min
#endif

#pragma once

//#include "stdafx.h"

#include <iostream>
#include <string>
#include <algorithm>
#include <stdio.h>

// #include <eigen3/Eigen/Eigen>
//// LP solver - lp_solve
#include "module/KUPCLMath.h"
#include "module/data_class/optimization_data.h"

#include "lpsolve/lp_lib.h"
#undef TIMEOUT

/**
* @class COPTIMODULE
* @brief 최적화 모듈 클래스
* @ingroup OPTIMIZATION
*/
class COPTIMODULE
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
	COPTIMODULE();
	~COPTIMODULE();

private:


public:
	KUPCLMATH m_math;

	Eigen::VectorXd calLowerBoundWeightUsingTanh(const Eigen::VectorXd &list_data, double min_threshold, double margin);
	Eigen::VectorXd calUpperBoundWeightUsingTanh(const Eigen::VectorXd &list_data, double max_threshold, double margin);
	int solveMLP(const Eigen::VectorXd &f, const Eigen::MatrixXd &A, const Eigen::VectorXd &b, Eigen::VectorXd &output, bool do_save_model);
	Eigen::VectorXd solveMLP2();
	int optimizationMILPv2(COPTCONDITIONLIST &opt_cond_list, std::vector<size_t> &optimal_idx);
	void optimizationWeightedMILPv2(COPTCONDITIONLIST &opt_cond_list, std::vector<size_t> &optimal_idx);

};