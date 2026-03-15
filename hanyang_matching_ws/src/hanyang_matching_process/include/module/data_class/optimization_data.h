#pragma once

/**
* @class COPTCONDITIONLIST
* @brief 최적화를 위한 데이터 클래스
* @ingroup OPT_DATA_CLASS
*/
class COPTCONDITIONLIST
{
public:
	COPTCONDITIONLIST() {};
	~COPTCONDITIONLIST() {};

public:
    //// User input parameters
	size_t candidate_case;
    size_t candidate_num;
    size_t constraint_num;

	// quality threshold
    std::vector<double> threshold_set;
	// optimization parameters
    std::vector<double> constraint_set;

	size_t min_sol_num; // minimum # of solution
	size_t max_sol_num; // maximum # of solution

    std::vector<double> margin_set;

	double min_cutoff_value; // 0-1 cutoff minimum value
	std::vector<size_t> objective_case;
    std::vector<bool> is_objective_maximize;

	bool is_weight_applied;

    //// Condition list
    std::vector<std::vector<double>> list_condition_set;
    std::vector<bool> is_constraint_minimum_inequality;
};


