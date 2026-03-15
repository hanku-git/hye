/**
* @file optimization_module.cpp
* @brief 최적화를 위한 구현파일이다.
*/
#include "module/optimization_module.h"
#include <string>
#include <iostream>
#include <math.h>
#include "define.h"


COPTIMODULE::COPTIMODULE()
{

}

COPTIMODULE::~COPTIMODULE()
{

}

/** @brief Korean: 정수 선형 계획법의 하한(lower bound) 기준의 weight를 산출한다.
* @details -
*/
Eigen::VectorXd COPTIMODULE::calLowerBoundWeightUsingTanh(const Eigen::VectorXd &list_data, double min_threshold, double margin)
{
	double scale = 1.0 / margin;
	double x;
	double x_offset = min_threshold - margin / 2.0;
	double y_offset = 0.5;
	double k = 2.0*scale*M_PI;
	Eigen::VectorXd weight_list(list_data.size());
	for (size_t i = 0; i < weight_list.size(); i++)
	{
		x = list_data[i];
		weight_list[i] = 0.5*tanh(k*(x - x_offset)) + y_offset;
	}
	return weight_list;
}

/** @brief Korean: 정수 선형 계획법의 상한(upper bound) 기준의 weight를 산출한다.
* @details -
*/
Eigen::VectorXd COPTIMODULE::calUpperBoundWeightUsingTanh(const Eigen::VectorXd &list_data, double max_threshold, double margin)
{
	double scale = 1.0 / margin;
	double x;
	double x_offset = max_threshold + margin / 2.0;
	double y_offset = 0.5;
	double k = -2.0*scale*M_PI;
	Eigen::VectorXd weight_list(list_data.size());
	for (size_t i = 0; i < weight_list.size(); i++)
	{
		x = list_data[i];
		weight_list[i] = 0.5*tanh(k*(x - x_offset)) + y_offset;
	}
	return weight_list;
}

/** @brief Korean: Third party의 최적화 solver를 이용하여 MLP의 최적 해를 산출한다.
* @details -
*/
int COPTIMODULE::solveMLP(const Eigen::VectorXd &f, const Eigen::MatrixXd &A, const Eigen::VectorXd &b, Eigen::VectorXd &output, bool do_save_model)
{
	lprec *lp;
	int Ncol, *colno = NULL, ret = 0;
	REAL *row = NULL;

	/* We will build the model row by row
	So we start with creating a model with 0 rows and 2 columns */
	Ncol = f.size(); /* there are two variables in the model */
	Eigen::VectorXd opt_x(Ncol);
	lp = make_lp(0, Ncol);

	if (lp == NULL)
		ret = 1; /* couldn't construct a new model... */

	if (ret == 0) {
		/* let us name our variables. Not required, but can be useful for debugging */
		std::stringstream ss;
		for (size_t i = 0; i < Ncol; i++)
		{
			ss.str("");
			ss << "x" << i + 1;
			std::string str = ss.str();
			char* ch = strcpy(new char[str.length() + 1], str.c_str());
			set_col_name(lp, i + 1, ch);
			set_binary(lp, i + 1, TRUE); /* sets variable 1 to binary */
			// set_int(lp, i + 1, TRUE); /* sets variable 1 to integer */
			delete[] ch;
		}

		/* create space large enough for one row */
		colno = (int *)malloc(Ncol * sizeof(*colno));
		row = (REAL *)malloc(Ncol * sizeof(*row));
		if ((colno == NULL) || (row == NULL))
			ret = 2;
	}

	// Constraints A(i,:) <= b(i)
	if (ret == 0) {
		set_add_rowmode(lp, TRUE);  /* makes building the model faster if it is done rows by row */
		for (size_t i = 0; i < A.rows(); i++)
		{
			/* construct first row (120 x + 210 y <= 15000) */
			for (size_t j = 0; j < Ncol; j++)
			{
				colno[j] = j + 1; // the order of current column 
				row[j] = A(i, j);
			}

			/* add the row to lpsolve */
			if (!add_constraintex(lp, Ncol, row, colno, LE, b(i)))
				ret = 3;
		}
	}

	// Objective function
	if (ret == 0) {
		set_add_rowmode(lp, FALSE); /* rowmode should be turned off again when done building the model */
		/* set the objective function (143 x + 60 y) */
		for (size_t j = 0; j < Ncol; j++)
		{
			colno[j] = j + 1; // the order of current column 
			row[j] = f(j);
		}

		/* set the objective in lpsolve */
		if (!set_obj_fnex(lp, Ncol, row, colno))
			ret = 4;
	}

	if (ret == 0) {
		/* set the object direction to minimize */
		//set_maxim(lp);
		set_minim(lp);

		/* just out of curioucity, now show the model in lp format on screen */
		/* this only works if this is a console application. If not, use write_lp and a filename */
		std::string str = "/home/mando/Desktop/Untitled Folder/model.lp";
		std::vector<char> writable(str.begin(), str.end());
		writable.push_back('\0');
		char* ch = &writable[0];
		if (do_save_model) { write_lp(lp, ch); }

		/* I only want to see important messages on screen while solving */
		set_verbose(lp, IMPORTANT);

		/* Now let lpsolve calculate a solution */
		ret = solve(lp);
		if (ret == OPTIMAL)
			ret = 0;
		else
			ret = 5;
	}

	if (ret == 0) {
		/* a solution is calculated, now lets get some results */

		/* objective value */
		printf("Objective value: %f\n", get_objective(lp));

		/* variable values */
		get_variables(lp, row);
		for (size_t j = 0; j < Ncol; j++)
		{
			opt_x[j] = row[j];
			printf("%s: %f\n", get_col_name(lp, j + 1), row[j]);
		}

		/* we are done now */
	}

	/* free allocated memory */
	if (row != NULL)
		free(row);
	if (colno != NULL)
		free(colno);

	if (lp != NULL) {
		/* clean up such that all used memory by lpsolve is freed */
		delete_lp(lp);
	}
    output = opt_x;
	return ret;
}

/** @brief Korean: Third party의 최적화 solver를 이용하여 MLP의 최적 해를 산출한다.
* @details -
*/
Eigen::VectorXd COPTIMODULE::solveMLP2()
{
	Eigen::VectorXd x;

	lprec *lp;
	int Ncol, *colno = NULL, j, ret = 0;
	REAL *row = NULL;

	/* We will build the model row by row
	So we start with creating a model with 0 rows and 2 columns */
	Ncol = 2; /* there are two variables in the model */
	lp = make_lp(0, Ncol);
	if (lp == NULL)
		ret = 1; /* couldn't construct a new model... */

	if (ret == 0) {
		/* let us name our variables. Not required, but can be useful for debugging */
		//set_col_name(lp, 1, "x");
		//set_col_name(lp, 2, "y");
		std::stringstream ss;
		for (size_t i = 0; i < Ncol; i++)
		{
			ss.str("");
			ss << "x" << i + 1;
			//char* ch;
			//strcpy(ch, ss.str().c_str());
			std::string str = ss.str();
			char* ch = strcpy(new char[str.length() + 1], str.c_str());
			set_col_name(lp, i + 1, ch);
			delete[] ch;
		}

		/* create space large enough for one row */
		colno = (int *)malloc(Ncol * sizeof(*colno));
		row = (REAL *)malloc(Ncol * sizeof(*row));
		if ((colno == NULL) || (row == NULL))
			ret = 2;
	}

	if (ret == 0) {
		set_add_rowmode(lp, TRUE);  /* makes building the model faster if it is done rows by row */

		/* construct first row (120 x + 210 y <= 15000) */
		for (size_t i = 0; i < Ncol; i++)
		{
			if (i == 0)
			{
				colno[i] = i + 1; // the order of current column 
				row[i] = 120;
			}
			else
			{
				colno[i] = i + 1; // the order of current column 
				row[i] = 210;
			}
		}

		/* add the row to lpsolve */
		if (!add_constraintex(lp, Ncol, row, colno, LE, 15000))
			ret = 3;
	}

	if (ret == 0) {
		/* construct second row (110 x + 30 y <= 4000) */
		j = 0;

		colno[j] = 1; /* first column */
		row[j++] = 110;

		colno[j] = 2; /* second column */
		row[j++] = 30;

		/* add the row to lpsolve */
		if (!add_constraintex(lp, j, row, colno, LE, 4000))
			ret = 3;
	}

	if (ret == 0) {
		/* construct third row (x + y <= 75) */
		j = 0;

		colno[j] = 1; /* first column */
		row[j++] = 1;

		colno[j] = 2; /* second column */
		row[j++] = 1;

		/* add the row to lpsolve */
		if (!add_constraintex(lp, j, row, colno, LE, 75))
			ret = 3;
	}

	if (ret == 0) {
		set_add_rowmode(lp, FALSE); /* rowmode should be turned off again when done building the model */

		/* set the objective function (143 x + 60 y) */
		j = 0;

		colno[j] = 1; /* first column */
		row[j++] = -143;

		colno[j] = 2; /* second column */
		row[j++] = -60;

		/* set the objective in lpsolve */
		if (!set_obj_fnex(lp, j, row, colno))
			ret = 4;
	}

	if (ret == 0) {
		/* set the object direction to maximize */
		//set_maxim(lp);
		set_minim(lp);

		/* just out of curioucity, now show the model in lp format on screen */
		/* this only works if this is a console application. If not, use write_lp and a filename */
		//// write_LP(lp, stdout);
		//FILE *f;
		//CString str;
		//str.Format(_T("model.lp"));
		//f = fopen((CStringA)str, "w");
		//write_LP(lp, f);

		std::string str = "model.lp";
		std::vector<char> writable(str.begin(), str.end());
		writable.push_back('\0');
		char* ch = &writable[0];
		write_lp(lp, ch);

		/* I only want to see important messages on screen while solving */
		set_verbose(lp, IMPORTANT);

		/* Now let lpsolve calculate a solution */
		ret = solve(lp);
		if (ret == OPTIMAL)
			ret = 0;
		else
			ret = 5;
	}

	if (ret == 0) {
		/* a solution is calculated, now lets get some results */

		/* objective value */
		printf("Objective value: %f\n", get_objective(lp));

		/* variable values */
		get_variables(lp, row);
		for (j = 0; j < Ncol; j++)
			printf("%s: %f\n", get_col_name(lp, j + 1), row[j]);

		/* we are done now */
	}

	/* free allocated memory */
	if (row != NULL)
		free(row);
	if (colno != NULL)
		free(colno);

	if (lp != NULL) {
		/* clean up such that all used memory by lpsolve is freed */
		delete_lp(lp);
	}





	return x;
}

/** @brief Korean: 정수 선형 계획법(mixed-integer linear programming, MILP)을 이용하여 최적화를 수행한다.
* @details -
*/
int COPTIMODULE::optimizationMILPv2(COPTCONDITIONLIST &opt_cond_list, std::vector<size_t> &optimal_idx)
{
	size_t constraint_num = opt_cond_list.constraint_num;
	size_t n = opt_cond_list.candidate_num;
	Eigen::VectorXd one_vector = Eigen::VectorXd::Ones(n);
	Eigen::VectorXd zero_vector = Eigen::VectorXd::Zero(n);

    std::vector<Eigen::VectorXd> condition_set;
    for (size_t i = 0; i < constraint_num; i++)
    {
	    Eigen::VectorXd cond_tmp = m_math.vec2Eigen(opt_cond_list.list_condition_set[i]);
        if(1)
        {
            printf("*************************\n");
            printf("Constraint #%zu: ", i+1);
            for (size_t j = 0; j < cond_tmp.size(); j++) printf("%0.3f, ", cond_tmp[j]);
            printf("\n");
            printf("*************************\n");
        }
        condition_set.push_back(cond_tmp);
    }

	std::vector<size_t> idx_in = m_math.genSameWidthValueSet(1, n, 1); // 1:1:n

    //// constraints
	// 모든 부등식은 A*x <= b 형식이 기본이므로, A*x >= b일 경우, -A*x <= -b 의 형태로 변경하여 입력
	Eigen::MatrixXd A(constraint_num+2, n); // inspection conditions, [vq, le, pt, -1, 1]
    for (size_t i = 0; i < constraint_num; i++)
    {
        if(opt_cond_list.is_constraint_minimum_inequality[i]) A.row(i) = -condition_set[i]; // Minimum inequality (Ax > b)
        else A.row(i) = condition_set[i]; // Maximum inequality (Ax < b)
    }
	A.row(constraint_num) = -one_vector;
	A.row(constraint_num+1) = one_vector;
	// A.row(0) = -list_vq;
	// A.row(1) = list_le;
	// A.row(2) = list_pt;
	// A.row(3) = -one_vector;
	// A.row(4) = one_vector;

	Eigen::VectorXd b(constraint_num+2); 
    for (size_t i = 0; i < constraint_num; i++)
    {
        if(opt_cond_list.is_constraint_minimum_inequality[i]) b[i] = -opt_cond_list.constraint_set[i]; // Minimum inequality (Ax > b)
        else b[i] = opt_cond_list.constraint_set[i]; // Maximum inequality (Ax < b)
    }
    b[constraint_num] = -static_cast<double>(opt_cond_list.min_sol_num);
    b[constraint_num+1] = static_cast<double>(opt_cond_list.max_sol_num);
	// b[0] = -opt_cond_list.opt_setting_data.min_vq;
	// b[1] = opt_cond_list.opt_setting_data.max_le;
	// b[2] = opt_cond_list.opt_setting_data.max_pt;
	// b[3] = -opt_cond_list.opt_setting_data.min_sol_num;
	// b[4] = opt_cond_list.opt_setting_data.max_sol_num;

	//// objective function
	Eigen::VectorXd f = Eigen::VectorXd::Zero(n);
    size_t objective_variable_num = opt_cond_list.objective_case.size();
    for (size_t i = 0; i < objective_variable_num; i++)
    {
        size_t idx_now = opt_cond_list.objective_case[i];
        Eigen::VectorXd condition_now = condition_set[idx_now];
        if(opt_cond_list.is_objective_maximize[i])
        {
            f -= condition_now.cwiseProduct(one_vector);
        }
        else
        {
            f += condition_now.cwiseProduct(one_vector);
        }
    }
	// switch (opt_cond_list.objective_case)
	// {
	// case OBJECTIVE_MAXIMIZE_VIEW_QUALITY: //
	// 	f = -list_vq.cwiseProduct(one_vector);
	// 	break;
	// case OBJECTIVE_MINIMIZE_LIGHT_EFFECT: //
	// 	f = list_le.cwiseProduct(one_vector);
	// 	break;
	// case OBJECTIVE_MINIMIZE_PATH_TIME: //
	// 	f = list_pt.cwiseProduct(one_vector);
	// 	break;
	// }


	////////////////////
	//// Linear programming solver
	// Optimization solver (third party)
	// x1, ..., xn is binary
    Eigen::VectorXd x;
	int ret = solveMLP(f, A, b, x, true);
	////////////////////
	std::vector<size_t> idx_out;
	// optimal viewpoint index
	for (size_t i = 0; i < n; i++)
	{
		if (x[i]) // if x is not zero.
		{
			idx_out.push_back(i);
		}
	}
	optimal_idx = idx_out;
    return ret; // 0: optimal
}

/** @brief Korean: 가중치가 적용된 정수 선형 계획법(mixed-integer linear programming, MILP)을 이용하여 최적화를 수행한다.
* @details -
*/
void COPTIMODULE::optimizationWeightedMILPv2(COPTCONDITIONLIST &opt_cond_list, std::vector<size_t> &optimal_idx)
{
	// size_t n = opt_cond_list.list_view_quality.size();
	// Eigen::VectorXd one_vector = Eigen::VectorXd::Ones(n);
	// Eigen::VectorXd zero_vector = Eigen::VectorXd::Zero(n);

	// Eigen::VectorXd list_vq = m_math.vec2Eigen(opt_cond_list.list_view_quality);
	// Eigen::VectorXd list_le = m_math.vec2Eigen(opt_cond_list.list_light_effect);
	// Eigen::VectorXd list_pt = m_math.vec2Eigen(opt_cond_list.list_path_time);

	// double margin_vq = opt_cond_list.opt_setting_data.margin_vq;
	// double margin_le = opt_cond_list.opt_setting_data.margin_le;
	// double margin_pt = opt_cond_list.opt_setting_data.margin_pt;

	// std::vector<size_t> idx_in = m_math.genSameWidthValueSet(1, n, 1); // 1:1:n

	// Eigen::VectorXd weight_vq = calLowerBoundWeightUsingTanh(list_vq, opt_cond_list.opt_setting_data.min_vq, opt_cond_list.opt_setting_data.margin_vq);
	// Eigen::VectorXd weight_le = calUpperBoundWeightUsingTanh(list_le, opt_cond_list.opt_setting_data.min_vq, opt_cond_list.opt_setting_data.margin_le);
	// Eigen::VectorXd weight_pt = calUpperBoundWeightUsingTanh(list_pt, opt_cond_list.opt_setting_data.min_vq, opt_cond_list.opt_setting_data.margin_pt);

	// // 모든 부등식은 A*x <= b 형식이 기본이므로, A*x >= b일 경우, -A*x <= -b 의 형태로 변경하여 입력
	// Eigen::MatrixXd A(5, n); // inspection conditions, [vq, le, pt, -1, 1]
	// A.row(0) = -weight_vq;
	// A.row(1) = -weight_le;
	// A.row(2) = -weight_pt;
	// A.row(3) = -one_vector;
	// A.row(4) = one_vector;

	// Eigen::VectorXd b(5); // constraints
	// b[0] = -opt_cond_list.opt_setting_data.min_cutoff_value;
	// b[1] = -opt_cond_list.opt_setting_data.min_cutoff_value;
	// b[2] = -opt_cond_list.opt_setting_data.min_cutoff_value;
	// b[3] = -opt_cond_list.opt_setting_data.min_sol_num;
	// b[4] = opt_cond_list.opt_setting_data.max_sol_num;

	// // weighted inspection list
	// Eigen::VectorXd list_vq_weighted = list_vq - margin_vq*(one_vector - weight_vq);
	// Eigen::VectorXd list_le_weighted = list_le + margin_le*(one_vector - weight_le);
	// Eigen::VectorXd list_pt_weighted = list_pt + margin_pt*(one_vector - weight_pt);

	// // objective function
	// Eigen::VectorXd f;
	// switch (opt_cond_list.opt_setting_data.objective_case)
	// {
	// case OBJECTIVE_MAXIMIZE_VIEW_QUALITY: //
	// 	f = -list_vq_weighted.cwiseProduct(one_vector);
	// 	break;
	// case OBJECTIVE_MINIMIZE_LIGHT_EFFECT: //
	// 	f = list_le_weighted.cwiseProduct(one_vector);
	// 	break;
	// case OBJECTIVE_MINIMIZE_PATH_TIME: //
	// 	f = list_pt_weighted.cwiseProduct(one_vector);
	// 	break;
	// }
	// ////////////////////
	// //// Linear programming solver
	// // Optimization solver (third party)
	// // x1, ..., xn is binary
	// Eigen::VectorXd x = solveMLP(f, A, b, false);
	// ///////////////////
	// std::vector<size_t> idx_out;
	// // optimal viewpoint index
	// for (size_t i = 0; i < n; i++)
	// {
	// 	if (x[i]) // if x is not zero.
	// 	{
	// 		idx_out.push_back(i);
	// 	}
	// }
	// optimal_idx = idx_out;
}
