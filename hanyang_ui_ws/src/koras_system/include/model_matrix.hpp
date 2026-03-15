/**
 * @file model_matrix.hpp
 * @author Inhwan Yoon (inhwan94@korea.ac.kr)
 * @brief Classes for creating and calculating matrices
 * @version 1.0
 * @date 2022-12-29
 *
 * @copyright Copyright (c) 2022 KorasRobotics All Rights Reserved.
 *
 */

#ifndef MODEL_MATRIX_HPP
#define MODEL_MATRIX_HPP

#include <vector>
#include "task_parameter.hpp"
using namespace std;


/**
 * @brief Signum function macro
 */
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))

/**
 * @brief Classes for creating and calculating matrices
 */
class ModelMatrix {
public:
    /**
     * @brief Create a new ModelMatrix object.
     *
     * @details Create a matrix of size 4x4.
     */
    ModelMatrix();

    /**
     * @brief Create a new ModelMatrix object.
     *
     * @details Generates the same matrix as the input ModelMatrix
     */
    ModelMatrix(const ModelMatrix &other);

    /**
     * @brief Create a new ModelMatrix object.
     *
     * @details Create a matrix of (row x column)
     *
     * @param[in] row Number of rows
     * @param[in] column Number of columns
     */
    ModelMatrix(const unsigned int row, const unsigned int column);

    /**
     * @brief Create a new ModelMatrix object.
     *
     * @details After creating a matrix of (rows by columns), the elements of the input 1D array are assigned.
     *
     * @param[in] row Number of rows
     * @param[in] column Number of columns
     * @param[in] element Elements of a matrix (array)
     */
    ModelMatrix(const unsigned int row, const unsigned int column, const double *element);

    /**
     * @brief Create a new ModelMatrix object.
     *
     * @details After creating a matrix of (rows by columns), the elements of the input 2D array are assigned.
     *
     * @param[in] row Number of rows
     * @param[in] column Number of columns
     * @param[in] element Elements of a matrix (2D array)
     */
    ModelMatrix(const unsigned int row, const unsigned int column, const double **element);

    /**
     * @brief Create a new ModelMatrix object.
     *
     * @details After creating a matrix of (rows by columns), the elements of the input vector are assigned.
     *
     * @param[in] row Number of rows
     * @param[in] column Number of columns
     * @param[in] element Elements of a matrix (vector)
     */
    ModelMatrix(const unsigned int row, const unsigned int column, const vector<double> element);

    /**
     * @brief Create a new ModelMatrix object.
     *
     * @details Create a transformation matrix of (4 x 4). If it is not 4x4 size,
     * it is not treated as a transformation matrix.
     *
     * @param[in] row Number of rows (should be 4)
     * @param[in] column Number of columns (should be 4)
     * @param[in] is_tr_mat Transformation matrix flag
     */
    ModelMatrix(const unsigned int row, const unsigned int column, bool is_tr_mat);

    /**
     * @brief Destroy the Model Matrix object
     */
    ~ModelMatrix();

public:
    /**
     * @brief Get number of rows in matrix
     *
     * @returns uint Number of rows
     */
    unsigned int row() const;

    /**
     * @brief Get number of columns in matrix
     *
     * @returns uint Number of columns
     */
    unsigned int column() const;

    /**
     * @brief Get elements of matrix
     *
     * @returns vector<double> Elements of matrix
     */
    vector<double> element() const;

    /**
     * @brief Get the components of matrix according to input rows and columns
     *
     * @param[in] row Number of rows
     * @param[in] column Number of columns
     * @returns double Elements of matrix
     */
    double get(const unsigned int row, const unsigned int column) const;

    /**
     * @brief Get the components of matrix according to input rows and columns
     *
     * @details Input row, column must be +1
     *
     * @deprecated To be integrated into the get function
     *
     * @param[in] row Number of rows
     * @param[in] column Number of columns
     * @returns double Elements of matrix
     */
    double getElement_plus(unsigned int row, unsigned int column);

    /**
     * @brief Set the components of matrix according to input rows and columns
     *
     * @param[in] row Number of rows
     * @param[in] column Number of columns
     * @param[in] value Value of element
     */
    void set(const unsigned int row, const unsigned int column, const double value);

    /**
     * @brief Set the components of matrix according to input rows and columns
     *
     * @details Input row, column must be +1
     *
     * @deprecated To be integrated into the set function
     *
     * @param[in] row Number of rows
     * @param[in] column Number of columns
     * @param[in] element Value of element
     */
    void setElement_plus(unsigned int row, unsigned int column, double element);

    /**
     * @brief Get zero matrix
     *
     * @param[in] row Number of rows
     * @param[in] column Number of columns
     * @returns ModelMatrix Zero matrix
     */
    static ModelMatrix zero(const unsigned int row, const unsigned int column);

    /**
     * @brief Get one matrix
     *
     * @param[in] row Number of rows
     * @param[in] column Number of columns
     * @returns ModelMatrix One matrix
     */
    static ModelMatrix one(const unsigned int row, const unsigned int column);

    /**
     * @brief Get identity matrix
     *
     * @param[in] row Number of rows
     * @param[in] column Number of columns
     * @returns ModelMatrix Identity matrix
     */
    static ModelMatrix identity(const unsigned int row, const unsigned int column);

    /**
     * @brief Get transpose matrix
     *
     * @returns ModelMatrix Transpose matrix
     */
    ModelMatrix transpose();

    /**
     * @brief Get determinant of matrix
     *
     * @details Automatically perform appropriate determinant operations
     * according to the size of the rows and columns of the matrix
     *
     * @returns double Determinant
     */
    double determinant();

    /**
     * @brief Get inverse matrix
     *
     * @returns ModelMatrix Inverse matrix
     */
    ModelMatrix inverse();

    /**
     * @brief Get inverse matrix with damped least square (DLS)
     *
     * @param[in] sigma DLS sigma.
     * @returns ModelMatrix Inverse matrix with DLS
     */
    ModelMatrix inverse(const double sigma);

    /**
     * @brief Get length of vector
     *
     * @returns double Length of vector
     */
    double length() const;

    /**
     * @brief Get normalized vector
     *
     * @returns ModelMatrix Normalized vector
     */
    ModelMatrix normalize() const;

    /**
     * @brief Calculate vector dot product
     *
     * @returns double Result value
     */
    double dot(const ModelMatrix &rhs);

    /**
     * @brief Calculate vector cross product
     *
     * @returns ModelMatrix Result vector
     */
    ModelMatrix cross(const ModelMatrix &rhs);

    /**
     * @brief Convert vector to cross product matrix
     *
     * @returns ModelMatrix Result matrix
     */
    ModelMatrix cross();

    /**
     * @brief Singular value decomposition function
     *
     * @details Singular value decomposition program, svdcmp, from "Numerical Recipes in
     * C" (Cambridge Univ. Press) by W.H. Press, S.A. Teukolsky, W.T. Vetterling, and B.P. Flannery.
     * Given a matrix a[1..m][1..n], this routine computes its singular value decomposition,
     * A = U.W.VT. The matrix U replaces a on output.  The diagonal matrix of singular values
     * W is output as a vector w[1..n]. The matrix V (not the transpose VT) is output as v[1..n][1..n].
     *
     * @param[in] a
     * @param[in] m
     * @param[in] n
     * @param[in] w
     * @param[out] v
     */
    void svdcmp(double a[][JS_DOF], int m, int n, double w[], double v[][6]);

    ///@{
    /** Operators for matrix calculation */
    ModelMatrix &operator=(const ModelMatrix &other);
    ModelMatrix operator+(const double &rhs);
    ModelMatrix operator+(const ModelMatrix &rhs);
    ModelMatrix operator-(const double &rhs);
    ModelMatrix operator-(const ModelMatrix &rhs);
    ModelMatrix operator*(const double &rhs);
    ModelMatrix operator*(const ModelMatrix &rhs);

    friend ModelMatrix operator+(const double &lhs, const ModelMatrix &rhs);
    friend ModelMatrix operator-(const double &lhs, const ModelMatrix &rhs);
    friend ModelMatrix operator*(const double &lhs, const ModelMatrix &rhs);
    ///@}

public:
    unsigned int row_;       /**< Number of rows */
    unsigned int column_;    /**< Number of columns */
    vector<double> element_; /**< Elements of matrix */
    bool is_tr_mat_;         /**< Transformation matrix flag */

private:
    /**
     * @brief Get pseudo inverse matrix
     *
     * @details Automatically perform right pseudo inverse or left pseudo inverse operation
     * according to the row and column size of the matrix
     *
     * @return ModelMatrix Pseudo inverse matrix
     */
    ModelMatrix pseudoInverse();

    /**
     * @brief Get right pseudo inverse matrix
     *
     * @return ModelMatrix Right pseudo inverse matrix
     */
    ModelMatrix pseudoInverseR();

    /**
     * @brief Get left pseudo inverse matrix
     *
     * @return ModelMatrix Left pseudo inverse matrix
     */
    ModelMatrix pseudoInverseL();

public:
    static void quaternion2rpy(double quaternion[4], double rpy[3]);
    static void rpy2quaternion(double rpy[3], double quaternion[4]);
    static void rot2rpy(const double rot[3][3], double rpy[3]);
    static void rpy2rot(const double rpy[3], double rot[3][3], bool is_rad = false);
    static ModelMatrix pose2tr(ModelMatrix xyzrpy);
    static ModelMatrix tr2pose(ModelMatrix trans_mat);
    static ModelMatrix invTrMat(ModelMatrix trans_mat);
};

#endif // MODEL_MATRIX_HPP
