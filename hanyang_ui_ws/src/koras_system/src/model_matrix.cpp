#include "model_matrix.hpp"

// ModelMatrix
ModelMatrix::ModelMatrix()
    : row_(4), column_(4), is_tr_mat_(false) {
    element_.resize(row_ * column_);
}

ModelMatrix::ModelMatrix(const ModelMatrix &other)
    : row_(other.row_), column_(other.column_), is_tr_mat_(other.is_tr_mat_), element_(other.element_) {
}

ModelMatrix::ModelMatrix(const unsigned int row, const unsigned int column)
    : row_(row), column_(column), is_tr_mat_(false) {
    element_.resize(row_ * column_);
}

ModelMatrix::ModelMatrix(const unsigned int row, const unsigned int column, const double *element)
    : row_(row), column_(column), element_(element, element + (row * column)), is_tr_mat_(false) {
}

ModelMatrix::ModelMatrix(const unsigned int row, const unsigned int column, const double **element)
    : row_(row), column_(column), is_tr_mat_(false) {
    element_.resize(row_ * column_);

    for (unsigned int r = 0; r < row; r++) {
        for (unsigned int c = 0; c < column; c++) {
            element_[r * column + c] = element[r][c];
        }
    }
}

ModelMatrix::ModelMatrix(const unsigned int row, const unsigned int column, const vector<double> element)
    : row_(row), column_(column), element_(element), is_tr_mat_(false) {
}

ModelMatrix::ModelMatrix(const unsigned int row, const unsigned int column, bool is_tr_mat)
    : row_(row), column_(column) {
    if (row_ == 4 && column_ == 4) {
        is_tr_mat_ = true;
    }

    element_.resize(row_ * column_);
}

ModelMatrix::~ModelMatrix() {
    element_.clear();
}

unsigned int ModelMatrix::row() const {
    return row_;
}

unsigned int ModelMatrix::column() const {
    return column_;
}

vector<double> ModelMatrix::element() const {
    return element_;
}

double ModelMatrix::get(const unsigned int row, const unsigned int column) const {
    if (row > row_) {
        return 0.0;
    } else if (column > column_) {
        return 0.0;
    } else {
        return element_[row * column_ + column];
    }
}

double ModelMatrix::getElement_plus(unsigned int row, unsigned int column) {
    return element_[(row - 1) * column_ + (column - 1)];
}

void ModelMatrix::set(const unsigned int row, const unsigned int column, const double value) {
    if (row > row_) {
        return;
    } else if (column > column_) {
        return;
    } else {
        element_[row * column_ + column] = value;
    }
}

void ModelMatrix::setElement_plus(unsigned int row, unsigned int column, double element) {
    element_[(row - 1) * column_ + (column - 1)] = element;
}

ModelMatrix ModelMatrix::zero(const unsigned int row, const unsigned int column) {
    return ModelMatrix(row, column);
}

ModelMatrix ModelMatrix::one(const unsigned int row, const unsigned int column) {
    vector<double> mat(row * column);
    for (unsigned int r = 0; r < row; r++) {
        for (unsigned int c = 0; c < column; c++) {
            mat[r * column + c] = 1.0;
        }
    }
    return ModelMatrix(row, column, mat);
}

ModelMatrix ModelMatrix::identity(const unsigned int row, const unsigned int column) {
    vector<double> mat(row * column);
    for (unsigned int r = 0; r < row; r++) {
        for (unsigned int c = 0; c < column; c++) {
            if (r == c) {
                mat[r * column + c] = 1.0;
            } else {
                mat[r * column + c] = 0.0;
            }
        }
    }
    return ModelMatrix(row, column, mat);
}

ModelMatrix ModelMatrix::transpose() {
    vector<double> ele(row_ * column_);
    for (unsigned int r = 0; r < row_; r++) {
        for (unsigned int c = 0; c < column_; c++) {
            ele[c * row_ + r] = element_[r * column_ + c];
        }
    }
    return ModelMatrix(column_, row_, ele);
}

double ModelMatrix::determinant() {
    auto calDeterminant = [] (vector<double> matrix, int order) {
        // the determinant value
        double det = 1.0;

        // stop the recursion when matrix is a single element
        if (order == 1) {
            det = matrix[0];
        } else if (order == 2) {
            det = matrix[0 * 2 + 0] * matrix[1 * 2 + 1] - matrix[0 * 2 + 1] * matrix[1 * 2 + 0];
        } else if (order == 3) {
            det = matrix[0 * 3 + 0] * matrix[1 * 3 + 1] * matrix[2 * 3 + 2] + matrix[0 * 3 + 1] * matrix[1 * 3 + 2] * matrix[2 * 3 + 0] + matrix[0 * 3 + 2] * matrix[1 * 3 + 0] * matrix[2 * 3 + 1] - matrix[0 * 3 + 0] * matrix[1 * 3 + 2] * matrix[2 * 3 + 1] - matrix[0 * 3 + 1] * matrix[1 * 3 + 0] * matrix[2 * 3 + 2] - matrix[0 * 3 + 2] * matrix[1 * 3 + 1] * matrix[2 * 3 + 0];
        } else {
            // generation of temporary matrix
            vector<double> temp_matrix = matrix;

            // gaussian elimination
            for (int i = 0; i < order; i++) {
                // find max low
                double temp = 0.000;
                int max_row = i;
                for (int j = i; j < order; j++) {
                    if (fabs(temp_matrix[j * order + i]) > temp) {
                        temp = fabs(temp_matrix[j * order + i]);
                        max_row = j;
                    }
                }
                if (fabs(temp_matrix[max_row * order + i]) > 0.0001) {
                    // transfer row
                    if (max_row != i) {
                        for (int j = 0; j < order; j++) {
                            temp = -temp_matrix[max_row * order + j];
                            temp_matrix[max_row * order + j] = temp_matrix[i * order + j];
                            temp_matrix[i * order + j] = temp;
                        }
                    }
                    // elemination
                    for (int j = i + 1; j < order; j++) {
                        temp = temp_matrix[j * order + i] / temp_matrix[i * order + i];
                        for (int k = i; k < order; k++) {
                            temp_matrix[j * order + k] -= temp_matrix[i * order + k] * temp;
                        }
                    }
                }
            }

            for (int i = 0; i < order; i++) {
                det *= temp_matrix[i * order + i];
            }
        }

        return det;
    };

    if (row_ == column_) {
        return calDeterminant(element_, row_);
    } else if (row_ > column_) {
        return calDeterminant((this->transpose() * (*this)).element_, column_);
    } else {
        return calDeterminant(((*this) * this->transpose()).element_, row_);
    }
}

ModelMatrix ModelMatrix::inverse() {
    auto matInversion = [] (vector<double> matrix, int order) {
        vector<double> matA = matrix;
        vector<double> matB = ModelMatrix::identity(order, order).element_;

        // Gauss-Jordan
        // Forward
        for (int i = 0; i < order; i++) {
            // max row
            double temp = 0.000;
            int max_row = i;
            for (int j = i; j < order; j++) {
                if (fabs(matA[j * order + i]) > temp) {
                    temp = fabs(matA[j * order + i]);
                    max_row = j;
                }
            }
            // change row
            double temp2 =  matA[max_row * order + i];
            for (int j = 0; j < order; j++) {
                temp = matA[max_row * order + j];
                matA[max_row * order + j] = matA[i * order + j];
                matA[i * order + j] = temp / temp2;

                temp = matB[max_row * order + j];
                matB[max_row * order + j] = matB[i * order + j];
                matB[i * order + j] = temp / temp2;
            }
            for (int j = i + 1; j < order; j++) {
                temp = matA[j * order + i];
                for (int k = 0; k < order; k++) {
                    matA[j * order + k] -= matA[i * order + k] * temp;
                    matB[j * order + k] -= matB[i * order + k] * temp;
                }
            }
        }

        // Backward
        for (int i = order - 1; i >= 0; i--) {
            for (int j = i - 1; j >= 0; j--) {
                double temp = matA[j * order + i];
                for (int k = 0; k < order; k++) {
                    matA[j * order + k] -= matA[i * order + k] * temp;
                    matB[j * order + k] -= matB[i * order + k] * temp;
                }
            }
        }

        return matB;
    };

    if (row_ == column_) {
        // square matrix
        return ModelMatrix(row_, column_,  matInversion(element_, row_));
    } else {
        // rectangular matrix
        return pseudoInverse();
    }
}

ModelMatrix ModelMatrix::inverse(const double sigma) {
    if (row_ <= column_) {
        // m by n matrix (n >= m)
        // generate sigma digonal matrix
        ModelMatrix temp = ModelMatrix::identity(row_, row_) * sigma;
        // calculation of inverse matrix
        return this->transpose() * ((*this) * (this->transpose()) + temp).inverse();
    } else {
        // generate sigma digonal matrix
        ModelMatrix temp = ModelMatrix::identity(row_, row_) * sigma;
        // calculation of inverse matrix
        return (this->transpose() * (*this) + temp).inverse() * this->transpose();
    }
}

double ModelMatrix::length() const {
    double l = 0.0;
    for (unsigned int r = 0; r < row_; r++) {
        for (unsigned int c = 0; c < column_; c++) {
            l += element_[r * column_ + c] * element_[r * column_ + c];
        }
    }
    return sqrt(l);
}

ModelMatrix ModelMatrix::normalize() const {
    double l = length();
    if (l == 0.0) {
        return ModelMatrix::identity(row_, column_);
    } else {
        vector<double> ele(row_, column_);
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < column_; c++) {
                ele[r * column_ + c] = element_[r * column_ + c] / l;
            }
        }
        return ModelMatrix(row_, column_, ele);
    }
}

double ModelMatrix::dot(const ModelMatrix &rhs) {
    if (row_ == rhs.row_ && column_ == rhs.column_) {
        double dot = 0.0;
        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < column_; c++) {
                dot += element_[r * column_ + c] * rhs.element_[r * column_ + c];
            }
        }
        return dot;
    } else {
        return 0.0;
    }
}

ModelMatrix ModelMatrix::cross(const ModelMatrix &rhs) {
    if (row_ == 3 && column_ == 1 && rhs.row_ == 3 && rhs.column_ == 1) {
        ModelMatrix ele(row_, column_);

        ele.element_[0] = element_[1] * rhs.element_[2] - element_[2] * rhs.element_[1];
        ele.element_[1] = element_[2] * rhs.element_[0] - element_[0] * rhs.element_[2];
        ele.element_[2] = element_[0] * rhs.element_[1] - element_[1] * rhs.element_[0];

        return ele;
    } else {
        return ModelMatrix::zero(3, 1);
    }
}

ModelMatrix ModelMatrix::cross() {
    if (row_ == 3 && column_ == 1) {
        ModelMatrix ele(3, 3);

        ele.element_[0 * 3 + 0] = 0.0;
        ele.element_[0 * 3 + 1] = -element_[2];
        ele.element_[0 * 3 + 2] = element_[1];
        ele.element_[1 * 3 + 0] = element_[2];
        ele.element_[1 * 3 + 1] = 0.0;
        ele.element_[1 * 3 + 2] = -element_[0];
        ele.element_[2 * 3 + 0] = -element_[1];
        ele.element_[2 * 3 + 1] = element_[0];
        ele.element_[2 * 3 + 2] = 0.0;

        return ele;
    } else {
        return ModelMatrix::zero(3, 3);
    }
}

void ModelMatrix::svdcmp(double a[][JS_DOF], int m, int n, double w[], double v[][6]) {
    auto pythag = [] (double a, double b) {
        // compute (a2 + b2)^1/2 without destructive underflow or overflow
        double a_abs = fabs(a);
        double b_abs = fabs(b);

        if (a_abs > b_abs) {
            return a_abs * sqrt(1.0 + (b_abs / a_abs) * (b_abs / a_abs));
        } else {
            return (b_abs == 0.0 ? 0.0 : b_abs * sqrt(1.0 + (a_abs / b_abs) * (a_abs / b_abs)));
        }
    };

    int flag, i, its, j, jj, k, l, nm;
    double anorm, c, f, g, h, s, scale, x, y, z;

    ModelMatrix RowVector(1, n);

    g = scale = anorm = 0.0; /* Householder reduction to bidiagonal form */
    for (i = 1; i <= n; i++) {
        l = i + 1;
        RowVector.setElement_plus(1, i, scale * g);
        g = s = scale = 0.0;
        if (i <= m) {
            for (k = i; k <= m; k++) scale += fabs(a[k - 1][i - 1]);
            if (scale) {
                for (k = i; k <= m; k++) {
                    a[k - 1][i - 1] /= scale;
                    s += a[k - 1][i - 1] * a[k - 1][i - 1];
                }
                f = a[i - 1][i - 1];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                a[i - 1][i - 1] = f - g;
                for (j = l; j <= n; j++) {
                    for (s = 0.0, k = i; k <= m; k++) s += a[k - 1][i - 1] * a[k - 1][j - 1];
                    f = s / h;
                    for (k = i; k <= m; k++) a[k - 1][j - 1] += f * a[k - 1][i - 1];
                }
                for (k = i; k <= m; k++) a[k - 1][i - 1] *= scale;
            }
        }
        w[i - 1] = scale * g;
        g = s = scale = 0.0;
        if (i <= m && i != n) {
            for (k = l; k <= n; k++) scale += fabs(a[i - 1][k - 1]);
            if (scale) {
                for (k = l; k <= n; k++) {
                    a[i - 1][k - 1] /= scale;
                    s += a[i - 1][k - 1] * a[i - 1][k - 1];
                }
                f = a[i - 1][l - 1];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                a[i - 1][l - 1] = f - g;
                for (k = l; k <= n; k++) RowVector.setElement_plus(1, k, a[i - 1][k - 1] / h);
                for (j = l; j <= m; j++) {
                    for (s = 0.0, k = l; k <= n; k++) s += a[j - 1][k - 1] * a[i - 1][k - 1];
                    for (k = l; k <= n; k++) a[j - 1][k - 1] += s * RowVector.getElement_plus(1, k);
                }
                for (k = l; k <= n; k++) a[i - 1][k - 1] *= scale;
            }
        }
        anorm = max(anorm, (fabs(w[i - 1]) + fabs(RowVector.getElement_plus(1, i))));
    }

    for (i = n; i >= 1; i--) { /* Accumulation of right-hand transformations. */
        if (i < n) {
            if (g) {
                for (j = l; j <= n; j++) /* Double division to avoid possible underflow. */
                    v[j - 1][i - 1] = (a[i - 1][j - 1] / a[i - 1][l - 1]) / g;
                for (j = l; j <= n; j++) {
                    for (s = 0.0, k = l; k <= n; k++) s += a[i - 1][k - 1] * v[k - 1][j - 1];
                    for (k = l; k <= n; k++) v[k - 1][j - 1] += s * v[k - 1][i - 1];
                }
            }
            for (j = l; j <= n; j++) v[i - 1][j - 1] = v[j - 1][i - 1] = 0.0;
        }
        v[i - 1][i - 1] = 1.0;
        g = RowVector.getElement_plus(1, i);
        l = i;
    }

    for (i = min(m, n); i >= 1; i--) { /* Accumulation of left-hand transformations. */
        l = i + 1;
        g = w[i - 1];
        for (j = l; j <= n; j++) a[i - 1][j - 1] = 0.0;
        if (g) {
            g = 1.0 / g;
            for (j = l; j <= n; j++) {
                for (s = 0.0, k = l; k <= m; k++) s += a[k - 1][i - 1] * a[k - 1][j - 1];
                f = (s / a[i - 1][i - 1]) * g;
                for (k = i; k <= m; k++) a[k - 1][j - 1] += f * a[k - 1][i - 1];
            }
            for (j = i; j <= m; j++) a[j - 1][i - 1] *= g;
        }
        else for (j = i; j <= m; j++) a[j - 1][i - 1] = 0.0;
        ++a[i - 1][i - 1];
    }

    for (k = n; k >= 1; k--) { /* Diagonalization of the bidiagonal form. */
        for (its = 1; its <= 30; its++) {
            flag = 1;
            for (l = k; l >= 1; l--) { /* Test for splitting. */
                nm = l - 1; /* Note that rv1[1] is always zero. */
                if ((double)(fabs(RowVector.getElement_plus(1, l)) + anorm) == anorm) {
                    flag = 0;
                    break;
                }
                if ((double)(fabs(w[nm - 1]) + anorm) == anorm) break;
            }
            if (flag) {
                c = 0.0; /* Cancellation of rv1[l], if l > 1. */
                s = 1.0;
                for (i = l; i <= k; i++) {
                    f = s * RowVector.getElement_plus(1, i);
                    RowVector.setElement_plus(1, i, c * RowVector.getElement_plus(1, i));
                    if ((double)(fabs(f) + anorm) == anorm) break;
                    g = w[i - 1];
                    h = pythag(f, g);
                    w[i - 1] = h;
                    h = 1.0 / h;
                    c = g * h;
                    s = -f * h;
                    for (j = 1; j <= m; j++) {
                        y = a[j - 1][nm - 1];
                        z = a[j - 1][i - 1];
                        a[j - 1][nm - 1] = y * c + z * s;
                        a[j - 1][i - 1] = z * c - y * s;
                    }
                }
            }
            z = w[k - 1];
            if (l == k) { /* Convergence. */
                if (z < 0.0) { /* Singular value is made nonnegative. */
                    w[k - 1] = -z;
                    for (j = 1; j <= n; j++) v[j - 1][k - 1] = -v[j - 1][k - 1];
                }
                break;
            }
            // if (its == 30) printf("no convergence in 30 svdcmp iterations");
            x = w[l - 1]; /* Shift from bottom 2-by-2 minor. */
            nm = k - 1;
            y = w[nm - 1];
            g = RowVector.getElement_plus(1, nm);
            h = RowVector.getElement_plus(1, k);
            f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
            g = pythag(f, 1.0);
            f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
            c = s = 1.0; /* Next QR transformation: */
            for (j = l; j <= nm; j++) {
                i = j + 1;
                g = RowVector.getElement_plus(1, i);
                y = w[i - 1];
                h = s * g;
                g = c * g;
                z = pythag(f, h);
                RowVector.setElement_plus(1, j, z);
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = g * c - x * s;
                h = y * s;
                y *= c;
                for (jj = 1; jj <= n; jj++) {
                    x = v[jj - 1][j - 1];
                    z = v[jj - 1][i - 1];
                    v[jj - 1][j - 1] = x * c + z * s;
                    v[jj - 1][i - 1] = z * c - x * s;
                }
                z = pythag(f, h);
                w[j - 1] = z; /* Rotation can be arbitrary if z = 0. */
                if (z) {
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                }
                f = c * g + s * y;
                x = c * y - s * g;
                for (jj = 1; jj <= m; jj++) {
                    y = a[jj - 1][j - 1];
                    z = a[jj - 1][i - 1];
                    a[jj - 1][j - 1] = y * c + z * s;
                    a[jj - 1][i - 1] = z * c - y * s;
                }
            }
            RowVector.setElement_plus(1, l, 0.0);
            RowVector.setElement_plus(1, k, f);
            w[k - 1] = x;
        }
    }
}

ModelMatrix &ModelMatrix::operator=(const ModelMatrix &other) {
    this->row_       = other.row_;
    this->column_    = other.column_;
    this->element_   = other.element_;
    this->is_tr_mat_ = other.is_tr_mat_;
    return *this;
}

ModelMatrix ModelMatrix::operator+(const double &rhs) {
    ModelMatrix right = ModelMatrix::one(row_, column_) * rhs;
    return (*this) + right;
}

ModelMatrix ModelMatrix::operator+(const ModelMatrix &rhs) {
    if (row_ == rhs.row_ && column_ == rhs.column_) {
        ModelMatrix temp(row_, column_);

        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < column_; c++) {
                temp.element_[r * column_ + c] = element_[r * column_ + c] + rhs.element_[r * column_ + c];
            }
        }

        return temp;
    } else {
        return ModelMatrix::zero(row_, column_);
    }
}

ModelMatrix ModelMatrix::operator-(const double &rhs) {
    ModelMatrix right = ModelMatrix::one(row_, column_) * rhs;
    return (*this) - right;
}

ModelMatrix ModelMatrix::operator-(const ModelMatrix &rhs) {
    if (row_ == rhs.row_ && column_ == rhs.column_) {
        ModelMatrix temp(row_, column_);

        for (unsigned int r = 0; r < row_; r++) {
            for (unsigned int c = 0; c < column_; c++) {
                temp.element_[r * column_ + c] = element_[r * column_ + c] - rhs.element_[r * column_ + c];
            }
        }
        return temp;
    } else {
        return ModelMatrix::zero(row_, column_);
    }
}

ModelMatrix ModelMatrix::operator*(const double &rhs) {
    vector<double> temp(row_ * column_);
    for (int r = 0; r < row_; r++) {
        for (int c = 0; c < column_; c++) {
            temp[r * column_ + c] = element_[r * column_ + c] * rhs;
        }
    }
    return ModelMatrix(row_, column_, temp);
}

ModelMatrix ModelMatrix::operator*(const ModelMatrix &rhs) {
    if (column_ == rhs.row_) {
        if (is_tr_mat_ && rhs.is_tr_mat_) {
            ModelMatrix temp(4, 4, true);

            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    for (int k = 0; k < 3; k++) {
                        temp.element_[r * 4 + c] += element_[r * 4 + k] * rhs.element_[k * 4 + c];
                    }
                }
            }

            for (int i = 0; i < 3; i++) {
                temp.element_[i * 4 + 3] = element_[i * 4 + 3];

                for (int j = 0; j < 3; j++) {
                    temp.element_[i * 4 + 3] += element_[i * 4 + j] * rhs.element_[4 * j + 3];
                }
            }

            temp.element_[12] = 0;
            temp.element_[13] = 0;
            temp.element_[14] = 0;
            temp.element_[15] = 1;

            return temp;
        } else {
            ModelMatrix temp(row_, rhs.column_);

            for (int r = 0; r < row_; r++) {
                for (int c = 0; c < rhs.column_; c++) {
                    for (int k = 0; k < column_; k++) {
                        temp.element_[r * rhs.column_ + c] += element_[r * column_ + k] * rhs.element_[k * rhs.column_ + c];
                    }
                }
            }

            return temp;
        }
    } else {
        return ModelMatrix::zero(row_, column_);
    }
}

ModelMatrix operator+(const double &lhs, const ModelMatrix &rhs) {
    ModelMatrix left = ModelMatrix::one(rhs.row_, rhs.column_) * lhs;
    return left + rhs;
}

ModelMatrix operator-(const double &lhs, const ModelMatrix &rhs) {
    ModelMatrix left = ModelMatrix::one(rhs.row_, rhs.column_) * lhs;
    return left - rhs;
}

ModelMatrix operator*(const double &lhs, const ModelMatrix &rhs) {
    ModelMatrix temp(rhs.row_, rhs.column_);

    for (int r = 0; r < rhs.row_; r++) {
        for (int c = 0; c < rhs.column_; c++) {
            temp.element_[r * rhs.column_ + c] = rhs.element_[r * rhs.column_ + c] * lhs;
        }
    }

    if (rhs.row_ == 4 && rhs.column_ == 4) {
        temp.is_tr_mat_ = rhs.is_tr_mat_;
    }

    return temp;
}

ModelMatrix ModelMatrix::pseudoInverse() {
    if (row_ == column_) {
        return inverse();
    } else if (row_ > column_) {
        return pseudoInverseL();
    } else {
        return pseudoInverseR();
    }
}

ModelMatrix ModelMatrix::pseudoInverseR() {
    ModelMatrix a = this->transpose();
    ModelMatrix b = ModelMatrix(row_, column_, element_) * this->transpose();
    ModelMatrix c = b.inverse();
    return a * c;
    // return this->transpose() * ((*this) * (this->transpose())).inverse();
}

ModelMatrix ModelMatrix::pseudoInverseL() {
    ModelMatrix a = this->transpose();
    ModelMatrix d = ModelMatrix(row_, column_, element_);
    ModelMatrix e = a * d;
    ModelMatrix b = e.inverse();
    ModelMatrix c = this->transpose();
    return a * c;
    // return ((this->transpose()) * (*this)).inverse() * this->transpose();
}

void ModelMatrix::quaternion2rpy(double quaternion[4], double rpy[3]) {
    // quaternion = {x, y, z, w}
    // rpy        = {roll, pitch, yaw}

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (quaternion[3] * quaternion[0] + quaternion[1] * quaternion[2]);
    double cosr_cosp = +1.0 - 2.0 * (quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1]);
    rpy[0] = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (quaternion[3] * quaternion[1] - quaternion[2] * quaternion[0]);
    if (fabs(sinp) >= 1)
        rpy[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        rpy[1] = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1]);
    double cosy_cosp = +1.0 - 2.0 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]);
    rpy[2] = atan2(siny_cosp, cosy_cosp);
}

void ModelMatrix::rpy2quaternion(double rpy[3], double quaternion[4]){
    // quaternion = {x, y, z, w}
    // rpy        = {roll, pitch, yaw}

    // Abbreviations for the various angular functions
    double cy = cos(rpy[2] * 0.5);
    double sy = sin(rpy[2] * 0.5);
    double cp = cos(rpy[1] * 0.5);
    double sp = sin(rpy[1] * 0.5);
    double cr = cos(rpy[0] * 0.5);
    double sr = sin(rpy[0] * 0.5);

    quaternion[0] = cy * cp * sr - sy * sp * cr;
    quaternion[1] = sy * cp * sr + cy * sp * cr;
    quaternion[2] = sy * cp * cr - cy * sp * sr;
    quaternion[3] = cy * cp * cr + sy * sp * sr;
}

void ModelMatrix::rot2rpy(const double rot[3][3], double rpy[3]) {
    rpy[0] = atan2(rot[2][1], rot[2][2]) * kRad2Deg;
    rpy[1] = atan2(-rot[2][0], sqrt(rot[2][1] * rot[2][1] + rot[2][2] * rot[2][2])) * kRad2Deg;
    rpy[2] = atan2(rot[1][0], rot[0][0]) * kRad2Deg;
}

void ModelMatrix::rpy2rot(const double rpy[3], double rot[3][3], bool is_rad) {
    double rpy_temp[3];

    for (int i = 0; i < 3; i++) {
        rpy_temp[i] = is_rad ? rpy[i] : rpy[i] * kDeg2Rad;
    }

    rot[0][0] = cos(rpy_temp[2]) * cos(rpy_temp[1]);
    rot[0][1] = cos(rpy_temp[2]) * sin(rpy_temp[1]) * sin(rpy_temp[0]) - sin(rpy_temp[2]) * cos(rpy_temp[0]);
    rot[0][2] = cos(rpy_temp[2]) * sin(rpy_temp[1]) * cos(rpy_temp[0]) + sin(rpy_temp[2]) * sin(rpy_temp[0]);

    rot[1][0] = sin(rpy_temp[2]) * cos(rpy_temp[1]);
    rot[1][1] = sin(rpy_temp[2]) * sin(rpy_temp[1]) * sin(rpy_temp[0]) + cos(rpy_temp[2]) * cos(rpy_temp[0]);
    rot[1][2] = sin(rpy_temp[2]) * sin(rpy_temp[1]) * cos(rpy_temp[0]) - cos(rpy_temp[2]) * sin(rpy_temp[0]);

    rot[2][0] = -sin(rpy_temp[1]);
    rot[2][1] = cos(rpy_temp[1]) * sin(rpy_temp[0]);
    rot[2][2] = cos(rpy_temp[1]) * cos(rpy_temp[0]);
}

ModelMatrix ModelMatrix::pose2tr(ModelMatrix xyzrpy) {
    double rpy[3];
    double rot[3][3];

    ModelMatrix tr(4, 4, true);

    for (int i = 0; i < 3; i++) rpy[i] = xyzrpy.element_[i + 3];
    ModelMatrix::rpy2rot(rpy, rot, false);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) tr.element_[4 * i + j] = rot[i][j];
    }
    for (int i = 0; i < 3; i++) tr.element_[4 * i + 3] = xyzrpy.element_[i];

    tr.element_[12] = 0;
    tr.element_[13] = 0;
    tr.element_[14] = 0;
    tr.element_[15] = 1;

    return tr;
}

ModelMatrix ModelMatrix::tr2pose(ModelMatrix trans_mat) {
    double rot[3][3];
    double rpy[3];

    for (std::size_t i = 0; i < 3; i++)
        for (std::size_t  j = 0; j < 3; j++) rot[i][j] = trans_mat.element_[4*i + j];

    ModelMatrix::rot2rpy(rot, rpy);

    ModelMatrix pose(6, 1);

    for (std::size_t i = 0; i < 3; i++) pose.element_[i] = trans_mat.element_[4 * i + 3];
    for (std::size_t i = 3; i < 6; i++) pose.element_[i] = rpy[i - 3];

    return pose;
}

ModelMatrix ModelMatrix::invTrMat(ModelMatrix trans_mat) {
    ModelMatrix result(4, 4);
    ModelMatrix rot_transpose(3, 3);
    ModelMatrix xyz(3, 1);

    for (std::size_t i = 0; i < 3; i++) rot_transpose.element_[i] = trans_mat.element_[i];
    for (std::size_t i = 3; i < 6; i++) rot_transpose.element_[i] = trans_mat.element_[i + 1];
    for (std::size_t i = 6; i < 9; i++) rot_transpose.element_[i] = trans_mat.element_[i + 2];
    rot_transpose = rot_transpose.transpose();

    for (std::size_t i = 0; i < 3; i++) xyz.element_[i] = (-1)*trans_mat.element_[4*i + 3];
    xyz = rot_transpose * xyz;

    for (std::size_t i = 0; i < 3; i++) result.element_[i]       = rot_transpose.element_[i];
    for (std::size_t i = 3; i < 6; i++) result.element_[i + 1]   = rot_transpose.element_[i];
    for (std::size_t i = 6; i < 9; i++) result.element_[i + 2]   = rot_transpose.element_[i];
    for (std::size_t i = 0; i < 3; i++) result.element_[4*i + 3] = xyz.element_[i];

    result.element_[12] = 0;	result.element_[13] = 0;
    result.element_[14] = 0;	result.element_[15] = 1;

    return result;
}