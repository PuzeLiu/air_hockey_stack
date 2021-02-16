#include "air_hockey_baseline_agent/utils.h"

using namespace Eigen;

void air_hockey_baseline_agent::GetNullSpace(const Eigen::MatrixXd &A, MatrixXd &out_null_space) {
	CompleteOrthogonalDecomposition<Matrix<double, Dynamic, Dynamic> > cod;
	cod.compute(A);
	// Find URV^T
	MatrixXd V_ = cod.matrixZ().transpose();
	out_null_space = V_.block(0, cod.rank(), V_.rows(), V_.cols() - cod.rank());
//    MatrixXd P = cod.colsPermutation();
	out_null_space = cod.colsPermutation() * out_null_space; // Unpermute the columns
}