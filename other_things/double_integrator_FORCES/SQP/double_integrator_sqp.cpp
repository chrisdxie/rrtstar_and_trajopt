extern "C" {
#include "double_integrator_QP_solver.h"
double_integrator_QP_solver_FLOAT **f, **lb, **ub, **A, **b, **z;
}

#define TIMESTEPS 12
const int T = TIMESTEPS;

#define X_DIM 4
#define U_DIM 2
#define O_DIM 1
#define TOTAL_VARS

#include <vector>
#include <iostream>

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#include "boost/preprocessor.hpp"

typedef Matrix<double, X_DIM, 1> VectorX;
typedef Matrix<double, U_DIM, 1> VectorU;
typedef Matrix<double, O_DIM, 1> VectorO;

typedef std::vector<VectorX> StdVectorX;
typedef std::vector<VectorU> StdVectorU;
typedef std::vector<VectorO> StdVectorO;

namespace cfg { // TODO
const double improve_ratio_threshold = .1; // .1
const double min_approx_improve = 1; // 1e-2
const double min_trust_box_size = 1e-3; // 1e-3
const double trust_shrink_ratio = .5; // .5
const double trust_expand_ratio = 1.25; // 1.25
const double cnt_tolerance = .5; // .5
const double penalty_coeff_increase_ratio = 5; // 5
const double initial_penalty_coeff = 10; // 10
const double initial_trust_box_size = 5; // 5 // split up trust box size for X and U
const int max_penalty_coeff_increases = 3; // 3
const int max_sqp_iterations = 50; // 50
}

inline void fill_col_major(double *X, const MatrixXd& XMat) {
	int idx = 0;
	int num_cols = XMat.cols();
	int num_rows = XMat.rows();
	for(size_t c = 0; c < num_cols; ++c) {
		for(size_t r = 0; r < num_rows; ++r) {
			X[idx++] = XMat(c, r);
		}
	}
}

void setup_state_vars(double_integrator_QP_solver_params& problem, double_integrator_QP_solver_output& output)
{

//	 **f, **lb, **ub, **A, **b;
	 // problem inputs
	f = new double_integrator_QP_solver_FLOAT*[T];
	lb = new double_integrator_QP_solver_FLOAT*[T];
	ub = new double_integrator_QP_solver_FLOAT*[T];
	A = new double_integrator_QP_solver_FLOAT*[T];
	b = new double_integrator_QP_solver_FLOAT*[T];

	// problem outputs
	z = new double_integrator_QP_solver_FLOAT*[T];


#define SET_VARS(n) \
		f[ BOOST_PP_SUB(n,1) ] = problem.f##n ; \
		lb[ BOOST_PP_SUB(n,1) ] = problem.lb##n ; \
		ub[ BOOST_PP_SUB(n,1) ] = problem.ub##n ; \
		A[ BOOST_PP_SUB(n,1) ] = problem.ub##n ; \
		b[ BOOST_PP_SUB(n,1) ] = problem.ub##n ; \
		z[ BOOST_PP_SUB(n,1) ] = output.z##n ;
#define BOOST_PP_LOCAL_MACRO(n) SET_VARS(n)
#define BOOST_PP_LOCAL_LIMITS (1, TIMESTEPS)

	// TODO: fill in rest
	fill_col_major(f[0], INFINITY*Matrix<double,2*X_DIM+2*U_DIM+1+O_DIM+X_DIM,1>::Ones());

	for(int t = 0; t < T-1; ++t) {

	}

}

void cleanup_state_vars() {
	delete[] f;
	delete[] lb;
	delete[] ub;
	delete[] A;
	delete[] b;
	delete[] z;
}

bool is_valid_inputs() {
	// TODO: check none of the FORCES variables are infinity
	return true;
}

bool minimize_merit_function(StdVectorX& X, StdVectorU& U, double penalty_coeff,
		double_integrator_QP_solver_params& problem, double_integrator_QP_solver_output& output, double_integrator_QP_solver_info& info) {
	double trust_box_size = cfg::initial_trust_box_size;

	double merit = 0, optcost = 0;
	int index = 0;

	StdVectorX Xopt(T);
	StdVectorU Uopt(T-1);
	double deltaopt;

	// fill in f

	for(int sqp_iter=0; true; ++sqp_iter) {
		std::cout << "  sqp iter: " << sqp_iter << "\n";

		VectorXd gradient;
		MatrixXd hessian;
		// TODO: calculate

//		double merit = ...

		for(int trust_iter=0; true; ++trust_iter) {
			std::cout << "   trust region size: " << trust_box_size << "\n";

			// fill in lb, ub, A, b

			if (!is_valid_inputs()) {
				std::cout << "ERROR: invalid inputs\n";
				exit(0);
			}

			// call FORCES
			int exitflag = double_integrator_QP_solver_solve(&problem, &output, &info);
			if (exitflag == 1) {
				optcost = info.pobj;
				for(int t=0; t < T; ++t) {
					index = 0;
					for(int i=0; i < X_DIM; ++i) {
						Xopt[t](i) = z[t][index++];
					}
					if (t < T-1) {
						for(int i=0; i < U_DIM; ++i) {
							Uopt[t](i) = z[t][index++];
						}
					}
				}
//				deltaopt TODO
			} else {
				std::cout << "Some problem in solver\n";
				exit(0);
			}

			// TODO trust box update

		} // end third loop

	} // end second loop

}

double penalty_sqp(StdVectorX& X, StdVectorU& U,
		double_integrator_QP_solver_params& problem, double_integrator_QP_solver_output& output, double_integrator_QP_solver_info& info) {
	double penalty_coeff = cfg::initial_penalty_coeff;
	int penalty_increases = 0;

	while(penalty_increases < cfg::max_penalty_coeff_increases) {
		bool success = minimize_merit_function(X, U, penalty_coeff, problem, output, info);

		// TODO: compute dynamics violation
		bool constraints_satisfied = true;

		if (constraints_satisfied) {
			// TODO: return cost
		} else {
			penalty_increases++;
			penalty_coeff *= cfg::penalty_coeff_increase_ratio;
		}
	}
}

int main() {
	double_integrator_QP_solver_params problem;
	double_integrator_QP_solver_output output;
	double_integrator_QP_solver_info info;
	setup_state_vars(problem, output);

	StdVectorX X(T);
	StdVectorU U(T-1);

	penalty_sqp(X, U, problem, output, info);

	cleanup_state_vars();
}
