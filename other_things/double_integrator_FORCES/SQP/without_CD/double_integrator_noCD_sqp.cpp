
extern "C" {
#include "double_integrator_QP_solver_noCD.h"
double_integrator_QP_solver_noCD_FLOAT **f, **lb, **ub, **A, **b, **z;
}

#include <iostream>

// Include double integrator dynamics
#include "../../../double_integrator_dynamics_library/double_integrator_dynamics.hpp"
using namespace double_integrator_dynamics;

#define INFTY 1e10

#define TIMESTEPS 12
const int T = TIMESTEPS; // I am mixing N and T for timesteps. They are SAME THING.

#define X_DIM 4
#define U_DIM 2
#define TOTAL_VARS

// Simplicity is key
#define Z_DIM_1_TO_N_MINUS_2 2*X_DIM+2*U_DIM+1+X_DIM
#define Z_DIM_N_MINUS_1 2*X_DIM+U_DIM+1+X_DIM
#define Z_DIM_N X_DIM
#define LB_DIM_1 X_DIM+U_DIM+1
#define UB_DIM_1 X_DIM+U_DIM
#define LB_DIM_2_TO_N_MINUS_1 X_DIM/2+U_DIM
#define UB_DIM_2_TO_N_MINUS_1 X_DIM/2+U_DIM
#define LB_DIM_N X_DIM
#define UB_DIM_N X_DIM
#define A_DIM_OUTPUT_1_TO_N_MINUS_1 4*X_DIM+2*U_DIM+2
#define B_DIM_1_TO_N_MINUS_1 4*X_DIM+2*U_DIM+2
#define A_DIM_OUTPUT_N 2*X_DIM
#define B_DIM_N 2*X_DIM

#include <vector>
#include <iostream>

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#include "boost/preprocessor.hpp"

typedef Matrix<double, X_DIM, 1> VectorX;
typedef Matrix<double, U_DIM, 1> VectorU;

typedef std::vector<VectorX> StdVectorX;
typedef std::vector<VectorU> StdVectorU;

namespace cfg { // Taken from matlab implementation
const double improve_ratio_threshold = .3; // .25
const double min_approx_improve = 1e-2;
const double min_trust_box_size = 1e-4;
const double trust_shrink_ratio = .75; // .1
const double trust_expand_ratio = 1.2; // 1.5
const double cnt_tolerance = 1e-4;
const double penalty_coeff_increase_ratio = 10;
const double initial_penalty_coeff = 1;
const double initial_trust_box_size = .1;
const int max_penalty_coeff_increases = 5;
const int max_sqp_iterations = 50;
}

struct bounds_t {
	double x_min, x_max, v_min, v_max, u_min, u_max;
	VectorXd x_start;
	VectorXd x_goal;
};

// fill in X in column major format with matrix XMat
inline void fill_col_major(double *X, const MatrixXd& XMat) {
	int idx = 0;
	int num_cols = XMat.cols();
	int num_rows = XMat.rows();
	for(size_t c = 0; c < num_cols; ++c) {
		for(size_t r = 0; r < num_rows; ++r) {
			X[idx++] = XMat(r, c);
		}
	}
}

void setup_state_vars(double_integrator_QP_solver_noCD_params& problem, double_integrator_QP_solver_noCD_output& output)
{

	/* Initialize problem inputs and outputs as double arrays */

	// problem inputs
	f = new double_integrator_QP_solver_noCD_FLOAT*[T];
	lb = new double_integrator_QP_solver_noCD_FLOAT*[T];
	ub = new double_integrator_QP_solver_noCD_FLOAT*[T];
	A = new double_integrator_QP_solver_noCD_FLOAT*[T];
	b = new double_integrator_QP_solver_noCD_FLOAT*[T];

	// problem outputs
	z = new double_integrator_QP_solver_noCD_FLOAT*[T];

	/* Link them via boost to something, IDK how this works */
#define SET_VARS(n) \
		f[ BOOST_PP_SUB(n,1) ] = problem.f##n ; \
		lb[ BOOST_PP_SUB(n,1) ] = problem.lb##n ; \
		ub[ BOOST_PP_SUB(n,1) ] = problem.ub##n ; \
		A[ BOOST_PP_SUB(n,1) ] = problem.A##n ; \
		b[ BOOST_PP_SUB(n,1) ] = problem.b##n ; \
		z[ BOOST_PP_SUB(n,1) ] = output.z##n ;
#define BOOST_PP_LOCAL_MACRO(n) SET_VARS(n)
#define BOOST_PP_LOCAL_LIMITS (1, TIMESTEPS)
#include BOOST_PP_LOCAL_ITERATE()

	/* Initalize everything to infinity */

	// f_1, ..., f_{N-2}
	for(int t = 0; t < T-2; ++t) {
		fill_col_major(f[t], INFTY*Matrix<double,Z_DIM_1_TO_N_MINUS_2,1>::Ones());
	}
	// f_{N-1}
	fill_col_major(f[T-2], INFTY*Matrix<double,Z_DIM_N_MINUS_1,1>::Ones());
	// f_N
	fill_col_major(f[T-1], INFTY*Matrix<double,Z_DIM_N,1>::Ones());

	// lb_1, ub_1
	fill_col_major(lb[0], INFTY*Matrix<double,LB_DIM_1,1>::Ones());
	fill_col_major(ub[0], INFTY*Matrix<double,UB_DIM_1,1>::Ones());
	// lb_2, ..., lb_{N-1}
	for(int t = 1; t < T-1; ++t) {
		fill_col_major(lb[t], INFTY*Matrix<double,LB_DIM_2_TO_N_MINUS_1,1>::Ones());
		fill_col_major(ub[t], INFTY*Matrix<double,UB_DIM_2_TO_N_MINUS_1,1>::Ones());
	}
	// lb_N, ub_N
	fill_col_major(lb[T-1], INFTY*Matrix<double,LB_DIM_N,1>::Ones());
	fill_col_major(ub[T-1], INFTY*Matrix<double,UB_DIM_N,1>::Ones());

	// A_1, ..., A_{N-2}, b_1, ..., b_{N-2}
	for(int t = 0; t < T-2; ++t) {
		fill_col_major(A[t], INFTY*Matrix<double,A_DIM_OUTPUT_1_TO_N_MINUS_1,Z_DIM_1_TO_N_MINUS_2>::Ones());
		fill_col_major(b[t], INFTY*Matrix<double,B_DIM_1_TO_N_MINUS_1,1>::Ones());
	}
	// A_{N-1}, b_{N-1}
	fill_col_major(A[T-2], INFTY*Matrix<double,A_DIM_OUTPUT_1_TO_N_MINUS_1,Z_DIM_N_MINUS_1>::Ones());
	fill_col_major(b[T-2], INFTY*Matrix<double,B_DIM_1_TO_N_MINUS_1,1>::Ones());
	// A_N, b_N
	fill_col_major(A[T-1], INFTY*Matrix<double,A_DIM_OUTPUT_N,Z_DIM_N>::Ones());
	fill_col_major(b[T-1], INFTY*Matrix<double,B_DIM_N,1>::Ones());

	// z_1, ..., z_{N-2}
	for(int t = 0; t < T-1; ++t) {
		fill_col_major(z[t], INFTY*Matrix<double, Z_DIM_1_TO_N_MINUS_2, 1>::Ones());
	}
	// z_{N-1}
	fill_col_major(z[T-2], INFTY*Matrix<double, Z_DIM_N_MINUS_1, 1>::Ones());
	// z_N
	fill_col_major(z[T-1], INFTY*Matrix<double, Z_DIM_N, 1>::Ones());

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
	/* Check if any of the values have not been touched, i.e. they are
	 * still infinity.
	 */

	// Check f_1, ..., f_{N-2}
	for(int t = 0; t < T-2; ++t) {
		for(int i = 0; i < Z_DIM_1_TO_N_MINUS_2; ++i) {
			if (f[t][i] == INFTY) {return false;}
		}
	}
	// Check f_{N-1}
	for(int i = 0; i < Z_DIM_N_MINUS_1; ++i) {
		if (f[T-2][i] == INFTY) {return false;}
	}
	// Check f_{N}
	for(int i = 0; i < Z_DIM_N; ++i) {
		if (f[T-1][i] == INFTY) {return false;}
	}

	// Check lb_1, ub_1
	for(int i = 0; i < LB_DIM_1; ++i) {
		if (lb[0][i] == INFTY) {return false;}
	}
	for(int i = 0; i < UB_DIM_1; ++i) {
		if (ub[0][i] == INFTY) {return false;}
	}
	// Check lb_2, ..., lb_{N-1}, ub_2, ..., ub_{N-1}
	for(int t = 1; t < T-1; ++t) {
		for(int i = 0; i < LB_DIM_2_TO_N_MINUS_1; ++i) {
			if (lb[t][i] == INFTY) {return false;}
		}
		for(int i = 0; i < UB_DIM_2_TO_N_MINUS_1; ++i) {
			if (ub[t][i] == INFTY) {return false;}
		}
	}
	// CHeck lb_N, ub_N
	for(int i = 0; i < LB_DIM_N; ++i) {
		if (lb[T-1][i] == INFTY) {return false;}
	}
	for(int i = 0; i < UB_DIM_N; ++i) {
		if (ub[T-1][i] == INFTY) {return false;}
	}

	// Check A_1, ..., A_{N-2}, b_1, ..., b_{N-2}
	for(int t = 0; t < T-2; ++t) {
		// A is a matrix
		for (int i = 0; i < A_DIM_OUTPUT_1_TO_N_MINUS_1*Z_DIM_1_TO_N_MINUS_2; ++i) {
			if (A[t][i] == INFTY) {return false;}
		}
		// b is a vector
		for (int i = 0; i < B_DIM_1_TO_N_MINUS_1; ++i) {
			if (b[t][i] == INFTY) {return false;}
		}
	}
	// Check A_{N-1}, b_{N-1}
	for (int i = 0; i < A_DIM_OUTPUT_1_TO_N_MINUS_1*Z_DIM_N_MINUS_1; ++i) {
		if (A[T-2][i] == INFTY) {return false;}
	}
	for (int i = 0; i < B_DIM_1_TO_N_MINUS_1; ++i) {
		if (b[T-2][i] == INFTY) {return false;}
	}
	// Check A_N, b_N
	for (int i = 0; i < A_DIM_OUTPUT_N*Z_DIM_N; ++i) {
		if (A[T-1][i] == INFTY) {return false;}
	}
	for (int i = 0; i < B_DIM_N; ++i) {
		if (b[T-1][i] == INFTY) {return false;}
	}

	// Inputs are valid!
	return true;
}

/* Fill in f using penalty coefficient */
void fill_f(double penalty_coeff) {

	// f_1
	VectorXd f1_temp(Z_DIM_1_TO_N_MINUS_2);
	for(int i = 0; i < Z_DIM_1_TO_N_MINUS_2; ++i) {
		if (i == 2*X_DIM+2*U_DIM) { f1_temp(i) = 1;}
		else if (i >= 2*X_DIM+2*U_DIM+1) { f1_temp(i) = penalty_coeff; }
		else { f1_temp(i) = 0;}
	}
	fill_col_major(f[0], f1_temp);
//	std::cout << "f1:\n" << f1_temp << "\n";

	// f_2, ..., f_{N-2}
	VectorXd ft_temp(Z_DIM_1_TO_N_MINUS_2);
	for(int t = 1; t < T-2; ++t) {
		for(int i = 0; i < Z_DIM_1_TO_N_MINUS_2; ++i) {
			if (i >= 2*X_DIM+2*U_DIM+1) { ft_temp(i) = penalty_coeff; }
			else { ft_temp(i) = 0; }
		}
		fill_col_major(f[t], ft_temp);
//		std::cout << "f" << t+1 << ":\n" << ft_temp << "\n";
	}

	// f_{N-1}
	VectorXd f_almost_temp(Z_DIM_N_MINUS_1);
	for(int i = 0; i < Z_DIM_N_MINUS_1; ++i) {
		if (i >= 2*X_DIM+U_DIM+1) { f_almost_temp(i) = penalty_coeff; }
		else { f_almost_temp(i) = 0; }
	}
	fill_col_major(f[T-2], f_almost_temp);
//	std::cout << "f_" << T-1 << ":\n" << f_almost_temp << "\n";


	// f_N
	VectorXd f_N_temp(Z_DIM_N);
	for(int i = 0; i < Z_DIM_N; ++i) {
		if (i >= X_DIM) { f_N_temp(i) = penalty_coeff; } // Remember, this state has no time variable, thus the >= instead of >
		else { f_N_temp(i) = 0; }
	}
	fill_col_major(f[T-1], f_N_temp);
//	std::cout << "f_" << T << ":\n" << f_N_temp << "\n";


}

/* Fill in lower bounds and upper bounds using bounds struct */
void fill_lb_and_ub(bounds_t bounds) {

	double eps = 1e-16;

	// lb1, ub1
	VectorXd lb1_temp(LB_DIM_1);
	VectorXd ub1_temp(UB_DIM_1);
	for(int i = 0; i < LB_DIM_1; ++i) { // Taking advantage of fact that LB_DIM_1 = UB_DIM_1 + 1
		if (i < X_DIM) { lb1_temp(i) = bounds.x_start(i) - eps; ub1_temp(i) = bounds.x_start(i) + eps; } // Enforcing start state
		else if (i < X_DIM+U_DIM) {lb1_temp(i) = bounds.u_min; ub1_temp(i) = bounds.u_max; } // Control bounds
		else { lb1_temp(i) = 0; } // i = LB_DIM_1 - 1, this is lower bound on time
	}
	fill_col_major(lb[0], lb1_temp); fill_col_major(ub[0], ub1_temp);
//	std::cout << "lb1:\n" << lb1_temp << "\n";
//	std::cout << "ub1:\n" << ub1_temp << "\n";



	// lb2, ..., lb_{N-1}, ub2, ..., ub_{N-1}
	for(int t = 1; t < T-1; ++t) {
		VectorXd lbt_temp(LB_DIM_2_TO_N_MINUS_1);
		VectorXd ubt_temp(UB_DIM_2_TO_N_MINUS_1);
		for (int i = 0; i < LB_DIM_2_TO_N_MINUS_1; ++i) { // Taking advantage of fact that LB_DIM_2_TO_N_MINUS_1 = UB_DIM_2_TO_N_MINUS_1
			if (i < X_DIM/2) {lbt_temp(i) = bounds.v_min; ubt_temp(i) = bounds.v_max; } // Velocity bounds
			else if (i < X_DIM/2+U_DIM) {lbt_temp(i) = bounds.u_min; ubt_temp(i) = bounds.u_max; } // Control bounds
		}
		fill_col_major(lb[t], lbt_temp); fill_col_major(ub[t], ubt_temp);
//		std::cout << "lb" << t+1 << ":\n" << lbt_temp << "\n";
//		std::cout << "ub" << t+1 << ":\n" << ubt_temp << "\n";
	}

	// lb_N, ub_N
	VectorXd lbN_temp(LB_DIM_N);
	VectorXd ubN_temp(UB_DIM_N);
	for(int i = 0; i < LB_DIM_N; ++i) {
		if (i < X_DIM) { lbN_temp(i) = bounds.x_goal(i) - eps; ubN_temp(i) = bounds.x_goal(i) + eps; } // Enforcing goal state
	}
	fill_col_major(lb[T-1], lbN_temp); fill_col_major(ub[T-1], ubN_temp);
//	std::cout << "lbN:\n" << lbN_temp << "\n";
//	std::cout << "ubN:\n" << ubN_temp << "\n";

}

/* Fill in A and b by linearizing around point X, U, delta */
void fill_in_A_and_b(StdVectorX& X, StdVectorU& U, double* delta, double trust_box_size) {

	double d = *delta;

	// A_1, ..., A_{N-1}, b_1, ..., b_{N-1}
	for(int t = 0; t < T-1; ++t) {

		// Grab x, u, and x_next
		VectorX x = X[t];
		VectorU u = U[t];
		VectorX x_next = X[t+1];

		Matrix<double, 2*X_DIM+U_DIM+1, 1> x_prime;
		x_prime << x, x_next, u, d;

		// Instantiate A_t, b_t, set them to zero
		MatrixXd A_temp(A_DIM_OUTPUT_1_TO_N_MINUS_1, Z_DIM_1_TO_N_MINUS_2);
		Matrix<double, B_DIM_1_TO_N_MINUS_1, 1> b_temp;
		A_temp.setZero();b_temp.setZero();

		// Index of current row we are at
		int curr_row = 0;

		// Grab dynamics jacobian and value
		Matrix<double, X_DIM, X_DIM+U_DIM+1> jac = -1 * double_integrator_dynamics::numerical_jacobian(continuous_double_integrator_dynamics, x, u, d);
		Matrix<double, X_DIM, X_DIM> DH_X = jac.leftCols(X_DIM);
		Matrix<double, X_DIM, U_DIM> DH_U = jac.middleCols(X_DIM, U_DIM);
		Matrix<double, X_DIM, 1> DH_delta = jac.rightCols(1);
		Matrix<double, X_DIM, 2*X_DIM+U_DIM+1> DH;
		DH << DH_X, MatrixXd::Identity(X_DIM, X_DIM), DH_U, DH_delta;
		VectorXd hval = dynamics_difference(continuous_double_integrator_dynamics, x, x_next, u, d);

		// Fill in dynamics rows as given in my notes
		A_temp.middleRows(curr_row, X_DIM) << DH_X, DH_U, MatrixXd::Identity(X_DIM, X_DIM), MatrixXd::Zero(X_DIM, U_DIM), DH_delta, -1*MatrixXd::Identity(X_DIM, X_DIM);
		b_temp.middleRows(curr_row, X_DIM) << DH*x_prime - hval;
		curr_row += X_DIM;
		A_temp.middleRows(curr_row, X_DIM) << -1*DH_X, -1*DH_U, -1*MatrixXd::Identity(X_DIM, X_DIM), MatrixXd::Zero(X_DIM, U_DIM), -1*DH_delta, -1*MatrixXd::Identity(X_DIM, X_DIM);
		b_temp.middleRows(curr_row, X_DIM) << hval - DH*x_prime;
		curr_row += X_DIM;

		// Trust box stuff
		A_temp.middleRows(curr_row, X_DIM+U_DIM+1) << MatrixXd::Identity(X_DIM+U_DIM+1, X_DIM+U_DIM), MatrixXd::Zero(X_DIM+U_DIM+1, X_DIM+U_DIM+1+X_DIM);
		A_temp(curr_row+X_DIM+U_DIM, 2*X_DIM+2*U_DIM) = 1;
		b_temp.middleRows(curr_row, X_DIM) << x + trust_box_size*MatrixXd::Ones(X_DIM,1); curr_row += X_DIM;
		b_temp.middleRows(curr_row, U_DIM) << u + trust_box_size*MatrixXd::Ones(U_DIM,1); curr_row += U_DIM;
		b_temp.middleRows(curr_row, 1) << d + trust_box_size; curr_row += 1;

		A_temp.middleRows(curr_row, X_DIM+U_DIM+1) << -1*MatrixXd::Identity(X_DIM+U_DIM+1, X_DIM+U_DIM), MatrixXd::Zero(X_DIM+U_DIM+1, X_DIM+U_DIM+1+X_DIM);
		A_temp(curr_row+X_DIM+U_DIM, 2*X_DIM+2*U_DIM) = -1;
		b_temp.middleRows(curr_row, X_DIM) << trust_box_size*MatrixXd::Ones(X_DIM,1) - x; curr_row += X_DIM;
		b_temp.middleRows(curr_row, U_DIM) << trust_box_size*MatrixXd::Ones(U_DIM,1) - u; curr_row += U_DIM;
		b_temp.middleRows(curr_row, 1) << trust_box_size - d; curr_row += 1;

		// Take out certain cols, since dimension of Z_{N-1} is different
		if (t == T-2) {
			Matrix<double, A_DIM_OUTPUT_1_TO_N_MINUS_1, Z_DIM_N_MINUS_1> A_N_MINUS_1_temp;
			A_N_MINUS_1_temp << A_temp.middleCols(0, 2*X_DIM+U_DIM), A_temp.rightCols(X_DIM+1);
			A_temp = A_N_MINUS_1_temp;
		}

		fill_col_major(A[t], A_temp);
		fill_col_major(b[t], b_temp);
//		std::cout << "A" << t+1 << ":\n" << A_temp << "\n";
//		std::cout << "b" << t+1 << ":\n" << b_temp << "\n";
	}

	// A_N, b_N
	VectorX x = X[T-1];
	Matrix<double, A_DIM_OUTPUT_N, Z_DIM_N> A_temp;
	Matrix<double, B_DIM_N, 1> b_temp;
	A_temp.setZero(); b_temp.setZero();

	int curr_row = 0;
	A_temp.middleRows(curr_row, X_DIM) << MatrixXd::Identity(X_DIM, X_DIM);
	b_temp.middleRows(curr_row, X_DIM) << x + trust_box_size*MatrixXd::Ones(X_DIM,1);
	curr_row += X_DIM;
	A_temp.middleRows(curr_row, X_DIM) << -1*MatrixXd::Identity(X_DIM, X_DIM);
	b_temp.middleRows(curr_row, X_DIM) << trust_box_size*MatrixXd::Ones(X_DIM,1) - x;

	fill_col_major(A[T-1], A_temp);
	fill_col_major(b[T-1], b_temp);
//	std::cout << "A_N:\n" << A_temp << "\n";
//	std::cout << "b_N:\n" << b_temp << "\n";

}

double computeObjective(double* delta, StdVectorX& X, StdVectorU& U) {
	return *delta;
}

double computeMerit(double* delta, StdVectorX& X, StdVectorU& U, double penalty_coeff) {
	double merit = computeObjective(delta, X, U);
	for(int t = 0; t < T-1; ++t) {
		VectorXd hval = dynamics_difference(continuous_double_integrator_dynamics, X[t], X[t+1], U[t], *delta);
		merit += penalty_coeff*(hval.cwiseAbs()).sum();
	}
	return merit;
}


bool minimize_merit_function(StdVectorX& X, StdVectorU& U, double* delta, bounds_t bounds, double penalty_coeff,
		double_integrator_QP_solver_noCD_params& problem, double_integrator_QP_solver_noCD_output& output, double_integrator_QP_solver_noCD_info& info) {

	// Initialize trust box size
	double trust_box_size = cfg::initial_trust_box_size;

	// Initialize these things
	double merit = 0, optcost = 0;
	int index = 0;

	// Set best trajectory to input trajectory. This will allow us to linearize around the input trajectory for the first iteration.
	StdVectorX Xopt(T);
	StdVectorU Uopt(T-1);
	double deltaopt;

	bool success = true;

	// fill in f, lb, ub. These are constant across all iterations because the penalty is constant until we break from this "loop"
	fill_f(penalty_coeff);
	fill_lb_and_ub(bounds);

	for(int sqp_iter=0; true; ++sqp_iter) {
		std::cout << "  sqp iter: " << sqp_iter << "\n";

		merit = computeMerit(delta, X, U, penalty_coeff);

		for(int trust_iter=0; true; ++trust_iter) {
			std::cout << "   trust region size: " << trust_box_size << "\n";

			// fill in A, b
			fill_in_A_and_b(X, U, delta, trust_box_size);

			if (!is_valid_inputs()) {
				std::cout << "ERROR: invalid inputs\n";
				exit(0);
			}

			// call FORCES
			int exitflag = double_integrator_QP_solver_noCD_solve(&problem, &output, &info);
			if (exitflag == 1) {
				optcost = info.pobj;
				deltaopt = z[0][X_DIM+U_DIM]; // Hard coded, I know the index of this
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
			} else {
				std::cout << "Some problem in solver\n";
				success = false;
				return success;
			}

			double model_merit = optcost;
			double new_merit = computeMerit(&deltaopt, Xopt, Uopt, penalty_coeff);
			double approx_merit_improve = merit - model_merit;
			double exact_merit_improve = merit - new_merit;
			double merit_improve_ratio = exact_merit_improve/approx_merit_improve;

			printf("\tapprox improve: %.3f. exact improve: %.3f. ratio: %.3f \n", approx_merit_improve, exact_merit_improve, merit_improve_ratio);
			if (approx_merit_improve < -1) {
				printf("Approximate merit function got worse (%.3e).\n", approx_merit_improve);
				printf("Either convexification is wrong to zeroth order, or you're in numerical trouble\n");
				success = false;
				return success;
			} else if (approx_merit_improve < cfg::min_approx_improve) {
				printf("Converged: y tolerance\n");
				X = Xopt; U = Uopt; *delta = deltaopt;
				return success;
			} else if ((exact_merit_improve < 0) || (merit_improve_ratio < cfg::improve_ratio_threshold)) {
				trust_box_size = trust_box_size * cfg::trust_shrink_ratio;
			} else {
				trust_box_size = trust_box_size * cfg::trust_expand_ratio;
				X = Xopt; U = Uopt; *delta = deltaopt;
				break;
			}

			if (trust_box_size < cfg::min_trust_box_size) {
				printf("Converged x tolerance\n");
				return success;
			}

		} // end trust_region loop

	} // end second loop

	return success;

}

bool penalty_sqp(StdVectorX& X, StdVectorU& U, double* delta, bounds_t bounds,
		double_integrator_QP_solver_noCD_params& problem, double_integrator_QP_solver_noCD_output& output, double_integrator_QP_solver_noCD_info& info) {
	double penalty_coeff = cfg::initial_penalty_coeff;
	int penalty_increases = 0;

	bool success = true;

	while(penalty_increases < cfg::max_penalty_coeff_increases) {
		success = minimize_merit_function(X, U, delta, bounds, penalty_coeff, problem, output, info);

		if (!success) {
			printf("Merit function not minimized successfully\n");
			break;
		}

		double constraint_violation = (computeMerit(delta, X, U, penalty_coeff) - computeObjective(delta, X, U))/penalty_coeff;
		printf("Constraint violation: %.5f\n", constraint_violation);

		if (constraint_violation <= cfg::cnt_tolerance) {
			break;
		} else {
			penalty_increases++;
			penalty_coeff *= cfg::penalty_coeff_increase_ratio;
			if (penalty_increases == cfg::max_penalty_coeff_increases) {
				printf("Number of penalty coefficient increases exceeded maximum allowed.\n");
				success = false;
				break;
			}
		}
	}

	return success;
}

/* Wrapper function to solve BVP. This will be called from RRT* code.
 * 1 is returned on success, 0 on failure.
 *
 * X, U are empty, with length T, T-1, respectively.
 * delta is a pointer that should be null. I have these passed in so
 * the caller of this code can have access to the solution of the BVP.
 */
int solve_double_integrator_noCD_BVP(VectorX x_start, VectorX x_goal, StdVectorX& X, StdVectorU& U, double* delta, bounds_t bounds) {

	double_integrator_QP_solver_noCD_params problem;
	double_integrator_QP_solver_noCD_output output;
	double_integrator_QP_solver_noCD_info info;
	setup_state_vars(problem, output);

	*delta = 1;

	// Initialize X variable
	Matrix<double, X_DIM, T> init;
	for(int i = 0; i < X_DIM; ++i) {
		init.row(i).setLinSpaced(T, x_start(i), x_goal(i));
	}

	for(int t = 0; t < T; ++t) {
		X[t] = init.col(t);
	}

	// Initialize U variable
	for(int t = 0; t < T-1; ++t) {
		U[t] = MatrixXd::Zero(U_DIM, 1);
	}

	bool success = penalty_sqp(X, U, delta, bounds, problem, output, info);

	cleanup_state_vars();

	if (success) {
		return 1;
	} else {
		return 0;
	}

}

/*
 *  This function serves as a testing function for the methods written above
 */
int main() {

	double_integrator_QP_solver_noCD_params problem;
	double_integrator_QP_solver_noCD_output output;
	double_integrator_QP_solver_noCD_info info;
	setup_state_vars(problem, output);

	StdVectorX X(T);
	StdVectorU U(T-1);
	double *delta; double delta_init = 1;
	delta = &delta_init;

	// Start and goal state
	Vector4d x_start, x_goal;
	x_start << 9.78713, 0.442322, -0.913086, -0.850323;
	x_goal << 9.29103, 9.45342, -0.852262, -0.86858;

	// Initialize X variable
	Matrix<double, X_DIM, T> init;
	for(int i = 0; i < X_DIM; ++i) {
		init.row(i).setLinSpaced(T, x_start(i), x_goal(i));
	}

//	std::cout << "init:\n" << init << "\n";

	for(int t = 0; t < T; ++t) {
		X[t] = init.col(t);
	}

	// Initialize U variable
	for(int t = 0; t < T-1; ++t) {
		U[t] = .5*MatrixXd::Zero(U_DIM, 1);
	}

	// Init bounds
	bounds_t bounds;

	bounds.u_max = 1;
	bounds.u_min = -1;
	bounds.x_max = 10;
	bounds.x_min = -10;
	bounds.v_max = 1;
	bounds.v_min = -1;
	bounds.x_goal = x_goal;
	bounds.x_start = x_start;

	bool success = penalty_sqp(X, U, delta, bounds, problem, output, info);

	if (success) {
		printf("Success!!");
	}

	cleanup_state_vars();
}
