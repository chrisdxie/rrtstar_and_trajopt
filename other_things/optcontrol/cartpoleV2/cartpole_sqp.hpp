#ifndef CARTPOLE_SQP_HPP_
#define CARTPOLE_SQP_HPP_

extern "C" {
#include "cartpole_QP_solver.h"
cartpole_QP_solver_FLOAT **f, **lb, **ub, **C, **e, **z;
}

#include <iostream>
#include <vector>

#include "../util/logging.h"

#include "../../dynamics_library/dynamics_library.hpp"
using namespace dynamics_library;

#define INFTY 1e10

#define TIMESTEPS 12
const int T = TIMESTEPS;

#define X_DIM 4
#define U_DIM 1
#define TOTAL_VARS

#include <eigen3/Eigen/Eigen>
using namespace Eigen;

#include "boost/preprocessor.hpp"

typedef Matrix<double, X_DIM, 1> VectorX;
typedef Matrix<double, U_DIM, 1> VectorU;

typedef std::vector<VectorX> StdVectorX;
typedef std::vector<VectorU> StdVectorU;

namespace cfg {
const double improve_ratio_threshold = .25; // .25
const double min_approx_improve = 1e-2;
const double min_trust_box_size = 1e-4;
const double trust_shrink_ratio = .5; // .1
const double trust_expand_ratio = 1.25; // 1.5
const double cnt_tolerance = 1e-5;
const double penalty_coeff_increase_ratio = 10;
const double initial_penalty_coeff = 10;
const double initial_trust_box_size = 0.5;
const int max_penalty_coeff_increases = 5;
const int max_sqp_iterations = 50;
}

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

Matrix<double, X_DIM,X_DIM> I = Matrix<double,X_DIM,X_DIM>::Identity(X_DIM,X_DIM);
Matrix<double, X_DIM,X_DIM> minusI = -I;

struct bounds_t {
	double delta_min, delta_max;
	VectorX x_min, x_max;
	VectorU u_min, u_max;
	VectorX x_start;
	VectorX x_goal;
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

void setup_state_vars(cartpole_QP_solver_params& problem, cartpole_QP_solver_output& output)
{

	/* Initialize problem inputs and outputs as double arrays */
	// problem inputs
	f = new cartpole_QP_solver_FLOAT*[T-1];
	lb = new cartpole_QP_solver_FLOAT*[T];
	ub = new cartpole_QP_solver_FLOAT*[T];
	C = new cartpole_QP_solver_FLOAT*[T-1];
	e = new cartpole_QP_solver_FLOAT*[T-1];

	// problem outputs
	z = new cartpole_QP_solver_FLOAT*[T];

	/* Link them via boost to something, IDK how this works */
#define SET_VARS(n) \
		lb[ BOOST_PP_SUB(n,1) ] = problem.lb##n ; \
		ub[ BOOST_PP_SUB(n,1) ] = problem.ub##n ; \
		z[ BOOST_PP_SUB(n,1) ] = output.z##n ;
#define BOOST_PP_LOCAL_MACRO(n) SET_VARS(n)
#define BOOST_PP_LOCAL_LIMITS (1, TIMESTEPS)
#include BOOST_PP_LOCAL_ITERATE()

#define SET_TMINUSONE_VARS(n) \
		f[ BOOST_PP_SUB(n,1) ] = problem.f##n ; \
		C[ BOOST_PP_SUB(n,1) ] = problem.C##n ; \
		e[ BOOST_PP_SUB(n,1) ] = problem.e##n ;
#define BOOST_PP_LOCAL_MACRO(n) SET_TMINUSONE_VARS(n)
#define BOOST_PP_LOCAL_LIMITS (1, TIMESTEPS-1)
#include BOOST_PP_LOCAL_ITERATE()

	// Initalize everything to infinity

	for(int t = 0; t < T-1; ++t) {
		fill_col_major(f[t], INFTY*Matrix<double,3*X_DIM+U_DIM+1,1>::Ones());
	}

	for(int t = 0; t < T-1; ++t) {
		fill_col_major(lb[t], INFTY*Matrix<double,3*X_DIM+U_DIM+1,1>::Ones());
		fill_col_major(ub[t], INFTY*Matrix<double,X_DIM+U_DIM+1,1>::Ones());
	}
	fill_col_major(lb[T-1], INFTY*Matrix<double,X_DIM+1,1>::Ones());
	fill_col_major(ub[T-1], INFTY*Matrix<double,X_DIM+1,1>::Ones());

	fill_col_major(C[0], INFTY*Matrix<double,2*X_DIM+1,3*X_DIM+U_DIM+1>::Ones());
	fill_col_major(e[0], INFTY*Matrix<double,2*X_DIM+1,1>::Ones());
	for(int t = 1; t < T-1; ++t) {
		fill_col_major(C[t], INFTY*Matrix<double,X_DIM+1,3*X_DIM+U_DIM+1>::Ones());
		fill_col_major(e[t], INFTY*Matrix<double,X_DIM+1,1>::Ones());
	}

	for(int t = 0; t < T-1; ++t) {
		fill_col_major(z[t], INFTY*Matrix<double, X_DIM+U_DIM+1, 1>::Ones());
	}
	fill_col_major(z[T-1], INFTY*Matrix<double, X_DIM+1, 1>::Ones());

}

void cleanup_state_vars() {
	delete[] f;
	delete[] lb;
	delete[] ub;
	delete[] C;
	delete[] e;
	delete[] z;
}

bool is_valid_inputs() {
	// Check if any of the values have not been touched, i.e. they are still infinity.

	for(int t = 0; t < T-1; ++t) {
		for(int i = 0; i < 3*X_DIM+U_DIM+1; ++i) {
			if (f[t][i] == INFTY) {return false;}
		}
	}

	for(int t = 0; t < T-1; ++t) {
		for(int i = 0; i < 3*X_DIM+U_DIM+1; ++i) {
			if (lb[t][i] == INFTY) {return false;}
		}
		for(int i = 0; i < X_DIM+U_DIM+1; ++i) {
			if (ub[t][i] == INFTY) {return false;}
		}
	}
	for(int i = 0; i < X_DIM+1; ++i) {
		if (lb[T-1][i] == INFTY) {return false;}
	}
	for(int i = 0; i < X_DIM+1; ++i) {
		if (ub[T-1][i] == INFTY) {return false;}
	}


	for (int i = 0; i < (2*X_DIM+1)*(3*X_DIM+U_DIM+1); ++i) {
		if (C[0][i] == INFTY) {return false;}
	}
	for (int i = 0; i < 2*X_DIM+1; ++i) {
		if (e[0][i] == INFTY) {return false;}
	}
	for(int t = 1; t < T-1; ++t) {
		for (int i = 0; i < (X_DIM+1)*(3*X_DIM+U_DIM+1); ++i) {
			if (C[t][i] == INFTY) {return false;}
		}
		for (int i = 0; i < X_DIM+1; ++i) {
			if (e[t][i] == INFTY) {return false;}
		}
	}

	// Inputs are valid!
	return true;
}

// Fill in f using penalty coefficient
void fill_f(double penalty_coeff)
{
	VectorXd f_temp(3*X_DIM+U_DIM+1);
	for(int t = 0; t < T-1; ++t) {
		for(int i = 0; i < 3*X_DIM+U_DIM+1; ++i) {
			if (t==0 && i == X_DIM) { f_temp(i) = 1;}
			else if (i >= X_DIM+U_DIM+1) { f_temp(i) = penalty_coeff; }
			else { f_temp(i) = 0;}
		}
		fill_col_major(f[t], f_temp);
	}
}

// Fill in lower bounds and upper bounds
void fill_lb_and_ub(StdVectorX& X, StdVectorU& U, double& delta, double trust_box_size, bounds_t bounds)
{
	VectorXd lb_temp(3*X_DIM+U_DIM+1);
	VectorXd ub_temp(X_DIM+U_DIM+1);
	for(int t = 0; t < T-1; ++t)
	{
		VectorX& xt = X[t];
		VectorU& ut = U[t];

		for(int i = 0; i < 3*X_DIM+U_DIM+1; ++i) {
			if (i < X_DIM) {
				lb_temp(i) = MAX(bounds.x_min(i), xt[i] - trust_box_size);
				ub_temp(i) = MIN(bounds.x_max(i), xt[i] + trust_box_size);
			}
			else if (i == X_DIM) {
				lb_temp(i) = MAX(bounds.delta_min, delta - trust_box_size);
				ub_temp(i) = MIN(bounds.delta_max, delta + trust_box_size);
			}
			else if (i > X_DIM && i < X_DIM+U_DIM+1) {
				lb_temp(i) = MAX(bounds.u_min(i-X_DIM-1), ut[i-X_DIM-1] - trust_box_size);
				ub_temp(i) = MIN(bounds.u_max(i-X_DIM-1), ut[i-X_DIM-1] + trust_box_size);
			}
			else { lb_temp(i) = 0; }
		}
		fill_col_major(lb[t], lb_temp);
		fill_col_major(ub[t], ub_temp);
	}

	VectorX& xT = X[T-1];

	double eps = 1e-10;

	VectorXd lbT_temp(X_DIM+1);
	VectorXd ubT_temp(X_DIM+1);
	for(int i = 0; i < X_DIM; ++i) {
		lbT_temp(i) = MAX(bounds.x_goal(i) - eps, xT[i] - trust_box_size);
		ubT_temp(i) = MIN(bounds.x_goal(i) + eps, xT[i] + trust_box_size);
	}
	lbT_temp(X_DIM) = MAX(bounds.delta_min, delta - trust_box_size);
	ubT_temp(X_DIM) = MIN(bounds.delta_max, delta + trust_box_size);

	fill_col_major(lb[T-1], lbT_temp);
	fill_col_major(ub[T-1], ubT_temp);
}

// Fill in C and e by linearizing around point X, U, delta
void fill_in_C_and_e(StdVectorX& X, StdVectorU& U, double& delta, double trust_box_size, bounds_t bounds)
{
	VectorX& x0 = X[0];
	VectorU& u0 = U[0];

	VectorX xt1;
	Matrix<double, X_DIM, X_DIM+U_DIM+1> jac = numerical_jacobian(continuous_cartpole_dynamics, x0, u0, delta);
	Matrix<double, X_DIM, X_DIM> DH_X = jac.leftCols(X_DIM);
	Matrix<double, X_DIM, U_DIM> DH_U = jac.middleCols(X_DIM, U_DIM);
	Matrix<double, X_DIM, 1> DH_delta = jac.rightCols(1);

	Matrix<double, 2*X_DIM+1,3*X_DIM+U_DIM+1> C0_temp;
	Matrix<double, 2*X_DIM+1, 1> e0_temp;

	C0_temp.setZero();

	C0_temp.block<X_DIM,X_DIM>(0,0) = I;

	C0_temp.block<X_DIM,X_DIM>(X_DIM,0) = DH_X;
	C0_temp.block<X_DIM,1>(X_DIM,X_DIM) = DH_delta;
	C0_temp.block<X_DIM,U_DIM>(X_DIM,X_DIM+1) = DH_U;
	C0_temp.block<X_DIM,X_DIM>(X_DIM,X_DIM+1+U_DIM) = I;
	C0_temp.block<X_DIM,X_DIM>(X_DIM,X_DIM+1+U_DIM+X_DIM) = minusI;
	C0_temp(2*X_DIM,X_DIM) = 1;

	fill_col_major(C[0], C0_temp);

	xt1 = rk4(continuous_cartpole_dynamics, x0, u0, delta);

	e0_temp.setZero();
	e0_temp.head(X_DIM) = bounds.x_start;
	e0_temp.block<X_DIM,1>(X_DIM,0) = -xt1 + DH_X*x0 + DH_U*u0 + delta*DH_delta;

	fill_col_major(e[0], e0_temp);

	Matrix<double, X_DIM+1,3*X_DIM+U_DIM+1> Ct_temp;
	Matrix<double, X_DIM+1, 1> et_temp;

	for(int t = 1; t < T-1; ++t)
	{
		VectorX& xt = X[t];
		VectorU& ut = U[t];

		xt1 = rk4(continuous_cartpole_dynamics, xt, ut, delta);
		jac = numerical_jacobian(continuous_cartpole_dynamics, xt, ut, delta);
		DH_X = jac.leftCols(X_DIM);
		DH_U = jac.middleCols(X_DIM, U_DIM);
		DH_delta = jac.rightCols(1);

		Ct_temp.setZero();

		Ct_temp.block<X_DIM,X_DIM>(0,0) = DH_X;
		Ct_temp.block<X_DIM,1>(0,X_DIM) = DH_delta;
		Ct_temp.block<X_DIM,U_DIM>(0,X_DIM+1) = DH_U;
		Ct_temp.block<X_DIM,X_DIM>(0,X_DIM+1+U_DIM) = I;
		Ct_temp.block<X_DIM,X_DIM>(0,X_DIM+1+U_DIM+X_DIM) = minusI;
		Ct_temp(X_DIM,X_DIM) = 1;

		//std::cout << "DH_X:\n" << DH_X << std::endl;
		//std::cout << "DH_U:\n" << DH_U << std::endl;
		//std::cout << "DH_delta:\n" << DH_delta << std::endl;

		//std::cout << Ct_temp.block<X_DIM+1,3*X_DIM+U_DIM+1>(0,0) << std::endl;
		//int num;
		//std::cin >> num;

		fill_col_major(C[t], Ct_temp);

		et_temp.setZero();
		et_temp.head(X_DIM) = -xt1 + DH_X*xt + DH_U*ut + delta*DH_delta;
		fill_col_major(e[t], et_temp);
	}

}

double computeObjective(double& delta, StdVectorX& X, StdVectorU& U) {
	return (delta);
}

double computeMerit(double& delta, StdVectorX& X, StdVectorU& U, double penalty_coeff) {
	double merit = computeObjective(delta, X, U);
	for(int t = 0; t < T-1; ++t) {
		VectorXd hval = dynamics_difference(continuous_cartpole_dynamics, X[t], X[t+1], U[t], delta);
		merit += penalty_coeff*(hval.cwiseAbs()).sum();
	}
	return merit;
}


bool minimize_merit_function(StdVectorX& X, StdVectorU& U, double& delta, bounds_t bounds, double penalty_coeff,
		cartpole_QP_solver_params& problem, cartpole_QP_solver_output& output, cartpole_QP_solver_info& info) {

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

	// fill in f. Constant across all iterations because the penalty is constant until we break from this "loop"
	fill_f(penalty_coeff);

	for(int sqp_iter=0; true; ++sqp_iter) {
		LOG_INFO("  sqp iter: %d",sqp_iter);

		merit = computeMerit(delta, X, U, penalty_coeff);

		for(int trust_iter=0; true; ++trust_iter) {
			LOG_INFO("       trust region size: %f",trust_box_size);

			// fill in lb, ub
			fill_lb_and_ub(X, U, delta, trust_box_size, bounds);

			// fill in C, e
			fill_in_C_and_e(X, U, delta, trust_box_size, bounds);

			if (!is_valid_inputs()) {
				LOG_FATAL("ERROR: invalid inputs");
				exit(0);
			}

			// call FORCES
			int exitflag = cartpole_QP_solver_solve(&problem, &output, &info);
			if (exitflag == 1) {
				optcost = info.pobj;
				deltaopt = z[0][X_DIM]; // Hard coded, I know the index of this
				//std::cout << z[0][0] << " " << z[0][1] << " " << z[0][2] << " " << z[0][3] << " " << z[0][4] << " " << z[0][5] << " " << z[0][6] << std::endl;
				//std::cout << z[1][0] << " " << z[1][1] << " " << z[1][2] << " " << z[1][3] << " " << z[1][4] << " " << z[1][5] << " " << z[1][6] << std::endl;
				for(int t=0; t < T; ++t) {
					index = 0;
					for(int i=0; i < X_DIM; ++i) {
						Xopt[t](i) = z[t][index++];
					}
					index++;
					if (t < T-1) {
						for(int i=0; i < U_DIM; ++i) {
							Uopt[t](i) = z[t][index++];
						}
					}
				}
				//int num;
				//std::cin >> num;
			} else {
				LOG_ERROR("Some problem in solver");
				success = false;
				return success;
			}

			double model_merit = optcost;
			double new_merit = computeMerit(deltaopt, Xopt, Uopt, penalty_coeff);
			double approx_merit_improve = merit - model_merit;
			double exact_merit_improve = merit - new_merit;
			double merit_improve_ratio = exact_merit_improve/approx_merit_improve;

			LOG_INFO("       approx improve: %.3f. exact improve: %.3f. ratio: %.3f", approx_merit_improve, exact_merit_improve, merit_improve_ratio);
			if (approx_merit_improve < -1) {
				LOG_INFO("Approximate merit function got worse (%.3e).", approx_merit_improve);
				LOG_INFO("Either convexification is wrong to zeroth order, or you're in numerical trouble");
				success = false;
				return success;
			} else if (approx_merit_improve < cfg::min_approx_improve) {
				LOG_INFO("Converged: y tolerance");
				X = Xopt; U = Uopt; delta = deltaopt;
				return success;
			} else if ((exact_merit_improve < 0) || (merit_improve_ratio < cfg::improve_ratio_threshold)) {
				trust_box_size = trust_box_size * cfg::trust_shrink_ratio;
			} else {
				trust_box_size = trust_box_size * cfg::trust_expand_ratio;
				X = Xopt; U = Uopt; delta = deltaopt;
				break;
			}

			if (trust_box_size < cfg::min_trust_box_size) {
				LOG_INFO("Converged x tolerance\n");
				return success;
			}

		} // end trust_region loop

	} // end second loop

	return success;

}

bool penalty_sqp(StdVectorX& X, StdVectorU& U, double& delta, bounds_t bounds,
		cartpole_QP_solver_params& problem, cartpole_QP_solver_output& output, cartpole_QP_solver_info& info) {
	double penalty_coeff = cfg::initial_penalty_coeff;
	int penalty_increases = 0;

	bool success = true;

	while(penalty_increases < cfg::max_penalty_coeff_increases) {
		success = minimize_merit_function(X, U, delta, bounds, penalty_coeff, problem, output, info);

		if (!success) {
			LOG_ERROR("Merit function not minimized successfully\n");
			break;
		}

		double constraint_violation = (computeMerit(delta, X, U, penalty_coeff) - computeObjective(delta, X, U))/penalty_coeff;
		LOG_INFO("Constraint violation: %.5f\n", constraint_violation);

		if (constraint_violation <= cfg::cnt_tolerance) {
			break;
		} else {
			penalty_increases++;
			penalty_coeff *= cfg::penalty_coeff_increase_ratio;
			if (penalty_increases == cfg::max_penalty_coeff_increases) {
				LOG_ERROR("Number of penalty coefficient increases exceeded maximum allowed.\n");
				success = false;
				break;
			}
		}
	}

	return success;
}

int solve_cartpole_BVP(StdVectorX& X, StdVectorU& U, double& delta, bounds_t bounds) {

	cartpole_QP_solver_params problem;
	cartpole_QP_solver_output output;
	cartpole_QP_solver_info info;
	setup_state_vars(problem, output);

	// Smart initialization
	delta = std::min((bounds.x_start - bounds.x_goal).norm()/10, .5);

	//std::cout << "Initial delta: " << delta << "\n";

	// Initialize X variable
	Matrix<double, X_DIM, T> init;
	for(int i = 0; i < X_DIM; ++i) {
		init.row(i).setLinSpaced(T, bounds.x_start(i), bounds.x_goal(i));
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

#endif /* CARTPOLE_SQP_HPP_ */

