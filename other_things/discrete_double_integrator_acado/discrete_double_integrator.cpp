/**
 *	Compute the controls for a 2D double integrator model
 *	from start state 0,0,0,0 to some goal state.
 */

#include <acado_toolkit.hpp>
#include <include/acado_gnuplot/gnuplot_window.hpp>

#include <eigen3/Eigen/Eigen>
#include "../2d_signed_distance_library_cpp/signedDistancePolygons.hpp"

// collision detection function
void signedDistanceFunction( double *z, double *f, void *user_data ) {

    double x = z[0];
    double y = z[1];

    Matrix<double, 2, 2> point_robot;
    point_robot << x, y, x, y;

    // Hard coded obstacle
    Matrix<double, 4, 2> obs;
    obs << 1, 2,
           1, 1,
           3, 1,
           3, 2;

    Matrix<double, 3, 2> ans = signed_distance_2d::signedDistancePolygons(point_robot, obs);
    f[0] = ans(0,0);

}

void sweptThroughVolumeSignedDistanceFunction( double *z, double *f, void *user_data ) {

    double x = z[0];
    double y = z[1];
    double xdot = z[2];
    double ydot = z[3];
    double ux = z[4];
    double uy = z[5];
    double T = z[6];

    double x_next = x + T*xdot + .5*ux*T*T;
    double y_next = y + T*ydot + .5*uy*T*T;

    // std::cout << "Current " << x << " " << y << std::endl;
    // std::cout << "Next: " << x_next << " " << y_next << std::endl; 

    Matrix<double, 2, 2> traveling_point_robot;
    traveling_point_robot << x, y, x_next, y_next;

    // Hard coded obstacle
    Matrix<double, 4, 2> obs;
    obs << 1, 2,
           1, 1,
           3, 1,
           3, 2;

    Matrix<double, 3, 2> ans = signed_distance_2d::signedDistancePolygons(traveling_point_robot, obs);
    f[0] = ans(0,0);

    //std::cout << "Signed distance: " << f[0] << std::endl;

}

// RK4 integration
void double_integrator_dynamics( double *z, double *f, void *user_data ) {

    // Double integrator dynamics, 2d
    int num_states = 4;
    int num_controls = 2;
    MatrixXd A(num_states, num_states);
    MatrixXd B(num_states, num_controls);
    Vector4d x_t(z[0], z[1], z[2], z[3]);
    Vector2d u_t(z[4], z[5]);
    double t = z[6];
    //double t = ((double*) user_data)[0];

    A.topRightCorner(num_states/2, num_states/2).setIdentity();
    B.bottomRows(num_controls).setIdentity();

    Vector4d k1 = A * x_t + B * u_t;
    Vector4d k2 = A * (x_t + (k1.array()/2.0).matrix()) + B * u_t;
    Vector4d k3 = A * (x_t + (k2.array()/2.0).matrix()) + B * u_t;
    Vector4d k4 = A * (x_t + k3) + B * u_t;

    Vector4d x_new = (x_t.array() + t/6 * (k1.array() + 2*k2.array() + 2*k3.array() + k4.array())).matrix();

    f[0] = x_new(0);
    f[1] = x_new(1);
    f[2] = x_new(2);
    f[3] = x_new(3);

}

int main() {

    double start_x = 0.0;
    double start_y = 0.0;
	double end_x = 4;
	double end_y = 4; // Weird, [4, 2] doesn't work. [4,4] or [4,0] works just fine w/out obstacles.
    double d_safe = 0.05;
	int num_discretization = 20;

    double t_start = 0.0;
    double t_end = 1.0;
    const double t_step = (t_end - t_start)/num_discretization;

    USING_NAMESPACE_ACADO

    //---------------------------------------------------------------------

    DifferentialState 			x, y, xdot, ydot; // Differential States
    Control						ux, uy			; // Control variables
    Parameter					T		 		; // the time horizon
    DiscretizedDifferentialEquation		f( t_step )		; // Differential dynamics

    //---------------------------------------------------------------------

    CFunction SD( 1, signedDistanceFunction);
    //CFunction F( 4, double_integrator_dynamics);
    //CFunction SWVSD( 1, sweptThroughVolumeSignedDistanceFunction);

    //F.setUserData((void*) &t_step);

    //---------------------------------------------------------------------

    IntermediateState z(7);
    z(0) = x; z(1) = y; z(2) = xdot; z(3) = ydot; z(4) = ux; z(5) = uy; z(6) = T;
    //IntermediateState z(4);
    //z(0) = x; z(1) = y; z(2) = next(x); z(3) = next(y);

    f << next(x) == x + T*xdot + .5*ux*T*T;
    f << next(y) == y + T*ydot + .5*uy*T*T;
    f << next(xdot) == xdot + T*ux;
    f << next(ydot) == ydot + T*uy;

    //---------------------------------------------------------------------

    //---------------------------------------------------------------------

    OCP ocp( t_start, t_end, num_discretization)        ; // time horizon of the optimal control problem is [0, T]
    ocp.minimizeMayerTerm( T )                    ; // Minimize T

    ocp.subjectTo( f )					;

    ocp.subjectTo( AT_START, x == start_x )		;
    ocp.subjectTo( AT_START, y == start_y )		;
    ocp.subjectTo( AT_START, xdot == 0.0 )		;
    ocp.subjectTo( AT_START, ydot == 0.0 )		;

    ocp.subjectTo( AT_END, x == end_x )			;
    ocp.subjectTo( AT_END, y == end_y )			; 
    ocp.subjectTo( AT_END, xdot == 0.0 )		;
    ocp.subjectTo( AT_END, ydot == 0.0 )		;

    // Obstacles
    //ocp.subjectTo( SD(z) >= d_safe );
    //ocp.subjectTo( SWVSD(z) >= d_safe );

    ocp.subjectTo( -1 <= ux <= 1 )				;
    ocp.subjectTo( -1 <= uy <= 1 )				;

    ocp.subjectTo( -1 <= xdot <= 1 )			;
    ocp.subjectTo( -1 <= ydot <= 1 )			;
      
    ocp.subjectTo( 0 <= T )			;

    //---------------------------------------------------------------------

    // GnuplotWindow w1(PLOT_AT_START) 	;
    // w1.addSubplot( x, y, "Position" , "", "", PM_POINTS)			;
    // w1.addSubplot( xdot, "Velocity x" )			;
    // w1.addSubplot( ydot, "Velocity y" )			;
    // w1.addSubplot( ux, "Control for x" )			;
    // w1.addSubplot( uy, "Control for y" )			;

    GnuplotWindow w2(PLOT_AT_EACH_ITERATION)  ;
    w2.addSubplot( x, y, "Position" , "", "", PM_POINTS)         ;
    w2.addSubplot( xdot, "Velocity x" )          ;
    w2.addSubplot( ydot, "Velocity y" )            ;
    w2.addSubplot( ux, "Control for x" )         ;
    w2.addSubplot( uy, "Control for y" )           ;

    //---------------------------------------------------------------------

    // Initialize time to something, otherwise things get wacky
    double time_init = 20.00e+00;
    VariablesGrid T_init(1, 0, 1, num_discretization+1);
    for (int i = 0; i <= num_discretization; i++) {
        T_init(i, 0) = time_init;
    }

    // Initialize to straight line with 0 velocities
    VariablesGrid state_init(4, 0, 1, num_discretization+1);
    state_init.setZero();
    for (int i = 0; i <= num_discretization; i++) {
        state_init(i, 0) = ((float) i)/num_discretization * (end_x - start_x) + start_x;
    }
    for (int i = 0; i <= num_discretization; i++) {
        state_init(i, 1) = ((float) i)/num_discretization * (end_y - start_y) + start_y;
    }


     // T_init.print();
     // state_init.print();

    //---------------------------------------------------------------------

    OptimizationAlgorithm algorithm( ocp )		;
    //algorithm << w1 							;
    algorithm << w2                             ;
    algorithm.initializeParameters(T_init)		;
    algorithm.initializeDifferentialStates(state_init);
    //algorithm.set(HESSIAN_APPROXIMATION, FULL_BFGS_UPDATE);
    algorithm.set(HESSIAN_APPROXIMATION, BLOCK_BFGS_UPDATE);    // default
    //algorithm.set(HESSIAN_APPROXIMATION, EXACT_HESSIAN);
    algorithm.solve()							;

    VariablesGrid states, parameters;
    algorithm.getDifferentialStates(states);
    algorithm.getParameters(parameters);
    states.print();
    parameters.print();

    // MUST MULTIPLY PARAMETERS (WHICH IS TIME VALUE) BY NUM_DISCRETIZATION TO GET MINIMUM TIME

    return 0									;

}		