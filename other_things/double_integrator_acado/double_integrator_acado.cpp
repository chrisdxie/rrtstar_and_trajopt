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

int main() {

    double start_x = 0.0;
    double start_y = 0.0;
	double end_x = 3.5;
	double end_y = 1.5;
    double d_safe = 0.05;
	int num_discretization = 20;

    USING_NAMESPACE_ACADO

    //---------------------------------------------------------------------

    DifferentialState 			x, y, xdot, ydot; // Differential States
    Control						ux, uy			; // Control variables
    Parameter					T		 		; // the time horizon
    DifferentialEquation		f( 0.0, T )		; // Differential dynamics

    //---------------------------------------------------------------------

    OCP ocp( 0.0, T , num_discretization)		; // time horizon of the optimal control problem is [0, T]
    ocp.minimizeMayerTerm( T )					; // Minimize T

    //---------------------------------------------------------------------

    f << dot(x) == xdot							;
    f << dot(y) == ydot							;
    f << dot(xdot) == ux						;
    f << dot(ydot) == uy						;

    //---------------------------------------------------------------------

    IntermediateState z(7);
    z(0) = x; z(1) = y; z(2) = xdot; z(3) = ydot; z(4) = ux; z(5) = uy; z(6) = T;

    CFunction SD( 1, signedDistanceFunction);
    CFunction SWVSD( 1, sweptThroughVolumeSignedDistanceFunction);

    //---------------------------------------------------------------------

    ocp.subjectTo( f )							;

    ocp.subjectTo( AT_START, x == start_x )		;
    ocp.subjectTo( AT_START, y == start_y )		;
    ocp.subjectTo( AT_START, xdot == 0.0 )		;
    ocp.subjectTo( AT_START, ydot == 0.0 )		;

    ocp.subjectTo( AT_END, x == end_x )			;
    ocp.subjectTo( AT_END, y == end_y )			; 
    ocp.subjectTo( AT_END, xdot == 0.0 )		;
    ocp.subjectTo( AT_END, ydot == 0.0 )		;

    // Obstacles
    ocp.subjectTo( SD(z) >= d_safe );
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

    return 0									;

}		