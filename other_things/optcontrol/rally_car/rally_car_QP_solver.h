/*
FORCES - Fast interior point code generation for multistage problems.
Copyright (C) 2011-14 Alexander Domahidi [domahidi@control.ee.ethz.ch],
Automatic Control Laboratory, ETH Zurich.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __rally_car_QP_solver_H__
#define __rally_car_QP_solver_H__


/* DATA TYPE ------------------------------------------------------------*/
typedef double rally_car_QP_solver_FLOAT;


/* SOLVER SETTINGS ------------------------------------------------------*/
/* print level */
#ifndef rally_car_QP_solver_SET_PRINTLEVEL
#define rally_car_QP_solver_SET_PRINTLEVEL    (0)
#endif

/* timing */
#ifndef rally_car_QP_solver_SET_TIMING
#define rally_car_QP_solver_SET_TIMING    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define rally_car_QP_solver_SET_MAXIT         (50)	

/* scaling factor of line search (affine direction) */
#define rally_car_QP_solver_SET_LS_SCALE_AFF  (0.9)      

/* scaling factor of line search (combined direction) */
#define rally_car_QP_solver_SET_LS_SCALE      (0.95)  

/* minimum required step size in each iteration */
#define rally_car_QP_solver_SET_LS_MINSTEP    (1E-08)

/* maximum step size (combined direction) */
#define rally_car_QP_solver_SET_LS_MAXSTEP    (0.995)

/* desired relative duality gap */
#define rally_car_QP_solver_SET_ACC_RDGAP     (0.0001)

/* desired maximum residual on equality constraints */
#define rally_car_QP_solver_SET_ACC_RESEQ     (1E-06)

/* desired maximum residual on inequality constraints */
#define rally_car_QP_solver_SET_ACC_RESINEQ   (1E-06)

/* desired maximum violation of complementarity */
#define rally_car_QP_solver_SET_ACC_KKTCOMPL  (1E-06)


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define rally_car_QP_solver_OPTIMAL      (1)

/* maximum number of iterations has been reached */
#define rally_car_QP_solver_MAXITREACHED (0)

/* no progress in line search possible */
#define rally_car_QP_solver_NOPROGRESS   (-7)




/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct rally_car_QP_solver_params
{
    /* vector of size 28 */
    rally_car_QP_solver_FLOAT f1[28];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT lb1[28];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT ub1[12];

    /* matrix of size [17 x 28] (column major format) */
    rally_car_QP_solver_FLOAT C1[476];

    /* vector of size 17 */
    rally_car_QP_solver_FLOAT e1[17];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT f2[28];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT lb2[28];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT ub2[12];

    /* matrix of size [9 x 28] (column major format) */
    rally_car_QP_solver_FLOAT C2[252];

    /* vector of size 9 */
    rally_car_QP_solver_FLOAT e2[9];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT f3[28];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT lb3[28];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT ub3[12];

    /* matrix of size [9 x 28] (column major format) */
    rally_car_QP_solver_FLOAT C3[252];

    /* vector of size 9 */
    rally_car_QP_solver_FLOAT e3[9];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT f4[28];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT lb4[28];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT ub4[12];

    /* matrix of size [9 x 28] (column major format) */
    rally_car_QP_solver_FLOAT C4[252];

    /* vector of size 9 */
    rally_car_QP_solver_FLOAT e4[9];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT f5[28];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT lb5[28];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT ub5[12];

    /* matrix of size [9 x 28] (column major format) */
    rally_car_QP_solver_FLOAT C5[252];

    /* vector of size 9 */
    rally_car_QP_solver_FLOAT e5[9];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT f6[28];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT lb6[28];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT ub6[12];

    /* matrix of size [9 x 28] (column major format) */
    rally_car_QP_solver_FLOAT C6[252];

    /* vector of size 9 */
    rally_car_QP_solver_FLOAT e6[9];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT f7[28];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT lb7[28];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT ub7[12];

    /* matrix of size [9 x 28] (column major format) */
    rally_car_QP_solver_FLOAT C7[252];

    /* vector of size 9 */
    rally_car_QP_solver_FLOAT e7[9];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT f8[28];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT lb8[28];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT ub8[12];

    /* matrix of size [9 x 28] (column major format) */
    rally_car_QP_solver_FLOAT C8[252];

    /* vector of size 9 */
    rally_car_QP_solver_FLOAT e8[9];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT f9[28];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT lb9[28];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT ub9[12];

    /* matrix of size [9 x 28] (column major format) */
    rally_car_QP_solver_FLOAT C9[252];

    /* vector of size 9 */
    rally_car_QP_solver_FLOAT e9[9];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT f10[28];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT lb10[28];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT ub10[12];

    /* matrix of size [9 x 28] (column major format) */
    rally_car_QP_solver_FLOAT C10[252];

    /* vector of size 9 */
    rally_car_QP_solver_FLOAT e10[9];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT f11[28];

    /* vector of size 28 */
    rally_car_QP_solver_FLOAT lb11[28];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT ub11[12];

    /* matrix of size [9 x 28] (column major format) */
    rally_car_QP_solver_FLOAT C11[252];

    /* vector of size 9 */
    rally_car_QP_solver_FLOAT e11[9];

    /* vector of size 9 */
    rally_car_QP_solver_FLOAT lb12[9];

    /* vector of size 9 */
    rally_car_QP_solver_FLOAT ub12[9];

} rally_car_QP_solver_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct rally_car_QP_solver_output
{
    /* vector of size 12 */
    rally_car_QP_solver_FLOAT z1[12];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT z2[12];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT z3[12];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT z4[12];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT z5[12];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT z6[12];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT z7[12];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT z8[12];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT z9[12];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT z10[12];

    /* vector of size 12 */
    rally_car_QP_solver_FLOAT z11[12];

    /* vector of size 9 */
    rally_car_QP_solver_FLOAT z12[9];

} rally_car_QP_solver_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct rally_car_QP_solver_info
{
    /* iteration number */
    int it;
	
    /* inf-norm of equality constraint residuals */
    rally_car_QP_solver_FLOAT res_eq;
	
    /* inf-norm of inequality constraint residuals */
    rally_car_QP_solver_FLOAT res_ineq;

    /* primal objective */
    rally_car_QP_solver_FLOAT pobj;	
	
    /* dual objective */
    rally_car_QP_solver_FLOAT dobj;	

    /* duality gap := pobj - dobj */
    rally_car_QP_solver_FLOAT dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    rally_car_QP_solver_FLOAT rdgap;		

    /* duality measure */
    rally_car_QP_solver_FLOAT mu;

	/* duality measure (after affine step) */
    rally_car_QP_solver_FLOAT mu_aff;
	
    /* centering parameter */
    rally_car_QP_solver_FLOAT sigma;
	
    /* number of backtracking line search steps (affine direction) */
    int lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    int lsit_cc;
    
    /* step size (affine direction) */
    rally_car_QP_solver_FLOAT step_aff;
    
    /* step size (combined direction) */
    rally_car_QP_solver_FLOAT step_cc;    

	/* solvertime */
	rally_car_QP_solver_FLOAT solvetime;   

} rally_car_QP_solver_info;


/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* examine exitflag before using the result! */
int rally_car_QP_solver_solve(rally_car_QP_solver_params* params, rally_car_QP_solver_output* output, rally_car_QP_solver_info* info);


#endif