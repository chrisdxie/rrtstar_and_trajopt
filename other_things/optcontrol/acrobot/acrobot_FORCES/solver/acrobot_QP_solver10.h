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

#ifndef __acrobot_QP_solver_H__
#define __acrobot_QP_solver_H__


/* DATA TYPE ------------------------------------------------------------*/
typedef double acrobot_QP_solver_FLOAT;


/* SOLVER SETTINGS ------------------------------------------------------*/
/* print level */
#ifndef acrobot_QP_solver_SET_PRINTLEVEL
#define acrobot_QP_solver_SET_PRINTLEVEL    (0)
#endif

/* timing */
#ifndef acrobot_QP_solver_SET_TIMING
#define acrobot_QP_solver_SET_TIMING    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define acrobot_QP_solver_SET_MAXIT         (50)	

/* scaling factor of line search (affine direction) */
#define acrobot_QP_solver_SET_LS_SCALE_AFF  (0.9)      

/* scaling factor of line search (combined direction) */
#define acrobot_QP_solver_SET_LS_SCALE      (0.95)  

/* minimum required step size in each iteration */
#define acrobot_QP_solver_SET_LS_MINSTEP    (1E-08)

/* maximum step size (combined direction) */
#define acrobot_QP_solver_SET_LS_MAXSTEP    (0.995)

/* desired relative duality gap */
#define acrobot_QP_solver_SET_ACC_RDGAP     (0.0001)

/* desired maximum residual on equality constraints */
#define acrobot_QP_solver_SET_ACC_RESEQ     (1E-06)

/* desired maximum residual on inequality constraints */
#define acrobot_QP_solver_SET_ACC_RESINEQ   (1E-06)

/* desired maximum violation of complementarity */
#define acrobot_QP_solver_SET_ACC_KKTCOMPL  (1E-06)


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define acrobot_QP_solver_OPTIMAL      (1)

/* maximum number of iterations has been reached */
#define acrobot_QP_solver_MAXITREACHED (0)

/* no progress in line search possible */
#define acrobot_QP_solver_NOPROGRESS   (-7)




/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct acrobot_QP_solver_params
{
    /* vector of size 14 */
    acrobot_QP_solver_FLOAT f1[14];

    /* vector of size 14 */
    acrobot_QP_solver_FLOAT lb1[14];

    /* vector of size 6 */
    acrobot_QP_solver_FLOAT ub1[6];

    /* matrix of size [9 x 14] (column major format) */
    acrobot_QP_solver_FLOAT C1[126];

    /* vector of size 9 */
    acrobot_QP_solver_FLOAT e1[9];

    /* vector of size 14 */
    acrobot_QP_solver_FLOAT f2[14];

    /* vector of size 14 */
    acrobot_QP_solver_FLOAT lb2[14];

    /* vector of size 6 */
    acrobot_QP_solver_FLOAT ub2[6];

    /* matrix of size [5 x 14] (column major format) */
    acrobot_QP_solver_FLOAT C2[70];

    /* vector of size 5 */
    acrobot_QP_solver_FLOAT e2[5];

    /* vector of size 14 */
    acrobot_QP_solver_FLOAT f3[14];

    /* vector of size 14 */
    acrobot_QP_solver_FLOAT lb3[14];

    /* vector of size 6 */
    acrobot_QP_solver_FLOAT ub3[6];

    /* matrix of size [5 x 14] (column major format) */
    acrobot_QP_solver_FLOAT C3[70];

    /* vector of size 5 */
    acrobot_QP_solver_FLOAT e3[5];

    /* vector of size 14 */
    acrobot_QP_solver_FLOAT f4[14];

    /* vector of size 14 */
    acrobot_QP_solver_FLOAT lb4[14];

    /* vector of size 6 */
    acrobot_QP_solver_FLOAT ub4[6];

    /* matrix of size [5 x 14] (column major format) */
    acrobot_QP_solver_FLOAT C4[70];

    /* vector of size 5 */
    acrobot_QP_solver_FLOAT e4[5];

    /* vector of size 14 */
    acrobot_QP_solver_FLOAT f5[14];

    /* vector of size 14 */
    acrobot_QP_solver_FLOAT lb5[14];

    /* vector of size 6 */
    acrobot_QP_solver_FLOAT ub5[6];

    /* matrix of size [5 x 14] (column major format) */
    acrobot_QP_solver_FLOAT C5[70];

    /* vector of size 5 */
    acrobot_QP_solver_FLOAT e5[5];

    /* vector of size 14 */
    acrobot_QP_solver_FLOAT f6[14];

    /* vector of size 14 */
    acrobot_QP_solver_FLOAT lb6[14];

    /* vector of size 6 */
    acrobot_QP_solver_FLOAT ub6[6];

    /* matrix of size [5 x 14] (column major format) */
    acrobot_QP_solver_FLOAT C6[70];

    /* vector of size 5 */
    acrobot_QP_solver_FLOAT e6[5];

    /* vector of size 14 */
    acrobot_QP_solver_FLOAT f7[14];

    /* vector of size 14 */
    acrobot_QP_solver_FLOAT lb7[14];

    /* vector of size 6 */
    acrobot_QP_solver_FLOAT ub7[6];

    /* matrix of size [5 x 14] (column major format) */
    acrobot_QP_solver_FLOAT C7[70];

    /* vector of size 5 */
    acrobot_QP_solver_FLOAT e7[5];

    /* vector of size 14 */
    acrobot_QP_solver_FLOAT f8[14];

    /* vector of size 14 */
    acrobot_QP_solver_FLOAT lb8[14];

    /* vector of size 6 */
    acrobot_QP_solver_FLOAT ub8[6];

    /* matrix of size [5 x 14] (column major format) */
    acrobot_QP_solver_FLOAT C8[70];

    /* vector of size 5 */
    acrobot_QP_solver_FLOAT e8[5];

    /* vector of size 14 */
    acrobot_QP_solver_FLOAT f9[14];

    /* vector of size 14 */
    acrobot_QP_solver_FLOAT lb9[14];

    /* vector of size 6 */
    acrobot_QP_solver_FLOAT ub9[6];

    /* matrix of size [5 x 14] (column major format) */
    acrobot_QP_solver_FLOAT C9[70];

    /* vector of size 5 */
    acrobot_QP_solver_FLOAT e9[5];

    /* vector of size 5 */
    acrobot_QP_solver_FLOAT lb10[5];

    /* vector of size 5 */
    acrobot_QP_solver_FLOAT ub10[5];

} acrobot_QP_solver_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct acrobot_QP_solver_output
{
    /* vector of size 6 */
    acrobot_QP_solver_FLOAT z1[6];

    /* vector of size 6 */
    acrobot_QP_solver_FLOAT z2[6];

    /* vector of size 6 */
    acrobot_QP_solver_FLOAT z3[6];

    /* vector of size 6 */
    acrobot_QP_solver_FLOAT z4[6];

    /* vector of size 6 */
    acrobot_QP_solver_FLOAT z5[6];

    /* vector of size 6 */
    acrobot_QP_solver_FLOAT z6[6];

    /* vector of size 6 */
    acrobot_QP_solver_FLOAT z7[6];

    /* vector of size 6 */
    acrobot_QP_solver_FLOAT z8[6];

    /* vector of size 6 */
    acrobot_QP_solver_FLOAT z9[6];

    /* vector of size 5 */
    acrobot_QP_solver_FLOAT z10[5];

} acrobot_QP_solver_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct acrobot_QP_solver_info
{
    /* iteration number */
    int it;
	
    /* inf-norm of equality constraint residuals */
    acrobot_QP_solver_FLOAT res_eq;
	
    /* inf-norm of inequality constraint residuals */
    acrobot_QP_solver_FLOAT res_ineq;

    /* primal objective */
    acrobot_QP_solver_FLOAT pobj;	
	
    /* dual objective */
    acrobot_QP_solver_FLOAT dobj;	

    /* duality gap := pobj - dobj */
    acrobot_QP_solver_FLOAT dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    acrobot_QP_solver_FLOAT rdgap;		

    /* duality measure */
    acrobot_QP_solver_FLOAT mu;

	/* duality measure (after affine step) */
    acrobot_QP_solver_FLOAT mu_aff;
	
    /* centering parameter */
    acrobot_QP_solver_FLOAT sigma;
	
    /* number of backtracking line search steps (affine direction) */
    int lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    int lsit_cc;
    
    /* step size (affine direction) */
    acrobot_QP_solver_FLOAT step_aff;
    
    /* step size (combined direction) */
    acrobot_QP_solver_FLOAT step_cc;    

	/* solvertime */
	acrobot_QP_solver_FLOAT solvetime;   

} acrobot_QP_solver_info;


/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* examine exitflag before using the result! */
int acrobot_QP_solver_solve(acrobot_QP_solver_params* params, acrobot_QP_solver_output* output, acrobot_QP_solver_info* info);


#endif