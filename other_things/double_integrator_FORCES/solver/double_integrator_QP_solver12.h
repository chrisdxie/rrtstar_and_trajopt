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

#ifndef __double_integrator_QP_solver_H__
#define __double_integrator_QP_solver_H__


/* DATA TYPE ------------------------------------------------------------*/
typedef double double_integrator_QP_solver_FLOAT;


/* SOLVER SETTINGS ------------------------------------------------------*/
/* print level */
#ifndef double_integrator_QP_solver_SET_PRINTLEVEL
#define double_integrator_QP_solver_SET_PRINTLEVEL    (2)
#endif

/* timing */
#ifndef double_integrator_QP_solver_SET_TIMING
#define double_integrator_QP_solver_SET_TIMING    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define double_integrator_QP_solver_SET_MAXIT         (100)	

/* scaling factor of line search (affine direction) */
#define double_integrator_QP_solver_SET_LS_SCALE_AFF  (0.9)      

/* scaling factor of line search (combined direction) */
#define double_integrator_QP_solver_SET_LS_SCALE      (0.95)  

/* minimum required step size in each iteration */
#define double_integrator_QP_solver_SET_LS_MINSTEP    (1E-08)

/* maximum step size (combined direction) */
#define double_integrator_QP_solver_SET_LS_MAXSTEP    (0.995)

/* desired relative duality gap */
#define double_integrator_QP_solver_SET_ACC_RDGAP     (0.0001)

/* desired maximum residual on equality constraints */
#define double_integrator_QP_solver_SET_ACC_RESEQ     (1E-06)

/* desired maximum residual on inequality constraints */
#define double_integrator_QP_solver_SET_ACC_RESINEQ   (1E-06)

/* desired maximum violation of complementarity */
#define double_integrator_QP_solver_SET_ACC_KKTCOMPL  (1E-06)


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define double_integrator_QP_solver_OPTIMAL      (1)

/* maximum number of iterations has been reached */
#define double_integrator_QP_solver_MAXITREACHED (0)

/* no progress in line search possible */
#define double_integrator_QP_solver_NOPROGRESS   (-7)




/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct double_integrator_QP_solver_params
{
    /* vector of size 18 */
    double_integrator_QP_solver_FLOAT f1[18];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb1[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub1[6];

    /* matrix of size [24 x 18] (column major format) */
    double_integrator_QP_solver_FLOAT A1[432];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b1[24];

    /* vector of size 18 */
    double_integrator_QP_solver_FLOAT f2[18];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT lb2[6];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub2[6];

    /* matrix of size [24 x 18] (column major format) */
    double_integrator_QP_solver_FLOAT A2[432];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b2[24];

    /* vector of size 18 */
    double_integrator_QP_solver_FLOAT f3[18];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT lb3[6];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub3[6];

    /* matrix of size [24 x 18] (column major format) */
    double_integrator_QP_solver_FLOAT A3[432];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b3[24];

    /* vector of size 18 */
    double_integrator_QP_solver_FLOAT f4[18];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT lb4[6];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub4[6];

    /* matrix of size [24 x 18] (column major format) */
    double_integrator_QP_solver_FLOAT A4[432];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b4[24];

    /* vector of size 18 */
    double_integrator_QP_solver_FLOAT f5[18];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT lb5[6];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub5[6];

    /* matrix of size [24 x 18] (column major format) */
    double_integrator_QP_solver_FLOAT A5[432];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b5[24];

    /* vector of size 18 */
    double_integrator_QP_solver_FLOAT f6[18];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT lb6[6];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub6[6];

    /* matrix of size [24 x 18] (column major format) */
    double_integrator_QP_solver_FLOAT A6[432];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b6[24];

    /* vector of size 18 */
    double_integrator_QP_solver_FLOAT f7[18];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT lb7[6];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub7[6];

    /* matrix of size [24 x 18] (column major format) */
    double_integrator_QP_solver_FLOAT A7[432];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b7[24];

    /* vector of size 18 */
    double_integrator_QP_solver_FLOAT f8[18];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT lb8[6];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub8[6];

    /* matrix of size [24 x 18] (column major format) */
    double_integrator_QP_solver_FLOAT A8[432];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b8[24];

    /* vector of size 18 */
    double_integrator_QP_solver_FLOAT f9[18];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT lb9[6];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub9[6];

    /* matrix of size [24 x 18] (column major format) */
    double_integrator_QP_solver_FLOAT A9[432];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b9[24];

    /* vector of size 18 */
    double_integrator_QP_solver_FLOAT f10[18];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT lb10[6];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub10[6];

    /* matrix of size [24 x 18] (column major format) */
    double_integrator_QP_solver_FLOAT A10[432];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b10[24];

    /* vector of size 16 */
    double_integrator_QP_solver_FLOAT f11[16];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT lb11[6];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub11[6];

    /* matrix of size [24 x 16] (column major format) */
    double_integrator_QP_solver_FLOAT A11[384];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b11[24];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT f12[5];

    /* vector of size 4 */
    double_integrator_QP_solver_FLOAT lb12[4];

    /* vector of size 4 */
    double_integrator_QP_solver_FLOAT ub12[4];

    /* matrix of size [10 x 5] (column major format) */
    double_integrator_QP_solver_FLOAT A12[50];

    /* vector of size 10 */
    double_integrator_QP_solver_FLOAT b12[10];

} double_integrator_QP_solver_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct double_integrator_QP_solver_output
{
    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT z1[7];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT z2[7];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT z3[7];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT z4[7];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT z5[7];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT z6[7];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT z7[7];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT z8[7];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT z9[7];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT z10[7];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT z11[7];

    /* vector of size 4 */
    double_integrator_QP_solver_FLOAT z12[4];

} double_integrator_QP_solver_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct double_integrator_QP_solver_info
{
    /* iteration number */
    int it;
	
    /* inf-norm of equality constraint residuals */
    double_integrator_QP_solver_FLOAT res_eq;
	
    /* inf-norm of inequality constraint residuals */
    double_integrator_QP_solver_FLOAT res_ineq;

    /* primal objective */
    double_integrator_QP_solver_FLOAT pobj;	
	
    /* dual objective */
    double_integrator_QP_solver_FLOAT dobj;	

    /* duality gap := pobj - dobj */
    double_integrator_QP_solver_FLOAT dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    double_integrator_QP_solver_FLOAT rdgap;		

    /* duality measure */
    double_integrator_QP_solver_FLOAT mu;

	/* duality measure (after affine step) */
    double_integrator_QP_solver_FLOAT mu_aff;
	
    /* centering parameter */
    double_integrator_QP_solver_FLOAT sigma;
	
    /* number of backtracking line search steps (affine direction) */
    int lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    int lsit_cc;
    
    /* step size (affine direction) */
    double_integrator_QP_solver_FLOAT step_aff;
    
    /* step size (combined direction) */
    double_integrator_QP_solver_FLOAT step_cc;    

	/* solvertime */
	double_integrator_QP_solver_FLOAT solvetime;   

} double_integrator_QP_solver_info;


/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* examine exitflag before using the result! */
int double_integrator_QP_solver_solve(double_integrator_QP_solver_params* params, double_integrator_QP_solver_output* output, double_integrator_QP_solver_info* info);


#endif