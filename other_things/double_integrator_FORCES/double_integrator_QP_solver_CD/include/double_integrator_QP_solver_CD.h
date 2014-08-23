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

#ifndef __double_integrator_QP_solver_CD_H__
#define __double_integrator_QP_solver_CD_H__


/* DATA TYPE ------------------------------------------------------------*/
typedef double double_integrator_QP_solver_CD_FLOAT;


/* SOLVER SETTINGS ------------------------------------------------------*/
/* print level */
#ifndef double_integrator_QP_solver_CD_SET_PRINTLEVEL
#define double_integrator_QP_solver_CD_SET_PRINTLEVEL    (0)
#endif

/* timing */
#ifndef double_integrator_QP_solver_CD_SET_TIMING
#define double_integrator_QP_solver_CD_SET_TIMING    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define double_integrator_QP_solver_CD_SET_MAXIT         (100)	

/* scaling factor of line search (affine direction) */
#define double_integrator_QP_solver_CD_SET_LS_SCALE_AFF  (0.9)      

/* scaling factor of line search (combined direction) */
#define double_integrator_QP_solver_CD_SET_LS_SCALE      (0.95)  

/* minimum required step size in each iteration */
#define double_integrator_QP_solver_CD_SET_LS_MINSTEP    (1E-08)

/* maximum step size (combined direction) */
#define double_integrator_QP_solver_CD_SET_LS_MAXSTEP    (0.995)

/* desired relative duality gap */
#define double_integrator_QP_solver_CD_SET_ACC_RDGAP     (0.0001)

/* desired maximum residual on equality constraints */
#define double_integrator_QP_solver_CD_SET_ACC_RESEQ     (1E-06)

/* desired maximum residual on inequality constraints */
#define double_integrator_QP_solver_CD_SET_ACC_RESINEQ   (1E-06)

/* desired maximum violation of complementarity */
#define double_integrator_QP_solver_CD_SET_ACC_KKTCOMPL  (1E-06)


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define double_integrator_QP_solver_CD_OPTIMAL      (1)

/* maximum number of iterations has been reached */
#define double_integrator_QP_solver_CD_MAXITREACHED (0)

/* no progress in line search possible */
#define double_integrator_QP_solver_CD_NOPROGRESS   (-7)




/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct double_integrator_QP_solver_CD_params
{
    /* vector of size 20 */
    double_integrator_QP_solver_CD_FLOAT f1[20];

    /* vector of size 7 */
    double_integrator_QP_solver_CD_FLOAT lb1[7];

    /* vector of size 6 */
    double_integrator_QP_solver_CD_FLOAT ub1[6];

    /* matrix of size [14 x 20] (column major format) */
    double_integrator_QP_solver_CD_FLOAT A1[280];

    /* vector of size 14 */
    double_integrator_QP_solver_CD_FLOAT b1[14];

    /* vector of size 20 */
    double_integrator_QP_solver_CD_FLOAT f2[20];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT lb2[4];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT ub2[4];

    /* matrix of size [28 x 20] (column major format) */
    double_integrator_QP_solver_CD_FLOAT A2[560];

    /* vector of size 28 */
    double_integrator_QP_solver_CD_FLOAT b2[28];

    /* vector of size 20 */
    double_integrator_QP_solver_CD_FLOAT f3[20];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT lb3[4];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT ub3[4];

    /* matrix of size [28 x 20] (column major format) */
    double_integrator_QP_solver_CD_FLOAT A3[560];

    /* vector of size 28 */
    double_integrator_QP_solver_CD_FLOAT b3[28];

    /* vector of size 20 */
    double_integrator_QP_solver_CD_FLOAT f4[20];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT lb4[4];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT ub4[4];

    /* matrix of size [28 x 20] (column major format) */
    double_integrator_QP_solver_CD_FLOAT A4[560];

    /* vector of size 28 */
    double_integrator_QP_solver_CD_FLOAT b4[28];

    /* vector of size 20 */
    double_integrator_QP_solver_CD_FLOAT f5[20];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT lb5[4];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT ub5[4];

    /* matrix of size [28 x 20] (column major format) */
    double_integrator_QP_solver_CD_FLOAT A5[560];

    /* vector of size 28 */
    double_integrator_QP_solver_CD_FLOAT b5[28];

    /* vector of size 20 */
    double_integrator_QP_solver_CD_FLOAT f6[20];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT lb6[4];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT ub6[4];

    /* matrix of size [28 x 20] (column major format) */
    double_integrator_QP_solver_CD_FLOAT A6[560];

    /* vector of size 28 */
    double_integrator_QP_solver_CD_FLOAT b6[28];

    /* vector of size 20 */
    double_integrator_QP_solver_CD_FLOAT f7[20];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT lb7[4];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT ub7[4];

    /* matrix of size [28 x 20] (column major format) */
    double_integrator_QP_solver_CD_FLOAT A7[560];

    /* vector of size 28 */
    double_integrator_QP_solver_CD_FLOAT b7[28];

    /* vector of size 20 */
    double_integrator_QP_solver_CD_FLOAT f8[20];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT lb8[4];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT ub8[4];

    /* matrix of size [28 x 20] (column major format) */
    double_integrator_QP_solver_CD_FLOAT A8[560];

    /* vector of size 28 */
    double_integrator_QP_solver_CD_FLOAT b8[28];

    /* vector of size 20 */
    double_integrator_QP_solver_CD_FLOAT f9[20];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT lb9[4];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT ub9[4];

    /* matrix of size [28 x 20] (column major format) */
    double_integrator_QP_solver_CD_FLOAT A9[560];

    /* vector of size 28 */
    double_integrator_QP_solver_CD_FLOAT b9[28];

    /* vector of size 20 */
    double_integrator_QP_solver_CD_FLOAT f10[20];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT lb10[4];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT ub10[4];

    /* matrix of size [28 x 20] (column major format) */
    double_integrator_QP_solver_CD_FLOAT A10[560];

    /* vector of size 28 */
    double_integrator_QP_solver_CD_FLOAT b10[28];

    /* vector of size 18 */
    double_integrator_QP_solver_CD_FLOAT f11[18];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT lb11[4];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT ub11[4];

    /* matrix of size [28 x 18] (column major format) */
    double_integrator_QP_solver_CD_FLOAT A11[504];

    /* vector of size 28 */
    double_integrator_QP_solver_CD_FLOAT b11[28];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT f12[4];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT lb12[4];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT ub12[4];

} double_integrator_QP_solver_CD_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct double_integrator_QP_solver_CD_output
{
    /* vector of size 7 */
    double_integrator_QP_solver_CD_FLOAT z1[7];

    /* vector of size 7 */
    double_integrator_QP_solver_CD_FLOAT z2[7];

    /* vector of size 7 */
    double_integrator_QP_solver_CD_FLOAT z3[7];

    /* vector of size 7 */
    double_integrator_QP_solver_CD_FLOAT z4[7];

    /* vector of size 7 */
    double_integrator_QP_solver_CD_FLOAT z5[7];

    /* vector of size 7 */
    double_integrator_QP_solver_CD_FLOAT z6[7];

    /* vector of size 7 */
    double_integrator_QP_solver_CD_FLOAT z7[7];

    /* vector of size 7 */
    double_integrator_QP_solver_CD_FLOAT z8[7];

    /* vector of size 7 */
    double_integrator_QP_solver_CD_FLOAT z9[7];

    /* vector of size 7 */
    double_integrator_QP_solver_CD_FLOAT z10[7];

    /* vector of size 7 */
    double_integrator_QP_solver_CD_FLOAT z11[7];

    /* vector of size 4 */
    double_integrator_QP_solver_CD_FLOAT z12[4];

} double_integrator_QP_solver_CD_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct double_integrator_QP_solver_CD_info
{
    /* iteration number */
    int it;
	
    /* inf-norm of equality constraint residuals */
    double_integrator_QP_solver_CD_FLOAT res_eq;
	
    /* inf-norm of inequality constraint residuals */
    double_integrator_QP_solver_CD_FLOAT res_ineq;

    /* primal objective */
    double_integrator_QP_solver_CD_FLOAT pobj;	
	
    /* dual objective */
    double_integrator_QP_solver_CD_FLOAT dobj;	

    /* duality gap := pobj - dobj */
    double_integrator_QP_solver_CD_FLOAT dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    double_integrator_QP_solver_CD_FLOAT rdgap;		

    /* duality measure */
    double_integrator_QP_solver_CD_FLOAT mu;

	/* duality measure (after affine step) */
    double_integrator_QP_solver_CD_FLOAT mu_aff;
	
    /* centering parameter */
    double_integrator_QP_solver_CD_FLOAT sigma;
	
    /* number of backtracking line search steps (affine direction) */
    int lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    int lsit_cc;
    
    /* step size (affine direction) */
    double_integrator_QP_solver_CD_FLOAT step_aff;
    
    /* step size (combined direction) */
    double_integrator_QP_solver_CD_FLOAT step_cc;    

	/* solvertime */
	double_integrator_QP_solver_CD_FLOAT solvetime;   

} double_integrator_QP_solver_CD_info;


/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* examine exitflag before using the result! */
int double_integrator_QP_solver_CD_solve(double_integrator_QP_solver_CD_params* params, double_integrator_QP_solver_CD_output* output, double_integrator_QP_solver_CD_info* info);


#endif