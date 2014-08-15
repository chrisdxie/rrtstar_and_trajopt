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
#define double_integrator_QP_solver_SET_PRINTLEVEL    (3)
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
    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f1[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb1[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub1[6];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT c1[12];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A1[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b1[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f2[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb2[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub2[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A2[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b2[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f3[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb3[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub3[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A3[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b3[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f4[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb4[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub4[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A4[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b4[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f5[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb5[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub5[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A5[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b5[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f6[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb6[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub6[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A6[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b6[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f7[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb7[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub7[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A7[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b7[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f8[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb8[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub8[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A8[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b8[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f9[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb9[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub9[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A9[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b9[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f10[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb10[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub10[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A10[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b10[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f11[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb11[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub11[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A11[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b11[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f12[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb12[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub12[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A12[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b12[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f13[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb13[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub13[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A13[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b13[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f14[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb14[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub14[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A14[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b14[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f15[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb15[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub15[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A15[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b15[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f16[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb16[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub16[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A16[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b16[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f17[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb17[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub17[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A17[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b17[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f18[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb18[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub18[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A18[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b18[24];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT f19[12];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT lb19[7];

    /* vector of size 6 */
    double_integrator_QP_solver_FLOAT ub19[6];

    /* matrix of size [24 x 12] (column major format) */
    double_integrator_QP_solver_FLOAT A19[288];

    /* vector of size 24 */
    double_integrator_QP_solver_FLOAT b19[24];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT f20[5];

    /* vector of size 4 */
    double_integrator_QP_solver_FLOAT lb20[4];

    /* vector of size 4 */
    double_integrator_QP_solver_FLOAT ub20[4];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT c20[5];

    /* matrix of size [10 x 5] (column major format) */
    double_integrator_QP_solver_FLOAT A20[50];

    /* vector of size 10 */
    double_integrator_QP_solver_FLOAT b20[10];

} double_integrator_QP_solver_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct double_integrator_QP_solver_output
{
    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z1[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z2[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z3[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z4[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z5[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z6[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z7[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z8[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z9[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z10[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z11[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z12[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z13[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z14[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z15[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z16[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z17[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z18[12];

    /* vector of size 12 */
    double_integrator_QP_solver_FLOAT z19[12];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT z20[5];

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