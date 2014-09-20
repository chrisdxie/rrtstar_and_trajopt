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

#ifndef __cartpole_QP_solver_H__
#define __cartpole_QP_solver_H__


/* DATA TYPE ------------------------------------------------------------*/
typedef double cartpole_QP_solver_FLOAT;


/* SOLVER SETTINGS ------------------------------------------------------*/
/* print level */
#ifndef cartpole_QP_solver_SET_PRINTLEVEL
#define cartpole_QP_solver_SET_PRINTLEVEL    (2)
#endif

/* timing */
#ifndef cartpole_QP_solver_SET_TIMING
#define cartpole_QP_solver_SET_TIMING    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define cartpole_QP_solver_SET_MAXIT         (100)	

/* scaling factor of line search (affine direction) */
#define cartpole_QP_solver_SET_LS_SCALE_AFF  (0.9)      

/* scaling factor of line search (combined direction) */
#define cartpole_QP_solver_SET_LS_SCALE      (0.95)  

/* minimum required step size in each iteration */
#define cartpole_QP_solver_SET_LS_MINSTEP    (1E-08)

/* maximum step size (combined direction) */
#define cartpole_QP_solver_SET_LS_MAXSTEP    (0.995)

/* desired relative duality gap */
#define cartpole_QP_solver_SET_ACC_RDGAP     (0.0001)

/* desired maximum residual on equality constraints */
#define cartpole_QP_solver_SET_ACC_RESEQ     (1E-06)

/* desired maximum residual on inequality constraints */
#define cartpole_QP_solver_SET_ACC_RESINEQ   (1E-06)

/* desired maximum violation of complementarity */
#define cartpole_QP_solver_SET_ACC_KKTCOMPL  (1E-06)


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define cartpole_QP_solver_OPTIMAL      (1)

/* maximum number of iterations has been reached */
#define cartpole_QP_solver_MAXITREACHED (0)

/* no progress in line search possible */
#define cartpole_QP_solver_NOPROGRESS   (-7)




/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct cartpole_QP_solver_params
{
    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f1[15];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT lb1[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT ub1[6];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A1[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b1[20];

    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f2[15];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb2[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub2[1];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A2[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b2[20];

    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f3[15];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb3[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub3[1];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A3[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b3[20];

    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f4[15];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb4[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub4[1];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A4[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b4[20];

    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f5[15];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb5[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub5[1];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A5[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b5[20];

    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f6[15];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb6[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub6[1];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A6[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b6[20];

    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f7[15];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb7[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub7[1];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A7[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b7[20];

    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f8[15];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb8[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub8[1];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A8[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b8[20];

    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f9[15];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb9[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub9[1];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A9[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b9[20];

    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f10[15];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb10[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub10[1];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A10[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b10[20];

    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f11[15];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb11[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub11[1];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A11[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b11[20];

    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f12[15];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb12[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub12[1];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A12[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b12[20];

    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f13[15];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb13[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub13[1];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A13[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b13[20];

    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f14[15];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb14[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub14[1];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A14[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b14[20];

    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f15[15];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb15[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub15[1];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A15[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b15[20];

    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f16[15];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb16[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub16[1];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A16[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b16[20];

    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f17[15];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb17[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub17[1];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A17[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b17[20];

    /* vector of size 15 */
    cartpole_QP_solver_FLOAT f18[15];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb18[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub18[1];

    /* matrix of size [20 x 15] (column major format) */
    cartpole_QP_solver_FLOAT A18[300];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b18[20];

    /* vector of size 14 */
    cartpole_QP_solver_FLOAT f19[14];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT lb19[1];

    /* vector of size 1 */
    cartpole_QP_solver_FLOAT ub19[1];

    /* matrix of size [20 x 14] (column major format) */
    cartpole_QP_solver_FLOAT A19[280];

    /* vector of size 20 */
    cartpole_QP_solver_FLOAT b19[20];

    /* vector of size 4 */
    cartpole_QP_solver_FLOAT f20[4];

    /* vector of size 4 */
    cartpole_QP_solver_FLOAT lb20[4];

    /* vector of size 4 */
    cartpole_QP_solver_FLOAT ub20[4];

} cartpole_QP_solver_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct cartpole_QP_solver_output
{
    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z1[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z2[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z3[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z4[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z5[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z6[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z7[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z8[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z9[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z10[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z11[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z12[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z13[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z14[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z15[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z16[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z17[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z18[6];

    /* vector of size 6 */
    cartpole_QP_solver_FLOAT z19[6];

    /* vector of size 4 */
    cartpole_QP_solver_FLOAT z20[4];

} cartpole_QP_solver_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct cartpole_QP_solver_info
{
    /* iteration number */
    int it;
	
    /* inf-norm of equality constraint residuals */
    cartpole_QP_solver_FLOAT res_eq;
	
    /* inf-norm of inequality constraint residuals */
    cartpole_QP_solver_FLOAT res_ineq;

    /* primal objective */
    cartpole_QP_solver_FLOAT pobj;	
	
    /* dual objective */
    cartpole_QP_solver_FLOAT dobj;	

    /* duality gap := pobj - dobj */
    cartpole_QP_solver_FLOAT dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    cartpole_QP_solver_FLOAT rdgap;		

    /* duality measure */
    cartpole_QP_solver_FLOAT mu;

	/* duality measure (after affine step) */
    cartpole_QP_solver_FLOAT mu_aff;
	
    /* centering parameter */
    cartpole_QP_solver_FLOAT sigma;
	
    /* number of backtracking line search steps (affine direction) */
    int lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    int lsit_cc;
    
    /* step size (affine direction) */
    cartpole_QP_solver_FLOAT step_aff;
    
    /* step size (combined direction) */
    cartpole_QP_solver_FLOAT step_cc;    

	/* solvertime */
	cartpole_QP_solver_FLOAT solvetime;   

} cartpole_QP_solver_info;


/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* examine exitflag before using the result! */
int cartpole_QP_solver_solve(cartpole_QP_solver_params* params, cartpole_QP_solver_output* output, cartpole_QP_solver_info* info);


#endif