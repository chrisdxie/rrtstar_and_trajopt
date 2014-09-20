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
#define double_integrator_QP_solver_SET_PRINTLEVEL    (0)
#endif

/* timing */
#ifndef double_integrator_QP_solver_SET_TIMING
#define double_integrator_QP_solver_SET_TIMING    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define double_integrator_QP_solver_SET_MAXIT         (50)	

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
    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT f1[15];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT lb1[15];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT ub1[7];

    /* matrix of size [9 x 15] (column major format) */
    double_integrator_QP_solver_FLOAT C1[135];

    /* vector of size 9 */
    double_integrator_QP_solver_FLOAT e1[9];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT f2[15];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT lb2[15];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT ub2[7];

    /* matrix of size [5 x 15] (column major format) */
    double_integrator_QP_solver_FLOAT C2[75];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT e2[5];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT f3[15];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT lb3[15];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT ub3[7];

    /* matrix of size [5 x 15] (column major format) */
    double_integrator_QP_solver_FLOAT C3[75];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT e3[5];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT f4[15];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT lb4[15];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT ub4[7];

    /* matrix of size [5 x 15] (column major format) */
    double_integrator_QP_solver_FLOAT C4[75];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT e4[5];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT f5[15];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT lb5[15];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT ub5[7];

    /* matrix of size [5 x 15] (column major format) */
    double_integrator_QP_solver_FLOAT C5[75];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT e5[5];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT f6[15];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT lb6[15];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT ub6[7];

    /* matrix of size [5 x 15] (column major format) */
    double_integrator_QP_solver_FLOAT C6[75];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT e6[5];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT f7[15];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT lb7[15];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT ub7[7];

    /* matrix of size [5 x 15] (column major format) */
    double_integrator_QP_solver_FLOAT C7[75];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT e7[5];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT f8[15];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT lb8[15];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT ub8[7];

    /* matrix of size [5 x 15] (column major format) */
    double_integrator_QP_solver_FLOAT C8[75];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT e8[5];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT f9[15];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT lb9[15];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT ub9[7];

    /* matrix of size [5 x 15] (column major format) */
    double_integrator_QP_solver_FLOAT C9[75];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT e9[5];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT f10[15];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT lb10[15];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT ub10[7];

    /* matrix of size [5 x 15] (column major format) */
    double_integrator_QP_solver_FLOAT C10[75];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT e10[5];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT f11[15];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT lb11[15];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT ub11[7];

    /* matrix of size [5 x 15] (column major format) */
    double_integrator_QP_solver_FLOAT C11[75];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT e11[5];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT f12[15];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT lb12[15];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT ub12[7];

    /* matrix of size [5 x 15] (column major format) */
    double_integrator_QP_solver_FLOAT C12[75];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT e12[5];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT f13[15];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT lb13[15];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT ub13[7];

    /* matrix of size [5 x 15] (column major format) */
    double_integrator_QP_solver_FLOAT C13[75];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT e13[5];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT f14[15];

    /* vector of size 15 */
    double_integrator_QP_solver_FLOAT lb14[15];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT ub14[7];

    /* matrix of size [5 x 15] (column major format) */
    double_integrator_QP_solver_FLOAT C14[75];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT e14[5];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT lb15[5];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT ub15[5];

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

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT z12[7];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT z13[7];

    /* vector of size 7 */
    double_integrator_QP_solver_FLOAT z14[7];

    /* vector of size 5 */
    double_integrator_QP_solver_FLOAT z15[5];

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