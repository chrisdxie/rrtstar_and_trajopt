/*
double_integrator_QP_solver_noCD : A fast customized optimization solver.

Copyright (C) 2014 EMBOTECH GMBH [info@embotech.com]


This software is intended for simulation and testing purposes only. 
Use of this software for any commercial purpose is prohibited.

This program is distributed in the hope that it will be useful.
EMBOTECH makes NO WARRANTIES with respect to the use of the software 
without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE. 

EMBOTECH shall not have any liability for any damage arising from the use
of the software.

This Agreement shall exclusively be governed by and interpreted in 
accordance with the laws of Switzerland, excluding its principles
of conflict of laws. The Courts of Zurich-City shall have exclusive 
jurisdiction in case of any dispute.

*/

#ifndef __double_integrator_QP_solver_noCD_H__
#define __double_integrator_QP_solver_noCD_H__


/* DATA TYPE ------------------------------------------------------------*/
typedef double double_integrator_QP_solver_noCD_FLOAT;

typedef double INTERFACE_FLOAT;

/* SOLVER SETTINGS ------------------------------------------------------*/
/* print level */
#ifndef double_integrator_QP_solver_noCD_SET_PRINTLEVEL
#define double_integrator_QP_solver_noCD_SET_PRINTLEVEL    (0)
#endif

/* timing */
#ifndef double_integrator_QP_solver_noCD_SET_TIMING
#define double_integrator_QP_solver_noCD_SET_TIMING    (1)
#endif

/* Numeric Warnings */
/* #define PRINTNUMERICALWARNINGS */

/* maximum number of iterations  */
#define double_integrator_QP_solver_noCD_SET_MAXIT         (100)	

/* scaling factor of line search (affine direction) */
#define double_integrator_QP_solver_noCD_SET_LS_SCALE_AFF  (0.9)      

/* scaling factor of line search (combined direction) */
#define double_integrator_QP_solver_noCD_SET_LS_SCALE      (0.95)  

/* minimum required step size in each iteration */
#define double_integrator_QP_solver_noCD_SET_LS_MINSTEP    (1E-08)

/* maximum step size (combined direction) */
#define double_integrator_QP_solver_noCD_SET_LS_MAXSTEP    (0.995)

/* desired relative duality gap */
#define double_integrator_QP_solver_noCD_SET_ACC_RDGAP     (0.0001)

/* desired maximum residual on equality constraints */
#define double_integrator_QP_solver_noCD_SET_ACC_RESEQ     (1E-06)

/* desired maximum residual on inequality constraints */
#define double_integrator_QP_solver_noCD_SET_ACC_RESINEQ   (1E-06)

/* desired maximum violation of complementarity */
#define double_integrator_QP_solver_noCD_SET_ACC_KKTCOMPL  (1E-06)


/* RETURN CODES----------------------------------------------------------*/
/* solver has converged within desired accuracy */
#define double_integrator_QP_solver_noCD_OPTIMAL      (1)

/* maximum number of iterations has been reached */
#define double_integrator_QP_solver_noCD_MAXITREACHED (0)

/* no progress in line search possible */
#define double_integrator_QP_solver_noCD_NOPROGRESS   (-7)




/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct double_integrator_QP_solver_noCD_params
{
    /* vector of size 17 */
    double_integrator_QP_solver_noCD_FLOAT f1[17];

    /* vector of size 7 */
    double_integrator_QP_solver_noCD_FLOAT lb1[7];

    /* vector of size 6 */
    double_integrator_QP_solver_noCD_FLOAT ub1[6];

    /* matrix of size [22 x 17] (column major format) */
    double_integrator_QP_solver_noCD_FLOAT A1[374];

    /* vector of size 22 */
    double_integrator_QP_solver_noCD_FLOAT b1[22];

    /* vector of size 17 */
    double_integrator_QP_solver_noCD_FLOAT f2[17];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT lb2[4];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT ub2[4];

    /* matrix of size [22 x 17] (column major format) */
    double_integrator_QP_solver_noCD_FLOAT A2[374];

    /* vector of size 22 */
    double_integrator_QP_solver_noCD_FLOAT b2[22];

    /* vector of size 17 */
    double_integrator_QP_solver_noCD_FLOAT f3[17];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT lb3[4];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT ub3[4];

    /* matrix of size [22 x 17] (column major format) */
    double_integrator_QP_solver_noCD_FLOAT A3[374];

    /* vector of size 22 */
    double_integrator_QP_solver_noCD_FLOAT b3[22];

    /* vector of size 17 */
    double_integrator_QP_solver_noCD_FLOAT f4[17];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT lb4[4];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT ub4[4];

    /* matrix of size [22 x 17] (column major format) */
    double_integrator_QP_solver_noCD_FLOAT A4[374];

    /* vector of size 22 */
    double_integrator_QP_solver_noCD_FLOAT b4[22];

    /* vector of size 17 */
    double_integrator_QP_solver_noCD_FLOAT f5[17];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT lb5[4];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT ub5[4];

    /* matrix of size [22 x 17] (column major format) */
    double_integrator_QP_solver_noCD_FLOAT A5[374];

    /* vector of size 22 */
    double_integrator_QP_solver_noCD_FLOAT b5[22];

    /* vector of size 17 */
    double_integrator_QP_solver_noCD_FLOAT f6[17];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT lb6[4];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT ub6[4];

    /* matrix of size [22 x 17] (column major format) */
    double_integrator_QP_solver_noCD_FLOAT A6[374];

    /* vector of size 22 */
    double_integrator_QP_solver_noCD_FLOAT b6[22];

    /* vector of size 17 */
    double_integrator_QP_solver_noCD_FLOAT f7[17];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT lb7[4];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT ub7[4];

    /* matrix of size [22 x 17] (column major format) */
    double_integrator_QP_solver_noCD_FLOAT A7[374];

    /* vector of size 22 */
    double_integrator_QP_solver_noCD_FLOAT b7[22];

    /* vector of size 17 */
    double_integrator_QP_solver_noCD_FLOAT f8[17];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT lb8[4];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT ub8[4];

    /* matrix of size [22 x 17] (column major format) */
    double_integrator_QP_solver_noCD_FLOAT A8[374];

    /* vector of size 22 */
    double_integrator_QP_solver_noCD_FLOAT b8[22];

    /* vector of size 17 */
    double_integrator_QP_solver_noCD_FLOAT f9[17];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT lb9[4];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT ub9[4];

    /* matrix of size [22 x 17] (column major format) */
    double_integrator_QP_solver_noCD_FLOAT A9[374];

    /* vector of size 22 */
    double_integrator_QP_solver_noCD_FLOAT b9[22];

    /* vector of size 17 */
    double_integrator_QP_solver_noCD_FLOAT f10[17];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT lb10[4];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT ub10[4];

    /* matrix of size [22 x 17] (column major format) */
    double_integrator_QP_solver_noCD_FLOAT A10[374];

    /* vector of size 22 */
    double_integrator_QP_solver_noCD_FLOAT b10[22];

    /* vector of size 15 */
    double_integrator_QP_solver_noCD_FLOAT f11[15];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT lb11[4];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT ub11[4];

    /* matrix of size [22 x 15] (column major format) */
    double_integrator_QP_solver_noCD_FLOAT A11[330];

    /* vector of size 22 */
    double_integrator_QP_solver_noCD_FLOAT b11[22];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT f12[4];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT lb12[4];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT ub12[4];

    /* matrix of size [8 x 4] (column major format) */
    double_integrator_QP_solver_noCD_FLOAT A12[32];

    /* vector of size 8 */
    double_integrator_QP_solver_noCD_FLOAT b12[8];

} double_integrator_QP_solver_noCD_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct double_integrator_QP_solver_noCD_output
{
    /* vector of size 7 */
    double_integrator_QP_solver_noCD_FLOAT z1[7];

    /* vector of size 7 */
    double_integrator_QP_solver_noCD_FLOAT z2[7];

    /* vector of size 7 */
    double_integrator_QP_solver_noCD_FLOAT z3[7];

    /* vector of size 7 */
    double_integrator_QP_solver_noCD_FLOAT z4[7];

    /* vector of size 7 */
    double_integrator_QP_solver_noCD_FLOAT z5[7];

    /* vector of size 7 */
    double_integrator_QP_solver_noCD_FLOAT z6[7];

    /* vector of size 7 */
    double_integrator_QP_solver_noCD_FLOAT z7[7];

    /* vector of size 7 */
    double_integrator_QP_solver_noCD_FLOAT z8[7];

    /* vector of size 7 */
    double_integrator_QP_solver_noCD_FLOAT z9[7];

    /* vector of size 7 */
    double_integrator_QP_solver_noCD_FLOAT z10[7];

    /* vector of size 7 */
    double_integrator_QP_solver_noCD_FLOAT z11[7];

    /* vector of size 4 */
    double_integrator_QP_solver_noCD_FLOAT z12[4];

} double_integrator_QP_solver_noCD_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct double_integrator_QP_solver_noCD_info
{
    /* iteration number */
    int it;
	
    /* inf-norm of equality constraint residuals */
    double_integrator_QP_solver_noCD_FLOAT res_eq;
	
    /* inf-norm of inequality constraint residuals */
    double_integrator_QP_solver_noCD_FLOAT res_ineq;

    /* primal objective */
    double_integrator_QP_solver_noCD_FLOAT pobj;	
	
    /* dual objective */
    double_integrator_QP_solver_noCD_FLOAT dobj;	

    /* duality gap := pobj - dobj */
    double_integrator_QP_solver_noCD_FLOAT dgap;		
	
    /* relative duality gap := |dgap / pobj | */
    double_integrator_QP_solver_noCD_FLOAT rdgap;		

    /* duality measure */
    double_integrator_QP_solver_noCD_FLOAT mu;

	/* duality measure (after affine step) */
    double_integrator_QP_solver_noCD_FLOAT mu_aff;
	
    /* centering parameter */
    double_integrator_QP_solver_noCD_FLOAT sigma;
	
    /* number of backtracking line search steps (affine direction) */
    int lsit_aff;
    
    /* number of backtracking line search steps (combined direction) */
    int lsit_cc;
    
    /* step size (affine direction) */
    double_integrator_QP_solver_noCD_FLOAT step_aff;
    
    /* step size (combined direction) */
    double_integrator_QP_solver_noCD_FLOAT step_cc;    

	/* solvertime */
	double_integrator_QP_solver_noCD_FLOAT solvetime;   

} double_integrator_QP_solver_noCD_info;



/* SOLVER FUNCTION DEFINITION -------------------------------------------*/
/* examine exitflag before using the result! */
int double_integrator_QP_solver_noCD_solve(double_integrator_QP_solver_noCD_params* params, double_integrator_QP_solver_noCD_output* output, double_integrator_QP_solver_noCD_info* info);



#endif