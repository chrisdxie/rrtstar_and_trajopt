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

#include "double_integrator_QP_solver.h"

/* for square root */
#include <math.h> 

/* SAFE DIVISION ------------------------------------------------------- */
#define MAX(X,Y)  ((X) < (Y) ? (Y) : (X))
#define MIN(X,Y)  ((X) < (Y) ? (X) : (Y))
/*#define SAFEDIV_POS(X,Y)  ( (Y) < EPS ? ((X)/EPS) : (X)/(Y) ) 
#define EPS (1.0000E-013) */
#define BIGM (1E30)
#define BIGMM (1E60)

/* includes for parallel computation if necessary */


/* SYSTEM INCLUDES FOR PRINTING ---------------------------------------- */


/* TIMING LIBRARY ------------------------------------------------- */

/* ARE WE ON WINDOWS? */
#if (defined WIN32 || defined _WIN64 || defined _WIN32)

/* Use Windows QueryPerformanceCounter for timing */

#include <windows.h>

typedef struct double_integrator_QP_solver_timer{
	LARGE_INTEGER tic;
	LARGE_INTEGER toc;
	LARGE_INTEGER freq;
} double_integrator_QP_solver_timer;


void double_integrator_QP_solver_tic(double_integrator_QP_solver_timer* t)
{
	QueryPerformanceFrequency(&t->freq);
	QueryPerformanceCounter(&t->tic);
}



double_integrator_QP_solver_FLOAT double_integrator_QP_solver_toc(double_integrator_QP_solver_timer* t)
{
	QueryPerformanceCounter(&t->toc);
	return ((t->toc.QuadPart - t->tic.QuadPart) / (double_integrator_QP_solver_FLOAT)t->freq.QuadPart);
}


/* WE ARE ON THE MAC */
#elif (defined __APPLE__)
#include <mach/mach_time.h>


/* Use MAC OSX  mach_time for timing */
typedef struct double_integrator_QP_solver_timer{
	uint64_t tic;
	uint64_t toc;
	mach_timebase_info_data_t tinfo;

} double_integrator_QP_solver_timer;


void double_integrator_QP_solver_tic(double_integrator_QP_solver_timer* t)
{
    /* read current clock cycles */
    t->tic = mach_absolute_time();
}



double_integrator_QP_solver_FLOAT double_integrator_QP_solver_toc(double_integrator_QP_solver_timer* t)
{
    uint64_t duration; /* elapsed time in clock cycles*/
    t->toc = mach_absolute_time();
	duration = t->toc - t->tic;

    /*conversion from clock cycles to nanoseconds*/
    mach_timebase_info(&(t->tinfo));
    duration *= t->tinfo.numer;
    duration /= t->tinfo.denom;

    return (double_integrator_QP_solver_FLOAT)duration / 1000000000;
}

/* WE ARE ON SOME TEXAS INSTRUMENTS PLATFORM */
#elif (defined __TI_COMPILER_VERSION__)

/* TimeStamps */
#include <c6x.h> /* make use of TSCL, TSCH */


typedef struct double_integrator_QP_solver_timer{
	unsigned long long tic;
	unsigned long long toc;
} double_integrator_QP_solver_timer;


void double_integrator_QP_solver_tic(double_integrator_QP_solver_timer* t)
{
	TSCL = 0;	/* Initiate CPU timer by writing any val to TSCL */
	t->tic = _itoll( TSCH, TSCL );
}



double_integrator_QP_solver_FLOAT double_integrator_QP_solver_toc(double_integrator_QP_solver_timer* t)
{
	t->toc = _itoll( TSCH, TSCL );
	unsigned long long t0;
	unsigned long long overhead;
	t0 = _itoll( TSCH, TSCL );
	overhead = _itoll( TSCH, TSCL )  - t0;

	return (double_integrator_QP_solver_FLOAT)(t->toc - t->tic - overhead) / 1000000000;
}



/* WE ARE ON SOME OTHER UNIX/LINUX SYSTEM */
#else

/* Use POSIX clocl_gettime() for timing on non-Windows machines */
#include <time.h>
typedef struct double_integrator_QP_solver_timer{
	struct timespec tic;
	struct timespec toc;
} double_integrator_QP_solver_timer;


/* read current time */
void double_integrator_QP_solver_tic(double_integrator_QP_solver_timer* t)
{
	clock_gettime(CLOCK_MONOTONIC, &t->tic);
}



/* return time passed since last call to tic on this timer */
double double_integrator_QP_solver_toc(double_integrator_QP_solver_timer* t)
{
	struct timespec temp;
	clock_gettime(CLOCK_MONOTONIC, &t->toc);	

	if ((t->toc.tv_nsec - t->tic.tv_nsec)<0) {
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec-1;
		temp.tv_nsec = 1000000000+t->toc.tv_nsec - t->tic.tv_nsec;
	} else {
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec;
		temp.tv_nsec = t->toc.tv_nsec - t->tic.tv_nsec;
	}

	return (double_integrator_QP_solver_FLOAT)temp.tv_sec + (double_integrator_QP_solver_FLOAT)temp.tv_nsec / 1000000000;
}


#endif

/* LINEAR ALGEBRA LIBRARY ---------------------------------------------- */
/*
 * Initializes a vector of length 170 with a value.
 */
void double_integrator_QP_solver_LA_INITIALIZEVECTOR_170(double_integrator_QP_solver_FLOAT* vec, double_integrator_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<170; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 59 with a value.
 */
void double_integrator_QP_solver_LA_INITIALIZEVECTOR_59(double_integrator_QP_solver_FLOAT* vec, double_integrator_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<59; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 252 with a value.
 */
void double_integrator_QP_solver_LA_INITIALIZEVECTOR_252(double_integrator_QP_solver_FLOAT* vec, double_integrator_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<252; i++ )
	{
		vec[i] = value;
	}
}


/* 
 * Calculates a dot product and adds it to a variable: z += x'*y; 
 * This function is for vectors of length 252.
 */
void double_integrator_QP_solver_LA_DOTACC_252(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<252; i++ ){
		*z += x[i]*y[i];
	}
}


/*
 * Calculates the gradient and the value for a quadratic function 0.5*z'*H*z + f'*z
 *
 * INPUTS:     H  - Symmetric Hessian, diag matrix of size [15 x 15]
 *             f  - column vector of size 15
 *             z  - column vector of size 15
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 15
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void double_integrator_QP_solver_LA_DIAG_QUADFCN_15(double_integrator_QP_solver_FLOAT* H, double_integrator_QP_solver_FLOAT* f, double_integrator_QP_solver_FLOAT* z, double_integrator_QP_solver_FLOAT* grad, double_integrator_QP_solver_FLOAT* value)
{
	int i;
	double_integrator_QP_solver_FLOAT hz;	
	for( i=0; i<15; i++){
		hz = H[i]*z[i];
		grad[i] = hz + f[i];
		*value += 0.5*hz*z[i] + f[i]*z[i];
	}
}


/*
 * Calculates the gradient and the value for a quadratic function 0.5*z'*H*z + f'*z
 *
 * INPUTS:     H  - Symmetric Hessian, diag matrix of size [5 x 5]
 *             f  - column vector of size 5
 *             z  - column vector of size 5
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 5
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void double_integrator_QP_solver_LA_DIAG_QUADFCN_5(double_integrator_QP_solver_FLOAT* H, double_integrator_QP_solver_FLOAT* f, double_integrator_QP_solver_FLOAT* z, double_integrator_QP_solver_FLOAT* grad, double_integrator_QP_solver_FLOAT* value)
{
	int i;
	double_integrator_QP_solver_FLOAT hz;	
	for( i=0; i<5; i++){
		hz = H[i]*z[i];
		grad[i] = hz + f[i];
		*value += 0.5*hz*z[i] + f[i]*z[i];
	}
}


/* 
 * Computes r = A*x + B*u - b
 * and      y = max([norm(r,inf), y])
 * and      z -= l'*r
 * where A is stored in column major format
 */
void double_integrator_QP_solver_LA_DENSE_MVMSUB3_9_15_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;
	double_integrator_QP_solver_FLOAT AxBu[9];
	double_integrator_QP_solver_FLOAT norm = *y;
	double_integrator_QP_solver_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<9; i++ ){
		AxBu[i] = A[k++]*x[0] + B[m++]*u[0];
	}	
	for( j=1; j<15; j++ ){		
		for( i=0; i<9; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<15; n++ ){
		for( i=0; i<9; i++ ){
			AxBu[i] += B[m++]*u[n];
		}		
	}

	for( i=0; i<9; i++ ){
		r[i] = AxBu[i] - b[i];
		lr += l[i]*r[i];
		if( r[i] > norm ){
			norm = r[i];
		}
		if( -r[i] > norm ){
			norm = -r[i];
		}
	}
	*y = norm;
	*z -= lr;
}


/* 
 * Computes r = A*x + B*u - b
 * and      y = max([norm(r,inf), y])
 * and      z -= l'*r
 * where A is stored in column major format
 */
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_15_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	double_integrator_QP_solver_FLOAT AxBu[5];
	double_integrator_QP_solver_FLOAT norm = *y;
	double_integrator_QP_solver_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<5; i++ ){
		AxBu[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<15; j++ ){		
		for( i=0; i<5; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<5; i++ ){
		r[i] = AxBu[i] - b[i];
		lr += l[i]*r[i];
		if( r[i] > norm ){
			norm = r[i];
		}
		if( -r[i] > norm ){
			norm = -r[i];
		}
	}
	*y = norm;
	*z -= lr;
}


/* 
 * Computes r = A*x + B*u - b
 * and      y = max([norm(r,inf), y])
 * and      z -= l'*r
 * where A is stored in column major format
 */
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_15_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	double_integrator_QP_solver_FLOAT AxBu[5];
	double_integrator_QP_solver_FLOAT norm = *y;
	double_integrator_QP_solver_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<5; i++ ){
		AxBu[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<15; j++ ){		
		for( i=0; i<5; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<5; i++ ){
		r[i] = AxBu[i] - b[i];
		lr += l[i]*r[i];
		if( r[i] > norm ){
			norm = r[i];
		}
		if( -r[i] > norm ){
			norm = -r[i];
		}
	}
	*y = norm;
	*z -= lr;
}


/*
 * Matrix vector multiplication y = M'*x where M is of size [9 x 15]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_LA_DENSE_MTVM_9_15(double_integrator_QP_solver_FLOAT *M, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<15; i++ ){
		y[i] = 0;
		for( j=0; j<9; j++ ){
			y[i] += M[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [5 x 15]
 * and B is of size [9 x 15]
 * and stored in column major format. Note the transposes of A and B!
 */
void double_integrator_QP_solver_LA_DENSE_MTVM2_5_15_9(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	int n;
	int m = 0;
	for( i=0; i<15; i++ ){
		z[i] = 0;
		for( j=0; j<5; j++ ){
			z[i] += A[k++]*x[j];
		}
		for( n=0; n<9; n++ ){
			z[i] += B[m++]*y[n];
		}
	}
}


/*
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [5 x 15] and stored in column major format.
 * and B is of size [5 x 15] and stored in diagzero format
 * Note the transposes of A and B!
 */
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	for( i=0; i<5; i++ ){
		z[i] = 0;
		for( j=0; j<5; j++ ){
			z[i] += A[k++]*x[j];
		}
		z[i] += B[i]*y[i];
	}
	for( i=5 ;i<15; i++ ){
		z[i] = 0;
		for( j=0; j<5; j++ ){
			z[i] += A[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication y = M'*x where M is of size [5 x 5]
 * and stored in diagzero format. Note the transpose of M!
 */
void double_integrator_QP_solver_LA_DIAGZERO_MTVM_5_5(double_integrator_QP_solver_FLOAT *M, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<5; i++ ){
		y[i] = M[i]*x[i];
	}
}


/*
 * Vector subtraction and addition.
 *	 Input: five vectors t, tidx, u, v, w and two scalars z and r
 *	 Output: y = t(tidx) - u + w
 *           z = z - v'*x;
 *           r = max([norm(y,inf), z]);
 * for vectors of length 15. Output z is of course scalar.
 */
void double_integrator_QP_solver_LA_VSUBADD3_15(double_integrator_QP_solver_FLOAT* t, double_integrator_QP_solver_FLOAT* u, int* uidx, double_integrator_QP_solver_FLOAT* v, double_integrator_QP_solver_FLOAT* w, double_integrator_QP_solver_FLOAT* y, double_integrator_QP_solver_FLOAT* z, double_integrator_QP_solver_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_FLOAT norm = *r;
	double_integrator_QP_solver_FLOAT vx = 0;
	double_integrator_QP_solver_FLOAT x;
	for( i=0; i<15; i++){
		x = t[i] - u[uidx[i]];
		y[i] = x + w[i];
		vx += v[i]*x;
		if( y[i] > norm ){
			norm = y[i];
		}
		if( -y[i] > norm ){
			norm = -y[i];
		}
	}
	*z -= vx;
	*r = norm;
}


/*
 * Vector subtraction and addition.
 *	 Input: five vectors t, tidx, u, v, w and two scalars z and r
 *	 Output: y = t(tidx) - u + w
 *           z = z - v'*x;
 *           r = max([norm(y,inf), z]);
 * for vectors of length 7. Output z is of course scalar.
 */
void double_integrator_QP_solver_LA_VSUBADD2_7(double_integrator_QP_solver_FLOAT* t, int* tidx, double_integrator_QP_solver_FLOAT* u, double_integrator_QP_solver_FLOAT* v, double_integrator_QP_solver_FLOAT* w, double_integrator_QP_solver_FLOAT* y, double_integrator_QP_solver_FLOAT* z, double_integrator_QP_solver_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_FLOAT norm = *r;
	double_integrator_QP_solver_FLOAT vx = 0;
	double_integrator_QP_solver_FLOAT x;
	for( i=0; i<7; i++){
		x = t[tidx[i]] - u[i];
		y[i] = x + w[i];
		vx += v[i]*x;
		if( y[i] > norm ){
			norm = y[i];
		}
		if( -y[i] > norm ){
			norm = -y[i];
		}
	}
	*z -= vx;
	*r = norm;
}


/*
 * Vector subtraction and addition.
 *	 Input: five vectors t, tidx, u, v, w and two scalars z and r
 *	 Output: y = t(tidx) - u + w
 *           z = z - v'*x;
 *           r = max([norm(y,inf), z]);
 * for vectors of length 5. Output z is of course scalar.
 */
void double_integrator_QP_solver_LA_VSUBADD3_5(double_integrator_QP_solver_FLOAT* t, double_integrator_QP_solver_FLOAT* u, int* uidx, double_integrator_QP_solver_FLOAT* v, double_integrator_QP_solver_FLOAT* w, double_integrator_QP_solver_FLOAT* y, double_integrator_QP_solver_FLOAT* z, double_integrator_QP_solver_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_FLOAT norm = *r;
	double_integrator_QP_solver_FLOAT vx = 0;
	double_integrator_QP_solver_FLOAT x;
	for( i=0; i<5; i++){
		x = t[i] - u[uidx[i]];
		y[i] = x + w[i];
		vx += v[i]*x;
		if( y[i] > norm ){
			norm = y[i];
		}
		if( -y[i] > norm ){
			norm = -y[i];
		}
	}
	*z -= vx;
	*r = norm;
}


/*
 * Vector subtraction and addition.
 *	 Input: five vectors t, tidx, u, v, w and two scalars z and r
 *	 Output: y = t(tidx) - u + w
 *           z = z - v'*x;
 *           r = max([norm(y,inf), z]);
 * for vectors of length 5. Output z is of course scalar.
 */
void double_integrator_QP_solver_LA_VSUBADD2_5(double_integrator_QP_solver_FLOAT* t, int* tidx, double_integrator_QP_solver_FLOAT* u, double_integrator_QP_solver_FLOAT* v, double_integrator_QP_solver_FLOAT* w, double_integrator_QP_solver_FLOAT* y, double_integrator_QP_solver_FLOAT* z, double_integrator_QP_solver_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_FLOAT norm = *r;
	double_integrator_QP_solver_FLOAT vx = 0;
	double_integrator_QP_solver_FLOAT x;
	for( i=0; i<5; i++){
		x = t[tidx[i]] - u[i];
		y[i] = x + w[i];
		vx += v[i]*x;
		if( y[i] > norm ){
			norm = y[i];
		}
		if( -y[i] > norm ){
			norm = -y[i];
		}
	}
	*z -= vx;
	*r = norm;
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 15
 * Returns also L/S, a value that is often used elsewhere.
 */
void double_integrator_QP_solver_LA_INEQ_B_GRAD_15_15_7(double_integrator_QP_solver_FLOAT *lu, double_integrator_QP_solver_FLOAT *su, double_integrator_QP_solver_FLOAT *ru, double_integrator_QP_solver_FLOAT *ll, double_integrator_QP_solver_FLOAT *sl, double_integrator_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_FLOAT *grad, double_integrator_QP_solver_FLOAT *lubysu, double_integrator_QP_solver_FLOAT *llbysl)
{
	int i;
	for( i=0; i<15; i++ ){
		grad[i] = 0;
	}
	for( i=0; i<15; i++ ){		
		llbysl[i] = ll[i] / sl[i];
		grad[lbIdx[i]] -= llbysl[i]*rl[i];
	}
	for( i=0; i<7; i++ ){
		lubysu[i] = lu[i] / su[i];
		grad[ubIdx[i]] += lubysu[i]*ru[i];
	}
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 5
 * Returns also L/S, a value that is often used elsewhere.
 */
void double_integrator_QP_solver_LA_INEQ_B_GRAD_5_5_5(double_integrator_QP_solver_FLOAT *lu, double_integrator_QP_solver_FLOAT *su, double_integrator_QP_solver_FLOAT *ru, double_integrator_QP_solver_FLOAT *ll, double_integrator_QP_solver_FLOAT *sl, double_integrator_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_FLOAT *grad, double_integrator_QP_solver_FLOAT *lubysu, double_integrator_QP_solver_FLOAT *llbysl)
{
	int i;
	for( i=0; i<5; i++ ){
		grad[i] = 0;
	}
	for( i=0; i<5; i++ ){		
		llbysl[i] = ll[i] / sl[i];
		grad[lbIdx[i]] -= llbysl[i]*rl[i];
	}
	for( i=0; i<5; i++ ){
		lubysu[i] = lu[i] / su[i];
		grad[ubIdx[i]] += lubysu[i]*ru[i];
	}
}


/*
 * Addition of three vectors  z = u + w + v
 * of length 170.
 */
void double_integrator_QP_solver_LA_VVADD3_170(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *w, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<170; i++ ){
		z[i] = u[i] + v[i] + w[i];
	}
}


/*
 * Special function to compute the diagonal cholesky factorization of the 
 * positive definite augmented Hessian for block size 15.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = sqrt(H + diag(llbysl) + diag(lubysu))
 * where Phi is stored in diagonal storage format
 */
void double_integrator_QP_solver_LA_DIAG_CHOL_LBUB_15_15_7(double_integrator_QP_solver_FLOAT *H, double_integrator_QP_solver_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_FLOAT *Phi)


{
	int i;
	
	/* copy  H into PHI */
	for( i=0; i<15; i++ ){
		Phi[i] = H[i];
	}

	/* add llbysl onto Phi where necessary */
	for( i=0; i<15; i++ ){
		Phi[lbIdx[i]] += llbysl[i];
	}

	/* add lubysu onto Phi where necessary */
	for( i=0; i<7; i++){
		Phi[ubIdx[i]] +=  lubysu[i];
	}
	
	/* compute cholesky */
	for(i=0; i<15; i++)
	{
#if double_integrator_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
		if( Phi[i] < 1.0000000000000000E-013 )
		{
            PRINTTEXT("WARNING: small pivot in Cholesky fact. (=%3.1e < eps=%3.1e), regularizing to %3.1e\n",Phi[i],1.0000000000000000E-013,4.0000000000000002E-004);
			Phi[i] = 2.0000000000000000E-002;
		}
		else
		{
			Phi[i] = sqrt(Phi[i]);
		}
#else
		Phi[i] = Phi[i] < 1.0000000000000000E-013 ? 2.0000000000000000E-002 : sqrt(Phi[i]);
#endif
	}

}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [9 x 15],
 * B is given and of size [9 x 15], L is a diagonal
 * matrix of size 9 stored in diagonal matrix 
 * storage format. Note the transpose of L has no impact!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_15(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j;
	 int k = 0;

	for( j=0; j<15; j++){
		for( i=0; i<9; i++){
			A[k] = B[k]/L[j];
			k++;
		}
	}

}


/**
 * Forward substitution to solve L*y = b where L is a
 * diagonal matrix in vector storage format.
 * 
 * The dimensions involved are 15.
 */
void double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *y)
{
    int i;

    for( i=0; i<15; i++ ){
		y[i] = b[i]/L[i];
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [5 x 15],
 * B is given and of size [5 x 15], L is a diagonal
 * matrix of size 5 stored in diagonal matrix 
 * storage format. Note the transpose of L has no impact!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_15(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j;
	 int k = 0;

	for( j=0; j<15; j++){
		for( i=0; i<5; i++){
			A[k] = B[k]/L[j];
			k++;
		}
	}

}


/**
 * Compute C = A*B' where 
 *
 *	size(A) = [9 x 15]
 *  size(B) = [5 x 15]
 * 
 * and all matrices are stored in column major format.
 *
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE.  
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMTM_9_15_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *C)
{
    int i, j, k;
    double_integrator_QP_solver_FLOAT temp;
    
    for( i=0; i<9; i++ ){        
        for( j=0; j<5; j++ ){
            temp = 0; 
            for( k=0; k<15; k++ ){
                temp += A[k*9+i]*B[k*5+j];
            }						
            C[j*9+i] = temp;
        }
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [5 x 15],
 * B is given and of size [5 x 15], L is a diagonal
 *  matrix of size 15 stored in diagonal 
 * storage format. Note the transpose of L!
 *
 * Result: A in diagonalzero storage format.
 *
 */
void double_integrator_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_15(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
	int j;
    for( j=0; j<15; j++ ){   
		A[j] = B[j]/L[j];
     }
}


/**
 * Compute C = A*B' where 
 *
 *	size(A) = [5 x 15]
 *  size(B) = [5 x 15] in diagzero format
 * 
 * A and C matrices are stored in column major format.
 * 
 * 
 */
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_15_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *C)
{
    int i, j;
	
	for( i=0; i<5; i++ ){
		for( j=0; j<5; j++){
			C[j*5+i] = B[i*5+j]*A[i];
		}
	}

}


/*
 * Special function to compute the diagonal cholesky factorization of the 
 * positive definite augmented Hessian for block size 5.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = sqrt(H + diag(llbysl) + diag(lubysu))
 * where Phi is stored in diagonal storage format
 */
void double_integrator_QP_solver_LA_DIAG_CHOL_ONELOOP_LBUB_5_5_5(double_integrator_QP_solver_FLOAT *H, double_integrator_QP_solver_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_FLOAT *Phi)


{
	int i;
	
	/* compute cholesky */
	for( i=0; i<5; i++ ){
		Phi[i] = H[i] + llbysl[i] + lubysu[i];

#if double_integrator_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
		if( Phi[i] < 1.0000000000000000E-013 )
		{
            PRINTTEXT("WARNING: small pivot in Cholesky fact. (=%3.1e < eps=%3.1e), regularizing to %3.1e\n",Phi[i],1.0000000000000000E-013,4.0000000000000002E-004);
			Phi[i] = 2.0000000000000000E-002;
		}
		else
		{
			Phi[i] = sqrt(Phi[i]);
		}
#else
		Phi[i] = Phi[i] < 1.0000000000000000E-013 ? 2.0000000000000000E-002 : sqrt(Phi[i]);
#endif
	}
	
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [5 x 5],
 * B is given and of size [5 x 5], L is a diagonal
 *  matrix of size 5 stored in diagonal 
 * storage format. Note the transpose of L!
 *
 * Result: A in diagonalzero storage format.
 *
 */
void double_integrator_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_5(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
	int j;
    for( j=0; j<5; j++ ){   
		A[j] = B[j]/L[j];
     }
}


/**
 * Forward substitution to solve L*y = b where L is a
 * diagonal matrix in vector storage format.
 * 
 * The dimensions involved are 5.
 */
void double_integrator_QP_solver_LA_DIAG_FORWARDSUB_5(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *y)
{
    int i;

    for( i=0; i<5; i++ ){
		y[i] = b[i]/L[i];
    }
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [9 x 15] in column
 * storage format, and B is of size [9 x 15] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMT2_9_15_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<9; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<15; k++ ){
                ltemp += A[k*9+i]*A[k*9+j];
            }			
			for( k=0; k<15; k++ ){
                ltemp += B[k*9+i]*B[k*9+j];
            }
            L[ii+j] = ltemp;
        }
        ii += ++di;
    }
}


/* 
 * Computes r = b - A*x - B*u
 * where A an B are stored in column major format
 */
void double_integrator_QP_solver_LA_DENSE_MVMSUB2_9_15_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<9; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<15; j++ ){		
		for( i=0; i<9; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<15; n++ ){
		for( i=0; i<9; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [5 x 15] in column
 * storage format, and B is of size [5 x 15] diagonalzero
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_15_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<5; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<15; k++ ){
                ltemp += A[k*5+i]*A[k*5+j];
            }		
            L[ii+j] = ltemp;
        }
		/* work on the diagonal
		 * there might be i == j, but j has already been incremented so it is i == j-1 */
		L[ii+i] += B[i]*B[i];
        ii += ++di;
    }
}


/* 
 * Computes r = b - A*x - B*u
 * where A is stored in column major format
 * and B is stored in diagzero format
 */
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_15_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<5; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[i]*u[i];
	}	

	for( j=1; j<15; j++ ){		
		for( i=0; i<5; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [5 x 15] in column
 * storage format, and B is of size [5 x 5] diagonalzero
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_15_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<5; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<15; k++ ){
                ltemp += A[k*5+i]*A[k*5+j];
            }		
            L[ii+j] = ltemp;
        }
		/* work on the diagonal
		 * there might be i == j, but j has already been incremented so it is i == j-1 */
		L[ii+i] += B[i]*B[i];
        ii += ++di;
    }
}


/* 
 * Computes r = b - A*x - B*u
 * where A is stored in column major format
 * and B is stored in diagzero format
 */
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_15_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<5; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[i]*u[i];
	}	

	for( j=1; j<15; j++ ){		
		for( i=0; i<5; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 9 and outputting
 * the Cholesky factor to matrix L in lower triangular format.
 */
void double_integrator_QP_solver_LA_DENSE_CHOL_9(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    double_integrator_QP_solver_FLOAT l;
    double_integrator_QP_solver_FLOAT Mii;

	/* copy A to L first and then operate on L */
	/* COULD BE OPTIMIZED */
	ii=0; di=0;
	for( i=0; i<9; i++ ){
		for( j=0; j<=i; j++ ){
			L[ii+j] = A[ii+j];
		}
		ii += ++di;
	}    
	
	/* factor L */
	ii=0; di=0;
    for( i=0; i<9; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += L[ii+k]*L[ii+k];
        }        
        
        Mii = L[ii+i] - l;
        
#if double_integrator_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
        if( Mii < 1.0000000000000000E-013 ){
             PRINTTEXT("WARNING (CHOL): small %d-th pivot in Cholesky fact. (=%3.1e < eps=%3.1e), regularizing to %3.1e\n",i,Mii,1.0000000000000000E-013,4.0000000000000002E-004);
			 L[ii+i] = 2.0000000000000000E-002;
		} else
		{
			L[ii+i] = sqrt(Mii);
		}
#else
		L[ii+i] = Mii < 1.0000000000000000E-013 ? 2.0000000000000000E-002 : sqrt(Mii);
#endif

		jj = ((i+1)*(i+2))/2; dj = i+1;
        for( j=i+1; j<9; j++ ){
            l = 0;            
            for( k=0; k<i; k++ ){
                l += L[jj+k]*L[ii+k];
            }

			/* saturate values for numerical stability */
			l = MIN(l,  BIGMM);
			l = MAX(l, -BIGMM);

            L[jj+i] = (L[jj+i] - l)/L[ii+i];            
			jj += ++dj;
        }
		ii += ++di;
    }	
}


/**
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 9.
 */
void double_integrator_QP_solver_LA_DENSE_FORWARDSUB_9(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<9; i++ ){
        yel = b[i];        
        for( j=0; j<i; j++ ){
            yel -= y[j]*L[ii+j];
        }

		/* saturate for numerical stability  */
		yel = MIN(yel, BIGM);
		yel = MAX(yel, -BIGM);

        y[i] = yel / L[ii+i];
        ii += ++di;
    }
}


/** 
 * Forward substitution for the matrix equation A*L' = B'
 * where A is to be computed and is of size [5 x 9],
 * B is given and of size [5 x 9], L is a lower tri-
 * angular matrix of size 9 stored in lower triangular 
 * storage format. Note the transpose of L AND B!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_9(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j,k,ii,di;
    double_integrator_QP_solver_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<9; j++ ){        
        for( i=0; i<5; i++ ){
            a = B[i*9+j];
            for( k=0; k<j; k++ ){
                a -= A[k*5+i]*L[ii+k];
            }    

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

			A[j*5+i] = a/L[ii+j];			
        }
        ii += ++di;
    }
}


/**
 * Compute L = L - A*A', where L is lower triangular of size 5
 * and A is a dense matrix of size [5 x 9] in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMTSUB_5_9(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<5; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<9; k++ ){
                ltemp += A[k*5+i]*A[k*5+j];
            }						
            L[ii+j] -= ltemp;
        }
        ii += ++di;
    }
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 5 and outputting
 * the Cholesky factor to matrix L in lower triangular format.
 */
void double_integrator_QP_solver_LA_DENSE_CHOL_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    double_integrator_QP_solver_FLOAT l;
    double_integrator_QP_solver_FLOAT Mii;

	/* copy A to L first and then operate on L */
	/* COULD BE OPTIMIZED */
	ii=0; di=0;
	for( i=0; i<5; i++ ){
		for( j=0; j<=i; j++ ){
			L[ii+j] = A[ii+j];
		}
		ii += ++di;
	}    
	
	/* factor L */
	ii=0; di=0;
    for( i=0; i<5; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += L[ii+k]*L[ii+k];
        }        
        
        Mii = L[ii+i] - l;
        
#if double_integrator_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
        if( Mii < 1.0000000000000000E-013 ){
             PRINTTEXT("WARNING (CHOL): small %d-th pivot in Cholesky fact. (=%3.1e < eps=%3.1e), regularizing to %3.1e\n",i,Mii,1.0000000000000000E-013,4.0000000000000002E-004);
			 L[ii+i] = 2.0000000000000000E-002;
		} else
		{
			L[ii+i] = sqrt(Mii);
		}
#else
		L[ii+i] = Mii < 1.0000000000000000E-013 ? 2.0000000000000000E-002 : sqrt(Mii);
#endif

		jj = ((i+1)*(i+2))/2; dj = i+1;
        for( j=i+1; j<5; j++ ){
            l = 0;            
            for( k=0; k<i; k++ ){
                l += L[jj+k]*L[ii+k];
            }

			/* saturate values for numerical stability */
			l = MIN(l,  BIGMM);
			l = MAX(l, -BIGMM);

            L[jj+i] = (L[jj+i] - l)/L[ii+i];            
			jj += ++dj;
        }
		ii += ++di;
    }	
}


/* 
 * Computes r = b - A*x
 * where A is stored in column major format
 */
void double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_9(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<5; i++ ){
		r[i] = b[i] - A[k++]*x[0];
	}	
	for( j=1; j<9; j++ ){		
		for( i=0; i<5; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/**
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 5.
 */
void double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<5; i++ ){
        yel = b[i];        
        for( j=0; j<i; j++ ){
            yel -= y[j]*L[ii+j];
        }

		/* saturate for numerical stability  */
		yel = MIN(yel, BIGM);
		yel = MAX(yel, -BIGM);

        y[i] = yel / L[ii+i];
        ii += ++di;
    }
}


/** 
 * Forward substitution for the matrix equation A*L' = B'
 * where A is to be computed and is of size [5 x 5],
 * B is given and of size [5 x 5], L is a lower tri-
 * angular matrix of size 5 stored in lower triangular 
 * storage format. Note the transpose of L AND B!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j,k,ii,di;
    double_integrator_QP_solver_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<5; j++ ){        
        for( i=0; i<5; i++ ){
            a = B[i*5+j];
            for( k=0; k<j; k++ ){
                a -= A[k*5+i]*L[ii+k];
            }    

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

			A[j*5+i] = a/L[ii+j];			
        }
        ii += ++di;
    }
}


/**
 * Compute L = L - A*A', where L is lower triangular of size 5
 * and A is a dense matrix of size [5 x 5] in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMTSUB_5_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<5; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<5; k++ ){
                ltemp += A[k*5+i]*A[k*5+j];
            }						
            L[ii+j] -= ltemp;
        }
        ii += ++di;
    }
}


/* 
 * Computes r = b - A*x
 * where A is stored in column major format
 */
void double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<5; i++ ){
		r[i] = b[i] - A[k++]*x[0];
	}	
	for( j=1; j<5; j++ ){		
		for( i=0; i<5; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/**
 * Backward Substitution to solve L^T*x = y where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * All involved dimensions are 5.
 */
void double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_FLOAT xel;    
	int start = 10;
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 4;
    for( i=4; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 4;
        for( j=4; j>i; j-- ){
            xel -= x[j]*L[jj+i];
            jj -= dj--;
        }

		/* saturate for numerical stability */
		xel = MIN(xel, BIGM);
		xel = MAX(xel, -BIGM); 

        x[i] = xel / L[ii+i];
        ii -= di--;
    }
}


/*
 * Matrix vector multiplication y = b - M'*x where M is of size [5 x 5]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<5; i++ ){
		r[i] = b[i];
		for( j=0; j<5; j++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication y = b - M'*x where M is of size [5 x 9]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_9(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<9; i++ ){
		r[i] = b[i];
		for( j=0; j<5; j++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/**
 * Backward Substitution to solve L^T*x = y where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * All involved dimensions are 9.
 */
void double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_9(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_FLOAT xel;    
	int start = 36;
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 8;
    for( i=8; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 8;
        for( j=8; j>i; j-- ){
            xel -= x[j]*L[jj+i];
            jj -= dj--;
        }

		/* saturate for numerical stability */
		xel = MIN(xel, BIGM);
		xel = MAX(xel, -BIGM); 

        x[i] = xel / L[ii+i];
        ii -= di--;
    }
}


/*
 * Vector subtraction z = -x - y for vectors of length 170.
 */
void double_integrator_QP_solver_LA_VSUB2_170(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<170; i++){
		z[i] = -x[i] - y[i];
	}
}


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * diagonal matrix of size 15 in vector
 * storage format.
 */
void double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *x)
{
    int i;
            
    /* solve Ly = b by forward and backward substitution */
    for( i=0; i<15; i++ ){
		x[i] = b[i]/(L[i]*L[i]);
    }
    
}


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * diagonal matrix of size 5 in vector
 * storage format.
 */
void double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_5(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *x)
{
    int i;
            
    /* solve Ly = b by forward and backward substitution */
    for( i=0; i<5; i++ ){
		x[i] = b[i]/(L[i]*L[i]);
    }
    
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 15,
 * and x has length 15 and is indexed through yidx.
 */
void double_integrator_QP_solver_LA_VSUB_INDEXED_15(double_integrator_QP_solver_FLOAT *x, int* xidx, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<15; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 15.
 */
void double_integrator_QP_solver_LA_VSUB3_15(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *w, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<15; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = -x - y(yidx) where y is of length 15
 * and z, x and yidx are of length 7.
 */
void double_integrator_QP_solver_LA_VSUB2_INDEXED_7(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y, int* yidx, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<7; i++){
		z[i] = -x[i] - y[yidx[i]];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 7.
 */
void double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *w, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<7; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 5,
 * and x has length 5 and is indexed through yidx.
 */
void double_integrator_QP_solver_LA_VSUB_INDEXED_5(double_integrator_QP_solver_FLOAT *x, int* xidx, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<5; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 5.
 */
void double_integrator_QP_solver_LA_VSUB3_5(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *w, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<5; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = -x - y(yidx) where y is of length 5
 * and z, x and yidx are of length 5.
 */
void double_integrator_QP_solver_LA_VSUB2_INDEXED_5(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y, int* yidx, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<5; i++){
		z[i] = -x[i] - y[yidx[i]];
	}
}


/**
 * Backtracking line search.
 * 
 * First determine the maximum line length by a feasibility line
 * search, i.e. a ~= argmax{ a \in [0...1] s.t. l+a*dl >= 0 and s+a*ds >= 0}.
 *
 * The function returns either the number of iterations or exits the error code
 * double_integrator_QP_solver_NOPROGRESS (should be negative).
 */
int double_integrator_QP_solver_LINESEARCH_BACKTRACKING_AFFINE(double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *dl, double_integrator_QP_solver_FLOAT *ds, double_integrator_QP_solver_FLOAT *a, double_integrator_QP_solver_FLOAT *mu_aff)
{
    int i;
	int lsIt=1;    
    double_integrator_QP_solver_FLOAT dltemp;
    double_integrator_QP_solver_FLOAT dstemp;
    double_integrator_QP_solver_FLOAT mya = 1.0;
    double_integrator_QP_solver_FLOAT mymu;
        
    while( 1 ){                        

        /* 
         * Compute both snew and wnew together.
         * We compute also mu_affine along the way here, as the
         * values might be in registers, so it should be cheaper.
         */
        mymu = 0;
        for( i=0; i<252; i++ ){
            dltemp = l[i] + mya*dl[i];
            dstemp = s[i] + mya*ds[i];
            if( dltemp < 0 || dstemp < 0 ){
                lsIt++;
                break;
            } else {                
                mymu += dstemp*dltemp;
            }
        }
        
        /* 
         * If no early termination of the for-loop above occurred, we
         * found the required value of a and we can quit the while loop.
         */
        if( i == 252 ){
            break;
        } else {
            mya *= double_integrator_QP_solver_SET_LS_SCALE_AFF;
            if( mya < double_integrator_QP_solver_SET_LS_MINSTEP ){
                return double_integrator_QP_solver_NOPROGRESS;
            }
        }
    }
    
    /* return new values and iteration counter */
    *a = mya;
    *mu_aff = mymu / (double_integrator_QP_solver_FLOAT)252;
    return lsIt;
}


/*
 * Vector subtraction x = (u.*v - mu)*sigma where a is a scalar
*  and x,u,v are vectors of length 252.
 */
void double_integrator_QP_solver_LA_VSUB5_252(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT mu,  double_integrator_QP_solver_FLOAT sigma, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<252; i++){
		x[i] = u[i]*v[i] - mu;
		x[i] *= sigma;
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 15,
 * u, su, uidx are of length 7 and v, sv, vidx are of length 15.
 */
void double_integrator_QP_solver_LA_VSUB6_INDEXED_15_7_15(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *su, int* uidx, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *sv, int* vidx, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<15; i++ ){
		x[i] = 0;
	}
	for( i=0; i<7; i++){
		x[uidx[i]] += u[i]/su[i];
	}
	for( i=0; i<15; i++){
		x[vidx[i]] -= v[i]/sv[i];
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void double_integrator_QP_solver_LA_DENSE_2MVMADD_9_15_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<9; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<15; j++ ){		
		for( i=0; i<9; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<15; n++ ){
		for( i=0; i<9; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/* 
 * Computes r = A*x + B*u
 * where A is stored in column major format
 * and B is stored in diagzero format
 */
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_15_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<5; i++ ){
		r[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<15; j++ ){		
		for( i=0; i<5; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 5,
 * u, su, uidx are of length 5 and v, sv, vidx are of length 5.
 */
void double_integrator_QP_solver_LA_VSUB6_INDEXED_5_5_5(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *su, int* uidx, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *sv, int* vidx, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<5; i++ ){
		x[i] = 0;
	}
	for( i=0; i<5; i++){
		x[uidx[i]] += u[i]/su[i];
	}
	for( i=0; i<5; i++){
		x[vidx[i]] -= v[i]/sv[i];
	}
}


/* 
 * Computes r = A*x + B*u
 * where A is stored in column major format
 * and B is stored in diagzero format
 */
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_15_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<5; i++ ){
		r[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<15; j++ ){		
		for( i=0; i<5; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
}


/*
 * Vector subtraction z = x - y for vectors of length 170.
 */
void double_integrator_QP_solver_LA_VSUB_170(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<170; i++){
		z[i] = x[i] - y[i];
	}
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 15 (length of y >= 15).
 */
void double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_15(double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *y, int* yidx, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<15; i++ ){
		z[i] = -r[i]/s[i] - u[i]*y[yidx[i]];
	}
}


/** 
 * Computes z = -r./s + u.*y(y)
 * where all vectors except of y are of length 7 (length of y >= 7).
 */
void double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_7(double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *y, int* yidx, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<7; i++ ){
		z[i] = -r[i]/s[i] + u[i]*y[yidx[i]];
	}
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 5 (length of y >= 5).
 */
void double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_5(double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *y, int* yidx, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<5; i++ ){
		z[i] = -r[i]/s[i] - u[i]*y[yidx[i]];
	}
}


/** 
 * Computes z = -r./s + u.*y(y)
 * where all vectors except of y are of length 5 (length of y >= 5).
 */
void double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_5(double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *y, int* yidx, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<5; i++ ){
		z[i] = -r[i]/s[i] + u[i]*y[yidx[i]];
	}
}


/*
 * Computes ds = -l.\(r + s.*dl) for vectors of length 252.
 */
void double_integrator_QP_solver_LA_VSUB7_252(double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *dl, double_integrator_QP_solver_FLOAT *ds)
{
	int i;
	for( i=0; i<252; i++){
		ds[i] = -(r[i] + s[i]*dl[i])/l[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 170.
 */
void double_integrator_QP_solver_LA_VADD_170(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<170; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 59.
 */
void double_integrator_QP_solver_LA_VADD_59(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<59; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 252.
 */
void double_integrator_QP_solver_LA_VADD_252(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<252; i++){
		x[i] += y[i];
	}
}


/**
 * Backtracking line search for combined predictor/corrector step.
 * Update on variables with safety factor gamma (to keep us away from
 * boundary).
 */
int double_integrator_QP_solver_LINESEARCH_BACKTRACKING_COMBINED(double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *dz, double_integrator_QP_solver_FLOAT *dv, double_integrator_QP_solver_FLOAT *dl, double_integrator_QP_solver_FLOAT *ds, double_integrator_QP_solver_FLOAT *a, double_integrator_QP_solver_FLOAT *mu)
{
    int i, lsIt=1;       
    double_integrator_QP_solver_FLOAT dltemp;
    double_integrator_QP_solver_FLOAT dstemp;    
    double_integrator_QP_solver_FLOAT a_gamma;
            
    *a = 1.0;
    while( 1 ){                        

        /* check whether search criterion is fulfilled */
        for( i=0; i<252; i++ ){
            dltemp = l[i] + (*a)*dl[i];
            dstemp = s[i] + (*a)*ds[i];
            if( dltemp < 0 || dstemp < 0 ){
                lsIt++;
                break;
            }
        }
        
        /* 
         * If no early termination of the for-loop above occurred, we
         * found the required value of a and we can quit the while loop.
         */
        if( i == 252 ){
            break;
        } else {
            *a *= double_integrator_QP_solver_SET_LS_SCALE;
            if( *a < double_integrator_QP_solver_SET_LS_MINSTEP ){
                return double_integrator_QP_solver_NOPROGRESS;
            }
        }
    }
    
    /* update variables with safety margin */
    a_gamma = (*a)*double_integrator_QP_solver_SET_LS_MAXSTEP;
    
    /* primal variables */
    for( i=0; i<170; i++ ){
        z[i] += a_gamma*dz[i];
    }
    
    /* equality constraint multipliers */
    for( i=0; i<59; i++ ){
        v[i] += a_gamma*dv[i];
    }
    
    /* inequality constraint multipliers & slacks, also update mu */
    *mu = 0;
    for( i=0; i<252; i++ ){
        dltemp = l[i] + a_gamma*dl[i]; l[i] = dltemp;
        dstemp = s[i] + a_gamma*ds[i]; s[i] = dstemp;
        *mu += dltemp*dstemp;
    }
    
    *a = a_gamma;
    *mu /= (double_integrator_QP_solver_FLOAT)252;
    return lsIt;
}




/* VARIABLE DEFINITIONS ------------------------------------------------ */
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_z[170];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_v[59];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dz_aff[170];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dv_aff[59];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_grad_cost[170];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_grad_eq[170];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rd[170];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_l[252];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_s[252];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_lbys[252];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dl_aff[252];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ds_aff[252];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dz_cc[170];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dv_cc[59];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dl_cc[252];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ds_cc[252];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ccrhs[252];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_grad_ineq[170];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_H00[15] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z00 = double_integrator_QP_solver_z + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff00 = double_integrator_QP_solver_dz_aff + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc00 = double_integrator_QP_solver_dz_cc + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd00 = double_integrator_QP_solver_rd + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd00[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost00 = double_integrator_QP_solver_grad_cost + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq00 = double_integrator_QP_solver_grad_eq + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq00 = double_integrator_QP_solver_grad_ineq + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv00[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v00 = double_integrator_QP_solver_v + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re00[9];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta00[9];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc00[9];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff00 = double_integrator_QP_solver_dv_aff + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc00 = double_integrator_QP_solver_dv_cc + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V00[135];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd00[45];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld00[45];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy00[9];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy00[9];
int double_integrator_QP_solver_lbIdx00[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb00 = double_integrator_QP_solver_l + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb00 = double_integrator_QP_solver_s + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb00 = double_integrator_QP_solver_lbys + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb00[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff00 = double_integrator_QP_solver_dl_aff + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff00 = double_integrator_QP_solver_ds_aff + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc00 = double_integrator_QP_solver_dl_cc + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc00 = double_integrator_QP_solver_ds_cc + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl00 = double_integrator_QP_solver_ccrhs + 0;
int double_integrator_QP_solver_ubIdx00[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub00 = double_integrator_QP_solver_l + 15;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub00 = double_integrator_QP_solver_s + 15;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub00 = double_integrator_QP_solver_lbys + 15;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub00[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff00 = double_integrator_QP_solver_dl_aff + 15;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff00 = double_integrator_QP_solver_ds_aff + 15;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc00 = double_integrator_QP_solver_dl_cc + 15;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc00 = double_integrator_QP_solver_ds_cc + 15;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub00 = double_integrator_QP_solver_ccrhs + 15;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi00[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z01 = double_integrator_QP_solver_z + 15;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff01 = double_integrator_QP_solver_dz_aff + 15;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc01 = double_integrator_QP_solver_dz_cc + 15;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd01 = double_integrator_QP_solver_rd + 15;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd01[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost01 = double_integrator_QP_solver_grad_cost + 15;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq01 = double_integrator_QP_solver_grad_eq + 15;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq01 = double_integrator_QP_solver_grad_ineq + 15;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv01[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v01 = double_integrator_QP_solver_v + 9;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re01[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta01[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc01[5];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff01 = double_integrator_QP_solver_dv_aff + 9;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc01 = double_integrator_QP_solver_dv_cc + 9;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V01[75];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd01[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld01[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy01[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy01[5];
int double_integrator_QP_solver_lbIdx01[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb01 = double_integrator_QP_solver_l + 22;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb01 = double_integrator_QP_solver_s + 22;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb01 = double_integrator_QP_solver_lbys + 22;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb01[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff01 = double_integrator_QP_solver_dl_aff + 22;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff01 = double_integrator_QP_solver_ds_aff + 22;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc01 = double_integrator_QP_solver_dl_cc + 22;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc01 = double_integrator_QP_solver_ds_cc + 22;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl01 = double_integrator_QP_solver_ccrhs + 22;
int double_integrator_QP_solver_ubIdx01[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub01 = double_integrator_QP_solver_l + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub01 = double_integrator_QP_solver_s + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub01 = double_integrator_QP_solver_lbys + 37;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub01[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff01 = double_integrator_QP_solver_dl_aff + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff01 = double_integrator_QP_solver_ds_aff + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc01 = double_integrator_QP_solver_dl_cc + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc01 = double_integrator_QP_solver_ds_cc + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub01 = double_integrator_QP_solver_ccrhs + 37;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi01[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_D01[135] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W01[135];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd01[45];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd01[45];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z02 = double_integrator_QP_solver_z + 30;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff02 = double_integrator_QP_solver_dz_aff + 30;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc02 = double_integrator_QP_solver_dz_cc + 30;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd02 = double_integrator_QP_solver_rd + 30;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd02[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost02 = double_integrator_QP_solver_grad_cost + 30;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq02 = double_integrator_QP_solver_grad_eq + 30;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq02 = double_integrator_QP_solver_grad_ineq + 30;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv02[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v02 = double_integrator_QP_solver_v + 14;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re02[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta02[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc02[5];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff02 = double_integrator_QP_solver_dv_aff + 14;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc02 = double_integrator_QP_solver_dv_cc + 14;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V02[75];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd02[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld02[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy02[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy02[5];
int double_integrator_QP_solver_lbIdx02[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb02 = double_integrator_QP_solver_l + 44;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb02 = double_integrator_QP_solver_s + 44;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb02 = double_integrator_QP_solver_lbys + 44;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb02[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff02 = double_integrator_QP_solver_dl_aff + 44;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff02 = double_integrator_QP_solver_ds_aff + 44;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc02 = double_integrator_QP_solver_dl_cc + 44;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc02 = double_integrator_QP_solver_ds_cc + 44;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl02 = double_integrator_QP_solver_ccrhs + 44;
int double_integrator_QP_solver_ubIdx02[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub02 = double_integrator_QP_solver_l + 59;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub02 = double_integrator_QP_solver_s + 59;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub02 = double_integrator_QP_solver_lbys + 59;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub02[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff02 = double_integrator_QP_solver_dl_aff + 59;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff02 = double_integrator_QP_solver_ds_aff + 59;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc02 = double_integrator_QP_solver_dl_cc + 59;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc02 = double_integrator_QP_solver_ds_cc + 59;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub02 = double_integrator_QP_solver_ccrhs + 59;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi02[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_D02[15] = {-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000};
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W02[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd02[25];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd02[25];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z03 = double_integrator_QP_solver_z + 45;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff03 = double_integrator_QP_solver_dz_aff + 45;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc03 = double_integrator_QP_solver_dz_cc + 45;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd03 = double_integrator_QP_solver_rd + 45;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd03[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost03 = double_integrator_QP_solver_grad_cost + 45;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq03 = double_integrator_QP_solver_grad_eq + 45;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq03 = double_integrator_QP_solver_grad_ineq + 45;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv03[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v03 = double_integrator_QP_solver_v + 19;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re03[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta03[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc03[5];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff03 = double_integrator_QP_solver_dv_aff + 19;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc03 = double_integrator_QP_solver_dv_cc + 19;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V03[75];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd03[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld03[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy03[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy03[5];
int double_integrator_QP_solver_lbIdx03[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb03 = double_integrator_QP_solver_l + 66;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb03 = double_integrator_QP_solver_s + 66;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb03 = double_integrator_QP_solver_lbys + 66;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb03[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff03 = double_integrator_QP_solver_dl_aff + 66;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff03 = double_integrator_QP_solver_ds_aff + 66;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc03 = double_integrator_QP_solver_dl_cc + 66;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc03 = double_integrator_QP_solver_ds_cc + 66;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl03 = double_integrator_QP_solver_ccrhs + 66;
int double_integrator_QP_solver_ubIdx03[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub03 = double_integrator_QP_solver_l + 81;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub03 = double_integrator_QP_solver_s + 81;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub03 = double_integrator_QP_solver_lbys + 81;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub03[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff03 = double_integrator_QP_solver_dl_aff + 81;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff03 = double_integrator_QP_solver_ds_aff + 81;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc03 = double_integrator_QP_solver_dl_cc + 81;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc03 = double_integrator_QP_solver_ds_cc + 81;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub03 = double_integrator_QP_solver_ccrhs + 81;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi03[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W03[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd03[25];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd03[25];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z04 = double_integrator_QP_solver_z + 60;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff04 = double_integrator_QP_solver_dz_aff + 60;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc04 = double_integrator_QP_solver_dz_cc + 60;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd04 = double_integrator_QP_solver_rd + 60;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd04[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost04 = double_integrator_QP_solver_grad_cost + 60;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq04 = double_integrator_QP_solver_grad_eq + 60;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq04 = double_integrator_QP_solver_grad_ineq + 60;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv04[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v04 = double_integrator_QP_solver_v + 24;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re04[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta04[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc04[5];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff04 = double_integrator_QP_solver_dv_aff + 24;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc04 = double_integrator_QP_solver_dv_cc + 24;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V04[75];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd04[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld04[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy04[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy04[5];
int double_integrator_QP_solver_lbIdx04[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb04 = double_integrator_QP_solver_l + 88;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb04 = double_integrator_QP_solver_s + 88;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb04 = double_integrator_QP_solver_lbys + 88;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb04[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff04 = double_integrator_QP_solver_dl_aff + 88;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff04 = double_integrator_QP_solver_ds_aff + 88;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc04 = double_integrator_QP_solver_dl_cc + 88;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc04 = double_integrator_QP_solver_ds_cc + 88;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl04 = double_integrator_QP_solver_ccrhs + 88;
int double_integrator_QP_solver_ubIdx04[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub04 = double_integrator_QP_solver_l + 103;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub04 = double_integrator_QP_solver_s + 103;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub04 = double_integrator_QP_solver_lbys + 103;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub04[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff04 = double_integrator_QP_solver_dl_aff + 103;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff04 = double_integrator_QP_solver_ds_aff + 103;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc04 = double_integrator_QP_solver_dl_cc + 103;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc04 = double_integrator_QP_solver_ds_cc + 103;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub04 = double_integrator_QP_solver_ccrhs + 103;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi04[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W04[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd04[25];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd04[25];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z05 = double_integrator_QP_solver_z + 75;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff05 = double_integrator_QP_solver_dz_aff + 75;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc05 = double_integrator_QP_solver_dz_cc + 75;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd05 = double_integrator_QP_solver_rd + 75;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd05[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost05 = double_integrator_QP_solver_grad_cost + 75;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq05 = double_integrator_QP_solver_grad_eq + 75;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq05 = double_integrator_QP_solver_grad_ineq + 75;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv05[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v05 = double_integrator_QP_solver_v + 29;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re05[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta05[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc05[5];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff05 = double_integrator_QP_solver_dv_aff + 29;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc05 = double_integrator_QP_solver_dv_cc + 29;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V05[75];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd05[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld05[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy05[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy05[5];
int double_integrator_QP_solver_lbIdx05[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb05 = double_integrator_QP_solver_l + 110;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb05 = double_integrator_QP_solver_s + 110;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb05 = double_integrator_QP_solver_lbys + 110;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb05[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff05 = double_integrator_QP_solver_dl_aff + 110;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff05 = double_integrator_QP_solver_ds_aff + 110;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc05 = double_integrator_QP_solver_dl_cc + 110;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc05 = double_integrator_QP_solver_ds_cc + 110;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl05 = double_integrator_QP_solver_ccrhs + 110;
int double_integrator_QP_solver_ubIdx05[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub05 = double_integrator_QP_solver_l + 125;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub05 = double_integrator_QP_solver_s + 125;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub05 = double_integrator_QP_solver_lbys + 125;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub05[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff05 = double_integrator_QP_solver_dl_aff + 125;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff05 = double_integrator_QP_solver_ds_aff + 125;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc05 = double_integrator_QP_solver_dl_cc + 125;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc05 = double_integrator_QP_solver_ds_cc + 125;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub05 = double_integrator_QP_solver_ccrhs + 125;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi05[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W05[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd05[25];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd05[25];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z06 = double_integrator_QP_solver_z + 90;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff06 = double_integrator_QP_solver_dz_aff + 90;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc06 = double_integrator_QP_solver_dz_cc + 90;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd06 = double_integrator_QP_solver_rd + 90;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd06[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost06 = double_integrator_QP_solver_grad_cost + 90;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq06 = double_integrator_QP_solver_grad_eq + 90;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq06 = double_integrator_QP_solver_grad_ineq + 90;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv06[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v06 = double_integrator_QP_solver_v + 34;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re06[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta06[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc06[5];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff06 = double_integrator_QP_solver_dv_aff + 34;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc06 = double_integrator_QP_solver_dv_cc + 34;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V06[75];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd06[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld06[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy06[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy06[5];
int double_integrator_QP_solver_lbIdx06[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb06 = double_integrator_QP_solver_l + 132;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb06 = double_integrator_QP_solver_s + 132;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb06 = double_integrator_QP_solver_lbys + 132;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb06[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff06 = double_integrator_QP_solver_dl_aff + 132;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff06 = double_integrator_QP_solver_ds_aff + 132;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc06 = double_integrator_QP_solver_dl_cc + 132;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc06 = double_integrator_QP_solver_ds_cc + 132;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl06 = double_integrator_QP_solver_ccrhs + 132;
int double_integrator_QP_solver_ubIdx06[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub06 = double_integrator_QP_solver_l + 147;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub06 = double_integrator_QP_solver_s + 147;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub06 = double_integrator_QP_solver_lbys + 147;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub06[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff06 = double_integrator_QP_solver_dl_aff + 147;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff06 = double_integrator_QP_solver_ds_aff + 147;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc06 = double_integrator_QP_solver_dl_cc + 147;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc06 = double_integrator_QP_solver_ds_cc + 147;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub06 = double_integrator_QP_solver_ccrhs + 147;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi06[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W06[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd06[25];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd06[25];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z07 = double_integrator_QP_solver_z + 105;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff07 = double_integrator_QP_solver_dz_aff + 105;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc07 = double_integrator_QP_solver_dz_cc + 105;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd07 = double_integrator_QP_solver_rd + 105;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd07[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost07 = double_integrator_QP_solver_grad_cost + 105;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq07 = double_integrator_QP_solver_grad_eq + 105;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq07 = double_integrator_QP_solver_grad_ineq + 105;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv07[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v07 = double_integrator_QP_solver_v + 39;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re07[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta07[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc07[5];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff07 = double_integrator_QP_solver_dv_aff + 39;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc07 = double_integrator_QP_solver_dv_cc + 39;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V07[75];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd07[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld07[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy07[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy07[5];
int double_integrator_QP_solver_lbIdx07[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb07 = double_integrator_QP_solver_l + 154;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb07 = double_integrator_QP_solver_s + 154;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb07 = double_integrator_QP_solver_lbys + 154;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb07[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff07 = double_integrator_QP_solver_dl_aff + 154;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff07 = double_integrator_QP_solver_ds_aff + 154;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc07 = double_integrator_QP_solver_dl_cc + 154;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc07 = double_integrator_QP_solver_ds_cc + 154;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl07 = double_integrator_QP_solver_ccrhs + 154;
int double_integrator_QP_solver_ubIdx07[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub07 = double_integrator_QP_solver_l + 169;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub07 = double_integrator_QP_solver_s + 169;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub07 = double_integrator_QP_solver_lbys + 169;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub07[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff07 = double_integrator_QP_solver_dl_aff + 169;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff07 = double_integrator_QP_solver_ds_aff + 169;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc07 = double_integrator_QP_solver_dl_cc + 169;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc07 = double_integrator_QP_solver_ds_cc + 169;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub07 = double_integrator_QP_solver_ccrhs + 169;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi07[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W07[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd07[25];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd07[25];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z08 = double_integrator_QP_solver_z + 120;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff08 = double_integrator_QP_solver_dz_aff + 120;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc08 = double_integrator_QP_solver_dz_cc + 120;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd08 = double_integrator_QP_solver_rd + 120;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd08[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost08 = double_integrator_QP_solver_grad_cost + 120;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq08 = double_integrator_QP_solver_grad_eq + 120;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq08 = double_integrator_QP_solver_grad_ineq + 120;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv08[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v08 = double_integrator_QP_solver_v + 44;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re08[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta08[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc08[5];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff08 = double_integrator_QP_solver_dv_aff + 44;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc08 = double_integrator_QP_solver_dv_cc + 44;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V08[75];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd08[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld08[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy08[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy08[5];
int double_integrator_QP_solver_lbIdx08[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb08 = double_integrator_QP_solver_l + 176;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb08 = double_integrator_QP_solver_s + 176;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb08 = double_integrator_QP_solver_lbys + 176;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb08[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff08 = double_integrator_QP_solver_dl_aff + 176;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff08 = double_integrator_QP_solver_ds_aff + 176;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc08 = double_integrator_QP_solver_dl_cc + 176;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc08 = double_integrator_QP_solver_ds_cc + 176;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl08 = double_integrator_QP_solver_ccrhs + 176;
int double_integrator_QP_solver_ubIdx08[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub08 = double_integrator_QP_solver_l + 191;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub08 = double_integrator_QP_solver_s + 191;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub08 = double_integrator_QP_solver_lbys + 191;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub08[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff08 = double_integrator_QP_solver_dl_aff + 191;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff08 = double_integrator_QP_solver_ds_aff + 191;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc08 = double_integrator_QP_solver_dl_cc + 191;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc08 = double_integrator_QP_solver_ds_cc + 191;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub08 = double_integrator_QP_solver_ccrhs + 191;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi08[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W08[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd08[25];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd08[25];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z09 = double_integrator_QP_solver_z + 135;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff09 = double_integrator_QP_solver_dz_aff + 135;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc09 = double_integrator_QP_solver_dz_cc + 135;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd09 = double_integrator_QP_solver_rd + 135;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd09[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost09 = double_integrator_QP_solver_grad_cost + 135;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq09 = double_integrator_QP_solver_grad_eq + 135;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq09 = double_integrator_QP_solver_grad_ineq + 135;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv09[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v09 = double_integrator_QP_solver_v + 49;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re09[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta09[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc09[5];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff09 = double_integrator_QP_solver_dv_aff + 49;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc09 = double_integrator_QP_solver_dv_cc + 49;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V09[75];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd09[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld09[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy09[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy09[5];
int double_integrator_QP_solver_lbIdx09[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb09 = double_integrator_QP_solver_l + 198;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb09 = double_integrator_QP_solver_s + 198;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb09 = double_integrator_QP_solver_lbys + 198;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb09[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff09 = double_integrator_QP_solver_dl_aff + 198;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff09 = double_integrator_QP_solver_ds_aff + 198;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc09 = double_integrator_QP_solver_dl_cc + 198;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc09 = double_integrator_QP_solver_ds_cc + 198;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl09 = double_integrator_QP_solver_ccrhs + 198;
int double_integrator_QP_solver_ubIdx09[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub09 = double_integrator_QP_solver_l + 213;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub09 = double_integrator_QP_solver_s + 213;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub09 = double_integrator_QP_solver_lbys + 213;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub09[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff09 = double_integrator_QP_solver_dl_aff + 213;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff09 = double_integrator_QP_solver_ds_aff + 213;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc09 = double_integrator_QP_solver_dl_cc + 213;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc09 = double_integrator_QP_solver_ds_cc + 213;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub09 = double_integrator_QP_solver_ccrhs + 213;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi09[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W09[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd09[25];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd09[25];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z10 = double_integrator_QP_solver_z + 150;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff10 = double_integrator_QP_solver_dz_aff + 150;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc10 = double_integrator_QP_solver_dz_cc + 150;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd10 = double_integrator_QP_solver_rd + 150;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd10[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost10 = double_integrator_QP_solver_grad_cost + 150;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq10 = double_integrator_QP_solver_grad_eq + 150;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq10 = double_integrator_QP_solver_grad_ineq + 150;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv10[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v10 = double_integrator_QP_solver_v + 54;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re10[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta10[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc10[5];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff10 = double_integrator_QP_solver_dv_aff + 54;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc10 = double_integrator_QP_solver_dv_cc + 54;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V10[75];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd10[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld10[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy10[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy10[5];
int double_integrator_QP_solver_lbIdx10[15] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb10 = double_integrator_QP_solver_l + 220;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb10 = double_integrator_QP_solver_s + 220;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb10 = double_integrator_QP_solver_lbys + 220;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb10[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff10 = double_integrator_QP_solver_dl_aff + 220;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff10 = double_integrator_QP_solver_ds_aff + 220;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc10 = double_integrator_QP_solver_dl_cc + 220;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc10 = double_integrator_QP_solver_ds_cc + 220;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl10 = double_integrator_QP_solver_ccrhs + 220;
int double_integrator_QP_solver_ubIdx10[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub10 = double_integrator_QP_solver_l + 235;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub10 = double_integrator_QP_solver_s + 235;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub10 = double_integrator_QP_solver_lbys + 235;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub10[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff10 = double_integrator_QP_solver_dl_aff + 235;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff10 = double_integrator_QP_solver_ds_aff + 235;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc10 = double_integrator_QP_solver_dl_cc + 235;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc10 = double_integrator_QP_solver_ds_cc + 235;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub10 = double_integrator_QP_solver_ccrhs + 235;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi10[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W10[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd10[25];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd10[25];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_H11[5] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_f11[5] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z11 = double_integrator_QP_solver_z + 165;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff11 = double_integrator_QP_solver_dz_aff + 165;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc11 = double_integrator_QP_solver_dz_cc + 165;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd11 = double_integrator_QP_solver_rd + 165;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd11[5];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost11 = double_integrator_QP_solver_grad_cost + 165;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq11 = double_integrator_QP_solver_grad_eq + 165;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq11 = double_integrator_QP_solver_grad_ineq + 165;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv11[5];
int double_integrator_QP_solver_lbIdx11[5] = {0, 1, 2, 3, 4};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb11 = double_integrator_QP_solver_l + 242;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb11 = double_integrator_QP_solver_s + 242;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb11 = double_integrator_QP_solver_lbys + 242;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb11[5];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff11 = double_integrator_QP_solver_dl_aff + 242;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff11 = double_integrator_QP_solver_ds_aff + 242;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc11 = double_integrator_QP_solver_dl_cc + 242;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc11 = double_integrator_QP_solver_ds_cc + 242;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl11 = double_integrator_QP_solver_ccrhs + 242;
int double_integrator_QP_solver_ubIdx11[5] = {0, 1, 2, 3, 4};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub11 = double_integrator_QP_solver_l + 247;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub11 = double_integrator_QP_solver_s + 247;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub11 = double_integrator_QP_solver_lbys + 247;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub11[5];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff11 = double_integrator_QP_solver_dl_aff + 247;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff11 = double_integrator_QP_solver_ds_aff + 247;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc11 = double_integrator_QP_solver_dl_cc + 247;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc11 = double_integrator_QP_solver_ds_cc + 247;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub11 = double_integrator_QP_solver_ccrhs + 247;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi11[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_D11[5] = {-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000};
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W11[5];
double_integrator_QP_solver_FLOAT musigma;
double_integrator_QP_solver_FLOAT sigma_3rdroot;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Diag1_0[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Diag2_0[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_L_0[105];




/* SOLVER CODE --------------------------------------------------------- */
int double_integrator_QP_solver_solve(double_integrator_QP_solver_params* params, double_integrator_QP_solver_output* output, double_integrator_QP_solver_info* info)
{	
int exitcode;

#if double_integrator_QP_solver_SET_TIMING == 1
	double_integrator_QP_solver_timer solvertimer;
	double_integrator_QP_solver_tic(&solvertimer);
#endif
/* FUNCTION CALLS INTO LA LIBRARY -------------------------------------- */
info->it = 0;
double_integrator_QP_solver_LA_INITIALIZEVECTOR_170(double_integrator_QP_solver_z, 0);
double_integrator_QP_solver_LA_INITIALIZEVECTOR_59(double_integrator_QP_solver_v, 1);
double_integrator_QP_solver_LA_INITIALIZEVECTOR_252(double_integrator_QP_solver_l, 10);
double_integrator_QP_solver_LA_INITIALIZEVECTOR_252(double_integrator_QP_solver_s, 10);
info->mu = 0;
double_integrator_QP_solver_LA_DOTACC_252(double_integrator_QP_solver_l, double_integrator_QP_solver_s, &info->mu);
info->mu /= 252;
while( 1 ){
info->pobj = 0;
double_integrator_QP_solver_LA_DIAG_QUADFCN_15(double_integrator_QP_solver_H00, params->f1, double_integrator_QP_solver_z00, double_integrator_QP_solver_grad_cost00, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_15(double_integrator_QP_solver_H00, params->f2, double_integrator_QP_solver_z01, double_integrator_QP_solver_grad_cost01, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_15(double_integrator_QP_solver_H00, params->f3, double_integrator_QP_solver_z02, double_integrator_QP_solver_grad_cost02, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_15(double_integrator_QP_solver_H00, params->f4, double_integrator_QP_solver_z03, double_integrator_QP_solver_grad_cost03, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_15(double_integrator_QP_solver_H00, params->f5, double_integrator_QP_solver_z04, double_integrator_QP_solver_grad_cost04, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_15(double_integrator_QP_solver_H00, params->f6, double_integrator_QP_solver_z05, double_integrator_QP_solver_grad_cost05, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_15(double_integrator_QP_solver_H00, params->f7, double_integrator_QP_solver_z06, double_integrator_QP_solver_grad_cost06, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_15(double_integrator_QP_solver_H00, params->f8, double_integrator_QP_solver_z07, double_integrator_QP_solver_grad_cost07, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_15(double_integrator_QP_solver_H00, params->f9, double_integrator_QP_solver_z08, double_integrator_QP_solver_grad_cost08, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_15(double_integrator_QP_solver_H00, params->f10, double_integrator_QP_solver_z09, double_integrator_QP_solver_grad_cost09, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_15(double_integrator_QP_solver_H00, params->f11, double_integrator_QP_solver_z10, double_integrator_QP_solver_grad_cost10, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_5(double_integrator_QP_solver_H11, double_integrator_QP_solver_f11, double_integrator_QP_solver_z11, double_integrator_QP_solver_grad_cost11, &info->pobj);
info->res_eq = 0;
info->dgap = 0;
double_integrator_QP_solver_LA_DENSE_MVMSUB3_9_15_15(params->C1, double_integrator_QP_solver_z00, double_integrator_QP_solver_D01, double_integrator_QP_solver_z01, params->e1, double_integrator_QP_solver_v00, double_integrator_QP_solver_re00, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_15_15(params->C2, double_integrator_QP_solver_z01, double_integrator_QP_solver_D02, double_integrator_QP_solver_z02, params->e2, double_integrator_QP_solver_v01, double_integrator_QP_solver_re01, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_15_15(params->C3, double_integrator_QP_solver_z02, double_integrator_QP_solver_D02, double_integrator_QP_solver_z03, params->e3, double_integrator_QP_solver_v02, double_integrator_QP_solver_re02, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_15_15(params->C4, double_integrator_QP_solver_z03, double_integrator_QP_solver_D02, double_integrator_QP_solver_z04, params->e4, double_integrator_QP_solver_v03, double_integrator_QP_solver_re03, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_15_15(params->C5, double_integrator_QP_solver_z04, double_integrator_QP_solver_D02, double_integrator_QP_solver_z05, params->e5, double_integrator_QP_solver_v04, double_integrator_QP_solver_re04, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_15_15(params->C6, double_integrator_QP_solver_z05, double_integrator_QP_solver_D02, double_integrator_QP_solver_z06, params->e6, double_integrator_QP_solver_v05, double_integrator_QP_solver_re05, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_15_15(params->C7, double_integrator_QP_solver_z06, double_integrator_QP_solver_D02, double_integrator_QP_solver_z07, params->e7, double_integrator_QP_solver_v06, double_integrator_QP_solver_re06, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_15_15(params->C8, double_integrator_QP_solver_z07, double_integrator_QP_solver_D02, double_integrator_QP_solver_z08, params->e8, double_integrator_QP_solver_v07, double_integrator_QP_solver_re07, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_15_15(params->C9, double_integrator_QP_solver_z08, double_integrator_QP_solver_D02, double_integrator_QP_solver_z09, params->e9, double_integrator_QP_solver_v08, double_integrator_QP_solver_re08, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_15_15(params->C10, double_integrator_QP_solver_z09, double_integrator_QP_solver_D02, double_integrator_QP_solver_z10, params->e10, double_integrator_QP_solver_v09, double_integrator_QP_solver_re09, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_15_5(params->C11, double_integrator_QP_solver_z10, double_integrator_QP_solver_D11, double_integrator_QP_solver_z11, params->e11, double_integrator_QP_solver_v10, double_integrator_QP_solver_re10, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MTVM_9_15(params->C1, double_integrator_QP_solver_v00, double_integrator_QP_solver_grad_eq00);
double_integrator_QP_solver_LA_DENSE_MTVM2_5_15_9(params->C2, double_integrator_QP_solver_v01, double_integrator_QP_solver_D01, double_integrator_QP_solver_v00, double_integrator_QP_solver_grad_eq01);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C3, double_integrator_QP_solver_v02, double_integrator_QP_solver_D02, double_integrator_QP_solver_v01, double_integrator_QP_solver_grad_eq02);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C4, double_integrator_QP_solver_v03, double_integrator_QP_solver_D02, double_integrator_QP_solver_v02, double_integrator_QP_solver_grad_eq03);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C5, double_integrator_QP_solver_v04, double_integrator_QP_solver_D02, double_integrator_QP_solver_v03, double_integrator_QP_solver_grad_eq04);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C6, double_integrator_QP_solver_v05, double_integrator_QP_solver_D02, double_integrator_QP_solver_v04, double_integrator_QP_solver_grad_eq05);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C7, double_integrator_QP_solver_v06, double_integrator_QP_solver_D02, double_integrator_QP_solver_v05, double_integrator_QP_solver_grad_eq06);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C8, double_integrator_QP_solver_v07, double_integrator_QP_solver_D02, double_integrator_QP_solver_v06, double_integrator_QP_solver_grad_eq07);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C9, double_integrator_QP_solver_v08, double_integrator_QP_solver_D02, double_integrator_QP_solver_v07, double_integrator_QP_solver_grad_eq08);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C10, double_integrator_QP_solver_v09, double_integrator_QP_solver_D02, double_integrator_QP_solver_v08, double_integrator_QP_solver_grad_eq09);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C11, double_integrator_QP_solver_v10, double_integrator_QP_solver_D02, double_integrator_QP_solver_v09, double_integrator_QP_solver_grad_eq10);
double_integrator_QP_solver_LA_DIAGZERO_MTVM_5_5(double_integrator_QP_solver_D11, double_integrator_QP_solver_v10, double_integrator_QP_solver_grad_eq11);
info->res_ineq = 0;
double_integrator_QP_solver_LA_VSUBADD3_15(params->lb1, double_integrator_QP_solver_z00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_llb00, double_integrator_QP_solver_slb00, double_integrator_QP_solver_rilb00, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_7(double_integrator_QP_solver_z00, double_integrator_QP_solver_ubIdx00, params->ub1, double_integrator_QP_solver_lub00, double_integrator_QP_solver_sub00, double_integrator_QP_solver_riub00, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_15(params->lb2, double_integrator_QP_solver_z01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_llb01, double_integrator_QP_solver_slb01, double_integrator_QP_solver_rilb01, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_7(double_integrator_QP_solver_z01, double_integrator_QP_solver_ubIdx01, params->ub2, double_integrator_QP_solver_lub01, double_integrator_QP_solver_sub01, double_integrator_QP_solver_riub01, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_15(params->lb3, double_integrator_QP_solver_z02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_llb02, double_integrator_QP_solver_slb02, double_integrator_QP_solver_rilb02, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_7(double_integrator_QP_solver_z02, double_integrator_QP_solver_ubIdx02, params->ub3, double_integrator_QP_solver_lub02, double_integrator_QP_solver_sub02, double_integrator_QP_solver_riub02, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_15(params->lb4, double_integrator_QP_solver_z03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_llb03, double_integrator_QP_solver_slb03, double_integrator_QP_solver_rilb03, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_7(double_integrator_QP_solver_z03, double_integrator_QP_solver_ubIdx03, params->ub4, double_integrator_QP_solver_lub03, double_integrator_QP_solver_sub03, double_integrator_QP_solver_riub03, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_15(params->lb5, double_integrator_QP_solver_z04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_llb04, double_integrator_QP_solver_slb04, double_integrator_QP_solver_rilb04, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_7(double_integrator_QP_solver_z04, double_integrator_QP_solver_ubIdx04, params->ub5, double_integrator_QP_solver_lub04, double_integrator_QP_solver_sub04, double_integrator_QP_solver_riub04, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_15(params->lb6, double_integrator_QP_solver_z05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_llb05, double_integrator_QP_solver_slb05, double_integrator_QP_solver_rilb05, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_7(double_integrator_QP_solver_z05, double_integrator_QP_solver_ubIdx05, params->ub6, double_integrator_QP_solver_lub05, double_integrator_QP_solver_sub05, double_integrator_QP_solver_riub05, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_15(params->lb7, double_integrator_QP_solver_z06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_llb06, double_integrator_QP_solver_slb06, double_integrator_QP_solver_rilb06, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_7(double_integrator_QP_solver_z06, double_integrator_QP_solver_ubIdx06, params->ub7, double_integrator_QP_solver_lub06, double_integrator_QP_solver_sub06, double_integrator_QP_solver_riub06, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_15(params->lb8, double_integrator_QP_solver_z07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_llb07, double_integrator_QP_solver_slb07, double_integrator_QP_solver_rilb07, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_7(double_integrator_QP_solver_z07, double_integrator_QP_solver_ubIdx07, params->ub8, double_integrator_QP_solver_lub07, double_integrator_QP_solver_sub07, double_integrator_QP_solver_riub07, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_15(params->lb9, double_integrator_QP_solver_z08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_llb08, double_integrator_QP_solver_slb08, double_integrator_QP_solver_rilb08, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_7(double_integrator_QP_solver_z08, double_integrator_QP_solver_ubIdx08, params->ub9, double_integrator_QP_solver_lub08, double_integrator_QP_solver_sub08, double_integrator_QP_solver_riub08, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_15(params->lb10, double_integrator_QP_solver_z09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_llb09, double_integrator_QP_solver_slb09, double_integrator_QP_solver_rilb09, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_7(double_integrator_QP_solver_z09, double_integrator_QP_solver_ubIdx09, params->ub10, double_integrator_QP_solver_lub09, double_integrator_QP_solver_sub09, double_integrator_QP_solver_riub09, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_15(params->lb11, double_integrator_QP_solver_z10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_llb10, double_integrator_QP_solver_slb10, double_integrator_QP_solver_rilb10, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_7(double_integrator_QP_solver_z10, double_integrator_QP_solver_ubIdx10, params->ub11, double_integrator_QP_solver_lub10, double_integrator_QP_solver_sub10, double_integrator_QP_solver_riub10, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_5(params->lb12, double_integrator_QP_solver_z11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_llb11, double_integrator_QP_solver_slb11, double_integrator_QP_solver_rilb11, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_5(double_integrator_QP_solver_z11, double_integrator_QP_solver_ubIdx11, params->ub12, double_integrator_QP_solver_lub11, double_integrator_QP_solver_sub11, double_integrator_QP_solver_riub11, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_INEQ_B_GRAD_15_15_7(double_integrator_QP_solver_lub00, double_integrator_QP_solver_sub00, double_integrator_QP_solver_riub00, double_integrator_QP_solver_llb00, double_integrator_QP_solver_slb00, double_integrator_QP_solver_rilb00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_grad_ineq00, double_integrator_QP_solver_lubbysub00, double_integrator_QP_solver_llbbyslb00);
double_integrator_QP_solver_LA_INEQ_B_GRAD_15_15_7(double_integrator_QP_solver_lub01, double_integrator_QP_solver_sub01, double_integrator_QP_solver_riub01, double_integrator_QP_solver_llb01, double_integrator_QP_solver_slb01, double_integrator_QP_solver_rilb01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_grad_ineq01, double_integrator_QP_solver_lubbysub01, double_integrator_QP_solver_llbbyslb01);
double_integrator_QP_solver_LA_INEQ_B_GRAD_15_15_7(double_integrator_QP_solver_lub02, double_integrator_QP_solver_sub02, double_integrator_QP_solver_riub02, double_integrator_QP_solver_llb02, double_integrator_QP_solver_slb02, double_integrator_QP_solver_rilb02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_grad_ineq02, double_integrator_QP_solver_lubbysub02, double_integrator_QP_solver_llbbyslb02);
double_integrator_QP_solver_LA_INEQ_B_GRAD_15_15_7(double_integrator_QP_solver_lub03, double_integrator_QP_solver_sub03, double_integrator_QP_solver_riub03, double_integrator_QP_solver_llb03, double_integrator_QP_solver_slb03, double_integrator_QP_solver_rilb03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_grad_ineq03, double_integrator_QP_solver_lubbysub03, double_integrator_QP_solver_llbbyslb03);
double_integrator_QP_solver_LA_INEQ_B_GRAD_15_15_7(double_integrator_QP_solver_lub04, double_integrator_QP_solver_sub04, double_integrator_QP_solver_riub04, double_integrator_QP_solver_llb04, double_integrator_QP_solver_slb04, double_integrator_QP_solver_rilb04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_grad_ineq04, double_integrator_QP_solver_lubbysub04, double_integrator_QP_solver_llbbyslb04);
double_integrator_QP_solver_LA_INEQ_B_GRAD_15_15_7(double_integrator_QP_solver_lub05, double_integrator_QP_solver_sub05, double_integrator_QP_solver_riub05, double_integrator_QP_solver_llb05, double_integrator_QP_solver_slb05, double_integrator_QP_solver_rilb05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_grad_ineq05, double_integrator_QP_solver_lubbysub05, double_integrator_QP_solver_llbbyslb05);
double_integrator_QP_solver_LA_INEQ_B_GRAD_15_15_7(double_integrator_QP_solver_lub06, double_integrator_QP_solver_sub06, double_integrator_QP_solver_riub06, double_integrator_QP_solver_llb06, double_integrator_QP_solver_slb06, double_integrator_QP_solver_rilb06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_grad_ineq06, double_integrator_QP_solver_lubbysub06, double_integrator_QP_solver_llbbyslb06);
double_integrator_QP_solver_LA_INEQ_B_GRAD_15_15_7(double_integrator_QP_solver_lub07, double_integrator_QP_solver_sub07, double_integrator_QP_solver_riub07, double_integrator_QP_solver_llb07, double_integrator_QP_solver_slb07, double_integrator_QP_solver_rilb07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_grad_ineq07, double_integrator_QP_solver_lubbysub07, double_integrator_QP_solver_llbbyslb07);
double_integrator_QP_solver_LA_INEQ_B_GRAD_15_15_7(double_integrator_QP_solver_lub08, double_integrator_QP_solver_sub08, double_integrator_QP_solver_riub08, double_integrator_QP_solver_llb08, double_integrator_QP_solver_slb08, double_integrator_QP_solver_rilb08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_grad_ineq08, double_integrator_QP_solver_lubbysub08, double_integrator_QP_solver_llbbyslb08);
double_integrator_QP_solver_LA_INEQ_B_GRAD_15_15_7(double_integrator_QP_solver_lub09, double_integrator_QP_solver_sub09, double_integrator_QP_solver_riub09, double_integrator_QP_solver_llb09, double_integrator_QP_solver_slb09, double_integrator_QP_solver_rilb09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_grad_ineq09, double_integrator_QP_solver_lubbysub09, double_integrator_QP_solver_llbbyslb09);
double_integrator_QP_solver_LA_INEQ_B_GRAD_15_15_7(double_integrator_QP_solver_lub10, double_integrator_QP_solver_sub10, double_integrator_QP_solver_riub10, double_integrator_QP_solver_llb10, double_integrator_QP_solver_slb10, double_integrator_QP_solver_rilb10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_grad_ineq10, double_integrator_QP_solver_lubbysub10, double_integrator_QP_solver_llbbyslb10);
double_integrator_QP_solver_LA_INEQ_B_GRAD_5_5_5(double_integrator_QP_solver_lub11, double_integrator_QP_solver_sub11, double_integrator_QP_solver_riub11, double_integrator_QP_solver_llb11, double_integrator_QP_solver_slb11, double_integrator_QP_solver_rilb11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_grad_ineq11, double_integrator_QP_solver_lubbysub11, double_integrator_QP_solver_llbbyslb11);
info->dobj = info->pobj - info->dgap;
info->rdgap = info->pobj ? info->dgap / info->pobj : 1e6;
if( info->rdgap < 0 ) info->rdgap = -info->rdgap;
if( info->mu < double_integrator_QP_solver_SET_ACC_KKTCOMPL
    && (info->rdgap < double_integrator_QP_solver_SET_ACC_RDGAP || info->dgap < double_integrator_QP_solver_SET_ACC_KKTCOMPL)
    && info->res_eq < double_integrator_QP_solver_SET_ACC_RESEQ
    && info->res_ineq < double_integrator_QP_solver_SET_ACC_RESINEQ ){
exitcode = double_integrator_QP_solver_OPTIMAL; break; }
if( info->it == double_integrator_QP_solver_SET_MAXIT ){
exitcode = double_integrator_QP_solver_MAXITREACHED; break; }
double_integrator_QP_solver_LA_VVADD3_170(double_integrator_QP_solver_grad_cost, double_integrator_QP_solver_grad_eq, double_integrator_QP_solver_grad_ineq, double_integrator_QP_solver_rd);
double_integrator_QP_solver_LA_DIAG_CHOL_LBUB_15_15_7(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_lubbysub00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_Phi00);
double_integrator_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_15(double_integrator_QP_solver_Phi00, params->C1, double_integrator_QP_solver_V00);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_rd00, double_integrator_QP_solver_Lbyrd00);
double_integrator_QP_solver_LA_DIAG_CHOL_LBUB_15_15_7(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_lubbysub01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_Phi01);
double_integrator_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_15(double_integrator_QP_solver_Phi01, params->C2, double_integrator_QP_solver_V01);
double_integrator_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_15(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_D01, double_integrator_QP_solver_W01);
double_integrator_QP_solver_LA_DENSE_MMTM_9_15_5(double_integrator_QP_solver_W01, double_integrator_QP_solver_V01, double_integrator_QP_solver_Ysd01);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_rd01, double_integrator_QP_solver_Lbyrd01);
double_integrator_QP_solver_LA_DIAG_CHOL_LBUB_15_15_7(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_lubbysub02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_Phi02);
double_integrator_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_15(double_integrator_QP_solver_Phi02, params->C3, double_integrator_QP_solver_V02);
double_integrator_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_15(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_D02, double_integrator_QP_solver_W02);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_15_5(double_integrator_QP_solver_W02, double_integrator_QP_solver_V02, double_integrator_QP_solver_Ysd02);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_rd02, double_integrator_QP_solver_Lbyrd02);
double_integrator_QP_solver_LA_DIAG_CHOL_LBUB_15_15_7(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_lubbysub03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_Phi03);
double_integrator_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_15(double_integrator_QP_solver_Phi03, params->C4, double_integrator_QP_solver_V03);
double_integrator_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_15(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_D02, double_integrator_QP_solver_W03);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_15_5(double_integrator_QP_solver_W03, double_integrator_QP_solver_V03, double_integrator_QP_solver_Ysd03);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_rd03, double_integrator_QP_solver_Lbyrd03);
double_integrator_QP_solver_LA_DIAG_CHOL_LBUB_15_15_7(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_lubbysub04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_Phi04);
double_integrator_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_15(double_integrator_QP_solver_Phi04, params->C5, double_integrator_QP_solver_V04);
double_integrator_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_15(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_D02, double_integrator_QP_solver_W04);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_15_5(double_integrator_QP_solver_W04, double_integrator_QP_solver_V04, double_integrator_QP_solver_Ysd04);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_rd04, double_integrator_QP_solver_Lbyrd04);
double_integrator_QP_solver_LA_DIAG_CHOL_LBUB_15_15_7(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_lubbysub05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_Phi05);
double_integrator_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_15(double_integrator_QP_solver_Phi05, params->C6, double_integrator_QP_solver_V05);
double_integrator_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_15(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_D02, double_integrator_QP_solver_W05);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_15_5(double_integrator_QP_solver_W05, double_integrator_QP_solver_V05, double_integrator_QP_solver_Ysd05);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_rd05, double_integrator_QP_solver_Lbyrd05);
double_integrator_QP_solver_LA_DIAG_CHOL_LBUB_15_15_7(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_lubbysub06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_Phi06);
double_integrator_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_15(double_integrator_QP_solver_Phi06, params->C7, double_integrator_QP_solver_V06);
double_integrator_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_15(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_D02, double_integrator_QP_solver_W06);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_15_5(double_integrator_QP_solver_W06, double_integrator_QP_solver_V06, double_integrator_QP_solver_Ysd06);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_rd06, double_integrator_QP_solver_Lbyrd06);
double_integrator_QP_solver_LA_DIAG_CHOL_LBUB_15_15_7(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_lubbysub07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_Phi07);
double_integrator_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_15(double_integrator_QP_solver_Phi07, params->C8, double_integrator_QP_solver_V07);
double_integrator_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_15(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_D02, double_integrator_QP_solver_W07);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_15_5(double_integrator_QP_solver_W07, double_integrator_QP_solver_V07, double_integrator_QP_solver_Ysd07);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_rd07, double_integrator_QP_solver_Lbyrd07);
double_integrator_QP_solver_LA_DIAG_CHOL_LBUB_15_15_7(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_lubbysub08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_Phi08);
double_integrator_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_15(double_integrator_QP_solver_Phi08, params->C9, double_integrator_QP_solver_V08);
double_integrator_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_15(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_D02, double_integrator_QP_solver_W08);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_15_5(double_integrator_QP_solver_W08, double_integrator_QP_solver_V08, double_integrator_QP_solver_Ysd08);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_rd08, double_integrator_QP_solver_Lbyrd08);
double_integrator_QP_solver_LA_DIAG_CHOL_LBUB_15_15_7(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_lubbysub09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_Phi09);
double_integrator_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_15(double_integrator_QP_solver_Phi09, params->C10, double_integrator_QP_solver_V09);
double_integrator_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_15(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_D02, double_integrator_QP_solver_W09);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_15_5(double_integrator_QP_solver_W09, double_integrator_QP_solver_V09, double_integrator_QP_solver_Ysd09);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_rd09, double_integrator_QP_solver_Lbyrd09);
double_integrator_QP_solver_LA_DIAG_CHOL_LBUB_15_15_7(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_lubbysub10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_Phi10);
double_integrator_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_15(double_integrator_QP_solver_Phi10, params->C11, double_integrator_QP_solver_V10);
double_integrator_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_15(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_D02, double_integrator_QP_solver_W10);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_15_5(double_integrator_QP_solver_W10, double_integrator_QP_solver_V10, double_integrator_QP_solver_Ysd10);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_rd10, double_integrator_QP_solver_Lbyrd10);
double_integrator_QP_solver_LA_DIAG_CHOL_ONELOOP_LBUB_5_5_5(double_integrator_QP_solver_H11, double_integrator_QP_solver_llbbyslb11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_lubbysub11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_Phi11);
double_integrator_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_5(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_D11, double_integrator_QP_solver_W11);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_5(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_rd11, double_integrator_QP_solver_Lbyrd11);
double_integrator_QP_solver_LA_DENSE_MMT2_9_15_15(double_integrator_QP_solver_V00, double_integrator_QP_solver_W01, double_integrator_QP_solver_Yd00);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_9_15_15(double_integrator_QP_solver_V00, double_integrator_QP_solver_Lbyrd00, double_integrator_QP_solver_W01, double_integrator_QP_solver_Lbyrd01, double_integrator_QP_solver_re00, double_integrator_QP_solver_beta00);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_15_15(double_integrator_QP_solver_V01, double_integrator_QP_solver_W02, double_integrator_QP_solver_Yd01);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_15_15(double_integrator_QP_solver_V01, double_integrator_QP_solver_Lbyrd01, double_integrator_QP_solver_W02, double_integrator_QP_solver_Lbyrd02, double_integrator_QP_solver_re01, double_integrator_QP_solver_beta01);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_15_15(double_integrator_QP_solver_V02, double_integrator_QP_solver_W03, double_integrator_QP_solver_Yd02);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_15_15(double_integrator_QP_solver_V02, double_integrator_QP_solver_Lbyrd02, double_integrator_QP_solver_W03, double_integrator_QP_solver_Lbyrd03, double_integrator_QP_solver_re02, double_integrator_QP_solver_beta02);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_15_15(double_integrator_QP_solver_V03, double_integrator_QP_solver_W04, double_integrator_QP_solver_Yd03);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_15_15(double_integrator_QP_solver_V03, double_integrator_QP_solver_Lbyrd03, double_integrator_QP_solver_W04, double_integrator_QP_solver_Lbyrd04, double_integrator_QP_solver_re03, double_integrator_QP_solver_beta03);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_15_15(double_integrator_QP_solver_V04, double_integrator_QP_solver_W05, double_integrator_QP_solver_Yd04);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_15_15(double_integrator_QP_solver_V04, double_integrator_QP_solver_Lbyrd04, double_integrator_QP_solver_W05, double_integrator_QP_solver_Lbyrd05, double_integrator_QP_solver_re04, double_integrator_QP_solver_beta04);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_15_15(double_integrator_QP_solver_V05, double_integrator_QP_solver_W06, double_integrator_QP_solver_Yd05);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_15_15(double_integrator_QP_solver_V05, double_integrator_QP_solver_Lbyrd05, double_integrator_QP_solver_W06, double_integrator_QP_solver_Lbyrd06, double_integrator_QP_solver_re05, double_integrator_QP_solver_beta05);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_15_15(double_integrator_QP_solver_V06, double_integrator_QP_solver_W07, double_integrator_QP_solver_Yd06);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_15_15(double_integrator_QP_solver_V06, double_integrator_QP_solver_Lbyrd06, double_integrator_QP_solver_W07, double_integrator_QP_solver_Lbyrd07, double_integrator_QP_solver_re06, double_integrator_QP_solver_beta06);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_15_15(double_integrator_QP_solver_V07, double_integrator_QP_solver_W08, double_integrator_QP_solver_Yd07);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_15_15(double_integrator_QP_solver_V07, double_integrator_QP_solver_Lbyrd07, double_integrator_QP_solver_W08, double_integrator_QP_solver_Lbyrd08, double_integrator_QP_solver_re07, double_integrator_QP_solver_beta07);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_15_15(double_integrator_QP_solver_V08, double_integrator_QP_solver_W09, double_integrator_QP_solver_Yd08);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_15_15(double_integrator_QP_solver_V08, double_integrator_QP_solver_Lbyrd08, double_integrator_QP_solver_W09, double_integrator_QP_solver_Lbyrd09, double_integrator_QP_solver_re08, double_integrator_QP_solver_beta08);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_15_15(double_integrator_QP_solver_V09, double_integrator_QP_solver_W10, double_integrator_QP_solver_Yd09);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_15_15(double_integrator_QP_solver_V09, double_integrator_QP_solver_Lbyrd09, double_integrator_QP_solver_W10, double_integrator_QP_solver_Lbyrd10, double_integrator_QP_solver_re09, double_integrator_QP_solver_beta09);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_15_5(double_integrator_QP_solver_V10, double_integrator_QP_solver_W11, double_integrator_QP_solver_Yd10);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_15_5(double_integrator_QP_solver_V10, double_integrator_QP_solver_Lbyrd10, double_integrator_QP_solver_W11, double_integrator_QP_solver_Lbyrd11, double_integrator_QP_solver_re10, double_integrator_QP_solver_beta10);
double_integrator_QP_solver_LA_DENSE_CHOL_9(double_integrator_QP_solver_Yd00, double_integrator_QP_solver_Ld00);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_9(double_integrator_QP_solver_Ld00, double_integrator_QP_solver_beta00, double_integrator_QP_solver_yy00);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_9(double_integrator_QP_solver_Ld00, double_integrator_QP_solver_Ysd01, double_integrator_QP_solver_Lsd01);
double_integrator_QP_solver_LA_DENSE_MMTSUB_5_9(double_integrator_QP_solver_Lsd01, double_integrator_QP_solver_Yd01);
double_integrator_QP_solver_LA_DENSE_CHOL_5(double_integrator_QP_solver_Yd01, double_integrator_QP_solver_Ld01);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_9(double_integrator_QP_solver_Lsd01, double_integrator_QP_solver_yy00, double_integrator_QP_solver_beta01, double_integrator_QP_solver_bmy01);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld01, double_integrator_QP_solver_bmy01, double_integrator_QP_solver_yy01);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(double_integrator_QP_solver_Ld01, double_integrator_QP_solver_Ysd02, double_integrator_QP_solver_Lsd02);
double_integrator_QP_solver_LA_DENSE_MMTSUB_5_5(double_integrator_QP_solver_Lsd02, double_integrator_QP_solver_Yd02);
double_integrator_QP_solver_LA_DENSE_CHOL_5(double_integrator_QP_solver_Yd02, double_integrator_QP_solver_Ld02);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd02, double_integrator_QP_solver_yy01, double_integrator_QP_solver_beta02, double_integrator_QP_solver_bmy02);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld02, double_integrator_QP_solver_bmy02, double_integrator_QP_solver_yy02);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(double_integrator_QP_solver_Ld02, double_integrator_QP_solver_Ysd03, double_integrator_QP_solver_Lsd03);
double_integrator_QP_solver_LA_DENSE_MMTSUB_5_5(double_integrator_QP_solver_Lsd03, double_integrator_QP_solver_Yd03);
double_integrator_QP_solver_LA_DENSE_CHOL_5(double_integrator_QP_solver_Yd03, double_integrator_QP_solver_Ld03);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd03, double_integrator_QP_solver_yy02, double_integrator_QP_solver_beta03, double_integrator_QP_solver_bmy03);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld03, double_integrator_QP_solver_bmy03, double_integrator_QP_solver_yy03);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(double_integrator_QP_solver_Ld03, double_integrator_QP_solver_Ysd04, double_integrator_QP_solver_Lsd04);
double_integrator_QP_solver_LA_DENSE_MMTSUB_5_5(double_integrator_QP_solver_Lsd04, double_integrator_QP_solver_Yd04);
double_integrator_QP_solver_LA_DENSE_CHOL_5(double_integrator_QP_solver_Yd04, double_integrator_QP_solver_Ld04);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd04, double_integrator_QP_solver_yy03, double_integrator_QP_solver_beta04, double_integrator_QP_solver_bmy04);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld04, double_integrator_QP_solver_bmy04, double_integrator_QP_solver_yy04);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(double_integrator_QP_solver_Ld04, double_integrator_QP_solver_Ysd05, double_integrator_QP_solver_Lsd05);
double_integrator_QP_solver_LA_DENSE_MMTSUB_5_5(double_integrator_QP_solver_Lsd05, double_integrator_QP_solver_Yd05);
double_integrator_QP_solver_LA_DENSE_CHOL_5(double_integrator_QP_solver_Yd05, double_integrator_QP_solver_Ld05);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd05, double_integrator_QP_solver_yy04, double_integrator_QP_solver_beta05, double_integrator_QP_solver_bmy05);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld05, double_integrator_QP_solver_bmy05, double_integrator_QP_solver_yy05);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(double_integrator_QP_solver_Ld05, double_integrator_QP_solver_Ysd06, double_integrator_QP_solver_Lsd06);
double_integrator_QP_solver_LA_DENSE_MMTSUB_5_5(double_integrator_QP_solver_Lsd06, double_integrator_QP_solver_Yd06);
double_integrator_QP_solver_LA_DENSE_CHOL_5(double_integrator_QP_solver_Yd06, double_integrator_QP_solver_Ld06);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd06, double_integrator_QP_solver_yy05, double_integrator_QP_solver_beta06, double_integrator_QP_solver_bmy06);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld06, double_integrator_QP_solver_bmy06, double_integrator_QP_solver_yy06);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(double_integrator_QP_solver_Ld06, double_integrator_QP_solver_Ysd07, double_integrator_QP_solver_Lsd07);
double_integrator_QP_solver_LA_DENSE_MMTSUB_5_5(double_integrator_QP_solver_Lsd07, double_integrator_QP_solver_Yd07);
double_integrator_QP_solver_LA_DENSE_CHOL_5(double_integrator_QP_solver_Yd07, double_integrator_QP_solver_Ld07);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd07, double_integrator_QP_solver_yy06, double_integrator_QP_solver_beta07, double_integrator_QP_solver_bmy07);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld07, double_integrator_QP_solver_bmy07, double_integrator_QP_solver_yy07);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(double_integrator_QP_solver_Ld07, double_integrator_QP_solver_Ysd08, double_integrator_QP_solver_Lsd08);
double_integrator_QP_solver_LA_DENSE_MMTSUB_5_5(double_integrator_QP_solver_Lsd08, double_integrator_QP_solver_Yd08);
double_integrator_QP_solver_LA_DENSE_CHOL_5(double_integrator_QP_solver_Yd08, double_integrator_QP_solver_Ld08);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd08, double_integrator_QP_solver_yy07, double_integrator_QP_solver_beta08, double_integrator_QP_solver_bmy08);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld08, double_integrator_QP_solver_bmy08, double_integrator_QP_solver_yy08);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(double_integrator_QP_solver_Ld08, double_integrator_QP_solver_Ysd09, double_integrator_QP_solver_Lsd09);
double_integrator_QP_solver_LA_DENSE_MMTSUB_5_5(double_integrator_QP_solver_Lsd09, double_integrator_QP_solver_Yd09);
double_integrator_QP_solver_LA_DENSE_CHOL_5(double_integrator_QP_solver_Yd09, double_integrator_QP_solver_Ld09);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd09, double_integrator_QP_solver_yy08, double_integrator_QP_solver_beta09, double_integrator_QP_solver_bmy09);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld09, double_integrator_QP_solver_bmy09, double_integrator_QP_solver_yy09);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(double_integrator_QP_solver_Ld09, double_integrator_QP_solver_Ysd10, double_integrator_QP_solver_Lsd10);
double_integrator_QP_solver_LA_DENSE_MMTSUB_5_5(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_Yd10);
double_integrator_QP_solver_LA_DENSE_CHOL_5(double_integrator_QP_solver_Yd10, double_integrator_QP_solver_Ld10);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_yy09, double_integrator_QP_solver_beta10, double_integrator_QP_solver_bmy10);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_bmy10, double_integrator_QP_solver_yy10);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_yy10, double_integrator_QP_solver_dvaff10);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_dvaff10, double_integrator_QP_solver_yy09, double_integrator_QP_solver_bmy09);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld09, double_integrator_QP_solver_bmy09, double_integrator_QP_solver_dvaff09);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd09, double_integrator_QP_solver_dvaff09, double_integrator_QP_solver_yy08, double_integrator_QP_solver_bmy08);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld08, double_integrator_QP_solver_bmy08, double_integrator_QP_solver_dvaff08);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd08, double_integrator_QP_solver_dvaff08, double_integrator_QP_solver_yy07, double_integrator_QP_solver_bmy07);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld07, double_integrator_QP_solver_bmy07, double_integrator_QP_solver_dvaff07);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd07, double_integrator_QP_solver_dvaff07, double_integrator_QP_solver_yy06, double_integrator_QP_solver_bmy06);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld06, double_integrator_QP_solver_bmy06, double_integrator_QP_solver_dvaff06);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd06, double_integrator_QP_solver_dvaff06, double_integrator_QP_solver_yy05, double_integrator_QP_solver_bmy05);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld05, double_integrator_QP_solver_bmy05, double_integrator_QP_solver_dvaff05);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd05, double_integrator_QP_solver_dvaff05, double_integrator_QP_solver_yy04, double_integrator_QP_solver_bmy04);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld04, double_integrator_QP_solver_bmy04, double_integrator_QP_solver_dvaff04);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd04, double_integrator_QP_solver_dvaff04, double_integrator_QP_solver_yy03, double_integrator_QP_solver_bmy03);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld03, double_integrator_QP_solver_bmy03, double_integrator_QP_solver_dvaff03);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd03, double_integrator_QP_solver_dvaff03, double_integrator_QP_solver_yy02, double_integrator_QP_solver_bmy02);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld02, double_integrator_QP_solver_bmy02, double_integrator_QP_solver_dvaff02);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd02, double_integrator_QP_solver_dvaff02, double_integrator_QP_solver_yy01, double_integrator_QP_solver_bmy01);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld01, double_integrator_QP_solver_bmy01, double_integrator_QP_solver_dvaff01);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_9(double_integrator_QP_solver_Lsd01, double_integrator_QP_solver_dvaff01, double_integrator_QP_solver_yy00, double_integrator_QP_solver_bmy00);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_9(double_integrator_QP_solver_Ld00, double_integrator_QP_solver_bmy00, double_integrator_QP_solver_dvaff00);
double_integrator_QP_solver_LA_DENSE_MTVM_9_15(params->C1, double_integrator_QP_solver_dvaff00, double_integrator_QP_solver_grad_eq00);
double_integrator_QP_solver_LA_DENSE_MTVM2_5_15_9(params->C2, double_integrator_QP_solver_dvaff01, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff00, double_integrator_QP_solver_grad_eq01);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C3, double_integrator_QP_solver_dvaff02, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvaff01, double_integrator_QP_solver_grad_eq02);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C4, double_integrator_QP_solver_dvaff03, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvaff02, double_integrator_QP_solver_grad_eq03);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C5, double_integrator_QP_solver_dvaff04, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvaff03, double_integrator_QP_solver_grad_eq04);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C6, double_integrator_QP_solver_dvaff05, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvaff04, double_integrator_QP_solver_grad_eq05);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C7, double_integrator_QP_solver_dvaff06, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvaff05, double_integrator_QP_solver_grad_eq06);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C8, double_integrator_QP_solver_dvaff07, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvaff06, double_integrator_QP_solver_grad_eq07);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C9, double_integrator_QP_solver_dvaff08, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvaff07, double_integrator_QP_solver_grad_eq08);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C10, double_integrator_QP_solver_dvaff09, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvaff08, double_integrator_QP_solver_grad_eq09);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C11, double_integrator_QP_solver_dvaff10, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvaff09, double_integrator_QP_solver_grad_eq10);
double_integrator_QP_solver_LA_DIAGZERO_MTVM_5_5(double_integrator_QP_solver_D11, double_integrator_QP_solver_dvaff10, double_integrator_QP_solver_grad_eq11);
double_integrator_QP_solver_LA_VSUB2_170(double_integrator_QP_solver_rd, double_integrator_QP_solver_grad_eq, double_integrator_QP_solver_rd);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_rd00, double_integrator_QP_solver_dzaff00);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_rd01, double_integrator_QP_solver_dzaff01);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_rd02, double_integrator_QP_solver_dzaff02);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_rd03, double_integrator_QP_solver_dzaff03);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_rd04, double_integrator_QP_solver_dzaff04);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_rd05, double_integrator_QP_solver_dzaff05);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_rd06, double_integrator_QP_solver_dzaff06);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_rd07, double_integrator_QP_solver_dzaff07);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_rd08, double_integrator_QP_solver_dzaff08);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_rd09, double_integrator_QP_solver_dzaff09);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_rd10, double_integrator_QP_solver_dzaff10);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_5(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_rd11, double_integrator_QP_solver_dzaff11);
double_integrator_QP_solver_LA_VSUB_INDEXED_15(double_integrator_QP_solver_dzaff00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_rilb00, double_integrator_QP_solver_dslbaff00);
double_integrator_QP_solver_LA_VSUB3_15(double_integrator_QP_solver_llbbyslb00, double_integrator_QP_solver_dslbaff00, double_integrator_QP_solver_llb00, double_integrator_QP_solver_dllbaff00);
double_integrator_QP_solver_LA_VSUB2_INDEXED_7(double_integrator_QP_solver_riub00, double_integrator_QP_solver_dzaff00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_dsubaff00);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_lubbysub00, double_integrator_QP_solver_dsubaff00, double_integrator_QP_solver_lub00, double_integrator_QP_solver_dlubaff00);
double_integrator_QP_solver_LA_VSUB_INDEXED_15(double_integrator_QP_solver_dzaff01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_rilb01, double_integrator_QP_solver_dslbaff01);
double_integrator_QP_solver_LA_VSUB3_15(double_integrator_QP_solver_llbbyslb01, double_integrator_QP_solver_dslbaff01, double_integrator_QP_solver_llb01, double_integrator_QP_solver_dllbaff01);
double_integrator_QP_solver_LA_VSUB2_INDEXED_7(double_integrator_QP_solver_riub01, double_integrator_QP_solver_dzaff01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_dsubaff01);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_lubbysub01, double_integrator_QP_solver_dsubaff01, double_integrator_QP_solver_lub01, double_integrator_QP_solver_dlubaff01);
double_integrator_QP_solver_LA_VSUB_INDEXED_15(double_integrator_QP_solver_dzaff02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_rilb02, double_integrator_QP_solver_dslbaff02);
double_integrator_QP_solver_LA_VSUB3_15(double_integrator_QP_solver_llbbyslb02, double_integrator_QP_solver_dslbaff02, double_integrator_QP_solver_llb02, double_integrator_QP_solver_dllbaff02);
double_integrator_QP_solver_LA_VSUB2_INDEXED_7(double_integrator_QP_solver_riub02, double_integrator_QP_solver_dzaff02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_dsubaff02);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_lubbysub02, double_integrator_QP_solver_dsubaff02, double_integrator_QP_solver_lub02, double_integrator_QP_solver_dlubaff02);
double_integrator_QP_solver_LA_VSUB_INDEXED_15(double_integrator_QP_solver_dzaff03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_rilb03, double_integrator_QP_solver_dslbaff03);
double_integrator_QP_solver_LA_VSUB3_15(double_integrator_QP_solver_llbbyslb03, double_integrator_QP_solver_dslbaff03, double_integrator_QP_solver_llb03, double_integrator_QP_solver_dllbaff03);
double_integrator_QP_solver_LA_VSUB2_INDEXED_7(double_integrator_QP_solver_riub03, double_integrator_QP_solver_dzaff03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_dsubaff03);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_lubbysub03, double_integrator_QP_solver_dsubaff03, double_integrator_QP_solver_lub03, double_integrator_QP_solver_dlubaff03);
double_integrator_QP_solver_LA_VSUB_INDEXED_15(double_integrator_QP_solver_dzaff04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_rilb04, double_integrator_QP_solver_dslbaff04);
double_integrator_QP_solver_LA_VSUB3_15(double_integrator_QP_solver_llbbyslb04, double_integrator_QP_solver_dslbaff04, double_integrator_QP_solver_llb04, double_integrator_QP_solver_dllbaff04);
double_integrator_QP_solver_LA_VSUB2_INDEXED_7(double_integrator_QP_solver_riub04, double_integrator_QP_solver_dzaff04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_dsubaff04);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_lubbysub04, double_integrator_QP_solver_dsubaff04, double_integrator_QP_solver_lub04, double_integrator_QP_solver_dlubaff04);
double_integrator_QP_solver_LA_VSUB_INDEXED_15(double_integrator_QP_solver_dzaff05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_rilb05, double_integrator_QP_solver_dslbaff05);
double_integrator_QP_solver_LA_VSUB3_15(double_integrator_QP_solver_llbbyslb05, double_integrator_QP_solver_dslbaff05, double_integrator_QP_solver_llb05, double_integrator_QP_solver_dllbaff05);
double_integrator_QP_solver_LA_VSUB2_INDEXED_7(double_integrator_QP_solver_riub05, double_integrator_QP_solver_dzaff05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_dsubaff05);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_lubbysub05, double_integrator_QP_solver_dsubaff05, double_integrator_QP_solver_lub05, double_integrator_QP_solver_dlubaff05);
double_integrator_QP_solver_LA_VSUB_INDEXED_15(double_integrator_QP_solver_dzaff06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_rilb06, double_integrator_QP_solver_dslbaff06);
double_integrator_QP_solver_LA_VSUB3_15(double_integrator_QP_solver_llbbyslb06, double_integrator_QP_solver_dslbaff06, double_integrator_QP_solver_llb06, double_integrator_QP_solver_dllbaff06);
double_integrator_QP_solver_LA_VSUB2_INDEXED_7(double_integrator_QP_solver_riub06, double_integrator_QP_solver_dzaff06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_dsubaff06);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_lubbysub06, double_integrator_QP_solver_dsubaff06, double_integrator_QP_solver_lub06, double_integrator_QP_solver_dlubaff06);
double_integrator_QP_solver_LA_VSUB_INDEXED_15(double_integrator_QP_solver_dzaff07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_rilb07, double_integrator_QP_solver_dslbaff07);
double_integrator_QP_solver_LA_VSUB3_15(double_integrator_QP_solver_llbbyslb07, double_integrator_QP_solver_dslbaff07, double_integrator_QP_solver_llb07, double_integrator_QP_solver_dllbaff07);
double_integrator_QP_solver_LA_VSUB2_INDEXED_7(double_integrator_QP_solver_riub07, double_integrator_QP_solver_dzaff07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_dsubaff07);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_lubbysub07, double_integrator_QP_solver_dsubaff07, double_integrator_QP_solver_lub07, double_integrator_QP_solver_dlubaff07);
double_integrator_QP_solver_LA_VSUB_INDEXED_15(double_integrator_QP_solver_dzaff08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_rilb08, double_integrator_QP_solver_dslbaff08);
double_integrator_QP_solver_LA_VSUB3_15(double_integrator_QP_solver_llbbyslb08, double_integrator_QP_solver_dslbaff08, double_integrator_QP_solver_llb08, double_integrator_QP_solver_dllbaff08);
double_integrator_QP_solver_LA_VSUB2_INDEXED_7(double_integrator_QP_solver_riub08, double_integrator_QP_solver_dzaff08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_dsubaff08);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_lubbysub08, double_integrator_QP_solver_dsubaff08, double_integrator_QP_solver_lub08, double_integrator_QP_solver_dlubaff08);
double_integrator_QP_solver_LA_VSUB_INDEXED_15(double_integrator_QP_solver_dzaff09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_rilb09, double_integrator_QP_solver_dslbaff09);
double_integrator_QP_solver_LA_VSUB3_15(double_integrator_QP_solver_llbbyslb09, double_integrator_QP_solver_dslbaff09, double_integrator_QP_solver_llb09, double_integrator_QP_solver_dllbaff09);
double_integrator_QP_solver_LA_VSUB2_INDEXED_7(double_integrator_QP_solver_riub09, double_integrator_QP_solver_dzaff09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_dsubaff09);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_lubbysub09, double_integrator_QP_solver_dsubaff09, double_integrator_QP_solver_lub09, double_integrator_QP_solver_dlubaff09);
double_integrator_QP_solver_LA_VSUB_INDEXED_15(double_integrator_QP_solver_dzaff10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_rilb10, double_integrator_QP_solver_dslbaff10);
double_integrator_QP_solver_LA_VSUB3_15(double_integrator_QP_solver_llbbyslb10, double_integrator_QP_solver_dslbaff10, double_integrator_QP_solver_llb10, double_integrator_QP_solver_dllbaff10);
double_integrator_QP_solver_LA_VSUB2_INDEXED_7(double_integrator_QP_solver_riub10, double_integrator_QP_solver_dzaff10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_dsubaff10);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_lubbysub10, double_integrator_QP_solver_dsubaff10, double_integrator_QP_solver_lub10, double_integrator_QP_solver_dlubaff10);
double_integrator_QP_solver_LA_VSUB_INDEXED_5(double_integrator_QP_solver_dzaff11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_rilb11, double_integrator_QP_solver_dslbaff11);
double_integrator_QP_solver_LA_VSUB3_5(double_integrator_QP_solver_llbbyslb11, double_integrator_QP_solver_dslbaff11, double_integrator_QP_solver_llb11, double_integrator_QP_solver_dllbaff11);
double_integrator_QP_solver_LA_VSUB2_INDEXED_5(double_integrator_QP_solver_riub11, double_integrator_QP_solver_dzaff11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_dsubaff11);
double_integrator_QP_solver_LA_VSUB3_5(double_integrator_QP_solver_lubbysub11, double_integrator_QP_solver_dsubaff11, double_integrator_QP_solver_lub11, double_integrator_QP_solver_dlubaff11);
info->lsit_aff = double_integrator_QP_solver_LINESEARCH_BACKTRACKING_AFFINE(double_integrator_QP_solver_l, double_integrator_QP_solver_s, double_integrator_QP_solver_dl_aff, double_integrator_QP_solver_ds_aff, &info->step_aff, &info->mu_aff);
if( info->lsit_aff == double_integrator_QP_solver_NOPROGRESS ){
exitcode = double_integrator_QP_solver_NOPROGRESS; break;
}
sigma_3rdroot = info->mu_aff / info->mu;
info->sigma = sigma_3rdroot*sigma_3rdroot*sigma_3rdroot;
musigma = info->mu * info->sigma;
double_integrator_QP_solver_LA_VSUB5_252(double_integrator_QP_solver_ds_aff, double_integrator_QP_solver_dl_aff, info->mu, info->sigma, double_integrator_QP_solver_ccrhs);
double_integrator_QP_solver_LA_VSUB6_INDEXED_15_7_15(double_integrator_QP_solver_ccrhsub00, double_integrator_QP_solver_sub00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_ccrhsl00, double_integrator_QP_solver_slb00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_rd00);
double_integrator_QP_solver_LA_VSUB6_INDEXED_15_7_15(double_integrator_QP_solver_ccrhsub01, double_integrator_QP_solver_sub01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_ccrhsl01, double_integrator_QP_solver_slb01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_rd01);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_rd00, double_integrator_QP_solver_Lbyrd00);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_rd01, double_integrator_QP_solver_Lbyrd01);
double_integrator_QP_solver_LA_DENSE_2MVMADD_9_15_15(double_integrator_QP_solver_V00, double_integrator_QP_solver_Lbyrd00, double_integrator_QP_solver_W01, double_integrator_QP_solver_Lbyrd01, double_integrator_QP_solver_beta00);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_9(double_integrator_QP_solver_Ld00, double_integrator_QP_solver_beta00, double_integrator_QP_solver_yy00);
double_integrator_QP_solver_LA_VSUB6_INDEXED_15_7_15(double_integrator_QP_solver_ccrhsub02, double_integrator_QP_solver_sub02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_ccrhsl02, double_integrator_QP_solver_slb02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_rd02);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_rd02, double_integrator_QP_solver_Lbyrd02);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_15_15(double_integrator_QP_solver_V01, double_integrator_QP_solver_Lbyrd01, double_integrator_QP_solver_W02, double_integrator_QP_solver_Lbyrd02, double_integrator_QP_solver_beta01);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_9(double_integrator_QP_solver_Lsd01, double_integrator_QP_solver_yy00, double_integrator_QP_solver_beta01, double_integrator_QP_solver_bmy01);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld01, double_integrator_QP_solver_bmy01, double_integrator_QP_solver_yy01);
double_integrator_QP_solver_LA_VSUB6_INDEXED_15_7_15(double_integrator_QP_solver_ccrhsub03, double_integrator_QP_solver_sub03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_ccrhsl03, double_integrator_QP_solver_slb03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_rd03);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_rd03, double_integrator_QP_solver_Lbyrd03);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_15_15(double_integrator_QP_solver_V02, double_integrator_QP_solver_Lbyrd02, double_integrator_QP_solver_W03, double_integrator_QP_solver_Lbyrd03, double_integrator_QP_solver_beta02);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd02, double_integrator_QP_solver_yy01, double_integrator_QP_solver_beta02, double_integrator_QP_solver_bmy02);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld02, double_integrator_QP_solver_bmy02, double_integrator_QP_solver_yy02);
double_integrator_QP_solver_LA_VSUB6_INDEXED_15_7_15(double_integrator_QP_solver_ccrhsub04, double_integrator_QP_solver_sub04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_ccrhsl04, double_integrator_QP_solver_slb04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_rd04);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_rd04, double_integrator_QP_solver_Lbyrd04);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_15_15(double_integrator_QP_solver_V03, double_integrator_QP_solver_Lbyrd03, double_integrator_QP_solver_W04, double_integrator_QP_solver_Lbyrd04, double_integrator_QP_solver_beta03);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd03, double_integrator_QP_solver_yy02, double_integrator_QP_solver_beta03, double_integrator_QP_solver_bmy03);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld03, double_integrator_QP_solver_bmy03, double_integrator_QP_solver_yy03);
double_integrator_QP_solver_LA_VSUB6_INDEXED_15_7_15(double_integrator_QP_solver_ccrhsub05, double_integrator_QP_solver_sub05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_ccrhsl05, double_integrator_QP_solver_slb05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_rd05);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_rd05, double_integrator_QP_solver_Lbyrd05);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_15_15(double_integrator_QP_solver_V04, double_integrator_QP_solver_Lbyrd04, double_integrator_QP_solver_W05, double_integrator_QP_solver_Lbyrd05, double_integrator_QP_solver_beta04);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd04, double_integrator_QP_solver_yy03, double_integrator_QP_solver_beta04, double_integrator_QP_solver_bmy04);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld04, double_integrator_QP_solver_bmy04, double_integrator_QP_solver_yy04);
double_integrator_QP_solver_LA_VSUB6_INDEXED_15_7_15(double_integrator_QP_solver_ccrhsub06, double_integrator_QP_solver_sub06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_ccrhsl06, double_integrator_QP_solver_slb06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_rd06);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_rd06, double_integrator_QP_solver_Lbyrd06);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_15_15(double_integrator_QP_solver_V05, double_integrator_QP_solver_Lbyrd05, double_integrator_QP_solver_W06, double_integrator_QP_solver_Lbyrd06, double_integrator_QP_solver_beta05);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd05, double_integrator_QP_solver_yy04, double_integrator_QP_solver_beta05, double_integrator_QP_solver_bmy05);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld05, double_integrator_QP_solver_bmy05, double_integrator_QP_solver_yy05);
double_integrator_QP_solver_LA_VSUB6_INDEXED_15_7_15(double_integrator_QP_solver_ccrhsub07, double_integrator_QP_solver_sub07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_ccrhsl07, double_integrator_QP_solver_slb07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_rd07);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_rd07, double_integrator_QP_solver_Lbyrd07);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_15_15(double_integrator_QP_solver_V06, double_integrator_QP_solver_Lbyrd06, double_integrator_QP_solver_W07, double_integrator_QP_solver_Lbyrd07, double_integrator_QP_solver_beta06);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd06, double_integrator_QP_solver_yy05, double_integrator_QP_solver_beta06, double_integrator_QP_solver_bmy06);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld06, double_integrator_QP_solver_bmy06, double_integrator_QP_solver_yy06);
double_integrator_QP_solver_LA_VSUB6_INDEXED_15_7_15(double_integrator_QP_solver_ccrhsub08, double_integrator_QP_solver_sub08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_ccrhsl08, double_integrator_QP_solver_slb08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_rd08);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_rd08, double_integrator_QP_solver_Lbyrd08);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_15_15(double_integrator_QP_solver_V07, double_integrator_QP_solver_Lbyrd07, double_integrator_QP_solver_W08, double_integrator_QP_solver_Lbyrd08, double_integrator_QP_solver_beta07);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd07, double_integrator_QP_solver_yy06, double_integrator_QP_solver_beta07, double_integrator_QP_solver_bmy07);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld07, double_integrator_QP_solver_bmy07, double_integrator_QP_solver_yy07);
double_integrator_QP_solver_LA_VSUB6_INDEXED_15_7_15(double_integrator_QP_solver_ccrhsub09, double_integrator_QP_solver_sub09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_ccrhsl09, double_integrator_QP_solver_slb09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_rd09);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_rd09, double_integrator_QP_solver_Lbyrd09);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_15_15(double_integrator_QP_solver_V08, double_integrator_QP_solver_Lbyrd08, double_integrator_QP_solver_W09, double_integrator_QP_solver_Lbyrd09, double_integrator_QP_solver_beta08);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd08, double_integrator_QP_solver_yy07, double_integrator_QP_solver_beta08, double_integrator_QP_solver_bmy08);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld08, double_integrator_QP_solver_bmy08, double_integrator_QP_solver_yy08);
double_integrator_QP_solver_LA_VSUB6_INDEXED_15_7_15(double_integrator_QP_solver_ccrhsub10, double_integrator_QP_solver_sub10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_ccrhsl10, double_integrator_QP_solver_slb10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_rd10);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_15(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_rd10, double_integrator_QP_solver_Lbyrd10);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_15_15(double_integrator_QP_solver_V09, double_integrator_QP_solver_Lbyrd09, double_integrator_QP_solver_W10, double_integrator_QP_solver_Lbyrd10, double_integrator_QP_solver_beta09);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd09, double_integrator_QP_solver_yy08, double_integrator_QP_solver_beta09, double_integrator_QP_solver_bmy09);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld09, double_integrator_QP_solver_bmy09, double_integrator_QP_solver_yy09);
double_integrator_QP_solver_LA_VSUB6_INDEXED_5_5_5(double_integrator_QP_solver_ccrhsub11, double_integrator_QP_solver_sub11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_ccrhsl11, double_integrator_QP_solver_slb11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_rd11);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_5(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_rd11, double_integrator_QP_solver_Lbyrd11);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_15_5(double_integrator_QP_solver_V10, double_integrator_QP_solver_Lbyrd10, double_integrator_QP_solver_W11, double_integrator_QP_solver_Lbyrd11, double_integrator_QP_solver_beta10);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_5(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_yy09, double_integrator_QP_solver_beta10, double_integrator_QP_solver_bmy10);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_bmy10, double_integrator_QP_solver_yy10);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_yy10, double_integrator_QP_solver_dvcc10);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_dvcc10, double_integrator_QP_solver_yy09, double_integrator_QP_solver_bmy09);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld09, double_integrator_QP_solver_bmy09, double_integrator_QP_solver_dvcc09);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd09, double_integrator_QP_solver_dvcc09, double_integrator_QP_solver_yy08, double_integrator_QP_solver_bmy08);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld08, double_integrator_QP_solver_bmy08, double_integrator_QP_solver_dvcc08);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd08, double_integrator_QP_solver_dvcc08, double_integrator_QP_solver_yy07, double_integrator_QP_solver_bmy07);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld07, double_integrator_QP_solver_bmy07, double_integrator_QP_solver_dvcc07);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd07, double_integrator_QP_solver_dvcc07, double_integrator_QP_solver_yy06, double_integrator_QP_solver_bmy06);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld06, double_integrator_QP_solver_bmy06, double_integrator_QP_solver_dvcc06);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd06, double_integrator_QP_solver_dvcc06, double_integrator_QP_solver_yy05, double_integrator_QP_solver_bmy05);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld05, double_integrator_QP_solver_bmy05, double_integrator_QP_solver_dvcc05);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd05, double_integrator_QP_solver_dvcc05, double_integrator_QP_solver_yy04, double_integrator_QP_solver_bmy04);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld04, double_integrator_QP_solver_bmy04, double_integrator_QP_solver_dvcc04);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd04, double_integrator_QP_solver_dvcc04, double_integrator_QP_solver_yy03, double_integrator_QP_solver_bmy03);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld03, double_integrator_QP_solver_bmy03, double_integrator_QP_solver_dvcc03);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd03, double_integrator_QP_solver_dvcc03, double_integrator_QP_solver_yy02, double_integrator_QP_solver_bmy02);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld02, double_integrator_QP_solver_bmy02, double_integrator_QP_solver_dvcc02);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_5(double_integrator_QP_solver_Lsd02, double_integrator_QP_solver_dvcc02, double_integrator_QP_solver_yy01, double_integrator_QP_solver_bmy01);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld01, double_integrator_QP_solver_bmy01, double_integrator_QP_solver_dvcc01);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_9(double_integrator_QP_solver_Lsd01, double_integrator_QP_solver_dvcc01, double_integrator_QP_solver_yy00, double_integrator_QP_solver_bmy00);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_9(double_integrator_QP_solver_Ld00, double_integrator_QP_solver_bmy00, double_integrator_QP_solver_dvcc00);
double_integrator_QP_solver_LA_DENSE_MTVM_9_15(params->C1, double_integrator_QP_solver_dvcc00, double_integrator_QP_solver_grad_eq00);
double_integrator_QP_solver_LA_DENSE_MTVM2_5_15_9(params->C2, double_integrator_QP_solver_dvcc01, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc00, double_integrator_QP_solver_grad_eq01);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C3, double_integrator_QP_solver_dvcc02, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvcc01, double_integrator_QP_solver_grad_eq02);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C4, double_integrator_QP_solver_dvcc03, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvcc02, double_integrator_QP_solver_grad_eq03);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C5, double_integrator_QP_solver_dvcc04, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvcc03, double_integrator_QP_solver_grad_eq04);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C6, double_integrator_QP_solver_dvcc05, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvcc04, double_integrator_QP_solver_grad_eq05);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C7, double_integrator_QP_solver_dvcc06, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvcc05, double_integrator_QP_solver_grad_eq06);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C8, double_integrator_QP_solver_dvcc07, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvcc06, double_integrator_QP_solver_grad_eq07);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C9, double_integrator_QP_solver_dvcc08, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvcc07, double_integrator_QP_solver_grad_eq08);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C10, double_integrator_QP_solver_dvcc09, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvcc08, double_integrator_QP_solver_grad_eq09);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_15_5(params->C11, double_integrator_QP_solver_dvcc10, double_integrator_QP_solver_D02, double_integrator_QP_solver_dvcc09, double_integrator_QP_solver_grad_eq10);
double_integrator_QP_solver_LA_DIAGZERO_MTVM_5_5(double_integrator_QP_solver_D11, double_integrator_QP_solver_dvcc10, double_integrator_QP_solver_grad_eq11);
double_integrator_QP_solver_LA_VSUB_170(double_integrator_QP_solver_rd, double_integrator_QP_solver_grad_eq, double_integrator_QP_solver_rd);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_rd00, double_integrator_QP_solver_dzcc00);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_rd01, double_integrator_QP_solver_dzcc01);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_rd02, double_integrator_QP_solver_dzcc02);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_rd03, double_integrator_QP_solver_dzcc03);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_rd04, double_integrator_QP_solver_dzcc04);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_rd05, double_integrator_QP_solver_dzcc05);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_rd06, double_integrator_QP_solver_dzcc06);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_rd07, double_integrator_QP_solver_dzcc07);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_rd08, double_integrator_QP_solver_dzcc08);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_rd09, double_integrator_QP_solver_dzcc09);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_rd10, double_integrator_QP_solver_dzcc10);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_5(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_rd11, double_integrator_QP_solver_dzcc11);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_15(double_integrator_QP_solver_ccrhsl00, double_integrator_QP_solver_slb00, double_integrator_QP_solver_llbbyslb00, double_integrator_QP_solver_dzcc00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_dllbcc00);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_7(double_integrator_QP_solver_ccrhsub00, double_integrator_QP_solver_sub00, double_integrator_QP_solver_lubbysub00, double_integrator_QP_solver_dzcc00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_dlubcc00);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_15(double_integrator_QP_solver_ccrhsl01, double_integrator_QP_solver_slb01, double_integrator_QP_solver_llbbyslb01, double_integrator_QP_solver_dzcc01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_dllbcc01);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_7(double_integrator_QP_solver_ccrhsub01, double_integrator_QP_solver_sub01, double_integrator_QP_solver_lubbysub01, double_integrator_QP_solver_dzcc01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_dlubcc01);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_15(double_integrator_QP_solver_ccrhsl02, double_integrator_QP_solver_slb02, double_integrator_QP_solver_llbbyslb02, double_integrator_QP_solver_dzcc02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_dllbcc02);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_7(double_integrator_QP_solver_ccrhsub02, double_integrator_QP_solver_sub02, double_integrator_QP_solver_lubbysub02, double_integrator_QP_solver_dzcc02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_dlubcc02);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_15(double_integrator_QP_solver_ccrhsl03, double_integrator_QP_solver_slb03, double_integrator_QP_solver_llbbyslb03, double_integrator_QP_solver_dzcc03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_dllbcc03);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_7(double_integrator_QP_solver_ccrhsub03, double_integrator_QP_solver_sub03, double_integrator_QP_solver_lubbysub03, double_integrator_QP_solver_dzcc03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_dlubcc03);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_15(double_integrator_QP_solver_ccrhsl04, double_integrator_QP_solver_slb04, double_integrator_QP_solver_llbbyslb04, double_integrator_QP_solver_dzcc04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_dllbcc04);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_7(double_integrator_QP_solver_ccrhsub04, double_integrator_QP_solver_sub04, double_integrator_QP_solver_lubbysub04, double_integrator_QP_solver_dzcc04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_dlubcc04);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_15(double_integrator_QP_solver_ccrhsl05, double_integrator_QP_solver_slb05, double_integrator_QP_solver_llbbyslb05, double_integrator_QP_solver_dzcc05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_dllbcc05);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_7(double_integrator_QP_solver_ccrhsub05, double_integrator_QP_solver_sub05, double_integrator_QP_solver_lubbysub05, double_integrator_QP_solver_dzcc05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_dlubcc05);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_15(double_integrator_QP_solver_ccrhsl06, double_integrator_QP_solver_slb06, double_integrator_QP_solver_llbbyslb06, double_integrator_QP_solver_dzcc06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_dllbcc06);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_7(double_integrator_QP_solver_ccrhsub06, double_integrator_QP_solver_sub06, double_integrator_QP_solver_lubbysub06, double_integrator_QP_solver_dzcc06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_dlubcc06);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_15(double_integrator_QP_solver_ccrhsl07, double_integrator_QP_solver_slb07, double_integrator_QP_solver_llbbyslb07, double_integrator_QP_solver_dzcc07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_dllbcc07);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_7(double_integrator_QP_solver_ccrhsub07, double_integrator_QP_solver_sub07, double_integrator_QP_solver_lubbysub07, double_integrator_QP_solver_dzcc07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_dlubcc07);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_15(double_integrator_QP_solver_ccrhsl08, double_integrator_QP_solver_slb08, double_integrator_QP_solver_llbbyslb08, double_integrator_QP_solver_dzcc08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_dllbcc08);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_7(double_integrator_QP_solver_ccrhsub08, double_integrator_QP_solver_sub08, double_integrator_QP_solver_lubbysub08, double_integrator_QP_solver_dzcc08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_dlubcc08);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_15(double_integrator_QP_solver_ccrhsl09, double_integrator_QP_solver_slb09, double_integrator_QP_solver_llbbyslb09, double_integrator_QP_solver_dzcc09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_dllbcc09);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_7(double_integrator_QP_solver_ccrhsub09, double_integrator_QP_solver_sub09, double_integrator_QP_solver_lubbysub09, double_integrator_QP_solver_dzcc09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_dlubcc09);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_15(double_integrator_QP_solver_ccrhsl10, double_integrator_QP_solver_slb10, double_integrator_QP_solver_llbbyslb10, double_integrator_QP_solver_dzcc10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_dllbcc10);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_7(double_integrator_QP_solver_ccrhsub10, double_integrator_QP_solver_sub10, double_integrator_QP_solver_lubbysub10, double_integrator_QP_solver_dzcc10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_dlubcc10);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_5(double_integrator_QP_solver_ccrhsl11, double_integrator_QP_solver_slb11, double_integrator_QP_solver_llbbyslb11, double_integrator_QP_solver_dzcc11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_dllbcc11);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_5(double_integrator_QP_solver_ccrhsub11, double_integrator_QP_solver_sub11, double_integrator_QP_solver_lubbysub11, double_integrator_QP_solver_dzcc11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_dlubcc11);
double_integrator_QP_solver_LA_VSUB7_252(double_integrator_QP_solver_l, double_integrator_QP_solver_ccrhs, double_integrator_QP_solver_s, double_integrator_QP_solver_dl_cc, double_integrator_QP_solver_ds_cc);
double_integrator_QP_solver_LA_VADD_170(double_integrator_QP_solver_dz_cc, double_integrator_QP_solver_dz_aff);
double_integrator_QP_solver_LA_VADD_59(double_integrator_QP_solver_dv_cc, double_integrator_QP_solver_dv_aff);
double_integrator_QP_solver_LA_VADD_252(double_integrator_QP_solver_dl_cc, double_integrator_QP_solver_dl_aff);
double_integrator_QP_solver_LA_VADD_252(double_integrator_QP_solver_ds_cc, double_integrator_QP_solver_ds_aff);
info->lsit_cc = double_integrator_QP_solver_LINESEARCH_BACKTRACKING_COMBINED(double_integrator_QP_solver_z, double_integrator_QP_solver_v, double_integrator_QP_solver_l, double_integrator_QP_solver_s, double_integrator_QP_solver_dz_cc, double_integrator_QP_solver_dv_cc, double_integrator_QP_solver_dl_cc, double_integrator_QP_solver_ds_cc, &info->step_cc, &info->mu);
if( info->lsit_cc == double_integrator_QP_solver_NOPROGRESS ){
exitcode = double_integrator_QP_solver_NOPROGRESS; break;
}
info->it++;
}
output->z1[0] = double_integrator_QP_solver_z00[0];
output->z1[1] = double_integrator_QP_solver_z00[1];
output->z1[2] = double_integrator_QP_solver_z00[2];
output->z1[3] = double_integrator_QP_solver_z00[3];
output->z1[4] = double_integrator_QP_solver_z00[4];
output->z1[5] = double_integrator_QP_solver_z00[5];
output->z1[6] = double_integrator_QP_solver_z00[6];
output->z2[0] = double_integrator_QP_solver_z01[0];
output->z2[1] = double_integrator_QP_solver_z01[1];
output->z2[2] = double_integrator_QP_solver_z01[2];
output->z2[3] = double_integrator_QP_solver_z01[3];
output->z2[4] = double_integrator_QP_solver_z01[4];
output->z2[5] = double_integrator_QP_solver_z01[5];
output->z2[6] = double_integrator_QP_solver_z01[6];
output->z3[0] = double_integrator_QP_solver_z02[0];
output->z3[1] = double_integrator_QP_solver_z02[1];
output->z3[2] = double_integrator_QP_solver_z02[2];
output->z3[3] = double_integrator_QP_solver_z02[3];
output->z3[4] = double_integrator_QP_solver_z02[4];
output->z3[5] = double_integrator_QP_solver_z02[5];
output->z3[6] = double_integrator_QP_solver_z02[6];
output->z4[0] = double_integrator_QP_solver_z03[0];
output->z4[1] = double_integrator_QP_solver_z03[1];
output->z4[2] = double_integrator_QP_solver_z03[2];
output->z4[3] = double_integrator_QP_solver_z03[3];
output->z4[4] = double_integrator_QP_solver_z03[4];
output->z4[5] = double_integrator_QP_solver_z03[5];
output->z4[6] = double_integrator_QP_solver_z03[6];
output->z5[0] = double_integrator_QP_solver_z04[0];
output->z5[1] = double_integrator_QP_solver_z04[1];
output->z5[2] = double_integrator_QP_solver_z04[2];
output->z5[3] = double_integrator_QP_solver_z04[3];
output->z5[4] = double_integrator_QP_solver_z04[4];
output->z5[5] = double_integrator_QP_solver_z04[5];
output->z5[6] = double_integrator_QP_solver_z04[6];
output->z6[0] = double_integrator_QP_solver_z05[0];
output->z6[1] = double_integrator_QP_solver_z05[1];
output->z6[2] = double_integrator_QP_solver_z05[2];
output->z6[3] = double_integrator_QP_solver_z05[3];
output->z6[4] = double_integrator_QP_solver_z05[4];
output->z6[5] = double_integrator_QP_solver_z05[5];
output->z6[6] = double_integrator_QP_solver_z05[6];
output->z7[0] = double_integrator_QP_solver_z06[0];
output->z7[1] = double_integrator_QP_solver_z06[1];
output->z7[2] = double_integrator_QP_solver_z06[2];
output->z7[3] = double_integrator_QP_solver_z06[3];
output->z7[4] = double_integrator_QP_solver_z06[4];
output->z7[5] = double_integrator_QP_solver_z06[5];
output->z7[6] = double_integrator_QP_solver_z06[6];
output->z8[0] = double_integrator_QP_solver_z07[0];
output->z8[1] = double_integrator_QP_solver_z07[1];
output->z8[2] = double_integrator_QP_solver_z07[2];
output->z8[3] = double_integrator_QP_solver_z07[3];
output->z8[4] = double_integrator_QP_solver_z07[4];
output->z8[5] = double_integrator_QP_solver_z07[5];
output->z8[6] = double_integrator_QP_solver_z07[6];
output->z9[0] = double_integrator_QP_solver_z08[0];
output->z9[1] = double_integrator_QP_solver_z08[1];
output->z9[2] = double_integrator_QP_solver_z08[2];
output->z9[3] = double_integrator_QP_solver_z08[3];
output->z9[4] = double_integrator_QP_solver_z08[4];
output->z9[5] = double_integrator_QP_solver_z08[5];
output->z9[6] = double_integrator_QP_solver_z08[6];
output->z10[0] = double_integrator_QP_solver_z09[0];
output->z10[1] = double_integrator_QP_solver_z09[1];
output->z10[2] = double_integrator_QP_solver_z09[2];
output->z10[3] = double_integrator_QP_solver_z09[3];
output->z10[4] = double_integrator_QP_solver_z09[4];
output->z10[5] = double_integrator_QP_solver_z09[5];
output->z10[6] = double_integrator_QP_solver_z09[6];
output->z11[0] = double_integrator_QP_solver_z10[0];
output->z11[1] = double_integrator_QP_solver_z10[1];
output->z11[2] = double_integrator_QP_solver_z10[2];
output->z11[3] = double_integrator_QP_solver_z10[3];
output->z11[4] = double_integrator_QP_solver_z10[4];
output->z11[5] = double_integrator_QP_solver_z10[5];
output->z11[6] = double_integrator_QP_solver_z10[6];
output->z12[0] = double_integrator_QP_solver_z11[0];
output->z12[1] = double_integrator_QP_solver_z11[1];
output->z12[2] = double_integrator_QP_solver_z11[2];
output->z12[3] = double_integrator_QP_solver_z11[3];
output->z12[4] = double_integrator_QP_solver_z11[4];

#if double_integrator_QP_solver_SET_TIMING == 1
info->solvetime = double_integrator_QP_solver_toc(&solvertimer);
#if double_integrator_QP_solver_SET_PRINTLEVEL > 0 && double_integrator_QP_solver_SET_TIMING == 1
if( info->it > 1 ){
	PRINTTEXT("Solve time: %5.3f ms (%d iterations)\n\n", info->solvetime*1000, info->it);
} else {
	PRINTTEXT("Solve time: %5.3f ms (%d iteration)\n\n", info->solvetime*1000, info->it);
}
#endif
#else
info->solvetime = -1;
#endif
return exitcode;
}
