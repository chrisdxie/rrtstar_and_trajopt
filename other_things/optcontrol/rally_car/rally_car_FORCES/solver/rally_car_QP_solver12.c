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

#include "rally_car_QP_solver.h"

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

typedef struct rally_car_QP_solver_timer{
	LARGE_INTEGER tic;
	LARGE_INTEGER toc;
	LARGE_INTEGER freq;
} rally_car_QP_solver_timer;


void rally_car_QP_solver_tic(rally_car_QP_solver_timer* t)
{
	QueryPerformanceFrequency(&t->freq);
	QueryPerformanceCounter(&t->tic);
}



rally_car_QP_solver_FLOAT rally_car_QP_solver_toc(rally_car_QP_solver_timer* t)
{
	QueryPerformanceCounter(&t->toc);
	return ((t->toc.QuadPart - t->tic.QuadPart) / (rally_car_QP_solver_FLOAT)t->freq.QuadPart);
}


/* WE ARE ON THE MAC */
#elif (defined __APPLE__)
#include <mach/mach_time.h>


/* Use MAC OSX  mach_time for timing */
typedef struct rally_car_QP_solver_timer{
	uint64_t tic;
	uint64_t toc;
	mach_timebase_info_data_t tinfo;

} rally_car_QP_solver_timer;


void rally_car_QP_solver_tic(rally_car_QP_solver_timer* t)
{
    /* read current clock cycles */
    t->tic = mach_absolute_time();
}



rally_car_QP_solver_FLOAT rally_car_QP_solver_toc(rally_car_QP_solver_timer* t)
{
    uint64_t duration; /* elapsed time in clock cycles*/
    t->toc = mach_absolute_time();
	duration = t->toc - t->tic;

    /*conversion from clock cycles to nanoseconds*/
    mach_timebase_info(&(t->tinfo));
    duration *= t->tinfo.numer;
    duration /= t->tinfo.denom;

    return (rally_car_QP_solver_FLOAT)duration / 1000000000;
}

/* WE ARE ON SOME TEXAS INSTRUMENTS PLATFORM */
#elif (defined __TI_COMPILER_VERSION__)

/* TimeStamps */
#include <c6x.h> /* make use of TSCL, TSCH */


typedef struct rally_car_QP_solver_timer{
	unsigned long long tic;
	unsigned long long toc;
} rally_car_QP_solver_timer;


void rally_car_QP_solver_tic(rally_car_QP_solver_timer* t)
{
	TSCL = 0;	/* Initiate CPU timer by writing any val to TSCL */
	t->tic = _itoll( TSCH, TSCL );
}



rally_car_QP_solver_FLOAT rally_car_QP_solver_toc(rally_car_QP_solver_timer* t)
{
	t->toc = _itoll( TSCH, TSCL );
	unsigned long long t0;
	unsigned long long overhead;
	t0 = _itoll( TSCH, TSCL );
	overhead = _itoll( TSCH, TSCL )  - t0;

	return (rally_car_QP_solver_FLOAT)(t->toc - t->tic - overhead) / 1000000000;
}



/* WE ARE ON SOME OTHER UNIX/LINUX SYSTEM */
#else

/* Use POSIX clocl_gettime() for timing on non-Windows machines */
#include <time.h>
typedef struct rally_car_QP_solver_timer{
	struct timespec tic;
	struct timespec toc;
} rally_car_QP_solver_timer;


/* read current time */
void rally_car_QP_solver_tic(rally_car_QP_solver_timer* t)
{
	clock_gettime(CLOCK_MONOTONIC, &t->tic);
}



/* return time passed since last call to tic on this timer */
double rally_car_QP_solver_toc(rally_car_QP_solver_timer* t)
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

	return (rally_car_QP_solver_FLOAT)temp.tv_sec + (rally_car_QP_solver_FLOAT)temp.tv_nsec / 1000000000;
}


#endif

/* LINEAR ALGEBRA LIBRARY ---------------------------------------------- */
/*
 * Initializes a vector of length 317 with a value.
 */
void rally_car_QP_solver_LA_INITIALIZEVECTOR_317(rally_car_QP_solver_FLOAT* vec, rally_car_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<317; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 107 with a value.
 */
void rally_car_QP_solver_LA_INITIALIZEVECTOR_107(rally_car_QP_solver_FLOAT* vec, rally_car_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<107; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 458 with a value.
 */
void rally_car_QP_solver_LA_INITIALIZEVECTOR_458(rally_car_QP_solver_FLOAT* vec, rally_car_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<458; i++ )
	{
		vec[i] = value;
	}
}


/* 
 * Calculates a dot product and adds it to a variable: z += x'*y; 
 * This function is for vectors of length 458.
 */
void rally_car_QP_solver_LA_DOTACC_458(rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *y, rally_car_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<458; i++ ){
		*z += x[i]*y[i];
	}
}


/*
 * Calculates the gradient and the value for a quadratic function 0.5*z'*H*z + f'*z
 *
 * INPUTS:     H  - Symmetric Hessian, diag matrix of size [28 x 28]
 *             f  - column vector of size 28
 *             z  - column vector of size 28
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 28
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void rally_car_QP_solver_LA_DIAG_QUADFCN_28(rally_car_QP_solver_FLOAT* H, rally_car_QP_solver_FLOAT* f, rally_car_QP_solver_FLOAT* z, rally_car_QP_solver_FLOAT* grad, rally_car_QP_solver_FLOAT* value)
{
	int i;
	rally_car_QP_solver_FLOAT hz;	
	for( i=0; i<28; i++){
		hz = H[i]*z[i];
		grad[i] = hz + f[i];
		*value += 0.5*hz*z[i] + f[i]*z[i];
	}
}


/*
 * Calculates the gradient and the value for a quadratic function 0.5*z'*H*z + f'*z
 *
 * INPUTS:     H  - Symmetric Hessian, diag matrix of size [9 x 9]
 *             f  - column vector of size 9
 *             z  - column vector of size 9
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 9
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void rally_car_QP_solver_LA_DIAG_QUADFCN_9(rally_car_QP_solver_FLOAT* H, rally_car_QP_solver_FLOAT* f, rally_car_QP_solver_FLOAT* z, rally_car_QP_solver_FLOAT* grad, rally_car_QP_solver_FLOAT* value)
{
	int i;
	rally_car_QP_solver_FLOAT hz;	
	for( i=0; i<9; i++){
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
void rally_car_QP_solver_LA_DENSE_MVMSUB3_17_28_28(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *b, rally_car_QP_solver_FLOAT *l, rally_car_QP_solver_FLOAT *r, rally_car_QP_solver_FLOAT *z, rally_car_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;
	rally_car_QP_solver_FLOAT AxBu[17];
	rally_car_QP_solver_FLOAT norm = *y;
	rally_car_QP_solver_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<17; i++ ){
		AxBu[i] = A[k++]*x[0] + B[m++]*u[0];
	}	
	for( j=1; j<28; j++ ){		
		for( i=0; i<17; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<28; n++ ){
		for( i=0; i<17; i++ ){
			AxBu[i] += B[m++]*u[n];
		}		
	}

	for( i=0; i<17; i++ ){
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
void rally_car_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_9_28_28(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *b, rally_car_QP_solver_FLOAT *l, rally_car_QP_solver_FLOAT *r, rally_car_QP_solver_FLOAT *z, rally_car_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	rally_car_QP_solver_FLOAT AxBu[9];
	rally_car_QP_solver_FLOAT norm = *y;
	rally_car_QP_solver_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<9; i++ ){
		AxBu[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<28; j++ ){		
		for( i=0; i<9; i++ ){
			AxBu[i] += A[k++]*x[j];
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
void rally_car_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_9_28_9(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *b, rally_car_QP_solver_FLOAT *l, rally_car_QP_solver_FLOAT *r, rally_car_QP_solver_FLOAT *z, rally_car_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	rally_car_QP_solver_FLOAT AxBu[9];
	rally_car_QP_solver_FLOAT norm = *y;
	rally_car_QP_solver_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<9; i++ ){
		AxBu[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<28; j++ ){		
		for( i=0; i<9; i++ ){
			AxBu[i] += A[k++]*x[j];
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
 * Matrix vector multiplication y = M'*x where M is of size [17 x 28]
 * and stored in column major format. Note the transpose of M!
 */
void rally_car_QP_solver_LA_DENSE_MTVM_17_28(rally_car_QP_solver_FLOAT *M, rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<28; i++ ){
		y[i] = 0;
		for( j=0; j<17; j++ ){
			y[i] += M[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [9 x 28]
 * and B is of size [17 x 28]
 * and stored in column major format. Note the transposes of A and B!
 */
void rally_car_QP_solver_LA_DENSE_MTVM2_9_28_17(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *y, rally_car_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	int n;
	int m = 0;
	for( i=0; i<28; i++ ){
		z[i] = 0;
		for( j=0; j<9; j++ ){
			z[i] += A[k++]*x[j];
		}
		for( n=0; n<17; n++ ){
			z[i] += B[m++]*y[n];
		}
	}
}


/*
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [9 x 28] and stored in column major format.
 * and B is of size [9 x 28] and stored in diagzero format
 * Note the transposes of A and B!
 */
void rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *y, rally_car_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	for( i=0; i<9; i++ ){
		z[i] = 0;
		for( j=0; j<9; j++ ){
			z[i] += A[k++]*x[j];
		}
		z[i] += B[i]*y[i];
	}
	for( i=9 ;i<28; i++ ){
		z[i] = 0;
		for( j=0; j<9; j++ ){
			z[i] += A[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication y = M'*x where M is of size [9 x 9]
 * and stored in diagzero format. Note the transpose of M!
 */
void rally_car_QP_solver_LA_DIAGZERO_MTVM_9_9(rally_car_QP_solver_FLOAT *M, rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<9; i++ ){
		y[i] = M[i]*x[i];
	}
}


/*
 * Vector subtraction and addition.
 *	 Input: five vectors t, tidx, u, v, w and two scalars z and r
 *	 Output: y = t(tidx) - u + w
 *           z = z - v'*x;
 *           r = max([norm(y,inf), z]);
 * for vectors of length 28. Output z is of course scalar.
 */
void rally_car_QP_solver_LA_VSUBADD3_28(rally_car_QP_solver_FLOAT* t, rally_car_QP_solver_FLOAT* u, int* uidx, rally_car_QP_solver_FLOAT* v, rally_car_QP_solver_FLOAT* w, rally_car_QP_solver_FLOAT* y, rally_car_QP_solver_FLOAT* z, rally_car_QP_solver_FLOAT* r)
{
	int i;
	rally_car_QP_solver_FLOAT norm = *r;
	rally_car_QP_solver_FLOAT vx = 0;
	rally_car_QP_solver_FLOAT x;
	for( i=0; i<28; i++){
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
 * for vectors of length 12. Output z is of course scalar.
 */
void rally_car_QP_solver_LA_VSUBADD2_12(rally_car_QP_solver_FLOAT* t, int* tidx, rally_car_QP_solver_FLOAT* u, rally_car_QP_solver_FLOAT* v, rally_car_QP_solver_FLOAT* w, rally_car_QP_solver_FLOAT* y, rally_car_QP_solver_FLOAT* z, rally_car_QP_solver_FLOAT* r)
{
	int i;
	rally_car_QP_solver_FLOAT norm = *r;
	rally_car_QP_solver_FLOAT vx = 0;
	rally_car_QP_solver_FLOAT x;
	for( i=0; i<12; i++){
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
 * for vectors of length 9. Output z is of course scalar.
 */
void rally_car_QP_solver_LA_VSUBADD3_9(rally_car_QP_solver_FLOAT* t, rally_car_QP_solver_FLOAT* u, int* uidx, rally_car_QP_solver_FLOAT* v, rally_car_QP_solver_FLOAT* w, rally_car_QP_solver_FLOAT* y, rally_car_QP_solver_FLOAT* z, rally_car_QP_solver_FLOAT* r)
{
	int i;
	rally_car_QP_solver_FLOAT norm = *r;
	rally_car_QP_solver_FLOAT vx = 0;
	rally_car_QP_solver_FLOAT x;
	for( i=0; i<9; i++){
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
 * for vectors of length 9. Output z is of course scalar.
 */
void rally_car_QP_solver_LA_VSUBADD2_9(rally_car_QP_solver_FLOAT* t, int* tidx, rally_car_QP_solver_FLOAT* u, rally_car_QP_solver_FLOAT* v, rally_car_QP_solver_FLOAT* w, rally_car_QP_solver_FLOAT* y, rally_car_QP_solver_FLOAT* z, rally_car_QP_solver_FLOAT* r)
{
	int i;
	rally_car_QP_solver_FLOAT norm = *r;
	rally_car_QP_solver_FLOAT vx = 0;
	rally_car_QP_solver_FLOAT x;
	for( i=0; i<9; i++){
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
 * Special function for box constraints of length 28
 * Returns also L/S, a value that is often used elsewhere.
 */
void rally_car_QP_solver_LA_INEQ_B_GRAD_28_28_12(rally_car_QP_solver_FLOAT *lu, rally_car_QP_solver_FLOAT *su, rally_car_QP_solver_FLOAT *ru, rally_car_QP_solver_FLOAT *ll, rally_car_QP_solver_FLOAT *sl, rally_car_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, rally_car_QP_solver_FLOAT *grad, rally_car_QP_solver_FLOAT *lubysu, rally_car_QP_solver_FLOAT *llbysl)
{
	int i;
	for( i=0; i<28; i++ ){
		grad[i] = 0;
	}
	for( i=0; i<28; i++ ){		
		llbysl[i] = ll[i] / sl[i];
		grad[lbIdx[i]] -= llbysl[i]*rl[i];
	}
	for( i=0; i<12; i++ ){
		lubysu[i] = lu[i] / su[i];
		grad[ubIdx[i]] += lubysu[i]*ru[i];
	}
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 9
 * Returns also L/S, a value that is often used elsewhere.
 */
void rally_car_QP_solver_LA_INEQ_B_GRAD_9_9_9(rally_car_QP_solver_FLOAT *lu, rally_car_QP_solver_FLOAT *su, rally_car_QP_solver_FLOAT *ru, rally_car_QP_solver_FLOAT *ll, rally_car_QP_solver_FLOAT *sl, rally_car_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, rally_car_QP_solver_FLOAT *grad, rally_car_QP_solver_FLOAT *lubysu, rally_car_QP_solver_FLOAT *llbysl)
{
	int i;
	for( i=0; i<9; i++ ){
		grad[i] = 0;
	}
	for( i=0; i<9; i++ ){		
		llbysl[i] = ll[i] / sl[i];
		grad[lbIdx[i]] -= llbysl[i]*rl[i];
	}
	for( i=0; i<9; i++ ){
		lubysu[i] = lu[i] / su[i];
		grad[ubIdx[i]] += lubysu[i]*ru[i];
	}
}


/*
 * Addition of three vectors  z = u + w + v
 * of length 317.
 */
void rally_car_QP_solver_LA_VVADD3_317(rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *v, rally_car_QP_solver_FLOAT *w, rally_car_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<317; i++ ){
		z[i] = u[i] + v[i] + w[i];
	}
}


/*
 * Special function to compute the diagonal cholesky factorization of the 
 * positive definite augmented Hessian for block size 28.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = sqrt(H + diag(llbysl) + diag(lubysu))
 * where Phi is stored in diagonal storage format
 */
void rally_car_QP_solver_LA_DIAG_CHOL_LBUB_28_28_12(rally_car_QP_solver_FLOAT *H, rally_car_QP_solver_FLOAT *llbysl, int* lbIdx, rally_car_QP_solver_FLOAT *lubysu, int* ubIdx, rally_car_QP_solver_FLOAT *Phi)


{
	int i;
	
	/* copy  H into PHI */
	for( i=0; i<28; i++ ){
		Phi[i] = H[i];
	}

	/* add llbysl onto Phi where necessary */
	for( i=0; i<28; i++ ){
		Phi[lbIdx[i]] += llbysl[i];
	}

	/* add lubysu onto Phi where necessary */
	for( i=0; i<12; i++){
		Phi[ubIdx[i]] +=  lubysu[i];
	}
	
	/* compute cholesky */
	for(i=0; i<28; i++)
	{
#if rally_car_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
 * where A is to be computed and is of size [17 x 28],
 * B is given and of size [17 x 28], L is a diagonal
 * matrix of size 17 stored in diagonal matrix 
 * storage format. Note the transpose of L has no impact!
 *
 * Result: A in column major storage format.
 *
 */
void rally_car_QP_solver_LA_DIAG_MATRIXFORWARDSUB_17_28(rally_car_QP_solver_FLOAT *L, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *A)
{
    int i,j;
	 int k = 0;

	for( j=0; j<28; j++){
		for( i=0; i<17; i++){
			A[k] = B[k]/L[j];
			k++;
		}
	}

}


/**
 * Forward substitution to solve L*y = b where L is a
 * diagonal matrix in vector storage format.
 * 
 * The dimensions involved are 28.
 */
void rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_FLOAT *L, rally_car_QP_solver_FLOAT *b, rally_car_QP_solver_FLOAT *y)
{
    int i;

    for( i=0; i<28; i++ ){
		y[i] = b[i]/L[i];
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [9 x 28],
 * B is given and of size [9 x 28], L is a diagonal
 * matrix of size 9 stored in diagonal matrix 
 * storage format. Note the transpose of L has no impact!
 *
 * Result: A in column major storage format.
 *
 */
void rally_car_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_28(rally_car_QP_solver_FLOAT *L, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *A)
{
    int i,j;
	 int k = 0;

	for( j=0; j<28; j++){
		for( i=0; i<9; i++){
			A[k] = B[k]/L[j];
			k++;
		}
	}

}


/**
 * Compute C = A*B' where 
 *
 *	size(A) = [17 x 28]
 *  size(B) = [9 x 28]
 * 
 * and all matrices are stored in column major format.
 *
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE.  
 * 
 */
void rally_car_QP_solver_LA_DENSE_MMTM_17_28_9(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *C)
{
    int i, j, k;
    rally_car_QP_solver_FLOAT temp;
    
    for( i=0; i<17; i++ ){        
        for( j=0; j<9; j++ ){
            temp = 0; 
            for( k=0; k<28; k++ ){
                temp += A[k*17+i]*B[k*9+j];
            }						
            C[j*17+i] = temp;
        }
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [9 x 28],
 * B is given and of size [9 x 28], L is a diagonal
 *  matrix of size 28 stored in diagonal 
 * storage format. Note the transpose of L!
 *
 * Result: A in diagonalzero storage format.
 *
 */
void rally_car_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_9_28(rally_car_QP_solver_FLOAT *L, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *A)
{
	int j;
    for( j=0; j<28; j++ ){   
		A[j] = B[j]/L[j];
     }
}


/**
 * Compute C = A*B' where 
 *
 *	size(A) = [9 x 28]
 *  size(B) = [9 x 28] in diagzero format
 * 
 * A and C matrices are stored in column major format.
 * 
 * 
 */
void rally_car_QP_solver_LA_DENSE_DIAGZERO_MMTM_9_28_9(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *C)
{
    int i, j;
	
	for( i=0; i<9; i++ ){
		for( j=0; j<9; j++){
			C[j*9+i] = B[i*9+j]*A[i];
		}
	}

}


/*
 * Special function to compute the diagonal cholesky factorization of the 
 * positive definite augmented Hessian for block size 9.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = sqrt(H + diag(llbysl) + diag(lubysu))
 * where Phi is stored in diagonal storage format
 */
void rally_car_QP_solver_LA_DIAG_CHOL_ONELOOP_LBUB_9_9_9(rally_car_QP_solver_FLOAT *H, rally_car_QP_solver_FLOAT *llbysl, int* lbIdx, rally_car_QP_solver_FLOAT *lubysu, int* ubIdx, rally_car_QP_solver_FLOAT *Phi)


{
	int i;
	
	/* compute cholesky */
	for( i=0; i<9; i++ ){
		Phi[i] = H[i] + llbysl[i] + lubysu[i];

#if rally_car_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
 * where A is to be computed and is of size [9 x 9],
 * B is given and of size [9 x 9], L is a diagonal
 *  matrix of size 9 stored in diagonal 
 * storage format. Note the transpose of L!
 *
 * Result: A in diagonalzero storage format.
 *
 */
void rally_car_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_9_9(rally_car_QP_solver_FLOAT *L, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *A)
{
	int j;
    for( j=0; j<9; j++ ){   
		A[j] = B[j]/L[j];
     }
}


/**
 * Forward substitution to solve L*y = b where L is a
 * diagonal matrix in vector storage format.
 * 
 * The dimensions involved are 9.
 */
void rally_car_QP_solver_LA_DIAG_FORWARDSUB_9(rally_car_QP_solver_FLOAT *L, rally_car_QP_solver_FLOAT *b, rally_car_QP_solver_FLOAT *y)
{
    int i;

    for( i=0; i<9; i++ ){
		y[i] = b[i]/L[i];
    }
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [17 x 28] in column
 * storage format, and B is of size [17 x 28] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void rally_car_QP_solver_LA_DENSE_MMT2_17_28_28(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    rally_car_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<17; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<28; k++ ){
                ltemp += A[k*17+i]*A[k*17+j];
            }			
			for( k=0; k<28; k++ ){
                ltemp += B[k*17+i]*B[k*17+j];
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
void rally_car_QP_solver_LA_DENSE_MVMSUB2_17_28_28(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *b, rally_car_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<17; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<28; j++ ){		
		for( i=0; i<17; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<28; n++ ){
		for( i=0; i<17; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [9 x 28] in column
 * storage format, and B is of size [9 x 28] diagonalzero
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void rally_car_QP_solver_LA_DENSE_DIAGZERO_MMT2_9_28_28(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    rally_car_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<9; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<28; k++ ){
                ltemp += A[k*9+i]*A[k*9+j];
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
void rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_9_28_28(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *b, rally_car_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<9; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[i]*u[i];
	}	

	for( j=1; j<28; j++ ){		
		for( i=0; i<9; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [9 x 28] in column
 * storage format, and B is of size [9 x 9] diagonalzero
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void rally_car_QP_solver_LA_DENSE_DIAGZERO_MMT2_9_28_9(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    rally_car_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<9; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<28; k++ ){
                ltemp += A[k*9+i]*A[k*9+j];
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
void rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_9_28_9(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *b, rally_car_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<9; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[i]*u[i];
	}	

	for( j=1; j<28; j++ ){		
		for( i=0; i<9; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 17 and outputting
 * the Cholesky factor to matrix L in lower triangular format.
 */
void rally_car_QP_solver_LA_DENSE_CHOL_17(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    rally_car_QP_solver_FLOAT l;
    rally_car_QP_solver_FLOAT Mii;

	/* copy A to L first and then operate on L */
	/* COULD BE OPTIMIZED */
	ii=0; di=0;
	for( i=0; i<17; i++ ){
		for( j=0; j<=i; j++ ){
			L[ii+j] = A[ii+j];
		}
		ii += ++di;
	}    
	
	/* factor L */
	ii=0; di=0;
    for( i=0; i<17; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += L[ii+k]*L[ii+k];
        }        
        
        Mii = L[ii+i] - l;
        
#if rally_car_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
        for( j=i+1; j<17; j++ ){
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
 * The dimensions involved are 17.
 */
void rally_car_QP_solver_LA_DENSE_FORWARDSUB_17(rally_car_QP_solver_FLOAT *L, rally_car_QP_solver_FLOAT *b, rally_car_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    rally_car_QP_solver_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<17; i++ ){
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
 * where A is to be computed and is of size [9 x 17],
 * B is given and of size [9 x 17], L is a lower tri-
 * angular matrix of size 17 stored in lower triangular 
 * storage format. Note the transpose of L AND B!
 *
 * Result: A in column major storage format.
 *
 */
void rally_car_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_9_17(rally_car_QP_solver_FLOAT *L, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *A)
{
    int i,j,k,ii,di;
    rally_car_QP_solver_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<17; j++ ){        
        for( i=0; i<9; i++ ){
            a = B[i*17+j];
            for( k=0; k<j; k++ ){
                a -= A[k*9+i]*L[ii+k];
            }    

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

			A[j*9+i] = a/L[ii+j];			
        }
        ii += ++di;
    }
}


/**
 * Compute L = L - A*A', where L is lower triangular of size 9
 * and A is a dense matrix of size [9 x 17] in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void rally_car_QP_solver_LA_DENSE_MMTSUB_9_17(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    rally_car_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<9; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<17; k++ ){
                ltemp += A[k*9+i]*A[k*9+j];
            }						
            L[ii+j] -= ltemp;
        }
        ii += ++di;
    }
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 9 and outputting
 * the Cholesky factor to matrix L in lower triangular format.
 */
void rally_car_QP_solver_LA_DENSE_CHOL_9(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    rally_car_QP_solver_FLOAT l;
    rally_car_QP_solver_FLOAT Mii;

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
        
#if rally_car_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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


/* 
 * Computes r = b - A*x
 * where A is stored in column major format
 */
void rally_car_QP_solver_LA_DENSE_MVMSUB1_9_17(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *b, rally_car_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<9; i++ ){
		r[i] = b[i] - A[k++]*x[0];
	}	
	for( j=1; j<17; j++ ){		
		for( i=0; i<9; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/**
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 9.
 */
void rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_FLOAT *L, rally_car_QP_solver_FLOAT *b, rally_car_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    rally_car_QP_solver_FLOAT yel;
            
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
 * where A is to be computed and is of size [9 x 9],
 * B is given and of size [9 x 9], L is a lower tri-
 * angular matrix of size 9 stored in lower triangular 
 * storage format. Note the transpose of L AND B!
 *
 * Result: A in column major storage format.
 *
 */
void rally_car_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_9_9(rally_car_QP_solver_FLOAT *L, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *A)
{
    int i,j,k,ii,di;
    rally_car_QP_solver_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<9; j++ ){        
        for( i=0; i<9; i++ ){
            a = B[i*9+j];
            for( k=0; k<j; k++ ){
                a -= A[k*9+i]*L[ii+k];
            }    

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

			A[j*9+i] = a/L[ii+j];			
        }
        ii += ++di;
    }
}


/**
 * Compute L = L - A*A', where L is lower triangular of size 9
 * and A is a dense matrix of size [9 x 9] in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void rally_car_QP_solver_LA_DENSE_MMTSUB_9_9(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    rally_car_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<9; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<9; k++ ){
                ltemp += A[k*9+i]*A[k*9+j];
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
void rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *b, rally_car_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<9; i++ ){
		r[i] = b[i] - A[k++]*x[0];
	}	
	for( j=1; j<9; j++ ){		
		for( i=0; i<9; i++ ){
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
void rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_FLOAT *L, rally_car_QP_solver_FLOAT *y, rally_car_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    rally_car_QP_solver_FLOAT xel;    
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
 * Matrix vector multiplication y = b - M'*x where M is of size [9 x 9]
 * and stored in column major format. Note the transpose of M!
 */
void rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *b, rally_car_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<9; i++ ){
		r[i] = b[i];
		for( j=0; j<9; j++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication y = b - M'*x where M is of size [9 x 17]
 * and stored in column major format. Note the transpose of M!
 */
void rally_car_QP_solver_LA_DENSE_MTVMSUB_9_17(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *b, rally_car_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<17; i++ ){
		r[i] = b[i];
		for( j=0; j<9; j++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/**
 * Backward Substitution to solve L^T*x = y where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * All involved dimensions are 17.
 */
void rally_car_QP_solver_LA_DENSE_BACKWARDSUB_17(rally_car_QP_solver_FLOAT *L, rally_car_QP_solver_FLOAT *y, rally_car_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    rally_car_QP_solver_FLOAT xel;    
	int start = 136;
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 16;
    for( i=16; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 16;
        for( j=16; j>i; j-- ){
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
 * Vector subtraction z = -x - y for vectors of length 317.
 */
void rally_car_QP_solver_LA_VSUB2_317(rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *y, rally_car_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<317; i++){
		z[i] = -x[i] - y[i];
	}
}


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * diagonal matrix of size 28 in vector
 * storage format.
 */
void rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_FLOAT *L, rally_car_QP_solver_FLOAT *b, rally_car_QP_solver_FLOAT *x)
{
    int i;
            
    /* solve Ly = b by forward and backward substitution */
    for( i=0; i<28; i++ ){
		x[i] = b[i]/(L[i]*L[i]);
    }
    
}


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * diagonal matrix of size 9 in vector
 * storage format.
 */
void rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_9(rally_car_QP_solver_FLOAT *L, rally_car_QP_solver_FLOAT *b, rally_car_QP_solver_FLOAT *x)
{
    int i;
            
    /* solve Ly = b by forward and backward substitution */
    for( i=0; i<9; i++ ){
		x[i] = b[i]/(L[i]*L[i]);
    }
    
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 28,
 * and x has length 28 and is indexed through yidx.
 */
void rally_car_QP_solver_LA_VSUB_INDEXED_28(rally_car_QP_solver_FLOAT *x, int* xidx, rally_car_QP_solver_FLOAT *y, rally_car_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<28; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 28.
 */
void rally_car_QP_solver_LA_VSUB3_28(rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *v, rally_car_QP_solver_FLOAT *w, rally_car_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<28; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = -x - y(yidx) where y is of length 28
 * and z, x and yidx are of length 12.
 */
void rally_car_QP_solver_LA_VSUB2_INDEXED_12(rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *y, int* yidx, rally_car_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<12; i++){
		z[i] = -x[i] - y[yidx[i]];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 12.
 */
void rally_car_QP_solver_LA_VSUB3_12(rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *v, rally_car_QP_solver_FLOAT *w, rally_car_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<12; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 9,
 * and x has length 9 and is indexed through yidx.
 */
void rally_car_QP_solver_LA_VSUB_INDEXED_9(rally_car_QP_solver_FLOAT *x, int* xidx, rally_car_QP_solver_FLOAT *y, rally_car_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<9; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 9.
 */
void rally_car_QP_solver_LA_VSUB3_9(rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *v, rally_car_QP_solver_FLOAT *w, rally_car_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<9; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = -x - y(yidx) where y is of length 9
 * and z, x and yidx are of length 9.
 */
void rally_car_QP_solver_LA_VSUB2_INDEXED_9(rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *y, int* yidx, rally_car_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<9; i++){
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
 * rally_car_QP_solver_NOPROGRESS (should be negative).
 */
int rally_car_QP_solver_LINESEARCH_BACKTRACKING_AFFINE(rally_car_QP_solver_FLOAT *l, rally_car_QP_solver_FLOAT *s, rally_car_QP_solver_FLOAT *dl, rally_car_QP_solver_FLOAT *ds, rally_car_QP_solver_FLOAT *a, rally_car_QP_solver_FLOAT *mu_aff)
{
    int i;
	int lsIt=1;    
    rally_car_QP_solver_FLOAT dltemp;
    rally_car_QP_solver_FLOAT dstemp;
    rally_car_QP_solver_FLOAT mya = 1.0;
    rally_car_QP_solver_FLOAT mymu;
        
    while( 1 ){                        

        /* 
         * Compute both snew and wnew together.
         * We compute also mu_affine along the way here, as the
         * values might be in registers, so it should be cheaper.
         */
        mymu = 0;
        for( i=0; i<458; i++ ){
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
        if( i == 458 ){
            break;
        } else {
            mya *= rally_car_QP_solver_SET_LS_SCALE_AFF;
            if( mya < rally_car_QP_solver_SET_LS_MINSTEP ){
                return rally_car_QP_solver_NOPROGRESS;
            }
        }
    }
    
    /* return new values and iteration counter */
    *a = mya;
    *mu_aff = mymu / (rally_car_QP_solver_FLOAT)458;
    return lsIt;
}


/*
 * Vector subtraction x = (u.*v - mu)*sigma where a is a scalar
*  and x,u,v are vectors of length 458.
 */
void rally_car_QP_solver_LA_VSUB5_458(rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *v, rally_car_QP_solver_FLOAT mu,  rally_car_QP_solver_FLOAT sigma, rally_car_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<458; i++){
		x[i] = u[i]*v[i] - mu;
		x[i] *= sigma;
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 28,
 * u, su, uidx are of length 12 and v, sv, vidx are of length 28.
 */
void rally_car_QP_solver_LA_VSUB6_INDEXED_28_12_28(rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *su, int* uidx, rally_car_QP_solver_FLOAT *v, rally_car_QP_solver_FLOAT *sv, int* vidx, rally_car_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<28; i++ ){
		x[i] = 0;
	}
	for( i=0; i<12; i++){
		x[uidx[i]] += u[i]/su[i];
	}
	for( i=0; i<28; i++){
		x[vidx[i]] -= v[i]/sv[i];
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void rally_car_QP_solver_LA_DENSE_2MVMADD_17_28_28(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<17; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<28; j++ ){		
		for( i=0; i<17; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<28; n++ ){
		for( i=0; i<17; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/* 
 * Computes r = A*x + B*u
 * where A is stored in column major format
 * and B is stored in diagzero format
 */
void rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_9_28_28(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<9; i++ ){
		r[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<28; j++ ){		
		for( i=0; i<9; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 9,
 * u, su, uidx are of length 9 and v, sv, vidx are of length 9.
 */
void rally_car_QP_solver_LA_VSUB6_INDEXED_9_9_9(rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *su, int* uidx, rally_car_QP_solver_FLOAT *v, rally_car_QP_solver_FLOAT *sv, int* vidx, rally_car_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<9; i++ ){
		x[i] = 0;
	}
	for( i=0; i<9; i++){
		x[uidx[i]] += u[i]/su[i];
	}
	for( i=0; i<9; i++){
		x[vidx[i]] -= v[i]/sv[i];
	}
}


/* 
 * Computes r = A*x + B*u
 * where A is stored in column major format
 * and B is stored in diagzero format
 */
void rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_9_28_9(rally_car_QP_solver_FLOAT *A, rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *B, rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<9; i++ ){
		r[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<28; j++ ){		
		for( i=0; i<9; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
}


/*
 * Vector subtraction z = x - y for vectors of length 317.
 */
void rally_car_QP_solver_LA_VSUB_317(rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *y, rally_car_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<317; i++){
		z[i] = x[i] - y[i];
	}
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 28 (length of y >= 28).
 */
void rally_car_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_28(rally_car_QP_solver_FLOAT *r, rally_car_QP_solver_FLOAT *s, rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *y, int* yidx, rally_car_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<28; i++ ){
		z[i] = -r[i]/s[i] - u[i]*y[yidx[i]];
	}
}


/** 
 * Computes z = -r./s + u.*y(y)
 * where all vectors except of y are of length 12 (length of y >= 12).
 */
void rally_car_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_12(rally_car_QP_solver_FLOAT *r, rally_car_QP_solver_FLOAT *s, rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *y, int* yidx, rally_car_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<12; i++ ){
		z[i] = -r[i]/s[i] + u[i]*y[yidx[i]];
	}
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 9 (length of y >= 9).
 */
void rally_car_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_9(rally_car_QP_solver_FLOAT *r, rally_car_QP_solver_FLOAT *s, rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *y, int* yidx, rally_car_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<9; i++ ){
		z[i] = -r[i]/s[i] - u[i]*y[yidx[i]];
	}
}


/** 
 * Computes z = -r./s + u.*y(y)
 * where all vectors except of y are of length 9 (length of y >= 9).
 */
void rally_car_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_9(rally_car_QP_solver_FLOAT *r, rally_car_QP_solver_FLOAT *s, rally_car_QP_solver_FLOAT *u, rally_car_QP_solver_FLOAT *y, int* yidx, rally_car_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<9; i++ ){
		z[i] = -r[i]/s[i] + u[i]*y[yidx[i]];
	}
}


/*
 * Computes ds = -l.\(r + s.*dl) for vectors of length 458.
 */
void rally_car_QP_solver_LA_VSUB7_458(rally_car_QP_solver_FLOAT *l, rally_car_QP_solver_FLOAT *r, rally_car_QP_solver_FLOAT *s, rally_car_QP_solver_FLOAT *dl, rally_car_QP_solver_FLOAT *ds)
{
	int i;
	for( i=0; i<458; i++){
		ds[i] = -(r[i] + s[i]*dl[i])/l[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 317.
 */
void rally_car_QP_solver_LA_VADD_317(rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<317; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 107.
 */
void rally_car_QP_solver_LA_VADD_107(rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<107; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 458.
 */
void rally_car_QP_solver_LA_VADD_458(rally_car_QP_solver_FLOAT *x, rally_car_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<458; i++){
		x[i] += y[i];
	}
}


/**
 * Backtracking line search for combined predictor/corrector step.
 * Update on variables with safety factor gamma (to keep us away from
 * boundary).
 */
int rally_car_QP_solver_LINESEARCH_BACKTRACKING_COMBINED(rally_car_QP_solver_FLOAT *z, rally_car_QP_solver_FLOAT *v, rally_car_QP_solver_FLOAT *l, rally_car_QP_solver_FLOAT *s, rally_car_QP_solver_FLOAT *dz, rally_car_QP_solver_FLOAT *dv, rally_car_QP_solver_FLOAT *dl, rally_car_QP_solver_FLOAT *ds, rally_car_QP_solver_FLOAT *a, rally_car_QP_solver_FLOAT *mu)
{
    int i, lsIt=1;       
    rally_car_QP_solver_FLOAT dltemp;
    rally_car_QP_solver_FLOAT dstemp;    
    rally_car_QP_solver_FLOAT a_gamma;
            
    *a = 1.0;
    while( 1 ){                        

        /* check whether search criterion is fulfilled */
        for( i=0; i<458; i++ ){
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
        if( i == 458 ){
            break;
        } else {
            *a *= rally_car_QP_solver_SET_LS_SCALE;
            if( *a < rally_car_QP_solver_SET_LS_MINSTEP ){
                return rally_car_QP_solver_NOPROGRESS;
            }
        }
    }
    
    /* update variables with safety margin */
    a_gamma = (*a)*rally_car_QP_solver_SET_LS_MAXSTEP;
    
    /* primal variables */
    for( i=0; i<317; i++ ){
        z[i] += a_gamma*dz[i];
    }
    
    /* equality constraint multipliers */
    for( i=0; i<107; i++ ){
        v[i] += a_gamma*dv[i];
    }
    
    /* inequality constraint multipliers & slacks, also update mu */
    *mu = 0;
    for( i=0; i<458; i++ ){
        dltemp = l[i] + a_gamma*dl[i]; l[i] = dltemp;
        dstemp = s[i] + a_gamma*ds[i]; s[i] = dstemp;
        *mu += dltemp*dstemp;
    }
    
    *a = a_gamma;
    *mu /= (rally_car_QP_solver_FLOAT)458;
    return lsIt;
}




/* VARIABLE DEFINITIONS ------------------------------------------------ */
rally_car_QP_solver_FLOAT rally_car_QP_solver_z[317];
rally_car_QP_solver_FLOAT rally_car_QP_solver_v[107];
rally_car_QP_solver_FLOAT rally_car_QP_solver_dz_aff[317];
rally_car_QP_solver_FLOAT rally_car_QP_solver_dv_aff[107];
rally_car_QP_solver_FLOAT rally_car_QP_solver_grad_cost[317];
rally_car_QP_solver_FLOAT rally_car_QP_solver_grad_eq[317];
rally_car_QP_solver_FLOAT rally_car_QP_solver_rd[317];
rally_car_QP_solver_FLOAT rally_car_QP_solver_l[458];
rally_car_QP_solver_FLOAT rally_car_QP_solver_s[458];
rally_car_QP_solver_FLOAT rally_car_QP_solver_lbys[458];
rally_car_QP_solver_FLOAT rally_car_QP_solver_dl_aff[458];
rally_car_QP_solver_FLOAT rally_car_QP_solver_ds_aff[458];
rally_car_QP_solver_FLOAT rally_car_QP_solver_dz_cc[317];
rally_car_QP_solver_FLOAT rally_car_QP_solver_dv_cc[107];
rally_car_QP_solver_FLOAT rally_car_QP_solver_dl_cc[458];
rally_car_QP_solver_FLOAT rally_car_QP_solver_ds_cc[458];
rally_car_QP_solver_FLOAT rally_car_QP_solver_ccrhs[458];
rally_car_QP_solver_FLOAT rally_car_QP_solver_grad_ineq[317];
rally_car_QP_solver_FLOAT rally_car_QP_solver_H00[28] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_z00 = rally_car_QP_solver_z + 0;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzaff00 = rally_car_QP_solver_dz_aff + 0;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzcc00 = rally_car_QP_solver_dz_cc + 0;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_rd00 = rally_car_QP_solver_rd + 0;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lbyrd00[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_cost00 = rally_car_QP_solver_grad_cost + 0;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_eq00 = rally_car_QP_solver_grad_eq + 0;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_ineq00 = rally_car_QP_solver_grad_ineq + 0;
rally_car_QP_solver_FLOAT rally_car_QP_solver_ctv00[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_v00 = rally_car_QP_solver_v + 0;
rally_car_QP_solver_FLOAT rally_car_QP_solver_re00[17];
rally_car_QP_solver_FLOAT rally_car_QP_solver_beta00[17];
rally_car_QP_solver_FLOAT rally_car_QP_solver_betacc00[17];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvaff00 = rally_car_QP_solver_dv_aff + 0;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvcc00 = rally_car_QP_solver_dv_cc + 0;
rally_car_QP_solver_FLOAT rally_car_QP_solver_V00[476];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Yd00[153];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ld00[153];
rally_car_QP_solver_FLOAT rally_car_QP_solver_yy00[17];
rally_car_QP_solver_FLOAT rally_car_QP_solver_bmy00[17];
int rally_car_QP_solver_lbIdx00[28] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llb00 = rally_car_QP_solver_l + 0;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_slb00 = rally_car_QP_solver_s + 0;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llbbyslb00 = rally_car_QP_solver_lbys + 0;
rally_car_QP_solver_FLOAT rally_car_QP_solver_rilb00[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbaff00 = rally_car_QP_solver_dl_aff + 0;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbaff00 = rally_car_QP_solver_ds_aff + 0;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbcc00 = rally_car_QP_solver_dl_cc + 0;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbcc00 = rally_car_QP_solver_ds_cc + 0;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsl00 = rally_car_QP_solver_ccrhs + 0;
int rally_car_QP_solver_ubIdx00[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lub00 = rally_car_QP_solver_l + 28;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_sub00 = rally_car_QP_solver_s + 28;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lubbysub00 = rally_car_QP_solver_lbys + 28;
rally_car_QP_solver_FLOAT rally_car_QP_solver_riub00[12];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubaff00 = rally_car_QP_solver_dl_aff + 28;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubaff00 = rally_car_QP_solver_ds_aff + 28;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubcc00 = rally_car_QP_solver_dl_cc + 28;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubcc00 = rally_car_QP_solver_ds_cc + 28;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsub00 = rally_car_QP_solver_ccrhs + 28;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Phi00[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_z01 = rally_car_QP_solver_z + 28;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzaff01 = rally_car_QP_solver_dz_aff + 28;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzcc01 = rally_car_QP_solver_dz_cc + 28;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_rd01 = rally_car_QP_solver_rd + 28;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lbyrd01[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_cost01 = rally_car_QP_solver_grad_cost + 28;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_eq01 = rally_car_QP_solver_grad_eq + 28;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_ineq01 = rally_car_QP_solver_grad_ineq + 28;
rally_car_QP_solver_FLOAT rally_car_QP_solver_ctv01[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_v01 = rally_car_QP_solver_v + 17;
rally_car_QP_solver_FLOAT rally_car_QP_solver_re01[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_beta01[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_betacc01[9];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvaff01 = rally_car_QP_solver_dv_aff + 17;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvcc01 = rally_car_QP_solver_dv_cc + 17;
rally_car_QP_solver_FLOAT rally_car_QP_solver_V01[252];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Yd01[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ld01[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_yy01[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_bmy01[9];
int rally_car_QP_solver_lbIdx01[28] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llb01 = rally_car_QP_solver_l + 40;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_slb01 = rally_car_QP_solver_s + 40;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llbbyslb01 = rally_car_QP_solver_lbys + 40;
rally_car_QP_solver_FLOAT rally_car_QP_solver_rilb01[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbaff01 = rally_car_QP_solver_dl_aff + 40;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbaff01 = rally_car_QP_solver_ds_aff + 40;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbcc01 = rally_car_QP_solver_dl_cc + 40;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbcc01 = rally_car_QP_solver_ds_cc + 40;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsl01 = rally_car_QP_solver_ccrhs + 40;
int rally_car_QP_solver_ubIdx01[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lub01 = rally_car_QP_solver_l + 68;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_sub01 = rally_car_QP_solver_s + 68;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lubbysub01 = rally_car_QP_solver_lbys + 68;
rally_car_QP_solver_FLOAT rally_car_QP_solver_riub01[12];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubaff01 = rally_car_QP_solver_dl_aff + 68;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubaff01 = rally_car_QP_solver_ds_aff + 68;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubcc01 = rally_car_QP_solver_dl_cc + 68;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubcc01 = rally_car_QP_solver_ds_cc + 68;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsub01 = rally_car_QP_solver_ccrhs + 68;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Phi01[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_D01[476] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
rally_car_QP_solver_FLOAT rally_car_QP_solver_W01[476];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ysd01[153];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lsd01[153];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_z02 = rally_car_QP_solver_z + 56;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzaff02 = rally_car_QP_solver_dz_aff + 56;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzcc02 = rally_car_QP_solver_dz_cc + 56;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_rd02 = rally_car_QP_solver_rd + 56;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lbyrd02[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_cost02 = rally_car_QP_solver_grad_cost + 56;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_eq02 = rally_car_QP_solver_grad_eq + 56;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_ineq02 = rally_car_QP_solver_grad_ineq + 56;
rally_car_QP_solver_FLOAT rally_car_QP_solver_ctv02[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_v02 = rally_car_QP_solver_v + 26;
rally_car_QP_solver_FLOAT rally_car_QP_solver_re02[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_beta02[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_betacc02[9];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvaff02 = rally_car_QP_solver_dv_aff + 26;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvcc02 = rally_car_QP_solver_dv_cc + 26;
rally_car_QP_solver_FLOAT rally_car_QP_solver_V02[252];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Yd02[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ld02[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_yy02[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_bmy02[9];
int rally_car_QP_solver_lbIdx02[28] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llb02 = rally_car_QP_solver_l + 80;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_slb02 = rally_car_QP_solver_s + 80;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llbbyslb02 = rally_car_QP_solver_lbys + 80;
rally_car_QP_solver_FLOAT rally_car_QP_solver_rilb02[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbaff02 = rally_car_QP_solver_dl_aff + 80;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbaff02 = rally_car_QP_solver_ds_aff + 80;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbcc02 = rally_car_QP_solver_dl_cc + 80;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbcc02 = rally_car_QP_solver_ds_cc + 80;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsl02 = rally_car_QP_solver_ccrhs + 80;
int rally_car_QP_solver_ubIdx02[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lub02 = rally_car_QP_solver_l + 108;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_sub02 = rally_car_QP_solver_s + 108;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lubbysub02 = rally_car_QP_solver_lbys + 108;
rally_car_QP_solver_FLOAT rally_car_QP_solver_riub02[12];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubaff02 = rally_car_QP_solver_dl_aff + 108;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubaff02 = rally_car_QP_solver_ds_aff + 108;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubcc02 = rally_car_QP_solver_dl_cc + 108;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubcc02 = rally_car_QP_solver_ds_cc + 108;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsub02 = rally_car_QP_solver_ccrhs + 108;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Phi02[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_D02[28] = {-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000};
rally_car_QP_solver_FLOAT rally_car_QP_solver_W02[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ysd02[81];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lsd02[81];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_z03 = rally_car_QP_solver_z + 84;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzaff03 = rally_car_QP_solver_dz_aff + 84;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzcc03 = rally_car_QP_solver_dz_cc + 84;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_rd03 = rally_car_QP_solver_rd + 84;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lbyrd03[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_cost03 = rally_car_QP_solver_grad_cost + 84;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_eq03 = rally_car_QP_solver_grad_eq + 84;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_ineq03 = rally_car_QP_solver_grad_ineq + 84;
rally_car_QP_solver_FLOAT rally_car_QP_solver_ctv03[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_v03 = rally_car_QP_solver_v + 35;
rally_car_QP_solver_FLOAT rally_car_QP_solver_re03[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_beta03[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_betacc03[9];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvaff03 = rally_car_QP_solver_dv_aff + 35;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvcc03 = rally_car_QP_solver_dv_cc + 35;
rally_car_QP_solver_FLOAT rally_car_QP_solver_V03[252];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Yd03[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ld03[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_yy03[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_bmy03[9];
int rally_car_QP_solver_lbIdx03[28] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llb03 = rally_car_QP_solver_l + 120;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_slb03 = rally_car_QP_solver_s + 120;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llbbyslb03 = rally_car_QP_solver_lbys + 120;
rally_car_QP_solver_FLOAT rally_car_QP_solver_rilb03[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbaff03 = rally_car_QP_solver_dl_aff + 120;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbaff03 = rally_car_QP_solver_ds_aff + 120;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbcc03 = rally_car_QP_solver_dl_cc + 120;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbcc03 = rally_car_QP_solver_ds_cc + 120;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsl03 = rally_car_QP_solver_ccrhs + 120;
int rally_car_QP_solver_ubIdx03[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lub03 = rally_car_QP_solver_l + 148;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_sub03 = rally_car_QP_solver_s + 148;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lubbysub03 = rally_car_QP_solver_lbys + 148;
rally_car_QP_solver_FLOAT rally_car_QP_solver_riub03[12];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubaff03 = rally_car_QP_solver_dl_aff + 148;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubaff03 = rally_car_QP_solver_ds_aff + 148;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubcc03 = rally_car_QP_solver_dl_cc + 148;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubcc03 = rally_car_QP_solver_ds_cc + 148;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsub03 = rally_car_QP_solver_ccrhs + 148;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Phi03[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_W03[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ysd03[81];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lsd03[81];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_z04 = rally_car_QP_solver_z + 112;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzaff04 = rally_car_QP_solver_dz_aff + 112;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzcc04 = rally_car_QP_solver_dz_cc + 112;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_rd04 = rally_car_QP_solver_rd + 112;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lbyrd04[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_cost04 = rally_car_QP_solver_grad_cost + 112;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_eq04 = rally_car_QP_solver_grad_eq + 112;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_ineq04 = rally_car_QP_solver_grad_ineq + 112;
rally_car_QP_solver_FLOAT rally_car_QP_solver_ctv04[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_v04 = rally_car_QP_solver_v + 44;
rally_car_QP_solver_FLOAT rally_car_QP_solver_re04[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_beta04[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_betacc04[9];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvaff04 = rally_car_QP_solver_dv_aff + 44;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvcc04 = rally_car_QP_solver_dv_cc + 44;
rally_car_QP_solver_FLOAT rally_car_QP_solver_V04[252];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Yd04[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ld04[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_yy04[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_bmy04[9];
int rally_car_QP_solver_lbIdx04[28] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llb04 = rally_car_QP_solver_l + 160;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_slb04 = rally_car_QP_solver_s + 160;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llbbyslb04 = rally_car_QP_solver_lbys + 160;
rally_car_QP_solver_FLOAT rally_car_QP_solver_rilb04[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbaff04 = rally_car_QP_solver_dl_aff + 160;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbaff04 = rally_car_QP_solver_ds_aff + 160;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbcc04 = rally_car_QP_solver_dl_cc + 160;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbcc04 = rally_car_QP_solver_ds_cc + 160;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsl04 = rally_car_QP_solver_ccrhs + 160;
int rally_car_QP_solver_ubIdx04[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lub04 = rally_car_QP_solver_l + 188;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_sub04 = rally_car_QP_solver_s + 188;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lubbysub04 = rally_car_QP_solver_lbys + 188;
rally_car_QP_solver_FLOAT rally_car_QP_solver_riub04[12];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubaff04 = rally_car_QP_solver_dl_aff + 188;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubaff04 = rally_car_QP_solver_ds_aff + 188;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubcc04 = rally_car_QP_solver_dl_cc + 188;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubcc04 = rally_car_QP_solver_ds_cc + 188;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsub04 = rally_car_QP_solver_ccrhs + 188;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Phi04[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_W04[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ysd04[81];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lsd04[81];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_z05 = rally_car_QP_solver_z + 140;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzaff05 = rally_car_QP_solver_dz_aff + 140;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzcc05 = rally_car_QP_solver_dz_cc + 140;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_rd05 = rally_car_QP_solver_rd + 140;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lbyrd05[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_cost05 = rally_car_QP_solver_grad_cost + 140;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_eq05 = rally_car_QP_solver_grad_eq + 140;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_ineq05 = rally_car_QP_solver_grad_ineq + 140;
rally_car_QP_solver_FLOAT rally_car_QP_solver_ctv05[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_v05 = rally_car_QP_solver_v + 53;
rally_car_QP_solver_FLOAT rally_car_QP_solver_re05[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_beta05[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_betacc05[9];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvaff05 = rally_car_QP_solver_dv_aff + 53;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvcc05 = rally_car_QP_solver_dv_cc + 53;
rally_car_QP_solver_FLOAT rally_car_QP_solver_V05[252];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Yd05[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ld05[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_yy05[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_bmy05[9];
int rally_car_QP_solver_lbIdx05[28] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llb05 = rally_car_QP_solver_l + 200;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_slb05 = rally_car_QP_solver_s + 200;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llbbyslb05 = rally_car_QP_solver_lbys + 200;
rally_car_QP_solver_FLOAT rally_car_QP_solver_rilb05[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbaff05 = rally_car_QP_solver_dl_aff + 200;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbaff05 = rally_car_QP_solver_ds_aff + 200;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbcc05 = rally_car_QP_solver_dl_cc + 200;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbcc05 = rally_car_QP_solver_ds_cc + 200;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsl05 = rally_car_QP_solver_ccrhs + 200;
int rally_car_QP_solver_ubIdx05[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lub05 = rally_car_QP_solver_l + 228;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_sub05 = rally_car_QP_solver_s + 228;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lubbysub05 = rally_car_QP_solver_lbys + 228;
rally_car_QP_solver_FLOAT rally_car_QP_solver_riub05[12];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubaff05 = rally_car_QP_solver_dl_aff + 228;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubaff05 = rally_car_QP_solver_ds_aff + 228;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubcc05 = rally_car_QP_solver_dl_cc + 228;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubcc05 = rally_car_QP_solver_ds_cc + 228;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsub05 = rally_car_QP_solver_ccrhs + 228;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Phi05[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_W05[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ysd05[81];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lsd05[81];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_z06 = rally_car_QP_solver_z + 168;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzaff06 = rally_car_QP_solver_dz_aff + 168;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzcc06 = rally_car_QP_solver_dz_cc + 168;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_rd06 = rally_car_QP_solver_rd + 168;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lbyrd06[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_cost06 = rally_car_QP_solver_grad_cost + 168;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_eq06 = rally_car_QP_solver_grad_eq + 168;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_ineq06 = rally_car_QP_solver_grad_ineq + 168;
rally_car_QP_solver_FLOAT rally_car_QP_solver_ctv06[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_v06 = rally_car_QP_solver_v + 62;
rally_car_QP_solver_FLOAT rally_car_QP_solver_re06[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_beta06[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_betacc06[9];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvaff06 = rally_car_QP_solver_dv_aff + 62;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvcc06 = rally_car_QP_solver_dv_cc + 62;
rally_car_QP_solver_FLOAT rally_car_QP_solver_V06[252];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Yd06[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ld06[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_yy06[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_bmy06[9];
int rally_car_QP_solver_lbIdx06[28] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llb06 = rally_car_QP_solver_l + 240;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_slb06 = rally_car_QP_solver_s + 240;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llbbyslb06 = rally_car_QP_solver_lbys + 240;
rally_car_QP_solver_FLOAT rally_car_QP_solver_rilb06[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbaff06 = rally_car_QP_solver_dl_aff + 240;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbaff06 = rally_car_QP_solver_ds_aff + 240;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbcc06 = rally_car_QP_solver_dl_cc + 240;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbcc06 = rally_car_QP_solver_ds_cc + 240;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsl06 = rally_car_QP_solver_ccrhs + 240;
int rally_car_QP_solver_ubIdx06[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lub06 = rally_car_QP_solver_l + 268;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_sub06 = rally_car_QP_solver_s + 268;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lubbysub06 = rally_car_QP_solver_lbys + 268;
rally_car_QP_solver_FLOAT rally_car_QP_solver_riub06[12];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubaff06 = rally_car_QP_solver_dl_aff + 268;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubaff06 = rally_car_QP_solver_ds_aff + 268;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubcc06 = rally_car_QP_solver_dl_cc + 268;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubcc06 = rally_car_QP_solver_ds_cc + 268;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsub06 = rally_car_QP_solver_ccrhs + 268;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Phi06[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_W06[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ysd06[81];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lsd06[81];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_z07 = rally_car_QP_solver_z + 196;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzaff07 = rally_car_QP_solver_dz_aff + 196;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzcc07 = rally_car_QP_solver_dz_cc + 196;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_rd07 = rally_car_QP_solver_rd + 196;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lbyrd07[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_cost07 = rally_car_QP_solver_grad_cost + 196;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_eq07 = rally_car_QP_solver_grad_eq + 196;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_ineq07 = rally_car_QP_solver_grad_ineq + 196;
rally_car_QP_solver_FLOAT rally_car_QP_solver_ctv07[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_v07 = rally_car_QP_solver_v + 71;
rally_car_QP_solver_FLOAT rally_car_QP_solver_re07[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_beta07[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_betacc07[9];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvaff07 = rally_car_QP_solver_dv_aff + 71;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvcc07 = rally_car_QP_solver_dv_cc + 71;
rally_car_QP_solver_FLOAT rally_car_QP_solver_V07[252];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Yd07[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ld07[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_yy07[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_bmy07[9];
int rally_car_QP_solver_lbIdx07[28] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llb07 = rally_car_QP_solver_l + 280;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_slb07 = rally_car_QP_solver_s + 280;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llbbyslb07 = rally_car_QP_solver_lbys + 280;
rally_car_QP_solver_FLOAT rally_car_QP_solver_rilb07[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbaff07 = rally_car_QP_solver_dl_aff + 280;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbaff07 = rally_car_QP_solver_ds_aff + 280;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbcc07 = rally_car_QP_solver_dl_cc + 280;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbcc07 = rally_car_QP_solver_ds_cc + 280;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsl07 = rally_car_QP_solver_ccrhs + 280;
int rally_car_QP_solver_ubIdx07[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lub07 = rally_car_QP_solver_l + 308;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_sub07 = rally_car_QP_solver_s + 308;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lubbysub07 = rally_car_QP_solver_lbys + 308;
rally_car_QP_solver_FLOAT rally_car_QP_solver_riub07[12];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubaff07 = rally_car_QP_solver_dl_aff + 308;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubaff07 = rally_car_QP_solver_ds_aff + 308;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubcc07 = rally_car_QP_solver_dl_cc + 308;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubcc07 = rally_car_QP_solver_ds_cc + 308;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsub07 = rally_car_QP_solver_ccrhs + 308;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Phi07[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_W07[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ysd07[81];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lsd07[81];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_z08 = rally_car_QP_solver_z + 224;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzaff08 = rally_car_QP_solver_dz_aff + 224;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzcc08 = rally_car_QP_solver_dz_cc + 224;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_rd08 = rally_car_QP_solver_rd + 224;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lbyrd08[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_cost08 = rally_car_QP_solver_grad_cost + 224;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_eq08 = rally_car_QP_solver_grad_eq + 224;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_ineq08 = rally_car_QP_solver_grad_ineq + 224;
rally_car_QP_solver_FLOAT rally_car_QP_solver_ctv08[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_v08 = rally_car_QP_solver_v + 80;
rally_car_QP_solver_FLOAT rally_car_QP_solver_re08[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_beta08[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_betacc08[9];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvaff08 = rally_car_QP_solver_dv_aff + 80;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvcc08 = rally_car_QP_solver_dv_cc + 80;
rally_car_QP_solver_FLOAT rally_car_QP_solver_V08[252];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Yd08[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ld08[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_yy08[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_bmy08[9];
int rally_car_QP_solver_lbIdx08[28] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llb08 = rally_car_QP_solver_l + 320;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_slb08 = rally_car_QP_solver_s + 320;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llbbyslb08 = rally_car_QP_solver_lbys + 320;
rally_car_QP_solver_FLOAT rally_car_QP_solver_rilb08[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbaff08 = rally_car_QP_solver_dl_aff + 320;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbaff08 = rally_car_QP_solver_ds_aff + 320;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbcc08 = rally_car_QP_solver_dl_cc + 320;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbcc08 = rally_car_QP_solver_ds_cc + 320;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsl08 = rally_car_QP_solver_ccrhs + 320;
int rally_car_QP_solver_ubIdx08[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lub08 = rally_car_QP_solver_l + 348;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_sub08 = rally_car_QP_solver_s + 348;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lubbysub08 = rally_car_QP_solver_lbys + 348;
rally_car_QP_solver_FLOAT rally_car_QP_solver_riub08[12];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubaff08 = rally_car_QP_solver_dl_aff + 348;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubaff08 = rally_car_QP_solver_ds_aff + 348;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubcc08 = rally_car_QP_solver_dl_cc + 348;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubcc08 = rally_car_QP_solver_ds_cc + 348;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsub08 = rally_car_QP_solver_ccrhs + 348;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Phi08[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_W08[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ysd08[81];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lsd08[81];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_z09 = rally_car_QP_solver_z + 252;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzaff09 = rally_car_QP_solver_dz_aff + 252;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzcc09 = rally_car_QP_solver_dz_cc + 252;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_rd09 = rally_car_QP_solver_rd + 252;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lbyrd09[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_cost09 = rally_car_QP_solver_grad_cost + 252;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_eq09 = rally_car_QP_solver_grad_eq + 252;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_ineq09 = rally_car_QP_solver_grad_ineq + 252;
rally_car_QP_solver_FLOAT rally_car_QP_solver_ctv09[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_v09 = rally_car_QP_solver_v + 89;
rally_car_QP_solver_FLOAT rally_car_QP_solver_re09[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_beta09[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_betacc09[9];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvaff09 = rally_car_QP_solver_dv_aff + 89;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvcc09 = rally_car_QP_solver_dv_cc + 89;
rally_car_QP_solver_FLOAT rally_car_QP_solver_V09[252];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Yd09[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ld09[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_yy09[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_bmy09[9];
int rally_car_QP_solver_lbIdx09[28] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llb09 = rally_car_QP_solver_l + 360;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_slb09 = rally_car_QP_solver_s + 360;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llbbyslb09 = rally_car_QP_solver_lbys + 360;
rally_car_QP_solver_FLOAT rally_car_QP_solver_rilb09[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbaff09 = rally_car_QP_solver_dl_aff + 360;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbaff09 = rally_car_QP_solver_ds_aff + 360;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbcc09 = rally_car_QP_solver_dl_cc + 360;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbcc09 = rally_car_QP_solver_ds_cc + 360;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsl09 = rally_car_QP_solver_ccrhs + 360;
int rally_car_QP_solver_ubIdx09[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lub09 = rally_car_QP_solver_l + 388;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_sub09 = rally_car_QP_solver_s + 388;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lubbysub09 = rally_car_QP_solver_lbys + 388;
rally_car_QP_solver_FLOAT rally_car_QP_solver_riub09[12];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubaff09 = rally_car_QP_solver_dl_aff + 388;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubaff09 = rally_car_QP_solver_ds_aff + 388;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubcc09 = rally_car_QP_solver_dl_cc + 388;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubcc09 = rally_car_QP_solver_ds_cc + 388;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsub09 = rally_car_QP_solver_ccrhs + 388;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Phi09[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_W09[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ysd09[81];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lsd09[81];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_z10 = rally_car_QP_solver_z + 280;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzaff10 = rally_car_QP_solver_dz_aff + 280;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzcc10 = rally_car_QP_solver_dz_cc + 280;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_rd10 = rally_car_QP_solver_rd + 280;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lbyrd10[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_cost10 = rally_car_QP_solver_grad_cost + 280;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_eq10 = rally_car_QP_solver_grad_eq + 280;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_ineq10 = rally_car_QP_solver_grad_ineq + 280;
rally_car_QP_solver_FLOAT rally_car_QP_solver_ctv10[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_v10 = rally_car_QP_solver_v + 98;
rally_car_QP_solver_FLOAT rally_car_QP_solver_re10[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_beta10[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_betacc10[9];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvaff10 = rally_car_QP_solver_dv_aff + 98;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dvcc10 = rally_car_QP_solver_dv_cc + 98;
rally_car_QP_solver_FLOAT rally_car_QP_solver_V10[252];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Yd10[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ld10[45];
rally_car_QP_solver_FLOAT rally_car_QP_solver_yy10[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_bmy10[9];
int rally_car_QP_solver_lbIdx10[28] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llb10 = rally_car_QP_solver_l + 400;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_slb10 = rally_car_QP_solver_s + 400;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llbbyslb10 = rally_car_QP_solver_lbys + 400;
rally_car_QP_solver_FLOAT rally_car_QP_solver_rilb10[28];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbaff10 = rally_car_QP_solver_dl_aff + 400;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbaff10 = rally_car_QP_solver_ds_aff + 400;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbcc10 = rally_car_QP_solver_dl_cc + 400;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbcc10 = rally_car_QP_solver_ds_cc + 400;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsl10 = rally_car_QP_solver_ccrhs + 400;
int rally_car_QP_solver_ubIdx10[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lub10 = rally_car_QP_solver_l + 428;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_sub10 = rally_car_QP_solver_s + 428;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lubbysub10 = rally_car_QP_solver_lbys + 428;
rally_car_QP_solver_FLOAT rally_car_QP_solver_riub10[12];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubaff10 = rally_car_QP_solver_dl_aff + 428;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubaff10 = rally_car_QP_solver_ds_aff + 428;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubcc10 = rally_car_QP_solver_dl_cc + 428;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubcc10 = rally_car_QP_solver_ds_cc + 428;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsub10 = rally_car_QP_solver_ccrhs + 428;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Phi10[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_W10[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Ysd10[81];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lsd10[81];
rally_car_QP_solver_FLOAT rally_car_QP_solver_H11[9] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
rally_car_QP_solver_FLOAT rally_car_QP_solver_f11[9] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_z11 = rally_car_QP_solver_z + 308;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzaff11 = rally_car_QP_solver_dz_aff + 308;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dzcc11 = rally_car_QP_solver_dz_cc + 308;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_rd11 = rally_car_QP_solver_rd + 308;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Lbyrd11[9];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_cost11 = rally_car_QP_solver_grad_cost + 308;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_eq11 = rally_car_QP_solver_grad_eq + 308;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_grad_ineq11 = rally_car_QP_solver_grad_ineq + 308;
rally_car_QP_solver_FLOAT rally_car_QP_solver_ctv11[9];
int rally_car_QP_solver_lbIdx11[9] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llb11 = rally_car_QP_solver_l + 440;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_slb11 = rally_car_QP_solver_s + 440;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_llbbyslb11 = rally_car_QP_solver_lbys + 440;
rally_car_QP_solver_FLOAT rally_car_QP_solver_rilb11[9];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbaff11 = rally_car_QP_solver_dl_aff + 440;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbaff11 = rally_car_QP_solver_ds_aff + 440;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dllbcc11 = rally_car_QP_solver_dl_cc + 440;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dslbcc11 = rally_car_QP_solver_ds_cc + 440;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsl11 = rally_car_QP_solver_ccrhs + 440;
int rally_car_QP_solver_ubIdx11[9] = {0, 1, 2, 3, 4, 5, 6, 7, 8};
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lub11 = rally_car_QP_solver_l + 449;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_sub11 = rally_car_QP_solver_s + 449;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_lubbysub11 = rally_car_QP_solver_lbys + 449;
rally_car_QP_solver_FLOAT rally_car_QP_solver_riub11[9];
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubaff11 = rally_car_QP_solver_dl_aff + 449;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubaff11 = rally_car_QP_solver_ds_aff + 449;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dlubcc11 = rally_car_QP_solver_dl_cc + 449;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_dsubcc11 = rally_car_QP_solver_ds_cc + 449;
rally_car_QP_solver_FLOAT* rally_car_QP_solver_ccrhsub11 = rally_car_QP_solver_ccrhs + 449;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Phi11[9];
rally_car_QP_solver_FLOAT rally_car_QP_solver_D11[9] = {-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000};
rally_car_QP_solver_FLOAT rally_car_QP_solver_W11[9];
rally_car_QP_solver_FLOAT musigma;
rally_car_QP_solver_FLOAT sigma_3rdroot;
rally_car_QP_solver_FLOAT rally_car_QP_solver_Diag1_0[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_Diag2_0[28];
rally_car_QP_solver_FLOAT rally_car_QP_solver_L_0[378];




/* SOLVER CODE --------------------------------------------------------- */
int rally_car_QP_solver_solve(rally_car_QP_solver_params* params, rally_car_QP_solver_output* output, rally_car_QP_solver_info* info)
{	
int exitcode;

#if rally_car_QP_solver_SET_TIMING == 1
	rally_car_QP_solver_timer solvertimer;
	rally_car_QP_solver_tic(&solvertimer);
#endif
/* FUNCTION CALLS INTO LA LIBRARY -------------------------------------- */
info->it = 0;
rally_car_QP_solver_LA_INITIALIZEVECTOR_317(rally_car_QP_solver_z, 0);
rally_car_QP_solver_LA_INITIALIZEVECTOR_107(rally_car_QP_solver_v, 1);
rally_car_QP_solver_LA_INITIALIZEVECTOR_458(rally_car_QP_solver_l, 10);
rally_car_QP_solver_LA_INITIALIZEVECTOR_458(rally_car_QP_solver_s, 10);
info->mu = 0;
rally_car_QP_solver_LA_DOTACC_458(rally_car_QP_solver_l, rally_car_QP_solver_s, &info->mu);
info->mu /= 458;
while( 1 ){
info->pobj = 0;
rally_car_QP_solver_LA_DIAG_QUADFCN_28(rally_car_QP_solver_H00, params->f1, rally_car_QP_solver_z00, rally_car_QP_solver_grad_cost00, &info->pobj);
rally_car_QP_solver_LA_DIAG_QUADFCN_28(rally_car_QP_solver_H00, params->f2, rally_car_QP_solver_z01, rally_car_QP_solver_grad_cost01, &info->pobj);
rally_car_QP_solver_LA_DIAG_QUADFCN_28(rally_car_QP_solver_H00, params->f3, rally_car_QP_solver_z02, rally_car_QP_solver_grad_cost02, &info->pobj);
rally_car_QP_solver_LA_DIAG_QUADFCN_28(rally_car_QP_solver_H00, params->f4, rally_car_QP_solver_z03, rally_car_QP_solver_grad_cost03, &info->pobj);
rally_car_QP_solver_LA_DIAG_QUADFCN_28(rally_car_QP_solver_H00, params->f5, rally_car_QP_solver_z04, rally_car_QP_solver_grad_cost04, &info->pobj);
rally_car_QP_solver_LA_DIAG_QUADFCN_28(rally_car_QP_solver_H00, params->f6, rally_car_QP_solver_z05, rally_car_QP_solver_grad_cost05, &info->pobj);
rally_car_QP_solver_LA_DIAG_QUADFCN_28(rally_car_QP_solver_H00, params->f7, rally_car_QP_solver_z06, rally_car_QP_solver_grad_cost06, &info->pobj);
rally_car_QP_solver_LA_DIAG_QUADFCN_28(rally_car_QP_solver_H00, params->f8, rally_car_QP_solver_z07, rally_car_QP_solver_grad_cost07, &info->pobj);
rally_car_QP_solver_LA_DIAG_QUADFCN_28(rally_car_QP_solver_H00, params->f9, rally_car_QP_solver_z08, rally_car_QP_solver_grad_cost08, &info->pobj);
rally_car_QP_solver_LA_DIAG_QUADFCN_28(rally_car_QP_solver_H00, params->f10, rally_car_QP_solver_z09, rally_car_QP_solver_grad_cost09, &info->pobj);
rally_car_QP_solver_LA_DIAG_QUADFCN_28(rally_car_QP_solver_H00, params->f11, rally_car_QP_solver_z10, rally_car_QP_solver_grad_cost10, &info->pobj);
rally_car_QP_solver_LA_DIAG_QUADFCN_9(rally_car_QP_solver_H11, rally_car_QP_solver_f11, rally_car_QP_solver_z11, rally_car_QP_solver_grad_cost11, &info->pobj);
info->res_eq = 0;
info->dgap = 0;
rally_car_QP_solver_LA_DENSE_MVMSUB3_17_28_28(params->C1, rally_car_QP_solver_z00, rally_car_QP_solver_D01, rally_car_QP_solver_z01, params->e1, rally_car_QP_solver_v00, rally_car_QP_solver_re00, &info->dgap, &info->res_eq);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_9_28_28(params->C2, rally_car_QP_solver_z01, rally_car_QP_solver_D02, rally_car_QP_solver_z02, params->e2, rally_car_QP_solver_v01, rally_car_QP_solver_re01, &info->dgap, &info->res_eq);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_9_28_28(params->C3, rally_car_QP_solver_z02, rally_car_QP_solver_D02, rally_car_QP_solver_z03, params->e3, rally_car_QP_solver_v02, rally_car_QP_solver_re02, &info->dgap, &info->res_eq);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_9_28_28(params->C4, rally_car_QP_solver_z03, rally_car_QP_solver_D02, rally_car_QP_solver_z04, params->e4, rally_car_QP_solver_v03, rally_car_QP_solver_re03, &info->dgap, &info->res_eq);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_9_28_28(params->C5, rally_car_QP_solver_z04, rally_car_QP_solver_D02, rally_car_QP_solver_z05, params->e5, rally_car_QP_solver_v04, rally_car_QP_solver_re04, &info->dgap, &info->res_eq);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_9_28_28(params->C6, rally_car_QP_solver_z05, rally_car_QP_solver_D02, rally_car_QP_solver_z06, params->e6, rally_car_QP_solver_v05, rally_car_QP_solver_re05, &info->dgap, &info->res_eq);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_9_28_28(params->C7, rally_car_QP_solver_z06, rally_car_QP_solver_D02, rally_car_QP_solver_z07, params->e7, rally_car_QP_solver_v06, rally_car_QP_solver_re06, &info->dgap, &info->res_eq);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_9_28_28(params->C8, rally_car_QP_solver_z07, rally_car_QP_solver_D02, rally_car_QP_solver_z08, params->e8, rally_car_QP_solver_v07, rally_car_QP_solver_re07, &info->dgap, &info->res_eq);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_9_28_28(params->C9, rally_car_QP_solver_z08, rally_car_QP_solver_D02, rally_car_QP_solver_z09, params->e9, rally_car_QP_solver_v08, rally_car_QP_solver_re08, &info->dgap, &info->res_eq);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_9_28_28(params->C10, rally_car_QP_solver_z09, rally_car_QP_solver_D02, rally_car_QP_solver_z10, params->e10, rally_car_QP_solver_v09, rally_car_QP_solver_re09, &info->dgap, &info->res_eq);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_9_28_9(params->C11, rally_car_QP_solver_z10, rally_car_QP_solver_D11, rally_car_QP_solver_z11, params->e11, rally_car_QP_solver_v10, rally_car_QP_solver_re10, &info->dgap, &info->res_eq);
rally_car_QP_solver_LA_DENSE_MTVM_17_28(params->C1, rally_car_QP_solver_v00, rally_car_QP_solver_grad_eq00);
rally_car_QP_solver_LA_DENSE_MTVM2_9_28_17(params->C2, rally_car_QP_solver_v01, rally_car_QP_solver_D01, rally_car_QP_solver_v00, rally_car_QP_solver_grad_eq01);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C3, rally_car_QP_solver_v02, rally_car_QP_solver_D02, rally_car_QP_solver_v01, rally_car_QP_solver_grad_eq02);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C4, rally_car_QP_solver_v03, rally_car_QP_solver_D02, rally_car_QP_solver_v02, rally_car_QP_solver_grad_eq03);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C5, rally_car_QP_solver_v04, rally_car_QP_solver_D02, rally_car_QP_solver_v03, rally_car_QP_solver_grad_eq04);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C6, rally_car_QP_solver_v05, rally_car_QP_solver_D02, rally_car_QP_solver_v04, rally_car_QP_solver_grad_eq05);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C7, rally_car_QP_solver_v06, rally_car_QP_solver_D02, rally_car_QP_solver_v05, rally_car_QP_solver_grad_eq06);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C8, rally_car_QP_solver_v07, rally_car_QP_solver_D02, rally_car_QP_solver_v06, rally_car_QP_solver_grad_eq07);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C9, rally_car_QP_solver_v08, rally_car_QP_solver_D02, rally_car_QP_solver_v07, rally_car_QP_solver_grad_eq08);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C10, rally_car_QP_solver_v09, rally_car_QP_solver_D02, rally_car_QP_solver_v08, rally_car_QP_solver_grad_eq09);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C11, rally_car_QP_solver_v10, rally_car_QP_solver_D02, rally_car_QP_solver_v09, rally_car_QP_solver_grad_eq10);
rally_car_QP_solver_LA_DIAGZERO_MTVM_9_9(rally_car_QP_solver_D11, rally_car_QP_solver_v10, rally_car_QP_solver_grad_eq11);
info->res_ineq = 0;
rally_car_QP_solver_LA_VSUBADD3_28(params->lb1, rally_car_QP_solver_z00, rally_car_QP_solver_lbIdx00, rally_car_QP_solver_llb00, rally_car_QP_solver_slb00, rally_car_QP_solver_rilb00, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD2_12(rally_car_QP_solver_z00, rally_car_QP_solver_ubIdx00, params->ub1, rally_car_QP_solver_lub00, rally_car_QP_solver_sub00, rally_car_QP_solver_riub00, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD3_28(params->lb2, rally_car_QP_solver_z01, rally_car_QP_solver_lbIdx01, rally_car_QP_solver_llb01, rally_car_QP_solver_slb01, rally_car_QP_solver_rilb01, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD2_12(rally_car_QP_solver_z01, rally_car_QP_solver_ubIdx01, params->ub2, rally_car_QP_solver_lub01, rally_car_QP_solver_sub01, rally_car_QP_solver_riub01, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD3_28(params->lb3, rally_car_QP_solver_z02, rally_car_QP_solver_lbIdx02, rally_car_QP_solver_llb02, rally_car_QP_solver_slb02, rally_car_QP_solver_rilb02, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD2_12(rally_car_QP_solver_z02, rally_car_QP_solver_ubIdx02, params->ub3, rally_car_QP_solver_lub02, rally_car_QP_solver_sub02, rally_car_QP_solver_riub02, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD3_28(params->lb4, rally_car_QP_solver_z03, rally_car_QP_solver_lbIdx03, rally_car_QP_solver_llb03, rally_car_QP_solver_slb03, rally_car_QP_solver_rilb03, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD2_12(rally_car_QP_solver_z03, rally_car_QP_solver_ubIdx03, params->ub4, rally_car_QP_solver_lub03, rally_car_QP_solver_sub03, rally_car_QP_solver_riub03, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD3_28(params->lb5, rally_car_QP_solver_z04, rally_car_QP_solver_lbIdx04, rally_car_QP_solver_llb04, rally_car_QP_solver_slb04, rally_car_QP_solver_rilb04, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD2_12(rally_car_QP_solver_z04, rally_car_QP_solver_ubIdx04, params->ub5, rally_car_QP_solver_lub04, rally_car_QP_solver_sub04, rally_car_QP_solver_riub04, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD3_28(params->lb6, rally_car_QP_solver_z05, rally_car_QP_solver_lbIdx05, rally_car_QP_solver_llb05, rally_car_QP_solver_slb05, rally_car_QP_solver_rilb05, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD2_12(rally_car_QP_solver_z05, rally_car_QP_solver_ubIdx05, params->ub6, rally_car_QP_solver_lub05, rally_car_QP_solver_sub05, rally_car_QP_solver_riub05, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD3_28(params->lb7, rally_car_QP_solver_z06, rally_car_QP_solver_lbIdx06, rally_car_QP_solver_llb06, rally_car_QP_solver_slb06, rally_car_QP_solver_rilb06, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD2_12(rally_car_QP_solver_z06, rally_car_QP_solver_ubIdx06, params->ub7, rally_car_QP_solver_lub06, rally_car_QP_solver_sub06, rally_car_QP_solver_riub06, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD3_28(params->lb8, rally_car_QP_solver_z07, rally_car_QP_solver_lbIdx07, rally_car_QP_solver_llb07, rally_car_QP_solver_slb07, rally_car_QP_solver_rilb07, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD2_12(rally_car_QP_solver_z07, rally_car_QP_solver_ubIdx07, params->ub8, rally_car_QP_solver_lub07, rally_car_QP_solver_sub07, rally_car_QP_solver_riub07, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD3_28(params->lb9, rally_car_QP_solver_z08, rally_car_QP_solver_lbIdx08, rally_car_QP_solver_llb08, rally_car_QP_solver_slb08, rally_car_QP_solver_rilb08, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD2_12(rally_car_QP_solver_z08, rally_car_QP_solver_ubIdx08, params->ub9, rally_car_QP_solver_lub08, rally_car_QP_solver_sub08, rally_car_QP_solver_riub08, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD3_28(params->lb10, rally_car_QP_solver_z09, rally_car_QP_solver_lbIdx09, rally_car_QP_solver_llb09, rally_car_QP_solver_slb09, rally_car_QP_solver_rilb09, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD2_12(rally_car_QP_solver_z09, rally_car_QP_solver_ubIdx09, params->ub10, rally_car_QP_solver_lub09, rally_car_QP_solver_sub09, rally_car_QP_solver_riub09, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD3_28(params->lb11, rally_car_QP_solver_z10, rally_car_QP_solver_lbIdx10, rally_car_QP_solver_llb10, rally_car_QP_solver_slb10, rally_car_QP_solver_rilb10, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD2_12(rally_car_QP_solver_z10, rally_car_QP_solver_ubIdx10, params->ub11, rally_car_QP_solver_lub10, rally_car_QP_solver_sub10, rally_car_QP_solver_riub10, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD3_9(params->lb12, rally_car_QP_solver_z11, rally_car_QP_solver_lbIdx11, rally_car_QP_solver_llb11, rally_car_QP_solver_slb11, rally_car_QP_solver_rilb11, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_VSUBADD2_9(rally_car_QP_solver_z11, rally_car_QP_solver_ubIdx11, params->ub12, rally_car_QP_solver_lub11, rally_car_QP_solver_sub11, rally_car_QP_solver_riub11, &info->dgap, &info->res_ineq);
rally_car_QP_solver_LA_INEQ_B_GRAD_28_28_12(rally_car_QP_solver_lub00, rally_car_QP_solver_sub00, rally_car_QP_solver_riub00, rally_car_QP_solver_llb00, rally_car_QP_solver_slb00, rally_car_QP_solver_rilb00, rally_car_QP_solver_lbIdx00, rally_car_QP_solver_ubIdx00, rally_car_QP_solver_grad_ineq00, rally_car_QP_solver_lubbysub00, rally_car_QP_solver_llbbyslb00);
rally_car_QP_solver_LA_INEQ_B_GRAD_28_28_12(rally_car_QP_solver_lub01, rally_car_QP_solver_sub01, rally_car_QP_solver_riub01, rally_car_QP_solver_llb01, rally_car_QP_solver_slb01, rally_car_QP_solver_rilb01, rally_car_QP_solver_lbIdx01, rally_car_QP_solver_ubIdx01, rally_car_QP_solver_grad_ineq01, rally_car_QP_solver_lubbysub01, rally_car_QP_solver_llbbyslb01);
rally_car_QP_solver_LA_INEQ_B_GRAD_28_28_12(rally_car_QP_solver_lub02, rally_car_QP_solver_sub02, rally_car_QP_solver_riub02, rally_car_QP_solver_llb02, rally_car_QP_solver_slb02, rally_car_QP_solver_rilb02, rally_car_QP_solver_lbIdx02, rally_car_QP_solver_ubIdx02, rally_car_QP_solver_grad_ineq02, rally_car_QP_solver_lubbysub02, rally_car_QP_solver_llbbyslb02);
rally_car_QP_solver_LA_INEQ_B_GRAD_28_28_12(rally_car_QP_solver_lub03, rally_car_QP_solver_sub03, rally_car_QP_solver_riub03, rally_car_QP_solver_llb03, rally_car_QP_solver_slb03, rally_car_QP_solver_rilb03, rally_car_QP_solver_lbIdx03, rally_car_QP_solver_ubIdx03, rally_car_QP_solver_grad_ineq03, rally_car_QP_solver_lubbysub03, rally_car_QP_solver_llbbyslb03);
rally_car_QP_solver_LA_INEQ_B_GRAD_28_28_12(rally_car_QP_solver_lub04, rally_car_QP_solver_sub04, rally_car_QP_solver_riub04, rally_car_QP_solver_llb04, rally_car_QP_solver_slb04, rally_car_QP_solver_rilb04, rally_car_QP_solver_lbIdx04, rally_car_QP_solver_ubIdx04, rally_car_QP_solver_grad_ineq04, rally_car_QP_solver_lubbysub04, rally_car_QP_solver_llbbyslb04);
rally_car_QP_solver_LA_INEQ_B_GRAD_28_28_12(rally_car_QP_solver_lub05, rally_car_QP_solver_sub05, rally_car_QP_solver_riub05, rally_car_QP_solver_llb05, rally_car_QP_solver_slb05, rally_car_QP_solver_rilb05, rally_car_QP_solver_lbIdx05, rally_car_QP_solver_ubIdx05, rally_car_QP_solver_grad_ineq05, rally_car_QP_solver_lubbysub05, rally_car_QP_solver_llbbyslb05);
rally_car_QP_solver_LA_INEQ_B_GRAD_28_28_12(rally_car_QP_solver_lub06, rally_car_QP_solver_sub06, rally_car_QP_solver_riub06, rally_car_QP_solver_llb06, rally_car_QP_solver_slb06, rally_car_QP_solver_rilb06, rally_car_QP_solver_lbIdx06, rally_car_QP_solver_ubIdx06, rally_car_QP_solver_grad_ineq06, rally_car_QP_solver_lubbysub06, rally_car_QP_solver_llbbyslb06);
rally_car_QP_solver_LA_INEQ_B_GRAD_28_28_12(rally_car_QP_solver_lub07, rally_car_QP_solver_sub07, rally_car_QP_solver_riub07, rally_car_QP_solver_llb07, rally_car_QP_solver_slb07, rally_car_QP_solver_rilb07, rally_car_QP_solver_lbIdx07, rally_car_QP_solver_ubIdx07, rally_car_QP_solver_grad_ineq07, rally_car_QP_solver_lubbysub07, rally_car_QP_solver_llbbyslb07);
rally_car_QP_solver_LA_INEQ_B_GRAD_28_28_12(rally_car_QP_solver_lub08, rally_car_QP_solver_sub08, rally_car_QP_solver_riub08, rally_car_QP_solver_llb08, rally_car_QP_solver_slb08, rally_car_QP_solver_rilb08, rally_car_QP_solver_lbIdx08, rally_car_QP_solver_ubIdx08, rally_car_QP_solver_grad_ineq08, rally_car_QP_solver_lubbysub08, rally_car_QP_solver_llbbyslb08);
rally_car_QP_solver_LA_INEQ_B_GRAD_28_28_12(rally_car_QP_solver_lub09, rally_car_QP_solver_sub09, rally_car_QP_solver_riub09, rally_car_QP_solver_llb09, rally_car_QP_solver_slb09, rally_car_QP_solver_rilb09, rally_car_QP_solver_lbIdx09, rally_car_QP_solver_ubIdx09, rally_car_QP_solver_grad_ineq09, rally_car_QP_solver_lubbysub09, rally_car_QP_solver_llbbyslb09);
rally_car_QP_solver_LA_INEQ_B_GRAD_28_28_12(rally_car_QP_solver_lub10, rally_car_QP_solver_sub10, rally_car_QP_solver_riub10, rally_car_QP_solver_llb10, rally_car_QP_solver_slb10, rally_car_QP_solver_rilb10, rally_car_QP_solver_lbIdx10, rally_car_QP_solver_ubIdx10, rally_car_QP_solver_grad_ineq10, rally_car_QP_solver_lubbysub10, rally_car_QP_solver_llbbyslb10);
rally_car_QP_solver_LA_INEQ_B_GRAD_9_9_9(rally_car_QP_solver_lub11, rally_car_QP_solver_sub11, rally_car_QP_solver_riub11, rally_car_QP_solver_llb11, rally_car_QP_solver_slb11, rally_car_QP_solver_rilb11, rally_car_QP_solver_lbIdx11, rally_car_QP_solver_ubIdx11, rally_car_QP_solver_grad_ineq11, rally_car_QP_solver_lubbysub11, rally_car_QP_solver_llbbyslb11);
info->dobj = info->pobj - info->dgap;
info->rdgap = info->pobj ? info->dgap / info->pobj : 1e6;
if( info->rdgap < 0 ) info->rdgap = -info->rdgap;
if( info->mu < rally_car_QP_solver_SET_ACC_KKTCOMPL
    && (info->rdgap < rally_car_QP_solver_SET_ACC_RDGAP || info->dgap < rally_car_QP_solver_SET_ACC_KKTCOMPL)
    && info->res_eq < rally_car_QP_solver_SET_ACC_RESEQ
    && info->res_ineq < rally_car_QP_solver_SET_ACC_RESINEQ ){
exitcode = rally_car_QP_solver_OPTIMAL; break; }
if( info->it == rally_car_QP_solver_SET_MAXIT ){
exitcode = rally_car_QP_solver_MAXITREACHED; break; }
rally_car_QP_solver_LA_VVADD3_317(rally_car_QP_solver_grad_cost, rally_car_QP_solver_grad_eq, rally_car_QP_solver_grad_ineq, rally_car_QP_solver_rd);
rally_car_QP_solver_LA_DIAG_CHOL_LBUB_28_28_12(rally_car_QP_solver_H00, rally_car_QP_solver_llbbyslb00, rally_car_QP_solver_lbIdx00, rally_car_QP_solver_lubbysub00, rally_car_QP_solver_ubIdx00, rally_car_QP_solver_Phi00);
rally_car_QP_solver_LA_DIAG_MATRIXFORWARDSUB_17_28(rally_car_QP_solver_Phi00, params->C1, rally_car_QP_solver_V00);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi00, rally_car_QP_solver_rd00, rally_car_QP_solver_Lbyrd00);
rally_car_QP_solver_LA_DIAG_CHOL_LBUB_28_28_12(rally_car_QP_solver_H00, rally_car_QP_solver_llbbyslb01, rally_car_QP_solver_lbIdx01, rally_car_QP_solver_lubbysub01, rally_car_QP_solver_ubIdx01, rally_car_QP_solver_Phi01);
rally_car_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_28(rally_car_QP_solver_Phi01, params->C2, rally_car_QP_solver_V01);
rally_car_QP_solver_LA_DIAG_MATRIXFORWARDSUB_17_28(rally_car_QP_solver_Phi01, rally_car_QP_solver_D01, rally_car_QP_solver_W01);
rally_car_QP_solver_LA_DENSE_MMTM_17_28_9(rally_car_QP_solver_W01, rally_car_QP_solver_V01, rally_car_QP_solver_Ysd01);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi01, rally_car_QP_solver_rd01, rally_car_QP_solver_Lbyrd01);
rally_car_QP_solver_LA_DIAG_CHOL_LBUB_28_28_12(rally_car_QP_solver_H00, rally_car_QP_solver_llbbyslb02, rally_car_QP_solver_lbIdx02, rally_car_QP_solver_lubbysub02, rally_car_QP_solver_ubIdx02, rally_car_QP_solver_Phi02);
rally_car_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_28(rally_car_QP_solver_Phi02, params->C3, rally_car_QP_solver_V02);
rally_car_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_9_28(rally_car_QP_solver_Phi02, rally_car_QP_solver_D02, rally_car_QP_solver_W02);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMTM_9_28_9(rally_car_QP_solver_W02, rally_car_QP_solver_V02, rally_car_QP_solver_Ysd02);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi02, rally_car_QP_solver_rd02, rally_car_QP_solver_Lbyrd02);
rally_car_QP_solver_LA_DIAG_CHOL_LBUB_28_28_12(rally_car_QP_solver_H00, rally_car_QP_solver_llbbyslb03, rally_car_QP_solver_lbIdx03, rally_car_QP_solver_lubbysub03, rally_car_QP_solver_ubIdx03, rally_car_QP_solver_Phi03);
rally_car_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_28(rally_car_QP_solver_Phi03, params->C4, rally_car_QP_solver_V03);
rally_car_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_9_28(rally_car_QP_solver_Phi03, rally_car_QP_solver_D02, rally_car_QP_solver_W03);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMTM_9_28_9(rally_car_QP_solver_W03, rally_car_QP_solver_V03, rally_car_QP_solver_Ysd03);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi03, rally_car_QP_solver_rd03, rally_car_QP_solver_Lbyrd03);
rally_car_QP_solver_LA_DIAG_CHOL_LBUB_28_28_12(rally_car_QP_solver_H00, rally_car_QP_solver_llbbyslb04, rally_car_QP_solver_lbIdx04, rally_car_QP_solver_lubbysub04, rally_car_QP_solver_ubIdx04, rally_car_QP_solver_Phi04);
rally_car_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_28(rally_car_QP_solver_Phi04, params->C5, rally_car_QP_solver_V04);
rally_car_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_9_28(rally_car_QP_solver_Phi04, rally_car_QP_solver_D02, rally_car_QP_solver_W04);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMTM_9_28_9(rally_car_QP_solver_W04, rally_car_QP_solver_V04, rally_car_QP_solver_Ysd04);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi04, rally_car_QP_solver_rd04, rally_car_QP_solver_Lbyrd04);
rally_car_QP_solver_LA_DIAG_CHOL_LBUB_28_28_12(rally_car_QP_solver_H00, rally_car_QP_solver_llbbyslb05, rally_car_QP_solver_lbIdx05, rally_car_QP_solver_lubbysub05, rally_car_QP_solver_ubIdx05, rally_car_QP_solver_Phi05);
rally_car_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_28(rally_car_QP_solver_Phi05, params->C6, rally_car_QP_solver_V05);
rally_car_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_9_28(rally_car_QP_solver_Phi05, rally_car_QP_solver_D02, rally_car_QP_solver_W05);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMTM_9_28_9(rally_car_QP_solver_W05, rally_car_QP_solver_V05, rally_car_QP_solver_Ysd05);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi05, rally_car_QP_solver_rd05, rally_car_QP_solver_Lbyrd05);
rally_car_QP_solver_LA_DIAG_CHOL_LBUB_28_28_12(rally_car_QP_solver_H00, rally_car_QP_solver_llbbyslb06, rally_car_QP_solver_lbIdx06, rally_car_QP_solver_lubbysub06, rally_car_QP_solver_ubIdx06, rally_car_QP_solver_Phi06);
rally_car_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_28(rally_car_QP_solver_Phi06, params->C7, rally_car_QP_solver_V06);
rally_car_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_9_28(rally_car_QP_solver_Phi06, rally_car_QP_solver_D02, rally_car_QP_solver_W06);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMTM_9_28_9(rally_car_QP_solver_W06, rally_car_QP_solver_V06, rally_car_QP_solver_Ysd06);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi06, rally_car_QP_solver_rd06, rally_car_QP_solver_Lbyrd06);
rally_car_QP_solver_LA_DIAG_CHOL_LBUB_28_28_12(rally_car_QP_solver_H00, rally_car_QP_solver_llbbyslb07, rally_car_QP_solver_lbIdx07, rally_car_QP_solver_lubbysub07, rally_car_QP_solver_ubIdx07, rally_car_QP_solver_Phi07);
rally_car_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_28(rally_car_QP_solver_Phi07, params->C8, rally_car_QP_solver_V07);
rally_car_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_9_28(rally_car_QP_solver_Phi07, rally_car_QP_solver_D02, rally_car_QP_solver_W07);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMTM_9_28_9(rally_car_QP_solver_W07, rally_car_QP_solver_V07, rally_car_QP_solver_Ysd07);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi07, rally_car_QP_solver_rd07, rally_car_QP_solver_Lbyrd07);
rally_car_QP_solver_LA_DIAG_CHOL_LBUB_28_28_12(rally_car_QP_solver_H00, rally_car_QP_solver_llbbyslb08, rally_car_QP_solver_lbIdx08, rally_car_QP_solver_lubbysub08, rally_car_QP_solver_ubIdx08, rally_car_QP_solver_Phi08);
rally_car_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_28(rally_car_QP_solver_Phi08, params->C9, rally_car_QP_solver_V08);
rally_car_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_9_28(rally_car_QP_solver_Phi08, rally_car_QP_solver_D02, rally_car_QP_solver_W08);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMTM_9_28_9(rally_car_QP_solver_W08, rally_car_QP_solver_V08, rally_car_QP_solver_Ysd08);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi08, rally_car_QP_solver_rd08, rally_car_QP_solver_Lbyrd08);
rally_car_QP_solver_LA_DIAG_CHOL_LBUB_28_28_12(rally_car_QP_solver_H00, rally_car_QP_solver_llbbyslb09, rally_car_QP_solver_lbIdx09, rally_car_QP_solver_lubbysub09, rally_car_QP_solver_ubIdx09, rally_car_QP_solver_Phi09);
rally_car_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_28(rally_car_QP_solver_Phi09, params->C10, rally_car_QP_solver_V09);
rally_car_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_9_28(rally_car_QP_solver_Phi09, rally_car_QP_solver_D02, rally_car_QP_solver_W09);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMTM_9_28_9(rally_car_QP_solver_W09, rally_car_QP_solver_V09, rally_car_QP_solver_Ysd09);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi09, rally_car_QP_solver_rd09, rally_car_QP_solver_Lbyrd09);
rally_car_QP_solver_LA_DIAG_CHOL_LBUB_28_28_12(rally_car_QP_solver_H00, rally_car_QP_solver_llbbyslb10, rally_car_QP_solver_lbIdx10, rally_car_QP_solver_lubbysub10, rally_car_QP_solver_ubIdx10, rally_car_QP_solver_Phi10);
rally_car_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_28(rally_car_QP_solver_Phi10, params->C11, rally_car_QP_solver_V10);
rally_car_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_9_28(rally_car_QP_solver_Phi10, rally_car_QP_solver_D02, rally_car_QP_solver_W10);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMTM_9_28_9(rally_car_QP_solver_W10, rally_car_QP_solver_V10, rally_car_QP_solver_Ysd10);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi10, rally_car_QP_solver_rd10, rally_car_QP_solver_Lbyrd10);
rally_car_QP_solver_LA_DIAG_CHOL_ONELOOP_LBUB_9_9_9(rally_car_QP_solver_H11, rally_car_QP_solver_llbbyslb11, rally_car_QP_solver_lbIdx11, rally_car_QP_solver_lubbysub11, rally_car_QP_solver_ubIdx11, rally_car_QP_solver_Phi11);
rally_car_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_9_9(rally_car_QP_solver_Phi11, rally_car_QP_solver_D11, rally_car_QP_solver_W11);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_9(rally_car_QP_solver_Phi11, rally_car_QP_solver_rd11, rally_car_QP_solver_Lbyrd11);
rally_car_QP_solver_LA_DENSE_MMT2_17_28_28(rally_car_QP_solver_V00, rally_car_QP_solver_W01, rally_car_QP_solver_Yd00);
rally_car_QP_solver_LA_DENSE_MVMSUB2_17_28_28(rally_car_QP_solver_V00, rally_car_QP_solver_Lbyrd00, rally_car_QP_solver_W01, rally_car_QP_solver_Lbyrd01, rally_car_QP_solver_re00, rally_car_QP_solver_beta00);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMT2_9_28_28(rally_car_QP_solver_V01, rally_car_QP_solver_W02, rally_car_QP_solver_Yd01);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_9_28_28(rally_car_QP_solver_V01, rally_car_QP_solver_Lbyrd01, rally_car_QP_solver_W02, rally_car_QP_solver_Lbyrd02, rally_car_QP_solver_re01, rally_car_QP_solver_beta01);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMT2_9_28_28(rally_car_QP_solver_V02, rally_car_QP_solver_W03, rally_car_QP_solver_Yd02);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_9_28_28(rally_car_QP_solver_V02, rally_car_QP_solver_Lbyrd02, rally_car_QP_solver_W03, rally_car_QP_solver_Lbyrd03, rally_car_QP_solver_re02, rally_car_QP_solver_beta02);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMT2_9_28_28(rally_car_QP_solver_V03, rally_car_QP_solver_W04, rally_car_QP_solver_Yd03);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_9_28_28(rally_car_QP_solver_V03, rally_car_QP_solver_Lbyrd03, rally_car_QP_solver_W04, rally_car_QP_solver_Lbyrd04, rally_car_QP_solver_re03, rally_car_QP_solver_beta03);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMT2_9_28_28(rally_car_QP_solver_V04, rally_car_QP_solver_W05, rally_car_QP_solver_Yd04);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_9_28_28(rally_car_QP_solver_V04, rally_car_QP_solver_Lbyrd04, rally_car_QP_solver_W05, rally_car_QP_solver_Lbyrd05, rally_car_QP_solver_re04, rally_car_QP_solver_beta04);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMT2_9_28_28(rally_car_QP_solver_V05, rally_car_QP_solver_W06, rally_car_QP_solver_Yd05);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_9_28_28(rally_car_QP_solver_V05, rally_car_QP_solver_Lbyrd05, rally_car_QP_solver_W06, rally_car_QP_solver_Lbyrd06, rally_car_QP_solver_re05, rally_car_QP_solver_beta05);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMT2_9_28_28(rally_car_QP_solver_V06, rally_car_QP_solver_W07, rally_car_QP_solver_Yd06);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_9_28_28(rally_car_QP_solver_V06, rally_car_QP_solver_Lbyrd06, rally_car_QP_solver_W07, rally_car_QP_solver_Lbyrd07, rally_car_QP_solver_re06, rally_car_QP_solver_beta06);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMT2_9_28_28(rally_car_QP_solver_V07, rally_car_QP_solver_W08, rally_car_QP_solver_Yd07);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_9_28_28(rally_car_QP_solver_V07, rally_car_QP_solver_Lbyrd07, rally_car_QP_solver_W08, rally_car_QP_solver_Lbyrd08, rally_car_QP_solver_re07, rally_car_QP_solver_beta07);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMT2_9_28_28(rally_car_QP_solver_V08, rally_car_QP_solver_W09, rally_car_QP_solver_Yd08);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_9_28_28(rally_car_QP_solver_V08, rally_car_QP_solver_Lbyrd08, rally_car_QP_solver_W09, rally_car_QP_solver_Lbyrd09, rally_car_QP_solver_re08, rally_car_QP_solver_beta08);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMT2_9_28_28(rally_car_QP_solver_V09, rally_car_QP_solver_W10, rally_car_QP_solver_Yd09);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_9_28_28(rally_car_QP_solver_V09, rally_car_QP_solver_Lbyrd09, rally_car_QP_solver_W10, rally_car_QP_solver_Lbyrd10, rally_car_QP_solver_re09, rally_car_QP_solver_beta09);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MMT2_9_28_9(rally_car_QP_solver_V10, rally_car_QP_solver_W11, rally_car_QP_solver_Yd10);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_9_28_9(rally_car_QP_solver_V10, rally_car_QP_solver_Lbyrd10, rally_car_QP_solver_W11, rally_car_QP_solver_Lbyrd11, rally_car_QP_solver_re10, rally_car_QP_solver_beta10);
rally_car_QP_solver_LA_DENSE_CHOL_17(rally_car_QP_solver_Yd00, rally_car_QP_solver_Ld00);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_17(rally_car_QP_solver_Ld00, rally_car_QP_solver_beta00, rally_car_QP_solver_yy00);
rally_car_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_9_17(rally_car_QP_solver_Ld00, rally_car_QP_solver_Ysd01, rally_car_QP_solver_Lsd01);
rally_car_QP_solver_LA_DENSE_MMTSUB_9_17(rally_car_QP_solver_Lsd01, rally_car_QP_solver_Yd01);
rally_car_QP_solver_LA_DENSE_CHOL_9(rally_car_QP_solver_Yd01, rally_car_QP_solver_Ld01);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_17(rally_car_QP_solver_Lsd01, rally_car_QP_solver_yy00, rally_car_QP_solver_beta01, rally_car_QP_solver_bmy01);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld01, rally_car_QP_solver_bmy01, rally_car_QP_solver_yy01);
rally_car_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_9_9(rally_car_QP_solver_Ld01, rally_car_QP_solver_Ysd02, rally_car_QP_solver_Lsd02);
rally_car_QP_solver_LA_DENSE_MMTSUB_9_9(rally_car_QP_solver_Lsd02, rally_car_QP_solver_Yd02);
rally_car_QP_solver_LA_DENSE_CHOL_9(rally_car_QP_solver_Yd02, rally_car_QP_solver_Ld02);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd02, rally_car_QP_solver_yy01, rally_car_QP_solver_beta02, rally_car_QP_solver_bmy02);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld02, rally_car_QP_solver_bmy02, rally_car_QP_solver_yy02);
rally_car_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_9_9(rally_car_QP_solver_Ld02, rally_car_QP_solver_Ysd03, rally_car_QP_solver_Lsd03);
rally_car_QP_solver_LA_DENSE_MMTSUB_9_9(rally_car_QP_solver_Lsd03, rally_car_QP_solver_Yd03);
rally_car_QP_solver_LA_DENSE_CHOL_9(rally_car_QP_solver_Yd03, rally_car_QP_solver_Ld03);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd03, rally_car_QP_solver_yy02, rally_car_QP_solver_beta03, rally_car_QP_solver_bmy03);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld03, rally_car_QP_solver_bmy03, rally_car_QP_solver_yy03);
rally_car_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_9_9(rally_car_QP_solver_Ld03, rally_car_QP_solver_Ysd04, rally_car_QP_solver_Lsd04);
rally_car_QP_solver_LA_DENSE_MMTSUB_9_9(rally_car_QP_solver_Lsd04, rally_car_QP_solver_Yd04);
rally_car_QP_solver_LA_DENSE_CHOL_9(rally_car_QP_solver_Yd04, rally_car_QP_solver_Ld04);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd04, rally_car_QP_solver_yy03, rally_car_QP_solver_beta04, rally_car_QP_solver_bmy04);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld04, rally_car_QP_solver_bmy04, rally_car_QP_solver_yy04);
rally_car_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_9_9(rally_car_QP_solver_Ld04, rally_car_QP_solver_Ysd05, rally_car_QP_solver_Lsd05);
rally_car_QP_solver_LA_DENSE_MMTSUB_9_9(rally_car_QP_solver_Lsd05, rally_car_QP_solver_Yd05);
rally_car_QP_solver_LA_DENSE_CHOL_9(rally_car_QP_solver_Yd05, rally_car_QP_solver_Ld05);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd05, rally_car_QP_solver_yy04, rally_car_QP_solver_beta05, rally_car_QP_solver_bmy05);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld05, rally_car_QP_solver_bmy05, rally_car_QP_solver_yy05);
rally_car_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_9_9(rally_car_QP_solver_Ld05, rally_car_QP_solver_Ysd06, rally_car_QP_solver_Lsd06);
rally_car_QP_solver_LA_DENSE_MMTSUB_9_9(rally_car_QP_solver_Lsd06, rally_car_QP_solver_Yd06);
rally_car_QP_solver_LA_DENSE_CHOL_9(rally_car_QP_solver_Yd06, rally_car_QP_solver_Ld06);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd06, rally_car_QP_solver_yy05, rally_car_QP_solver_beta06, rally_car_QP_solver_bmy06);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld06, rally_car_QP_solver_bmy06, rally_car_QP_solver_yy06);
rally_car_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_9_9(rally_car_QP_solver_Ld06, rally_car_QP_solver_Ysd07, rally_car_QP_solver_Lsd07);
rally_car_QP_solver_LA_DENSE_MMTSUB_9_9(rally_car_QP_solver_Lsd07, rally_car_QP_solver_Yd07);
rally_car_QP_solver_LA_DENSE_CHOL_9(rally_car_QP_solver_Yd07, rally_car_QP_solver_Ld07);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd07, rally_car_QP_solver_yy06, rally_car_QP_solver_beta07, rally_car_QP_solver_bmy07);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld07, rally_car_QP_solver_bmy07, rally_car_QP_solver_yy07);
rally_car_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_9_9(rally_car_QP_solver_Ld07, rally_car_QP_solver_Ysd08, rally_car_QP_solver_Lsd08);
rally_car_QP_solver_LA_DENSE_MMTSUB_9_9(rally_car_QP_solver_Lsd08, rally_car_QP_solver_Yd08);
rally_car_QP_solver_LA_DENSE_CHOL_9(rally_car_QP_solver_Yd08, rally_car_QP_solver_Ld08);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd08, rally_car_QP_solver_yy07, rally_car_QP_solver_beta08, rally_car_QP_solver_bmy08);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld08, rally_car_QP_solver_bmy08, rally_car_QP_solver_yy08);
rally_car_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_9_9(rally_car_QP_solver_Ld08, rally_car_QP_solver_Ysd09, rally_car_QP_solver_Lsd09);
rally_car_QP_solver_LA_DENSE_MMTSUB_9_9(rally_car_QP_solver_Lsd09, rally_car_QP_solver_Yd09);
rally_car_QP_solver_LA_DENSE_CHOL_9(rally_car_QP_solver_Yd09, rally_car_QP_solver_Ld09);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd09, rally_car_QP_solver_yy08, rally_car_QP_solver_beta09, rally_car_QP_solver_bmy09);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld09, rally_car_QP_solver_bmy09, rally_car_QP_solver_yy09);
rally_car_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_9_9(rally_car_QP_solver_Ld09, rally_car_QP_solver_Ysd10, rally_car_QP_solver_Lsd10);
rally_car_QP_solver_LA_DENSE_MMTSUB_9_9(rally_car_QP_solver_Lsd10, rally_car_QP_solver_Yd10);
rally_car_QP_solver_LA_DENSE_CHOL_9(rally_car_QP_solver_Yd10, rally_car_QP_solver_Ld10);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd10, rally_car_QP_solver_yy09, rally_car_QP_solver_beta10, rally_car_QP_solver_bmy10);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld10, rally_car_QP_solver_bmy10, rally_car_QP_solver_yy10);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld10, rally_car_QP_solver_yy10, rally_car_QP_solver_dvaff10);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd10, rally_car_QP_solver_dvaff10, rally_car_QP_solver_yy09, rally_car_QP_solver_bmy09);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld09, rally_car_QP_solver_bmy09, rally_car_QP_solver_dvaff09);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd09, rally_car_QP_solver_dvaff09, rally_car_QP_solver_yy08, rally_car_QP_solver_bmy08);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld08, rally_car_QP_solver_bmy08, rally_car_QP_solver_dvaff08);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd08, rally_car_QP_solver_dvaff08, rally_car_QP_solver_yy07, rally_car_QP_solver_bmy07);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld07, rally_car_QP_solver_bmy07, rally_car_QP_solver_dvaff07);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd07, rally_car_QP_solver_dvaff07, rally_car_QP_solver_yy06, rally_car_QP_solver_bmy06);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld06, rally_car_QP_solver_bmy06, rally_car_QP_solver_dvaff06);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd06, rally_car_QP_solver_dvaff06, rally_car_QP_solver_yy05, rally_car_QP_solver_bmy05);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld05, rally_car_QP_solver_bmy05, rally_car_QP_solver_dvaff05);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd05, rally_car_QP_solver_dvaff05, rally_car_QP_solver_yy04, rally_car_QP_solver_bmy04);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld04, rally_car_QP_solver_bmy04, rally_car_QP_solver_dvaff04);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd04, rally_car_QP_solver_dvaff04, rally_car_QP_solver_yy03, rally_car_QP_solver_bmy03);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld03, rally_car_QP_solver_bmy03, rally_car_QP_solver_dvaff03);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd03, rally_car_QP_solver_dvaff03, rally_car_QP_solver_yy02, rally_car_QP_solver_bmy02);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld02, rally_car_QP_solver_bmy02, rally_car_QP_solver_dvaff02);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd02, rally_car_QP_solver_dvaff02, rally_car_QP_solver_yy01, rally_car_QP_solver_bmy01);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld01, rally_car_QP_solver_bmy01, rally_car_QP_solver_dvaff01);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_17(rally_car_QP_solver_Lsd01, rally_car_QP_solver_dvaff01, rally_car_QP_solver_yy00, rally_car_QP_solver_bmy00);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_17(rally_car_QP_solver_Ld00, rally_car_QP_solver_bmy00, rally_car_QP_solver_dvaff00);
rally_car_QP_solver_LA_DENSE_MTVM_17_28(params->C1, rally_car_QP_solver_dvaff00, rally_car_QP_solver_grad_eq00);
rally_car_QP_solver_LA_DENSE_MTVM2_9_28_17(params->C2, rally_car_QP_solver_dvaff01, rally_car_QP_solver_D01, rally_car_QP_solver_dvaff00, rally_car_QP_solver_grad_eq01);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C3, rally_car_QP_solver_dvaff02, rally_car_QP_solver_D02, rally_car_QP_solver_dvaff01, rally_car_QP_solver_grad_eq02);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C4, rally_car_QP_solver_dvaff03, rally_car_QP_solver_D02, rally_car_QP_solver_dvaff02, rally_car_QP_solver_grad_eq03);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C5, rally_car_QP_solver_dvaff04, rally_car_QP_solver_D02, rally_car_QP_solver_dvaff03, rally_car_QP_solver_grad_eq04);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C6, rally_car_QP_solver_dvaff05, rally_car_QP_solver_D02, rally_car_QP_solver_dvaff04, rally_car_QP_solver_grad_eq05);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C7, rally_car_QP_solver_dvaff06, rally_car_QP_solver_D02, rally_car_QP_solver_dvaff05, rally_car_QP_solver_grad_eq06);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C8, rally_car_QP_solver_dvaff07, rally_car_QP_solver_D02, rally_car_QP_solver_dvaff06, rally_car_QP_solver_grad_eq07);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C9, rally_car_QP_solver_dvaff08, rally_car_QP_solver_D02, rally_car_QP_solver_dvaff07, rally_car_QP_solver_grad_eq08);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C10, rally_car_QP_solver_dvaff09, rally_car_QP_solver_D02, rally_car_QP_solver_dvaff08, rally_car_QP_solver_grad_eq09);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C11, rally_car_QP_solver_dvaff10, rally_car_QP_solver_D02, rally_car_QP_solver_dvaff09, rally_car_QP_solver_grad_eq10);
rally_car_QP_solver_LA_DIAGZERO_MTVM_9_9(rally_car_QP_solver_D11, rally_car_QP_solver_dvaff10, rally_car_QP_solver_grad_eq11);
rally_car_QP_solver_LA_VSUB2_317(rally_car_QP_solver_rd, rally_car_QP_solver_grad_eq, rally_car_QP_solver_rd);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi00, rally_car_QP_solver_rd00, rally_car_QP_solver_dzaff00);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi01, rally_car_QP_solver_rd01, rally_car_QP_solver_dzaff01);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi02, rally_car_QP_solver_rd02, rally_car_QP_solver_dzaff02);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi03, rally_car_QP_solver_rd03, rally_car_QP_solver_dzaff03);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi04, rally_car_QP_solver_rd04, rally_car_QP_solver_dzaff04);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi05, rally_car_QP_solver_rd05, rally_car_QP_solver_dzaff05);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi06, rally_car_QP_solver_rd06, rally_car_QP_solver_dzaff06);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi07, rally_car_QP_solver_rd07, rally_car_QP_solver_dzaff07);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi08, rally_car_QP_solver_rd08, rally_car_QP_solver_dzaff08);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi09, rally_car_QP_solver_rd09, rally_car_QP_solver_dzaff09);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi10, rally_car_QP_solver_rd10, rally_car_QP_solver_dzaff10);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_9(rally_car_QP_solver_Phi11, rally_car_QP_solver_rd11, rally_car_QP_solver_dzaff11);
rally_car_QP_solver_LA_VSUB_INDEXED_28(rally_car_QP_solver_dzaff00, rally_car_QP_solver_lbIdx00, rally_car_QP_solver_rilb00, rally_car_QP_solver_dslbaff00);
rally_car_QP_solver_LA_VSUB3_28(rally_car_QP_solver_llbbyslb00, rally_car_QP_solver_dslbaff00, rally_car_QP_solver_llb00, rally_car_QP_solver_dllbaff00);
rally_car_QP_solver_LA_VSUB2_INDEXED_12(rally_car_QP_solver_riub00, rally_car_QP_solver_dzaff00, rally_car_QP_solver_ubIdx00, rally_car_QP_solver_dsubaff00);
rally_car_QP_solver_LA_VSUB3_12(rally_car_QP_solver_lubbysub00, rally_car_QP_solver_dsubaff00, rally_car_QP_solver_lub00, rally_car_QP_solver_dlubaff00);
rally_car_QP_solver_LA_VSUB_INDEXED_28(rally_car_QP_solver_dzaff01, rally_car_QP_solver_lbIdx01, rally_car_QP_solver_rilb01, rally_car_QP_solver_dslbaff01);
rally_car_QP_solver_LA_VSUB3_28(rally_car_QP_solver_llbbyslb01, rally_car_QP_solver_dslbaff01, rally_car_QP_solver_llb01, rally_car_QP_solver_dllbaff01);
rally_car_QP_solver_LA_VSUB2_INDEXED_12(rally_car_QP_solver_riub01, rally_car_QP_solver_dzaff01, rally_car_QP_solver_ubIdx01, rally_car_QP_solver_dsubaff01);
rally_car_QP_solver_LA_VSUB3_12(rally_car_QP_solver_lubbysub01, rally_car_QP_solver_dsubaff01, rally_car_QP_solver_lub01, rally_car_QP_solver_dlubaff01);
rally_car_QP_solver_LA_VSUB_INDEXED_28(rally_car_QP_solver_dzaff02, rally_car_QP_solver_lbIdx02, rally_car_QP_solver_rilb02, rally_car_QP_solver_dslbaff02);
rally_car_QP_solver_LA_VSUB3_28(rally_car_QP_solver_llbbyslb02, rally_car_QP_solver_dslbaff02, rally_car_QP_solver_llb02, rally_car_QP_solver_dllbaff02);
rally_car_QP_solver_LA_VSUB2_INDEXED_12(rally_car_QP_solver_riub02, rally_car_QP_solver_dzaff02, rally_car_QP_solver_ubIdx02, rally_car_QP_solver_dsubaff02);
rally_car_QP_solver_LA_VSUB3_12(rally_car_QP_solver_lubbysub02, rally_car_QP_solver_dsubaff02, rally_car_QP_solver_lub02, rally_car_QP_solver_dlubaff02);
rally_car_QP_solver_LA_VSUB_INDEXED_28(rally_car_QP_solver_dzaff03, rally_car_QP_solver_lbIdx03, rally_car_QP_solver_rilb03, rally_car_QP_solver_dslbaff03);
rally_car_QP_solver_LA_VSUB3_28(rally_car_QP_solver_llbbyslb03, rally_car_QP_solver_dslbaff03, rally_car_QP_solver_llb03, rally_car_QP_solver_dllbaff03);
rally_car_QP_solver_LA_VSUB2_INDEXED_12(rally_car_QP_solver_riub03, rally_car_QP_solver_dzaff03, rally_car_QP_solver_ubIdx03, rally_car_QP_solver_dsubaff03);
rally_car_QP_solver_LA_VSUB3_12(rally_car_QP_solver_lubbysub03, rally_car_QP_solver_dsubaff03, rally_car_QP_solver_lub03, rally_car_QP_solver_dlubaff03);
rally_car_QP_solver_LA_VSUB_INDEXED_28(rally_car_QP_solver_dzaff04, rally_car_QP_solver_lbIdx04, rally_car_QP_solver_rilb04, rally_car_QP_solver_dslbaff04);
rally_car_QP_solver_LA_VSUB3_28(rally_car_QP_solver_llbbyslb04, rally_car_QP_solver_dslbaff04, rally_car_QP_solver_llb04, rally_car_QP_solver_dllbaff04);
rally_car_QP_solver_LA_VSUB2_INDEXED_12(rally_car_QP_solver_riub04, rally_car_QP_solver_dzaff04, rally_car_QP_solver_ubIdx04, rally_car_QP_solver_dsubaff04);
rally_car_QP_solver_LA_VSUB3_12(rally_car_QP_solver_lubbysub04, rally_car_QP_solver_dsubaff04, rally_car_QP_solver_lub04, rally_car_QP_solver_dlubaff04);
rally_car_QP_solver_LA_VSUB_INDEXED_28(rally_car_QP_solver_dzaff05, rally_car_QP_solver_lbIdx05, rally_car_QP_solver_rilb05, rally_car_QP_solver_dslbaff05);
rally_car_QP_solver_LA_VSUB3_28(rally_car_QP_solver_llbbyslb05, rally_car_QP_solver_dslbaff05, rally_car_QP_solver_llb05, rally_car_QP_solver_dllbaff05);
rally_car_QP_solver_LA_VSUB2_INDEXED_12(rally_car_QP_solver_riub05, rally_car_QP_solver_dzaff05, rally_car_QP_solver_ubIdx05, rally_car_QP_solver_dsubaff05);
rally_car_QP_solver_LA_VSUB3_12(rally_car_QP_solver_lubbysub05, rally_car_QP_solver_dsubaff05, rally_car_QP_solver_lub05, rally_car_QP_solver_dlubaff05);
rally_car_QP_solver_LA_VSUB_INDEXED_28(rally_car_QP_solver_dzaff06, rally_car_QP_solver_lbIdx06, rally_car_QP_solver_rilb06, rally_car_QP_solver_dslbaff06);
rally_car_QP_solver_LA_VSUB3_28(rally_car_QP_solver_llbbyslb06, rally_car_QP_solver_dslbaff06, rally_car_QP_solver_llb06, rally_car_QP_solver_dllbaff06);
rally_car_QP_solver_LA_VSUB2_INDEXED_12(rally_car_QP_solver_riub06, rally_car_QP_solver_dzaff06, rally_car_QP_solver_ubIdx06, rally_car_QP_solver_dsubaff06);
rally_car_QP_solver_LA_VSUB3_12(rally_car_QP_solver_lubbysub06, rally_car_QP_solver_dsubaff06, rally_car_QP_solver_lub06, rally_car_QP_solver_dlubaff06);
rally_car_QP_solver_LA_VSUB_INDEXED_28(rally_car_QP_solver_dzaff07, rally_car_QP_solver_lbIdx07, rally_car_QP_solver_rilb07, rally_car_QP_solver_dslbaff07);
rally_car_QP_solver_LA_VSUB3_28(rally_car_QP_solver_llbbyslb07, rally_car_QP_solver_dslbaff07, rally_car_QP_solver_llb07, rally_car_QP_solver_dllbaff07);
rally_car_QP_solver_LA_VSUB2_INDEXED_12(rally_car_QP_solver_riub07, rally_car_QP_solver_dzaff07, rally_car_QP_solver_ubIdx07, rally_car_QP_solver_dsubaff07);
rally_car_QP_solver_LA_VSUB3_12(rally_car_QP_solver_lubbysub07, rally_car_QP_solver_dsubaff07, rally_car_QP_solver_lub07, rally_car_QP_solver_dlubaff07);
rally_car_QP_solver_LA_VSUB_INDEXED_28(rally_car_QP_solver_dzaff08, rally_car_QP_solver_lbIdx08, rally_car_QP_solver_rilb08, rally_car_QP_solver_dslbaff08);
rally_car_QP_solver_LA_VSUB3_28(rally_car_QP_solver_llbbyslb08, rally_car_QP_solver_dslbaff08, rally_car_QP_solver_llb08, rally_car_QP_solver_dllbaff08);
rally_car_QP_solver_LA_VSUB2_INDEXED_12(rally_car_QP_solver_riub08, rally_car_QP_solver_dzaff08, rally_car_QP_solver_ubIdx08, rally_car_QP_solver_dsubaff08);
rally_car_QP_solver_LA_VSUB3_12(rally_car_QP_solver_lubbysub08, rally_car_QP_solver_dsubaff08, rally_car_QP_solver_lub08, rally_car_QP_solver_dlubaff08);
rally_car_QP_solver_LA_VSUB_INDEXED_28(rally_car_QP_solver_dzaff09, rally_car_QP_solver_lbIdx09, rally_car_QP_solver_rilb09, rally_car_QP_solver_dslbaff09);
rally_car_QP_solver_LA_VSUB3_28(rally_car_QP_solver_llbbyslb09, rally_car_QP_solver_dslbaff09, rally_car_QP_solver_llb09, rally_car_QP_solver_dllbaff09);
rally_car_QP_solver_LA_VSUB2_INDEXED_12(rally_car_QP_solver_riub09, rally_car_QP_solver_dzaff09, rally_car_QP_solver_ubIdx09, rally_car_QP_solver_dsubaff09);
rally_car_QP_solver_LA_VSUB3_12(rally_car_QP_solver_lubbysub09, rally_car_QP_solver_dsubaff09, rally_car_QP_solver_lub09, rally_car_QP_solver_dlubaff09);
rally_car_QP_solver_LA_VSUB_INDEXED_28(rally_car_QP_solver_dzaff10, rally_car_QP_solver_lbIdx10, rally_car_QP_solver_rilb10, rally_car_QP_solver_dslbaff10);
rally_car_QP_solver_LA_VSUB3_28(rally_car_QP_solver_llbbyslb10, rally_car_QP_solver_dslbaff10, rally_car_QP_solver_llb10, rally_car_QP_solver_dllbaff10);
rally_car_QP_solver_LA_VSUB2_INDEXED_12(rally_car_QP_solver_riub10, rally_car_QP_solver_dzaff10, rally_car_QP_solver_ubIdx10, rally_car_QP_solver_dsubaff10);
rally_car_QP_solver_LA_VSUB3_12(rally_car_QP_solver_lubbysub10, rally_car_QP_solver_dsubaff10, rally_car_QP_solver_lub10, rally_car_QP_solver_dlubaff10);
rally_car_QP_solver_LA_VSUB_INDEXED_9(rally_car_QP_solver_dzaff11, rally_car_QP_solver_lbIdx11, rally_car_QP_solver_rilb11, rally_car_QP_solver_dslbaff11);
rally_car_QP_solver_LA_VSUB3_9(rally_car_QP_solver_llbbyslb11, rally_car_QP_solver_dslbaff11, rally_car_QP_solver_llb11, rally_car_QP_solver_dllbaff11);
rally_car_QP_solver_LA_VSUB2_INDEXED_9(rally_car_QP_solver_riub11, rally_car_QP_solver_dzaff11, rally_car_QP_solver_ubIdx11, rally_car_QP_solver_dsubaff11);
rally_car_QP_solver_LA_VSUB3_9(rally_car_QP_solver_lubbysub11, rally_car_QP_solver_dsubaff11, rally_car_QP_solver_lub11, rally_car_QP_solver_dlubaff11);
info->lsit_aff = rally_car_QP_solver_LINESEARCH_BACKTRACKING_AFFINE(rally_car_QP_solver_l, rally_car_QP_solver_s, rally_car_QP_solver_dl_aff, rally_car_QP_solver_ds_aff, &info->step_aff, &info->mu_aff);
if( info->lsit_aff == rally_car_QP_solver_NOPROGRESS ){
exitcode = rally_car_QP_solver_NOPROGRESS; break;
}
sigma_3rdroot = info->mu_aff / info->mu;
info->sigma = sigma_3rdroot*sigma_3rdroot*sigma_3rdroot;
musigma = info->mu * info->sigma;
rally_car_QP_solver_LA_VSUB5_458(rally_car_QP_solver_ds_aff, rally_car_QP_solver_dl_aff, info->mu, info->sigma, rally_car_QP_solver_ccrhs);
rally_car_QP_solver_LA_VSUB6_INDEXED_28_12_28(rally_car_QP_solver_ccrhsub00, rally_car_QP_solver_sub00, rally_car_QP_solver_ubIdx00, rally_car_QP_solver_ccrhsl00, rally_car_QP_solver_slb00, rally_car_QP_solver_lbIdx00, rally_car_QP_solver_rd00);
rally_car_QP_solver_LA_VSUB6_INDEXED_28_12_28(rally_car_QP_solver_ccrhsub01, rally_car_QP_solver_sub01, rally_car_QP_solver_ubIdx01, rally_car_QP_solver_ccrhsl01, rally_car_QP_solver_slb01, rally_car_QP_solver_lbIdx01, rally_car_QP_solver_rd01);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi00, rally_car_QP_solver_rd00, rally_car_QP_solver_Lbyrd00);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi01, rally_car_QP_solver_rd01, rally_car_QP_solver_Lbyrd01);
rally_car_QP_solver_LA_DENSE_2MVMADD_17_28_28(rally_car_QP_solver_V00, rally_car_QP_solver_Lbyrd00, rally_car_QP_solver_W01, rally_car_QP_solver_Lbyrd01, rally_car_QP_solver_beta00);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_17(rally_car_QP_solver_Ld00, rally_car_QP_solver_beta00, rally_car_QP_solver_yy00);
rally_car_QP_solver_LA_VSUB6_INDEXED_28_12_28(rally_car_QP_solver_ccrhsub02, rally_car_QP_solver_sub02, rally_car_QP_solver_ubIdx02, rally_car_QP_solver_ccrhsl02, rally_car_QP_solver_slb02, rally_car_QP_solver_lbIdx02, rally_car_QP_solver_rd02);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi02, rally_car_QP_solver_rd02, rally_car_QP_solver_Lbyrd02);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_9_28_28(rally_car_QP_solver_V01, rally_car_QP_solver_Lbyrd01, rally_car_QP_solver_W02, rally_car_QP_solver_Lbyrd02, rally_car_QP_solver_beta01);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_17(rally_car_QP_solver_Lsd01, rally_car_QP_solver_yy00, rally_car_QP_solver_beta01, rally_car_QP_solver_bmy01);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld01, rally_car_QP_solver_bmy01, rally_car_QP_solver_yy01);
rally_car_QP_solver_LA_VSUB6_INDEXED_28_12_28(rally_car_QP_solver_ccrhsub03, rally_car_QP_solver_sub03, rally_car_QP_solver_ubIdx03, rally_car_QP_solver_ccrhsl03, rally_car_QP_solver_slb03, rally_car_QP_solver_lbIdx03, rally_car_QP_solver_rd03);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi03, rally_car_QP_solver_rd03, rally_car_QP_solver_Lbyrd03);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_9_28_28(rally_car_QP_solver_V02, rally_car_QP_solver_Lbyrd02, rally_car_QP_solver_W03, rally_car_QP_solver_Lbyrd03, rally_car_QP_solver_beta02);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd02, rally_car_QP_solver_yy01, rally_car_QP_solver_beta02, rally_car_QP_solver_bmy02);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld02, rally_car_QP_solver_bmy02, rally_car_QP_solver_yy02);
rally_car_QP_solver_LA_VSUB6_INDEXED_28_12_28(rally_car_QP_solver_ccrhsub04, rally_car_QP_solver_sub04, rally_car_QP_solver_ubIdx04, rally_car_QP_solver_ccrhsl04, rally_car_QP_solver_slb04, rally_car_QP_solver_lbIdx04, rally_car_QP_solver_rd04);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi04, rally_car_QP_solver_rd04, rally_car_QP_solver_Lbyrd04);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_9_28_28(rally_car_QP_solver_V03, rally_car_QP_solver_Lbyrd03, rally_car_QP_solver_W04, rally_car_QP_solver_Lbyrd04, rally_car_QP_solver_beta03);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd03, rally_car_QP_solver_yy02, rally_car_QP_solver_beta03, rally_car_QP_solver_bmy03);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld03, rally_car_QP_solver_bmy03, rally_car_QP_solver_yy03);
rally_car_QP_solver_LA_VSUB6_INDEXED_28_12_28(rally_car_QP_solver_ccrhsub05, rally_car_QP_solver_sub05, rally_car_QP_solver_ubIdx05, rally_car_QP_solver_ccrhsl05, rally_car_QP_solver_slb05, rally_car_QP_solver_lbIdx05, rally_car_QP_solver_rd05);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi05, rally_car_QP_solver_rd05, rally_car_QP_solver_Lbyrd05);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_9_28_28(rally_car_QP_solver_V04, rally_car_QP_solver_Lbyrd04, rally_car_QP_solver_W05, rally_car_QP_solver_Lbyrd05, rally_car_QP_solver_beta04);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd04, rally_car_QP_solver_yy03, rally_car_QP_solver_beta04, rally_car_QP_solver_bmy04);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld04, rally_car_QP_solver_bmy04, rally_car_QP_solver_yy04);
rally_car_QP_solver_LA_VSUB6_INDEXED_28_12_28(rally_car_QP_solver_ccrhsub06, rally_car_QP_solver_sub06, rally_car_QP_solver_ubIdx06, rally_car_QP_solver_ccrhsl06, rally_car_QP_solver_slb06, rally_car_QP_solver_lbIdx06, rally_car_QP_solver_rd06);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi06, rally_car_QP_solver_rd06, rally_car_QP_solver_Lbyrd06);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_9_28_28(rally_car_QP_solver_V05, rally_car_QP_solver_Lbyrd05, rally_car_QP_solver_W06, rally_car_QP_solver_Lbyrd06, rally_car_QP_solver_beta05);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd05, rally_car_QP_solver_yy04, rally_car_QP_solver_beta05, rally_car_QP_solver_bmy05);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld05, rally_car_QP_solver_bmy05, rally_car_QP_solver_yy05);
rally_car_QP_solver_LA_VSUB6_INDEXED_28_12_28(rally_car_QP_solver_ccrhsub07, rally_car_QP_solver_sub07, rally_car_QP_solver_ubIdx07, rally_car_QP_solver_ccrhsl07, rally_car_QP_solver_slb07, rally_car_QP_solver_lbIdx07, rally_car_QP_solver_rd07);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi07, rally_car_QP_solver_rd07, rally_car_QP_solver_Lbyrd07);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_9_28_28(rally_car_QP_solver_V06, rally_car_QP_solver_Lbyrd06, rally_car_QP_solver_W07, rally_car_QP_solver_Lbyrd07, rally_car_QP_solver_beta06);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd06, rally_car_QP_solver_yy05, rally_car_QP_solver_beta06, rally_car_QP_solver_bmy06);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld06, rally_car_QP_solver_bmy06, rally_car_QP_solver_yy06);
rally_car_QP_solver_LA_VSUB6_INDEXED_28_12_28(rally_car_QP_solver_ccrhsub08, rally_car_QP_solver_sub08, rally_car_QP_solver_ubIdx08, rally_car_QP_solver_ccrhsl08, rally_car_QP_solver_slb08, rally_car_QP_solver_lbIdx08, rally_car_QP_solver_rd08);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi08, rally_car_QP_solver_rd08, rally_car_QP_solver_Lbyrd08);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_9_28_28(rally_car_QP_solver_V07, rally_car_QP_solver_Lbyrd07, rally_car_QP_solver_W08, rally_car_QP_solver_Lbyrd08, rally_car_QP_solver_beta07);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd07, rally_car_QP_solver_yy06, rally_car_QP_solver_beta07, rally_car_QP_solver_bmy07);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld07, rally_car_QP_solver_bmy07, rally_car_QP_solver_yy07);
rally_car_QP_solver_LA_VSUB6_INDEXED_28_12_28(rally_car_QP_solver_ccrhsub09, rally_car_QP_solver_sub09, rally_car_QP_solver_ubIdx09, rally_car_QP_solver_ccrhsl09, rally_car_QP_solver_slb09, rally_car_QP_solver_lbIdx09, rally_car_QP_solver_rd09);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi09, rally_car_QP_solver_rd09, rally_car_QP_solver_Lbyrd09);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_9_28_28(rally_car_QP_solver_V08, rally_car_QP_solver_Lbyrd08, rally_car_QP_solver_W09, rally_car_QP_solver_Lbyrd09, rally_car_QP_solver_beta08);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd08, rally_car_QP_solver_yy07, rally_car_QP_solver_beta08, rally_car_QP_solver_bmy08);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld08, rally_car_QP_solver_bmy08, rally_car_QP_solver_yy08);
rally_car_QP_solver_LA_VSUB6_INDEXED_28_12_28(rally_car_QP_solver_ccrhsub10, rally_car_QP_solver_sub10, rally_car_QP_solver_ubIdx10, rally_car_QP_solver_ccrhsl10, rally_car_QP_solver_slb10, rally_car_QP_solver_lbIdx10, rally_car_QP_solver_rd10);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_28(rally_car_QP_solver_Phi10, rally_car_QP_solver_rd10, rally_car_QP_solver_Lbyrd10);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_9_28_28(rally_car_QP_solver_V09, rally_car_QP_solver_Lbyrd09, rally_car_QP_solver_W10, rally_car_QP_solver_Lbyrd10, rally_car_QP_solver_beta09);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd09, rally_car_QP_solver_yy08, rally_car_QP_solver_beta09, rally_car_QP_solver_bmy09);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld09, rally_car_QP_solver_bmy09, rally_car_QP_solver_yy09);
rally_car_QP_solver_LA_VSUB6_INDEXED_9_9_9(rally_car_QP_solver_ccrhsub11, rally_car_QP_solver_sub11, rally_car_QP_solver_ubIdx11, rally_car_QP_solver_ccrhsl11, rally_car_QP_solver_slb11, rally_car_QP_solver_lbIdx11, rally_car_QP_solver_rd11);
rally_car_QP_solver_LA_DIAG_FORWARDSUB_9(rally_car_QP_solver_Phi11, rally_car_QP_solver_rd11, rally_car_QP_solver_Lbyrd11);
rally_car_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_9_28_9(rally_car_QP_solver_V10, rally_car_QP_solver_Lbyrd10, rally_car_QP_solver_W11, rally_car_QP_solver_Lbyrd11, rally_car_QP_solver_beta10);
rally_car_QP_solver_LA_DENSE_MVMSUB1_9_9(rally_car_QP_solver_Lsd10, rally_car_QP_solver_yy09, rally_car_QP_solver_beta10, rally_car_QP_solver_bmy10);
rally_car_QP_solver_LA_DENSE_FORWARDSUB_9(rally_car_QP_solver_Ld10, rally_car_QP_solver_bmy10, rally_car_QP_solver_yy10);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld10, rally_car_QP_solver_yy10, rally_car_QP_solver_dvcc10);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd10, rally_car_QP_solver_dvcc10, rally_car_QP_solver_yy09, rally_car_QP_solver_bmy09);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld09, rally_car_QP_solver_bmy09, rally_car_QP_solver_dvcc09);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd09, rally_car_QP_solver_dvcc09, rally_car_QP_solver_yy08, rally_car_QP_solver_bmy08);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld08, rally_car_QP_solver_bmy08, rally_car_QP_solver_dvcc08);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd08, rally_car_QP_solver_dvcc08, rally_car_QP_solver_yy07, rally_car_QP_solver_bmy07);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld07, rally_car_QP_solver_bmy07, rally_car_QP_solver_dvcc07);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd07, rally_car_QP_solver_dvcc07, rally_car_QP_solver_yy06, rally_car_QP_solver_bmy06);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld06, rally_car_QP_solver_bmy06, rally_car_QP_solver_dvcc06);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd06, rally_car_QP_solver_dvcc06, rally_car_QP_solver_yy05, rally_car_QP_solver_bmy05);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld05, rally_car_QP_solver_bmy05, rally_car_QP_solver_dvcc05);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd05, rally_car_QP_solver_dvcc05, rally_car_QP_solver_yy04, rally_car_QP_solver_bmy04);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld04, rally_car_QP_solver_bmy04, rally_car_QP_solver_dvcc04);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd04, rally_car_QP_solver_dvcc04, rally_car_QP_solver_yy03, rally_car_QP_solver_bmy03);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld03, rally_car_QP_solver_bmy03, rally_car_QP_solver_dvcc03);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd03, rally_car_QP_solver_dvcc03, rally_car_QP_solver_yy02, rally_car_QP_solver_bmy02);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld02, rally_car_QP_solver_bmy02, rally_car_QP_solver_dvcc02);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_9(rally_car_QP_solver_Lsd02, rally_car_QP_solver_dvcc02, rally_car_QP_solver_yy01, rally_car_QP_solver_bmy01);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_9(rally_car_QP_solver_Ld01, rally_car_QP_solver_bmy01, rally_car_QP_solver_dvcc01);
rally_car_QP_solver_LA_DENSE_MTVMSUB_9_17(rally_car_QP_solver_Lsd01, rally_car_QP_solver_dvcc01, rally_car_QP_solver_yy00, rally_car_QP_solver_bmy00);
rally_car_QP_solver_LA_DENSE_BACKWARDSUB_17(rally_car_QP_solver_Ld00, rally_car_QP_solver_bmy00, rally_car_QP_solver_dvcc00);
rally_car_QP_solver_LA_DENSE_MTVM_17_28(params->C1, rally_car_QP_solver_dvcc00, rally_car_QP_solver_grad_eq00);
rally_car_QP_solver_LA_DENSE_MTVM2_9_28_17(params->C2, rally_car_QP_solver_dvcc01, rally_car_QP_solver_D01, rally_car_QP_solver_dvcc00, rally_car_QP_solver_grad_eq01);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C3, rally_car_QP_solver_dvcc02, rally_car_QP_solver_D02, rally_car_QP_solver_dvcc01, rally_car_QP_solver_grad_eq02);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C4, rally_car_QP_solver_dvcc03, rally_car_QP_solver_D02, rally_car_QP_solver_dvcc02, rally_car_QP_solver_grad_eq03);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C5, rally_car_QP_solver_dvcc04, rally_car_QP_solver_D02, rally_car_QP_solver_dvcc03, rally_car_QP_solver_grad_eq04);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C6, rally_car_QP_solver_dvcc05, rally_car_QP_solver_D02, rally_car_QP_solver_dvcc04, rally_car_QP_solver_grad_eq05);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C7, rally_car_QP_solver_dvcc06, rally_car_QP_solver_D02, rally_car_QP_solver_dvcc05, rally_car_QP_solver_grad_eq06);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C8, rally_car_QP_solver_dvcc07, rally_car_QP_solver_D02, rally_car_QP_solver_dvcc06, rally_car_QP_solver_grad_eq07);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C9, rally_car_QP_solver_dvcc08, rally_car_QP_solver_D02, rally_car_QP_solver_dvcc07, rally_car_QP_solver_grad_eq08);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C10, rally_car_QP_solver_dvcc09, rally_car_QP_solver_D02, rally_car_QP_solver_dvcc08, rally_car_QP_solver_grad_eq09);
rally_car_QP_solver_LA_DENSE_DIAGZERO_MTVM2_9_28_9(params->C11, rally_car_QP_solver_dvcc10, rally_car_QP_solver_D02, rally_car_QP_solver_dvcc09, rally_car_QP_solver_grad_eq10);
rally_car_QP_solver_LA_DIAGZERO_MTVM_9_9(rally_car_QP_solver_D11, rally_car_QP_solver_dvcc10, rally_car_QP_solver_grad_eq11);
rally_car_QP_solver_LA_VSUB_317(rally_car_QP_solver_rd, rally_car_QP_solver_grad_eq, rally_car_QP_solver_rd);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi00, rally_car_QP_solver_rd00, rally_car_QP_solver_dzcc00);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi01, rally_car_QP_solver_rd01, rally_car_QP_solver_dzcc01);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi02, rally_car_QP_solver_rd02, rally_car_QP_solver_dzcc02);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi03, rally_car_QP_solver_rd03, rally_car_QP_solver_dzcc03);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi04, rally_car_QP_solver_rd04, rally_car_QP_solver_dzcc04);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi05, rally_car_QP_solver_rd05, rally_car_QP_solver_dzcc05);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi06, rally_car_QP_solver_rd06, rally_car_QP_solver_dzcc06);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi07, rally_car_QP_solver_rd07, rally_car_QP_solver_dzcc07);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi08, rally_car_QP_solver_rd08, rally_car_QP_solver_dzcc08);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi09, rally_car_QP_solver_rd09, rally_car_QP_solver_dzcc09);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_28(rally_car_QP_solver_Phi10, rally_car_QP_solver_rd10, rally_car_QP_solver_dzcc10);
rally_car_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_9(rally_car_QP_solver_Phi11, rally_car_QP_solver_rd11, rally_car_QP_solver_dzcc11);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_28(rally_car_QP_solver_ccrhsl00, rally_car_QP_solver_slb00, rally_car_QP_solver_llbbyslb00, rally_car_QP_solver_dzcc00, rally_car_QP_solver_lbIdx00, rally_car_QP_solver_dllbcc00);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_12(rally_car_QP_solver_ccrhsub00, rally_car_QP_solver_sub00, rally_car_QP_solver_lubbysub00, rally_car_QP_solver_dzcc00, rally_car_QP_solver_ubIdx00, rally_car_QP_solver_dlubcc00);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_28(rally_car_QP_solver_ccrhsl01, rally_car_QP_solver_slb01, rally_car_QP_solver_llbbyslb01, rally_car_QP_solver_dzcc01, rally_car_QP_solver_lbIdx01, rally_car_QP_solver_dllbcc01);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_12(rally_car_QP_solver_ccrhsub01, rally_car_QP_solver_sub01, rally_car_QP_solver_lubbysub01, rally_car_QP_solver_dzcc01, rally_car_QP_solver_ubIdx01, rally_car_QP_solver_dlubcc01);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_28(rally_car_QP_solver_ccrhsl02, rally_car_QP_solver_slb02, rally_car_QP_solver_llbbyslb02, rally_car_QP_solver_dzcc02, rally_car_QP_solver_lbIdx02, rally_car_QP_solver_dllbcc02);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_12(rally_car_QP_solver_ccrhsub02, rally_car_QP_solver_sub02, rally_car_QP_solver_lubbysub02, rally_car_QP_solver_dzcc02, rally_car_QP_solver_ubIdx02, rally_car_QP_solver_dlubcc02);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_28(rally_car_QP_solver_ccrhsl03, rally_car_QP_solver_slb03, rally_car_QP_solver_llbbyslb03, rally_car_QP_solver_dzcc03, rally_car_QP_solver_lbIdx03, rally_car_QP_solver_dllbcc03);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_12(rally_car_QP_solver_ccrhsub03, rally_car_QP_solver_sub03, rally_car_QP_solver_lubbysub03, rally_car_QP_solver_dzcc03, rally_car_QP_solver_ubIdx03, rally_car_QP_solver_dlubcc03);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_28(rally_car_QP_solver_ccrhsl04, rally_car_QP_solver_slb04, rally_car_QP_solver_llbbyslb04, rally_car_QP_solver_dzcc04, rally_car_QP_solver_lbIdx04, rally_car_QP_solver_dllbcc04);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_12(rally_car_QP_solver_ccrhsub04, rally_car_QP_solver_sub04, rally_car_QP_solver_lubbysub04, rally_car_QP_solver_dzcc04, rally_car_QP_solver_ubIdx04, rally_car_QP_solver_dlubcc04);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_28(rally_car_QP_solver_ccrhsl05, rally_car_QP_solver_slb05, rally_car_QP_solver_llbbyslb05, rally_car_QP_solver_dzcc05, rally_car_QP_solver_lbIdx05, rally_car_QP_solver_dllbcc05);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_12(rally_car_QP_solver_ccrhsub05, rally_car_QP_solver_sub05, rally_car_QP_solver_lubbysub05, rally_car_QP_solver_dzcc05, rally_car_QP_solver_ubIdx05, rally_car_QP_solver_dlubcc05);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_28(rally_car_QP_solver_ccrhsl06, rally_car_QP_solver_slb06, rally_car_QP_solver_llbbyslb06, rally_car_QP_solver_dzcc06, rally_car_QP_solver_lbIdx06, rally_car_QP_solver_dllbcc06);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_12(rally_car_QP_solver_ccrhsub06, rally_car_QP_solver_sub06, rally_car_QP_solver_lubbysub06, rally_car_QP_solver_dzcc06, rally_car_QP_solver_ubIdx06, rally_car_QP_solver_dlubcc06);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_28(rally_car_QP_solver_ccrhsl07, rally_car_QP_solver_slb07, rally_car_QP_solver_llbbyslb07, rally_car_QP_solver_dzcc07, rally_car_QP_solver_lbIdx07, rally_car_QP_solver_dllbcc07);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_12(rally_car_QP_solver_ccrhsub07, rally_car_QP_solver_sub07, rally_car_QP_solver_lubbysub07, rally_car_QP_solver_dzcc07, rally_car_QP_solver_ubIdx07, rally_car_QP_solver_dlubcc07);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_28(rally_car_QP_solver_ccrhsl08, rally_car_QP_solver_slb08, rally_car_QP_solver_llbbyslb08, rally_car_QP_solver_dzcc08, rally_car_QP_solver_lbIdx08, rally_car_QP_solver_dllbcc08);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_12(rally_car_QP_solver_ccrhsub08, rally_car_QP_solver_sub08, rally_car_QP_solver_lubbysub08, rally_car_QP_solver_dzcc08, rally_car_QP_solver_ubIdx08, rally_car_QP_solver_dlubcc08);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_28(rally_car_QP_solver_ccrhsl09, rally_car_QP_solver_slb09, rally_car_QP_solver_llbbyslb09, rally_car_QP_solver_dzcc09, rally_car_QP_solver_lbIdx09, rally_car_QP_solver_dllbcc09);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_12(rally_car_QP_solver_ccrhsub09, rally_car_QP_solver_sub09, rally_car_QP_solver_lubbysub09, rally_car_QP_solver_dzcc09, rally_car_QP_solver_ubIdx09, rally_car_QP_solver_dlubcc09);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_28(rally_car_QP_solver_ccrhsl10, rally_car_QP_solver_slb10, rally_car_QP_solver_llbbyslb10, rally_car_QP_solver_dzcc10, rally_car_QP_solver_lbIdx10, rally_car_QP_solver_dllbcc10);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_12(rally_car_QP_solver_ccrhsub10, rally_car_QP_solver_sub10, rally_car_QP_solver_lubbysub10, rally_car_QP_solver_dzcc10, rally_car_QP_solver_ubIdx10, rally_car_QP_solver_dlubcc10);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_9(rally_car_QP_solver_ccrhsl11, rally_car_QP_solver_slb11, rally_car_QP_solver_llbbyslb11, rally_car_QP_solver_dzcc11, rally_car_QP_solver_lbIdx11, rally_car_QP_solver_dllbcc11);
rally_car_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_9(rally_car_QP_solver_ccrhsub11, rally_car_QP_solver_sub11, rally_car_QP_solver_lubbysub11, rally_car_QP_solver_dzcc11, rally_car_QP_solver_ubIdx11, rally_car_QP_solver_dlubcc11);
rally_car_QP_solver_LA_VSUB7_458(rally_car_QP_solver_l, rally_car_QP_solver_ccrhs, rally_car_QP_solver_s, rally_car_QP_solver_dl_cc, rally_car_QP_solver_ds_cc);
rally_car_QP_solver_LA_VADD_317(rally_car_QP_solver_dz_cc, rally_car_QP_solver_dz_aff);
rally_car_QP_solver_LA_VADD_107(rally_car_QP_solver_dv_cc, rally_car_QP_solver_dv_aff);
rally_car_QP_solver_LA_VADD_458(rally_car_QP_solver_dl_cc, rally_car_QP_solver_dl_aff);
rally_car_QP_solver_LA_VADD_458(rally_car_QP_solver_ds_cc, rally_car_QP_solver_ds_aff);
info->lsit_cc = rally_car_QP_solver_LINESEARCH_BACKTRACKING_COMBINED(rally_car_QP_solver_z, rally_car_QP_solver_v, rally_car_QP_solver_l, rally_car_QP_solver_s, rally_car_QP_solver_dz_cc, rally_car_QP_solver_dv_cc, rally_car_QP_solver_dl_cc, rally_car_QP_solver_ds_cc, &info->step_cc, &info->mu);
if( info->lsit_cc == rally_car_QP_solver_NOPROGRESS ){
exitcode = rally_car_QP_solver_NOPROGRESS; break;
}
info->it++;
}
output->z1[0] = rally_car_QP_solver_z00[0];
output->z1[1] = rally_car_QP_solver_z00[1];
output->z1[2] = rally_car_QP_solver_z00[2];
output->z1[3] = rally_car_QP_solver_z00[3];
output->z1[4] = rally_car_QP_solver_z00[4];
output->z1[5] = rally_car_QP_solver_z00[5];
output->z1[6] = rally_car_QP_solver_z00[6];
output->z1[7] = rally_car_QP_solver_z00[7];
output->z1[8] = rally_car_QP_solver_z00[8];
output->z1[9] = rally_car_QP_solver_z00[9];
output->z1[10] = rally_car_QP_solver_z00[10];
output->z1[11] = rally_car_QP_solver_z00[11];
output->z2[0] = rally_car_QP_solver_z01[0];
output->z2[1] = rally_car_QP_solver_z01[1];
output->z2[2] = rally_car_QP_solver_z01[2];
output->z2[3] = rally_car_QP_solver_z01[3];
output->z2[4] = rally_car_QP_solver_z01[4];
output->z2[5] = rally_car_QP_solver_z01[5];
output->z2[6] = rally_car_QP_solver_z01[6];
output->z2[7] = rally_car_QP_solver_z01[7];
output->z2[8] = rally_car_QP_solver_z01[8];
output->z2[9] = rally_car_QP_solver_z01[9];
output->z2[10] = rally_car_QP_solver_z01[10];
output->z2[11] = rally_car_QP_solver_z01[11];
output->z3[0] = rally_car_QP_solver_z02[0];
output->z3[1] = rally_car_QP_solver_z02[1];
output->z3[2] = rally_car_QP_solver_z02[2];
output->z3[3] = rally_car_QP_solver_z02[3];
output->z3[4] = rally_car_QP_solver_z02[4];
output->z3[5] = rally_car_QP_solver_z02[5];
output->z3[6] = rally_car_QP_solver_z02[6];
output->z3[7] = rally_car_QP_solver_z02[7];
output->z3[8] = rally_car_QP_solver_z02[8];
output->z3[9] = rally_car_QP_solver_z02[9];
output->z3[10] = rally_car_QP_solver_z02[10];
output->z3[11] = rally_car_QP_solver_z02[11];
output->z4[0] = rally_car_QP_solver_z03[0];
output->z4[1] = rally_car_QP_solver_z03[1];
output->z4[2] = rally_car_QP_solver_z03[2];
output->z4[3] = rally_car_QP_solver_z03[3];
output->z4[4] = rally_car_QP_solver_z03[4];
output->z4[5] = rally_car_QP_solver_z03[5];
output->z4[6] = rally_car_QP_solver_z03[6];
output->z4[7] = rally_car_QP_solver_z03[7];
output->z4[8] = rally_car_QP_solver_z03[8];
output->z4[9] = rally_car_QP_solver_z03[9];
output->z4[10] = rally_car_QP_solver_z03[10];
output->z4[11] = rally_car_QP_solver_z03[11];
output->z5[0] = rally_car_QP_solver_z04[0];
output->z5[1] = rally_car_QP_solver_z04[1];
output->z5[2] = rally_car_QP_solver_z04[2];
output->z5[3] = rally_car_QP_solver_z04[3];
output->z5[4] = rally_car_QP_solver_z04[4];
output->z5[5] = rally_car_QP_solver_z04[5];
output->z5[6] = rally_car_QP_solver_z04[6];
output->z5[7] = rally_car_QP_solver_z04[7];
output->z5[8] = rally_car_QP_solver_z04[8];
output->z5[9] = rally_car_QP_solver_z04[9];
output->z5[10] = rally_car_QP_solver_z04[10];
output->z5[11] = rally_car_QP_solver_z04[11];
output->z6[0] = rally_car_QP_solver_z05[0];
output->z6[1] = rally_car_QP_solver_z05[1];
output->z6[2] = rally_car_QP_solver_z05[2];
output->z6[3] = rally_car_QP_solver_z05[3];
output->z6[4] = rally_car_QP_solver_z05[4];
output->z6[5] = rally_car_QP_solver_z05[5];
output->z6[6] = rally_car_QP_solver_z05[6];
output->z6[7] = rally_car_QP_solver_z05[7];
output->z6[8] = rally_car_QP_solver_z05[8];
output->z6[9] = rally_car_QP_solver_z05[9];
output->z6[10] = rally_car_QP_solver_z05[10];
output->z6[11] = rally_car_QP_solver_z05[11];
output->z7[0] = rally_car_QP_solver_z06[0];
output->z7[1] = rally_car_QP_solver_z06[1];
output->z7[2] = rally_car_QP_solver_z06[2];
output->z7[3] = rally_car_QP_solver_z06[3];
output->z7[4] = rally_car_QP_solver_z06[4];
output->z7[5] = rally_car_QP_solver_z06[5];
output->z7[6] = rally_car_QP_solver_z06[6];
output->z7[7] = rally_car_QP_solver_z06[7];
output->z7[8] = rally_car_QP_solver_z06[8];
output->z7[9] = rally_car_QP_solver_z06[9];
output->z7[10] = rally_car_QP_solver_z06[10];
output->z7[11] = rally_car_QP_solver_z06[11];
output->z8[0] = rally_car_QP_solver_z07[0];
output->z8[1] = rally_car_QP_solver_z07[1];
output->z8[2] = rally_car_QP_solver_z07[2];
output->z8[3] = rally_car_QP_solver_z07[3];
output->z8[4] = rally_car_QP_solver_z07[4];
output->z8[5] = rally_car_QP_solver_z07[5];
output->z8[6] = rally_car_QP_solver_z07[6];
output->z8[7] = rally_car_QP_solver_z07[7];
output->z8[8] = rally_car_QP_solver_z07[8];
output->z8[9] = rally_car_QP_solver_z07[9];
output->z8[10] = rally_car_QP_solver_z07[10];
output->z8[11] = rally_car_QP_solver_z07[11];
output->z9[0] = rally_car_QP_solver_z08[0];
output->z9[1] = rally_car_QP_solver_z08[1];
output->z9[2] = rally_car_QP_solver_z08[2];
output->z9[3] = rally_car_QP_solver_z08[3];
output->z9[4] = rally_car_QP_solver_z08[4];
output->z9[5] = rally_car_QP_solver_z08[5];
output->z9[6] = rally_car_QP_solver_z08[6];
output->z9[7] = rally_car_QP_solver_z08[7];
output->z9[8] = rally_car_QP_solver_z08[8];
output->z9[9] = rally_car_QP_solver_z08[9];
output->z9[10] = rally_car_QP_solver_z08[10];
output->z9[11] = rally_car_QP_solver_z08[11];
output->z10[0] = rally_car_QP_solver_z09[0];
output->z10[1] = rally_car_QP_solver_z09[1];
output->z10[2] = rally_car_QP_solver_z09[2];
output->z10[3] = rally_car_QP_solver_z09[3];
output->z10[4] = rally_car_QP_solver_z09[4];
output->z10[5] = rally_car_QP_solver_z09[5];
output->z10[6] = rally_car_QP_solver_z09[6];
output->z10[7] = rally_car_QP_solver_z09[7];
output->z10[8] = rally_car_QP_solver_z09[8];
output->z10[9] = rally_car_QP_solver_z09[9];
output->z10[10] = rally_car_QP_solver_z09[10];
output->z10[11] = rally_car_QP_solver_z09[11];
output->z11[0] = rally_car_QP_solver_z10[0];
output->z11[1] = rally_car_QP_solver_z10[1];
output->z11[2] = rally_car_QP_solver_z10[2];
output->z11[3] = rally_car_QP_solver_z10[3];
output->z11[4] = rally_car_QP_solver_z10[4];
output->z11[5] = rally_car_QP_solver_z10[5];
output->z11[6] = rally_car_QP_solver_z10[6];
output->z11[7] = rally_car_QP_solver_z10[7];
output->z11[8] = rally_car_QP_solver_z10[8];
output->z11[9] = rally_car_QP_solver_z10[9];
output->z11[10] = rally_car_QP_solver_z10[10];
output->z11[11] = rally_car_QP_solver_z10[11];
output->z12[0] = rally_car_QP_solver_z11[0];
output->z12[1] = rally_car_QP_solver_z11[1];
output->z12[2] = rally_car_QP_solver_z11[2];
output->z12[3] = rally_car_QP_solver_z11[3];
output->z12[4] = rally_car_QP_solver_z11[4];
output->z12[5] = rally_car_QP_solver_z11[5];
output->z12[6] = rally_car_QP_solver_z11[6];
output->z12[7] = rally_car_QP_solver_z11[7];
output->z12[8] = rally_car_QP_solver_z11[8];

#if rally_car_QP_solver_SET_TIMING == 1
info->solvetime = rally_car_QP_solver_toc(&solvertimer);
#if rally_car_QP_solver_SET_PRINTLEVEL > 0 && rally_car_QP_solver_SET_TIMING == 1
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
