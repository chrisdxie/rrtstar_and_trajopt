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

#include "../include/cartpole_QP_solver.h"

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

typedef struct cartpole_QP_solver_timer{
	LARGE_INTEGER tic;
	LARGE_INTEGER toc;
	LARGE_INTEGER freq;
} cartpole_QP_solver_timer;


void cartpole_QP_solver_tic(cartpole_QP_solver_timer* t)
{
	QueryPerformanceFrequency(&t->freq);
	QueryPerformanceCounter(&t->tic);
}



cartpole_QP_solver_FLOAT cartpole_QP_solver_toc(cartpole_QP_solver_timer* t)
{
	QueryPerformanceCounter(&t->toc);
	return ((t->toc.QuadPart - t->tic.QuadPart) / (cartpole_QP_solver_FLOAT)t->freq.QuadPart);
}


/* WE ARE ON THE MAC */
#elif (defined __APPLE__)
#include <mach/mach_time.h>


/* Use MAC OSX  mach_time for timing */
typedef struct cartpole_QP_solver_timer{
	uint64_t tic;
	uint64_t toc;
	mach_timebase_info_data_t tinfo;

} cartpole_QP_solver_timer;


void cartpole_QP_solver_tic(cartpole_QP_solver_timer* t)
{
    /* read current clock cycles */
    t->tic = mach_absolute_time();
}



cartpole_QP_solver_FLOAT cartpole_QP_solver_toc(cartpole_QP_solver_timer* t)
{
    uint64_t duration; /* elapsed time in clock cycles*/
    t->toc = mach_absolute_time();
	duration = t->toc - t->tic;

    /*conversion from clock cycles to nanoseconds*/
    mach_timebase_info(&(t->tinfo));
    duration *= t->tinfo.numer;
    duration /= t->tinfo.denom;

    return (cartpole_QP_solver_FLOAT)duration / 1000000000;
}

/* WE ARE ON SOME TEXAS INSTRUMENTS PLATFORM */
#elif (defined __TI_COMPILER_VERSION__)

/* TimeStamps */
#include <c6x.h> /* make use of TSCL, TSCH */


typedef struct cartpole_QP_solver_timer{
	unsigned long long tic;
	unsigned long long toc;
} cartpole_QP_solver_timer;


void cartpole_QP_solver_tic(cartpole_QP_solver_timer* t)
{
	TSCL = 0;	/* Initiate CPU timer by writing any val to TSCL */
	t->tic = _itoll( TSCH, TSCL );
}



cartpole_QP_solver_FLOAT cartpole_QP_solver_toc(cartpole_QP_solver_timer* t)
{
	t->toc = _itoll( TSCH, TSCL );
	unsigned long long t0;
	unsigned long long overhead;
	t0 = _itoll( TSCH, TSCL );
	overhead = _itoll( TSCH, TSCL )  - t0;

	return (cartpole_QP_solver_FLOAT)(t->toc - t->tic - overhead) / 1000000000;
}



/* WE ARE ON SOME OTHER UNIX/LINUX SYSTEM */
#else

/* Use POSIX clocl_gettime() for timing on non-Windows machines */
#include <time.h>
typedef struct cartpole_QP_solver_timer{
	struct timespec tic;
	struct timespec toc;
} cartpole_QP_solver_timer;


/* read current time */
void cartpole_QP_solver_tic(cartpole_QP_solver_timer* t)
{
	clock_gettime(CLOCK_MONOTONIC, &t->tic);
}



/* return time passed since last call to tic on this timer */
double cartpole_QP_solver_toc(cartpole_QP_solver_timer* t)
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

	return (cartpole_QP_solver_FLOAT)temp.tv_sec + (cartpole_QP_solver_FLOAT)temp.tv_nsec / 1000000000;
}


#endif

/* LINEAR ALGEBRA LIBRARY ---------------------------------------------- */
/*
 * Initializes a vector of length 103 with a value.
 */
void cartpole_QP_solver_LA_INITIALIZEVECTOR_103(cartpole_QP_solver_FLOAT* vec, cartpole_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<103; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 39 with a value.
 */
void cartpole_QP_solver_LA_INITIALIZEVECTOR_39(cartpole_QP_solver_FLOAT* vec, cartpole_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<39; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 150 with a value.
 */
void cartpole_QP_solver_LA_INITIALIZEVECTOR_150(cartpole_QP_solver_FLOAT* vec, cartpole_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<150; i++ )
	{
		vec[i] = value;
	}
}


/* 
 * Calculates a dot product and adds it to a variable: z += x'*y; 
 * This function is for vectors of length 150.
 */
void cartpole_QP_solver_LA_DOTACC_150(cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<150; i++ ){
		*z += x[i]*y[i];
	}
}


/*
 * Calculates the gradient and the value for a quadratic function 0.5*z'*H*z + f'*z
 *
 * INPUTS:     H  - Symmetric Hessian, diag matrix of size [14 x 14]
 *             f  - column vector of size 14
 *             z  - column vector of size 14
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 14
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void cartpole_QP_solver_LA_DIAG_QUADFCN_14(cartpole_QP_solver_FLOAT* H, cartpole_QP_solver_FLOAT* f, cartpole_QP_solver_FLOAT* z, cartpole_QP_solver_FLOAT* grad, cartpole_QP_solver_FLOAT* value)
{
	int i;
	cartpole_QP_solver_FLOAT hz;	
	for( i=0; i<14; i++){
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
void cartpole_QP_solver_LA_DIAG_QUADFCN_5(cartpole_QP_solver_FLOAT* H, cartpole_QP_solver_FLOAT* f, cartpole_QP_solver_FLOAT* z, cartpole_QP_solver_FLOAT* grad, cartpole_QP_solver_FLOAT* value)
{
	int i;
	cartpole_QP_solver_FLOAT hz;	
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
void cartpole_QP_solver_LA_DENSE_MVMSUB3_9_14_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *l, cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *z, cartpole_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;
	cartpole_QP_solver_FLOAT AxBu[9];
	cartpole_QP_solver_FLOAT norm = *y;
	cartpole_QP_solver_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<9; i++ ){
		AxBu[i] = A[k++]*x[0] + B[m++]*u[0];
	}	
	for( j=1; j<14; j++ ){		
		for( i=0; i<9; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<14; n++ ){
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
void cartpole_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *l, cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *z, cartpole_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	cartpole_QP_solver_FLOAT AxBu[5];
	cartpole_QP_solver_FLOAT norm = *y;
	cartpole_QP_solver_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<5; i++ ){
		AxBu[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<14; j++ ){		
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
void cartpole_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_5(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *l, cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *z, cartpole_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	cartpole_QP_solver_FLOAT AxBu[5];
	cartpole_QP_solver_FLOAT norm = *y;
	cartpole_QP_solver_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<5; i++ ){
		AxBu[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<14; j++ ){		
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
 * Matrix vector multiplication y = M'*x where M is of size [9 x 14]
 * and stored in column major format. Note the transpose of M!
 */
void cartpole_QP_solver_LA_DENSE_MTVM_9_14(cartpole_QP_solver_FLOAT *M, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<14; i++ ){
		y[i] = 0;
		for( j=0; j<9; j++ ){
			y[i] += M[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [5 x 14]
 * and B is of size [9 x 14]
 * and stored in column major format. Note the transposes of A and B!
 */
void cartpole_QP_solver_LA_DENSE_MTVM2_5_14_9(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	int n;
	int m = 0;
	for( i=0; i<14; i++ ){
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
 * where A is of size [5 x 14] and stored in column major format.
 * and B is of size [5 x 14] and stored in diagzero format
 * Note the transposes of A and B!
 */
void cartpole_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *z)
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
	for( i=5 ;i<14; i++ ){
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
void cartpole_QP_solver_LA_DIAGZERO_MTVM_5_5(cartpole_QP_solver_FLOAT *M, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y)
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
 * for vectors of length 14. Output z is of course scalar.
 */
void cartpole_QP_solver_LA_VSUBADD3_14(cartpole_QP_solver_FLOAT* t, cartpole_QP_solver_FLOAT* u, int* uidx, cartpole_QP_solver_FLOAT* v, cartpole_QP_solver_FLOAT* w, cartpole_QP_solver_FLOAT* y, cartpole_QP_solver_FLOAT* z, cartpole_QP_solver_FLOAT* r)
{
	int i;
	cartpole_QP_solver_FLOAT norm = *r;
	cartpole_QP_solver_FLOAT vx = 0;
	cartpole_QP_solver_FLOAT x;
	for( i=0; i<14; i++){
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
 * for vectors of length 6. Output z is of course scalar.
 */
void cartpole_QP_solver_LA_VSUBADD2_6(cartpole_QP_solver_FLOAT* t, int* tidx, cartpole_QP_solver_FLOAT* u, cartpole_QP_solver_FLOAT* v, cartpole_QP_solver_FLOAT* w, cartpole_QP_solver_FLOAT* y, cartpole_QP_solver_FLOAT* z, cartpole_QP_solver_FLOAT* r)
{
	int i;
	cartpole_QP_solver_FLOAT norm = *r;
	cartpole_QP_solver_FLOAT vx = 0;
	cartpole_QP_solver_FLOAT x;
	for( i=0; i<6; i++){
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
void cartpole_QP_solver_LA_VSUBADD3_5(cartpole_QP_solver_FLOAT* t, cartpole_QP_solver_FLOAT* u, int* uidx, cartpole_QP_solver_FLOAT* v, cartpole_QP_solver_FLOAT* w, cartpole_QP_solver_FLOAT* y, cartpole_QP_solver_FLOAT* z, cartpole_QP_solver_FLOAT* r)
{
	int i;
	cartpole_QP_solver_FLOAT norm = *r;
	cartpole_QP_solver_FLOAT vx = 0;
	cartpole_QP_solver_FLOAT x;
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
void cartpole_QP_solver_LA_VSUBADD2_5(cartpole_QP_solver_FLOAT* t, int* tidx, cartpole_QP_solver_FLOAT* u, cartpole_QP_solver_FLOAT* v, cartpole_QP_solver_FLOAT* w, cartpole_QP_solver_FLOAT* y, cartpole_QP_solver_FLOAT* z, cartpole_QP_solver_FLOAT* r)
{
	int i;
	cartpole_QP_solver_FLOAT norm = *r;
	cartpole_QP_solver_FLOAT vx = 0;
	cartpole_QP_solver_FLOAT x;
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
 * Special function for box constraints of length 14
 * Returns also L/S, a value that is often used elsewhere.
 */
void cartpole_QP_solver_LA_INEQ_B_GRAD_14_14_6(cartpole_QP_solver_FLOAT *lu, cartpole_QP_solver_FLOAT *su, cartpole_QP_solver_FLOAT *ru, cartpole_QP_solver_FLOAT *ll, cartpole_QP_solver_FLOAT *sl, cartpole_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, cartpole_QP_solver_FLOAT *grad, cartpole_QP_solver_FLOAT *lubysu, cartpole_QP_solver_FLOAT *llbysl)
{
	int i;
	for( i=0; i<14; i++ ){
		grad[i] = 0;
	}
	for( i=0; i<14; i++ ){		
		llbysl[i] = ll[i] / sl[i];
		grad[lbIdx[i]] -= llbysl[i]*rl[i];
	}
	for( i=0; i<6; i++ ){
		lubysu[i] = lu[i] / su[i];
		grad[ubIdx[i]] += lubysu[i]*ru[i];
	}
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 5
 * Returns also L/S, a value that is often used elsewhere.
 */
void cartpole_QP_solver_LA_INEQ_B_GRAD_5_5_5(cartpole_QP_solver_FLOAT *lu, cartpole_QP_solver_FLOAT *su, cartpole_QP_solver_FLOAT *ru, cartpole_QP_solver_FLOAT *ll, cartpole_QP_solver_FLOAT *sl, cartpole_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, cartpole_QP_solver_FLOAT *grad, cartpole_QP_solver_FLOAT *lubysu, cartpole_QP_solver_FLOAT *llbysl)
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
 * of length 103.
 */
void cartpole_QP_solver_LA_VVADD3_103(cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *v, cartpole_QP_solver_FLOAT *w, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<103; i++ ){
		z[i] = u[i] + v[i] + w[i];
	}
}


/*
 * Special function to compute the diagonal cholesky factorization of the 
 * positive definite augmented Hessian for block size 14.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = sqrt(H + diag(llbysl) + diag(lubysu))
 * where Phi is stored in diagonal storage format
 */
void cartpole_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(cartpole_QP_solver_FLOAT *H, cartpole_QP_solver_FLOAT *llbysl, int* lbIdx, cartpole_QP_solver_FLOAT *lubysu, int* ubIdx, cartpole_QP_solver_FLOAT *Phi)


{
	int i;
	
	/* copy  H into PHI */
	for( i=0; i<14; i++ ){
		Phi[i] = H[i];
	}

	/* add llbysl onto Phi where necessary */
	for( i=0; i<14; i++ ){
		Phi[lbIdx[i]] += llbysl[i];
	}

	/* add lubysu onto Phi where necessary */
	for( i=0; i<6; i++){
		Phi[ubIdx[i]] +=  lubysu[i];
	}
	
	/* compute cholesky */
	for(i=0; i<14; i++)
	{
#if cartpole_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
 * where A is to be computed and is of size [9 x 14],
 * B is given and of size [9 x 14], L is a diagonal
 * matrix of size 9 stored in diagonal matrix 
 * storage format. Note the transpose of L has no impact!
 *
 * Result: A in column major storage format.
 *
 */
void cartpole_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_14(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *A)
{
    int i,j;
	 int k = 0;

	for( j=0; j<14; j++){
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
 * The dimensions involved are 14.
 */
void cartpole_QP_solver_LA_DIAG_FORWARDSUB_14(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *y)
{
    int i;

    for( i=0; i<14; i++ ){
		y[i] = b[i]/L[i];
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [5 x 14],
 * B is given and of size [5 x 14], L is a diagonal
 * matrix of size 5 stored in diagonal matrix 
 * storage format. Note the transpose of L has no impact!
 *
 * Result: A in column major storage format.
 *
 */
void cartpole_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_14(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *A)
{
    int i,j;
	 int k = 0;

	for( j=0; j<14; j++){
		for( i=0; i<5; i++){
			A[k] = B[k]/L[j];
			k++;
		}
	}

}


/**
 * Compute C = A*B' where 
 *
 *	size(A) = [9 x 14]
 *  size(B) = [5 x 14]
 * 
 * and all matrices are stored in column major format.
 *
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE.  
 * 
 */
void cartpole_QP_solver_LA_DENSE_MMTM_9_14_5(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *C)
{
    int i, j, k;
    cartpole_QP_solver_FLOAT temp;
    
    for( i=0; i<9; i++ ){        
        for( j=0; j<5; j++ ){
            temp = 0; 
            for( k=0; k<14; k++ ){
                temp += A[k*9+i]*B[k*5+j];
            }						
            C[j*9+i] = temp;
        }
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [5 x 14],
 * B is given and of size [5 x 14], L is a diagonal
 *  matrix of size 14 stored in diagonal 
 * storage format. Note the transpose of L!
 *
 * Result: A in diagonalzero storage format.
 *
 */
void cartpole_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_14(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *A)
{
	int j;
    for( j=0; j<14; j++ ){   
		A[j] = B[j]/L[j];
     }
}


/**
 * Compute C = A*B' where 
 *
 *	size(A) = [5 x 14]
 *  size(B) = [5 x 14] in diagzero format
 * 
 * A and C matrices are stored in column major format.
 * 
 * 
 */
void cartpole_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_14_5(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *C)
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
void cartpole_QP_solver_LA_DIAG_CHOL_ONELOOP_LBUB_5_5_5(cartpole_QP_solver_FLOAT *H, cartpole_QP_solver_FLOAT *llbysl, int* lbIdx, cartpole_QP_solver_FLOAT *lubysu, int* ubIdx, cartpole_QP_solver_FLOAT *Phi)


{
	int i;
	
	/* compute cholesky */
	for( i=0; i<5; i++ ){
		Phi[i] = H[i] + llbysl[i] + lubysu[i];

#if cartpole_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
void cartpole_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_5(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *A)
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
void cartpole_QP_solver_LA_DIAG_FORWARDSUB_5(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *y)
{
    int i;

    for( i=0; i<5; i++ ){
		y[i] = b[i]/L[i];
    }
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [9 x 14] in column
 * storage format, and B is of size [9 x 14] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void cartpole_QP_solver_LA_DENSE_MMT2_9_14_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    cartpole_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<9; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<14; k++ ){
                ltemp += A[k*9+i]*A[k*9+j];
            }			
			for( k=0; k<14; k++ ){
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
void cartpole_QP_solver_LA_DENSE_MVMSUB2_9_14_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<9; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<14; j++ ){		
		for( i=0; i<9; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<14; n++ ){
		for( i=0; i<9; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [5 x 14] in column
 * storage format, and B is of size [5 x 14] diagonalzero
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void cartpole_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    cartpole_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<5; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<14; k++ ){
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
void cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<5; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[i]*u[i];
	}	

	for( j=1; j<14; j++ ){		
		for( i=0; i<5; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [5 x 14] in column
 * storage format, and B is of size [5 x 5] diagonalzero
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void cartpole_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_5(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    cartpole_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<5; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<14; k++ ){
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
void cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_5(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<5; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[i]*u[i];
	}	

	for( j=1; j<14; j++ ){		
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
void cartpole_QP_solver_LA_DENSE_CHOL_9(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    cartpole_QP_solver_FLOAT l;
    cartpole_QP_solver_FLOAT Mii;

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
        
#if cartpole_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
void cartpole_QP_solver_LA_DENSE_FORWARDSUB_9(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    cartpole_QP_solver_FLOAT yel;
            
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
void cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_9(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *A)
{
    int i,j,k,ii,di;
    cartpole_QP_solver_FLOAT a;
    
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
void cartpole_QP_solver_LA_DENSE_MMTSUB_5_9(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    cartpole_QP_solver_FLOAT ltemp;
    
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
void cartpole_QP_solver_LA_DENSE_CHOL_5(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    cartpole_QP_solver_FLOAT l;
    cartpole_QP_solver_FLOAT Mii;

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
        
#if cartpole_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
void cartpole_QP_solver_LA_DENSE_MVMSUB1_5_9(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *r)
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
void cartpole_QP_solver_LA_DENSE_FORWARDSUB_5(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    cartpole_QP_solver_FLOAT yel;
            
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
void cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *A)
{
    int i,j,k,ii,di;
    cartpole_QP_solver_FLOAT a;
    
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
void cartpole_QP_solver_LA_DENSE_MMTSUB_5_5(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    cartpole_QP_solver_FLOAT ltemp;
    
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
void cartpole_QP_solver_LA_DENSE_MVMSUB1_5_5(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *r)
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
void cartpole_QP_solver_LA_DENSE_BACKWARDSUB_5(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    cartpole_QP_solver_FLOAT xel;    
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
void cartpole_QP_solver_LA_DENSE_MTVMSUB_5_5(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *r)
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
void cartpole_QP_solver_LA_DENSE_MTVMSUB_5_9(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *r)
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
void cartpole_QP_solver_LA_DENSE_BACKWARDSUB_9(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    cartpole_QP_solver_FLOAT xel;    
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
 * Vector subtraction z = -x - y for vectors of length 103.
 */
void cartpole_QP_solver_LA_VSUB2_103(cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<103; i++){
		z[i] = -x[i] - y[i];
	}
}


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * diagonal matrix of size 14 in vector
 * storage format.
 */
void cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *x)
{
    int i;
            
    /* solve Ly = b by forward and backward substitution */
    for( i=0; i<14; i++ ){
		x[i] = b[i]/(L[i]*L[i]);
    }
    
}


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * diagonal matrix of size 5 in vector
 * storage format.
 */
void cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_5(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *x)
{
    int i;
            
    /* solve Ly = b by forward and backward substitution */
    for( i=0; i<5; i++ ){
		x[i] = b[i]/(L[i]*L[i]);
    }
    
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 14,
 * and x has length 14 and is indexed through yidx.
 */
void cartpole_QP_solver_LA_VSUB_INDEXED_14(cartpole_QP_solver_FLOAT *x, int* xidx, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<14; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 14.
 */
void cartpole_QP_solver_LA_VSUB3_14(cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *v, cartpole_QP_solver_FLOAT *w, cartpole_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<14; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = -x - y(yidx) where y is of length 14
 * and z, x and yidx are of length 6.
 */
void cartpole_QP_solver_LA_VSUB2_INDEXED_6(cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y, int* yidx, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<6; i++){
		z[i] = -x[i] - y[yidx[i]];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 6.
 */
void cartpole_QP_solver_LA_VSUB3_6(cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *v, cartpole_QP_solver_FLOAT *w, cartpole_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<6; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 5,
 * and x has length 5 and is indexed through yidx.
 */
void cartpole_QP_solver_LA_VSUB_INDEXED_5(cartpole_QP_solver_FLOAT *x, int* xidx, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<5; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 5.
 */
void cartpole_QP_solver_LA_VSUB3_5(cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *v, cartpole_QP_solver_FLOAT *w, cartpole_QP_solver_FLOAT *x)
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
void cartpole_QP_solver_LA_VSUB2_INDEXED_5(cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y, int* yidx, cartpole_QP_solver_FLOAT *z)
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
 * cartpole_QP_solver_NOPROGRESS (should be negative).
 */
int cartpole_QP_solver_LINESEARCH_BACKTRACKING_AFFINE(cartpole_QP_solver_FLOAT *l, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *dl, cartpole_QP_solver_FLOAT *ds, cartpole_QP_solver_FLOAT *a, cartpole_QP_solver_FLOAT *mu_aff)
{
    int i;
	int lsIt=1;    
    cartpole_QP_solver_FLOAT dltemp;
    cartpole_QP_solver_FLOAT dstemp;
    cartpole_QP_solver_FLOAT mya = 1.0;
    cartpole_QP_solver_FLOAT mymu;
        
    while( 1 ){                        

        /* 
         * Compute both snew and wnew together.
         * We compute also mu_affine along the way here, as the
         * values might be in registers, so it should be cheaper.
         */
        mymu = 0;
        for( i=0; i<150; i++ ){
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
        if( i == 150 ){
            break;
        } else {
            mya *= cartpole_QP_solver_SET_LS_SCALE_AFF;
            if( mya < cartpole_QP_solver_SET_LS_MINSTEP ){
                return cartpole_QP_solver_NOPROGRESS;
            }
        }
    }
    
    /* return new values and iteration counter */
    *a = mya;
    *mu_aff = mymu / (cartpole_QP_solver_FLOAT)150;
    return lsIt;
}


/*
 * Vector subtraction x = (u.*v - mu)*sigma where a is a scalar
*  and x,u,v are vectors of length 150.
 */
void cartpole_QP_solver_LA_VSUB5_150(cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *v, cartpole_QP_solver_FLOAT mu,  cartpole_QP_solver_FLOAT sigma, cartpole_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<150; i++){
		x[i] = u[i]*v[i] - mu;
		x[i] *= sigma;
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 14,
 * u, su, uidx are of length 6 and v, sv, vidx are of length 14.
 */
void cartpole_QP_solver_LA_VSUB6_INDEXED_14_6_14(cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *su, int* uidx, cartpole_QP_solver_FLOAT *v, cartpole_QP_solver_FLOAT *sv, int* vidx, cartpole_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<14; i++ ){
		x[i] = 0;
	}
	for( i=0; i<6; i++){
		x[uidx[i]] += u[i]/su[i];
	}
	for( i=0; i<14; i++){
		x[vidx[i]] -= v[i]/sv[i];
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void cartpole_QP_solver_LA_DENSE_2MVMADD_9_14_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<9; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<14; j++ ){		
		for( i=0; i<9; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<14; n++ ){
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
void cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<5; i++ ){
		r[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<14; j++ ){		
		for( i=0; i<5; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 5,
 * u, su, uidx are of length 5 and v, sv, vidx are of length 5.
 */
void cartpole_QP_solver_LA_VSUB6_INDEXED_5_5_5(cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *su, int* uidx, cartpole_QP_solver_FLOAT *v, cartpole_QP_solver_FLOAT *sv, int* vidx, cartpole_QP_solver_FLOAT *x)
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
void cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_5(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<5; i++ ){
		r[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<14; j++ ){		
		for( i=0; i<5; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
}


/*
 * Vector subtraction z = x - y for vectors of length 103.
 */
void cartpole_QP_solver_LA_VSUB_103(cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<103; i++){
		z[i] = x[i] - y[i];
	}
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 14 (length of y >= 14).
 */
void cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *y, int* yidx, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<14; i++ ){
		z[i] = -r[i]/s[i] - u[i]*y[yidx[i]];
	}
}


/** 
 * Computes z = -r./s + u.*y(y)
 * where all vectors except of y are of length 6 (length of y >= 6).
 */
void cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *y, int* yidx, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<6; i++ ){
		z[i] = -r[i]/s[i] + u[i]*y[yidx[i]];
	}
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 5 (length of y >= 5).
 */
void cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_5(cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *y, int* yidx, cartpole_QP_solver_FLOAT *z)
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
void cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_5(cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *y, int* yidx, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<5; i++ ){
		z[i] = -r[i]/s[i] + u[i]*y[yidx[i]];
	}
}


/*
 * Computes ds = -l.\(r + s.*dl) for vectors of length 150.
 */
void cartpole_QP_solver_LA_VSUB7_150(cartpole_QP_solver_FLOAT *l, cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *dl, cartpole_QP_solver_FLOAT *ds)
{
	int i;
	for( i=0; i<150; i++){
		ds[i] = -(r[i] + s[i]*dl[i])/l[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 103.
 */
void cartpole_QP_solver_LA_VADD_103(cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<103; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 39.
 */
void cartpole_QP_solver_LA_VADD_39(cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<39; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 150.
 */
void cartpole_QP_solver_LA_VADD_150(cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<150; i++){
		x[i] += y[i];
	}
}


/**
 * Backtracking line search for combined predictor/corrector step.
 * Update on variables with safety factor gamma (to keep us away from
 * boundary).
 */
int cartpole_QP_solver_LINESEARCH_BACKTRACKING_COMBINED(cartpole_QP_solver_FLOAT *z, cartpole_QP_solver_FLOAT *v, cartpole_QP_solver_FLOAT *l, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *dz, cartpole_QP_solver_FLOAT *dv, cartpole_QP_solver_FLOAT *dl, cartpole_QP_solver_FLOAT *ds, cartpole_QP_solver_FLOAT *a, cartpole_QP_solver_FLOAT *mu)
{
    int i, lsIt=1;       
    cartpole_QP_solver_FLOAT dltemp;
    cartpole_QP_solver_FLOAT dstemp;    
    cartpole_QP_solver_FLOAT a_gamma;
            
    *a = 1.0;
    while( 1 ){                        

        /* check whether search criterion is fulfilled */
        for( i=0; i<150; i++ ){
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
        if( i == 150 ){
            break;
        } else {
            *a *= cartpole_QP_solver_SET_LS_SCALE;
            if( *a < cartpole_QP_solver_SET_LS_MINSTEP ){
                return cartpole_QP_solver_NOPROGRESS;
            }
        }
    }
    
    /* update variables with safety margin */
    a_gamma = (*a)*cartpole_QP_solver_SET_LS_MAXSTEP;
    
    /* primal variables */
    for( i=0; i<103; i++ ){
        z[i] += a_gamma*dz[i];
    }
    
    /* equality constraint multipliers */
    for( i=0; i<39; i++ ){
        v[i] += a_gamma*dv[i];
    }
    
    /* inequality constraint multipliers & slacks, also update mu */
    *mu = 0;
    for( i=0; i<150; i++ ){
        dltemp = l[i] + a_gamma*dl[i]; l[i] = dltemp;
        dstemp = s[i] + a_gamma*ds[i]; s[i] = dstemp;
        *mu += dltemp*dstemp;
    }
    
    *a = a_gamma;
    *mu /= (cartpole_QP_solver_FLOAT)150;
    return lsIt;
}




/* VARIABLE DEFINITIONS ------------------------------------------------ */
cartpole_QP_solver_FLOAT cartpole_QP_solver_z[103];
cartpole_QP_solver_FLOAT cartpole_QP_solver_v[39];
cartpole_QP_solver_FLOAT cartpole_QP_solver_dz_aff[103];
cartpole_QP_solver_FLOAT cartpole_QP_solver_dv_aff[39];
cartpole_QP_solver_FLOAT cartpole_QP_solver_grad_cost[103];
cartpole_QP_solver_FLOAT cartpole_QP_solver_grad_eq[103];
cartpole_QP_solver_FLOAT cartpole_QP_solver_rd[103];
cartpole_QP_solver_FLOAT cartpole_QP_solver_l[150];
cartpole_QP_solver_FLOAT cartpole_QP_solver_s[150];
cartpole_QP_solver_FLOAT cartpole_QP_solver_lbys[150];
cartpole_QP_solver_FLOAT cartpole_QP_solver_dl_aff[150];
cartpole_QP_solver_FLOAT cartpole_QP_solver_ds_aff[150];
cartpole_QP_solver_FLOAT cartpole_QP_solver_dz_cc[103];
cartpole_QP_solver_FLOAT cartpole_QP_solver_dv_cc[39];
cartpole_QP_solver_FLOAT cartpole_QP_solver_dl_cc[150];
cartpole_QP_solver_FLOAT cartpole_QP_solver_ds_cc[150];
cartpole_QP_solver_FLOAT cartpole_QP_solver_ccrhs[150];
cartpole_QP_solver_FLOAT cartpole_QP_solver_grad_ineq[103];
cartpole_QP_solver_FLOAT cartpole_QP_solver_H0[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z0 = cartpole_QP_solver_z + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff0 = cartpole_QP_solver_dz_aff + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc0 = cartpole_QP_solver_dz_cc + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd0 = cartpole_QP_solver_rd + 0;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd0[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost0 = cartpole_QP_solver_grad_cost + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq0 = cartpole_QP_solver_grad_eq + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq0 = cartpole_QP_solver_grad_ineq + 0;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv0[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v0 = cartpole_QP_solver_v + 0;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re0[9];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta0[9];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc0[9];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff0 = cartpole_QP_solver_dv_aff + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc0 = cartpole_QP_solver_dv_cc + 0;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V0[126];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd0[45];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld0[45];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy0[9];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy0[9];
int cartpole_QP_solver_lbIdx0[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb0 = cartpole_QP_solver_l + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb0 = cartpole_QP_solver_s + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb0 = cartpole_QP_solver_lbys + 0;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb0[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff0 = cartpole_QP_solver_dl_aff + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff0 = cartpole_QP_solver_ds_aff + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc0 = cartpole_QP_solver_dl_cc + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc0 = cartpole_QP_solver_ds_cc + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl0 = cartpole_QP_solver_ccrhs + 0;
int cartpole_QP_solver_ubIdx0[6] = {0, 1, 2, 3, 4, 5};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub0 = cartpole_QP_solver_l + 14;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub0 = cartpole_QP_solver_s + 14;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub0 = cartpole_QP_solver_lbys + 14;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub0[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff0 = cartpole_QP_solver_dl_aff + 14;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff0 = cartpole_QP_solver_ds_aff + 14;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc0 = cartpole_QP_solver_dl_cc + 14;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc0 = cartpole_QP_solver_ds_cc + 14;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub0 = cartpole_QP_solver_ccrhs + 14;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi0[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z1 = cartpole_QP_solver_z + 14;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff1 = cartpole_QP_solver_dz_aff + 14;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc1 = cartpole_QP_solver_dz_cc + 14;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd1 = cartpole_QP_solver_rd + 14;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd1[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost1 = cartpole_QP_solver_grad_cost + 14;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq1 = cartpole_QP_solver_grad_eq + 14;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq1 = cartpole_QP_solver_grad_ineq + 14;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv1[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v1 = cartpole_QP_solver_v + 9;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re1[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta1[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc1[5];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff1 = cartpole_QP_solver_dv_aff + 9;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc1 = cartpole_QP_solver_dv_cc + 9;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V1[70];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd1[15];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld1[15];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy1[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy1[5];
int cartpole_QP_solver_lbIdx1[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb1 = cartpole_QP_solver_l + 20;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb1 = cartpole_QP_solver_s + 20;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb1 = cartpole_QP_solver_lbys + 20;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb1[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff1 = cartpole_QP_solver_dl_aff + 20;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff1 = cartpole_QP_solver_ds_aff + 20;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc1 = cartpole_QP_solver_dl_cc + 20;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc1 = cartpole_QP_solver_ds_cc + 20;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl1 = cartpole_QP_solver_ccrhs + 20;
int cartpole_QP_solver_ubIdx1[6] = {0, 1, 2, 3, 4, 5};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub1 = cartpole_QP_solver_l + 34;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub1 = cartpole_QP_solver_s + 34;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub1 = cartpole_QP_solver_lbys + 34;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub1[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff1 = cartpole_QP_solver_dl_aff + 34;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff1 = cartpole_QP_solver_ds_aff + 34;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc1 = cartpole_QP_solver_dl_cc + 34;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc1 = cartpole_QP_solver_ds_cc + 34;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub1 = cartpole_QP_solver_ccrhs + 34;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi1[14];
cartpole_QP_solver_FLOAT cartpole_QP_solver_D1[126] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
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
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
cartpole_QP_solver_FLOAT cartpole_QP_solver_W1[126];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd1[45];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd1[45];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z2 = cartpole_QP_solver_z + 28;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff2 = cartpole_QP_solver_dz_aff + 28;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc2 = cartpole_QP_solver_dz_cc + 28;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd2 = cartpole_QP_solver_rd + 28;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd2[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost2 = cartpole_QP_solver_grad_cost + 28;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq2 = cartpole_QP_solver_grad_eq + 28;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq2 = cartpole_QP_solver_grad_ineq + 28;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv2[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v2 = cartpole_QP_solver_v + 14;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re2[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta2[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc2[5];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff2 = cartpole_QP_solver_dv_aff + 14;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc2 = cartpole_QP_solver_dv_cc + 14;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V2[70];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd2[15];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld2[15];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy2[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy2[5];
int cartpole_QP_solver_lbIdx2[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb2 = cartpole_QP_solver_l + 40;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb2 = cartpole_QP_solver_s + 40;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb2 = cartpole_QP_solver_lbys + 40;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb2[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff2 = cartpole_QP_solver_dl_aff + 40;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff2 = cartpole_QP_solver_ds_aff + 40;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc2 = cartpole_QP_solver_dl_cc + 40;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc2 = cartpole_QP_solver_ds_cc + 40;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl2 = cartpole_QP_solver_ccrhs + 40;
int cartpole_QP_solver_ubIdx2[6] = {0, 1, 2, 3, 4, 5};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub2 = cartpole_QP_solver_l + 54;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub2 = cartpole_QP_solver_s + 54;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub2 = cartpole_QP_solver_lbys + 54;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub2[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff2 = cartpole_QP_solver_dl_aff + 54;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff2 = cartpole_QP_solver_ds_aff + 54;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc2 = cartpole_QP_solver_dl_cc + 54;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc2 = cartpole_QP_solver_ds_cc + 54;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub2 = cartpole_QP_solver_ccrhs + 54;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi2[14];
cartpole_QP_solver_FLOAT cartpole_QP_solver_D2[14] = {-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000};
cartpole_QP_solver_FLOAT cartpole_QP_solver_W2[14];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd2[25];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd2[25];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z3 = cartpole_QP_solver_z + 42;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff3 = cartpole_QP_solver_dz_aff + 42;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc3 = cartpole_QP_solver_dz_cc + 42;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd3 = cartpole_QP_solver_rd + 42;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd3[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost3 = cartpole_QP_solver_grad_cost + 42;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq3 = cartpole_QP_solver_grad_eq + 42;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq3 = cartpole_QP_solver_grad_ineq + 42;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv3[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v3 = cartpole_QP_solver_v + 19;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re3[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta3[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc3[5];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff3 = cartpole_QP_solver_dv_aff + 19;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc3 = cartpole_QP_solver_dv_cc + 19;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V3[70];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd3[15];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld3[15];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy3[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy3[5];
int cartpole_QP_solver_lbIdx3[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb3 = cartpole_QP_solver_l + 60;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb3 = cartpole_QP_solver_s + 60;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb3 = cartpole_QP_solver_lbys + 60;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb3[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff3 = cartpole_QP_solver_dl_aff + 60;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff3 = cartpole_QP_solver_ds_aff + 60;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc3 = cartpole_QP_solver_dl_cc + 60;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc3 = cartpole_QP_solver_ds_cc + 60;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl3 = cartpole_QP_solver_ccrhs + 60;
int cartpole_QP_solver_ubIdx3[6] = {0, 1, 2, 3, 4, 5};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub3 = cartpole_QP_solver_l + 74;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub3 = cartpole_QP_solver_s + 74;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub3 = cartpole_QP_solver_lbys + 74;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub3[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff3 = cartpole_QP_solver_dl_aff + 74;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff3 = cartpole_QP_solver_ds_aff + 74;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc3 = cartpole_QP_solver_dl_cc + 74;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc3 = cartpole_QP_solver_ds_cc + 74;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub3 = cartpole_QP_solver_ccrhs + 74;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi3[14];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W3[14];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd3[25];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd3[25];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z4 = cartpole_QP_solver_z + 56;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff4 = cartpole_QP_solver_dz_aff + 56;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc4 = cartpole_QP_solver_dz_cc + 56;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd4 = cartpole_QP_solver_rd + 56;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd4[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost4 = cartpole_QP_solver_grad_cost + 56;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq4 = cartpole_QP_solver_grad_eq + 56;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq4 = cartpole_QP_solver_grad_ineq + 56;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv4[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v4 = cartpole_QP_solver_v + 24;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re4[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta4[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc4[5];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff4 = cartpole_QP_solver_dv_aff + 24;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc4 = cartpole_QP_solver_dv_cc + 24;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V4[70];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd4[15];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld4[15];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy4[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy4[5];
int cartpole_QP_solver_lbIdx4[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb4 = cartpole_QP_solver_l + 80;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb4 = cartpole_QP_solver_s + 80;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb4 = cartpole_QP_solver_lbys + 80;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb4[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff4 = cartpole_QP_solver_dl_aff + 80;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff4 = cartpole_QP_solver_ds_aff + 80;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc4 = cartpole_QP_solver_dl_cc + 80;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc4 = cartpole_QP_solver_ds_cc + 80;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl4 = cartpole_QP_solver_ccrhs + 80;
int cartpole_QP_solver_ubIdx4[6] = {0, 1, 2, 3, 4, 5};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub4 = cartpole_QP_solver_l + 94;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub4 = cartpole_QP_solver_s + 94;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub4 = cartpole_QP_solver_lbys + 94;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub4[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff4 = cartpole_QP_solver_dl_aff + 94;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff4 = cartpole_QP_solver_ds_aff + 94;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc4 = cartpole_QP_solver_dl_cc + 94;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc4 = cartpole_QP_solver_ds_cc + 94;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub4 = cartpole_QP_solver_ccrhs + 94;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi4[14];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W4[14];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd4[25];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd4[25];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z5 = cartpole_QP_solver_z + 70;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff5 = cartpole_QP_solver_dz_aff + 70;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc5 = cartpole_QP_solver_dz_cc + 70;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd5 = cartpole_QP_solver_rd + 70;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd5[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost5 = cartpole_QP_solver_grad_cost + 70;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq5 = cartpole_QP_solver_grad_eq + 70;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq5 = cartpole_QP_solver_grad_ineq + 70;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv5[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v5 = cartpole_QP_solver_v + 29;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re5[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta5[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc5[5];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff5 = cartpole_QP_solver_dv_aff + 29;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc5 = cartpole_QP_solver_dv_cc + 29;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V5[70];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd5[15];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld5[15];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy5[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy5[5];
int cartpole_QP_solver_lbIdx5[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb5 = cartpole_QP_solver_l + 100;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb5 = cartpole_QP_solver_s + 100;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb5 = cartpole_QP_solver_lbys + 100;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb5[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff5 = cartpole_QP_solver_dl_aff + 100;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff5 = cartpole_QP_solver_ds_aff + 100;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc5 = cartpole_QP_solver_dl_cc + 100;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc5 = cartpole_QP_solver_ds_cc + 100;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl5 = cartpole_QP_solver_ccrhs + 100;
int cartpole_QP_solver_ubIdx5[6] = {0, 1, 2, 3, 4, 5};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub5 = cartpole_QP_solver_l + 114;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub5 = cartpole_QP_solver_s + 114;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub5 = cartpole_QP_solver_lbys + 114;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub5[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff5 = cartpole_QP_solver_dl_aff + 114;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff5 = cartpole_QP_solver_ds_aff + 114;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc5 = cartpole_QP_solver_dl_cc + 114;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc5 = cartpole_QP_solver_ds_cc + 114;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub5 = cartpole_QP_solver_ccrhs + 114;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi5[14];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W5[14];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd5[25];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd5[25];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z6 = cartpole_QP_solver_z + 84;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff6 = cartpole_QP_solver_dz_aff + 84;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc6 = cartpole_QP_solver_dz_cc + 84;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd6 = cartpole_QP_solver_rd + 84;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd6[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost6 = cartpole_QP_solver_grad_cost + 84;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq6 = cartpole_QP_solver_grad_eq + 84;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq6 = cartpole_QP_solver_grad_ineq + 84;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv6[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v6 = cartpole_QP_solver_v + 34;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re6[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta6[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc6[5];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff6 = cartpole_QP_solver_dv_aff + 34;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc6 = cartpole_QP_solver_dv_cc + 34;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V6[70];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd6[15];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld6[15];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy6[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy6[5];
int cartpole_QP_solver_lbIdx6[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb6 = cartpole_QP_solver_l + 120;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb6 = cartpole_QP_solver_s + 120;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb6 = cartpole_QP_solver_lbys + 120;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb6[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff6 = cartpole_QP_solver_dl_aff + 120;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff6 = cartpole_QP_solver_ds_aff + 120;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc6 = cartpole_QP_solver_dl_cc + 120;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc6 = cartpole_QP_solver_ds_cc + 120;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl6 = cartpole_QP_solver_ccrhs + 120;
int cartpole_QP_solver_ubIdx6[6] = {0, 1, 2, 3, 4, 5};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub6 = cartpole_QP_solver_l + 134;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub6 = cartpole_QP_solver_s + 134;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub6 = cartpole_QP_solver_lbys + 134;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub6[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff6 = cartpole_QP_solver_dl_aff + 134;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff6 = cartpole_QP_solver_ds_aff + 134;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc6 = cartpole_QP_solver_dl_cc + 134;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc6 = cartpole_QP_solver_ds_cc + 134;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub6 = cartpole_QP_solver_ccrhs + 134;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi6[14];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W6[14];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd6[25];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd6[25];
cartpole_QP_solver_FLOAT cartpole_QP_solver_H7[5] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
cartpole_QP_solver_FLOAT cartpole_QP_solver_f7[5] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z7 = cartpole_QP_solver_z + 98;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff7 = cartpole_QP_solver_dz_aff + 98;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc7 = cartpole_QP_solver_dz_cc + 98;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd7 = cartpole_QP_solver_rd + 98;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd7[5];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost7 = cartpole_QP_solver_grad_cost + 98;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq7 = cartpole_QP_solver_grad_eq + 98;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq7 = cartpole_QP_solver_grad_ineq + 98;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv7[5];
int cartpole_QP_solver_lbIdx7[5] = {0, 1, 2, 3, 4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb7 = cartpole_QP_solver_l + 140;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb7 = cartpole_QP_solver_s + 140;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb7 = cartpole_QP_solver_lbys + 140;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb7[5];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff7 = cartpole_QP_solver_dl_aff + 140;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff7 = cartpole_QP_solver_ds_aff + 140;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc7 = cartpole_QP_solver_dl_cc + 140;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc7 = cartpole_QP_solver_ds_cc + 140;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl7 = cartpole_QP_solver_ccrhs + 140;
int cartpole_QP_solver_ubIdx7[5] = {0, 1, 2, 3, 4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub7 = cartpole_QP_solver_l + 145;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub7 = cartpole_QP_solver_s + 145;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub7 = cartpole_QP_solver_lbys + 145;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub7[5];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff7 = cartpole_QP_solver_dl_aff + 145;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff7 = cartpole_QP_solver_ds_aff + 145;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc7 = cartpole_QP_solver_dl_cc + 145;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc7 = cartpole_QP_solver_ds_cc + 145;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub7 = cartpole_QP_solver_ccrhs + 145;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi7[5];
cartpole_QP_solver_FLOAT cartpole_QP_solver_D7[5] = {-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000};
cartpole_QP_solver_FLOAT cartpole_QP_solver_W7[5];
cartpole_QP_solver_FLOAT musigma;
cartpole_QP_solver_FLOAT sigma_3rdroot;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Diag1_0[14];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Diag2_0[14];
cartpole_QP_solver_FLOAT cartpole_QP_solver_L_0[91];




/* SOLVER CODE --------------------------------------------------------- */
int cartpole_QP_solver_solve(cartpole_QP_solver_params* params, cartpole_QP_solver_output* output, cartpole_QP_solver_info* info)
{	
int exitcode;

#if cartpole_QP_solver_SET_TIMING == 1
	cartpole_QP_solver_timer solvertimer;
	cartpole_QP_solver_tic(&solvertimer);
#endif
/* FUNCTION CALLS INTO LA LIBRARY -------------------------------------- */
info->it = 0;
cartpole_QP_solver_LA_INITIALIZEVECTOR_103(cartpole_QP_solver_z, 0);
cartpole_QP_solver_LA_INITIALIZEVECTOR_39(cartpole_QP_solver_v, 1);
cartpole_QP_solver_LA_INITIALIZEVECTOR_150(cartpole_QP_solver_l, 10);
cartpole_QP_solver_LA_INITIALIZEVECTOR_150(cartpole_QP_solver_s, 10);
info->mu = 0;
cartpole_QP_solver_LA_DOTACC_150(cartpole_QP_solver_l, cartpole_QP_solver_s, &info->mu);
info->mu /= 150;
while( 1 ){
info->pobj = 0;
cartpole_QP_solver_LA_DIAG_QUADFCN_14(cartpole_QP_solver_H0, params->f1, cartpole_QP_solver_z0, cartpole_QP_solver_grad_cost0, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_14(cartpole_QP_solver_H0, params->f2, cartpole_QP_solver_z1, cartpole_QP_solver_grad_cost1, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_14(cartpole_QP_solver_H0, params->f3, cartpole_QP_solver_z2, cartpole_QP_solver_grad_cost2, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_14(cartpole_QP_solver_H0, params->f4, cartpole_QP_solver_z3, cartpole_QP_solver_grad_cost3, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_14(cartpole_QP_solver_H0, params->f5, cartpole_QP_solver_z4, cartpole_QP_solver_grad_cost4, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_14(cartpole_QP_solver_H0, params->f6, cartpole_QP_solver_z5, cartpole_QP_solver_grad_cost5, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_14(cartpole_QP_solver_H0, params->f7, cartpole_QP_solver_z6, cartpole_QP_solver_grad_cost6, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_5(cartpole_QP_solver_H7, cartpole_QP_solver_f7, cartpole_QP_solver_z7, cartpole_QP_solver_grad_cost7, &info->pobj);
info->res_eq = 0;
info->dgap = 0;
cartpole_QP_solver_LA_DENSE_MVMSUB3_9_14_14(params->C1, cartpole_QP_solver_z0, cartpole_QP_solver_D1, cartpole_QP_solver_z1, params->e1, cartpole_QP_solver_v0, cartpole_QP_solver_re0, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_14(params->C2, cartpole_QP_solver_z1, cartpole_QP_solver_D2, cartpole_QP_solver_z2, params->e2, cartpole_QP_solver_v1, cartpole_QP_solver_re1, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_14(params->C3, cartpole_QP_solver_z2, cartpole_QP_solver_D2, cartpole_QP_solver_z3, params->e3, cartpole_QP_solver_v2, cartpole_QP_solver_re2, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_14(params->C4, cartpole_QP_solver_z3, cartpole_QP_solver_D2, cartpole_QP_solver_z4, params->e4, cartpole_QP_solver_v3, cartpole_QP_solver_re3, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_14(params->C5, cartpole_QP_solver_z4, cartpole_QP_solver_D2, cartpole_QP_solver_z5, params->e5, cartpole_QP_solver_v4, cartpole_QP_solver_re4, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_14(params->C6, cartpole_QP_solver_z5, cartpole_QP_solver_D2, cartpole_QP_solver_z6, params->e6, cartpole_QP_solver_v5, cartpole_QP_solver_re5, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_5(params->C7, cartpole_QP_solver_z6, cartpole_QP_solver_D7, cartpole_QP_solver_z7, params->e7, cartpole_QP_solver_v6, cartpole_QP_solver_re6, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MTVM_9_14(params->C1, cartpole_QP_solver_v0, cartpole_QP_solver_grad_eq0);
cartpole_QP_solver_LA_DENSE_MTVM2_5_14_9(params->C2, cartpole_QP_solver_v1, cartpole_QP_solver_D1, cartpole_QP_solver_v0, cartpole_QP_solver_grad_eq1);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C3, cartpole_QP_solver_v2, cartpole_QP_solver_D2, cartpole_QP_solver_v1, cartpole_QP_solver_grad_eq2);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C4, cartpole_QP_solver_v3, cartpole_QP_solver_D2, cartpole_QP_solver_v2, cartpole_QP_solver_grad_eq3);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C5, cartpole_QP_solver_v4, cartpole_QP_solver_D2, cartpole_QP_solver_v3, cartpole_QP_solver_grad_eq4);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C6, cartpole_QP_solver_v5, cartpole_QP_solver_D2, cartpole_QP_solver_v4, cartpole_QP_solver_grad_eq5);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C7, cartpole_QP_solver_v6, cartpole_QP_solver_D2, cartpole_QP_solver_v5, cartpole_QP_solver_grad_eq6);
cartpole_QP_solver_LA_DIAGZERO_MTVM_5_5(cartpole_QP_solver_D7, cartpole_QP_solver_v6, cartpole_QP_solver_grad_eq7);
info->res_ineq = 0;
cartpole_QP_solver_LA_VSUBADD3_14(params->lb1, cartpole_QP_solver_z0, cartpole_QP_solver_lbIdx0, cartpole_QP_solver_llb0, cartpole_QP_solver_slb0, cartpole_QP_solver_rilb0, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_6(cartpole_QP_solver_z0, cartpole_QP_solver_ubIdx0, params->ub1, cartpole_QP_solver_lub0, cartpole_QP_solver_sub0, cartpole_QP_solver_riub0, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_14(params->lb2, cartpole_QP_solver_z1, cartpole_QP_solver_lbIdx1, cartpole_QP_solver_llb1, cartpole_QP_solver_slb1, cartpole_QP_solver_rilb1, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_6(cartpole_QP_solver_z1, cartpole_QP_solver_ubIdx1, params->ub2, cartpole_QP_solver_lub1, cartpole_QP_solver_sub1, cartpole_QP_solver_riub1, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_14(params->lb3, cartpole_QP_solver_z2, cartpole_QP_solver_lbIdx2, cartpole_QP_solver_llb2, cartpole_QP_solver_slb2, cartpole_QP_solver_rilb2, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_6(cartpole_QP_solver_z2, cartpole_QP_solver_ubIdx2, params->ub3, cartpole_QP_solver_lub2, cartpole_QP_solver_sub2, cartpole_QP_solver_riub2, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_14(params->lb4, cartpole_QP_solver_z3, cartpole_QP_solver_lbIdx3, cartpole_QP_solver_llb3, cartpole_QP_solver_slb3, cartpole_QP_solver_rilb3, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_6(cartpole_QP_solver_z3, cartpole_QP_solver_ubIdx3, params->ub4, cartpole_QP_solver_lub3, cartpole_QP_solver_sub3, cartpole_QP_solver_riub3, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_14(params->lb5, cartpole_QP_solver_z4, cartpole_QP_solver_lbIdx4, cartpole_QP_solver_llb4, cartpole_QP_solver_slb4, cartpole_QP_solver_rilb4, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_6(cartpole_QP_solver_z4, cartpole_QP_solver_ubIdx4, params->ub5, cartpole_QP_solver_lub4, cartpole_QP_solver_sub4, cartpole_QP_solver_riub4, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_14(params->lb6, cartpole_QP_solver_z5, cartpole_QP_solver_lbIdx5, cartpole_QP_solver_llb5, cartpole_QP_solver_slb5, cartpole_QP_solver_rilb5, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_6(cartpole_QP_solver_z5, cartpole_QP_solver_ubIdx5, params->ub6, cartpole_QP_solver_lub5, cartpole_QP_solver_sub5, cartpole_QP_solver_riub5, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_14(params->lb7, cartpole_QP_solver_z6, cartpole_QP_solver_lbIdx6, cartpole_QP_solver_llb6, cartpole_QP_solver_slb6, cartpole_QP_solver_rilb6, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_6(cartpole_QP_solver_z6, cartpole_QP_solver_ubIdx6, params->ub7, cartpole_QP_solver_lub6, cartpole_QP_solver_sub6, cartpole_QP_solver_riub6, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_5(params->lb8, cartpole_QP_solver_z7, cartpole_QP_solver_lbIdx7, cartpole_QP_solver_llb7, cartpole_QP_solver_slb7, cartpole_QP_solver_rilb7, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_5(cartpole_QP_solver_z7, cartpole_QP_solver_ubIdx7, params->ub8, cartpole_QP_solver_lub7, cartpole_QP_solver_sub7, cartpole_QP_solver_riub7, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_INEQ_B_GRAD_14_14_6(cartpole_QP_solver_lub0, cartpole_QP_solver_sub0, cartpole_QP_solver_riub0, cartpole_QP_solver_llb0, cartpole_QP_solver_slb0, cartpole_QP_solver_rilb0, cartpole_QP_solver_lbIdx0, cartpole_QP_solver_ubIdx0, cartpole_QP_solver_grad_ineq0, cartpole_QP_solver_lubbysub0, cartpole_QP_solver_llbbyslb0);
cartpole_QP_solver_LA_INEQ_B_GRAD_14_14_6(cartpole_QP_solver_lub1, cartpole_QP_solver_sub1, cartpole_QP_solver_riub1, cartpole_QP_solver_llb1, cartpole_QP_solver_slb1, cartpole_QP_solver_rilb1, cartpole_QP_solver_lbIdx1, cartpole_QP_solver_ubIdx1, cartpole_QP_solver_grad_ineq1, cartpole_QP_solver_lubbysub1, cartpole_QP_solver_llbbyslb1);
cartpole_QP_solver_LA_INEQ_B_GRAD_14_14_6(cartpole_QP_solver_lub2, cartpole_QP_solver_sub2, cartpole_QP_solver_riub2, cartpole_QP_solver_llb2, cartpole_QP_solver_slb2, cartpole_QP_solver_rilb2, cartpole_QP_solver_lbIdx2, cartpole_QP_solver_ubIdx2, cartpole_QP_solver_grad_ineq2, cartpole_QP_solver_lubbysub2, cartpole_QP_solver_llbbyslb2);
cartpole_QP_solver_LA_INEQ_B_GRAD_14_14_6(cartpole_QP_solver_lub3, cartpole_QP_solver_sub3, cartpole_QP_solver_riub3, cartpole_QP_solver_llb3, cartpole_QP_solver_slb3, cartpole_QP_solver_rilb3, cartpole_QP_solver_lbIdx3, cartpole_QP_solver_ubIdx3, cartpole_QP_solver_grad_ineq3, cartpole_QP_solver_lubbysub3, cartpole_QP_solver_llbbyslb3);
cartpole_QP_solver_LA_INEQ_B_GRAD_14_14_6(cartpole_QP_solver_lub4, cartpole_QP_solver_sub4, cartpole_QP_solver_riub4, cartpole_QP_solver_llb4, cartpole_QP_solver_slb4, cartpole_QP_solver_rilb4, cartpole_QP_solver_lbIdx4, cartpole_QP_solver_ubIdx4, cartpole_QP_solver_grad_ineq4, cartpole_QP_solver_lubbysub4, cartpole_QP_solver_llbbyslb4);
cartpole_QP_solver_LA_INEQ_B_GRAD_14_14_6(cartpole_QP_solver_lub5, cartpole_QP_solver_sub5, cartpole_QP_solver_riub5, cartpole_QP_solver_llb5, cartpole_QP_solver_slb5, cartpole_QP_solver_rilb5, cartpole_QP_solver_lbIdx5, cartpole_QP_solver_ubIdx5, cartpole_QP_solver_grad_ineq5, cartpole_QP_solver_lubbysub5, cartpole_QP_solver_llbbyslb5);
cartpole_QP_solver_LA_INEQ_B_GRAD_14_14_6(cartpole_QP_solver_lub6, cartpole_QP_solver_sub6, cartpole_QP_solver_riub6, cartpole_QP_solver_llb6, cartpole_QP_solver_slb6, cartpole_QP_solver_rilb6, cartpole_QP_solver_lbIdx6, cartpole_QP_solver_ubIdx6, cartpole_QP_solver_grad_ineq6, cartpole_QP_solver_lubbysub6, cartpole_QP_solver_llbbyslb6);
cartpole_QP_solver_LA_INEQ_B_GRAD_5_5_5(cartpole_QP_solver_lub7, cartpole_QP_solver_sub7, cartpole_QP_solver_riub7, cartpole_QP_solver_llb7, cartpole_QP_solver_slb7, cartpole_QP_solver_rilb7, cartpole_QP_solver_lbIdx7, cartpole_QP_solver_ubIdx7, cartpole_QP_solver_grad_ineq7, cartpole_QP_solver_lubbysub7, cartpole_QP_solver_llbbyslb7);
info->dobj = info->pobj - info->dgap;
info->rdgap = info->pobj ? info->dgap / info->pobj : 1e6;
if( info->rdgap < 0 ) info->rdgap = -info->rdgap;
if( info->mu < cartpole_QP_solver_SET_ACC_KKTCOMPL
    && (info->rdgap < cartpole_QP_solver_SET_ACC_RDGAP || info->dgap < cartpole_QP_solver_SET_ACC_KKTCOMPL)
    && info->res_eq < cartpole_QP_solver_SET_ACC_RESEQ
    && info->res_ineq < cartpole_QP_solver_SET_ACC_RESINEQ ){
exitcode = cartpole_QP_solver_OPTIMAL; break; }
if( info->it == cartpole_QP_solver_SET_MAXIT ){
exitcode = cartpole_QP_solver_MAXITREACHED; break; }
cartpole_QP_solver_LA_VVADD3_103(cartpole_QP_solver_grad_cost, cartpole_QP_solver_grad_eq, cartpole_QP_solver_grad_ineq, cartpole_QP_solver_rd);
cartpole_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(cartpole_QP_solver_H0, cartpole_QP_solver_llbbyslb0, cartpole_QP_solver_lbIdx0, cartpole_QP_solver_lubbysub0, cartpole_QP_solver_ubIdx0, cartpole_QP_solver_Phi0);
cartpole_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_14(cartpole_QP_solver_Phi0, params->C1, cartpole_QP_solver_V0);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_14(cartpole_QP_solver_Phi0, cartpole_QP_solver_rd0, cartpole_QP_solver_Lbyrd0);
cartpole_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(cartpole_QP_solver_H0, cartpole_QP_solver_llbbyslb1, cartpole_QP_solver_lbIdx1, cartpole_QP_solver_lubbysub1, cartpole_QP_solver_ubIdx1, cartpole_QP_solver_Phi1);
cartpole_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_14(cartpole_QP_solver_Phi1, params->C2, cartpole_QP_solver_V1);
cartpole_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_14(cartpole_QP_solver_Phi1, cartpole_QP_solver_D1, cartpole_QP_solver_W1);
cartpole_QP_solver_LA_DENSE_MMTM_9_14_5(cartpole_QP_solver_W1, cartpole_QP_solver_V1, cartpole_QP_solver_Ysd1);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_14(cartpole_QP_solver_Phi1, cartpole_QP_solver_rd1, cartpole_QP_solver_Lbyrd1);
cartpole_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(cartpole_QP_solver_H0, cartpole_QP_solver_llbbyslb2, cartpole_QP_solver_lbIdx2, cartpole_QP_solver_lubbysub2, cartpole_QP_solver_ubIdx2, cartpole_QP_solver_Phi2);
cartpole_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_14(cartpole_QP_solver_Phi2, params->C3, cartpole_QP_solver_V2);
cartpole_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_14(cartpole_QP_solver_Phi2, cartpole_QP_solver_D2, cartpole_QP_solver_W2);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_14_5(cartpole_QP_solver_W2, cartpole_QP_solver_V2, cartpole_QP_solver_Ysd2);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_14(cartpole_QP_solver_Phi2, cartpole_QP_solver_rd2, cartpole_QP_solver_Lbyrd2);
cartpole_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(cartpole_QP_solver_H0, cartpole_QP_solver_llbbyslb3, cartpole_QP_solver_lbIdx3, cartpole_QP_solver_lubbysub3, cartpole_QP_solver_ubIdx3, cartpole_QP_solver_Phi3);
cartpole_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_14(cartpole_QP_solver_Phi3, params->C4, cartpole_QP_solver_V3);
cartpole_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_14(cartpole_QP_solver_Phi3, cartpole_QP_solver_D2, cartpole_QP_solver_W3);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_14_5(cartpole_QP_solver_W3, cartpole_QP_solver_V3, cartpole_QP_solver_Ysd3);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_14(cartpole_QP_solver_Phi3, cartpole_QP_solver_rd3, cartpole_QP_solver_Lbyrd3);
cartpole_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(cartpole_QP_solver_H0, cartpole_QP_solver_llbbyslb4, cartpole_QP_solver_lbIdx4, cartpole_QP_solver_lubbysub4, cartpole_QP_solver_ubIdx4, cartpole_QP_solver_Phi4);
cartpole_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_14(cartpole_QP_solver_Phi4, params->C5, cartpole_QP_solver_V4);
cartpole_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_14(cartpole_QP_solver_Phi4, cartpole_QP_solver_D2, cartpole_QP_solver_W4);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_14_5(cartpole_QP_solver_W4, cartpole_QP_solver_V4, cartpole_QP_solver_Ysd4);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_14(cartpole_QP_solver_Phi4, cartpole_QP_solver_rd4, cartpole_QP_solver_Lbyrd4);
cartpole_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(cartpole_QP_solver_H0, cartpole_QP_solver_llbbyslb5, cartpole_QP_solver_lbIdx5, cartpole_QP_solver_lubbysub5, cartpole_QP_solver_ubIdx5, cartpole_QP_solver_Phi5);
cartpole_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_14(cartpole_QP_solver_Phi5, params->C6, cartpole_QP_solver_V5);
cartpole_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_14(cartpole_QP_solver_Phi5, cartpole_QP_solver_D2, cartpole_QP_solver_W5);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_14_5(cartpole_QP_solver_W5, cartpole_QP_solver_V5, cartpole_QP_solver_Ysd5);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_14(cartpole_QP_solver_Phi5, cartpole_QP_solver_rd5, cartpole_QP_solver_Lbyrd5);
cartpole_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(cartpole_QP_solver_H0, cartpole_QP_solver_llbbyslb6, cartpole_QP_solver_lbIdx6, cartpole_QP_solver_lubbysub6, cartpole_QP_solver_ubIdx6, cartpole_QP_solver_Phi6);
cartpole_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_14(cartpole_QP_solver_Phi6, params->C7, cartpole_QP_solver_V6);
cartpole_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_14(cartpole_QP_solver_Phi6, cartpole_QP_solver_D2, cartpole_QP_solver_W6);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_14_5(cartpole_QP_solver_W6, cartpole_QP_solver_V6, cartpole_QP_solver_Ysd6);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_14(cartpole_QP_solver_Phi6, cartpole_QP_solver_rd6, cartpole_QP_solver_Lbyrd6);
cartpole_QP_solver_LA_DIAG_CHOL_ONELOOP_LBUB_5_5_5(cartpole_QP_solver_H7, cartpole_QP_solver_llbbyslb7, cartpole_QP_solver_lbIdx7, cartpole_QP_solver_lubbysub7, cartpole_QP_solver_ubIdx7, cartpole_QP_solver_Phi7);
cartpole_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_5(cartpole_QP_solver_Phi7, cartpole_QP_solver_D7, cartpole_QP_solver_W7);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_5(cartpole_QP_solver_Phi7, cartpole_QP_solver_rd7, cartpole_QP_solver_Lbyrd7);
cartpole_QP_solver_LA_DENSE_MMT2_9_14_14(cartpole_QP_solver_V0, cartpole_QP_solver_W1, cartpole_QP_solver_Yd0);
cartpole_QP_solver_LA_DENSE_MVMSUB2_9_14_14(cartpole_QP_solver_V0, cartpole_QP_solver_Lbyrd0, cartpole_QP_solver_W1, cartpole_QP_solver_Lbyrd1, cartpole_QP_solver_re0, cartpole_QP_solver_beta0);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_14(cartpole_QP_solver_V1, cartpole_QP_solver_W2, cartpole_QP_solver_Yd1);
cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_14(cartpole_QP_solver_V1, cartpole_QP_solver_Lbyrd1, cartpole_QP_solver_W2, cartpole_QP_solver_Lbyrd2, cartpole_QP_solver_re1, cartpole_QP_solver_beta1);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_14(cartpole_QP_solver_V2, cartpole_QP_solver_W3, cartpole_QP_solver_Yd2);
cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_14(cartpole_QP_solver_V2, cartpole_QP_solver_Lbyrd2, cartpole_QP_solver_W3, cartpole_QP_solver_Lbyrd3, cartpole_QP_solver_re2, cartpole_QP_solver_beta2);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_14(cartpole_QP_solver_V3, cartpole_QP_solver_W4, cartpole_QP_solver_Yd3);
cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_14(cartpole_QP_solver_V3, cartpole_QP_solver_Lbyrd3, cartpole_QP_solver_W4, cartpole_QP_solver_Lbyrd4, cartpole_QP_solver_re3, cartpole_QP_solver_beta3);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_14(cartpole_QP_solver_V4, cartpole_QP_solver_W5, cartpole_QP_solver_Yd4);
cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_14(cartpole_QP_solver_V4, cartpole_QP_solver_Lbyrd4, cartpole_QP_solver_W5, cartpole_QP_solver_Lbyrd5, cartpole_QP_solver_re4, cartpole_QP_solver_beta4);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_14(cartpole_QP_solver_V5, cartpole_QP_solver_W6, cartpole_QP_solver_Yd5);
cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_14(cartpole_QP_solver_V5, cartpole_QP_solver_Lbyrd5, cartpole_QP_solver_W6, cartpole_QP_solver_Lbyrd6, cartpole_QP_solver_re5, cartpole_QP_solver_beta5);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_5(cartpole_QP_solver_V6, cartpole_QP_solver_W7, cartpole_QP_solver_Yd6);
cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_5(cartpole_QP_solver_V6, cartpole_QP_solver_Lbyrd6, cartpole_QP_solver_W7, cartpole_QP_solver_Lbyrd7, cartpole_QP_solver_re6, cartpole_QP_solver_beta6);
cartpole_QP_solver_LA_DENSE_CHOL_9(cartpole_QP_solver_Yd0, cartpole_QP_solver_Ld0);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_9(cartpole_QP_solver_Ld0, cartpole_QP_solver_beta0, cartpole_QP_solver_yy0);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_9(cartpole_QP_solver_Ld0, cartpole_QP_solver_Ysd1, cartpole_QP_solver_Lsd1);
cartpole_QP_solver_LA_DENSE_MMTSUB_5_9(cartpole_QP_solver_Lsd1, cartpole_QP_solver_Yd1);
cartpole_QP_solver_LA_DENSE_CHOL_5(cartpole_QP_solver_Yd1, cartpole_QP_solver_Ld1);
cartpole_QP_solver_LA_DENSE_MVMSUB1_5_9(cartpole_QP_solver_Lsd1, cartpole_QP_solver_yy0, cartpole_QP_solver_beta1, cartpole_QP_solver_bmy1);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_5(cartpole_QP_solver_Ld1, cartpole_QP_solver_bmy1, cartpole_QP_solver_yy1);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(cartpole_QP_solver_Ld1, cartpole_QP_solver_Ysd2, cartpole_QP_solver_Lsd2);
cartpole_QP_solver_LA_DENSE_MMTSUB_5_5(cartpole_QP_solver_Lsd2, cartpole_QP_solver_Yd2);
cartpole_QP_solver_LA_DENSE_CHOL_5(cartpole_QP_solver_Yd2, cartpole_QP_solver_Ld2);
cartpole_QP_solver_LA_DENSE_MVMSUB1_5_5(cartpole_QP_solver_Lsd2, cartpole_QP_solver_yy1, cartpole_QP_solver_beta2, cartpole_QP_solver_bmy2);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_5(cartpole_QP_solver_Ld2, cartpole_QP_solver_bmy2, cartpole_QP_solver_yy2);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(cartpole_QP_solver_Ld2, cartpole_QP_solver_Ysd3, cartpole_QP_solver_Lsd3);
cartpole_QP_solver_LA_DENSE_MMTSUB_5_5(cartpole_QP_solver_Lsd3, cartpole_QP_solver_Yd3);
cartpole_QP_solver_LA_DENSE_CHOL_5(cartpole_QP_solver_Yd3, cartpole_QP_solver_Ld3);
cartpole_QP_solver_LA_DENSE_MVMSUB1_5_5(cartpole_QP_solver_Lsd3, cartpole_QP_solver_yy2, cartpole_QP_solver_beta3, cartpole_QP_solver_bmy3);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_5(cartpole_QP_solver_Ld3, cartpole_QP_solver_bmy3, cartpole_QP_solver_yy3);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(cartpole_QP_solver_Ld3, cartpole_QP_solver_Ysd4, cartpole_QP_solver_Lsd4);
cartpole_QP_solver_LA_DENSE_MMTSUB_5_5(cartpole_QP_solver_Lsd4, cartpole_QP_solver_Yd4);
cartpole_QP_solver_LA_DENSE_CHOL_5(cartpole_QP_solver_Yd4, cartpole_QP_solver_Ld4);
cartpole_QP_solver_LA_DENSE_MVMSUB1_5_5(cartpole_QP_solver_Lsd4, cartpole_QP_solver_yy3, cartpole_QP_solver_beta4, cartpole_QP_solver_bmy4);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_5(cartpole_QP_solver_Ld4, cartpole_QP_solver_bmy4, cartpole_QP_solver_yy4);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(cartpole_QP_solver_Ld4, cartpole_QP_solver_Ysd5, cartpole_QP_solver_Lsd5);
cartpole_QP_solver_LA_DENSE_MMTSUB_5_5(cartpole_QP_solver_Lsd5, cartpole_QP_solver_Yd5);
cartpole_QP_solver_LA_DENSE_CHOL_5(cartpole_QP_solver_Yd5, cartpole_QP_solver_Ld5);
cartpole_QP_solver_LA_DENSE_MVMSUB1_5_5(cartpole_QP_solver_Lsd5, cartpole_QP_solver_yy4, cartpole_QP_solver_beta5, cartpole_QP_solver_bmy5);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_5(cartpole_QP_solver_Ld5, cartpole_QP_solver_bmy5, cartpole_QP_solver_yy5);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(cartpole_QP_solver_Ld5, cartpole_QP_solver_Ysd6, cartpole_QP_solver_Lsd6);
cartpole_QP_solver_LA_DENSE_MMTSUB_5_5(cartpole_QP_solver_Lsd6, cartpole_QP_solver_Yd6);
cartpole_QP_solver_LA_DENSE_CHOL_5(cartpole_QP_solver_Yd6, cartpole_QP_solver_Ld6);
cartpole_QP_solver_LA_DENSE_MVMSUB1_5_5(cartpole_QP_solver_Lsd6, cartpole_QP_solver_yy5, cartpole_QP_solver_beta6, cartpole_QP_solver_bmy6);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_5(cartpole_QP_solver_Ld6, cartpole_QP_solver_bmy6, cartpole_QP_solver_yy6);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_5(cartpole_QP_solver_Ld6, cartpole_QP_solver_yy6, cartpole_QP_solver_dvaff6);
cartpole_QP_solver_LA_DENSE_MTVMSUB_5_5(cartpole_QP_solver_Lsd6, cartpole_QP_solver_dvaff6, cartpole_QP_solver_yy5, cartpole_QP_solver_bmy5);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_5(cartpole_QP_solver_Ld5, cartpole_QP_solver_bmy5, cartpole_QP_solver_dvaff5);
cartpole_QP_solver_LA_DENSE_MTVMSUB_5_5(cartpole_QP_solver_Lsd5, cartpole_QP_solver_dvaff5, cartpole_QP_solver_yy4, cartpole_QP_solver_bmy4);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_5(cartpole_QP_solver_Ld4, cartpole_QP_solver_bmy4, cartpole_QP_solver_dvaff4);
cartpole_QP_solver_LA_DENSE_MTVMSUB_5_5(cartpole_QP_solver_Lsd4, cartpole_QP_solver_dvaff4, cartpole_QP_solver_yy3, cartpole_QP_solver_bmy3);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_5(cartpole_QP_solver_Ld3, cartpole_QP_solver_bmy3, cartpole_QP_solver_dvaff3);
cartpole_QP_solver_LA_DENSE_MTVMSUB_5_5(cartpole_QP_solver_Lsd3, cartpole_QP_solver_dvaff3, cartpole_QP_solver_yy2, cartpole_QP_solver_bmy2);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_5(cartpole_QP_solver_Ld2, cartpole_QP_solver_bmy2, cartpole_QP_solver_dvaff2);
cartpole_QP_solver_LA_DENSE_MTVMSUB_5_5(cartpole_QP_solver_Lsd2, cartpole_QP_solver_dvaff2, cartpole_QP_solver_yy1, cartpole_QP_solver_bmy1);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_5(cartpole_QP_solver_Ld1, cartpole_QP_solver_bmy1, cartpole_QP_solver_dvaff1);
cartpole_QP_solver_LA_DENSE_MTVMSUB_5_9(cartpole_QP_solver_Lsd1, cartpole_QP_solver_dvaff1, cartpole_QP_solver_yy0, cartpole_QP_solver_bmy0);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_9(cartpole_QP_solver_Ld0, cartpole_QP_solver_bmy0, cartpole_QP_solver_dvaff0);
cartpole_QP_solver_LA_DENSE_MTVM_9_14(params->C1, cartpole_QP_solver_dvaff0, cartpole_QP_solver_grad_eq0);
cartpole_QP_solver_LA_DENSE_MTVM2_5_14_9(params->C2, cartpole_QP_solver_dvaff1, cartpole_QP_solver_D1, cartpole_QP_solver_dvaff0, cartpole_QP_solver_grad_eq1);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C3, cartpole_QP_solver_dvaff2, cartpole_QP_solver_D2, cartpole_QP_solver_dvaff1, cartpole_QP_solver_grad_eq2);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C4, cartpole_QP_solver_dvaff3, cartpole_QP_solver_D2, cartpole_QP_solver_dvaff2, cartpole_QP_solver_grad_eq3);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C5, cartpole_QP_solver_dvaff4, cartpole_QP_solver_D2, cartpole_QP_solver_dvaff3, cartpole_QP_solver_grad_eq4);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C6, cartpole_QP_solver_dvaff5, cartpole_QP_solver_D2, cartpole_QP_solver_dvaff4, cartpole_QP_solver_grad_eq5);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C7, cartpole_QP_solver_dvaff6, cartpole_QP_solver_D2, cartpole_QP_solver_dvaff5, cartpole_QP_solver_grad_eq6);
cartpole_QP_solver_LA_DIAGZERO_MTVM_5_5(cartpole_QP_solver_D7, cartpole_QP_solver_dvaff6, cartpole_QP_solver_grad_eq7);
cartpole_QP_solver_LA_VSUB2_103(cartpole_QP_solver_rd, cartpole_QP_solver_grad_eq, cartpole_QP_solver_rd);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(cartpole_QP_solver_Phi0, cartpole_QP_solver_rd0, cartpole_QP_solver_dzaff0);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(cartpole_QP_solver_Phi1, cartpole_QP_solver_rd1, cartpole_QP_solver_dzaff1);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(cartpole_QP_solver_Phi2, cartpole_QP_solver_rd2, cartpole_QP_solver_dzaff2);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(cartpole_QP_solver_Phi3, cartpole_QP_solver_rd3, cartpole_QP_solver_dzaff3);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(cartpole_QP_solver_Phi4, cartpole_QP_solver_rd4, cartpole_QP_solver_dzaff4);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(cartpole_QP_solver_Phi5, cartpole_QP_solver_rd5, cartpole_QP_solver_dzaff5);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(cartpole_QP_solver_Phi6, cartpole_QP_solver_rd6, cartpole_QP_solver_dzaff6);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_5(cartpole_QP_solver_Phi7, cartpole_QP_solver_rd7, cartpole_QP_solver_dzaff7);
cartpole_QP_solver_LA_VSUB_INDEXED_14(cartpole_QP_solver_dzaff0, cartpole_QP_solver_lbIdx0, cartpole_QP_solver_rilb0, cartpole_QP_solver_dslbaff0);
cartpole_QP_solver_LA_VSUB3_14(cartpole_QP_solver_llbbyslb0, cartpole_QP_solver_dslbaff0, cartpole_QP_solver_llb0, cartpole_QP_solver_dllbaff0);
cartpole_QP_solver_LA_VSUB2_INDEXED_6(cartpole_QP_solver_riub0, cartpole_QP_solver_dzaff0, cartpole_QP_solver_ubIdx0, cartpole_QP_solver_dsubaff0);
cartpole_QP_solver_LA_VSUB3_6(cartpole_QP_solver_lubbysub0, cartpole_QP_solver_dsubaff0, cartpole_QP_solver_lub0, cartpole_QP_solver_dlubaff0);
cartpole_QP_solver_LA_VSUB_INDEXED_14(cartpole_QP_solver_dzaff1, cartpole_QP_solver_lbIdx1, cartpole_QP_solver_rilb1, cartpole_QP_solver_dslbaff1);
cartpole_QP_solver_LA_VSUB3_14(cartpole_QP_solver_llbbyslb1, cartpole_QP_solver_dslbaff1, cartpole_QP_solver_llb1, cartpole_QP_solver_dllbaff1);
cartpole_QP_solver_LA_VSUB2_INDEXED_6(cartpole_QP_solver_riub1, cartpole_QP_solver_dzaff1, cartpole_QP_solver_ubIdx1, cartpole_QP_solver_dsubaff1);
cartpole_QP_solver_LA_VSUB3_6(cartpole_QP_solver_lubbysub1, cartpole_QP_solver_dsubaff1, cartpole_QP_solver_lub1, cartpole_QP_solver_dlubaff1);
cartpole_QP_solver_LA_VSUB_INDEXED_14(cartpole_QP_solver_dzaff2, cartpole_QP_solver_lbIdx2, cartpole_QP_solver_rilb2, cartpole_QP_solver_dslbaff2);
cartpole_QP_solver_LA_VSUB3_14(cartpole_QP_solver_llbbyslb2, cartpole_QP_solver_dslbaff2, cartpole_QP_solver_llb2, cartpole_QP_solver_dllbaff2);
cartpole_QP_solver_LA_VSUB2_INDEXED_6(cartpole_QP_solver_riub2, cartpole_QP_solver_dzaff2, cartpole_QP_solver_ubIdx2, cartpole_QP_solver_dsubaff2);
cartpole_QP_solver_LA_VSUB3_6(cartpole_QP_solver_lubbysub2, cartpole_QP_solver_dsubaff2, cartpole_QP_solver_lub2, cartpole_QP_solver_dlubaff2);
cartpole_QP_solver_LA_VSUB_INDEXED_14(cartpole_QP_solver_dzaff3, cartpole_QP_solver_lbIdx3, cartpole_QP_solver_rilb3, cartpole_QP_solver_dslbaff3);
cartpole_QP_solver_LA_VSUB3_14(cartpole_QP_solver_llbbyslb3, cartpole_QP_solver_dslbaff3, cartpole_QP_solver_llb3, cartpole_QP_solver_dllbaff3);
cartpole_QP_solver_LA_VSUB2_INDEXED_6(cartpole_QP_solver_riub3, cartpole_QP_solver_dzaff3, cartpole_QP_solver_ubIdx3, cartpole_QP_solver_dsubaff3);
cartpole_QP_solver_LA_VSUB3_6(cartpole_QP_solver_lubbysub3, cartpole_QP_solver_dsubaff3, cartpole_QP_solver_lub3, cartpole_QP_solver_dlubaff3);
cartpole_QP_solver_LA_VSUB_INDEXED_14(cartpole_QP_solver_dzaff4, cartpole_QP_solver_lbIdx4, cartpole_QP_solver_rilb4, cartpole_QP_solver_dslbaff4);
cartpole_QP_solver_LA_VSUB3_14(cartpole_QP_solver_llbbyslb4, cartpole_QP_solver_dslbaff4, cartpole_QP_solver_llb4, cartpole_QP_solver_dllbaff4);
cartpole_QP_solver_LA_VSUB2_INDEXED_6(cartpole_QP_solver_riub4, cartpole_QP_solver_dzaff4, cartpole_QP_solver_ubIdx4, cartpole_QP_solver_dsubaff4);
cartpole_QP_solver_LA_VSUB3_6(cartpole_QP_solver_lubbysub4, cartpole_QP_solver_dsubaff4, cartpole_QP_solver_lub4, cartpole_QP_solver_dlubaff4);
cartpole_QP_solver_LA_VSUB_INDEXED_14(cartpole_QP_solver_dzaff5, cartpole_QP_solver_lbIdx5, cartpole_QP_solver_rilb5, cartpole_QP_solver_dslbaff5);
cartpole_QP_solver_LA_VSUB3_14(cartpole_QP_solver_llbbyslb5, cartpole_QP_solver_dslbaff5, cartpole_QP_solver_llb5, cartpole_QP_solver_dllbaff5);
cartpole_QP_solver_LA_VSUB2_INDEXED_6(cartpole_QP_solver_riub5, cartpole_QP_solver_dzaff5, cartpole_QP_solver_ubIdx5, cartpole_QP_solver_dsubaff5);
cartpole_QP_solver_LA_VSUB3_6(cartpole_QP_solver_lubbysub5, cartpole_QP_solver_dsubaff5, cartpole_QP_solver_lub5, cartpole_QP_solver_dlubaff5);
cartpole_QP_solver_LA_VSUB_INDEXED_14(cartpole_QP_solver_dzaff6, cartpole_QP_solver_lbIdx6, cartpole_QP_solver_rilb6, cartpole_QP_solver_dslbaff6);
cartpole_QP_solver_LA_VSUB3_14(cartpole_QP_solver_llbbyslb6, cartpole_QP_solver_dslbaff6, cartpole_QP_solver_llb6, cartpole_QP_solver_dllbaff6);
cartpole_QP_solver_LA_VSUB2_INDEXED_6(cartpole_QP_solver_riub6, cartpole_QP_solver_dzaff6, cartpole_QP_solver_ubIdx6, cartpole_QP_solver_dsubaff6);
cartpole_QP_solver_LA_VSUB3_6(cartpole_QP_solver_lubbysub6, cartpole_QP_solver_dsubaff6, cartpole_QP_solver_lub6, cartpole_QP_solver_dlubaff6);
cartpole_QP_solver_LA_VSUB_INDEXED_5(cartpole_QP_solver_dzaff7, cartpole_QP_solver_lbIdx7, cartpole_QP_solver_rilb7, cartpole_QP_solver_dslbaff7);
cartpole_QP_solver_LA_VSUB3_5(cartpole_QP_solver_llbbyslb7, cartpole_QP_solver_dslbaff7, cartpole_QP_solver_llb7, cartpole_QP_solver_dllbaff7);
cartpole_QP_solver_LA_VSUB2_INDEXED_5(cartpole_QP_solver_riub7, cartpole_QP_solver_dzaff7, cartpole_QP_solver_ubIdx7, cartpole_QP_solver_dsubaff7);
cartpole_QP_solver_LA_VSUB3_5(cartpole_QP_solver_lubbysub7, cartpole_QP_solver_dsubaff7, cartpole_QP_solver_lub7, cartpole_QP_solver_dlubaff7);
info->lsit_aff = cartpole_QP_solver_LINESEARCH_BACKTRACKING_AFFINE(cartpole_QP_solver_l, cartpole_QP_solver_s, cartpole_QP_solver_dl_aff, cartpole_QP_solver_ds_aff, &info->step_aff, &info->mu_aff);
if( info->lsit_aff == cartpole_QP_solver_NOPROGRESS ){
exitcode = cartpole_QP_solver_NOPROGRESS; break;
}
sigma_3rdroot = info->mu_aff / info->mu;
info->sigma = sigma_3rdroot*sigma_3rdroot*sigma_3rdroot;
musigma = info->mu * info->sigma;
cartpole_QP_solver_LA_VSUB5_150(cartpole_QP_solver_ds_aff, cartpole_QP_solver_dl_aff, info->mu, info->sigma, cartpole_QP_solver_ccrhs);
cartpole_QP_solver_LA_VSUB6_INDEXED_14_6_14(cartpole_QP_solver_ccrhsub0, cartpole_QP_solver_sub0, cartpole_QP_solver_ubIdx0, cartpole_QP_solver_ccrhsl0, cartpole_QP_solver_slb0, cartpole_QP_solver_lbIdx0, cartpole_QP_solver_rd0);
cartpole_QP_solver_LA_VSUB6_INDEXED_14_6_14(cartpole_QP_solver_ccrhsub1, cartpole_QP_solver_sub1, cartpole_QP_solver_ubIdx1, cartpole_QP_solver_ccrhsl1, cartpole_QP_solver_slb1, cartpole_QP_solver_lbIdx1, cartpole_QP_solver_rd1);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_14(cartpole_QP_solver_Phi0, cartpole_QP_solver_rd0, cartpole_QP_solver_Lbyrd0);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_14(cartpole_QP_solver_Phi1, cartpole_QP_solver_rd1, cartpole_QP_solver_Lbyrd1);
cartpole_QP_solver_LA_DENSE_2MVMADD_9_14_14(cartpole_QP_solver_V0, cartpole_QP_solver_Lbyrd0, cartpole_QP_solver_W1, cartpole_QP_solver_Lbyrd1, cartpole_QP_solver_beta0);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_9(cartpole_QP_solver_Ld0, cartpole_QP_solver_beta0, cartpole_QP_solver_yy0);
cartpole_QP_solver_LA_VSUB6_INDEXED_14_6_14(cartpole_QP_solver_ccrhsub2, cartpole_QP_solver_sub2, cartpole_QP_solver_ubIdx2, cartpole_QP_solver_ccrhsl2, cartpole_QP_solver_slb2, cartpole_QP_solver_lbIdx2, cartpole_QP_solver_rd2);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_14(cartpole_QP_solver_Phi2, cartpole_QP_solver_rd2, cartpole_QP_solver_Lbyrd2);
cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_14(cartpole_QP_solver_V1, cartpole_QP_solver_Lbyrd1, cartpole_QP_solver_W2, cartpole_QP_solver_Lbyrd2, cartpole_QP_solver_beta1);
cartpole_QP_solver_LA_DENSE_MVMSUB1_5_9(cartpole_QP_solver_Lsd1, cartpole_QP_solver_yy0, cartpole_QP_solver_beta1, cartpole_QP_solver_bmy1);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_5(cartpole_QP_solver_Ld1, cartpole_QP_solver_bmy1, cartpole_QP_solver_yy1);
cartpole_QP_solver_LA_VSUB6_INDEXED_14_6_14(cartpole_QP_solver_ccrhsub3, cartpole_QP_solver_sub3, cartpole_QP_solver_ubIdx3, cartpole_QP_solver_ccrhsl3, cartpole_QP_solver_slb3, cartpole_QP_solver_lbIdx3, cartpole_QP_solver_rd3);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_14(cartpole_QP_solver_Phi3, cartpole_QP_solver_rd3, cartpole_QP_solver_Lbyrd3);
cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_14(cartpole_QP_solver_V2, cartpole_QP_solver_Lbyrd2, cartpole_QP_solver_W3, cartpole_QP_solver_Lbyrd3, cartpole_QP_solver_beta2);
cartpole_QP_solver_LA_DENSE_MVMSUB1_5_5(cartpole_QP_solver_Lsd2, cartpole_QP_solver_yy1, cartpole_QP_solver_beta2, cartpole_QP_solver_bmy2);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_5(cartpole_QP_solver_Ld2, cartpole_QP_solver_bmy2, cartpole_QP_solver_yy2);
cartpole_QP_solver_LA_VSUB6_INDEXED_14_6_14(cartpole_QP_solver_ccrhsub4, cartpole_QP_solver_sub4, cartpole_QP_solver_ubIdx4, cartpole_QP_solver_ccrhsl4, cartpole_QP_solver_slb4, cartpole_QP_solver_lbIdx4, cartpole_QP_solver_rd4);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_14(cartpole_QP_solver_Phi4, cartpole_QP_solver_rd4, cartpole_QP_solver_Lbyrd4);
cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_14(cartpole_QP_solver_V3, cartpole_QP_solver_Lbyrd3, cartpole_QP_solver_W4, cartpole_QP_solver_Lbyrd4, cartpole_QP_solver_beta3);
cartpole_QP_solver_LA_DENSE_MVMSUB1_5_5(cartpole_QP_solver_Lsd3, cartpole_QP_solver_yy2, cartpole_QP_solver_beta3, cartpole_QP_solver_bmy3);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_5(cartpole_QP_solver_Ld3, cartpole_QP_solver_bmy3, cartpole_QP_solver_yy3);
cartpole_QP_solver_LA_VSUB6_INDEXED_14_6_14(cartpole_QP_solver_ccrhsub5, cartpole_QP_solver_sub5, cartpole_QP_solver_ubIdx5, cartpole_QP_solver_ccrhsl5, cartpole_QP_solver_slb5, cartpole_QP_solver_lbIdx5, cartpole_QP_solver_rd5);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_14(cartpole_QP_solver_Phi5, cartpole_QP_solver_rd5, cartpole_QP_solver_Lbyrd5);
cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_14(cartpole_QP_solver_V4, cartpole_QP_solver_Lbyrd4, cartpole_QP_solver_W5, cartpole_QP_solver_Lbyrd5, cartpole_QP_solver_beta4);
cartpole_QP_solver_LA_DENSE_MVMSUB1_5_5(cartpole_QP_solver_Lsd4, cartpole_QP_solver_yy3, cartpole_QP_solver_beta4, cartpole_QP_solver_bmy4);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_5(cartpole_QP_solver_Ld4, cartpole_QP_solver_bmy4, cartpole_QP_solver_yy4);
cartpole_QP_solver_LA_VSUB6_INDEXED_14_6_14(cartpole_QP_solver_ccrhsub6, cartpole_QP_solver_sub6, cartpole_QP_solver_ubIdx6, cartpole_QP_solver_ccrhsl6, cartpole_QP_solver_slb6, cartpole_QP_solver_lbIdx6, cartpole_QP_solver_rd6);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_14(cartpole_QP_solver_Phi6, cartpole_QP_solver_rd6, cartpole_QP_solver_Lbyrd6);
cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_14(cartpole_QP_solver_V5, cartpole_QP_solver_Lbyrd5, cartpole_QP_solver_W6, cartpole_QP_solver_Lbyrd6, cartpole_QP_solver_beta5);
cartpole_QP_solver_LA_DENSE_MVMSUB1_5_5(cartpole_QP_solver_Lsd5, cartpole_QP_solver_yy4, cartpole_QP_solver_beta5, cartpole_QP_solver_bmy5);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_5(cartpole_QP_solver_Ld5, cartpole_QP_solver_bmy5, cartpole_QP_solver_yy5);
cartpole_QP_solver_LA_VSUB6_INDEXED_5_5_5(cartpole_QP_solver_ccrhsub7, cartpole_QP_solver_sub7, cartpole_QP_solver_ubIdx7, cartpole_QP_solver_ccrhsl7, cartpole_QP_solver_slb7, cartpole_QP_solver_lbIdx7, cartpole_QP_solver_rd7);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_5(cartpole_QP_solver_Phi7, cartpole_QP_solver_rd7, cartpole_QP_solver_Lbyrd7);
cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_5(cartpole_QP_solver_V6, cartpole_QP_solver_Lbyrd6, cartpole_QP_solver_W7, cartpole_QP_solver_Lbyrd7, cartpole_QP_solver_beta6);
cartpole_QP_solver_LA_DENSE_MVMSUB1_5_5(cartpole_QP_solver_Lsd6, cartpole_QP_solver_yy5, cartpole_QP_solver_beta6, cartpole_QP_solver_bmy6);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_5(cartpole_QP_solver_Ld6, cartpole_QP_solver_bmy6, cartpole_QP_solver_yy6);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_5(cartpole_QP_solver_Ld6, cartpole_QP_solver_yy6, cartpole_QP_solver_dvcc6);
cartpole_QP_solver_LA_DENSE_MTVMSUB_5_5(cartpole_QP_solver_Lsd6, cartpole_QP_solver_dvcc6, cartpole_QP_solver_yy5, cartpole_QP_solver_bmy5);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_5(cartpole_QP_solver_Ld5, cartpole_QP_solver_bmy5, cartpole_QP_solver_dvcc5);
cartpole_QP_solver_LA_DENSE_MTVMSUB_5_5(cartpole_QP_solver_Lsd5, cartpole_QP_solver_dvcc5, cartpole_QP_solver_yy4, cartpole_QP_solver_bmy4);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_5(cartpole_QP_solver_Ld4, cartpole_QP_solver_bmy4, cartpole_QP_solver_dvcc4);
cartpole_QP_solver_LA_DENSE_MTVMSUB_5_5(cartpole_QP_solver_Lsd4, cartpole_QP_solver_dvcc4, cartpole_QP_solver_yy3, cartpole_QP_solver_bmy3);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_5(cartpole_QP_solver_Ld3, cartpole_QP_solver_bmy3, cartpole_QP_solver_dvcc3);
cartpole_QP_solver_LA_DENSE_MTVMSUB_5_5(cartpole_QP_solver_Lsd3, cartpole_QP_solver_dvcc3, cartpole_QP_solver_yy2, cartpole_QP_solver_bmy2);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_5(cartpole_QP_solver_Ld2, cartpole_QP_solver_bmy2, cartpole_QP_solver_dvcc2);
cartpole_QP_solver_LA_DENSE_MTVMSUB_5_5(cartpole_QP_solver_Lsd2, cartpole_QP_solver_dvcc2, cartpole_QP_solver_yy1, cartpole_QP_solver_bmy1);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_5(cartpole_QP_solver_Ld1, cartpole_QP_solver_bmy1, cartpole_QP_solver_dvcc1);
cartpole_QP_solver_LA_DENSE_MTVMSUB_5_9(cartpole_QP_solver_Lsd1, cartpole_QP_solver_dvcc1, cartpole_QP_solver_yy0, cartpole_QP_solver_bmy0);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_9(cartpole_QP_solver_Ld0, cartpole_QP_solver_bmy0, cartpole_QP_solver_dvcc0);
cartpole_QP_solver_LA_DENSE_MTVM_9_14(params->C1, cartpole_QP_solver_dvcc0, cartpole_QP_solver_grad_eq0);
cartpole_QP_solver_LA_DENSE_MTVM2_5_14_9(params->C2, cartpole_QP_solver_dvcc1, cartpole_QP_solver_D1, cartpole_QP_solver_dvcc0, cartpole_QP_solver_grad_eq1);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C3, cartpole_QP_solver_dvcc2, cartpole_QP_solver_D2, cartpole_QP_solver_dvcc1, cartpole_QP_solver_grad_eq2);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C4, cartpole_QP_solver_dvcc3, cartpole_QP_solver_D2, cartpole_QP_solver_dvcc2, cartpole_QP_solver_grad_eq3);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C5, cartpole_QP_solver_dvcc4, cartpole_QP_solver_D2, cartpole_QP_solver_dvcc3, cartpole_QP_solver_grad_eq4);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C6, cartpole_QP_solver_dvcc5, cartpole_QP_solver_D2, cartpole_QP_solver_dvcc4, cartpole_QP_solver_grad_eq5);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C7, cartpole_QP_solver_dvcc6, cartpole_QP_solver_D2, cartpole_QP_solver_dvcc5, cartpole_QP_solver_grad_eq6);
cartpole_QP_solver_LA_DIAGZERO_MTVM_5_5(cartpole_QP_solver_D7, cartpole_QP_solver_dvcc6, cartpole_QP_solver_grad_eq7);
cartpole_QP_solver_LA_VSUB_103(cartpole_QP_solver_rd, cartpole_QP_solver_grad_eq, cartpole_QP_solver_rd);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(cartpole_QP_solver_Phi0, cartpole_QP_solver_rd0, cartpole_QP_solver_dzcc0);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(cartpole_QP_solver_Phi1, cartpole_QP_solver_rd1, cartpole_QP_solver_dzcc1);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(cartpole_QP_solver_Phi2, cartpole_QP_solver_rd2, cartpole_QP_solver_dzcc2);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(cartpole_QP_solver_Phi3, cartpole_QP_solver_rd3, cartpole_QP_solver_dzcc3);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(cartpole_QP_solver_Phi4, cartpole_QP_solver_rd4, cartpole_QP_solver_dzcc4);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(cartpole_QP_solver_Phi5, cartpole_QP_solver_rd5, cartpole_QP_solver_dzcc5);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(cartpole_QP_solver_Phi6, cartpole_QP_solver_rd6, cartpole_QP_solver_dzcc6);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_5(cartpole_QP_solver_Phi7, cartpole_QP_solver_rd7, cartpole_QP_solver_dzcc7);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(cartpole_QP_solver_ccrhsl0, cartpole_QP_solver_slb0, cartpole_QP_solver_llbbyslb0, cartpole_QP_solver_dzcc0, cartpole_QP_solver_lbIdx0, cartpole_QP_solver_dllbcc0);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(cartpole_QP_solver_ccrhsub0, cartpole_QP_solver_sub0, cartpole_QP_solver_lubbysub0, cartpole_QP_solver_dzcc0, cartpole_QP_solver_ubIdx0, cartpole_QP_solver_dlubcc0);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(cartpole_QP_solver_ccrhsl1, cartpole_QP_solver_slb1, cartpole_QP_solver_llbbyslb1, cartpole_QP_solver_dzcc1, cartpole_QP_solver_lbIdx1, cartpole_QP_solver_dllbcc1);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(cartpole_QP_solver_ccrhsub1, cartpole_QP_solver_sub1, cartpole_QP_solver_lubbysub1, cartpole_QP_solver_dzcc1, cartpole_QP_solver_ubIdx1, cartpole_QP_solver_dlubcc1);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(cartpole_QP_solver_ccrhsl2, cartpole_QP_solver_slb2, cartpole_QP_solver_llbbyslb2, cartpole_QP_solver_dzcc2, cartpole_QP_solver_lbIdx2, cartpole_QP_solver_dllbcc2);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(cartpole_QP_solver_ccrhsub2, cartpole_QP_solver_sub2, cartpole_QP_solver_lubbysub2, cartpole_QP_solver_dzcc2, cartpole_QP_solver_ubIdx2, cartpole_QP_solver_dlubcc2);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(cartpole_QP_solver_ccrhsl3, cartpole_QP_solver_slb3, cartpole_QP_solver_llbbyslb3, cartpole_QP_solver_dzcc3, cartpole_QP_solver_lbIdx3, cartpole_QP_solver_dllbcc3);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(cartpole_QP_solver_ccrhsub3, cartpole_QP_solver_sub3, cartpole_QP_solver_lubbysub3, cartpole_QP_solver_dzcc3, cartpole_QP_solver_ubIdx3, cartpole_QP_solver_dlubcc3);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(cartpole_QP_solver_ccrhsl4, cartpole_QP_solver_slb4, cartpole_QP_solver_llbbyslb4, cartpole_QP_solver_dzcc4, cartpole_QP_solver_lbIdx4, cartpole_QP_solver_dllbcc4);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(cartpole_QP_solver_ccrhsub4, cartpole_QP_solver_sub4, cartpole_QP_solver_lubbysub4, cartpole_QP_solver_dzcc4, cartpole_QP_solver_ubIdx4, cartpole_QP_solver_dlubcc4);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(cartpole_QP_solver_ccrhsl5, cartpole_QP_solver_slb5, cartpole_QP_solver_llbbyslb5, cartpole_QP_solver_dzcc5, cartpole_QP_solver_lbIdx5, cartpole_QP_solver_dllbcc5);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(cartpole_QP_solver_ccrhsub5, cartpole_QP_solver_sub5, cartpole_QP_solver_lubbysub5, cartpole_QP_solver_dzcc5, cartpole_QP_solver_ubIdx5, cartpole_QP_solver_dlubcc5);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(cartpole_QP_solver_ccrhsl6, cartpole_QP_solver_slb6, cartpole_QP_solver_llbbyslb6, cartpole_QP_solver_dzcc6, cartpole_QP_solver_lbIdx6, cartpole_QP_solver_dllbcc6);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(cartpole_QP_solver_ccrhsub6, cartpole_QP_solver_sub6, cartpole_QP_solver_lubbysub6, cartpole_QP_solver_dzcc6, cartpole_QP_solver_ubIdx6, cartpole_QP_solver_dlubcc6);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_5(cartpole_QP_solver_ccrhsl7, cartpole_QP_solver_slb7, cartpole_QP_solver_llbbyslb7, cartpole_QP_solver_dzcc7, cartpole_QP_solver_lbIdx7, cartpole_QP_solver_dllbcc7);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_5(cartpole_QP_solver_ccrhsub7, cartpole_QP_solver_sub7, cartpole_QP_solver_lubbysub7, cartpole_QP_solver_dzcc7, cartpole_QP_solver_ubIdx7, cartpole_QP_solver_dlubcc7);
cartpole_QP_solver_LA_VSUB7_150(cartpole_QP_solver_l, cartpole_QP_solver_ccrhs, cartpole_QP_solver_s, cartpole_QP_solver_dl_cc, cartpole_QP_solver_ds_cc);
cartpole_QP_solver_LA_VADD_103(cartpole_QP_solver_dz_cc, cartpole_QP_solver_dz_aff);
cartpole_QP_solver_LA_VADD_39(cartpole_QP_solver_dv_cc, cartpole_QP_solver_dv_aff);
cartpole_QP_solver_LA_VADD_150(cartpole_QP_solver_dl_cc, cartpole_QP_solver_dl_aff);
cartpole_QP_solver_LA_VADD_150(cartpole_QP_solver_ds_cc, cartpole_QP_solver_ds_aff);
info->lsit_cc = cartpole_QP_solver_LINESEARCH_BACKTRACKING_COMBINED(cartpole_QP_solver_z, cartpole_QP_solver_v, cartpole_QP_solver_l, cartpole_QP_solver_s, cartpole_QP_solver_dz_cc, cartpole_QP_solver_dv_cc, cartpole_QP_solver_dl_cc, cartpole_QP_solver_ds_cc, &info->step_cc, &info->mu);
if( info->lsit_cc == cartpole_QP_solver_NOPROGRESS ){
exitcode = cartpole_QP_solver_NOPROGRESS; break;
}
info->it++;
}
output->z1[0] = cartpole_QP_solver_z0[0];
output->z1[1] = cartpole_QP_solver_z0[1];
output->z1[2] = cartpole_QP_solver_z0[2];
output->z1[3] = cartpole_QP_solver_z0[3];
output->z1[4] = cartpole_QP_solver_z0[4];
output->z1[5] = cartpole_QP_solver_z0[5];
output->z2[0] = cartpole_QP_solver_z1[0];
output->z2[1] = cartpole_QP_solver_z1[1];
output->z2[2] = cartpole_QP_solver_z1[2];
output->z2[3] = cartpole_QP_solver_z1[3];
output->z2[4] = cartpole_QP_solver_z1[4];
output->z2[5] = cartpole_QP_solver_z1[5];
output->z3[0] = cartpole_QP_solver_z2[0];
output->z3[1] = cartpole_QP_solver_z2[1];
output->z3[2] = cartpole_QP_solver_z2[2];
output->z3[3] = cartpole_QP_solver_z2[3];
output->z3[4] = cartpole_QP_solver_z2[4];
output->z3[5] = cartpole_QP_solver_z2[5];
output->z4[0] = cartpole_QP_solver_z3[0];
output->z4[1] = cartpole_QP_solver_z3[1];
output->z4[2] = cartpole_QP_solver_z3[2];
output->z4[3] = cartpole_QP_solver_z3[3];
output->z4[4] = cartpole_QP_solver_z3[4];
output->z4[5] = cartpole_QP_solver_z3[5];
output->z5[0] = cartpole_QP_solver_z4[0];
output->z5[1] = cartpole_QP_solver_z4[1];
output->z5[2] = cartpole_QP_solver_z4[2];
output->z5[3] = cartpole_QP_solver_z4[3];
output->z5[4] = cartpole_QP_solver_z4[4];
output->z5[5] = cartpole_QP_solver_z4[5];
output->z6[0] = cartpole_QP_solver_z5[0];
output->z6[1] = cartpole_QP_solver_z5[1];
output->z6[2] = cartpole_QP_solver_z5[2];
output->z6[3] = cartpole_QP_solver_z5[3];
output->z6[4] = cartpole_QP_solver_z5[4];
output->z6[5] = cartpole_QP_solver_z5[5];
output->z7[0] = cartpole_QP_solver_z6[0];
output->z7[1] = cartpole_QP_solver_z6[1];
output->z7[2] = cartpole_QP_solver_z6[2];
output->z7[3] = cartpole_QP_solver_z6[3];
output->z7[4] = cartpole_QP_solver_z6[4];
output->z7[5] = cartpole_QP_solver_z6[5];
output->z8[0] = cartpole_QP_solver_z7[0];
output->z8[1] = cartpole_QP_solver_z7[1];
output->z8[2] = cartpole_QP_solver_z7[2];
output->z8[3] = cartpole_QP_solver_z7[3];
output->z8[4] = cartpole_QP_solver_z7[4];

#if cartpole_QP_solver_SET_TIMING == 1
info->solvetime = cartpole_QP_solver_toc(&solvertimer);
#if cartpole_QP_solver_SET_PRINTLEVEL > 0 && cartpole_QP_solver_SET_TIMING == 1
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
