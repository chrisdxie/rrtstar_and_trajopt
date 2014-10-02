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

#include "../include/acrobot_QP_solver.h"

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

typedef struct acrobot_QP_solver_timer{
	LARGE_INTEGER tic;
	LARGE_INTEGER toc;
	LARGE_INTEGER freq;
} acrobot_QP_solver_timer;


void acrobot_QP_solver_tic(acrobot_QP_solver_timer* t)
{
	QueryPerformanceFrequency(&t->freq);
	QueryPerformanceCounter(&t->tic);
}



acrobot_QP_solver_FLOAT acrobot_QP_solver_toc(acrobot_QP_solver_timer* t)
{
	QueryPerformanceCounter(&t->toc);
	return ((t->toc.QuadPart - t->tic.QuadPart) / (acrobot_QP_solver_FLOAT)t->freq.QuadPart);
}


/* WE ARE ON THE MAC */
#elif (defined __APPLE__)
#include <mach/mach_time.h>


/* Use MAC OSX  mach_time for timing */
typedef struct acrobot_QP_solver_timer{
	uint64_t tic;
	uint64_t toc;
	mach_timebase_info_data_t tinfo;

} acrobot_QP_solver_timer;


void acrobot_QP_solver_tic(acrobot_QP_solver_timer* t)
{
    /* read current clock cycles */
    t->tic = mach_absolute_time();
}



acrobot_QP_solver_FLOAT acrobot_QP_solver_toc(acrobot_QP_solver_timer* t)
{
    uint64_t duration; /* elapsed time in clock cycles*/
    t->toc = mach_absolute_time();
	duration = t->toc - t->tic;

    /*conversion from clock cycles to nanoseconds*/
    mach_timebase_info(&(t->tinfo));
    duration *= t->tinfo.numer;
    duration /= t->tinfo.denom;

    return (acrobot_QP_solver_FLOAT)duration / 1000000000;
}

/* WE ARE ON SOME TEXAS INSTRUMENTS PLATFORM */
#elif (defined __TI_COMPILER_VERSION__)

/* TimeStamps */
#include <c6x.h> /* make use of TSCL, TSCH */


typedef struct acrobot_QP_solver_timer{
	unsigned long long tic;
	unsigned long long toc;
} acrobot_QP_solver_timer;


void acrobot_QP_solver_tic(acrobot_QP_solver_timer* t)
{
	TSCL = 0;	/* Initiate CPU timer by writing any val to TSCL */
	t->tic = _itoll( TSCH, TSCL );
}



acrobot_QP_solver_FLOAT acrobot_QP_solver_toc(acrobot_QP_solver_timer* t)
{
	t->toc = _itoll( TSCH, TSCL );
	unsigned long long t0;
	unsigned long long overhead;
	t0 = _itoll( TSCH, TSCL );
	overhead = _itoll( TSCH, TSCL )  - t0;

	return (acrobot_QP_solver_FLOAT)(t->toc - t->tic - overhead) / 1000000000;
}



/* WE ARE ON SOME OTHER UNIX/LINUX SYSTEM */
#else

/* Use POSIX clocl_gettime() for timing on non-Windows machines */
#include <time.h>
typedef struct acrobot_QP_solver_timer{
	struct timespec tic;
	struct timespec toc;
} acrobot_QP_solver_timer;


/* read current time */
void acrobot_QP_solver_tic(acrobot_QP_solver_timer* t)
{
	clock_gettime(CLOCK_MONOTONIC, &t->tic);
}



/* return time passed since last call to tic on this timer */
double acrobot_QP_solver_toc(acrobot_QP_solver_timer* t)
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

	return (acrobot_QP_solver_FLOAT)temp.tv_sec + (acrobot_QP_solver_FLOAT)temp.tv_nsec / 1000000000;
}


#endif

/* LINEAR ALGEBRA LIBRARY ---------------------------------------------- */
/*
 * Initializes a vector of length 131 with a value.
 */
void acrobot_QP_solver_LA_INITIALIZEVECTOR_131(acrobot_QP_solver_FLOAT* vec, acrobot_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<131; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 49 with a value.
 */
void acrobot_QP_solver_LA_INITIALIZEVECTOR_49(acrobot_QP_solver_FLOAT* vec, acrobot_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<49; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 190 with a value.
 */
void acrobot_QP_solver_LA_INITIALIZEVECTOR_190(acrobot_QP_solver_FLOAT* vec, acrobot_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<190; i++ )
	{
		vec[i] = value;
	}
}


/* 
 * Calculates a dot product and adds it to a variable: z += x'*y; 
 * This function is for vectors of length 190.
 */
void acrobot_QP_solver_LA_DOTACC_190(acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *y, acrobot_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<190; i++ ){
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
void acrobot_QP_solver_LA_DIAG_QUADFCN_14(acrobot_QP_solver_FLOAT* H, acrobot_QP_solver_FLOAT* f, acrobot_QP_solver_FLOAT* z, acrobot_QP_solver_FLOAT* grad, acrobot_QP_solver_FLOAT* value)
{
	int i;
	acrobot_QP_solver_FLOAT hz;	
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
void acrobot_QP_solver_LA_DIAG_QUADFCN_5(acrobot_QP_solver_FLOAT* H, acrobot_QP_solver_FLOAT* f, acrobot_QP_solver_FLOAT* z, acrobot_QP_solver_FLOAT* grad, acrobot_QP_solver_FLOAT* value)
{
	int i;
	acrobot_QP_solver_FLOAT hz;	
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
void acrobot_QP_solver_LA_DENSE_MVMSUB3_9_14_14(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *b, acrobot_QP_solver_FLOAT *l, acrobot_QP_solver_FLOAT *r, acrobot_QP_solver_FLOAT *z, acrobot_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;
	acrobot_QP_solver_FLOAT AxBu[9];
	acrobot_QP_solver_FLOAT norm = *y;
	acrobot_QP_solver_FLOAT lr = 0;

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
void acrobot_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_14(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *b, acrobot_QP_solver_FLOAT *l, acrobot_QP_solver_FLOAT *r, acrobot_QP_solver_FLOAT *z, acrobot_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	acrobot_QP_solver_FLOAT AxBu[5];
	acrobot_QP_solver_FLOAT norm = *y;
	acrobot_QP_solver_FLOAT lr = 0;

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
void acrobot_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_5(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *b, acrobot_QP_solver_FLOAT *l, acrobot_QP_solver_FLOAT *r, acrobot_QP_solver_FLOAT *z, acrobot_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	acrobot_QP_solver_FLOAT AxBu[5];
	acrobot_QP_solver_FLOAT norm = *y;
	acrobot_QP_solver_FLOAT lr = 0;

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
void acrobot_QP_solver_LA_DENSE_MTVM_9_14(acrobot_QP_solver_FLOAT *M, acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *y)
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
void acrobot_QP_solver_LA_DENSE_MTVM2_5_14_9(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *y, acrobot_QP_solver_FLOAT *z)
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
void acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *y, acrobot_QP_solver_FLOAT *z)
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
void acrobot_QP_solver_LA_DIAGZERO_MTVM_5_5(acrobot_QP_solver_FLOAT *M, acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *y)
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
void acrobot_QP_solver_LA_VSUBADD3_14(acrobot_QP_solver_FLOAT* t, acrobot_QP_solver_FLOAT* u, int* uidx, acrobot_QP_solver_FLOAT* v, acrobot_QP_solver_FLOAT* w, acrobot_QP_solver_FLOAT* y, acrobot_QP_solver_FLOAT* z, acrobot_QP_solver_FLOAT* r)
{
	int i;
	acrobot_QP_solver_FLOAT norm = *r;
	acrobot_QP_solver_FLOAT vx = 0;
	acrobot_QP_solver_FLOAT x;
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
void acrobot_QP_solver_LA_VSUBADD2_6(acrobot_QP_solver_FLOAT* t, int* tidx, acrobot_QP_solver_FLOAT* u, acrobot_QP_solver_FLOAT* v, acrobot_QP_solver_FLOAT* w, acrobot_QP_solver_FLOAT* y, acrobot_QP_solver_FLOAT* z, acrobot_QP_solver_FLOAT* r)
{
	int i;
	acrobot_QP_solver_FLOAT norm = *r;
	acrobot_QP_solver_FLOAT vx = 0;
	acrobot_QP_solver_FLOAT x;
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
void acrobot_QP_solver_LA_VSUBADD3_5(acrobot_QP_solver_FLOAT* t, acrobot_QP_solver_FLOAT* u, int* uidx, acrobot_QP_solver_FLOAT* v, acrobot_QP_solver_FLOAT* w, acrobot_QP_solver_FLOAT* y, acrobot_QP_solver_FLOAT* z, acrobot_QP_solver_FLOAT* r)
{
	int i;
	acrobot_QP_solver_FLOAT norm = *r;
	acrobot_QP_solver_FLOAT vx = 0;
	acrobot_QP_solver_FLOAT x;
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
void acrobot_QP_solver_LA_VSUBADD2_5(acrobot_QP_solver_FLOAT* t, int* tidx, acrobot_QP_solver_FLOAT* u, acrobot_QP_solver_FLOAT* v, acrobot_QP_solver_FLOAT* w, acrobot_QP_solver_FLOAT* y, acrobot_QP_solver_FLOAT* z, acrobot_QP_solver_FLOAT* r)
{
	int i;
	acrobot_QP_solver_FLOAT norm = *r;
	acrobot_QP_solver_FLOAT vx = 0;
	acrobot_QP_solver_FLOAT x;
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
void acrobot_QP_solver_LA_INEQ_B_GRAD_14_14_6(acrobot_QP_solver_FLOAT *lu, acrobot_QP_solver_FLOAT *su, acrobot_QP_solver_FLOAT *ru, acrobot_QP_solver_FLOAT *ll, acrobot_QP_solver_FLOAT *sl, acrobot_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, acrobot_QP_solver_FLOAT *grad, acrobot_QP_solver_FLOAT *lubysu, acrobot_QP_solver_FLOAT *llbysl)
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
void acrobot_QP_solver_LA_INEQ_B_GRAD_5_5_5(acrobot_QP_solver_FLOAT *lu, acrobot_QP_solver_FLOAT *su, acrobot_QP_solver_FLOAT *ru, acrobot_QP_solver_FLOAT *ll, acrobot_QP_solver_FLOAT *sl, acrobot_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, acrobot_QP_solver_FLOAT *grad, acrobot_QP_solver_FLOAT *lubysu, acrobot_QP_solver_FLOAT *llbysl)
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
 * of length 131.
 */
void acrobot_QP_solver_LA_VVADD3_131(acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *v, acrobot_QP_solver_FLOAT *w, acrobot_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<131; i++ ){
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
void acrobot_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(acrobot_QP_solver_FLOAT *H, acrobot_QP_solver_FLOAT *llbysl, int* lbIdx, acrobot_QP_solver_FLOAT *lubysu, int* ubIdx, acrobot_QP_solver_FLOAT *Phi)


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
#if acrobot_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
void acrobot_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_14(acrobot_QP_solver_FLOAT *L, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *A)
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
void acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_FLOAT *L, acrobot_QP_solver_FLOAT *b, acrobot_QP_solver_FLOAT *y)
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
void acrobot_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_14(acrobot_QP_solver_FLOAT *L, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *A)
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
void acrobot_QP_solver_LA_DENSE_MMTM_9_14_5(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *C)
{
    int i, j, k;
    acrobot_QP_solver_FLOAT temp;
    
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
void acrobot_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_14(acrobot_QP_solver_FLOAT *L, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *A)
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
void acrobot_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_14_5(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *C)
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
void acrobot_QP_solver_LA_DIAG_CHOL_ONELOOP_LBUB_5_5_5(acrobot_QP_solver_FLOAT *H, acrobot_QP_solver_FLOAT *llbysl, int* lbIdx, acrobot_QP_solver_FLOAT *lubysu, int* ubIdx, acrobot_QP_solver_FLOAT *Phi)


{
	int i;
	
	/* compute cholesky */
	for( i=0; i<5; i++ ){
		Phi[i] = H[i] + llbysl[i] + lubysu[i];

#if acrobot_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
void acrobot_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_5(acrobot_QP_solver_FLOAT *L, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *A)
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
void acrobot_QP_solver_LA_DIAG_FORWARDSUB_5(acrobot_QP_solver_FLOAT *L, acrobot_QP_solver_FLOAT *b, acrobot_QP_solver_FLOAT *y)
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
void acrobot_QP_solver_LA_DENSE_MMT2_9_14_14(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    acrobot_QP_solver_FLOAT ltemp;
    
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
void acrobot_QP_solver_LA_DENSE_MVMSUB2_9_14_14(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *b, acrobot_QP_solver_FLOAT *r)
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
void acrobot_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_14(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    acrobot_QP_solver_FLOAT ltemp;
    
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
void acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_14(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *b, acrobot_QP_solver_FLOAT *r)
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
void acrobot_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_5(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    acrobot_QP_solver_FLOAT ltemp;
    
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
void acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_5(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *b, acrobot_QP_solver_FLOAT *r)
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
void acrobot_QP_solver_LA_DENSE_CHOL_9(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    acrobot_QP_solver_FLOAT l;
    acrobot_QP_solver_FLOAT Mii;

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
        
#if acrobot_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
void acrobot_QP_solver_LA_DENSE_FORWARDSUB_9(acrobot_QP_solver_FLOAT *L, acrobot_QP_solver_FLOAT *b, acrobot_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    acrobot_QP_solver_FLOAT yel;
            
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
void acrobot_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_9(acrobot_QP_solver_FLOAT *L, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *A)
{
    int i,j,k,ii,di;
    acrobot_QP_solver_FLOAT a;
    
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
void acrobot_QP_solver_LA_DENSE_MMTSUB_5_9(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    acrobot_QP_solver_FLOAT ltemp;
    
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
void acrobot_QP_solver_LA_DENSE_CHOL_5(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    acrobot_QP_solver_FLOAT l;
    acrobot_QP_solver_FLOAT Mii;

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
        
#if acrobot_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
void acrobot_QP_solver_LA_DENSE_MVMSUB1_5_9(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *b, acrobot_QP_solver_FLOAT *r)
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
void acrobot_QP_solver_LA_DENSE_FORWARDSUB_5(acrobot_QP_solver_FLOAT *L, acrobot_QP_solver_FLOAT *b, acrobot_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    acrobot_QP_solver_FLOAT yel;
            
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
void acrobot_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(acrobot_QP_solver_FLOAT *L, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *A)
{
    int i,j,k,ii,di;
    acrobot_QP_solver_FLOAT a;
    
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
void acrobot_QP_solver_LA_DENSE_MMTSUB_5_5(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    acrobot_QP_solver_FLOAT ltemp;
    
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
void acrobot_QP_solver_LA_DENSE_MVMSUB1_5_5(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *b, acrobot_QP_solver_FLOAT *r)
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
void acrobot_QP_solver_LA_DENSE_BACKWARDSUB_5(acrobot_QP_solver_FLOAT *L, acrobot_QP_solver_FLOAT *y, acrobot_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    acrobot_QP_solver_FLOAT xel;    
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
void acrobot_QP_solver_LA_DENSE_MTVMSUB_5_5(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *b, acrobot_QP_solver_FLOAT *r)
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
void acrobot_QP_solver_LA_DENSE_MTVMSUB_5_9(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *b, acrobot_QP_solver_FLOAT *r)
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
void acrobot_QP_solver_LA_DENSE_BACKWARDSUB_9(acrobot_QP_solver_FLOAT *L, acrobot_QP_solver_FLOAT *y, acrobot_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    acrobot_QP_solver_FLOAT xel;    
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
 * Vector subtraction z = -x - y for vectors of length 131.
 */
void acrobot_QP_solver_LA_VSUB2_131(acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *y, acrobot_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<131; i++){
		z[i] = -x[i] - y[i];
	}
}


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * diagonal matrix of size 14 in vector
 * storage format.
 */
void acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_FLOAT *L, acrobot_QP_solver_FLOAT *b, acrobot_QP_solver_FLOAT *x)
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
void acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_5(acrobot_QP_solver_FLOAT *L, acrobot_QP_solver_FLOAT *b, acrobot_QP_solver_FLOAT *x)
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
void acrobot_QP_solver_LA_VSUB_INDEXED_14(acrobot_QP_solver_FLOAT *x, int* xidx, acrobot_QP_solver_FLOAT *y, acrobot_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<14; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 14.
 */
void acrobot_QP_solver_LA_VSUB3_14(acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *v, acrobot_QP_solver_FLOAT *w, acrobot_QP_solver_FLOAT *x)
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
void acrobot_QP_solver_LA_VSUB2_INDEXED_6(acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *y, int* yidx, acrobot_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<6; i++){
		z[i] = -x[i] - y[yidx[i]];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 6.
 */
void acrobot_QP_solver_LA_VSUB3_6(acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *v, acrobot_QP_solver_FLOAT *w, acrobot_QP_solver_FLOAT *x)
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
void acrobot_QP_solver_LA_VSUB_INDEXED_5(acrobot_QP_solver_FLOAT *x, int* xidx, acrobot_QP_solver_FLOAT *y, acrobot_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<5; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 5.
 */
void acrobot_QP_solver_LA_VSUB3_5(acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *v, acrobot_QP_solver_FLOAT *w, acrobot_QP_solver_FLOAT *x)
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
void acrobot_QP_solver_LA_VSUB2_INDEXED_5(acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *y, int* yidx, acrobot_QP_solver_FLOAT *z)
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
 * acrobot_QP_solver_NOPROGRESS (should be negative).
 */
int acrobot_QP_solver_LINESEARCH_BACKTRACKING_AFFINE(acrobot_QP_solver_FLOAT *l, acrobot_QP_solver_FLOAT *s, acrobot_QP_solver_FLOAT *dl, acrobot_QP_solver_FLOAT *ds, acrobot_QP_solver_FLOAT *a, acrobot_QP_solver_FLOAT *mu_aff)
{
    int i;
	int lsIt=1;    
    acrobot_QP_solver_FLOAT dltemp;
    acrobot_QP_solver_FLOAT dstemp;
    acrobot_QP_solver_FLOAT mya = 1.0;
    acrobot_QP_solver_FLOAT mymu;
        
    while( 1 ){                        

        /* 
         * Compute both snew and wnew together.
         * We compute also mu_affine along the way here, as the
         * values might be in registers, so it should be cheaper.
         */
        mymu = 0;
        for( i=0; i<190; i++ ){
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
        if( i == 190 ){
            break;
        } else {
            mya *= acrobot_QP_solver_SET_LS_SCALE_AFF;
            if( mya < acrobot_QP_solver_SET_LS_MINSTEP ){
                return acrobot_QP_solver_NOPROGRESS;
            }
        }
    }
    
    /* return new values and iteration counter */
    *a = mya;
    *mu_aff = mymu / (acrobot_QP_solver_FLOAT)190;
    return lsIt;
}


/*
 * Vector subtraction x = (u.*v - mu)*sigma where a is a scalar
*  and x,u,v are vectors of length 190.
 */
void acrobot_QP_solver_LA_VSUB5_190(acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *v, acrobot_QP_solver_FLOAT mu,  acrobot_QP_solver_FLOAT sigma, acrobot_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<190; i++){
		x[i] = u[i]*v[i] - mu;
		x[i] *= sigma;
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 14,
 * u, su, uidx are of length 6 and v, sv, vidx are of length 14.
 */
void acrobot_QP_solver_LA_VSUB6_INDEXED_14_6_14(acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *su, int* uidx, acrobot_QP_solver_FLOAT *v, acrobot_QP_solver_FLOAT *sv, int* vidx, acrobot_QP_solver_FLOAT *x)
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
void acrobot_QP_solver_LA_DENSE_2MVMADD_9_14_14(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *r)
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
void acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_14(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *r)
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
void acrobot_QP_solver_LA_VSUB6_INDEXED_5_5_5(acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *su, int* uidx, acrobot_QP_solver_FLOAT *v, acrobot_QP_solver_FLOAT *sv, int* vidx, acrobot_QP_solver_FLOAT *x)
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
void acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_5(acrobot_QP_solver_FLOAT *A, acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *B, acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *r)
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
 * Vector subtraction z = x - y for vectors of length 131.
 */
void acrobot_QP_solver_LA_VSUB_131(acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *y, acrobot_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<131; i++){
		z[i] = x[i] - y[i];
	}
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 14 (length of y >= 14).
 */
void acrobot_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(acrobot_QP_solver_FLOAT *r, acrobot_QP_solver_FLOAT *s, acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *y, int* yidx, acrobot_QP_solver_FLOAT *z)
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
void acrobot_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(acrobot_QP_solver_FLOAT *r, acrobot_QP_solver_FLOAT *s, acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *y, int* yidx, acrobot_QP_solver_FLOAT *z)
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
void acrobot_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_5(acrobot_QP_solver_FLOAT *r, acrobot_QP_solver_FLOAT *s, acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *y, int* yidx, acrobot_QP_solver_FLOAT *z)
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
void acrobot_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_5(acrobot_QP_solver_FLOAT *r, acrobot_QP_solver_FLOAT *s, acrobot_QP_solver_FLOAT *u, acrobot_QP_solver_FLOAT *y, int* yidx, acrobot_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<5; i++ ){
		z[i] = -r[i]/s[i] + u[i]*y[yidx[i]];
	}
}


/*
 * Computes ds = -l.\(r + s.*dl) for vectors of length 190.
 */
void acrobot_QP_solver_LA_VSUB7_190(acrobot_QP_solver_FLOAT *l, acrobot_QP_solver_FLOAT *r, acrobot_QP_solver_FLOAT *s, acrobot_QP_solver_FLOAT *dl, acrobot_QP_solver_FLOAT *ds)
{
	int i;
	for( i=0; i<190; i++){
		ds[i] = -(r[i] + s[i]*dl[i])/l[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 131.
 */
void acrobot_QP_solver_LA_VADD_131(acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<131; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 49.
 */
void acrobot_QP_solver_LA_VADD_49(acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<49; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 190.
 */
void acrobot_QP_solver_LA_VADD_190(acrobot_QP_solver_FLOAT *x, acrobot_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<190; i++){
		x[i] += y[i];
	}
}


/**
 * Backtracking line search for combined predictor/corrector step.
 * Update on variables with safety factor gamma (to keep us away from
 * boundary).
 */
int acrobot_QP_solver_LINESEARCH_BACKTRACKING_COMBINED(acrobot_QP_solver_FLOAT *z, acrobot_QP_solver_FLOAT *v, acrobot_QP_solver_FLOAT *l, acrobot_QP_solver_FLOAT *s, acrobot_QP_solver_FLOAT *dz, acrobot_QP_solver_FLOAT *dv, acrobot_QP_solver_FLOAT *dl, acrobot_QP_solver_FLOAT *ds, acrobot_QP_solver_FLOAT *a, acrobot_QP_solver_FLOAT *mu)
{
    int i, lsIt=1;       
    acrobot_QP_solver_FLOAT dltemp;
    acrobot_QP_solver_FLOAT dstemp;    
    acrobot_QP_solver_FLOAT a_gamma;
            
    *a = 1.0;
    while( 1 ){                        

        /* check whether search criterion is fulfilled */
        for( i=0; i<190; i++ ){
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
        if( i == 190 ){
            break;
        } else {
            *a *= acrobot_QP_solver_SET_LS_SCALE;
            if( *a < acrobot_QP_solver_SET_LS_MINSTEP ){
                return acrobot_QP_solver_NOPROGRESS;
            }
        }
    }
    
    /* update variables with safety margin */
    a_gamma = (*a)*acrobot_QP_solver_SET_LS_MAXSTEP;
    
    /* primal variables */
    for( i=0; i<131; i++ ){
        z[i] += a_gamma*dz[i];
    }
    
    /* equality constraint multipliers */
    for( i=0; i<49; i++ ){
        v[i] += a_gamma*dv[i];
    }
    
    /* inequality constraint multipliers & slacks, also update mu */
    *mu = 0;
    for( i=0; i<190; i++ ){
        dltemp = l[i] + a_gamma*dl[i]; l[i] = dltemp;
        dstemp = s[i] + a_gamma*ds[i]; s[i] = dstemp;
        *mu += dltemp*dstemp;
    }
    
    *a = a_gamma;
    *mu /= (acrobot_QP_solver_FLOAT)190;
    return lsIt;
}




/* VARIABLE DEFINITIONS ------------------------------------------------ */
acrobot_QP_solver_FLOAT acrobot_QP_solver_z[131];
acrobot_QP_solver_FLOAT acrobot_QP_solver_v[49];
acrobot_QP_solver_FLOAT acrobot_QP_solver_dz_aff[131];
acrobot_QP_solver_FLOAT acrobot_QP_solver_dv_aff[49];
acrobot_QP_solver_FLOAT acrobot_QP_solver_grad_cost[131];
acrobot_QP_solver_FLOAT acrobot_QP_solver_grad_eq[131];
acrobot_QP_solver_FLOAT acrobot_QP_solver_rd[131];
acrobot_QP_solver_FLOAT acrobot_QP_solver_l[190];
acrobot_QP_solver_FLOAT acrobot_QP_solver_s[190];
acrobot_QP_solver_FLOAT acrobot_QP_solver_lbys[190];
acrobot_QP_solver_FLOAT acrobot_QP_solver_dl_aff[190];
acrobot_QP_solver_FLOAT acrobot_QP_solver_ds_aff[190];
acrobot_QP_solver_FLOAT acrobot_QP_solver_dz_cc[131];
acrobot_QP_solver_FLOAT acrobot_QP_solver_dv_cc[49];
acrobot_QP_solver_FLOAT acrobot_QP_solver_dl_cc[190];
acrobot_QP_solver_FLOAT acrobot_QP_solver_ds_cc[190];
acrobot_QP_solver_FLOAT acrobot_QP_solver_ccrhs[190];
acrobot_QP_solver_FLOAT acrobot_QP_solver_grad_ineq[131];
acrobot_QP_solver_FLOAT acrobot_QP_solver_H0[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_z0 = acrobot_QP_solver_z + 0;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzaff0 = acrobot_QP_solver_dz_aff + 0;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzcc0 = acrobot_QP_solver_dz_cc + 0;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_rd0 = acrobot_QP_solver_rd + 0;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lbyrd0[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_cost0 = acrobot_QP_solver_grad_cost + 0;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_eq0 = acrobot_QP_solver_grad_eq + 0;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_ineq0 = acrobot_QP_solver_grad_ineq + 0;
acrobot_QP_solver_FLOAT acrobot_QP_solver_ctv0[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_v0 = acrobot_QP_solver_v + 0;
acrobot_QP_solver_FLOAT acrobot_QP_solver_re0[9];
acrobot_QP_solver_FLOAT acrobot_QP_solver_beta0[9];
acrobot_QP_solver_FLOAT acrobot_QP_solver_betacc0[9];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvaff0 = acrobot_QP_solver_dv_aff + 0;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvcc0 = acrobot_QP_solver_dv_cc + 0;
acrobot_QP_solver_FLOAT acrobot_QP_solver_V0[126];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Yd0[45];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Ld0[45];
acrobot_QP_solver_FLOAT acrobot_QP_solver_yy0[9];
acrobot_QP_solver_FLOAT acrobot_QP_solver_bmy0[9];
int acrobot_QP_solver_lbIdx0[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llb0 = acrobot_QP_solver_l + 0;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_slb0 = acrobot_QP_solver_s + 0;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llbbyslb0 = acrobot_QP_solver_lbys + 0;
acrobot_QP_solver_FLOAT acrobot_QP_solver_rilb0[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbaff0 = acrobot_QP_solver_dl_aff + 0;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbaff0 = acrobot_QP_solver_ds_aff + 0;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbcc0 = acrobot_QP_solver_dl_cc + 0;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbcc0 = acrobot_QP_solver_ds_cc + 0;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsl0 = acrobot_QP_solver_ccrhs + 0;
int acrobot_QP_solver_ubIdx0[6] = {0, 1, 2, 3, 4, 5};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lub0 = acrobot_QP_solver_l + 14;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_sub0 = acrobot_QP_solver_s + 14;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lubbysub0 = acrobot_QP_solver_lbys + 14;
acrobot_QP_solver_FLOAT acrobot_QP_solver_riub0[6];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubaff0 = acrobot_QP_solver_dl_aff + 14;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubaff0 = acrobot_QP_solver_ds_aff + 14;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubcc0 = acrobot_QP_solver_dl_cc + 14;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubcc0 = acrobot_QP_solver_ds_cc + 14;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsub0 = acrobot_QP_solver_ccrhs + 14;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Phi0[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_z1 = acrobot_QP_solver_z + 14;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzaff1 = acrobot_QP_solver_dz_aff + 14;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzcc1 = acrobot_QP_solver_dz_cc + 14;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_rd1 = acrobot_QP_solver_rd + 14;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lbyrd1[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_cost1 = acrobot_QP_solver_grad_cost + 14;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_eq1 = acrobot_QP_solver_grad_eq + 14;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_ineq1 = acrobot_QP_solver_grad_ineq + 14;
acrobot_QP_solver_FLOAT acrobot_QP_solver_ctv1[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_v1 = acrobot_QP_solver_v + 9;
acrobot_QP_solver_FLOAT acrobot_QP_solver_re1[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_beta1[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_betacc1[5];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvaff1 = acrobot_QP_solver_dv_aff + 9;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvcc1 = acrobot_QP_solver_dv_cc + 9;
acrobot_QP_solver_FLOAT acrobot_QP_solver_V1[70];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Yd1[15];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Ld1[15];
acrobot_QP_solver_FLOAT acrobot_QP_solver_yy1[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_bmy1[5];
int acrobot_QP_solver_lbIdx1[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llb1 = acrobot_QP_solver_l + 20;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_slb1 = acrobot_QP_solver_s + 20;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llbbyslb1 = acrobot_QP_solver_lbys + 20;
acrobot_QP_solver_FLOAT acrobot_QP_solver_rilb1[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbaff1 = acrobot_QP_solver_dl_aff + 20;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbaff1 = acrobot_QP_solver_ds_aff + 20;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbcc1 = acrobot_QP_solver_dl_cc + 20;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbcc1 = acrobot_QP_solver_ds_cc + 20;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsl1 = acrobot_QP_solver_ccrhs + 20;
int acrobot_QP_solver_ubIdx1[6] = {0, 1, 2, 3, 4, 5};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lub1 = acrobot_QP_solver_l + 34;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_sub1 = acrobot_QP_solver_s + 34;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lubbysub1 = acrobot_QP_solver_lbys + 34;
acrobot_QP_solver_FLOAT acrobot_QP_solver_riub1[6];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubaff1 = acrobot_QP_solver_dl_aff + 34;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubaff1 = acrobot_QP_solver_ds_aff + 34;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubcc1 = acrobot_QP_solver_dl_cc + 34;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubcc1 = acrobot_QP_solver_ds_cc + 34;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsub1 = acrobot_QP_solver_ccrhs + 34;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Phi1[14];
acrobot_QP_solver_FLOAT acrobot_QP_solver_D1[126] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
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
acrobot_QP_solver_FLOAT acrobot_QP_solver_W1[126];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Ysd1[45];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lsd1[45];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_z2 = acrobot_QP_solver_z + 28;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzaff2 = acrobot_QP_solver_dz_aff + 28;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzcc2 = acrobot_QP_solver_dz_cc + 28;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_rd2 = acrobot_QP_solver_rd + 28;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lbyrd2[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_cost2 = acrobot_QP_solver_grad_cost + 28;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_eq2 = acrobot_QP_solver_grad_eq + 28;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_ineq2 = acrobot_QP_solver_grad_ineq + 28;
acrobot_QP_solver_FLOAT acrobot_QP_solver_ctv2[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_v2 = acrobot_QP_solver_v + 14;
acrobot_QP_solver_FLOAT acrobot_QP_solver_re2[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_beta2[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_betacc2[5];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvaff2 = acrobot_QP_solver_dv_aff + 14;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvcc2 = acrobot_QP_solver_dv_cc + 14;
acrobot_QP_solver_FLOAT acrobot_QP_solver_V2[70];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Yd2[15];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Ld2[15];
acrobot_QP_solver_FLOAT acrobot_QP_solver_yy2[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_bmy2[5];
int acrobot_QP_solver_lbIdx2[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llb2 = acrobot_QP_solver_l + 40;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_slb2 = acrobot_QP_solver_s + 40;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llbbyslb2 = acrobot_QP_solver_lbys + 40;
acrobot_QP_solver_FLOAT acrobot_QP_solver_rilb2[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbaff2 = acrobot_QP_solver_dl_aff + 40;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbaff2 = acrobot_QP_solver_ds_aff + 40;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbcc2 = acrobot_QP_solver_dl_cc + 40;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbcc2 = acrobot_QP_solver_ds_cc + 40;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsl2 = acrobot_QP_solver_ccrhs + 40;
int acrobot_QP_solver_ubIdx2[6] = {0, 1, 2, 3, 4, 5};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lub2 = acrobot_QP_solver_l + 54;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_sub2 = acrobot_QP_solver_s + 54;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lubbysub2 = acrobot_QP_solver_lbys + 54;
acrobot_QP_solver_FLOAT acrobot_QP_solver_riub2[6];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubaff2 = acrobot_QP_solver_dl_aff + 54;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubaff2 = acrobot_QP_solver_ds_aff + 54;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubcc2 = acrobot_QP_solver_dl_cc + 54;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubcc2 = acrobot_QP_solver_ds_cc + 54;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsub2 = acrobot_QP_solver_ccrhs + 54;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Phi2[14];
acrobot_QP_solver_FLOAT acrobot_QP_solver_D2[14] = {-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000};
acrobot_QP_solver_FLOAT acrobot_QP_solver_W2[14];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Ysd2[25];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lsd2[25];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_z3 = acrobot_QP_solver_z + 42;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzaff3 = acrobot_QP_solver_dz_aff + 42;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzcc3 = acrobot_QP_solver_dz_cc + 42;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_rd3 = acrobot_QP_solver_rd + 42;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lbyrd3[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_cost3 = acrobot_QP_solver_grad_cost + 42;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_eq3 = acrobot_QP_solver_grad_eq + 42;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_ineq3 = acrobot_QP_solver_grad_ineq + 42;
acrobot_QP_solver_FLOAT acrobot_QP_solver_ctv3[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_v3 = acrobot_QP_solver_v + 19;
acrobot_QP_solver_FLOAT acrobot_QP_solver_re3[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_beta3[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_betacc3[5];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvaff3 = acrobot_QP_solver_dv_aff + 19;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvcc3 = acrobot_QP_solver_dv_cc + 19;
acrobot_QP_solver_FLOAT acrobot_QP_solver_V3[70];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Yd3[15];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Ld3[15];
acrobot_QP_solver_FLOAT acrobot_QP_solver_yy3[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_bmy3[5];
int acrobot_QP_solver_lbIdx3[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llb3 = acrobot_QP_solver_l + 60;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_slb3 = acrobot_QP_solver_s + 60;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llbbyslb3 = acrobot_QP_solver_lbys + 60;
acrobot_QP_solver_FLOAT acrobot_QP_solver_rilb3[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbaff3 = acrobot_QP_solver_dl_aff + 60;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbaff3 = acrobot_QP_solver_ds_aff + 60;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbcc3 = acrobot_QP_solver_dl_cc + 60;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbcc3 = acrobot_QP_solver_ds_cc + 60;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsl3 = acrobot_QP_solver_ccrhs + 60;
int acrobot_QP_solver_ubIdx3[6] = {0, 1, 2, 3, 4, 5};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lub3 = acrobot_QP_solver_l + 74;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_sub3 = acrobot_QP_solver_s + 74;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lubbysub3 = acrobot_QP_solver_lbys + 74;
acrobot_QP_solver_FLOAT acrobot_QP_solver_riub3[6];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubaff3 = acrobot_QP_solver_dl_aff + 74;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubaff3 = acrobot_QP_solver_ds_aff + 74;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubcc3 = acrobot_QP_solver_dl_cc + 74;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubcc3 = acrobot_QP_solver_ds_cc + 74;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsub3 = acrobot_QP_solver_ccrhs + 74;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Phi3[14];
acrobot_QP_solver_FLOAT acrobot_QP_solver_W3[14];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Ysd3[25];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lsd3[25];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_z4 = acrobot_QP_solver_z + 56;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzaff4 = acrobot_QP_solver_dz_aff + 56;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzcc4 = acrobot_QP_solver_dz_cc + 56;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_rd4 = acrobot_QP_solver_rd + 56;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lbyrd4[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_cost4 = acrobot_QP_solver_grad_cost + 56;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_eq4 = acrobot_QP_solver_grad_eq + 56;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_ineq4 = acrobot_QP_solver_grad_ineq + 56;
acrobot_QP_solver_FLOAT acrobot_QP_solver_ctv4[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_v4 = acrobot_QP_solver_v + 24;
acrobot_QP_solver_FLOAT acrobot_QP_solver_re4[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_beta4[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_betacc4[5];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvaff4 = acrobot_QP_solver_dv_aff + 24;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvcc4 = acrobot_QP_solver_dv_cc + 24;
acrobot_QP_solver_FLOAT acrobot_QP_solver_V4[70];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Yd4[15];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Ld4[15];
acrobot_QP_solver_FLOAT acrobot_QP_solver_yy4[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_bmy4[5];
int acrobot_QP_solver_lbIdx4[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llb4 = acrobot_QP_solver_l + 80;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_slb4 = acrobot_QP_solver_s + 80;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llbbyslb4 = acrobot_QP_solver_lbys + 80;
acrobot_QP_solver_FLOAT acrobot_QP_solver_rilb4[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbaff4 = acrobot_QP_solver_dl_aff + 80;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbaff4 = acrobot_QP_solver_ds_aff + 80;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbcc4 = acrobot_QP_solver_dl_cc + 80;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbcc4 = acrobot_QP_solver_ds_cc + 80;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsl4 = acrobot_QP_solver_ccrhs + 80;
int acrobot_QP_solver_ubIdx4[6] = {0, 1, 2, 3, 4, 5};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lub4 = acrobot_QP_solver_l + 94;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_sub4 = acrobot_QP_solver_s + 94;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lubbysub4 = acrobot_QP_solver_lbys + 94;
acrobot_QP_solver_FLOAT acrobot_QP_solver_riub4[6];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubaff4 = acrobot_QP_solver_dl_aff + 94;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubaff4 = acrobot_QP_solver_ds_aff + 94;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubcc4 = acrobot_QP_solver_dl_cc + 94;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubcc4 = acrobot_QP_solver_ds_cc + 94;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsub4 = acrobot_QP_solver_ccrhs + 94;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Phi4[14];
acrobot_QP_solver_FLOAT acrobot_QP_solver_W4[14];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Ysd4[25];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lsd4[25];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_z5 = acrobot_QP_solver_z + 70;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzaff5 = acrobot_QP_solver_dz_aff + 70;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzcc5 = acrobot_QP_solver_dz_cc + 70;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_rd5 = acrobot_QP_solver_rd + 70;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lbyrd5[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_cost5 = acrobot_QP_solver_grad_cost + 70;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_eq5 = acrobot_QP_solver_grad_eq + 70;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_ineq5 = acrobot_QP_solver_grad_ineq + 70;
acrobot_QP_solver_FLOAT acrobot_QP_solver_ctv5[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_v5 = acrobot_QP_solver_v + 29;
acrobot_QP_solver_FLOAT acrobot_QP_solver_re5[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_beta5[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_betacc5[5];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvaff5 = acrobot_QP_solver_dv_aff + 29;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvcc5 = acrobot_QP_solver_dv_cc + 29;
acrobot_QP_solver_FLOAT acrobot_QP_solver_V5[70];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Yd5[15];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Ld5[15];
acrobot_QP_solver_FLOAT acrobot_QP_solver_yy5[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_bmy5[5];
int acrobot_QP_solver_lbIdx5[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llb5 = acrobot_QP_solver_l + 100;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_slb5 = acrobot_QP_solver_s + 100;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llbbyslb5 = acrobot_QP_solver_lbys + 100;
acrobot_QP_solver_FLOAT acrobot_QP_solver_rilb5[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbaff5 = acrobot_QP_solver_dl_aff + 100;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbaff5 = acrobot_QP_solver_ds_aff + 100;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbcc5 = acrobot_QP_solver_dl_cc + 100;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbcc5 = acrobot_QP_solver_ds_cc + 100;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsl5 = acrobot_QP_solver_ccrhs + 100;
int acrobot_QP_solver_ubIdx5[6] = {0, 1, 2, 3, 4, 5};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lub5 = acrobot_QP_solver_l + 114;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_sub5 = acrobot_QP_solver_s + 114;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lubbysub5 = acrobot_QP_solver_lbys + 114;
acrobot_QP_solver_FLOAT acrobot_QP_solver_riub5[6];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubaff5 = acrobot_QP_solver_dl_aff + 114;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubaff5 = acrobot_QP_solver_ds_aff + 114;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubcc5 = acrobot_QP_solver_dl_cc + 114;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubcc5 = acrobot_QP_solver_ds_cc + 114;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsub5 = acrobot_QP_solver_ccrhs + 114;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Phi5[14];
acrobot_QP_solver_FLOAT acrobot_QP_solver_W5[14];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Ysd5[25];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lsd5[25];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_z6 = acrobot_QP_solver_z + 84;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzaff6 = acrobot_QP_solver_dz_aff + 84;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzcc6 = acrobot_QP_solver_dz_cc + 84;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_rd6 = acrobot_QP_solver_rd + 84;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lbyrd6[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_cost6 = acrobot_QP_solver_grad_cost + 84;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_eq6 = acrobot_QP_solver_grad_eq + 84;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_ineq6 = acrobot_QP_solver_grad_ineq + 84;
acrobot_QP_solver_FLOAT acrobot_QP_solver_ctv6[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_v6 = acrobot_QP_solver_v + 34;
acrobot_QP_solver_FLOAT acrobot_QP_solver_re6[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_beta6[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_betacc6[5];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvaff6 = acrobot_QP_solver_dv_aff + 34;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvcc6 = acrobot_QP_solver_dv_cc + 34;
acrobot_QP_solver_FLOAT acrobot_QP_solver_V6[70];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Yd6[15];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Ld6[15];
acrobot_QP_solver_FLOAT acrobot_QP_solver_yy6[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_bmy6[5];
int acrobot_QP_solver_lbIdx6[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llb6 = acrobot_QP_solver_l + 120;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_slb6 = acrobot_QP_solver_s + 120;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llbbyslb6 = acrobot_QP_solver_lbys + 120;
acrobot_QP_solver_FLOAT acrobot_QP_solver_rilb6[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbaff6 = acrobot_QP_solver_dl_aff + 120;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbaff6 = acrobot_QP_solver_ds_aff + 120;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbcc6 = acrobot_QP_solver_dl_cc + 120;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbcc6 = acrobot_QP_solver_ds_cc + 120;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsl6 = acrobot_QP_solver_ccrhs + 120;
int acrobot_QP_solver_ubIdx6[6] = {0, 1, 2, 3, 4, 5};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lub6 = acrobot_QP_solver_l + 134;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_sub6 = acrobot_QP_solver_s + 134;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lubbysub6 = acrobot_QP_solver_lbys + 134;
acrobot_QP_solver_FLOAT acrobot_QP_solver_riub6[6];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubaff6 = acrobot_QP_solver_dl_aff + 134;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubaff6 = acrobot_QP_solver_ds_aff + 134;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubcc6 = acrobot_QP_solver_dl_cc + 134;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubcc6 = acrobot_QP_solver_ds_cc + 134;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsub6 = acrobot_QP_solver_ccrhs + 134;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Phi6[14];
acrobot_QP_solver_FLOAT acrobot_QP_solver_W6[14];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Ysd6[25];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lsd6[25];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_z7 = acrobot_QP_solver_z + 98;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzaff7 = acrobot_QP_solver_dz_aff + 98;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzcc7 = acrobot_QP_solver_dz_cc + 98;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_rd7 = acrobot_QP_solver_rd + 98;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lbyrd7[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_cost7 = acrobot_QP_solver_grad_cost + 98;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_eq7 = acrobot_QP_solver_grad_eq + 98;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_ineq7 = acrobot_QP_solver_grad_ineq + 98;
acrobot_QP_solver_FLOAT acrobot_QP_solver_ctv7[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_v7 = acrobot_QP_solver_v + 39;
acrobot_QP_solver_FLOAT acrobot_QP_solver_re7[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_beta7[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_betacc7[5];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvaff7 = acrobot_QP_solver_dv_aff + 39;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvcc7 = acrobot_QP_solver_dv_cc + 39;
acrobot_QP_solver_FLOAT acrobot_QP_solver_V7[70];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Yd7[15];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Ld7[15];
acrobot_QP_solver_FLOAT acrobot_QP_solver_yy7[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_bmy7[5];
int acrobot_QP_solver_lbIdx7[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llb7 = acrobot_QP_solver_l + 140;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_slb7 = acrobot_QP_solver_s + 140;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llbbyslb7 = acrobot_QP_solver_lbys + 140;
acrobot_QP_solver_FLOAT acrobot_QP_solver_rilb7[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbaff7 = acrobot_QP_solver_dl_aff + 140;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbaff7 = acrobot_QP_solver_ds_aff + 140;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbcc7 = acrobot_QP_solver_dl_cc + 140;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbcc7 = acrobot_QP_solver_ds_cc + 140;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsl7 = acrobot_QP_solver_ccrhs + 140;
int acrobot_QP_solver_ubIdx7[6] = {0, 1, 2, 3, 4, 5};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lub7 = acrobot_QP_solver_l + 154;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_sub7 = acrobot_QP_solver_s + 154;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lubbysub7 = acrobot_QP_solver_lbys + 154;
acrobot_QP_solver_FLOAT acrobot_QP_solver_riub7[6];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubaff7 = acrobot_QP_solver_dl_aff + 154;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubaff7 = acrobot_QP_solver_ds_aff + 154;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubcc7 = acrobot_QP_solver_dl_cc + 154;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubcc7 = acrobot_QP_solver_ds_cc + 154;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsub7 = acrobot_QP_solver_ccrhs + 154;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Phi7[14];
acrobot_QP_solver_FLOAT acrobot_QP_solver_W7[14];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Ysd7[25];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lsd7[25];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_z8 = acrobot_QP_solver_z + 112;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzaff8 = acrobot_QP_solver_dz_aff + 112;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzcc8 = acrobot_QP_solver_dz_cc + 112;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_rd8 = acrobot_QP_solver_rd + 112;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lbyrd8[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_cost8 = acrobot_QP_solver_grad_cost + 112;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_eq8 = acrobot_QP_solver_grad_eq + 112;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_ineq8 = acrobot_QP_solver_grad_ineq + 112;
acrobot_QP_solver_FLOAT acrobot_QP_solver_ctv8[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_v8 = acrobot_QP_solver_v + 44;
acrobot_QP_solver_FLOAT acrobot_QP_solver_re8[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_beta8[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_betacc8[5];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvaff8 = acrobot_QP_solver_dv_aff + 44;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dvcc8 = acrobot_QP_solver_dv_cc + 44;
acrobot_QP_solver_FLOAT acrobot_QP_solver_V8[70];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Yd8[15];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Ld8[15];
acrobot_QP_solver_FLOAT acrobot_QP_solver_yy8[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_bmy8[5];
int acrobot_QP_solver_lbIdx8[14] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llb8 = acrobot_QP_solver_l + 160;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_slb8 = acrobot_QP_solver_s + 160;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llbbyslb8 = acrobot_QP_solver_lbys + 160;
acrobot_QP_solver_FLOAT acrobot_QP_solver_rilb8[14];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbaff8 = acrobot_QP_solver_dl_aff + 160;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbaff8 = acrobot_QP_solver_ds_aff + 160;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbcc8 = acrobot_QP_solver_dl_cc + 160;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbcc8 = acrobot_QP_solver_ds_cc + 160;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsl8 = acrobot_QP_solver_ccrhs + 160;
int acrobot_QP_solver_ubIdx8[6] = {0, 1, 2, 3, 4, 5};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lub8 = acrobot_QP_solver_l + 174;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_sub8 = acrobot_QP_solver_s + 174;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lubbysub8 = acrobot_QP_solver_lbys + 174;
acrobot_QP_solver_FLOAT acrobot_QP_solver_riub8[6];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubaff8 = acrobot_QP_solver_dl_aff + 174;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubaff8 = acrobot_QP_solver_ds_aff + 174;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubcc8 = acrobot_QP_solver_dl_cc + 174;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubcc8 = acrobot_QP_solver_ds_cc + 174;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsub8 = acrobot_QP_solver_ccrhs + 174;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Phi8[14];
acrobot_QP_solver_FLOAT acrobot_QP_solver_W8[14];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Ysd8[25];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lsd8[25];
acrobot_QP_solver_FLOAT acrobot_QP_solver_H9[5] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
acrobot_QP_solver_FLOAT acrobot_QP_solver_f9[5] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_z9 = acrobot_QP_solver_z + 126;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzaff9 = acrobot_QP_solver_dz_aff + 126;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dzcc9 = acrobot_QP_solver_dz_cc + 126;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_rd9 = acrobot_QP_solver_rd + 126;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Lbyrd9[5];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_cost9 = acrobot_QP_solver_grad_cost + 126;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_eq9 = acrobot_QP_solver_grad_eq + 126;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_grad_ineq9 = acrobot_QP_solver_grad_ineq + 126;
acrobot_QP_solver_FLOAT acrobot_QP_solver_ctv9[5];
int acrobot_QP_solver_lbIdx9[5] = {0, 1, 2, 3, 4};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llb9 = acrobot_QP_solver_l + 180;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_slb9 = acrobot_QP_solver_s + 180;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_llbbyslb9 = acrobot_QP_solver_lbys + 180;
acrobot_QP_solver_FLOAT acrobot_QP_solver_rilb9[5];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbaff9 = acrobot_QP_solver_dl_aff + 180;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbaff9 = acrobot_QP_solver_ds_aff + 180;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dllbcc9 = acrobot_QP_solver_dl_cc + 180;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dslbcc9 = acrobot_QP_solver_ds_cc + 180;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsl9 = acrobot_QP_solver_ccrhs + 180;
int acrobot_QP_solver_ubIdx9[5] = {0, 1, 2, 3, 4};
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lub9 = acrobot_QP_solver_l + 185;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_sub9 = acrobot_QP_solver_s + 185;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_lubbysub9 = acrobot_QP_solver_lbys + 185;
acrobot_QP_solver_FLOAT acrobot_QP_solver_riub9[5];
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubaff9 = acrobot_QP_solver_dl_aff + 185;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubaff9 = acrobot_QP_solver_ds_aff + 185;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dlubcc9 = acrobot_QP_solver_dl_cc + 185;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_dsubcc9 = acrobot_QP_solver_ds_cc + 185;
acrobot_QP_solver_FLOAT* acrobot_QP_solver_ccrhsub9 = acrobot_QP_solver_ccrhs + 185;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Phi9[5];
acrobot_QP_solver_FLOAT acrobot_QP_solver_D9[5] = {-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000};
acrobot_QP_solver_FLOAT acrobot_QP_solver_W9[5];
acrobot_QP_solver_FLOAT musigma;
acrobot_QP_solver_FLOAT sigma_3rdroot;
acrobot_QP_solver_FLOAT acrobot_QP_solver_Diag1_0[14];
acrobot_QP_solver_FLOAT acrobot_QP_solver_Diag2_0[14];
acrobot_QP_solver_FLOAT acrobot_QP_solver_L_0[91];




/* SOLVER CODE --------------------------------------------------------- */
int acrobot_QP_solver_solve(acrobot_QP_solver_params* params, acrobot_QP_solver_output* output, acrobot_QP_solver_info* info)
{	
int exitcode;

#if acrobot_QP_solver_SET_TIMING == 1
	acrobot_QP_solver_timer solvertimer;
	acrobot_QP_solver_tic(&solvertimer);
#endif
/* FUNCTION CALLS INTO LA LIBRARY -------------------------------------- */
info->it = 0;
acrobot_QP_solver_LA_INITIALIZEVECTOR_131(acrobot_QP_solver_z, 0);
acrobot_QP_solver_LA_INITIALIZEVECTOR_49(acrobot_QP_solver_v, 1);
acrobot_QP_solver_LA_INITIALIZEVECTOR_190(acrobot_QP_solver_l, 10);
acrobot_QP_solver_LA_INITIALIZEVECTOR_190(acrobot_QP_solver_s, 10);
info->mu = 0;
acrobot_QP_solver_LA_DOTACC_190(acrobot_QP_solver_l, acrobot_QP_solver_s, &info->mu);
info->mu /= 190;
while( 1 ){
info->pobj = 0;
acrobot_QP_solver_LA_DIAG_QUADFCN_14(acrobot_QP_solver_H0, params->f1, acrobot_QP_solver_z0, acrobot_QP_solver_grad_cost0, &info->pobj);
acrobot_QP_solver_LA_DIAG_QUADFCN_14(acrobot_QP_solver_H0, params->f2, acrobot_QP_solver_z1, acrobot_QP_solver_grad_cost1, &info->pobj);
acrobot_QP_solver_LA_DIAG_QUADFCN_14(acrobot_QP_solver_H0, params->f3, acrobot_QP_solver_z2, acrobot_QP_solver_grad_cost2, &info->pobj);
acrobot_QP_solver_LA_DIAG_QUADFCN_14(acrobot_QP_solver_H0, params->f4, acrobot_QP_solver_z3, acrobot_QP_solver_grad_cost3, &info->pobj);
acrobot_QP_solver_LA_DIAG_QUADFCN_14(acrobot_QP_solver_H0, params->f5, acrobot_QP_solver_z4, acrobot_QP_solver_grad_cost4, &info->pobj);
acrobot_QP_solver_LA_DIAG_QUADFCN_14(acrobot_QP_solver_H0, params->f6, acrobot_QP_solver_z5, acrobot_QP_solver_grad_cost5, &info->pobj);
acrobot_QP_solver_LA_DIAG_QUADFCN_14(acrobot_QP_solver_H0, params->f7, acrobot_QP_solver_z6, acrobot_QP_solver_grad_cost6, &info->pobj);
acrobot_QP_solver_LA_DIAG_QUADFCN_14(acrobot_QP_solver_H0, params->f8, acrobot_QP_solver_z7, acrobot_QP_solver_grad_cost7, &info->pobj);
acrobot_QP_solver_LA_DIAG_QUADFCN_14(acrobot_QP_solver_H0, params->f9, acrobot_QP_solver_z8, acrobot_QP_solver_grad_cost8, &info->pobj);
acrobot_QP_solver_LA_DIAG_QUADFCN_5(acrobot_QP_solver_H9, acrobot_QP_solver_f9, acrobot_QP_solver_z9, acrobot_QP_solver_grad_cost9, &info->pobj);
info->res_eq = 0;
info->dgap = 0;
acrobot_QP_solver_LA_DENSE_MVMSUB3_9_14_14(params->C1, acrobot_QP_solver_z0, acrobot_QP_solver_D1, acrobot_QP_solver_z1, params->e1, acrobot_QP_solver_v0, acrobot_QP_solver_re0, &info->dgap, &info->res_eq);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_14(params->C2, acrobot_QP_solver_z1, acrobot_QP_solver_D2, acrobot_QP_solver_z2, params->e2, acrobot_QP_solver_v1, acrobot_QP_solver_re1, &info->dgap, &info->res_eq);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_14(params->C3, acrobot_QP_solver_z2, acrobot_QP_solver_D2, acrobot_QP_solver_z3, params->e3, acrobot_QP_solver_v2, acrobot_QP_solver_re2, &info->dgap, &info->res_eq);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_14(params->C4, acrobot_QP_solver_z3, acrobot_QP_solver_D2, acrobot_QP_solver_z4, params->e4, acrobot_QP_solver_v3, acrobot_QP_solver_re3, &info->dgap, &info->res_eq);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_14(params->C5, acrobot_QP_solver_z4, acrobot_QP_solver_D2, acrobot_QP_solver_z5, params->e5, acrobot_QP_solver_v4, acrobot_QP_solver_re4, &info->dgap, &info->res_eq);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_14(params->C6, acrobot_QP_solver_z5, acrobot_QP_solver_D2, acrobot_QP_solver_z6, params->e6, acrobot_QP_solver_v5, acrobot_QP_solver_re5, &info->dgap, &info->res_eq);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_14(params->C7, acrobot_QP_solver_z6, acrobot_QP_solver_D2, acrobot_QP_solver_z7, params->e7, acrobot_QP_solver_v6, acrobot_QP_solver_re6, &info->dgap, &info->res_eq);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_14(params->C8, acrobot_QP_solver_z7, acrobot_QP_solver_D2, acrobot_QP_solver_z8, params->e8, acrobot_QP_solver_v7, acrobot_QP_solver_re7, &info->dgap, &info->res_eq);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_14_5(params->C9, acrobot_QP_solver_z8, acrobot_QP_solver_D9, acrobot_QP_solver_z9, params->e9, acrobot_QP_solver_v8, acrobot_QP_solver_re8, &info->dgap, &info->res_eq);
acrobot_QP_solver_LA_DENSE_MTVM_9_14(params->C1, acrobot_QP_solver_v0, acrobot_QP_solver_grad_eq0);
acrobot_QP_solver_LA_DENSE_MTVM2_5_14_9(params->C2, acrobot_QP_solver_v1, acrobot_QP_solver_D1, acrobot_QP_solver_v0, acrobot_QP_solver_grad_eq1);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C3, acrobot_QP_solver_v2, acrobot_QP_solver_D2, acrobot_QP_solver_v1, acrobot_QP_solver_grad_eq2);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C4, acrobot_QP_solver_v3, acrobot_QP_solver_D2, acrobot_QP_solver_v2, acrobot_QP_solver_grad_eq3);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C5, acrobot_QP_solver_v4, acrobot_QP_solver_D2, acrobot_QP_solver_v3, acrobot_QP_solver_grad_eq4);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C6, acrobot_QP_solver_v5, acrobot_QP_solver_D2, acrobot_QP_solver_v4, acrobot_QP_solver_grad_eq5);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C7, acrobot_QP_solver_v6, acrobot_QP_solver_D2, acrobot_QP_solver_v5, acrobot_QP_solver_grad_eq6);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C8, acrobot_QP_solver_v7, acrobot_QP_solver_D2, acrobot_QP_solver_v6, acrobot_QP_solver_grad_eq7);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C9, acrobot_QP_solver_v8, acrobot_QP_solver_D2, acrobot_QP_solver_v7, acrobot_QP_solver_grad_eq8);
acrobot_QP_solver_LA_DIAGZERO_MTVM_5_5(acrobot_QP_solver_D9, acrobot_QP_solver_v8, acrobot_QP_solver_grad_eq9);
info->res_ineq = 0;
acrobot_QP_solver_LA_VSUBADD3_14(params->lb1, acrobot_QP_solver_z0, acrobot_QP_solver_lbIdx0, acrobot_QP_solver_llb0, acrobot_QP_solver_slb0, acrobot_QP_solver_rilb0, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD2_6(acrobot_QP_solver_z0, acrobot_QP_solver_ubIdx0, params->ub1, acrobot_QP_solver_lub0, acrobot_QP_solver_sub0, acrobot_QP_solver_riub0, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD3_14(params->lb2, acrobot_QP_solver_z1, acrobot_QP_solver_lbIdx1, acrobot_QP_solver_llb1, acrobot_QP_solver_slb1, acrobot_QP_solver_rilb1, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD2_6(acrobot_QP_solver_z1, acrobot_QP_solver_ubIdx1, params->ub2, acrobot_QP_solver_lub1, acrobot_QP_solver_sub1, acrobot_QP_solver_riub1, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD3_14(params->lb3, acrobot_QP_solver_z2, acrobot_QP_solver_lbIdx2, acrobot_QP_solver_llb2, acrobot_QP_solver_slb2, acrobot_QP_solver_rilb2, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD2_6(acrobot_QP_solver_z2, acrobot_QP_solver_ubIdx2, params->ub3, acrobot_QP_solver_lub2, acrobot_QP_solver_sub2, acrobot_QP_solver_riub2, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD3_14(params->lb4, acrobot_QP_solver_z3, acrobot_QP_solver_lbIdx3, acrobot_QP_solver_llb3, acrobot_QP_solver_slb3, acrobot_QP_solver_rilb3, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD2_6(acrobot_QP_solver_z3, acrobot_QP_solver_ubIdx3, params->ub4, acrobot_QP_solver_lub3, acrobot_QP_solver_sub3, acrobot_QP_solver_riub3, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD3_14(params->lb5, acrobot_QP_solver_z4, acrobot_QP_solver_lbIdx4, acrobot_QP_solver_llb4, acrobot_QP_solver_slb4, acrobot_QP_solver_rilb4, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD2_6(acrobot_QP_solver_z4, acrobot_QP_solver_ubIdx4, params->ub5, acrobot_QP_solver_lub4, acrobot_QP_solver_sub4, acrobot_QP_solver_riub4, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD3_14(params->lb6, acrobot_QP_solver_z5, acrobot_QP_solver_lbIdx5, acrobot_QP_solver_llb5, acrobot_QP_solver_slb5, acrobot_QP_solver_rilb5, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD2_6(acrobot_QP_solver_z5, acrobot_QP_solver_ubIdx5, params->ub6, acrobot_QP_solver_lub5, acrobot_QP_solver_sub5, acrobot_QP_solver_riub5, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD3_14(params->lb7, acrobot_QP_solver_z6, acrobot_QP_solver_lbIdx6, acrobot_QP_solver_llb6, acrobot_QP_solver_slb6, acrobot_QP_solver_rilb6, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD2_6(acrobot_QP_solver_z6, acrobot_QP_solver_ubIdx6, params->ub7, acrobot_QP_solver_lub6, acrobot_QP_solver_sub6, acrobot_QP_solver_riub6, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD3_14(params->lb8, acrobot_QP_solver_z7, acrobot_QP_solver_lbIdx7, acrobot_QP_solver_llb7, acrobot_QP_solver_slb7, acrobot_QP_solver_rilb7, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD2_6(acrobot_QP_solver_z7, acrobot_QP_solver_ubIdx7, params->ub8, acrobot_QP_solver_lub7, acrobot_QP_solver_sub7, acrobot_QP_solver_riub7, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD3_14(params->lb9, acrobot_QP_solver_z8, acrobot_QP_solver_lbIdx8, acrobot_QP_solver_llb8, acrobot_QP_solver_slb8, acrobot_QP_solver_rilb8, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD2_6(acrobot_QP_solver_z8, acrobot_QP_solver_ubIdx8, params->ub9, acrobot_QP_solver_lub8, acrobot_QP_solver_sub8, acrobot_QP_solver_riub8, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD3_5(params->lb10, acrobot_QP_solver_z9, acrobot_QP_solver_lbIdx9, acrobot_QP_solver_llb9, acrobot_QP_solver_slb9, acrobot_QP_solver_rilb9, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_VSUBADD2_5(acrobot_QP_solver_z9, acrobot_QP_solver_ubIdx9, params->ub10, acrobot_QP_solver_lub9, acrobot_QP_solver_sub9, acrobot_QP_solver_riub9, &info->dgap, &info->res_ineq);
acrobot_QP_solver_LA_INEQ_B_GRAD_14_14_6(acrobot_QP_solver_lub0, acrobot_QP_solver_sub0, acrobot_QP_solver_riub0, acrobot_QP_solver_llb0, acrobot_QP_solver_slb0, acrobot_QP_solver_rilb0, acrobot_QP_solver_lbIdx0, acrobot_QP_solver_ubIdx0, acrobot_QP_solver_grad_ineq0, acrobot_QP_solver_lubbysub0, acrobot_QP_solver_llbbyslb0);
acrobot_QP_solver_LA_INEQ_B_GRAD_14_14_6(acrobot_QP_solver_lub1, acrobot_QP_solver_sub1, acrobot_QP_solver_riub1, acrobot_QP_solver_llb1, acrobot_QP_solver_slb1, acrobot_QP_solver_rilb1, acrobot_QP_solver_lbIdx1, acrobot_QP_solver_ubIdx1, acrobot_QP_solver_grad_ineq1, acrobot_QP_solver_lubbysub1, acrobot_QP_solver_llbbyslb1);
acrobot_QP_solver_LA_INEQ_B_GRAD_14_14_6(acrobot_QP_solver_lub2, acrobot_QP_solver_sub2, acrobot_QP_solver_riub2, acrobot_QP_solver_llb2, acrobot_QP_solver_slb2, acrobot_QP_solver_rilb2, acrobot_QP_solver_lbIdx2, acrobot_QP_solver_ubIdx2, acrobot_QP_solver_grad_ineq2, acrobot_QP_solver_lubbysub2, acrobot_QP_solver_llbbyslb2);
acrobot_QP_solver_LA_INEQ_B_GRAD_14_14_6(acrobot_QP_solver_lub3, acrobot_QP_solver_sub3, acrobot_QP_solver_riub3, acrobot_QP_solver_llb3, acrobot_QP_solver_slb3, acrobot_QP_solver_rilb3, acrobot_QP_solver_lbIdx3, acrobot_QP_solver_ubIdx3, acrobot_QP_solver_grad_ineq3, acrobot_QP_solver_lubbysub3, acrobot_QP_solver_llbbyslb3);
acrobot_QP_solver_LA_INEQ_B_GRAD_14_14_6(acrobot_QP_solver_lub4, acrobot_QP_solver_sub4, acrobot_QP_solver_riub4, acrobot_QP_solver_llb4, acrobot_QP_solver_slb4, acrobot_QP_solver_rilb4, acrobot_QP_solver_lbIdx4, acrobot_QP_solver_ubIdx4, acrobot_QP_solver_grad_ineq4, acrobot_QP_solver_lubbysub4, acrobot_QP_solver_llbbyslb4);
acrobot_QP_solver_LA_INEQ_B_GRAD_14_14_6(acrobot_QP_solver_lub5, acrobot_QP_solver_sub5, acrobot_QP_solver_riub5, acrobot_QP_solver_llb5, acrobot_QP_solver_slb5, acrobot_QP_solver_rilb5, acrobot_QP_solver_lbIdx5, acrobot_QP_solver_ubIdx5, acrobot_QP_solver_grad_ineq5, acrobot_QP_solver_lubbysub5, acrobot_QP_solver_llbbyslb5);
acrobot_QP_solver_LA_INEQ_B_GRAD_14_14_6(acrobot_QP_solver_lub6, acrobot_QP_solver_sub6, acrobot_QP_solver_riub6, acrobot_QP_solver_llb6, acrobot_QP_solver_slb6, acrobot_QP_solver_rilb6, acrobot_QP_solver_lbIdx6, acrobot_QP_solver_ubIdx6, acrobot_QP_solver_grad_ineq6, acrobot_QP_solver_lubbysub6, acrobot_QP_solver_llbbyslb6);
acrobot_QP_solver_LA_INEQ_B_GRAD_14_14_6(acrobot_QP_solver_lub7, acrobot_QP_solver_sub7, acrobot_QP_solver_riub7, acrobot_QP_solver_llb7, acrobot_QP_solver_slb7, acrobot_QP_solver_rilb7, acrobot_QP_solver_lbIdx7, acrobot_QP_solver_ubIdx7, acrobot_QP_solver_grad_ineq7, acrobot_QP_solver_lubbysub7, acrobot_QP_solver_llbbyslb7);
acrobot_QP_solver_LA_INEQ_B_GRAD_14_14_6(acrobot_QP_solver_lub8, acrobot_QP_solver_sub8, acrobot_QP_solver_riub8, acrobot_QP_solver_llb8, acrobot_QP_solver_slb8, acrobot_QP_solver_rilb8, acrobot_QP_solver_lbIdx8, acrobot_QP_solver_ubIdx8, acrobot_QP_solver_grad_ineq8, acrobot_QP_solver_lubbysub8, acrobot_QP_solver_llbbyslb8);
acrobot_QP_solver_LA_INEQ_B_GRAD_5_5_5(acrobot_QP_solver_lub9, acrobot_QP_solver_sub9, acrobot_QP_solver_riub9, acrobot_QP_solver_llb9, acrobot_QP_solver_slb9, acrobot_QP_solver_rilb9, acrobot_QP_solver_lbIdx9, acrobot_QP_solver_ubIdx9, acrobot_QP_solver_grad_ineq9, acrobot_QP_solver_lubbysub9, acrobot_QP_solver_llbbyslb9);
info->dobj = info->pobj - info->dgap;
info->rdgap = info->pobj ? info->dgap / info->pobj : 1e6;
if( info->rdgap < 0 ) info->rdgap = -info->rdgap;
if( info->mu < acrobot_QP_solver_SET_ACC_KKTCOMPL
    && (info->rdgap < acrobot_QP_solver_SET_ACC_RDGAP || info->dgap < acrobot_QP_solver_SET_ACC_KKTCOMPL)
    && info->res_eq < acrobot_QP_solver_SET_ACC_RESEQ
    && info->res_ineq < acrobot_QP_solver_SET_ACC_RESINEQ ){
exitcode = acrobot_QP_solver_OPTIMAL; break; }
if( info->it == acrobot_QP_solver_SET_MAXIT ){
exitcode = acrobot_QP_solver_MAXITREACHED; break; }
acrobot_QP_solver_LA_VVADD3_131(acrobot_QP_solver_grad_cost, acrobot_QP_solver_grad_eq, acrobot_QP_solver_grad_ineq, acrobot_QP_solver_rd);
acrobot_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(acrobot_QP_solver_H0, acrobot_QP_solver_llbbyslb0, acrobot_QP_solver_lbIdx0, acrobot_QP_solver_lubbysub0, acrobot_QP_solver_ubIdx0, acrobot_QP_solver_Phi0);
acrobot_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_14(acrobot_QP_solver_Phi0, params->C1, acrobot_QP_solver_V0);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi0, acrobot_QP_solver_rd0, acrobot_QP_solver_Lbyrd0);
acrobot_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(acrobot_QP_solver_H0, acrobot_QP_solver_llbbyslb1, acrobot_QP_solver_lbIdx1, acrobot_QP_solver_lubbysub1, acrobot_QP_solver_ubIdx1, acrobot_QP_solver_Phi1);
acrobot_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_14(acrobot_QP_solver_Phi1, params->C2, acrobot_QP_solver_V1);
acrobot_QP_solver_LA_DIAG_MATRIXFORWARDSUB_9_14(acrobot_QP_solver_Phi1, acrobot_QP_solver_D1, acrobot_QP_solver_W1);
acrobot_QP_solver_LA_DENSE_MMTM_9_14_5(acrobot_QP_solver_W1, acrobot_QP_solver_V1, acrobot_QP_solver_Ysd1);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi1, acrobot_QP_solver_rd1, acrobot_QP_solver_Lbyrd1);
acrobot_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(acrobot_QP_solver_H0, acrobot_QP_solver_llbbyslb2, acrobot_QP_solver_lbIdx2, acrobot_QP_solver_lubbysub2, acrobot_QP_solver_ubIdx2, acrobot_QP_solver_Phi2);
acrobot_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_14(acrobot_QP_solver_Phi2, params->C3, acrobot_QP_solver_V2);
acrobot_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_14(acrobot_QP_solver_Phi2, acrobot_QP_solver_D2, acrobot_QP_solver_W2);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_14_5(acrobot_QP_solver_W2, acrobot_QP_solver_V2, acrobot_QP_solver_Ysd2);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi2, acrobot_QP_solver_rd2, acrobot_QP_solver_Lbyrd2);
acrobot_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(acrobot_QP_solver_H0, acrobot_QP_solver_llbbyslb3, acrobot_QP_solver_lbIdx3, acrobot_QP_solver_lubbysub3, acrobot_QP_solver_ubIdx3, acrobot_QP_solver_Phi3);
acrobot_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_14(acrobot_QP_solver_Phi3, params->C4, acrobot_QP_solver_V3);
acrobot_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_14(acrobot_QP_solver_Phi3, acrobot_QP_solver_D2, acrobot_QP_solver_W3);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_14_5(acrobot_QP_solver_W3, acrobot_QP_solver_V3, acrobot_QP_solver_Ysd3);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi3, acrobot_QP_solver_rd3, acrobot_QP_solver_Lbyrd3);
acrobot_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(acrobot_QP_solver_H0, acrobot_QP_solver_llbbyslb4, acrobot_QP_solver_lbIdx4, acrobot_QP_solver_lubbysub4, acrobot_QP_solver_ubIdx4, acrobot_QP_solver_Phi4);
acrobot_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_14(acrobot_QP_solver_Phi4, params->C5, acrobot_QP_solver_V4);
acrobot_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_14(acrobot_QP_solver_Phi4, acrobot_QP_solver_D2, acrobot_QP_solver_W4);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_14_5(acrobot_QP_solver_W4, acrobot_QP_solver_V4, acrobot_QP_solver_Ysd4);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi4, acrobot_QP_solver_rd4, acrobot_QP_solver_Lbyrd4);
acrobot_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(acrobot_QP_solver_H0, acrobot_QP_solver_llbbyslb5, acrobot_QP_solver_lbIdx5, acrobot_QP_solver_lubbysub5, acrobot_QP_solver_ubIdx5, acrobot_QP_solver_Phi5);
acrobot_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_14(acrobot_QP_solver_Phi5, params->C6, acrobot_QP_solver_V5);
acrobot_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_14(acrobot_QP_solver_Phi5, acrobot_QP_solver_D2, acrobot_QP_solver_W5);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_14_5(acrobot_QP_solver_W5, acrobot_QP_solver_V5, acrobot_QP_solver_Ysd5);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi5, acrobot_QP_solver_rd5, acrobot_QP_solver_Lbyrd5);
acrobot_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(acrobot_QP_solver_H0, acrobot_QP_solver_llbbyslb6, acrobot_QP_solver_lbIdx6, acrobot_QP_solver_lubbysub6, acrobot_QP_solver_ubIdx6, acrobot_QP_solver_Phi6);
acrobot_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_14(acrobot_QP_solver_Phi6, params->C7, acrobot_QP_solver_V6);
acrobot_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_14(acrobot_QP_solver_Phi6, acrobot_QP_solver_D2, acrobot_QP_solver_W6);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_14_5(acrobot_QP_solver_W6, acrobot_QP_solver_V6, acrobot_QP_solver_Ysd6);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi6, acrobot_QP_solver_rd6, acrobot_QP_solver_Lbyrd6);
acrobot_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(acrobot_QP_solver_H0, acrobot_QP_solver_llbbyslb7, acrobot_QP_solver_lbIdx7, acrobot_QP_solver_lubbysub7, acrobot_QP_solver_ubIdx7, acrobot_QP_solver_Phi7);
acrobot_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_14(acrobot_QP_solver_Phi7, params->C8, acrobot_QP_solver_V7);
acrobot_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_14(acrobot_QP_solver_Phi7, acrobot_QP_solver_D2, acrobot_QP_solver_W7);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_14_5(acrobot_QP_solver_W7, acrobot_QP_solver_V7, acrobot_QP_solver_Ysd7);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi7, acrobot_QP_solver_rd7, acrobot_QP_solver_Lbyrd7);
acrobot_QP_solver_LA_DIAG_CHOL_LBUB_14_14_6(acrobot_QP_solver_H0, acrobot_QP_solver_llbbyslb8, acrobot_QP_solver_lbIdx8, acrobot_QP_solver_lubbysub8, acrobot_QP_solver_ubIdx8, acrobot_QP_solver_Phi8);
acrobot_QP_solver_LA_DIAG_MATRIXFORWARDSUB_5_14(acrobot_QP_solver_Phi8, params->C9, acrobot_QP_solver_V8);
acrobot_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_14(acrobot_QP_solver_Phi8, acrobot_QP_solver_D2, acrobot_QP_solver_W8);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MMTM_5_14_5(acrobot_QP_solver_W8, acrobot_QP_solver_V8, acrobot_QP_solver_Ysd8);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi8, acrobot_QP_solver_rd8, acrobot_QP_solver_Lbyrd8);
acrobot_QP_solver_LA_DIAG_CHOL_ONELOOP_LBUB_5_5_5(acrobot_QP_solver_H9, acrobot_QP_solver_llbbyslb9, acrobot_QP_solver_lbIdx9, acrobot_QP_solver_lubbysub9, acrobot_QP_solver_ubIdx9, acrobot_QP_solver_Phi9);
acrobot_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_5_5(acrobot_QP_solver_Phi9, acrobot_QP_solver_D9, acrobot_QP_solver_W9);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_5(acrobot_QP_solver_Phi9, acrobot_QP_solver_rd9, acrobot_QP_solver_Lbyrd9);
acrobot_QP_solver_LA_DENSE_MMT2_9_14_14(acrobot_QP_solver_V0, acrobot_QP_solver_W1, acrobot_QP_solver_Yd0);
acrobot_QP_solver_LA_DENSE_MVMSUB2_9_14_14(acrobot_QP_solver_V0, acrobot_QP_solver_Lbyrd0, acrobot_QP_solver_W1, acrobot_QP_solver_Lbyrd1, acrobot_QP_solver_re0, acrobot_QP_solver_beta0);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_14(acrobot_QP_solver_V1, acrobot_QP_solver_W2, acrobot_QP_solver_Yd1);
acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_14(acrobot_QP_solver_V1, acrobot_QP_solver_Lbyrd1, acrobot_QP_solver_W2, acrobot_QP_solver_Lbyrd2, acrobot_QP_solver_re1, acrobot_QP_solver_beta1);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_14(acrobot_QP_solver_V2, acrobot_QP_solver_W3, acrobot_QP_solver_Yd2);
acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_14(acrobot_QP_solver_V2, acrobot_QP_solver_Lbyrd2, acrobot_QP_solver_W3, acrobot_QP_solver_Lbyrd3, acrobot_QP_solver_re2, acrobot_QP_solver_beta2);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_14(acrobot_QP_solver_V3, acrobot_QP_solver_W4, acrobot_QP_solver_Yd3);
acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_14(acrobot_QP_solver_V3, acrobot_QP_solver_Lbyrd3, acrobot_QP_solver_W4, acrobot_QP_solver_Lbyrd4, acrobot_QP_solver_re3, acrobot_QP_solver_beta3);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_14(acrobot_QP_solver_V4, acrobot_QP_solver_W5, acrobot_QP_solver_Yd4);
acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_14(acrobot_QP_solver_V4, acrobot_QP_solver_Lbyrd4, acrobot_QP_solver_W5, acrobot_QP_solver_Lbyrd5, acrobot_QP_solver_re4, acrobot_QP_solver_beta4);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_14(acrobot_QP_solver_V5, acrobot_QP_solver_W6, acrobot_QP_solver_Yd5);
acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_14(acrobot_QP_solver_V5, acrobot_QP_solver_Lbyrd5, acrobot_QP_solver_W6, acrobot_QP_solver_Lbyrd6, acrobot_QP_solver_re5, acrobot_QP_solver_beta5);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_14(acrobot_QP_solver_V6, acrobot_QP_solver_W7, acrobot_QP_solver_Yd6);
acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_14(acrobot_QP_solver_V6, acrobot_QP_solver_Lbyrd6, acrobot_QP_solver_W7, acrobot_QP_solver_Lbyrd7, acrobot_QP_solver_re6, acrobot_QP_solver_beta6);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_14(acrobot_QP_solver_V7, acrobot_QP_solver_W8, acrobot_QP_solver_Yd7);
acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_14(acrobot_QP_solver_V7, acrobot_QP_solver_Lbyrd7, acrobot_QP_solver_W8, acrobot_QP_solver_Lbyrd8, acrobot_QP_solver_re7, acrobot_QP_solver_beta7);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MMT2_5_14_5(acrobot_QP_solver_V8, acrobot_QP_solver_W9, acrobot_QP_solver_Yd8);
acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_5_14_5(acrobot_QP_solver_V8, acrobot_QP_solver_Lbyrd8, acrobot_QP_solver_W9, acrobot_QP_solver_Lbyrd9, acrobot_QP_solver_re8, acrobot_QP_solver_beta8);
acrobot_QP_solver_LA_DENSE_CHOL_9(acrobot_QP_solver_Yd0, acrobot_QP_solver_Ld0);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_9(acrobot_QP_solver_Ld0, acrobot_QP_solver_beta0, acrobot_QP_solver_yy0);
acrobot_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_9(acrobot_QP_solver_Ld0, acrobot_QP_solver_Ysd1, acrobot_QP_solver_Lsd1);
acrobot_QP_solver_LA_DENSE_MMTSUB_5_9(acrobot_QP_solver_Lsd1, acrobot_QP_solver_Yd1);
acrobot_QP_solver_LA_DENSE_CHOL_5(acrobot_QP_solver_Yd1, acrobot_QP_solver_Ld1);
acrobot_QP_solver_LA_DENSE_MVMSUB1_5_9(acrobot_QP_solver_Lsd1, acrobot_QP_solver_yy0, acrobot_QP_solver_beta1, acrobot_QP_solver_bmy1);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_5(acrobot_QP_solver_Ld1, acrobot_QP_solver_bmy1, acrobot_QP_solver_yy1);
acrobot_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(acrobot_QP_solver_Ld1, acrobot_QP_solver_Ysd2, acrobot_QP_solver_Lsd2);
acrobot_QP_solver_LA_DENSE_MMTSUB_5_5(acrobot_QP_solver_Lsd2, acrobot_QP_solver_Yd2);
acrobot_QP_solver_LA_DENSE_CHOL_5(acrobot_QP_solver_Yd2, acrobot_QP_solver_Ld2);
acrobot_QP_solver_LA_DENSE_MVMSUB1_5_5(acrobot_QP_solver_Lsd2, acrobot_QP_solver_yy1, acrobot_QP_solver_beta2, acrobot_QP_solver_bmy2);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_5(acrobot_QP_solver_Ld2, acrobot_QP_solver_bmy2, acrobot_QP_solver_yy2);
acrobot_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(acrobot_QP_solver_Ld2, acrobot_QP_solver_Ysd3, acrobot_QP_solver_Lsd3);
acrobot_QP_solver_LA_DENSE_MMTSUB_5_5(acrobot_QP_solver_Lsd3, acrobot_QP_solver_Yd3);
acrobot_QP_solver_LA_DENSE_CHOL_5(acrobot_QP_solver_Yd3, acrobot_QP_solver_Ld3);
acrobot_QP_solver_LA_DENSE_MVMSUB1_5_5(acrobot_QP_solver_Lsd3, acrobot_QP_solver_yy2, acrobot_QP_solver_beta3, acrobot_QP_solver_bmy3);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_5(acrobot_QP_solver_Ld3, acrobot_QP_solver_bmy3, acrobot_QP_solver_yy3);
acrobot_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(acrobot_QP_solver_Ld3, acrobot_QP_solver_Ysd4, acrobot_QP_solver_Lsd4);
acrobot_QP_solver_LA_DENSE_MMTSUB_5_5(acrobot_QP_solver_Lsd4, acrobot_QP_solver_Yd4);
acrobot_QP_solver_LA_DENSE_CHOL_5(acrobot_QP_solver_Yd4, acrobot_QP_solver_Ld4);
acrobot_QP_solver_LA_DENSE_MVMSUB1_5_5(acrobot_QP_solver_Lsd4, acrobot_QP_solver_yy3, acrobot_QP_solver_beta4, acrobot_QP_solver_bmy4);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_5(acrobot_QP_solver_Ld4, acrobot_QP_solver_bmy4, acrobot_QP_solver_yy4);
acrobot_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(acrobot_QP_solver_Ld4, acrobot_QP_solver_Ysd5, acrobot_QP_solver_Lsd5);
acrobot_QP_solver_LA_DENSE_MMTSUB_5_5(acrobot_QP_solver_Lsd5, acrobot_QP_solver_Yd5);
acrobot_QP_solver_LA_DENSE_CHOL_5(acrobot_QP_solver_Yd5, acrobot_QP_solver_Ld5);
acrobot_QP_solver_LA_DENSE_MVMSUB1_5_5(acrobot_QP_solver_Lsd5, acrobot_QP_solver_yy4, acrobot_QP_solver_beta5, acrobot_QP_solver_bmy5);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_5(acrobot_QP_solver_Ld5, acrobot_QP_solver_bmy5, acrobot_QP_solver_yy5);
acrobot_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(acrobot_QP_solver_Ld5, acrobot_QP_solver_Ysd6, acrobot_QP_solver_Lsd6);
acrobot_QP_solver_LA_DENSE_MMTSUB_5_5(acrobot_QP_solver_Lsd6, acrobot_QP_solver_Yd6);
acrobot_QP_solver_LA_DENSE_CHOL_5(acrobot_QP_solver_Yd6, acrobot_QP_solver_Ld6);
acrobot_QP_solver_LA_DENSE_MVMSUB1_5_5(acrobot_QP_solver_Lsd6, acrobot_QP_solver_yy5, acrobot_QP_solver_beta6, acrobot_QP_solver_bmy6);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_5(acrobot_QP_solver_Ld6, acrobot_QP_solver_bmy6, acrobot_QP_solver_yy6);
acrobot_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(acrobot_QP_solver_Ld6, acrobot_QP_solver_Ysd7, acrobot_QP_solver_Lsd7);
acrobot_QP_solver_LA_DENSE_MMTSUB_5_5(acrobot_QP_solver_Lsd7, acrobot_QP_solver_Yd7);
acrobot_QP_solver_LA_DENSE_CHOL_5(acrobot_QP_solver_Yd7, acrobot_QP_solver_Ld7);
acrobot_QP_solver_LA_DENSE_MVMSUB1_5_5(acrobot_QP_solver_Lsd7, acrobot_QP_solver_yy6, acrobot_QP_solver_beta7, acrobot_QP_solver_bmy7);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_5(acrobot_QP_solver_Ld7, acrobot_QP_solver_bmy7, acrobot_QP_solver_yy7);
acrobot_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_5(acrobot_QP_solver_Ld7, acrobot_QP_solver_Ysd8, acrobot_QP_solver_Lsd8);
acrobot_QP_solver_LA_DENSE_MMTSUB_5_5(acrobot_QP_solver_Lsd8, acrobot_QP_solver_Yd8);
acrobot_QP_solver_LA_DENSE_CHOL_5(acrobot_QP_solver_Yd8, acrobot_QP_solver_Ld8);
acrobot_QP_solver_LA_DENSE_MVMSUB1_5_5(acrobot_QP_solver_Lsd8, acrobot_QP_solver_yy7, acrobot_QP_solver_beta8, acrobot_QP_solver_bmy8);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_5(acrobot_QP_solver_Ld8, acrobot_QP_solver_bmy8, acrobot_QP_solver_yy8);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_5(acrobot_QP_solver_Ld8, acrobot_QP_solver_yy8, acrobot_QP_solver_dvaff8);
acrobot_QP_solver_LA_DENSE_MTVMSUB_5_5(acrobot_QP_solver_Lsd8, acrobot_QP_solver_dvaff8, acrobot_QP_solver_yy7, acrobot_QP_solver_bmy7);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_5(acrobot_QP_solver_Ld7, acrobot_QP_solver_bmy7, acrobot_QP_solver_dvaff7);
acrobot_QP_solver_LA_DENSE_MTVMSUB_5_5(acrobot_QP_solver_Lsd7, acrobot_QP_solver_dvaff7, acrobot_QP_solver_yy6, acrobot_QP_solver_bmy6);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_5(acrobot_QP_solver_Ld6, acrobot_QP_solver_bmy6, acrobot_QP_solver_dvaff6);
acrobot_QP_solver_LA_DENSE_MTVMSUB_5_5(acrobot_QP_solver_Lsd6, acrobot_QP_solver_dvaff6, acrobot_QP_solver_yy5, acrobot_QP_solver_bmy5);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_5(acrobot_QP_solver_Ld5, acrobot_QP_solver_bmy5, acrobot_QP_solver_dvaff5);
acrobot_QP_solver_LA_DENSE_MTVMSUB_5_5(acrobot_QP_solver_Lsd5, acrobot_QP_solver_dvaff5, acrobot_QP_solver_yy4, acrobot_QP_solver_bmy4);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_5(acrobot_QP_solver_Ld4, acrobot_QP_solver_bmy4, acrobot_QP_solver_dvaff4);
acrobot_QP_solver_LA_DENSE_MTVMSUB_5_5(acrobot_QP_solver_Lsd4, acrobot_QP_solver_dvaff4, acrobot_QP_solver_yy3, acrobot_QP_solver_bmy3);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_5(acrobot_QP_solver_Ld3, acrobot_QP_solver_bmy3, acrobot_QP_solver_dvaff3);
acrobot_QP_solver_LA_DENSE_MTVMSUB_5_5(acrobot_QP_solver_Lsd3, acrobot_QP_solver_dvaff3, acrobot_QP_solver_yy2, acrobot_QP_solver_bmy2);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_5(acrobot_QP_solver_Ld2, acrobot_QP_solver_bmy2, acrobot_QP_solver_dvaff2);
acrobot_QP_solver_LA_DENSE_MTVMSUB_5_5(acrobot_QP_solver_Lsd2, acrobot_QP_solver_dvaff2, acrobot_QP_solver_yy1, acrobot_QP_solver_bmy1);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_5(acrobot_QP_solver_Ld1, acrobot_QP_solver_bmy1, acrobot_QP_solver_dvaff1);
acrobot_QP_solver_LA_DENSE_MTVMSUB_5_9(acrobot_QP_solver_Lsd1, acrobot_QP_solver_dvaff1, acrobot_QP_solver_yy0, acrobot_QP_solver_bmy0);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_9(acrobot_QP_solver_Ld0, acrobot_QP_solver_bmy0, acrobot_QP_solver_dvaff0);
acrobot_QP_solver_LA_DENSE_MTVM_9_14(params->C1, acrobot_QP_solver_dvaff0, acrobot_QP_solver_grad_eq0);
acrobot_QP_solver_LA_DENSE_MTVM2_5_14_9(params->C2, acrobot_QP_solver_dvaff1, acrobot_QP_solver_D1, acrobot_QP_solver_dvaff0, acrobot_QP_solver_grad_eq1);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C3, acrobot_QP_solver_dvaff2, acrobot_QP_solver_D2, acrobot_QP_solver_dvaff1, acrobot_QP_solver_grad_eq2);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C4, acrobot_QP_solver_dvaff3, acrobot_QP_solver_D2, acrobot_QP_solver_dvaff2, acrobot_QP_solver_grad_eq3);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C5, acrobot_QP_solver_dvaff4, acrobot_QP_solver_D2, acrobot_QP_solver_dvaff3, acrobot_QP_solver_grad_eq4);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C6, acrobot_QP_solver_dvaff5, acrobot_QP_solver_D2, acrobot_QP_solver_dvaff4, acrobot_QP_solver_grad_eq5);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C7, acrobot_QP_solver_dvaff6, acrobot_QP_solver_D2, acrobot_QP_solver_dvaff5, acrobot_QP_solver_grad_eq6);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C8, acrobot_QP_solver_dvaff7, acrobot_QP_solver_D2, acrobot_QP_solver_dvaff6, acrobot_QP_solver_grad_eq7);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C9, acrobot_QP_solver_dvaff8, acrobot_QP_solver_D2, acrobot_QP_solver_dvaff7, acrobot_QP_solver_grad_eq8);
acrobot_QP_solver_LA_DIAGZERO_MTVM_5_5(acrobot_QP_solver_D9, acrobot_QP_solver_dvaff8, acrobot_QP_solver_grad_eq9);
acrobot_QP_solver_LA_VSUB2_131(acrobot_QP_solver_rd, acrobot_QP_solver_grad_eq, acrobot_QP_solver_rd);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi0, acrobot_QP_solver_rd0, acrobot_QP_solver_dzaff0);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi1, acrobot_QP_solver_rd1, acrobot_QP_solver_dzaff1);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi2, acrobot_QP_solver_rd2, acrobot_QP_solver_dzaff2);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi3, acrobot_QP_solver_rd3, acrobot_QP_solver_dzaff3);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi4, acrobot_QP_solver_rd4, acrobot_QP_solver_dzaff4);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi5, acrobot_QP_solver_rd5, acrobot_QP_solver_dzaff5);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi6, acrobot_QP_solver_rd6, acrobot_QP_solver_dzaff6);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi7, acrobot_QP_solver_rd7, acrobot_QP_solver_dzaff7);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi8, acrobot_QP_solver_rd8, acrobot_QP_solver_dzaff8);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_5(acrobot_QP_solver_Phi9, acrobot_QP_solver_rd9, acrobot_QP_solver_dzaff9);
acrobot_QP_solver_LA_VSUB_INDEXED_14(acrobot_QP_solver_dzaff0, acrobot_QP_solver_lbIdx0, acrobot_QP_solver_rilb0, acrobot_QP_solver_dslbaff0);
acrobot_QP_solver_LA_VSUB3_14(acrobot_QP_solver_llbbyslb0, acrobot_QP_solver_dslbaff0, acrobot_QP_solver_llb0, acrobot_QP_solver_dllbaff0);
acrobot_QP_solver_LA_VSUB2_INDEXED_6(acrobot_QP_solver_riub0, acrobot_QP_solver_dzaff0, acrobot_QP_solver_ubIdx0, acrobot_QP_solver_dsubaff0);
acrobot_QP_solver_LA_VSUB3_6(acrobot_QP_solver_lubbysub0, acrobot_QP_solver_dsubaff0, acrobot_QP_solver_lub0, acrobot_QP_solver_dlubaff0);
acrobot_QP_solver_LA_VSUB_INDEXED_14(acrobot_QP_solver_dzaff1, acrobot_QP_solver_lbIdx1, acrobot_QP_solver_rilb1, acrobot_QP_solver_dslbaff1);
acrobot_QP_solver_LA_VSUB3_14(acrobot_QP_solver_llbbyslb1, acrobot_QP_solver_dslbaff1, acrobot_QP_solver_llb1, acrobot_QP_solver_dllbaff1);
acrobot_QP_solver_LA_VSUB2_INDEXED_6(acrobot_QP_solver_riub1, acrobot_QP_solver_dzaff1, acrobot_QP_solver_ubIdx1, acrobot_QP_solver_dsubaff1);
acrobot_QP_solver_LA_VSUB3_6(acrobot_QP_solver_lubbysub1, acrobot_QP_solver_dsubaff1, acrobot_QP_solver_lub1, acrobot_QP_solver_dlubaff1);
acrobot_QP_solver_LA_VSUB_INDEXED_14(acrobot_QP_solver_dzaff2, acrobot_QP_solver_lbIdx2, acrobot_QP_solver_rilb2, acrobot_QP_solver_dslbaff2);
acrobot_QP_solver_LA_VSUB3_14(acrobot_QP_solver_llbbyslb2, acrobot_QP_solver_dslbaff2, acrobot_QP_solver_llb2, acrobot_QP_solver_dllbaff2);
acrobot_QP_solver_LA_VSUB2_INDEXED_6(acrobot_QP_solver_riub2, acrobot_QP_solver_dzaff2, acrobot_QP_solver_ubIdx2, acrobot_QP_solver_dsubaff2);
acrobot_QP_solver_LA_VSUB3_6(acrobot_QP_solver_lubbysub2, acrobot_QP_solver_dsubaff2, acrobot_QP_solver_lub2, acrobot_QP_solver_dlubaff2);
acrobot_QP_solver_LA_VSUB_INDEXED_14(acrobot_QP_solver_dzaff3, acrobot_QP_solver_lbIdx3, acrobot_QP_solver_rilb3, acrobot_QP_solver_dslbaff3);
acrobot_QP_solver_LA_VSUB3_14(acrobot_QP_solver_llbbyslb3, acrobot_QP_solver_dslbaff3, acrobot_QP_solver_llb3, acrobot_QP_solver_dllbaff3);
acrobot_QP_solver_LA_VSUB2_INDEXED_6(acrobot_QP_solver_riub3, acrobot_QP_solver_dzaff3, acrobot_QP_solver_ubIdx3, acrobot_QP_solver_dsubaff3);
acrobot_QP_solver_LA_VSUB3_6(acrobot_QP_solver_lubbysub3, acrobot_QP_solver_dsubaff3, acrobot_QP_solver_lub3, acrobot_QP_solver_dlubaff3);
acrobot_QP_solver_LA_VSUB_INDEXED_14(acrobot_QP_solver_dzaff4, acrobot_QP_solver_lbIdx4, acrobot_QP_solver_rilb4, acrobot_QP_solver_dslbaff4);
acrobot_QP_solver_LA_VSUB3_14(acrobot_QP_solver_llbbyslb4, acrobot_QP_solver_dslbaff4, acrobot_QP_solver_llb4, acrobot_QP_solver_dllbaff4);
acrobot_QP_solver_LA_VSUB2_INDEXED_6(acrobot_QP_solver_riub4, acrobot_QP_solver_dzaff4, acrobot_QP_solver_ubIdx4, acrobot_QP_solver_dsubaff4);
acrobot_QP_solver_LA_VSUB3_6(acrobot_QP_solver_lubbysub4, acrobot_QP_solver_dsubaff4, acrobot_QP_solver_lub4, acrobot_QP_solver_dlubaff4);
acrobot_QP_solver_LA_VSUB_INDEXED_14(acrobot_QP_solver_dzaff5, acrobot_QP_solver_lbIdx5, acrobot_QP_solver_rilb5, acrobot_QP_solver_dslbaff5);
acrobot_QP_solver_LA_VSUB3_14(acrobot_QP_solver_llbbyslb5, acrobot_QP_solver_dslbaff5, acrobot_QP_solver_llb5, acrobot_QP_solver_dllbaff5);
acrobot_QP_solver_LA_VSUB2_INDEXED_6(acrobot_QP_solver_riub5, acrobot_QP_solver_dzaff5, acrobot_QP_solver_ubIdx5, acrobot_QP_solver_dsubaff5);
acrobot_QP_solver_LA_VSUB3_6(acrobot_QP_solver_lubbysub5, acrobot_QP_solver_dsubaff5, acrobot_QP_solver_lub5, acrobot_QP_solver_dlubaff5);
acrobot_QP_solver_LA_VSUB_INDEXED_14(acrobot_QP_solver_dzaff6, acrobot_QP_solver_lbIdx6, acrobot_QP_solver_rilb6, acrobot_QP_solver_dslbaff6);
acrobot_QP_solver_LA_VSUB3_14(acrobot_QP_solver_llbbyslb6, acrobot_QP_solver_dslbaff6, acrobot_QP_solver_llb6, acrobot_QP_solver_dllbaff6);
acrobot_QP_solver_LA_VSUB2_INDEXED_6(acrobot_QP_solver_riub6, acrobot_QP_solver_dzaff6, acrobot_QP_solver_ubIdx6, acrobot_QP_solver_dsubaff6);
acrobot_QP_solver_LA_VSUB3_6(acrobot_QP_solver_lubbysub6, acrobot_QP_solver_dsubaff6, acrobot_QP_solver_lub6, acrobot_QP_solver_dlubaff6);
acrobot_QP_solver_LA_VSUB_INDEXED_14(acrobot_QP_solver_dzaff7, acrobot_QP_solver_lbIdx7, acrobot_QP_solver_rilb7, acrobot_QP_solver_dslbaff7);
acrobot_QP_solver_LA_VSUB3_14(acrobot_QP_solver_llbbyslb7, acrobot_QP_solver_dslbaff7, acrobot_QP_solver_llb7, acrobot_QP_solver_dllbaff7);
acrobot_QP_solver_LA_VSUB2_INDEXED_6(acrobot_QP_solver_riub7, acrobot_QP_solver_dzaff7, acrobot_QP_solver_ubIdx7, acrobot_QP_solver_dsubaff7);
acrobot_QP_solver_LA_VSUB3_6(acrobot_QP_solver_lubbysub7, acrobot_QP_solver_dsubaff7, acrobot_QP_solver_lub7, acrobot_QP_solver_dlubaff7);
acrobot_QP_solver_LA_VSUB_INDEXED_14(acrobot_QP_solver_dzaff8, acrobot_QP_solver_lbIdx8, acrobot_QP_solver_rilb8, acrobot_QP_solver_dslbaff8);
acrobot_QP_solver_LA_VSUB3_14(acrobot_QP_solver_llbbyslb8, acrobot_QP_solver_dslbaff8, acrobot_QP_solver_llb8, acrobot_QP_solver_dllbaff8);
acrobot_QP_solver_LA_VSUB2_INDEXED_6(acrobot_QP_solver_riub8, acrobot_QP_solver_dzaff8, acrobot_QP_solver_ubIdx8, acrobot_QP_solver_dsubaff8);
acrobot_QP_solver_LA_VSUB3_6(acrobot_QP_solver_lubbysub8, acrobot_QP_solver_dsubaff8, acrobot_QP_solver_lub8, acrobot_QP_solver_dlubaff8);
acrobot_QP_solver_LA_VSUB_INDEXED_5(acrobot_QP_solver_dzaff9, acrobot_QP_solver_lbIdx9, acrobot_QP_solver_rilb9, acrobot_QP_solver_dslbaff9);
acrobot_QP_solver_LA_VSUB3_5(acrobot_QP_solver_llbbyslb9, acrobot_QP_solver_dslbaff9, acrobot_QP_solver_llb9, acrobot_QP_solver_dllbaff9);
acrobot_QP_solver_LA_VSUB2_INDEXED_5(acrobot_QP_solver_riub9, acrobot_QP_solver_dzaff9, acrobot_QP_solver_ubIdx9, acrobot_QP_solver_dsubaff9);
acrobot_QP_solver_LA_VSUB3_5(acrobot_QP_solver_lubbysub9, acrobot_QP_solver_dsubaff9, acrobot_QP_solver_lub9, acrobot_QP_solver_dlubaff9);
info->lsit_aff = acrobot_QP_solver_LINESEARCH_BACKTRACKING_AFFINE(acrobot_QP_solver_l, acrobot_QP_solver_s, acrobot_QP_solver_dl_aff, acrobot_QP_solver_ds_aff, &info->step_aff, &info->mu_aff);
if( info->lsit_aff == acrobot_QP_solver_NOPROGRESS ){
exitcode = acrobot_QP_solver_NOPROGRESS; break;
}
sigma_3rdroot = info->mu_aff / info->mu;
info->sigma = sigma_3rdroot*sigma_3rdroot*sigma_3rdroot;
musigma = info->mu * info->sigma;
acrobot_QP_solver_LA_VSUB5_190(acrobot_QP_solver_ds_aff, acrobot_QP_solver_dl_aff, info->mu, info->sigma, acrobot_QP_solver_ccrhs);
acrobot_QP_solver_LA_VSUB6_INDEXED_14_6_14(acrobot_QP_solver_ccrhsub0, acrobot_QP_solver_sub0, acrobot_QP_solver_ubIdx0, acrobot_QP_solver_ccrhsl0, acrobot_QP_solver_slb0, acrobot_QP_solver_lbIdx0, acrobot_QP_solver_rd0);
acrobot_QP_solver_LA_VSUB6_INDEXED_14_6_14(acrobot_QP_solver_ccrhsub1, acrobot_QP_solver_sub1, acrobot_QP_solver_ubIdx1, acrobot_QP_solver_ccrhsl1, acrobot_QP_solver_slb1, acrobot_QP_solver_lbIdx1, acrobot_QP_solver_rd1);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi0, acrobot_QP_solver_rd0, acrobot_QP_solver_Lbyrd0);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi1, acrobot_QP_solver_rd1, acrobot_QP_solver_Lbyrd1);
acrobot_QP_solver_LA_DENSE_2MVMADD_9_14_14(acrobot_QP_solver_V0, acrobot_QP_solver_Lbyrd0, acrobot_QP_solver_W1, acrobot_QP_solver_Lbyrd1, acrobot_QP_solver_beta0);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_9(acrobot_QP_solver_Ld0, acrobot_QP_solver_beta0, acrobot_QP_solver_yy0);
acrobot_QP_solver_LA_VSUB6_INDEXED_14_6_14(acrobot_QP_solver_ccrhsub2, acrobot_QP_solver_sub2, acrobot_QP_solver_ubIdx2, acrobot_QP_solver_ccrhsl2, acrobot_QP_solver_slb2, acrobot_QP_solver_lbIdx2, acrobot_QP_solver_rd2);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi2, acrobot_QP_solver_rd2, acrobot_QP_solver_Lbyrd2);
acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_14(acrobot_QP_solver_V1, acrobot_QP_solver_Lbyrd1, acrobot_QP_solver_W2, acrobot_QP_solver_Lbyrd2, acrobot_QP_solver_beta1);
acrobot_QP_solver_LA_DENSE_MVMSUB1_5_9(acrobot_QP_solver_Lsd1, acrobot_QP_solver_yy0, acrobot_QP_solver_beta1, acrobot_QP_solver_bmy1);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_5(acrobot_QP_solver_Ld1, acrobot_QP_solver_bmy1, acrobot_QP_solver_yy1);
acrobot_QP_solver_LA_VSUB6_INDEXED_14_6_14(acrobot_QP_solver_ccrhsub3, acrobot_QP_solver_sub3, acrobot_QP_solver_ubIdx3, acrobot_QP_solver_ccrhsl3, acrobot_QP_solver_slb3, acrobot_QP_solver_lbIdx3, acrobot_QP_solver_rd3);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi3, acrobot_QP_solver_rd3, acrobot_QP_solver_Lbyrd3);
acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_14(acrobot_QP_solver_V2, acrobot_QP_solver_Lbyrd2, acrobot_QP_solver_W3, acrobot_QP_solver_Lbyrd3, acrobot_QP_solver_beta2);
acrobot_QP_solver_LA_DENSE_MVMSUB1_5_5(acrobot_QP_solver_Lsd2, acrobot_QP_solver_yy1, acrobot_QP_solver_beta2, acrobot_QP_solver_bmy2);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_5(acrobot_QP_solver_Ld2, acrobot_QP_solver_bmy2, acrobot_QP_solver_yy2);
acrobot_QP_solver_LA_VSUB6_INDEXED_14_6_14(acrobot_QP_solver_ccrhsub4, acrobot_QP_solver_sub4, acrobot_QP_solver_ubIdx4, acrobot_QP_solver_ccrhsl4, acrobot_QP_solver_slb4, acrobot_QP_solver_lbIdx4, acrobot_QP_solver_rd4);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi4, acrobot_QP_solver_rd4, acrobot_QP_solver_Lbyrd4);
acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_14(acrobot_QP_solver_V3, acrobot_QP_solver_Lbyrd3, acrobot_QP_solver_W4, acrobot_QP_solver_Lbyrd4, acrobot_QP_solver_beta3);
acrobot_QP_solver_LA_DENSE_MVMSUB1_5_5(acrobot_QP_solver_Lsd3, acrobot_QP_solver_yy2, acrobot_QP_solver_beta3, acrobot_QP_solver_bmy3);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_5(acrobot_QP_solver_Ld3, acrobot_QP_solver_bmy3, acrobot_QP_solver_yy3);
acrobot_QP_solver_LA_VSUB6_INDEXED_14_6_14(acrobot_QP_solver_ccrhsub5, acrobot_QP_solver_sub5, acrobot_QP_solver_ubIdx5, acrobot_QP_solver_ccrhsl5, acrobot_QP_solver_slb5, acrobot_QP_solver_lbIdx5, acrobot_QP_solver_rd5);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi5, acrobot_QP_solver_rd5, acrobot_QP_solver_Lbyrd5);
acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_14(acrobot_QP_solver_V4, acrobot_QP_solver_Lbyrd4, acrobot_QP_solver_W5, acrobot_QP_solver_Lbyrd5, acrobot_QP_solver_beta4);
acrobot_QP_solver_LA_DENSE_MVMSUB1_5_5(acrobot_QP_solver_Lsd4, acrobot_QP_solver_yy3, acrobot_QP_solver_beta4, acrobot_QP_solver_bmy4);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_5(acrobot_QP_solver_Ld4, acrobot_QP_solver_bmy4, acrobot_QP_solver_yy4);
acrobot_QP_solver_LA_VSUB6_INDEXED_14_6_14(acrobot_QP_solver_ccrhsub6, acrobot_QP_solver_sub6, acrobot_QP_solver_ubIdx6, acrobot_QP_solver_ccrhsl6, acrobot_QP_solver_slb6, acrobot_QP_solver_lbIdx6, acrobot_QP_solver_rd6);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi6, acrobot_QP_solver_rd6, acrobot_QP_solver_Lbyrd6);
acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_14(acrobot_QP_solver_V5, acrobot_QP_solver_Lbyrd5, acrobot_QP_solver_W6, acrobot_QP_solver_Lbyrd6, acrobot_QP_solver_beta5);
acrobot_QP_solver_LA_DENSE_MVMSUB1_5_5(acrobot_QP_solver_Lsd5, acrobot_QP_solver_yy4, acrobot_QP_solver_beta5, acrobot_QP_solver_bmy5);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_5(acrobot_QP_solver_Ld5, acrobot_QP_solver_bmy5, acrobot_QP_solver_yy5);
acrobot_QP_solver_LA_VSUB6_INDEXED_14_6_14(acrobot_QP_solver_ccrhsub7, acrobot_QP_solver_sub7, acrobot_QP_solver_ubIdx7, acrobot_QP_solver_ccrhsl7, acrobot_QP_solver_slb7, acrobot_QP_solver_lbIdx7, acrobot_QP_solver_rd7);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi7, acrobot_QP_solver_rd7, acrobot_QP_solver_Lbyrd7);
acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_14(acrobot_QP_solver_V6, acrobot_QP_solver_Lbyrd6, acrobot_QP_solver_W7, acrobot_QP_solver_Lbyrd7, acrobot_QP_solver_beta6);
acrobot_QP_solver_LA_DENSE_MVMSUB1_5_5(acrobot_QP_solver_Lsd6, acrobot_QP_solver_yy5, acrobot_QP_solver_beta6, acrobot_QP_solver_bmy6);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_5(acrobot_QP_solver_Ld6, acrobot_QP_solver_bmy6, acrobot_QP_solver_yy6);
acrobot_QP_solver_LA_VSUB6_INDEXED_14_6_14(acrobot_QP_solver_ccrhsub8, acrobot_QP_solver_sub8, acrobot_QP_solver_ubIdx8, acrobot_QP_solver_ccrhsl8, acrobot_QP_solver_slb8, acrobot_QP_solver_lbIdx8, acrobot_QP_solver_rd8);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_14(acrobot_QP_solver_Phi8, acrobot_QP_solver_rd8, acrobot_QP_solver_Lbyrd8);
acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_14(acrobot_QP_solver_V7, acrobot_QP_solver_Lbyrd7, acrobot_QP_solver_W8, acrobot_QP_solver_Lbyrd8, acrobot_QP_solver_beta7);
acrobot_QP_solver_LA_DENSE_MVMSUB1_5_5(acrobot_QP_solver_Lsd7, acrobot_QP_solver_yy6, acrobot_QP_solver_beta7, acrobot_QP_solver_bmy7);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_5(acrobot_QP_solver_Ld7, acrobot_QP_solver_bmy7, acrobot_QP_solver_yy7);
acrobot_QP_solver_LA_VSUB6_INDEXED_5_5_5(acrobot_QP_solver_ccrhsub9, acrobot_QP_solver_sub9, acrobot_QP_solver_ubIdx9, acrobot_QP_solver_ccrhsl9, acrobot_QP_solver_slb9, acrobot_QP_solver_lbIdx9, acrobot_QP_solver_rd9);
acrobot_QP_solver_LA_DIAG_FORWARDSUB_5(acrobot_QP_solver_Phi9, acrobot_QP_solver_rd9, acrobot_QP_solver_Lbyrd9);
acrobot_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_5_14_5(acrobot_QP_solver_V8, acrobot_QP_solver_Lbyrd8, acrobot_QP_solver_W9, acrobot_QP_solver_Lbyrd9, acrobot_QP_solver_beta8);
acrobot_QP_solver_LA_DENSE_MVMSUB1_5_5(acrobot_QP_solver_Lsd8, acrobot_QP_solver_yy7, acrobot_QP_solver_beta8, acrobot_QP_solver_bmy8);
acrobot_QP_solver_LA_DENSE_FORWARDSUB_5(acrobot_QP_solver_Ld8, acrobot_QP_solver_bmy8, acrobot_QP_solver_yy8);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_5(acrobot_QP_solver_Ld8, acrobot_QP_solver_yy8, acrobot_QP_solver_dvcc8);
acrobot_QP_solver_LA_DENSE_MTVMSUB_5_5(acrobot_QP_solver_Lsd8, acrobot_QP_solver_dvcc8, acrobot_QP_solver_yy7, acrobot_QP_solver_bmy7);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_5(acrobot_QP_solver_Ld7, acrobot_QP_solver_bmy7, acrobot_QP_solver_dvcc7);
acrobot_QP_solver_LA_DENSE_MTVMSUB_5_5(acrobot_QP_solver_Lsd7, acrobot_QP_solver_dvcc7, acrobot_QP_solver_yy6, acrobot_QP_solver_bmy6);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_5(acrobot_QP_solver_Ld6, acrobot_QP_solver_bmy6, acrobot_QP_solver_dvcc6);
acrobot_QP_solver_LA_DENSE_MTVMSUB_5_5(acrobot_QP_solver_Lsd6, acrobot_QP_solver_dvcc6, acrobot_QP_solver_yy5, acrobot_QP_solver_bmy5);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_5(acrobot_QP_solver_Ld5, acrobot_QP_solver_bmy5, acrobot_QP_solver_dvcc5);
acrobot_QP_solver_LA_DENSE_MTVMSUB_5_5(acrobot_QP_solver_Lsd5, acrobot_QP_solver_dvcc5, acrobot_QP_solver_yy4, acrobot_QP_solver_bmy4);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_5(acrobot_QP_solver_Ld4, acrobot_QP_solver_bmy4, acrobot_QP_solver_dvcc4);
acrobot_QP_solver_LA_DENSE_MTVMSUB_5_5(acrobot_QP_solver_Lsd4, acrobot_QP_solver_dvcc4, acrobot_QP_solver_yy3, acrobot_QP_solver_bmy3);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_5(acrobot_QP_solver_Ld3, acrobot_QP_solver_bmy3, acrobot_QP_solver_dvcc3);
acrobot_QP_solver_LA_DENSE_MTVMSUB_5_5(acrobot_QP_solver_Lsd3, acrobot_QP_solver_dvcc3, acrobot_QP_solver_yy2, acrobot_QP_solver_bmy2);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_5(acrobot_QP_solver_Ld2, acrobot_QP_solver_bmy2, acrobot_QP_solver_dvcc2);
acrobot_QP_solver_LA_DENSE_MTVMSUB_5_5(acrobot_QP_solver_Lsd2, acrobot_QP_solver_dvcc2, acrobot_QP_solver_yy1, acrobot_QP_solver_bmy1);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_5(acrobot_QP_solver_Ld1, acrobot_QP_solver_bmy1, acrobot_QP_solver_dvcc1);
acrobot_QP_solver_LA_DENSE_MTVMSUB_5_9(acrobot_QP_solver_Lsd1, acrobot_QP_solver_dvcc1, acrobot_QP_solver_yy0, acrobot_QP_solver_bmy0);
acrobot_QP_solver_LA_DENSE_BACKWARDSUB_9(acrobot_QP_solver_Ld0, acrobot_QP_solver_bmy0, acrobot_QP_solver_dvcc0);
acrobot_QP_solver_LA_DENSE_MTVM_9_14(params->C1, acrobot_QP_solver_dvcc0, acrobot_QP_solver_grad_eq0);
acrobot_QP_solver_LA_DENSE_MTVM2_5_14_9(params->C2, acrobot_QP_solver_dvcc1, acrobot_QP_solver_D1, acrobot_QP_solver_dvcc0, acrobot_QP_solver_grad_eq1);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C3, acrobot_QP_solver_dvcc2, acrobot_QP_solver_D2, acrobot_QP_solver_dvcc1, acrobot_QP_solver_grad_eq2);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C4, acrobot_QP_solver_dvcc3, acrobot_QP_solver_D2, acrobot_QP_solver_dvcc2, acrobot_QP_solver_grad_eq3);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C5, acrobot_QP_solver_dvcc4, acrobot_QP_solver_D2, acrobot_QP_solver_dvcc3, acrobot_QP_solver_grad_eq4);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C6, acrobot_QP_solver_dvcc5, acrobot_QP_solver_D2, acrobot_QP_solver_dvcc4, acrobot_QP_solver_grad_eq5);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C7, acrobot_QP_solver_dvcc6, acrobot_QP_solver_D2, acrobot_QP_solver_dvcc5, acrobot_QP_solver_grad_eq6);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C8, acrobot_QP_solver_dvcc7, acrobot_QP_solver_D2, acrobot_QP_solver_dvcc6, acrobot_QP_solver_grad_eq7);
acrobot_QP_solver_LA_DENSE_DIAGZERO_MTVM2_5_14_5(params->C9, acrobot_QP_solver_dvcc8, acrobot_QP_solver_D2, acrobot_QP_solver_dvcc7, acrobot_QP_solver_grad_eq8);
acrobot_QP_solver_LA_DIAGZERO_MTVM_5_5(acrobot_QP_solver_D9, acrobot_QP_solver_dvcc8, acrobot_QP_solver_grad_eq9);
acrobot_QP_solver_LA_VSUB_131(acrobot_QP_solver_rd, acrobot_QP_solver_grad_eq, acrobot_QP_solver_rd);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi0, acrobot_QP_solver_rd0, acrobot_QP_solver_dzcc0);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi1, acrobot_QP_solver_rd1, acrobot_QP_solver_dzcc1);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi2, acrobot_QP_solver_rd2, acrobot_QP_solver_dzcc2);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi3, acrobot_QP_solver_rd3, acrobot_QP_solver_dzcc3);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi4, acrobot_QP_solver_rd4, acrobot_QP_solver_dzcc4);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi5, acrobot_QP_solver_rd5, acrobot_QP_solver_dzcc5);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi6, acrobot_QP_solver_rd6, acrobot_QP_solver_dzcc6);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi7, acrobot_QP_solver_rd7, acrobot_QP_solver_dzcc7);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_14(acrobot_QP_solver_Phi8, acrobot_QP_solver_rd8, acrobot_QP_solver_dzcc8);
acrobot_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_5(acrobot_QP_solver_Phi9, acrobot_QP_solver_rd9, acrobot_QP_solver_dzcc9);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(acrobot_QP_solver_ccrhsl0, acrobot_QP_solver_slb0, acrobot_QP_solver_llbbyslb0, acrobot_QP_solver_dzcc0, acrobot_QP_solver_lbIdx0, acrobot_QP_solver_dllbcc0);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(acrobot_QP_solver_ccrhsub0, acrobot_QP_solver_sub0, acrobot_QP_solver_lubbysub0, acrobot_QP_solver_dzcc0, acrobot_QP_solver_ubIdx0, acrobot_QP_solver_dlubcc0);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(acrobot_QP_solver_ccrhsl1, acrobot_QP_solver_slb1, acrobot_QP_solver_llbbyslb1, acrobot_QP_solver_dzcc1, acrobot_QP_solver_lbIdx1, acrobot_QP_solver_dllbcc1);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(acrobot_QP_solver_ccrhsub1, acrobot_QP_solver_sub1, acrobot_QP_solver_lubbysub1, acrobot_QP_solver_dzcc1, acrobot_QP_solver_ubIdx1, acrobot_QP_solver_dlubcc1);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(acrobot_QP_solver_ccrhsl2, acrobot_QP_solver_slb2, acrobot_QP_solver_llbbyslb2, acrobot_QP_solver_dzcc2, acrobot_QP_solver_lbIdx2, acrobot_QP_solver_dllbcc2);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(acrobot_QP_solver_ccrhsub2, acrobot_QP_solver_sub2, acrobot_QP_solver_lubbysub2, acrobot_QP_solver_dzcc2, acrobot_QP_solver_ubIdx2, acrobot_QP_solver_dlubcc2);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(acrobot_QP_solver_ccrhsl3, acrobot_QP_solver_slb3, acrobot_QP_solver_llbbyslb3, acrobot_QP_solver_dzcc3, acrobot_QP_solver_lbIdx3, acrobot_QP_solver_dllbcc3);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(acrobot_QP_solver_ccrhsub3, acrobot_QP_solver_sub3, acrobot_QP_solver_lubbysub3, acrobot_QP_solver_dzcc3, acrobot_QP_solver_ubIdx3, acrobot_QP_solver_dlubcc3);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(acrobot_QP_solver_ccrhsl4, acrobot_QP_solver_slb4, acrobot_QP_solver_llbbyslb4, acrobot_QP_solver_dzcc4, acrobot_QP_solver_lbIdx4, acrobot_QP_solver_dllbcc4);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(acrobot_QP_solver_ccrhsub4, acrobot_QP_solver_sub4, acrobot_QP_solver_lubbysub4, acrobot_QP_solver_dzcc4, acrobot_QP_solver_ubIdx4, acrobot_QP_solver_dlubcc4);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(acrobot_QP_solver_ccrhsl5, acrobot_QP_solver_slb5, acrobot_QP_solver_llbbyslb5, acrobot_QP_solver_dzcc5, acrobot_QP_solver_lbIdx5, acrobot_QP_solver_dllbcc5);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(acrobot_QP_solver_ccrhsub5, acrobot_QP_solver_sub5, acrobot_QP_solver_lubbysub5, acrobot_QP_solver_dzcc5, acrobot_QP_solver_ubIdx5, acrobot_QP_solver_dlubcc5);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(acrobot_QP_solver_ccrhsl6, acrobot_QP_solver_slb6, acrobot_QP_solver_llbbyslb6, acrobot_QP_solver_dzcc6, acrobot_QP_solver_lbIdx6, acrobot_QP_solver_dllbcc6);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(acrobot_QP_solver_ccrhsub6, acrobot_QP_solver_sub6, acrobot_QP_solver_lubbysub6, acrobot_QP_solver_dzcc6, acrobot_QP_solver_ubIdx6, acrobot_QP_solver_dlubcc6);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(acrobot_QP_solver_ccrhsl7, acrobot_QP_solver_slb7, acrobot_QP_solver_llbbyslb7, acrobot_QP_solver_dzcc7, acrobot_QP_solver_lbIdx7, acrobot_QP_solver_dllbcc7);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(acrobot_QP_solver_ccrhsub7, acrobot_QP_solver_sub7, acrobot_QP_solver_lubbysub7, acrobot_QP_solver_dzcc7, acrobot_QP_solver_ubIdx7, acrobot_QP_solver_dlubcc7);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_14(acrobot_QP_solver_ccrhsl8, acrobot_QP_solver_slb8, acrobot_QP_solver_llbbyslb8, acrobot_QP_solver_dzcc8, acrobot_QP_solver_lbIdx8, acrobot_QP_solver_dllbcc8);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(acrobot_QP_solver_ccrhsub8, acrobot_QP_solver_sub8, acrobot_QP_solver_lubbysub8, acrobot_QP_solver_dzcc8, acrobot_QP_solver_ubIdx8, acrobot_QP_solver_dlubcc8);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_5(acrobot_QP_solver_ccrhsl9, acrobot_QP_solver_slb9, acrobot_QP_solver_llbbyslb9, acrobot_QP_solver_dzcc9, acrobot_QP_solver_lbIdx9, acrobot_QP_solver_dllbcc9);
acrobot_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_5(acrobot_QP_solver_ccrhsub9, acrobot_QP_solver_sub9, acrobot_QP_solver_lubbysub9, acrobot_QP_solver_dzcc9, acrobot_QP_solver_ubIdx9, acrobot_QP_solver_dlubcc9);
acrobot_QP_solver_LA_VSUB7_190(acrobot_QP_solver_l, acrobot_QP_solver_ccrhs, acrobot_QP_solver_s, acrobot_QP_solver_dl_cc, acrobot_QP_solver_ds_cc);
acrobot_QP_solver_LA_VADD_131(acrobot_QP_solver_dz_cc, acrobot_QP_solver_dz_aff);
acrobot_QP_solver_LA_VADD_49(acrobot_QP_solver_dv_cc, acrobot_QP_solver_dv_aff);
acrobot_QP_solver_LA_VADD_190(acrobot_QP_solver_dl_cc, acrobot_QP_solver_dl_aff);
acrobot_QP_solver_LA_VADD_190(acrobot_QP_solver_ds_cc, acrobot_QP_solver_ds_aff);
info->lsit_cc = acrobot_QP_solver_LINESEARCH_BACKTRACKING_COMBINED(acrobot_QP_solver_z, acrobot_QP_solver_v, acrobot_QP_solver_l, acrobot_QP_solver_s, acrobot_QP_solver_dz_cc, acrobot_QP_solver_dv_cc, acrobot_QP_solver_dl_cc, acrobot_QP_solver_ds_cc, &info->step_cc, &info->mu);
if( info->lsit_cc == acrobot_QP_solver_NOPROGRESS ){
exitcode = acrobot_QP_solver_NOPROGRESS; break;
}
info->it++;
}
output->z1[0] = acrobot_QP_solver_z0[0];
output->z1[1] = acrobot_QP_solver_z0[1];
output->z1[2] = acrobot_QP_solver_z0[2];
output->z1[3] = acrobot_QP_solver_z0[3];
output->z1[4] = acrobot_QP_solver_z0[4];
output->z1[5] = acrobot_QP_solver_z0[5];
output->z2[0] = acrobot_QP_solver_z1[0];
output->z2[1] = acrobot_QP_solver_z1[1];
output->z2[2] = acrobot_QP_solver_z1[2];
output->z2[3] = acrobot_QP_solver_z1[3];
output->z2[4] = acrobot_QP_solver_z1[4];
output->z2[5] = acrobot_QP_solver_z1[5];
output->z3[0] = acrobot_QP_solver_z2[0];
output->z3[1] = acrobot_QP_solver_z2[1];
output->z3[2] = acrobot_QP_solver_z2[2];
output->z3[3] = acrobot_QP_solver_z2[3];
output->z3[4] = acrobot_QP_solver_z2[4];
output->z3[5] = acrobot_QP_solver_z2[5];
output->z4[0] = acrobot_QP_solver_z3[0];
output->z4[1] = acrobot_QP_solver_z3[1];
output->z4[2] = acrobot_QP_solver_z3[2];
output->z4[3] = acrobot_QP_solver_z3[3];
output->z4[4] = acrobot_QP_solver_z3[4];
output->z4[5] = acrobot_QP_solver_z3[5];
output->z5[0] = acrobot_QP_solver_z4[0];
output->z5[1] = acrobot_QP_solver_z4[1];
output->z5[2] = acrobot_QP_solver_z4[2];
output->z5[3] = acrobot_QP_solver_z4[3];
output->z5[4] = acrobot_QP_solver_z4[4];
output->z5[5] = acrobot_QP_solver_z4[5];
output->z6[0] = acrobot_QP_solver_z5[0];
output->z6[1] = acrobot_QP_solver_z5[1];
output->z6[2] = acrobot_QP_solver_z5[2];
output->z6[3] = acrobot_QP_solver_z5[3];
output->z6[4] = acrobot_QP_solver_z5[4];
output->z6[5] = acrobot_QP_solver_z5[5];
output->z7[0] = acrobot_QP_solver_z6[0];
output->z7[1] = acrobot_QP_solver_z6[1];
output->z7[2] = acrobot_QP_solver_z6[2];
output->z7[3] = acrobot_QP_solver_z6[3];
output->z7[4] = acrobot_QP_solver_z6[4];
output->z7[5] = acrobot_QP_solver_z6[5];
output->z8[0] = acrobot_QP_solver_z7[0];
output->z8[1] = acrobot_QP_solver_z7[1];
output->z8[2] = acrobot_QP_solver_z7[2];
output->z8[3] = acrobot_QP_solver_z7[3];
output->z8[4] = acrobot_QP_solver_z7[4];
output->z8[5] = acrobot_QP_solver_z7[5];
output->z9[0] = acrobot_QP_solver_z8[0];
output->z9[1] = acrobot_QP_solver_z8[1];
output->z9[2] = acrobot_QP_solver_z8[2];
output->z9[3] = acrobot_QP_solver_z8[3];
output->z9[4] = acrobot_QP_solver_z8[4];
output->z9[5] = acrobot_QP_solver_z8[5];
output->z10[0] = acrobot_QP_solver_z9[0];
output->z10[1] = acrobot_QP_solver_z9[1];
output->z10[2] = acrobot_QP_solver_z9[2];
output->z10[3] = acrobot_QP_solver_z9[3];
output->z10[4] = acrobot_QP_solver_z9[4];

#if acrobot_QP_solver_SET_TIMING == 1
info->solvetime = acrobot_QP_solver_toc(&solvertimer);
#if acrobot_QP_solver_SET_PRINTLEVEL > 0 && acrobot_QP_solver_SET_TIMING == 1
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
