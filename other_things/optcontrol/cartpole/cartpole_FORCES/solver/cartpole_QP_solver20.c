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

#include "cartpole_QP_solver.h"

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
#ifndef USEMEXPRINTS
#include <stdio.h>
#define PRINTTEXT printf
#else
#include "mex.h"
#define PRINTTEXT mexPrintf
#endif

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
 * Initializes a vector of length 288 with a value.
 */
void cartpole_QP_solver_LA_INITIALIZEVECTOR_288(cartpole_QP_solver_FLOAT* vec, cartpole_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<288; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 112 with a value.
 */
void cartpole_QP_solver_LA_INITIALIZEVECTOR_112(cartpole_QP_solver_FLOAT* vec, cartpole_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<112; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 436 with a value.
 */
void cartpole_QP_solver_LA_INITIALIZEVECTOR_436(cartpole_QP_solver_FLOAT* vec, cartpole_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<436; i++ )
	{
		vec[i] = value;
	}
}


/* 
 * Calculates a dot product and adds it to a variable: z += x'*y; 
 * This function is for vectors of length 436.
 */
void cartpole_QP_solver_LA_DOTACC_436(cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<436; i++ ){
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
void cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_FLOAT* H, cartpole_QP_solver_FLOAT* f, cartpole_QP_solver_FLOAT* z, cartpole_QP_solver_FLOAT* grad, cartpole_QP_solver_FLOAT* value)
{
	int i;
	cartpole_QP_solver_FLOAT hz;	
	for( i=0; i<15; i++){
		hz = H[i]*z[i];
		grad[i] = hz + f[i];
		*value += 0.5*hz*z[i] + f[i]*z[i];
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
 * INPUTS:     H  - Symmetric Hessian, diag matrix of size [4 x 4]
 *             f  - column vector of size 4
 *             z  - column vector of size 4
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 4
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void cartpole_QP_solver_LA_DIAG_QUADFCN_4(cartpole_QP_solver_FLOAT* H, cartpole_QP_solver_FLOAT* f, cartpole_QP_solver_FLOAT* z, cartpole_QP_solver_FLOAT* grad, cartpole_QP_solver_FLOAT* value)
{
	int i;
	cartpole_QP_solver_FLOAT hz;	
	for( i=0; i<4; i++){
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
void cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *l, cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *z, cartpole_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;
	cartpole_QP_solver_FLOAT AxBu[6];
	cartpole_QP_solver_FLOAT norm = *y;
	cartpole_QP_solver_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<6; i++ ){
		AxBu[i] = A[k++]*x[0] + B[m++]*u[0];
	}	
	for( j=1; j<15; j++ ){		
		for( i=0; i<6; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<15; n++ ){
		for( i=0; i<6; i++ ){
			AxBu[i] += B[m++]*u[n];
		}		
	}

	for( i=0; i<6; i++ ){
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
void cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *l, cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *z, cartpole_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;
	cartpole_QP_solver_FLOAT AxBu[6];
	cartpole_QP_solver_FLOAT norm = *y;
	cartpole_QP_solver_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<6; i++ ){
		AxBu[i] = A[k++]*x[0] + B[m++]*u[0];
	}	
	for( j=1; j<15; j++ ){		
		for( i=0; i<6; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<14; n++ ){
		for( i=0; i<6; i++ ){
			AxBu[i] += B[m++]*u[n];
		}		
	}

	for( i=0; i<6; i++ ){
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
void cartpole_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_4_14_4(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *l, cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *z, cartpole_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	cartpole_QP_solver_FLOAT AxBu[4];
	cartpole_QP_solver_FLOAT norm = *y;
	cartpole_QP_solver_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<4; i++ ){
		AxBu[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<14; j++ ){		
		for( i=0; i<4; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<4; i++ ){
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
 * Matrix vector multiplication y = M'*x where M is of size [6 x 15]
 * and stored in column major format. Note the transpose of M!
 */
void cartpole_QP_solver_LA_DENSE_MTVM_6_15(cartpole_QP_solver_FLOAT *M, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<15; i++ ){
		y[i] = 0;
		for( j=0; j<6; j++ ){
			y[i] += M[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [6 x 15]
 * and B is of size [6 x 15]
 * and stored in column major format. Note the transposes of A and B!
 */
void cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	int n;
	int m = 0;
	for( i=0; i<15; i++ ){
		z[i] = 0;
		for( j=0; j<6; j++ ){
			z[i] += A[k++]*x[j];
		}
		for( n=0; n<6; n++ ){
			z[i] += B[m++]*y[n];
		}
	}
}


/*
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [4 x 14]
 * and B is of size [6 x 14]
 * and stored in column major format. Note the transposes of A and B!
 */
void cartpole_QP_solver_LA_DENSE_MTVM2_4_14_6(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	int n;
	int m = 0;
	for( i=0; i<14; i++ ){
		z[i] = 0;
		for( j=0; j<4; j++ ){
			z[i] += A[k++]*x[j];
		}
		for( n=0; n<6; n++ ){
			z[i] += B[m++]*y[n];
		}
	}
}


/*
 * Matrix vector multiplication y = M'*x where M is of size [4 x 4]
 * and stored in diagzero format. Note the transpose of M!
 */
void cartpole_QP_solver_LA_DIAGZERO_MTVM_4_4(cartpole_QP_solver_FLOAT *M, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<4; i++ ){
		y[i] = M[i]*x[i];
	}
}


/*
 * Vector subtraction and addition.
 *	 Input: five vectors t, tidx, u, v, w and two scalars z and r
 *	 Output: y = t(tidx) - u + w
 *           z = z - v'*x;
 *           r = max([norm(y,inf), z]);
 * for vectors of length 6. Output z is of course scalar.
 */
void cartpole_QP_solver_LA_VSUBADD3_6(cartpole_QP_solver_FLOAT* t, cartpole_QP_solver_FLOAT* u, int* uidx, cartpole_QP_solver_FLOAT* v, cartpole_QP_solver_FLOAT* w, cartpole_QP_solver_FLOAT* y, cartpole_QP_solver_FLOAT* z, cartpole_QP_solver_FLOAT* r)
{
	int i;
	cartpole_QP_solver_FLOAT norm = *r;
	cartpole_QP_solver_FLOAT vx = 0;
	cartpole_QP_solver_FLOAT x;
	for( i=0; i<6; i++){
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
 * Computes r = A*x - b + s
 * and      y = max([norm(r,inf), y])
 * and      z -= l'*(Ax-b)
 * where A is stored in column major format
 */
void cartpole_QP_solver_LA_MVSUBADD_20_15(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *l, cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *z, cartpole_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	cartpole_QP_solver_FLOAT Ax[20];
	cartpole_QP_solver_FLOAT Axlessb;
	cartpole_QP_solver_FLOAT norm = *y;
	cartpole_QP_solver_FLOAT lAxlessb = 0;

	/* do A*x first */
	for( i=0; i<20; i++ ){
		Ax[i] = A[k++]*x[0];				
	}	
	for( j=1; j<15; j++ ){		
		for( i=0; i<20; i++ ){
			Ax[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<20; i++ ){
		Axlessb = Ax[i] - b[i];
		r[i] = Axlessb + s[i];
		lAxlessb += l[i]*Axlessb;
		if( r[i] > norm ){
			norm = r[i];
		}
		if( -r[i] > norm ){
			norm = -r[i];
		}
	}
	*y = norm;
	*z -= lAxlessb;
}


/*
 * Vector subtraction and addition.
 *	 Input: five vectors t, tidx, u, v, w and two scalars z and r
 *	 Output: y = t(tidx) - u + w
 *           z = z - v'*x;
 *           r = max([norm(y,inf), z]);
 * for vectors of length 1. Output z is of course scalar.
 */
void cartpole_QP_solver_LA_VSUBADD3_1(cartpole_QP_solver_FLOAT* t, cartpole_QP_solver_FLOAT* u, int* uidx, cartpole_QP_solver_FLOAT* v, cartpole_QP_solver_FLOAT* w, cartpole_QP_solver_FLOAT* y, cartpole_QP_solver_FLOAT* z, cartpole_QP_solver_FLOAT* r)
{
	int i;
	cartpole_QP_solver_FLOAT norm = *r;
	cartpole_QP_solver_FLOAT vx = 0;
	cartpole_QP_solver_FLOAT x;
	for( i=0; i<1; i++){
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
 * for vectors of length 1. Output z is of course scalar.
 */
void cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_FLOAT* t, int* tidx, cartpole_QP_solver_FLOAT* u, cartpole_QP_solver_FLOAT* v, cartpole_QP_solver_FLOAT* w, cartpole_QP_solver_FLOAT* y, cartpole_QP_solver_FLOAT* z, cartpole_QP_solver_FLOAT* r)
{
	int i;
	cartpole_QP_solver_FLOAT norm = *r;
	cartpole_QP_solver_FLOAT vx = 0;
	cartpole_QP_solver_FLOAT x;
	for( i=0; i<1; i++){
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
 * Computes r = A*x - b + s
 * and      y = max([norm(r,inf), y])
 * and      z -= l'*(Ax-b)
 * where A is stored in column major format
 */
void cartpole_QP_solver_LA_MVSUBADD_20_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *l, cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *z, cartpole_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	cartpole_QP_solver_FLOAT Ax[20];
	cartpole_QP_solver_FLOAT Axlessb;
	cartpole_QP_solver_FLOAT norm = *y;
	cartpole_QP_solver_FLOAT lAxlessb = 0;

	/* do A*x first */
	for( i=0; i<20; i++ ){
		Ax[i] = A[k++]*x[0];				
	}	
	for( j=1; j<14; j++ ){		
		for( i=0; i<20; i++ ){
			Ax[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<20; i++ ){
		Axlessb = Ax[i] - b[i];
		r[i] = Axlessb + s[i];
		lAxlessb += l[i]*Axlessb;
		if( r[i] > norm ){
			norm = r[i];
		}
		if( -r[i] > norm ){
			norm = -r[i];
		}
	}
	*y = norm;
	*z -= lAxlessb;
}


/*
 * Vector subtraction and addition.
 *	 Input: five vectors t, tidx, u, v, w and two scalars z and r
 *	 Output: y = t(tidx) - u + w
 *           z = z - v'*x;
 *           r = max([norm(y,inf), z]);
 * for vectors of length 4. Output z is of course scalar.
 */
void cartpole_QP_solver_LA_VSUBADD3_4(cartpole_QP_solver_FLOAT* t, cartpole_QP_solver_FLOAT* u, int* uidx, cartpole_QP_solver_FLOAT* v, cartpole_QP_solver_FLOAT* w, cartpole_QP_solver_FLOAT* y, cartpole_QP_solver_FLOAT* z, cartpole_QP_solver_FLOAT* r)
{
	int i;
	cartpole_QP_solver_FLOAT norm = *r;
	cartpole_QP_solver_FLOAT vx = 0;
	cartpole_QP_solver_FLOAT x;
	for( i=0; i<4; i++){
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
 * for vectors of length 4. Output z is of course scalar.
 */
void cartpole_QP_solver_LA_VSUBADD2_4(cartpole_QP_solver_FLOAT* t, int* tidx, cartpole_QP_solver_FLOAT* u, cartpole_QP_solver_FLOAT* v, cartpole_QP_solver_FLOAT* w, cartpole_QP_solver_FLOAT* y, cartpole_QP_solver_FLOAT* z, cartpole_QP_solver_FLOAT* r)
{
	int i;
	cartpole_QP_solver_FLOAT norm = *r;
	cartpole_QP_solver_FLOAT vx = 0;
	cartpole_QP_solver_FLOAT x;
	for( i=0; i<4; i++){
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
void cartpole_QP_solver_LA_INEQ_B_GRAD_15_6_6(cartpole_QP_solver_FLOAT *lu, cartpole_QP_solver_FLOAT *su, cartpole_QP_solver_FLOAT *ru, cartpole_QP_solver_FLOAT *ll, cartpole_QP_solver_FLOAT *sl, cartpole_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, cartpole_QP_solver_FLOAT *grad, cartpole_QP_solver_FLOAT *lubysu, cartpole_QP_solver_FLOAT *llbysl)
{
	int i;
	for( i=0; i<15; i++ ){
		grad[i] = 0;
	}
	for( i=0; i<6; i++ ){		
		llbysl[i] = ll[i] / sl[i];
		grad[lbIdx[i]] -= llbysl[i]*rl[i];
	}
	for( i=0; i<6; i++ ){
		lubysu[i] = lu[i] / su[i];
		grad[ubIdx[i]] += lubysu[i]*ru[i];
	}
}


/*
 * Special function for gradient of inequality constraints
 * Calculates grad += A'*(L/S)*rI
 */
void cartpole_QP_solver_LA_INEQ_P_20_15(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *lp, cartpole_QP_solver_FLOAT *sp, cartpole_QP_solver_FLOAT *rip, cartpole_QP_solver_FLOAT *grad, cartpole_QP_solver_FLOAT *lpbysp)
{
	int i;
	int j;
	int k = 0;

	cartpole_QP_solver_FLOAT lsr[20];
	
	/* do (L/S)*ri first */
	for( j=0; j<20; j++ ){
		lpbysp[j] = lp[j] / sp[j];
		lsr[j] = lpbysp[j]*rip[j];
	}

	for( i=0; i<15; i++ ){		
		for( j=0; j<20; j++ ){
			grad[i] += A[k++]*lsr[j];
		}
	}
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 15
 * Returns also L/S, a value that is often used elsewhere.
 */
void cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_FLOAT *lu, cartpole_QP_solver_FLOAT *su, cartpole_QP_solver_FLOAT *ru, cartpole_QP_solver_FLOAT *ll, cartpole_QP_solver_FLOAT *sl, cartpole_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, cartpole_QP_solver_FLOAT *grad, cartpole_QP_solver_FLOAT *lubysu, cartpole_QP_solver_FLOAT *llbysl)
{
	int i;
	for( i=0; i<15; i++ ){
		grad[i] = 0;
	}
	for( i=0; i<1; i++ ){		
		llbysl[i] = ll[i] / sl[i];
		grad[lbIdx[i]] -= llbysl[i]*rl[i];
	}
	for( i=0; i<1; i++ ){
		lubysu[i] = lu[i] / su[i];
		grad[ubIdx[i]] += lubysu[i]*ru[i];
	}
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 14
 * Returns also L/S, a value that is often used elsewhere.
 */
void cartpole_QP_solver_LA_INEQ_B_GRAD_14_1_1(cartpole_QP_solver_FLOAT *lu, cartpole_QP_solver_FLOAT *su, cartpole_QP_solver_FLOAT *ru, cartpole_QP_solver_FLOAT *ll, cartpole_QP_solver_FLOAT *sl, cartpole_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, cartpole_QP_solver_FLOAT *grad, cartpole_QP_solver_FLOAT *lubysu, cartpole_QP_solver_FLOAT *llbysl)
{
	int i;
	for( i=0; i<14; i++ ){
		grad[i] = 0;
	}
	for( i=0; i<1; i++ ){		
		llbysl[i] = ll[i] / sl[i];
		grad[lbIdx[i]] -= llbysl[i]*rl[i];
	}
	for( i=0; i<1; i++ ){
		lubysu[i] = lu[i] / su[i];
		grad[ubIdx[i]] += lubysu[i]*ru[i];
	}
}


/*
 * Special function for gradient of inequality constraints
 * Calculates grad += A'*(L/S)*rI
 */
void cartpole_QP_solver_LA_INEQ_P_20_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *lp, cartpole_QP_solver_FLOAT *sp, cartpole_QP_solver_FLOAT *rip, cartpole_QP_solver_FLOAT *grad, cartpole_QP_solver_FLOAT *lpbysp)
{
	int i;
	int j;
	int k = 0;

	cartpole_QP_solver_FLOAT lsr[20];
	
	/* do (L/S)*ri first */
	for( j=0; j<20; j++ ){
		lpbysp[j] = lp[j] / sp[j];
		lsr[j] = lpbysp[j]*rip[j];
	}

	for( i=0; i<14; i++ ){		
		for( j=0; j<20; j++ ){
			grad[i] += A[k++]*lsr[j];
		}
	}
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 4
 * Returns also L/S, a value that is often used elsewhere.
 */
void cartpole_QP_solver_LA_INEQ_B_GRAD_4_4_4(cartpole_QP_solver_FLOAT *lu, cartpole_QP_solver_FLOAT *su, cartpole_QP_solver_FLOAT *ru, cartpole_QP_solver_FLOAT *ll, cartpole_QP_solver_FLOAT *sl, cartpole_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, cartpole_QP_solver_FLOAT *grad, cartpole_QP_solver_FLOAT *lubysu, cartpole_QP_solver_FLOAT *llbysl)
{
	int i;
	for( i=0; i<4; i++ ){
		grad[i] = 0;
	}
	for( i=0; i<4; i++ ){		
		llbysl[i] = ll[i] / sl[i];
		grad[lbIdx[i]] -= llbysl[i]*rl[i];
	}
	for( i=0; i<4; i++ ){
		lubysu[i] = lu[i] / su[i];
		grad[ubIdx[i]] += lubysu[i]*ru[i];
	}
}


/*
 * Addition of three vectors  z = u + w + v
 * of length 288.
 */
void cartpole_QP_solver_LA_VVADD3_288(cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *v, cartpole_QP_solver_FLOAT *w, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<288; i++ ){
		z[i] = u[i] + v[i] + w[i];
	}
}


/*
 * Special function to compute the Dense positive definite 
 * augmented Hessian for block size 15.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_6_6(cartpole_QP_solver_FLOAT *H, cartpole_QP_solver_FLOAT *llbysl, int* lbIdx, cartpole_QP_solver_FLOAT *lubysu, int* ubIdx, cartpole_QP_solver_FLOAT *Phi)
{
	int i;
	int j;
	int k = 0;
	
	/* copy diagonal of H into PHI and set lower part of PHI = 0*/
	for( i=0; i<15; i++ ){
		for( j=0; j<i; j++ ){
			Phi[k++] = 0;
		}		
		/* we are on the diagonal */
		Phi[k++] = H[i];
	}

	/* add llbysl onto Phi where necessary */
	for( i=0; i<6; i++ ){
		j = lbIdx[i];
		Phi[((j+1)*(j+2))/2-1] += llbysl[i];
	}

	/* add lubysu onto Phi where necessary */
	for( i=0; i<6; i++){
		j = ubIdx[i];
		Phi[((j+1)*(j+2))/2-1] +=  lubysu[i];
	}

}


/**
 * Compute X = X + A'*D*A, where A is a general full matrix, D is
 * is a diagonal matrix stored in the vector d and X is a symmetric
 * positive definite matrix in lower triangular storage format. 
 * A is stored in column major format and is of size [20 x 15]
 * Phi is of size [15 x 15].
 */
void cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *d, cartpole_QP_solver_FLOAT *X)
{    
    int i,j,k,ii,di;
    cartpole_QP_solver_FLOAT x;
    
    di = 0; ii = 0;
    for( i=0; i<15; i++ ){        
        for( j=0; j<=i; j++ ){
            x = 0;
            for( k=0; k<20; k++ ){
                x += A[i*20+k]*A[j*20+k]*d[k];
            }
            X[ii+j] += x;
        }
        ii += ++di;
    }
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 15.
 */
void cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_FLOAT *A)
{
    int i, j, k, di, dj;
	 int ii, jj;
    cartpole_QP_solver_FLOAT l;
    cartpole_QP_solver_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<15; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += A[ii+k]*A[ii+k];
        }        
        
        Mii = A[ii+i] - l;
        
#if cartpole_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
        if( Mii < 1.0000000000000000E-013 ){
             PRINTTEXT("WARNING (CHOL2): small %d-th pivot in Cholesky fact. (=%3.1e < eps=%3.1e), regularizing to %3.1e\n",i,Mii,1.0000000000000000E-013,4.0000000000000002E-004);
			 A[ii+i] = 2.0000000000000000E-002;
		} else
		{
			A[ii+i] = sqrt(Mii);
		}
#else
		A[ii+i] = Mii < 1.0000000000000000E-013 ? 2.0000000000000000E-002 : sqrt(Mii);
#endif
                    
		jj = ((i+1)*(i+2))/2; dj = i+1;
        for( j=i+1; j<15; j++ ){
            l = 0;            
            for( k=0; k<i; k++ ){
                l += A[jj+k]*A[ii+k];
            }

			/* saturate values for numerical stability */
			l = MIN(l,  BIGMM);
			l = MAX(l, -BIGMM);

            A[jj+i] = (A[jj+i] - l)/A[ii+i];            
			jj += ++dj;
        }
		ii += ++di;
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [6 x 15],
 * B is given and of size [6 x 15], L is a lower tri-
 * angular matrix of size 15 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    cartpole_QP_solver_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<15; j++ ){        
        for( i=0; i<6; i++ ){
            a = B[j*6+i];
            for( k=0; k<j; k++ ){
                a -= A[k*6+i]*L[ii+k];
            }

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

            A[j*6+i] = a/L[ii+j];
        }
        ii += ++di;
    }
}


/**
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 15.
 */
void cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    cartpole_QP_solver_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<15; i++ ){
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


/*
 * Special function to compute the Dense positive definite 
 * augmented Hessian for block size 15.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_FLOAT *H, cartpole_QP_solver_FLOAT *llbysl, int* lbIdx, cartpole_QP_solver_FLOAT *lubysu, int* ubIdx, cartpole_QP_solver_FLOAT *Phi)
{
	int i;
	int j;
	int k = 0;
	
	/* copy diagonal of H into PHI and set lower part of PHI = 0*/
	for( i=0; i<15; i++ ){
		for( j=0; j<i; j++ ){
			Phi[k++] = 0;
		}		
		/* we are on the diagonal */
		Phi[k++] = H[i];
	}

	/* add llbysl onto Phi where necessary */
	for( i=0; i<1; i++ ){
		j = lbIdx[i];
		Phi[((j+1)*(j+2))/2-1] += llbysl[i];
	}

	/* add lubysu onto Phi where necessary */
	for( i=0; i<1; i++){
		j = ubIdx[i];
		Phi[((j+1)*(j+2))/2-1] +=  lubysu[i];
	}

}


/**
 * Compute C = A*B' where 
 *
 *	size(A) = [6 x 15]
 *  size(B) = [6 x 15]
 * 
 * and all matrices are stored in column major format.
 *
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE.  
 * 
 */
void cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *C)
{
    int i, j, k;
    cartpole_QP_solver_FLOAT temp;
    
    for( i=0; i<6; i++ ){        
        for( j=0; j<6; j++ ){
            temp = 0; 
            for( k=0; k<15; k++ ){
                temp += A[k*6+i]*B[k*6+j];
            }						
            C[j*6+i] = temp;
        }
    }
}


/*
 * Special function to compute the Dense positive definite 
 * augmented Hessian for block size 14.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_14_1_1(cartpole_QP_solver_FLOAT *H, cartpole_QP_solver_FLOAT *llbysl, int* lbIdx, cartpole_QP_solver_FLOAT *lubysu, int* ubIdx, cartpole_QP_solver_FLOAT *Phi)
{
	int i;
	int j;
	int k = 0;
	
	/* copy diagonal of H into PHI and set lower part of PHI = 0*/
	for( i=0; i<14; i++ ){
		for( j=0; j<i; j++ ){
			Phi[k++] = 0;
		}		
		/* we are on the diagonal */
		Phi[k++] = H[i];
	}

	/* add llbysl onto Phi where necessary */
	for( i=0; i<1; i++ ){
		j = lbIdx[i];
		Phi[((j+1)*(j+2))/2-1] += llbysl[i];
	}

	/* add lubysu onto Phi where necessary */
	for( i=0; i<1; i++){
		j = ubIdx[i];
		Phi[((j+1)*(j+2))/2-1] +=  lubysu[i];
	}

}


/**
 * Compute X = X + A'*D*A, where A is a general full matrix, D is
 * is a diagonal matrix stored in the vector d and X is a symmetric
 * positive definite matrix in lower triangular storage format. 
 * A is stored in column major format and is of size [20 x 14]
 * Phi is of size [14 x 14].
 */
void cartpole_QP_solver_LA_DENSE_ADDMTDM_20_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *d, cartpole_QP_solver_FLOAT *X)
{    
    int i,j,k,ii,di;
    cartpole_QP_solver_FLOAT x;
    
    di = 0; ii = 0;
    for( i=0; i<14; i++ ){        
        for( j=0; j<=i; j++ ){
            x = 0;
            for( k=0; k<20; k++ ){
                x += A[i*20+k]*A[j*20+k]*d[k];
            }
            X[ii+j] += x;
        }
        ii += ++di;
    }
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 14.
 */
void cartpole_QP_solver_LA_DENSE_CHOL2_14(cartpole_QP_solver_FLOAT *A)
{
    int i, j, k, di, dj;
	 int ii, jj;
    cartpole_QP_solver_FLOAT l;
    cartpole_QP_solver_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<14; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += A[ii+k]*A[ii+k];
        }        
        
        Mii = A[ii+i] - l;
        
#if cartpole_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
        if( Mii < 1.0000000000000000E-013 ){
             PRINTTEXT("WARNING (CHOL2): small %d-th pivot in Cholesky fact. (=%3.1e < eps=%3.1e), regularizing to %3.1e\n",i,Mii,1.0000000000000000E-013,4.0000000000000002E-004);
			 A[ii+i] = 2.0000000000000000E-002;
		} else
		{
			A[ii+i] = sqrt(Mii);
		}
#else
		A[ii+i] = Mii < 1.0000000000000000E-013 ? 2.0000000000000000E-002 : sqrt(Mii);
#endif
                    
		jj = ((i+1)*(i+2))/2; dj = i+1;
        for( j=i+1; j<14; j++ ){
            l = 0;            
            for( k=0; k<i; k++ ){
                l += A[jj+k]*A[ii+k];
            }

			/* saturate values for numerical stability */
			l = MIN(l,  BIGMM);
			l = MAX(l, -BIGMM);

            A[jj+i] = (A[jj+i] - l)/A[ii+i];            
			jj += ++dj;
        }
		ii += ++di;
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [4 x 14],
 * B is given and of size [4 x 14], L is a lower tri-
 * angular matrix of size 14 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_4_14(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    cartpole_QP_solver_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<14; j++ ){        
        for( i=0; i<4; i++ ){
            a = B[j*4+i];
            for( k=0; k<j; k++ ){
                a -= A[k*4+i]*L[ii+k];
            }

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

            A[j*4+i] = a/L[ii+j];
        }
        ii += ++di;
    }
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [6 x 14],
 * B is given and of size [6 x 14], L is a lower tri-
 * angular matrix of size 14 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_14(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    cartpole_QP_solver_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<14; j++ ){        
        for( i=0; i<6; i++ ){
            a = B[j*6+i];
            for( k=0; k<j; k++ ){
                a -= A[k*6+i]*L[ii+k];
            }

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

            A[j*6+i] = a/L[ii+j];
        }
        ii += ++di;
    }
}


/**
 * Compute C = A*B' where 
 *
 *	size(A) = [6 x 14]
 *  size(B) = [4 x 14]
 * 
 * and all matrices are stored in column major format.
 *
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE.  
 * 
 */
void cartpole_QP_solver_LA_DENSE_MMTM_6_14_4(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *C)
{
    int i, j, k;
    cartpole_QP_solver_FLOAT temp;
    
    for( i=0; i<6; i++ ){        
        for( j=0; j<4; j++ ){
            temp = 0; 
            for( k=0; k<14; k++ ){
                temp += A[k*6+i]*B[k*4+j];
            }						
            C[j*6+i] = temp;
        }
    }
}


/**
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 14.
 */
void cartpole_QP_solver_LA_DENSE_FORWARDSUB_14(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    cartpole_QP_solver_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<14; i++ ){
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


/*
 * Special function to compute the diagonal cholesky factorization of the 
 * positive definite augmented Hessian for block size 4.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = sqrt(H + diag(llbysl) + diag(lubysu))
 * where Phi is stored in diagonal storage format
 */
void cartpole_QP_solver_LA_DIAG_CHOL_ONELOOP_LBUB_4_4_4(cartpole_QP_solver_FLOAT *H, cartpole_QP_solver_FLOAT *llbysl, int* lbIdx, cartpole_QP_solver_FLOAT *lubysu, int* ubIdx, cartpole_QP_solver_FLOAT *Phi)


{
	int i;
	
	/* compute cholesky */
	for( i=0; i<4; i++ ){
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
 * where A is to be computed and is of size [4 x 4],
 * B is given and of size [4 x 4], L is a diagonal
 *  matrix of size 4 stored in diagonal 
 * storage format. Note the transpose of L!
 *
 * Result: A in diagonalzero storage format.
 *
 */
void cartpole_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_4_4(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *A)
{
	int j;
    for( j=0; j<4; j++ ){   
		A[j] = B[j]/L[j];
     }
}


/**
 * Forward substitution to solve L*y = b where L is a
 * diagonal matrix in vector storage format.
 * 
 * The dimensions involved are 4.
 */
void cartpole_QP_solver_LA_DIAG_FORWARDSUB_4(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *y)
{
    int i;

    for( i=0; i<4; i++ ){
		y[i] = b[i]/L[i];
    }
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [6 x 15] in column
 * storage format, and B is of size [6 x 15] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    cartpole_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<6; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<15; k++ ){
                ltemp += A[k*6+i]*A[k*6+j];
            }			
			for( k=0; k<15; k++ ){
                ltemp += B[k*6+i]*B[k*6+j];
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
void cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<6; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<15; j++ ){		
		for( i=0; i<6; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<15; n++ ){
		for( i=0; i<6; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [6 x 15] in column
 * storage format, and B is of size [6 x 14] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void cartpole_QP_solver_LA_DENSE_MMT2_6_15_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    cartpole_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<6; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<15; k++ ){
                ltemp += A[k*6+i]*A[k*6+j];
            }			
			for( k=0; k<14; k++ ){
                ltemp += B[k*6+i]*B[k*6+j];
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
void cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<6; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<15; j++ ){		
		for( i=0; i<6; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<14; n++ ){
		for( i=0; i<6; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [4 x 14] in column
 * storage format, and B is of size [4 x 4] diagonalzero
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void cartpole_QP_solver_LA_DENSE_DIAGZERO_MMT2_4_14_4(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    cartpole_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<4; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<14; k++ ){
                ltemp += A[k*4+i]*A[k*4+j];
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
void cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_4_14_4(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<4; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[i]*u[i];
	}	

	for( j=1; j<14; j++ ){		
		for( i=0; i<4; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 6 and outputting
 * the Cholesky factor to matrix L in lower triangular format.
 */
void cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    cartpole_QP_solver_FLOAT l;
    cartpole_QP_solver_FLOAT Mii;

	/* copy A to L first and then operate on L */
	/* COULD BE OPTIMIZED */
	ii=0; di=0;
	for( i=0; i<6; i++ ){
		for( j=0; j<=i; j++ ){
			L[ii+j] = A[ii+j];
		}
		ii += ++di;
	}    
	
	/* factor L */
	ii=0; di=0;
    for( i=0; i<6; i++ ){
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
        for( j=i+1; j<6; j++ ){
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
 * The dimensions involved are 6.
 */
void cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    cartpole_QP_solver_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<6; i++ ){
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
 * where A is to be computed and is of size [6 x 6],
 * B is given and of size [6 x 6], L is a lower tri-
 * angular matrix of size 6 stored in lower triangular 
 * storage format. Note the transpose of L AND B!
 *
 * Result: A in column major storage format.
 *
 */
void cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *A)
{
    int i,j,k,ii,di;
    cartpole_QP_solver_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<6; j++ ){        
        for( i=0; i<6; i++ ){
            a = B[i*6+j];
            for( k=0; k<j; k++ ){
                a -= A[k*6+i]*L[ii+k];
            }    

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

			A[j*6+i] = a/L[ii+j];			
        }
        ii += ++di;
    }
}


/**
 * Compute L = L - A*A', where L is lower triangular of size 6
 * and A is a dense matrix of size [6 x 6] in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    cartpole_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<6; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<6; k++ ){
                ltemp += A[k*6+i]*A[k*6+j];
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
void cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<6; i++ ){
		r[i] = b[i] - A[k++]*x[0];
	}	
	for( j=1; j<6; j++ ){		
		for( i=0; i<6; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/** 
 * Forward substitution for the matrix equation A*L' = B'
 * where A is to be computed and is of size [4 x 6],
 * B is given and of size [4 x 6], L is a lower tri-
 * angular matrix of size 6 stored in lower triangular 
 * storage format. Note the transpose of L AND B!
 *
 * Result: A in column major storage format.
 *
 */
void cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_4_6(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *A)
{
    int i,j,k,ii,di;
    cartpole_QP_solver_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<6; j++ ){        
        for( i=0; i<4; i++ ){
            a = B[i*6+j];
            for( k=0; k<j; k++ ){
                a -= A[k*4+i]*L[ii+k];
            }    

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

			A[j*4+i] = a/L[ii+j];			
        }
        ii += ++di;
    }
}


/**
 * Compute L = L - A*A', where L is lower triangular of size 4
 * and A is a dense matrix of size [4 x 6] in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void cartpole_QP_solver_LA_DENSE_MMTSUB_4_6(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    cartpole_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<4; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<6; k++ ){
                ltemp += A[k*4+i]*A[k*4+j];
            }						
            L[ii+j] -= ltemp;
        }
        ii += ++di;
    }
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 4 and outputting
 * the Cholesky factor to matrix L in lower triangular format.
 */
void cartpole_QP_solver_LA_DENSE_CHOL_4(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    cartpole_QP_solver_FLOAT l;
    cartpole_QP_solver_FLOAT Mii;

	/* copy A to L first and then operate on L */
	/* COULD BE OPTIMIZED */
	ii=0; di=0;
	for( i=0; i<4; i++ ){
		for( j=0; j<=i; j++ ){
			L[ii+j] = A[ii+j];
		}
		ii += ++di;
	}    
	
	/* factor L */
	ii=0; di=0;
    for( i=0; i<4; i++ ){
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
        for( j=i+1; j<4; j++ ){
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
void cartpole_QP_solver_LA_DENSE_MVMSUB1_4_6(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<4; i++ ){
		r[i] = b[i] - A[k++]*x[0];
	}	
	for( j=1; j<6; j++ ){		
		for( i=0; i<4; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/**
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 4.
 */
void cartpole_QP_solver_LA_DENSE_FORWARDSUB_4(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    cartpole_QP_solver_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<4; i++ ){
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
 * Backward Substitution to solve L^T*x = y where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * All involved dimensions are 4.
 */
void cartpole_QP_solver_LA_DENSE_BACKWARDSUB_4(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    cartpole_QP_solver_FLOAT xel;    
	int start = 6;
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 3;
    for( i=3; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 3;
        for( j=3; j>i; j-- ){
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
 * Matrix vector multiplication y = b - M'*x where M is of size [4 x 6]
 * and stored in column major format. Note the transpose of M!
 */
void cartpole_QP_solver_LA_DENSE_MTVMSUB_4_6(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<6; i++ ){
		r[i] = b[i];
		for( j=0; j<4; j++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/**
 * Backward Substitution to solve L^T*x = y where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * All involved dimensions are 6.
 */
void cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    cartpole_QP_solver_FLOAT xel;    
	int start = 15;
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 5;
    for( i=5; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 5;
        for( j=5; j>i; j-- ){
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
 * Matrix vector multiplication y = b - M'*x where M is of size [6 x 6]
 * and stored in column major format. Note the transpose of M!
 */
void cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<6; i++ ){
		r[i] = b[i];
		for( j=0; j<6; j++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Vector subtraction z = -x - y for vectors of length 288.
 */
void cartpole_QP_solver_LA_VSUB2_288(cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<288; i++){
		z[i] = -x[i] - y[i];
	}
}


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * lower triangular matrix of size 15 in lower triangular
 * storage format.
 */
void cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    cartpole_QP_solver_FLOAT y[15];
    cartpole_QP_solver_FLOAT yel,xel;
	int start = 105;
            
    /* first solve Ly = b by forward substitution */
     ii = 0; di = 0;
    for( i=0; i<15; i++ ){
        yel = b[i];        
        for( j=0; j<i; j++ ){
            yel -= y[j]*L[ii+j];
        }

		/* saturate for numerical stability */
		yel = MIN(yel, BIGM);
		yel = MAX(yel, -BIGM); 

        y[i] = yel / L[ii+i];
        ii += ++di;
    }
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 14;
    for( i=14; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 14;
        for( j=14; j>i; j-- ){
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


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * lower triangular matrix of size 14 in lower triangular
 * storage format.
 */
void cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_14(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    cartpole_QP_solver_FLOAT y[14];
    cartpole_QP_solver_FLOAT yel,xel;
	int start = 91;
            
    /* first solve Ly = b by forward substitution */
     ii = 0; di = 0;
    for( i=0; i<14; i++ ){
        yel = b[i];        
        for( j=0; j<i; j++ ){
            yel -= y[j]*L[ii+j];
        }

		/* saturate for numerical stability */
		yel = MIN(yel, BIGM);
		yel = MAX(yel, -BIGM); 

        y[i] = yel / L[ii+i];
        ii += ++di;
    }
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 13;
    for( i=13; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 13;
        for( j=13; j>i; j-- ){
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


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * diagonal matrix of size 4 in vector
 * storage format.
 */
void cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_4(cartpole_QP_solver_FLOAT *L, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *x)
{
    int i;
            
    /* solve Ly = b by forward and backward substitution */
    for( i=0; i<4; i++ ){
		x[i] = b[i]/(L[i]*L[i]);
    }
    
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 6,
 * and x has length 15 and is indexed through yidx.
 */
void cartpole_QP_solver_LA_VSUB_INDEXED_6(cartpole_QP_solver_FLOAT *x, int* xidx, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<6; i++){
		z[i] = x[xidx[i]] - y[i];
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
 * Vector subtraction z = -x - y(yidx) where y is of length 15
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
 * Computes r = -b - A*x
 * where A is stored in column major format
 */
void cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<20; i++ ){
		r[i] = -b[i] - A[k++]*x[0];
	}	
	for( j=1; j<15; j++ ){		
		for( i=0; i<20; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 20.
 */
void cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *v, cartpole_QP_solver_FLOAT *w, cartpole_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<20; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 1,
 * and x has length 15 and is indexed through yidx.
 */
void cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_FLOAT *x, int* xidx, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<1; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 1.
 */
void cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *v, cartpole_QP_solver_FLOAT *w, cartpole_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<1; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = -x - y(yidx) where y is of length 15
 * and z, x and yidx are of length 1.
 */
void cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y, int* yidx, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<1; i++){
		z[i] = -x[i] - y[yidx[i]];
	}
}


/* 
 * Computes r = -b - A*x
 * where A is stored in column major format
 */
void cartpole_QP_solver_LA_DENSE_MVMSUB4_20_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<20; i++ ){
		r[i] = -b[i] - A[k++]*x[0];
	}	
	for( j=1; j<14; j++ ){		
		for( i=0; i<20; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 4,
 * and x has length 4 and is indexed through yidx.
 */
void cartpole_QP_solver_LA_VSUB_INDEXED_4(cartpole_QP_solver_FLOAT *x, int* xidx, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<4; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 4.
 */
void cartpole_QP_solver_LA_VSUB3_4(cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *v, cartpole_QP_solver_FLOAT *w, cartpole_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<4; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = -x - y(yidx) where y is of length 4
 * and z, x and yidx are of length 4.
 */
void cartpole_QP_solver_LA_VSUB2_INDEXED_4(cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y, int* yidx, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<4; i++){
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
        for( i=0; i<436; i++ ){
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
        if( i == 436 ){
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
    *mu_aff = mymu / (cartpole_QP_solver_FLOAT)436;
    return lsIt;
}


/*
 * Vector subtraction x = (u.*v - mu)*sigma where a is a scalar
*  and x,u,v are vectors of length 436.
 */
void cartpole_QP_solver_LA_VSUB5_436(cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *v, cartpole_QP_solver_FLOAT mu,  cartpole_QP_solver_FLOAT sigma, cartpole_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<436; i++){
		x[i] = u[i]*v[i] - mu;
		x[i] *= sigma;
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 15,
 * u, su, uidx are of length 6 and v, sv, vidx are of length 6.
 */
void cartpole_QP_solver_LA_VSUB6_INDEXED_15_6_6(cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *su, int* uidx, cartpole_QP_solver_FLOAT *v, cartpole_QP_solver_FLOAT *sv, int* vidx, cartpole_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<15; i++ ){
		x[i] = 0;
	}
	for( i=0; i<6; i++){
		x[uidx[i]] += u[i]/su[i];
	}
	for( i=0; i<6; i++){
		x[vidx[i]] -= v[i]/sv[i];
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 15,
 * u, su, uidx are of length 1 and v, sv, vidx are of length 1.
 */
void cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *su, int* uidx, cartpole_QP_solver_FLOAT *v, cartpole_QP_solver_FLOAT *sv, int* vidx, cartpole_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<15; i++ ){
		x[i] = 0;
	}
	for( i=0; i<1; i++){
		x[uidx[i]] += u[i]/su[i];
	}
	for( i=0; i<1; i++){
		x[vidx[i]] -= v[i]/sv[i];
	}
}


/*
 * Matrix vector multiplication z = z + A'*(x./s) where A is of size [20 x 15]
 * and stored in column major format. Note the transpose of M!
 */
void cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0; 
	cartpole_QP_solver_FLOAT temp[20];

	for( j=0; j<20; j++ ){
		temp[j] = x[j] / s[j];
	}

	for( i=0; i<15; i++ ){
		for( j=0; j<20; j++ ){
			z[i] += A[k++]*temp[j];
		}
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<6; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<15; j++ ){		
		for( i=0; i<6; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<15; n++ ){
		for( i=0; i<6; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 14,
 * u, su, uidx are of length 1 and v, sv, vidx are of length 1.
 */
void cartpole_QP_solver_LA_VSUB6_INDEXED_14_1_1(cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *su, int* uidx, cartpole_QP_solver_FLOAT *v, cartpole_QP_solver_FLOAT *sv, int* vidx, cartpole_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<14; i++ ){
		x[i] = 0;
	}
	for( i=0; i<1; i++){
		x[uidx[i]] += u[i]/su[i];
	}
	for( i=0; i<1; i++){
		x[vidx[i]] -= v[i]/sv[i];
	}
}


/*
 * Matrix vector multiplication z = z + A'*(x./s) where A is of size [20 x 14]
 * and stored in column major format. Note the transpose of M!
 */
void cartpole_QP_solver_LA_DENSE_MTVMADD2_20_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0; 
	cartpole_QP_solver_FLOAT temp[20];

	for( j=0; j<20; j++ ){
		temp[j] = x[j] / s[j];
	}

	for( i=0; i<14; i++ ){
		for( j=0; j<20; j++ ){
			z[i] += A[k++]*temp[j];
		}
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<6; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<15; j++ ){		
		for( i=0; i<6; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<14; n++ ){
		for( i=0; i<6; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 4,
 * u, su, uidx are of length 4 and v, sv, vidx are of length 4.
 */
void cartpole_QP_solver_LA_VSUB6_INDEXED_4_4_4(cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *su, int* uidx, cartpole_QP_solver_FLOAT *v, cartpole_QP_solver_FLOAT *sv, int* vidx, cartpole_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<4; i++ ){
		x[i] = 0;
	}
	for( i=0; i<4; i++){
		x[uidx[i]] += u[i]/su[i];
	}
	for( i=0; i<4; i++){
		x[vidx[i]] -= v[i]/sv[i];
	}
}


/* 
 * Computes r = A*x + B*u
 * where A is stored in column major format
 * and B is stored in diagzero format
 */
void cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_4_14_4(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *B, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<4; i++ ){
		r[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<14; j++ ){		
		for( i=0; i<4; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
}


/*
 * Vector subtraction z = x - y for vectors of length 288.
 */
void cartpole_QP_solver_LA_VSUB_288(cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<288; i++){
		z[i] = x[i] - y[i];
	}
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 6 (length of y >= 6).
 */
void cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *y, int* yidx, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<6; i++ ){
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


/* 
 * Computes r = (-b + l.*(A*x))./s
 * where A is stored in column major format
 */
void cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *l, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	cartpole_QP_solver_FLOAT temp[20];

	
	for( i=0; i<20; i++ ){
		temp[i] = A[k++]*x[0];
	}
	

	for( j=1; j<15; j++ ){		
		for( i=0; i<20; i++ ){
			temp[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<20; i++ ){
		r[i] = (-b[i] + l[i]*temp[i])/s[i]; 
	}	
	
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 1 (length of y >= 1).
 */
void cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *y, int* yidx, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<1; i++ ){
		z[i] = -r[i]/s[i] - u[i]*y[yidx[i]];
	}
}


/** 
 * Computes z = -r./s + u.*y(y)
 * where all vectors except of y are of length 1 (length of y >= 1).
 */
void cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *y, int* yidx, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<1; i++ ){
		z[i] = -r[i]/s[i] + u[i]*y[yidx[i]];
	}
}


/* 
 * Computes r = (-b + l.*(A*x))./s
 * where A is stored in column major format
 */
void cartpole_QP_solver_LA_DENSE_MVMSUB5_20_14(cartpole_QP_solver_FLOAT *A, cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *b, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *l, cartpole_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	cartpole_QP_solver_FLOAT temp[20];

	
	for( i=0; i<20; i++ ){
		temp[i] = A[k++]*x[0];
	}
	

	for( j=1; j<14; j++ ){		
		for( i=0; i<20; i++ ){
			temp[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<20; i++ ){
		r[i] = (-b[i] + l[i]*temp[i])/s[i]; 
	}	
	
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 4 (length of y >= 4).
 */
void cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *y, int* yidx, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<4; i++ ){
		z[i] = -r[i]/s[i] - u[i]*y[yidx[i]];
	}
}


/** 
 * Computes z = -r./s + u.*y(y)
 * where all vectors except of y are of length 4 (length of y >= 4).
 */
void cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *u, cartpole_QP_solver_FLOAT *y, int* yidx, cartpole_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<4; i++ ){
		z[i] = -r[i]/s[i] + u[i]*y[yidx[i]];
	}
}


/*
 * Computes ds = -l.\(r + s.*dl) for vectors of length 436.
 */
void cartpole_QP_solver_LA_VSUB7_436(cartpole_QP_solver_FLOAT *l, cartpole_QP_solver_FLOAT *r, cartpole_QP_solver_FLOAT *s, cartpole_QP_solver_FLOAT *dl, cartpole_QP_solver_FLOAT *ds)
{
	int i;
	for( i=0; i<436; i++){
		ds[i] = -(r[i] + s[i]*dl[i])/l[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 288.
 */
void cartpole_QP_solver_LA_VADD_288(cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<288; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 112.
 */
void cartpole_QP_solver_LA_VADD_112(cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<112; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 436.
 */
void cartpole_QP_solver_LA_VADD_436(cartpole_QP_solver_FLOAT *x, cartpole_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<436; i++){
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
        for( i=0; i<436; i++ ){
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
        if( i == 436 ){
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
    for( i=0; i<288; i++ ){
        z[i] += a_gamma*dz[i];
    }
    
    /* equality constraint multipliers */
    for( i=0; i<112; i++ ){
        v[i] += a_gamma*dv[i];
    }
    
    /* inequality constraint multipliers & slacks, also update mu */
    *mu = 0;
    for( i=0; i<436; i++ ){
        dltemp = l[i] + a_gamma*dl[i]; l[i] = dltemp;
        dstemp = s[i] + a_gamma*ds[i]; s[i] = dstemp;
        *mu += dltemp*dstemp;
    }
    
    *a = a_gamma;
    *mu /= (cartpole_QP_solver_FLOAT)436;
    return lsIt;
}




/* VARIABLE DEFINITIONS ------------------------------------------------ */
cartpole_QP_solver_FLOAT cartpole_QP_solver_z[288];
cartpole_QP_solver_FLOAT cartpole_QP_solver_v[112];
cartpole_QP_solver_FLOAT cartpole_QP_solver_dz_aff[288];
cartpole_QP_solver_FLOAT cartpole_QP_solver_dv_aff[112];
cartpole_QP_solver_FLOAT cartpole_QP_solver_grad_cost[288];
cartpole_QP_solver_FLOAT cartpole_QP_solver_grad_eq[288];
cartpole_QP_solver_FLOAT cartpole_QP_solver_rd[288];
cartpole_QP_solver_FLOAT cartpole_QP_solver_l[436];
cartpole_QP_solver_FLOAT cartpole_QP_solver_s[436];
cartpole_QP_solver_FLOAT cartpole_QP_solver_lbys[436];
cartpole_QP_solver_FLOAT cartpole_QP_solver_dl_aff[436];
cartpole_QP_solver_FLOAT cartpole_QP_solver_ds_aff[436];
cartpole_QP_solver_FLOAT cartpole_QP_solver_dz_cc[288];
cartpole_QP_solver_FLOAT cartpole_QP_solver_dv_cc[112];
cartpole_QP_solver_FLOAT cartpole_QP_solver_dl_cc[436];
cartpole_QP_solver_FLOAT cartpole_QP_solver_ds_cc[436];
cartpole_QP_solver_FLOAT cartpole_QP_solver_ccrhs[436];
cartpole_QP_solver_FLOAT cartpole_QP_solver_grad_ineq[288];
cartpole_QP_solver_FLOAT cartpole_QP_solver_H00[15] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z00 = cartpole_QP_solver_z + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff00 = cartpole_QP_solver_dz_aff + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc00 = cartpole_QP_solver_dz_cc + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd00 = cartpole_QP_solver_rd + 0;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd00[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost00 = cartpole_QP_solver_grad_cost + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq00 = cartpole_QP_solver_grad_eq + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq00 = cartpole_QP_solver_grad_ineq + 0;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv00[15];
cartpole_QP_solver_FLOAT cartpole_QP_solver_C00[90] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v00 = cartpole_QP_solver_v + 0;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re00[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta00[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc00[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff00 = cartpole_QP_solver_dv_aff + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc00 = cartpole_QP_solver_dv_cc + 0;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V00[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd00[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld00[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy00[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy00[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c00[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx00[6] = {0, 1, 2, 3, 4, 10};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb00 = cartpole_QP_solver_l + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb00 = cartpole_QP_solver_s + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb00 = cartpole_QP_solver_lbys + 0;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb00[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff00 = cartpole_QP_solver_dl_aff + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff00 = cartpole_QP_solver_ds_aff + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc00 = cartpole_QP_solver_dl_cc + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc00 = cartpole_QP_solver_ds_cc + 0;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl00 = cartpole_QP_solver_ccrhs + 0;
int cartpole_QP_solver_ubIdx00[6] = {0, 1, 2, 3, 4, 10};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub00 = cartpole_QP_solver_l + 6;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub00 = cartpole_QP_solver_s + 6;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub00 = cartpole_QP_solver_lbys + 6;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub00[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff00 = cartpole_QP_solver_dl_aff + 6;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff00 = cartpole_QP_solver_ds_aff + 6;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc00 = cartpole_QP_solver_dl_cc + 6;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc00 = cartpole_QP_solver_ds_cc + 6;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub00 = cartpole_QP_solver_ccrhs + 6;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp00 = cartpole_QP_solver_s + 12;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp00 = cartpole_QP_solver_l + 12;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp00 = cartpole_QP_solver_lbys + 12;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff00 = cartpole_QP_solver_dl_aff + 12;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff00 = cartpole_QP_solver_ds_aff + 12;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc00 = cartpole_QP_solver_dl_cc + 12;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc00 = cartpole_QP_solver_ds_cc + 12;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp00 = cartpole_QP_solver_ccrhs + 12;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip00[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi00[120];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z01 = cartpole_QP_solver_z + 15;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff01 = cartpole_QP_solver_dz_aff + 15;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc01 = cartpole_QP_solver_dz_cc + 15;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd01 = cartpole_QP_solver_rd + 15;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd01[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost01 = cartpole_QP_solver_grad_cost + 15;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq01 = cartpole_QP_solver_grad_eq + 15;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq01 = cartpole_QP_solver_grad_ineq + 15;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv01[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v01 = cartpole_QP_solver_v + 6;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re01[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta01[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc01[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff01 = cartpole_QP_solver_dv_aff + 6;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc01 = cartpole_QP_solver_dv_cc + 6;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V01[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd01[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld01[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy01[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy01[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c01[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx01[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb01 = cartpole_QP_solver_l + 32;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb01 = cartpole_QP_solver_s + 32;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb01 = cartpole_QP_solver_lbys + 32;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb01[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff01 = cartpole_QP_solver_dl_aff + 32;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff01 = cartpole_QP_solver_ds_aff + 32;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc01 = cartpole_QP_solver_dl_cc + 32;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc01 = cartpole_QP_solver_ds_cc + 32;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl01 = cartpole_QP_solver_ccrhs + 32;
int cartpole_QP_solver_ubIdx01[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub01 = cartpole_QP_solver_l + 33;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub01 = cartpole_QP_solver_s + 33;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub01 = cartpole_QP_solver_lbys + 33;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub01[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff01 = cartpole_QP_solver_dl_aff + 33;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff01 = cartpole_QP_solver_ds_aff + 33;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc01 = cartpole_QP_solver_dl_cc + 33;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc01 = cartpole_QP_solver_ds_cc + 33;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub01 = cartpole_QP_solver_ccrhs + 33;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp01 = cartpole_QP_solver_s + 34;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp01 = cartpole_QP_solver_l + 34;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp01 = cartpole_QP_solver_lbys + 34;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff01 = cartpole_QP_solver_dl_aff + 34;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff01 = cartpole_QP_solver_ds_aff + 34;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc01 = cartpole_QP_solver_dl_cc + 34;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc01 = cartpole_QP_solver_ds_cc + 34;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp01 = cartpole_QP_solver_ccrhs + 34;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip01[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi01[120];
cartpole_QP_solver_FLOAT cartpole_QP_solver_D01[90] = {-1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
cartpole_QP_solver_FLOAT cartpole_QP_solver_W01[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd01[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd01[36];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z02 = cartpole_QP_solver_z + 30;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff02 = cartpole_QP_solver_dz_aff + 30;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc02 = cartpole_QP_solver_dz_cc + 30;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd02 = cartpole_QP_solver_rd + 30;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd02[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost02 = cartpole_QP_solver_grad_cost + 30;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq02 = cartpole_QP_solver_grad_eq + 30;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq02 = cartpole_QP_solver_grad_ineq + 30;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv02[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v02 = cartpole_QP_solver_v + 12;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re02[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta02[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc02[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff02 = cartpole_QP_solver_dv_aff + 12;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc02 = cartpole_QP_solver_dv_cc + 12;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V02[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd02[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld02[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy02[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy02[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c02[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx02[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb02 = cartpole_QP_solver_l + 54;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb02 = cartpole_QP_solver_s + 54;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb02 = cartpole_QP_solver_lbys + 54;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb02[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff02 = cartpole_QP_solver_dl_aff + 54;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff02 = cartpole_QP_solver_ds_aff + 54;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc02 = cartpole_QP_solver_dl_cc + 54;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc02 = cartpole_QP_solver_ds_cc + 54;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl02 = cartpole_QP_solver_ccrhs + 54;
int cartpole_QP_solver_ubIdx02[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub02 = cartpole_QP_solver_l + 55;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub02 = cartpole_QP_solver_s + 55;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub02 = cartpole_QP_solver_lbys + 55;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub02[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff02 = cartpole_QP_solver_dl_aff + 55;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff02 = cartpole_QP_solver_ds_aff + 55;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc02 = cartpole_QP_solver_dl_cc + 55;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc02 = cartpole_QP_solver_ds_cc + 55;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub02 = cartpole_QP_solver_ccrhs + 55;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp02 = cartpole_QP_solver_s + 56;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp02 = cartpole_QP_solver_l + 56;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp02 = cartpole_QP_solver_lbys + 56;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff02 = cartpole_QP_solver_dl_aff + 56;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff02 = cartpole_QP_solver_ds_aff + 56;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc02 = cartpole_QP_solver_dl_cc + 56;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc02 = cartpole_QP_solver_ds_cc + 56;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp02 = cartpole_QP_solver_ccrhs + 56;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip02[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi02[120];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W02[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd02[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd02[36];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z03 = cartpole_QP_solver_z + 45;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff03 = cartpole_QP_solver_dz_aff + 45;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc03 = cartpole_QP_solver_dz_cc + 45;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd03 = cartpole_QP_solver_rd + 45;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd03[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost03 = cartpole_QP_solver_grad_cost + 45;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq03 = cartpole_QP_solver_grad_eq + 45;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq03 = cartpole_QP_solver_grad_ineq + 45;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv03[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v03 = cartpole_QP_solver_v + 18;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re03[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta03[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc03[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff03 = cartpole_QP_solver_dv_aff + 18;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc03 = cartpole_QP_solver_dv_cc + 18;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V03[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd03[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld03[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy03[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy03[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c03[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx03[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb03 = cartpole_QP_solver_l + 76;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb03 = cartpole_QP_solver_s + 76;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb03 = cartpole_QP_solver_lbys + 76;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb03[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff03 = cartpole_QP_solver_dl_aff + 76;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff03 = cartpole_QP_solver_ds_aff + 76;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc03 = cartpole_QP_solver_dl_cc + 76;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc03 = cartpole_QP_solver_ds_cc + 76;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl03 = cartpole_QP_solver_ccrhs + 76;
int cartpole_QP_solver_ubIdx03[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub03 = cartpole_QP_solver_l + 77;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub03 = cartpole_QP_solver_s + 77;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub03 = cartpole_QP_solver_lbys + 77;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub03[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff03 = cartpole_QP_solver_dl_aff + 77;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff03 = cartpole_QP_solver_ds_aff + 77;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc03 = cartpole_QP_solver_dl_cc + 77;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc03 = cartpole_QP_solver_ds_cc + 77;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub03 = cartpole_QP_solver_ccrhs + 77;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp03 = cartpole_QP_solver_s + 78;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp03 = cartpole_QP_solver_l + 78;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp03 = cartpole_QP_solver_lbys + 78;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff03 = cartpole_QP_solver_dl_aff + 78;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff03 = cartpole_QP_solver_ds_aff + 78;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc03 = cartpole_QP_solver_dl_cc + 78;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc03 = cartpole_QP_solver_ds_cc + 78;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp03 = cartpole_QP_solver_ccrhs + 78;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip03[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi03[120];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W03[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd03[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd03[36];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z04 = cartpole_QP_solver_z + 60;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff04 = cartpole_QP_solver_dz_aff + 60;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc04 = cartpole_QP_solver_dz_cc + 60;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd04 = cartpole_QP_solver_rd + 60;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd04[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost04 = cartpole_QP_solver_grad_cost + 60;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq04 = cartpole_QP_solver_grad_eq + 60;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq04 = cartpole_QP_solver_grad_ineq + 60;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv04[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v04 = cartpole_QP_solver_v + 24;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re04[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta04[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc04[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff04 = cartpole_QP_solver_dv_aff + 24;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc04 = cartpole_QP_solver_dv_cc + 24;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V04[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd04[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld04[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy04[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy04[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c04[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx04[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb04 = cartpole_QP_solver_l + 98;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb04 = cartpole_QP_solver_s + 98;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb04 = cartpole_QP_solver_lbys + 98;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb04[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff04 = cartpole_QP_solver_dl_aff + 98;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff04 = cartpole_QP_solver_ds_aff + 98;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc04 = cartpole_QP_solver_dl_cc + 98;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc04 = cartpole_QP_solver_ds_cc + 98;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl04 = cartpole_QP_solver_ccrhs + 98;
int cartpole_QP_solver_ubIdx04[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub04 = cartpole_QP_solver_l + 99;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub04 = cartpole_QP_solver_s + 99;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub04 = cartpole_QP_solver_lbys + 99;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub04[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff04 = cartpole_QP_solver_dl_aff + 99;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff04 = cartpole_QP_solver_ds_aff + 99;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc04 = cartpole_QP_solver_dl_cc + 99;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc04 = cartpole_QP_solver_ds_cc + 99;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub04 = cartpole_QP_solver_ccrhs + 99;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp04 = cartpole_QP_solver_s + 100;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp04 = cartpole_QP_solver_l + 100;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp04 = cartpole_QP_solver_lbys + 100;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff04 = cartpole_QP_solver_dl_aff + 100;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff04 = cartpole_QP_solver_ds_aff + 100;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc04 = cartpole_QP_solver_dl_cc + 100;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc04 = cartpole_QP_solver_ds_cc + 100;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp04 = cartpole_QP_solver_ccrhs + 100;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip04[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi04[120];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W04[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd04[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd04[36];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z05 = cartpole_QP_solver_z + 75;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff05 = cartpole_QP_solver_dz_aff + 75;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc05 = cartpole_QP_solver_dz_cc + 75;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd05 = cartpole_QP_solver_rd + 75;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd05[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost05 = cartpole_QP_solver_grad_cost + 75;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq05 = cartpole_QP_solver_grad_eq + 75;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq05 = cartpole_QP_solver_grad_ineq + 75;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv05[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v05 = cartpole_QP_solver_v + 30;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re05[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta05[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc05[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff05 = cartpole_QP_solver_dv_aff + 30;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc05 = cartpole_QP_solver_dv_cc + 30;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V05[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd05[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld05[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy05[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy05[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c05[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx05[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb05 = cartpole_QP_solver_l + 120;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb05 = cartpole_QP_solver_s + 120;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb05 = cartpole_QP_solver_lbys + 120;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb05[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff05 = cartpole_QP_solver_dl_aff + 120;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff05 = cartpole_QP_solver_ds_aff + 120;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc05 = cartpole_QP_solver_dl_cc + 120;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc05 = cartpole_QP_solver_ds_cc + 120;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl05 = cartpole_QP_solver_ccrhs + 120;
int cartpole_QP_solver_ubIdx05[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub05 = cartpole_QP_solver_l + 121;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub05 = cartpole_QP_solver_s + 121;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub05 = cartpole_QP_solver_lbys + 121;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub05[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff05 = cartpole_QP_solver_dl_aff + 121;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff05 = cartpole_QP_solver_ds_aff + 121;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc05 = cartpole_QP_solver_dl_cc + 121;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc05 = cartpole_QP_solver_ds_cc + 121;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub05 = cartpole_QP_solver_ccrhs + 121;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp05 = cartpole_QP_solver_s + 122;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp05 = cartpole_QP_solver_l + 122;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp05 = cartpole_QP_solver_lbys + 122;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff05 = cartpole_QP_solver_dl_aff + 122;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff05 = cartpole_QP_solver_ds_aff + 122;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc05 = cartpole_QP_solver_dl_cc + 122;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc05 = cartpole_QP_solver_ds_cc + 122;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp05 = cartpole_QP_solver_ccrhs + 122;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip05[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi05[120];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W05[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd05[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd05[36];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z06 = cartpole_QP_solver_z + 90;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff06 = cartpole_QP_solver_dz_aff + 90;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc06 = cartpole_QP_solver_dz_cc + 90;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd06 = cartpole_QP_solver_rd + 90;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd06[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost06 = cartpole_QP_solver_grad_cost + 90;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq06 = cartpole_QP_solver_grad_eq + 90;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq06 = cartpole_QP_solver_grad_ineq + 90;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv06[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v06 = cartpole_QP_solver_v + 36;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re06[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta06[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc06[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff06 = cartpole_QP_solver_dv_aff + 36;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc06 = cartpole_QP_solver_dv_cc + 36;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V06[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd06[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld06[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy06[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy06[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c06[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx06[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb06 = cartpole_QP_solver_l + 142;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb06 = cartpole_QP_solver_s + 142;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb06 = cartpole_QP_solver_lbys + 142;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb06[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff06 = cartpole_QP_solver_dl_aff + 142;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff06 = cartpole_QP_solver_ds_aff + 142;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc06 = cartpole_QP_solver_dl_cc + 142;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc06 = cartpole_QP_solver_ds_cc + 142;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl06 = cartpole_QP_solver_ccrhs + 142;
int cartpole_QP_solver_ubIdx06[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub06 = cartpole_QP_solver_l + 143;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub06 = cartpole_QP_solver_s + 143;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub06 = cartpole_QP_solver_lbys + 143;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub06[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff06 = cartpole_QP_solver_dl_aff + 143;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff06 = cartpole_QP_solver_ds_aff + 143;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc06 = cartpole_QP_solver_dl_cc + 143;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc06 = cartpole_QP_solver_ds_cc + 143;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub06 = cartpole_QP_solver_ccrhs + 143;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp06 = cartpole_QP_solver_s + 144;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp06 = cartpole_QP_solver_l + 144;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp06 = cartpole_QP_solver_lbys + 144;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff06 = cartpole_QP_solver_dl_aff + 144;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff06 = cartpole_QP_solver_ds_aff + 144;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc06 = cartpole_QP_solver_dl_cc + 144;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc06 = cartpole_QP_solver_ds_cc + 144;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp06 = cartpole_QP_solver_ccrhs + 144;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip06[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi06[120];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W06[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd06[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd06[36];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z07 = cartpole_QP_solver_z + 105;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff07 = cartpole_QP_solver_dz_aff + 105;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc07 = cartpole_QP_solver_dz_cc + 105;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd07 = cartpole_QP_solver_rd + 105;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd07[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost07 = cartpole_QP_solver_grad_cost + 105;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq07 = cartpole_QP_solver_grad_eq + 105;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq07 = cartpole_QP_solver_grad_ineq + 105;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv07[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v07 = cartpole_QP_solver_v + 42;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re07[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta07[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc07[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff07 = cartpole_QP_solver_dv_aff + 42;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc07 = cartpole_QP_solver_dv_cc + 42;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V07[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd07[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld07[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy07[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy07[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c07[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx07[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb07 = cartpole_QP_solver_l + 164;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb07 = cartpole_QP_solver_s + 164;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb07 = cartpole_QP_solver_lbys + 164;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb07[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff07 = cartpole_QP_solver_dl_aff + 164;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff07 = cartpole_QP_solver_ds_aff + 164;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc07 = cartpole_QP_solver_dl_cc + 164;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc07 = cartpole_QP_solver_ds_cc + 164;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl07 = cartpole_QP_solver_ccrhs + 164;
int cartpole_QP_solver_ubIdx07[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub07 = cartpole_QP_solver_l + 165;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub07 = cartpole_QP_solver_s + 165;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub07 = cartpole_QP_solver_lbys + 165;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub07[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff07 = cartpole_QP_solver_dl_aff + 165;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff07 = cartpole_QP_solver_ds_aff + 165;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc07 = cartpole_QP_solver_dl_cc + 165;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc07 = cartpole_QP_solver_ds_cc + 165;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub07 = cartpole_QP_solver_ccrhs + 165;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp07 = cartpole_QP_solver_s + 166;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp07 = cartpole_QP_solver_l + 166;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp07 = cartpole_QP_solver_lbys + 166;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff07 = cartpole_QP_solver_dl_aff + 166;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff07 = cartpole_QP_solver_ds_aff + 166;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc07 = cartpole_QP_solver_dl_cc + 166;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc07 = cartpole_QP_solver_ds_cc + 166;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp07 = cartpole_QP_solver_ccrhs + 166;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip07[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi07[120];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W07[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd07[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd07[36];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z08 = cartpole_QP_solver_z + 120;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff08 = cartpole_QP_solver_dz_aff + 120;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc08 = cartpole_QP_solver_dz_cc + 120;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd08 = cartpole_QP_solver_rd + 120;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd08[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost08 = cartpole_QP_solver_grad_cost + 120;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq08 = cartpole_QP_solver_grad_eq + 120;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq08 = cartpole_QP_solver_grad_ineq + 120;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv08[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v08 = cartpole_QP_solver_v + 48;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re08[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta08[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc08[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff08 = cartpole_QP_solver_dv_aff + 48;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc08 = cartpole_QP_solver_dv_cc + 48;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V08[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd08[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld08[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy08[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy08[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c08[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx08[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb08 = cartpole_QP_solver_l + 186;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb08 = cartpole_QP_solver_s + 186;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb08 = cartpole_QP_solver_lbys + 186;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb08[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff08 = cartpole_QP_solver_dl_aff + 186;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff08 = cartpole_QP_solver_ds_aff + 186;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc08 = cartpole_QP_solver_dl_cc + 186;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc08 = cartpole_QP_solver_ds_cc + 186;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl08 = cartpole_QP_solver_ccrhs + 186;
int cartpole_QP_solver_ubIdx08[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub08 = cartpole_QP_solver_l + 187;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub08 = cartpole_QP_solver_s + 187;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub08 = cartpole_QP_solver_lbys + 187;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub08[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff08 = cartpole_QP_solver_dl_aff + 187;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff08 = cartpole_QP_solver_ds_aff + 187;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc08 = cartpole_QP_solver_dl_cc + 187;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc08 = cartpole_QP_solver_ds_cc + 187;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub08 = cartpole_QP_solver_ccrhs + 187;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp08 = cartpole_QP_solver_s + 188;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp08 = cartpole_QP_solver_l + 188;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp08 = cartpole_QP_solver_lbys + 188;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff08 = cartpole_QP_solver_dl_aff + 188;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff08 = cartpole_QP_solver_ds_aff + 188;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc08 = cartpole_QP_solver_dl_cc + 188;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc08 = cartpole_QP_solver_ds_cc + 188;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp08 = cartpole_QP_solver_ccrhs + 188;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip08[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi08[120];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W08[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd08[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd08[36];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z09 = cartpole_QP_solver_z + 135;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff09 = cartpole_QP_solver_dz_aff + 135;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc09 = cartpole_QP_solver_dz_cc + 135;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd09 = cartpole_QP_solver_rd + 135;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd09[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost09 = cartpole_QP_solver_grad_cost + 135;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq09 = cartpole_QP_solver_grad_eq + 135;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq09 = cartpole_QP_solver_grad_ineq + 135;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv09[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v09 = cartpole_QP_solver_v + 54;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re09[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta09[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc09[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff09 = cartpole_QP_solver_dv_aff + 54;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc09 = cartpole_QP_solver_dv_cc + 54;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V09[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd09[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld09[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy09[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy09[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c09[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx09[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb09 = cartpole_QP_solver_l + 208;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb09 = cartpole_QP_solver_s + 208;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb09 = cartpole_QP_solver_lbys + 208;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb09[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff09 = cartpole_QP_solver_dl_aff + 208;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff09 = cartpole_QP_solver_ds_aff + 208;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc09 = cartpole_QP_solver_dl_cc + 208;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc09 = cartpole_QP_solver_ds_cc + 208;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl09 = cartpole_QP_solver_ccrhs + 208;
int cartpole_QP_solver_ubIdx09[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub09 = cartpole_QP_solver_l + 209;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub09 = cartpole_QP_solver_s + 209;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub09 = cartpole_QP_solver_lbys + 209;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub09[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff09 = cartpole_QP_solver_dl_aff + 209;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff09 = cartpole_QP_solver_ds_aff + 209;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc09 = cartpole_QP_solver_dl_cc + 209;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc09 = cartpole_QP_solver_ds_cc + 209;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub09 = cartpole_QP_solver_ccrhs + 209;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp09 = cartpole_QP_solver_s + 210;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp09 = cartpole_QP_solver_l + 210;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp09 = cartpole_QP_solver_lbys + 210;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff09 = cartpole_QP_solver_dl_aff + 210;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff09 = cartpole_QP_solver_ds_aff + 210;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc09 = cartpole_QP_solver_dl_cc + 210;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc09 = cartpole_QP_solver_ds_cc + 210;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp09 = cartpole_QP_solver_ccrhs + 210;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip09[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi09[120];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W09[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd09[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd09[36];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z10 = cartpole_QP_solver_z + 150;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff10 = cartpole_QP_solver_dz_aff + 150;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc10 = cartpole_QP_solver_dz_cc + 150;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd10 = cartpole_QP_solver_rd + 150;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd10[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost10 = cartpole_QP_solver_grad_cost + 150;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq10 = cartpole_QP_solver_grad_eq + 150;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq10 = cartpole_QP_solver_grad_ineq + 150;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv10[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v10 = cartpole_QP_solver_v + 60;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re10[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta10[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc10[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff10 = cartpole_QP_solver_dv_aff + 60;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc10 = cartpole_QP_solver_dv_cc + 60;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V10[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd10[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld10[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy10[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy10[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c10[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx10[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb10 = cartpole_QP_solver_l + 230;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb10 = cartpole_QP_solver_s + 230;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb10 = cartpole_QP_solver_lbys + 230;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb10[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff10 = cartpole_QP_solver_dl_aff + 230;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff10 = cartpole_QP_solver_ds_aff + 230;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc10 = cartpole_QP_solver_dl_cc + 230;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc10 = cartpole_QP_solver_ds_cc + 230;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl10 = cartpole_QP_solver_ccrhs + 230;
int cartpole_QP_solver_ubIdx10[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub10 = cartpole_QP_solver_l + 231;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub10 = cartpole_QP_solver_s + 231;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub10 = cartpole_QP_solver_lbys + 231;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub10[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff10 = cartpole_QP_solver_dl_aff + 231;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff10 = cartpole_QP_solver_ds_aff + 231;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc10 = cartpole_QP_solver_dl_cc + 231;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc10 = cartpole_QP_solver_ds_cc + 231;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub10 = cartpole_QP_solver_ccrhs + 231;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp10 = cartpole_QP_solver_s + 232;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp10 = cartpole_QP_solver_l + 232;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp10 = cartpole_QP_solver_lbys + 232;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff10 = cartpole_QP_solver_dl_aff + 232;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff10 = cartpole_QP_solver_ds_aff + 232;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc10 = cartpole_QP_solver_dl_cc + 232;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc10 = cartpole_QP_solver_ds_cc + 232;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp10 = cartpole_QP_solver_ccrhs + 232;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip10[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi10[120];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W10[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd10[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd10[36];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z11 = cartpole_QP_solver_z + 165;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff11 = cartpole_QP_solver_dz_aff + 165;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc11 = cartpole_QP_solver_dz_cc + 165;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd11 = cartpole_QP_solver_rd + 165;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd11[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost11 = cartpole_QP_solver_grad_cost + 165;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq11 = cartpole_QP_solver_grad_eq + 165;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq11 = cartpole_QP_solver_grad_ineq + 165;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv11[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v11 = cartpole_QP_solver_v + 66;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re11[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta11[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc11[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff11 = cartpole_QP_solver_dv_aff + 66;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc11 = cartpole_QP_solver_dv_cc + 66;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V11[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd11[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld11[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy11[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy11[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c11[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx11[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb11 = cartpole_QP_solver_l + 252;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb11 = cartpole_QP_solver_s + 252;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb11 = cartpole_QP_solver_lbys + 252;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb11[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff11 = cartpole_QP_solver_dl_aff + 252;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff11 = cartpole_QP_solver_ds_aff + 252;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc11 = cartpole_QP_solver_dl_cc + 252;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc11 = cartpole_QP_solver_ds_cc + 252;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl11 = cartpole_QP_solver_ccrhs + 252;
int cartpole_QP_solver_ubIdx11[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub11 = cartpole_QP_solver_l + 253;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub11 = cartpole_QP_solver_s + 253;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub11 = cartpole_QP_solver_lbys + 253;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub11[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff11 = cartpole_QP_solver_dl_aff + 253;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff11 = cartpole_QP_solver_ds_aff + 253;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc11 = cartpole_QP_solver_dl_cc + 253;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc11 = cartpole_QP_solver_ds_cc + 253;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub11 = cartpole_QP_solver_ccrhs + 253;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp11 = cartpole_QP_solver_s + 254;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp11 = cartpole_QP_solver_l + 254;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp11 = cartpole_QP_solver_lbys + 254;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff11 = cartpole_QP_solver_dl_aff + 254;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff11 = cartpole_QP_solver_ds_aff + 254;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc11 = cartpole_QP_solver_dl_cc + 254;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc11 = cartpole_QP_solver_ds_cc + 254;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp11 = cartpole_QP_solver_ccrhs + 254;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip11[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi11[120];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W11[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd11[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd11[36];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z12 = cartpole_QP_solver_z + 180;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff12 = cartpole_QP_solver_dz_aff + 180;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc12 = cartpole_QP_solver_dz_cc + 180;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd12 = cartpole_QP_solver_rd + 180;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd12[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost12 = cartpole_QP_solver_grad_cost + 180;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq12 = cartpole_QP_solver_grad_eq + 180;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq12 = cartpole_QP_solver_grad_ineq + 180;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv12[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v12 = cartpole_QP_solver_v + 72;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re12[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta12[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc12[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff12 = cartpole_QP_solver_dv_aff + 72;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc12 = cartpole_QP_solver_dv_cc + 72;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V12[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd12[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld12[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy12[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy12[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c12[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx12[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb12 = cartpole_QP_solver_l + 274;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb12 = cartpole_QP_solver_s + 274;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb12 = cartpole_QP_solver_lbys + 274;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb12[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff12 = cartpole_QP_solver_dl_aff + 274;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff12 = cartpole_QP_solver_ds_aff + 274;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc12 = cartpole_QP_solver_dl_cc + 274;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc12 = cartpole_QP_solver_ds_cc + 274;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl12 = cartpole_QP_solver_ccrhs + 274;
int cartpole_QP_solver_ubIdx12[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub12 = cartpole_QP_solver_l + 275;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub12 = cartpole_QP_solver_s + 275;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub12 = cartpole_QP_solver_lbys + 275;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub12[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff12 = cartpole_QP_solver_dl_aff + 275;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff12 = cartpole_QP_solver_ds_aff + 275;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc12 = cartpole_QP_solver_dl_cc + 275;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc12 = cartpole_QP_solver_ds_cc + 275;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub12 = cartpole_QP_solver_ccrhs + 275;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp12 = cartpole_QP_solver_s + 276;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp12 = cartpole_QP_solver_l + 276;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp12 = cartpole_QP_solver_lbys + 276;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff12 = cartpole_QP_solver_dl_aff + 276;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff12 = cartpole_QP_solver_ds_aff + 276;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc12 = cartpole_QP_solver_dl_cc + 276;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc12 = cartpole_QP_solver_ds_cc + 276;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp12 = cartpole_QP_solver_ccrhs + 276;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip12[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi12[120];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W12[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd12[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd12[36];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z13 = cartpole_QP_solver_z + 195;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff13 = cartpole_QP_solver_dz_aff + 195;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc13 = cartpole_QP_solver_dz_cc + 195;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd13 = cartpole_QP_solver_rd + 195;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd13[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost13 = cartpole_QP_solver_grad_cost + 195;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq13 = cartpole_QP_solver_grad_eq + 195;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq13 = cartpole_QP_solver_grad_ineq + 195;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv13[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v13 = cartpole_QP_solver_v + 78;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re13[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta13[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc13[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff13 = cartpole_QP_solver_dv_aff + 78;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc13 = cartpole_QP_solver_dv_cc + 78;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V13[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd13[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld13[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy13[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy13[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c13[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx13[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb13 = cartpole_QP_solver_l + 296;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb13 = cartpole_QP_solver_s + 296;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb13 = cartpole_QP_solver_lbys + 296;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb13[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff13 = cartpole_QP_solver_dl_aff + 296;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff13 = cartpole_QP_solver_ds_aff + 296;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc13 = cartpole_QP_solver_dl_cc + 296;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc13 = cartpole_QP_solver_ds_cc + 296;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl13 = cartpole_QP_solver_ccrhs + 296;
int cartpole_QP_solver_ubIdx13[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub13 = cartpole_QP_solver_l + 297;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub13 = cartpole_QP_solver_s + 297;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub13 = cartpole_QP_solver_lbys + 297;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub13[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff13 = cartpole_QP_solver_dl_aff + 297;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff13 = cartpole_QP_solver_ds_aff + 297;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc13 = cartpole_QP_solver_dl_cc + 297;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc13 = cartpole_QP_solver_ds_cc + 297;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub13 = cartpole_QP_solver_ccrhs + 297;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp13 = cartpole_QP_solver_s + 298;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp13 = cartpole_QP_solver_l + 298;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp13 = cartpole_QP_solver_lbys + 298;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff13 = cartpole_QP_solver_dl_aff + 298;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff13 = cartpole_QP_solver_ds_aff + 298;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc13 = cartpole_QP_solver_dl_cc + 298;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc13 = cartpole_QP_solver_ds_cc + 298;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp13 = cartpole_QP_solver_ccrhs + 298;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip13[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi13[120];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W13[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd13[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd13[36];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z14 = cartpole_QP_solver_z + 210;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff14 = cartpole_QP_solver_dz_aff + 210;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc14 = cartpole_QP_solver_dz_cc + 210;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd14 = cartpole_QP_solver_rd + 210;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd14[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost14 = cartpole_QP_solver_grad_cost + 210;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq14 = cartpole_QP_solver_grad_eq + 210;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq14 = cartpole_QP_solver_grad_ineq + 210;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv14[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v14 = cartpole_QP_solver_v + 84;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re14[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta14[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc14[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff14 = cartpole_QP_solver_dv_aff + 84;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc14 = cartpole_QP_solver_dv_cc + 84;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V14[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd14[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld14[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy14[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy14[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c14[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx14[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb14 = cartpole_QP_solver_l + 318;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb14 = cartpole_QP_solver_s + 318;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb14 = cartpole_QP_solver_lbys + 318;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb14[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff14 = cartpole_QP_solver_dl_aff + 318;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff14 = cartpole_QP_solver_ds_aff + 318;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc14 = cartpole_QP_solver_dl_cc + 318;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc14 = cartpole_QP_solver_ds_cc + 318;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl14 = cartpole_QP_solver_ccrhs + 318;
int cartpole_QP_solver_ubIdx14[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub14 = cartpole_QP_solver_l + 319;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub14 = cartpole_QP_solver_s + 319;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub14 = cartpole_QP_solver_lbys + 319;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub14[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff14 = cartpole_QP_solver_dl_aff + 319;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff14 = cartpole_QP_solver_ds_aff + 319;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc14 = cartpole_QP_solver_dl_cc + 319;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc14 = cartpole_QP_solver_ds_cc + 319;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub14 = cartpole_QP_solver_ccrhs + 319;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp14 = cartpole_QP_solver_s + 320;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp14 = cartpole_QP_solver_l + 320;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp14 = cartpole_QP_solver_lbys + 320;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff14 = cartpole_QP_solver_dl_aff + 320;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff14 = cartpole_QP_solver_ds_aff + 320;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc14 = cartpole_QP_solver_dl_cc + 320;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc14 = cartpole_QP_solver_ds_cc + 320;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp14 = cartpole_QP_solver_ccrhs + 320;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip14[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi14[120];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W14[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd14[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd14[36];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z15 = cartpole_QP_solver_z + 225;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff15 = cartpole_QP_solver_dz_aff + 225;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc15 = cartpole_QP_solver_dz_cc + 225;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd15 = cartpole_QP_solver_rd + 225;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd15[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost15 = cartpole_QP_solver_grad_cost + 225;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq15 = cartpole_QP_solver_grad_eq + 225;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq15 = cartpole_QP_solver_grad_ineq + 225;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv15[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v15 = cartpole_QP_solver_v + 90;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re15[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta15[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc15[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff15 = cartpole_QP_solver_dv_aff + 90;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc15 = cartpole_QP_solver_dv_cc + 90;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V15[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd15[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld15[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy15[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy15[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c15[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx15[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb15 = cartpole_QP_solver_l + 340;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb15 = cartpole_QP_solver_s + 340;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb15 = cartpole_QP_solver_lbys + 340;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb15[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff15 = cartpole_QP_solver_dl_aff + 340;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff15 = cartpole_QP_solver_ds_aff + 340;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc15 = cartpole_QP_solver_dl_cc + 340;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc15 = cartpole_QP_solver_ds_cc + 340;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl15 = cartpole_QP_solver_ccrhs + 340;
int cartpole_QP_solver_ubIdx15[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub15 = cartpole_QP_solver_l + 341;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub15 = cartpole_QP_solver_s + 341;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub15 = cartpole_QP_solver_lbys + 341;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub15[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff15 = cartpole_QP_solver_dl_aff + 341;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff15 = cartpole_QP_solver_ds_aff + 341;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc15 = cartpole_QP_solver_dl_cc + 341;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc15 = cartpole_QP_solver_ds_cc + 341;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub15 = cartpole_QP_solver_ccrhs + 341;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp15 = cartpole_QP_solver_s + 342;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp15 = cartpole_QP_solver_l + 342;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp15 = cartpole_QP_solver_lbys + 342;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff15 = cartpole_QP_solver_dl_aff + 342;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff15 = cartpole_QP_solver_ds_aff + 342;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc15 = cartpole_QP_solver_dl_cc + 342;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc15 = cartpole_QP_solver_ds_cc + 342;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp15 = cartpole_QP_solver_ccrhs + 342;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip15[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi15[120];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W15[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd15[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd15[36];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z16 = cartpole_QP_solver_z + 240;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff16 = cartpole_QP_solver_dz_aff + 240;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc16 = cartpole_QP_solver_dz_cc + 240;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd16 = cartpole_QP_solver_rd + 240;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd16[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost16 = cartpole_QP_solver_grad_cost + 240;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq16 = cartpole_QP_solver_grad_eq + 240;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq16 = cartpole_QP_solver_grad_ineq + 240;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv16[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v16 = cartpole_QP_solver_v + 96;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re16[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta16[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc16[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff16 = cartpole_QP_solver_dv_aff + 96;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc16 = cartpole_QP_solver_dv_cc + 96;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V16[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd16[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld16[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy16[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy16[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c16[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx16[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb16 = cartpole_QP_solver_l + 362;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb16 = cartpole_QP_solver_s + 362;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb16 = cartpole_QP_solver_lbys + 362;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb16[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff16 = cartpole_QP_solver_dl_aff + 362;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff16 = cartpole_QP_solver_ds_aff + 362;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc16 = cartpole_QP_solver_dl_cc + 362;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc16 = cartpole_QP_solver_ds_cc + 362;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl16 = cartpole_QP_solver_ccrhs + 362;
int cartpole_QP_solver_ubIdx16[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub16 = cartpole_QP_solver_l + 363;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub16 = cartpole_QP_solver_s + 363;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub16 = cartpole_QP_solver_lbys + 363;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub16[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff16 = cartpole_QP_solver_dl_aff + 363;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff16 = cartpole_QP_solver_ds_aff + 363;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc16 = cartpole_QP_solver_dl_cc + 363;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc16 = cartpole_QP_solver_ds_cc + 363;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub16 = cartpole_QP_solver_ccrhs + 363;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp16 = cartpole_QP_solver_s + 364;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp16 = cartpole_QP_solver_l + 364;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp16 = cartpole_QP_solver_lbys + 364;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff16 = cartpole_QP_solver_dl_aff + 364;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff16 = cartpole_QP_solver_ds_aff + 364;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc16 = cartpole_QP_solver_dl_cc + 364;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc16 = cartpole_QP_solver_ds_cc + 364;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp16 = cartpole_QP_solver_ccrhs + 364;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip16[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi16[120];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W16[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd16[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd16[36];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z17 = cartpole_QP_solver_z + 255;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff17 = cartpole_QP_solver_dz_aff + 255;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc17 = cartpole_QP_solver_dz_cc + 255;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd17 = cartpole_QP_solver_rd + 255;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd17[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost17 = cartpole_QP_solver_grad_cost + 255;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq17 = cartpole_QP_solver_grad_eq + 255;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq17 = cartpole_QP_solver_grad_ineq + 255;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv17[15];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v17 = cartpole_QP_solver_v + 102;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re17[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta17[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc17[6];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff17 = cartpole_QP_solver_dv_aff + 102;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc17 = cartpole_QP_solver_dv_cc + 102;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V17[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd17[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld17[21];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy17[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy17[6];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c17[6] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx17[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb17 = cartpole_QP_solver_l + 384;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb17 = cartpole_QP_solver_s + 384;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb17 = cartpole_QP_solver_lbys + 384;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb17[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff17 = cartpole_QP_solver_dl_aff + 384;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff17 = cartpole_QP_solver_ds_aff + 384;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc17 = cartpole_QP_solver_dl_cc + 384;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc17 = cartpole_QP_solver_ds_cc + 384;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl17 = cartpole_QP_solver_ccrhs + 384;
int cartpole_QP_solver_ubIdx17[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub17 = cartpole_QP_solver_l + 385;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub17 = cartpole_QP_solver_s + 385;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub17 = cartpole_QP_solver_lbys + 385;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub17[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff17 = cartpole_QP_solver_dl_aff + 385;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff17 = cartpole_QP_solver_ds_aff + 385;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc17 = cartpole_QP_solver_dl_cc + 385;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc17 = cartpole_QP_solver_ds_cc + 385;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub17 = cartpole_QP_solver_ccrhs + 385;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp17 = cartpole_QP_solver_s + 386;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp17 = cartpole_QP_solver_l + 386;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp17 = cartpole_QP_solver_lbys + 386;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff17 = cartpole_QP_solver_dl_aff + 386;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff17 = cartpole_QP_solver_ds_aff + 386;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc17 = cartpole_QP_solver_dl_cc + 386;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc17 = cartpole_QP_solver_ds_cc + 386;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp17 = cartpole_QP_solver_ccrhs + 386;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip17[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi17[120];
cartpole_QP_solver_FLOAT cartpole_QP_solver_W17[90];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd17[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd17[36];
cartpole_QP_solver_FLOAT cartpole_QP_solver_H18[14] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z18 = cartpole_QP_solver_z + 270;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff18 = cartpole_QP_solver_dz_aff + 270;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc18 = cartpole_QP_solver_dz_cc + 270;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd18 = cartpole_QP_solver_rd + 270;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd18[14];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost18 = cartpole_QP_solver_grad_cost + 270;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq18 = cartpole_QP_solver_grad_eq + 270;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq18 = cartpole_QP_solver_grad_ineq + 270;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv18[14];
cartpole_QP_solver_FLOAT cartpole_QP_solver_C18[56] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_v18 = cartpole_QP_solver_v + 108;
cartpole_QP_solver_FLOAT cartpole_QP_solver_re18[4];
cartpole_QP_solver_FLOAT cartpole_QP_solver_beta18[4];
cartpole_QP_solver_FLOAT cartpole_QP_solver_betacc18[4];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvaff18 = cartpole_QP_solver_dv_aff + 108;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dvcc18 = cartpole_QP_solver_dv_cc + 108;
cartpole_QP_solver_FLOAT cartpole_QP_solver_V18[56];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Yd18[10];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ld18[10];
cartpole_QP_solver_FLOAT cartpole_QP_solver_yy18[4];
cartpole_QP_solver_FLOAT cartpole_QP_solver_bmy18[4];
cartpole_QP_solver_FLOAT cartpole_QP_solver_c18[4] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int cartpole_QP_solver_lbIdx18[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb18 = cartpole_QP_solver_l + 406;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb18 = cartpole_QP_solver_s + 406;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb18 = cartpole_QP_solver_lbys + 406;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb18[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff18 = cartpole_QP_solver_dl_aff + 406;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff18 = cartpole_QP_solver_ds_aff + 406;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc18 = cartpole_QP_solver_dl_cc + 406;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc18 = cartpole_QP_solver_ds_cc + 406;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl18 = cartpole_QP_solver_ccrhs + 406;
int cartpole_QP_solver_ubIdx18[1] = {4};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub18 = cartpole_QP_solver_l + 407;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub18 = cartpole_QP_solver_s + 407;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub18 = cartpole_QP_solver_lbys + 407;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub18[1];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff18 = cartpole_QP_solver_dl_aff + 407;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff18 = cartpole_QP_solver_ds_aff + 407;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc18 = cartpole_QP_solver_dl_cc + 407;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc18 = cartpole_QP_solver_ds_cc + 407;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub18 = cartpole_QP_solver_ccrhs + 407;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sp18 = cartpole_QP_solver_s + 408;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lp18 = cartpole_QP_solver_l + 408;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lpbysp18 = cartpole_QP_solver_lbys + 408;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_aff18 = cartpole_QP_solver_dl_aff + 408;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_aff18 = cartpole_QP_solver_ds_aff + 408;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlp_cc18 = cartpole_QP_solver_dl_cc + 408;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsp_cc18 = cartpole_QP_solver_ds_cc + 408;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsp18 = cartpole_QP_solver_ccrhs + 408;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rip18[20];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi18[105];
cartpole_QP_solver_FLOAT cartpole_QP_solver_D18[84] = {-1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
cartpole_QP_solver_FLOAT cartpole_QP_solver_W18[84];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Ysd18[24];
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lsd18[24];
cartpole_QP_solver_FLOAT cartpole_QP_solver_H19[4] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_z19 = cartpole_QP_solver_z + 284;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzaff19 = cartpole_QP_solver_dz_aff + 284;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dzcc19 = cartpole_QP_solver_dz_cc + 284;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_rd19 = cartpole_QP_solver_rd + 284;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Lbyrd19[4];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_cost19 = cartpole_QP_solver_grad_cost + 284;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_eq19 = cartpole_QP_solver_grad_eq + 284;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_grad_ineq19 = cartpole_QP_solver_grad_ineq + 284;
cartpole_QP_solver_FLOAT cartpole_QP_solver_ctv19[4];
int cartpole_QP_solver_lbIdx19[4] = {0, 1, 2, 3};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llb19 = cartpole_QP_solver_l + 428;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_slb19 = cartpole_QP_solver_s + 428;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_llbbyslb19 = cartpole_QP_solver_lbys + 428;
cartpole_QP_solver_FLOAT cartpole_QP_solver_rilb19[4];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbaff19 = cartpole_QP_solver_dl_aff + 428;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbaff19 = cartpole_QP_solver_ds_aff + 428;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dllbcc19 = cartpole_QP_solver_dl_cc + 428;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dslbcc19 = cartpole_QP_solver_ds_cc + 428;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsl19 = cartpole_QP_solver_ccrhs + 428;
int cartpole_QP_solver_ubIdx19[4] = {0, 1, 2, 3};
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lub19 = cartpole_QP_solver_l + 432;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_sub19 = cartpole_QP_solver_s + 432;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_lubbysub19 = cartpole_QP_solver_lbys + 432;
cartpole_QP_solver_FLOAT cartpole_QP_solver_riub19[4];
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubaff19 = cartpole_QP_solver_dl_aff + 432;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubaff19 = cartpole_QP_solver_ds_aff + 432;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dlubcc19 = cartpole_QP_solver_dl_cc + 432;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_dsubcc19 = cartpole_QP_solver_ds_cc + 432;
cartpole_QP_solver_FLOAT* cartpole_QP_solver_ccrhsub19 = cartpole_QP_solver_ccrhs + 432;
cartpole_QP_solver_FLOAT cartpole_QP_solver_Phi19[4];
cartpole_QP_solver_FLOAT cartpole_QP_solver_D19[4] = {-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000};
cartpole_QP_solver_FLOAT cartpole_QP_solver_W19[4];
cartpole_QP_solver_FLOAT musigma;
cartpole_QP_solver_FLOAT sigma_3rdroot;




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
cartpole_QP_solver_LA_INITIALIZEVECTOR_288(cartpole_QP_solver_z, 0);
cartpole_QP_solver_LA_INITIALIZEVECTOR_112(cartpole_QP_solver_v, 1);
cartpole_QP_solver_LA_INITIALIZEVECTOR_436(cartpole_QP_solver_l, 10);
cartpole_QP_solver_LA_INITIALIZEVECTOR_436(cartpole_QP_solver_s, 10);
info->mu = 0;
cartpole_QP_solver_LA_DOTACC_436(cartpole_QP_solver_l, cartpole_QP_solver_s, &info->mu);
info->mu /= 436;
PRINTTEXT("This is cartpole_QP_solver, a solver generated by FORCES (forces.ethz.ch).\n");
PRINTTEXT("(c) Alexander Domahidi, Automatic Control Laboratory, ETH Zurich, 2011-2014.\n");
PRINTTEXT("\n  #it  res_eq   res_ineq     pobj         dobj       dgap     rdgap     mu\n");
PRINTTEXT("  ---------------------------------------------------------------------------\n");
while( 1 ){
info->pobj = 0;
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f1, cartpole_QP_solver_z00, cartpole_QP_solver_grad_cost00, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f2, cartpole_QP_solver_z01, cartpole_QP_solver_grad_cost01, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f3, cartpole_QP_solver_z02, cartpole_QP_solver_grad_cost02, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f4, cartpole_QP_solver_z03, cartpole_QP_solver_grad_cost03, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f5, cartpole_QP_solver_z04, cartpole_QP_solver_grad_cost04, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f6, cartpole_QP_solver_z05, cartpole_QP_solver_grad_cost05, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f7, cartpole_QP_solver_z06, cartpole_QP_solver_grad_cost06, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f8, cartpole_QP_solver_z07, cartpole_QP_solver_grad_cost07, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f9, cartpole_QP_solver_z08, cartpole_QP_solver_grad_cost08, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f10, cartpole_QP_solver_z09, cartpole_QP_solver_grad_cost09, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f11, cartpole_QP_solver_z10, cartpole_QP_solver_grad_cost10, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f12, cartpole_QP_solver_z11, cartpole_QP_solver_grad_cost11, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f13, cartpole_QP_solver_z12, cartpole_QP_solver_grad_cost12, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f14, cartpole_QP_solver_z13, cartpole_QP_solver_grad_cost13, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f15, cartpole_QP_solver_z14, cartpole_QP_solver_grad_cost14, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f16, cartpole_QP_solver_z15, cartpole_QP_solver_grad_cost15, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f17, cartpole_QP_solver_z16, cartpole_QP_solver_grad_cost16, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_15(cartpole_QP_solver_H00, params->f18, cartpole_QP_solver_z17, cartpole_QP_solver_grad_cost17, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_14(cartpole_QP_solver_H18, params->f19, cartpole_QP_solver_z18, cartpole_QP_solver_grad_cost18, &info->pobj);
cartpole_QP_solver_LA_DIAG_QUADFCN_4(cartpole_QP_solver_H19, params->f20, cartpole_QP_solver_z19, cartpole_QP_solver_grad_cost19, &info->pobj);
info->res_eq = 0;
info->dgap = 0;
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_C00, cartpole_QP_solver_z00, cartpole_QP_solver_D01, cartpole_QP_solver_z01, cartpole_QP_solver_c00, cartpole_QP_solver_v00, cartpole_QP_solver_re00, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_C00, cartpole_QP_solver_z01, cartpole_QP_solver_D01, cartpole_QP_solver_z02, cartpole_QP_solver_c01, cartpole_QP_solver_v01, cartpole_QP_solver_re01, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_C00, cartpole_QP_solver_z02, cartpole_QP_solver_D01, cartpole_QP_solver_z03, cartpole_QP_solver_c02, cartpole_QP_solver_v02, cartpole_QP_solver_re02, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_C00, cartpole_QP_solver_z03, cartpole_QP_solver_D01, cartpole_QP_solver_z04, cartpole_QP_solver_c03, cartpole_QP_solver_v03, cartpole_QP_solver_re03, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_C00, cartpole_QP_solver_z04, cartpole_QP_solver_D01, cartpole_QP_solver_z05, cartpole_QP_solver_c04, cartpole_QP_solver_v04, cartpole_QP_solver_re04, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_C00, cartpole_QP_solver_z05, cartpole_QP_solver_D01, cartpole_QP_solver_z06, cartpole_QP_solver_c05, cartpole_QP_solver_v05, cartpole_QP_solver_re05, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_C00, cartpole_QP_solver_z06, cartpole_QP_solver_D01, cartpole_QP_solver_z07, cartpole_QP_solver_c06, cartpole_QP_solver_v06, cartpole_QP_solver_re06, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_C00, cartpole_QP_solver_z07, cartpole_QP_solver_D01, cartpole_QP_solver_z08, cartpole_QP_solver_c07, cartpole_QP_solver_v07, cartpole_QP_solver_re07, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_C00, cartpole_QP_solver_z08, cartpole_QP_solver_D01, cartpole_QP_solver_z09, cartpole_QP_solver_c08, cartpole_QP_solver_v08, cartpole_QP_solver_re08, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_C00, cartpole_QP_solver_z09, cartpole_QP_solver_D01, cartpole_QP_solver_z10, cartpole_QP_solver_c09, cartpole_QP_solver_v09, cartpole_QP_solver_re09, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_C00, cartpole_QP_solver_z10, cartpole_QP_solver_D01, cartpole_QP_solver_z11, cartpole_QP_solver_c10, cartpole_QP_solver_v10, cartpole_QP_solver_re10, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_C00, cartpole_QP_solver_z11, cartpole_QP_solver_D01, cartpole_QP_solver_z12, cartpole_QP_solver_c11, cartpole_QP_solver_v11, cartpole_QP_solver_re11, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_C00, cartpole_QP_solver_z12, cartpole_QP_solver_D01, cartpole_QP_solver_z13, cartpole_QP_solver_c12, cartpole_QP_solver_v12, cartpole_QP_solver_re12, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_C00, cartpole_QP_solver_z13, cartpole_QP_solver_D01, cartpole_QP_solver_z14, cartpole_QP_solver_c13, cartpole_QP_solver_v13, cartpole_QP_solver_re13, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_C00, cartpole_QP_solver_z14, cartpole_QP_solver_D01, cartpole_QP_solver_z15, cartpole_QP_solver_c14, cartpole_QP_solver_v14, cartpole_QP_solver_re14, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_C00, cartpole_QP_solver_z15, cartpole_QP_solver_D01, cartpole_QP_solver_z16, cartpole_QP_solver_c15, cartpole_QP_solver_v15, cartpole_QP_solver_re15, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_15(cartpole_QP_solver_C00, cartpole_QP_solver_z16, cartpole_QP_solver_D01, cartpole_QP_solver_z17, cartpole_QP_solver_c16, cartpole_QP_solver_v16, cartpole_QP_solver_re16, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MVMSUB3_6_15_14(cartpole_QP_solver_C00, cartpole_QP_solver_z17, cartpole_QP_solver_D18, cartpole_QP_solver_z18, cartpole_QP_solver_c17, cartpole_QP_solver_v17, cartpole_QP_solver_re17, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_4_14_4(cartpole_QP_solver_C18, cartpole_QP_solver_z18, cartpole_QP_solver_D19, cartpole_QP_solver_z19, cartpole_QP_solver_c18, cartpole_QP_solver_v18, cartpole_QP_solver_re18, &info->dgap, &info->res_eq);
cartpole_QP_solver_LA_DENSE_MTVM_6_15(cartpole_QP_solver_C00, cartpole_QP_solver_v00, cartpole_QP_solver_grad_eq00);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_v01, cartpole_QP_solver_D01, cartpole_QP_solver_v00, cartpole_QP_solver_grad_eq01);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_v02, cartpole_QP_solver_D01, cartpole_QP_solver_v01, cartpole_QP_solver_grad_eq02);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_v03, cartpole_QP_solver_D01, cartpole_QP_solver_v02, cartpole_QP_solver_grad_eq03);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_v04, cartpole_QP_solver_D01, cartpole_QP_solver_v03, cartpole_QP_solver_grad_eq04);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_v05, cartpole_QP_solver_D01, cartpole_QP_solver_v04, cartpole_QP_solver_grad_eq05);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_v06, cartpole_QP_solver_D01, cartpole_QP_solver_v05, cartpole_QP_solver_grad_eq06);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_v07, cartpole_QP_solver_D01, cartpole_QP_solver_v06, cartpole_QP_solver_grad_eq07);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_v08, cartpole_QP_solver_D01, cartpole_QP_solver_v07, cartpole_QP_solver_grad_eq08);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_v09, cartpole_QP_solver_D01, cartpole_QP_solver_v08, cartpole_QP_solver_grad_eq09);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_v10, cartpole_QP_solver_D01, cartpole_QP_solver_v09, cartpole_QP_solver_grad_eq10);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_v11, cartpole_QP_solver_D01, cartpole_QP_solver_v10, cartpole_QP_solver_grad_eq11);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_v12, cartpole_QP_solver_D01, cartpole_QP_solver_v11, cartpole_QP_solver_grad_eq12);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_v13, cartpole_QP_solver_D01, cartpole_QP_solver_v12, cartpole_QP_solver_grad_eq13);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_v14, cartpole_QP_solver_D01, cartpole_QP_solver_v13, cartpole_QP_solver_grad_eq14);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_v15, cartpole_QP_solver_D01, cartpole_QP_solver_v14, cartpole_QP_solver_grad_eq15);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_v16, cartpole_QP_solver_D01, cartpole_QP_solver_v15, cartpole_QP_solver_grad_eq16);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_v17, cartpole_QP_solver_D01, cartpole_QP_solver_v16, cartpole_QP_solver_grad_eq17);
cartpole_QP_solver_LA_DENSE_MTVM2_4_14_6(cartpole_QP_solver_C18, cartpole_QP_solver_v18, cartpole_QP_solver_D18, cartpole_QP_solver_v17, cartpole_QP_solver_grad_eq18);
cartpole_QP_solver_LA_DIAGZERO_MTVM_4_4(cartpole_QP_solver_D19, cartpole_QP_solver_v18, cartpole_QP_solver_grad_eq19);
info->res_ineq = 0;
cartpole_QP_solver_LA_VSUBADD3_6(params->lb1, cartpole_QP_solver_z00, cartpole_QP_solver_lbIdx00, cartpole_QP_solver_llb00, cartpole_QP_solver_slb00, cartpole_QP_solver_rilb00, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_6(cartpole_QP_solver_z00, cartpole_QP_solver_ubIdx00, params->ub1, cartpole_QP_solver_lub00, cartpole_QP_solver_sub00, cartpole_QP_solver_riub00, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A1, cartpole_QP_solver_z00, params->b1, cartpole_QP_solver_sp00, cartpole_QP_solver_lp00, cartpole_QP_solver_rip00, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb2, cartpole_QP_solver_z01, cartpole_QP_solver_lbIdx01, cartpole_QP_solver_llb01, cartpole_QP_solver_slb01, cartpole_QP_solver_rilb01, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z01, cartpole_QP_solver_ubIdx01, params->ub2, cartpole_QP_solver_lub01, cartpole_QP_solver_sub01, cartpole_QP_solver_riub01, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A2, cartpole_QP_solver_z01, params->b2, cartpole_QP_solver_sp01, cartpole_QP_solver_lp01, cartpole_QP_solver_rip01, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb3, cartpole_QP_solver_z02, cartpole_QP_solver_lbIdx02, cartpole_QP_solver_llb02, cartpole_QP_solver_slb02, cartpole_QP_solver_rilb02, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z02, cartpole_QP_solver_ubIdx02, params->ub3, cartpole_QP_solver_lub02, cartpole_QP_solver_sub02, cartpole_QP_solver_riub02, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A3, cartpole_QP_solver_z02, params->b3, cartpole_QP_solver_sp02, cartpole_QP_solver_lp02, cartpole_QP_solver_rip02, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb4, cartpole_QP_solver_z03, cartpole_QP_solver_lbIdx03, cartpole_QP_solver_llb03, cartpole_QP_solver_slb03, cartpole_QP_solver_rilb03, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z03, cartpole_QP_solver_ubIdx03, params->ub4, cartpole_QP_solver_lub03, cartpole_QP_solver_sub03, cartpole_QP_solver_riub03, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A4, cartpole_QP_solver_z03, params->b4, cartpole_QP_solver_sp03, cartpole_QP_solver_lp03, cartpole_QP_solver_rip03, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb5, cartpole_QP_solver_z04, cartpole_QP_solver_lbIdx04, cartpole_QP_solver_llb04, cartpole_QP_solver_slb04, cartpole_QP_solver_rilb04, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z04, cartpole_QP_solver_ubIdx04, params->ub5, cartpole_QP_solver_lub04, cartpole_QP_solver_sub04, cartpole_QP_solver_riub04, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A5, cartpole_QP_solver_z04, params->b5, cartpole_QP_solver_sp04, cartpole_QP_solver_lp04, cartpole_QP_solver_rip04, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb6, cartpole_QP_solver_z05, cartpole_QP_solver_lbIdx05, cartpole_QP_solver_llb05, cartpole_QP_solver_slb05, cartpole_QP_solver_rilb05, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z05, cartpole_QP_solver_ubIdx05, params->ub6, cartpole_QP_solver_lub05, cartpole_QP_solver_sub05, cartpole_QP_solver_riub05, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A6, cartpole_QP_solver_z05, params->b6, cartpole_QP_solver_sp05, cartpole_QP_solver_lp05, cartpole_QP_solver_rip05, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb7, cartpole_QP_solver_z06, cartpole_QP_solver_lbIdx06, cartpole_QP_solver_llb06, cartpole_QP_solver_slb06, cartpole_QP_solver_rilb06, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z06, cartpole_QP_solver_ubIdx06, params->ub7, cartpole_QP_solver_lub06, cartpole_QP_solver_sub06, cartpole_QP_solver_riub06, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A7, cartpole_QP_solver_z06, params->b7, cartpole_QP_solver_sp06, cartpole_QP_solver_lp06, cartpole_QP_solver_rip06, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb8, cartpole_QP_solver_z07, cartpole_QP_solver_lbIdx07, cartpole_QP_solver_llb07, cartpole_QP_solver_slb07, cartpole_QP_solver_rilb07, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z07, cartpole_QP_solver_ubIdx07, params->ub8, cartpole_QP_solver_lub07, cartpole_QP_solver_sub07, cartpole_QP_solver_riub07, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A8, cartpole_QP_solver_z07, params->b8, cartpole_QP_solver_sp07, cartpole_QP_solver_lp07, cartpole_QP_solver_rip07, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb9, cartpole_QP_solver_z08, cartpole_QP_solver_lbIdx08, cartpole_QP_solver_llb08, cartpole_QP_solver_slb08, cartpole_QP_solver_rilb08, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z08, cartpole_QP_solver_ubIdx08, params->ub9, cartpole_QP_solver_lub08, cartpole_QP_solver_sub08, cartpole_QP_solver_riub08, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A9, cartpole_QP_solver_z08, params->b9, cartpole_QP_solver_sp08, cartpole_QP_solver_lp08, cartpole_QP_solver_rip08, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb10, cartpole_QP_solver_z09, cartpole_QP_solver_lbIdx09, cartpole_QP_solver_llb09, cartpole_QP_solver_slb09, cartpole_QP_solver_rilb09, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z09, cartpole_QP_solver_ubIdx09, params->ub10, cartpole_QP_solver_lub09, cartpole_QP_solver_sub09, cartpole_QP_solver_riub09, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A10, cartpole_QP_solver_z09, params->b10, cartpole_QP_solver_sp09, cartpole_QP_solver_lp09, cartpole_QP_solver_rip09, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb11, cartpole_QP_solver_z10, cartpole_QP_solver_lbIdx10, cartpole_QP_solver_llb10, cartpole_QP_solver_slb10, cartpole_QP_solver_rilb10, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z10, cartpole_QP_solver_ubIdx10, params->ub11, cartpole_QP_solver_lub10, cartpole_QP_solver_sub10, cartpole_QP_solver_riub10, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A11, cartpole_QP_solver_z10, params->b11, cartpole_QP_solver_sp10, cartpole_QP_solver_lp10, cartpole_QP_solver_rip10, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb12, cartpole_QP_solver_z11, cartpole_QP_solver_lbIdx11, cartpole_QP_solver_llb11, cartpole_QP_solver_slb11, cartpole_QP_solver_rilb11, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z11, cartpole_QP_solver_ubIdx11, params->ub12, cartpole_QP_solver_lub11, cartpole_QP_solver_sub11, cartpole_QP_solver_riub11, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A12, cartpole_QP_solver_z11, params->b12, cartpole_QP_solver_sp11, cartpole_QP_solver_lp11, cartpole_QP_solver_rip11, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb13, cartpole_QP_solver_z12, cartpole_QP_solver_lbIdx12, cartpole_QP_solver_llb12, cartpole_QP_solver_slb12, cartpole_QP_solver_rilb12, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z12, cartpole_QP_solver_ubIdx12, params->ub13, cartpole_QP_solver_lub12, cartpole_QP_solver_sub12, cartpole_QP_solver_riub12, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A13, cartpole_QP_solver_z12, params->b13, cartpole_QP_solver_sp12, cartpole_QP_solver_lp12, cartpole_QP_solver_rip12, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb14, cartpole_QP_solver_z13, cartpole_QP_solver_lbIdx13, cartpole_QP_solver_llb13, cartpole_QP_solver_slb13, cartpole_QP_solver_rilb13, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z13, cartpole_QP_solver_ubIdx13, params->ub14, cartpole_QP_solver_lub13, cartpole_QP_solver_sub13, cartpole_QP_solver_riub13, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A14, cartpole_QP_solver_z13, params->b14, cartpole_QP_solver_sp13, cartpole_QP_solver_lp13, cartpole_QP_solver_rip13, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb15, cartpole_QP_solver_z14, cartpole_QP_solver_lbIdx14, cartpole_QP_solver_llb14, cartpole_QP_solver_slb14, cartpole_QP_solver_rilb14, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z14, cartpole_QP_solver_ubIdx14, params->ub15, cartpole_QP_solver_lub14, cartpole_QP_solver_sub14, cartpole_QP_solver_riub14, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A15, cartpole_QP_solver_z14, params->b15, cartpole_QP_solver_sp14, cartpole_QP_solver_lp14, cartpole_QP_solver_rip14, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb16, cartpole_QP_solver_z15, cartpole_QP_solver_lbIdx15, cartpole_QP_solver_llb15, cartpole_QP_solver_slb15, cartpole_QP_solver_rilb15, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z15, cartpole_QP_solver_ubIdx15, params->ub16, cartpole_QP_solver_lub15, cartpole_QP_solver_sub15, cartpole_QP_solver_riub15, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A16, cartpole_QP_solver_z15, params->b16, cartpole_QP_solver_sp15, cartpole_QP_solver_lp15, cartpole_QP_solver_rip15, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb17, cartpole_QP_solver_z16, cartpole_QP_solver_lbIdx16, cartpole_QP_solver_llb16, cartpole_QP_solver_slb16, cartpole_QP_solver_rilb16, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z16, cartpole_QP_solver_ubIdx16, params->ub17, cartpole_QP_solver_lub16, cartpole_QP_solver_sub16, cartpole_QP_solver_riub16, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A17, cartpole_QP_solver_z16, params->b17, cartpole_QP_solver_sp16, cartpole_QP_solver_lp16, cartpole_QP_solver_rip16, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb18, cartpole_QP_solver_z17, cartpole_QP_solver_lbIdx17, cartpole_QP_solver_llb17, cartpole_QP_solver_slb17, cartpole_QP_solver_rilb17, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z17, cartpole_QP_solver_ubIdx17, params->ub18, cartpole_QP_solver_lub17, cartpole_QP_solver_sub17, cartpole_QP_solver_riub17, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_15(params->A18, cartpole_QP_solver_z17, params->b18, cartpole_QP_solver_sp17, cartpole_QP_solver_lp17, cartpole_QP_solver_rip17, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_1(params->lb19, cartpole_QP_solver_z18, cartpole_QP_solver_lbIdx18, cartpole_QP_solver_llb18, cartpole_QP_solver_slb18, cartpole_QP_solver_rilb18, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_1(cartpole_QP_solver_z18, cartpole_QP_solver_ubIdx18, params->ub19, cartpole_QP_solver_lub18, cartpole_QP_solver_sub18, cartpole_QP_solver_riub18, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_MVSUBADD_20_14(params->A19, cartpole_QP_solver_z18, params->b19, cartpole_QP_solver_sp18, cartpole_QP_solver_lp18, cartpole_QP_solver_rip18, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD3_4(params->lb20, cartpole_QP_solver_z19, cartpole_QP_solver_lbIdx19, cartpole_QP_solver_llb19, cartpole_QP_solver_slb19, cartpole_QP_solver_rilb19, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_VSUBADD2_4(cartpole_QP_solver_z19, cartpole_QP_solver_ubIdx19, params->ub20, cartpole_QP_solver_lub19, cartpole_QP_solver_sub19, cartpole_QP_solver_riub19, &info->dgap, &info->res_ineq);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_6_6(cartpole_QP_solver_lub00, cartpole_QP_solver_sub00, cartpole_QP_solver_riub00, cartpole_QP_solver_llb00, cartpole_QP_solver_slb00, cartpole_QP_solver_rilb00, cartpole_QP_solver_lbIdx00, cartpole_QP_solver_ubIdx00, cartpole_QP_solver_grad_ineq00, cartpole_QP_solver_lubbysub00, cartpole_QP_solver_llbbyslb00);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A1, cartpole_QP_solver_lp00, cartpole_QP_solver_sp00, cartpole_QP_solver_rip00, cartpole_QP_solver_grad_ineq00, cartpole_QP_solver_lpbysp00);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_lub01, cartpole_QP_solver_sub01, cartpole_QP_solver_riub01, cartpole_QP_solver_llb01, cartpole_QP_solver_slb01, cartpole_QP_solver_rilb01, cartpole_QP_solver_lbIdx01, cartpole_QP_solver_ubIdx01, cartpole_QP_solver_grad_ineq01, cartpole_QP_solver_lubbysub01, cartpole_QP_solver_llbbyslb01);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A2, cartpole_QP_solver_lp01, cartpole_QP_solver_sp01, cartpole_QP_solver_rip01, cartpole_QP_solver_grad_ineq01, cartpole_QP_solver_lpbysp01);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_lub02, cartpole_QP_solver_sub02, cartpole_QP_solver_riub02, cartpole_QP_solver_llb02, cartpole_QP_solver_slb02, cartpole_QP_solver_rilb02, cartpole_QP_solver_lbIdx02, cartpole_QP_solver_ubIdx02, cartpole_QP_solver_grad_ineq02, cartpole_QP_solver_lubbysub02, cartpole_QP_solver_llbbyslb02);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A3, cartpole_QP_solver_lp02, cartpole_QP_solver_sp02, cartpole_QP_solver_rip02, cartpole_QP_solver_grad_ineq02, cartpole_QP_solver_lpbysp02);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_lub03, cartpole_QP_solver_sub03, cartpole_QP_solver_riub03, cartpole_QP_solver_llb03, cartpole_QP_solver_slb03, cartpole_QP_solver_rilb03, cartpole_QP_solver_lbIdx03, cartpole_QP_solver_ubIdx03, cartpole_QP_solver_grad_ineq03, cartpole_QP_solver_lubbysub03, cartpole_QP_solver_llbbyslb03);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A4, cartpole_QP_solver_lp03, cartpole_QP_solver_sp03, cartpole_QP_solver_rip03, cartpole_QP_solver_grad_ineq03, cartpole_QP_solver_lpbysp03);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_lub04, cartpole_QP_solver_sub04, cartpole_QP_solver_riub04, cartpole_QP_solver_llb04, cartpole_QP_solver_slb04, cartpole_QP_solver_rilb04, cartpole_QP_solver_lbIdx04, cartpole_QP_solver_ubIdx04, cartpole_QP_solver_grad_ineq04, cartpole_QP_solver_lubbysub04, cartpole_QP_solver_llbbyslb04);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A5, cartpole_QP_solver_lp04, cartpole_QP_solver_sp04, cartpole_QP_solver_rip04, cartpole_QP_solver_grad_ineq04, cartpole_QP_solver_lpbysp04);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_lub05, cartpole_QP_solver_sub05, cartpole_QP_solver_riub05, cartpole_QP_solver_llb05, cartpole_QP_solver_slb05, cartpole_QP_solver_rilb05, cartpole_QP_solver_lbIdx05, cartpole_QP_solver_ubIdx05, cartpole_QP_solver_grad_ineq05, cartpole_QP_solver_lubbysub05, cartpole_QP_solver_llbbyslb05);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A6, cartpole_QP_solver_lp05, cartpole_QP_solver_sp05, cartpole_QP_solver_rip05, cartpole_QP_solver_grad_ineq05, cartpole_QP_solver_lpbysp05);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_lub06, cartpole_QP_solver_sub06, cartpole_QP_solver_riub06, cartpole_QP_solver_llb06, cartpole_QP_solver_slb06, cartpole_QP_solver_rilb06, cartpole_QP_solver_lbIdx06, cartpole_QP_solver_ubIdx06, cartpole_QP_solver_grad_ineq06, cartpole_QP_solver_lubbysub06, cartpole_QP_solver_llbbyslb06);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A7, cartpole_QP_solver_lp06, cartpole_QP_solver_sp06, cartpole_QP_solver_rip06, cartpole_QP_solver_grad_ineq06, cartpole_QP_solver_lpbysp06);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_lub07, cartpole_QP_solver_sub07, cartpole_QP_solver_riub07, cartpole_QP_solver_llb07, cartpole_QP_solver_slb07, cartpole_QP_solver_rilb07, cartpole_QP_solver_lbIdx07, cartpole_QP_solver_ubIdx07, cartpole_QP_solver_grad_ineq07, cartpole_QP_solver_lubbysub07, cartpole_QP_solver_llbbyslb07);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A8, cartpole_QP_solver_lp07, cartpole_QP_solver_sp07, cartpole_QP_solver_rip07, cartpole_QP_solver_grad_ineq07, cartpole_QP_solver_lpbysp07);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_lub08, cartpole_QP_solver_sub08, cartpole_QP_solver_riub08, cartpole_QP_solver_llb08, cartpole_QP_solver_slb08, cartpole_QP_solver_rilb08, cartpole_QP_solver_lbIdx08, cartpole_QP_solver_ubIdx08, cartpole_QP_solver_grad_ineq08, cartpole_QP_solver_lubbysub08, cartpole_QP_solver_llbbyslb08);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A9, cartpole_QP_solver_lp08, cartpole_QP_solver_sp08, cartpole_QP_solver_rip08, cartpole_QP_solver_grad_ineq08, cartpole_QP_solver_lpbysp08);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_lub09, cartpole_QP_solver_sub09, cartpole_QP_solver_riub09, cartpole_QP_solver_llb09, cartpole_QP_solver_slb09, cartpole_QP_solver_rilb09, cartpole_QP_solver_lbIdx09, cartpole_QP_solver_ubIdx09, cartpole_QP_solver_grad_ineq09, cartpole_QP_solver_lubbysub09, cartpole_QP_solver_llbbyslb09);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A10, cartpole_QP_solver_lp09, cartpole_QP_solver_sp09, cartpole_QP_solver_rip09, cartpole_QP_solver_grad_ineq09, cartpole_QP_solver_lpbysp09);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_lub10, cartpole_QP_solver_sub10, cartpole_QP_solver_riub10, cartpole_QP_solver_llb10, cartpole_QP_solver_slb10, cartpole_QP_solver_rilb10, cartpole_QP_solver_lbIdx10, cartpole_QP_solver_ubIdx10, cartpole_QP_solver_grad_ineq10, cartpole_QP_solver_lubbysub10, cartpole_QP_solver_llbbyslb10);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A11, cartpole_QP_solver_lp10, cartpole_QP_solver_sp10, cartpole_QP_solver_rip10, cartpole_QP_solver_grad_ineq10, cartpole_QP_solver_lpbysp10);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_lub11, cartpole_QP_solver_sub11, cartpole_QP_solver_riub11, cartpole_QP_solver_llb11, cartpole_QP_solver_slb11, cartpole_QP_solver_rilb11, cartpole_QP_solver_lbIdx11, cartpole_QP_solver_ubIdx11, cartpole_QP_solver_grad_ineq11, cartpole_QP_solver_lubbysub11, cartpole_QP_solver_llbbyslb11);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A12, cartpole_QP_solver_lp11, cartpole_QP_solver_sp11, cartpole_QP_solver_rip11, cartpole_QP_solver_grad_ineq11, cartpole_QP_solver_lpbysp11);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_lub12, cartpole_QP_solver_sub12, cartpole_QP_solver_riub12, cartpole_QP_solver_llb12, cartpole_QP_solver_slb12, cartpole_QP_solver_rilb12, cartpole_QP_solver_lbIdx12, cartpole_QP_solver_ubIdx12, cartpole_QP_solver_grad_ineq12, cartpole_QP_solver_lubbysub12, cartpole_QP_solver_llbbyslb12);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A13, cartpole_QP_solver_lp12, cartpole_QP_solver_sp12, cartpole_QP_solver_rip12, cartpole_QP_solver_grad_ineq12, cartpole_QP_solver_lpbysp12);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_lub13, cartpole_QP_solver_sub13, cartpole_QP_solver_riub13, cartpole_QP_solver_llb13, cartpole_QP_solver_slb13, cartpole_QP_solver_rilb13, cartpole_QP_solver_lbIdx13, cartpole_QP_solver_ubIdx13, cartpole_QP_solver_grad_ineq13, cartpole_QP_solver_lubbysub13, cartpole_QP_solver_llbbyslb13);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A14, cartpole_QP_solver_lp13, cartpole_QP_solver_sp13, cartpole_QP_solver_rip13, cartpole_QP_solver_grad_ineq13, cartpole_QP_solver_lpbysp13);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_lub14, cartpole_QP_solver_sub14, cartpole_QP_solver_riub14, cartpole_QP_solver_llb14, cartpole_QP_solver_slb14, cartpole_QP_solver_rilb14, cartpole_QP_solver_lbIdx14, cartpole_QP_solver_ubIdx14, cartpole_QP_solver_grad_ineq14, cartpole_QP_solver_lubbysub14, cartpole_QP_solver_llbbyslb14);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A15, cartpole_QP_solver_lp14, cartpole_QP_solver_sp14, cartpole_QP_solver_rip14, cartpole_QP_solver_grad_ineq14, cartpole_QP_solver_lpbysp14);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_lub15, cartpole_QP_solver_sub15, cartpole_QP_solver_riub15, cartpole_QP_solver_llb15, cartpole_QP_solver_slb15, cartpole_QP_solver_rilb15, cartpole_QP_solver_lbIdx15, cartpole_QP_solver_ubIdx15, cartpole_QP_solver_grad_ineq15, cartpole_QP_solver_lubbysub15, cartpole_QP_solver_llbbyslb15);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A16, cartpole_QP_solver_lp15, cartpole_QP_solver_sp15, cartpole_QP_solver_rip15, cartpole_QP_solver_grad_ineq15, cartpole_QP_solver_lpbysp15);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_lub16, cartpole_QP_solver_sub16, cartpole_QP_solver_riub16, cartpole_QP_solver_llb16, cartpole_QP_solver_slb16, cartpole_QP_solver_rilb16, cartpole_QP_solver_lbIdx16, cartpole_QP_solver_ubIdx16, cartpole_QP_solver_grad_ineq16, cartpole_QP_solver_lubbysub16, cartpole_QP_solver_llbbyslb16);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A17, cartpole_QP_solver_lp16, cartpole_QP_solver_sp16, cartpole_QP_solver_rip16, cartpole_QP_solver_grad_ineq16, cartpole_QP_solver_lpbysp16);
cartpole_QP_solver_LA_INEQ_B_GRAD_15_1_1(cartpole_QP_solver_lub17, cartpole_QP_solver_sub17, cartpole_QP_solver_riub17, cartpole_QP_solver_llb17, cartpole_QP_solver_slb17, cartpole_QP_solver_rilb17, cartpole_QP_solver_lbIdx17, cartpole_QP_solver_ubIdx17, cartpole_QP_solver_grad_ineq17, cartpole_QP_solver_lubbysub17, cartpole_QP_solver_llbbyslb17);
cartpole_QP_solver_LA_INEQ_P_20_15(params->A18, cartpole_QP_solver_lp17, cartpole_QP_solver_sp17, cartpole_QP_solver_rip17, cartpole_QP_solver_grad_ineq17, cartpole_QP_solver_lpbysp17);
cartpole_QP_solver_LA_INEQ_B_GRAD_14_1_1(cartpole_QP_solver_lub18, cartpole_QP_solver_sub18, cartpole_QP_solver_riub18, cartpole_QP_solver_llb18, cartpole_QP_solver_slb18, cartpole_QP_solver_rilb18, cartpole_QP_solver_lbIdx18, cartpole_QP_solver_ubIdx18, cartpole_QP_solver_grad_ineq18, cartpole_QP_solver_lubbysub18, cartpole_QP_solver_llbbyslb18);
cartpole_QP_solver_LA_INEQ_P_20_14(params->A19, cartpole_QP_solver_lp18, cartpole_QP_solver_sp18, cartpole_QP_solver_rip18, cartpole_QP_solver_grad_ineq18, cartpole_QP_solver_lpbysp18);
cartpole_QP_solver_LA_INEQ_B_GRAD_4_4_4(cartpole_QP_solver_lub19, cartpole_QP_solver_sub19, cartpole_QP_solver_riub19, cartpole_QP_solver_llb19, cartpole_QP_solver_slb19, cartpole_QP_solver_rilb19, cartpole_QP_solver_lbIdx19, cartpole_QP_solver_ubIdx19, cartpole_QP_solver_grad_ineq19, cartpole_QP_solver_lubbysub19, cartpole_QP_solver_llbbyslb19);
info->dobj = info->pobj - info->dgap;
info->rdgap = info->pobj ? info->dgap / info->pobj : 1e6;
if( info->rdgap < 0 ) info->rdgap = -info->rdgap;
PRINTTEXT("  %3d  %3.1e  %3.1e  %+6.4e  %+6.4e  %+3.1e  %3.1e  %3.1e\n",info->it, info->res_eq, info->res_ineq, info->pobj, info->dobj, info->dgap, info->rdgap, info->mu);
if( info->mu < cartpole_QP_solver_SET_ACC_KKTCOMPL
    && (info->rdgap < cartpole_QP_solver_SET_ACC_RDGAP || info->dgap < cartpole_QP_solver_SET_ACC_KKTCOMPL)
    && info->res_eq < cartpole_QP_solver_SET_ACC_RESEQ
    && info->res_ineq < cartpole_QP_solver_SET_ACC_RESINEQ ){
PRINTTEXT("OPTIMAL (within RESEQ=%2.1e, RESINEQ=%2.1e, (R)DGAP=(%2.1e)%2.1e, MU=%2.1e).\n",cartpole_QP_solver_SET_ACC_RESEQ, cartpole_QP_solver_SET_ACC_RESINEQ,cartpole_QP_solver_SET_ACC_KKTCOMPL,cartpole_QP_solver_SET_ACC_RDGAP,cartpole_QP_solver_SET_ACC_KKTCOMPL);
exitcode = cartpole_QP_solver_OPTIMAL; break; }
if( info->it == cartpole_QP_solver_SET_MAXIT ){
PRINTTEXT("Maximum number of iterations reached, exiting.\n");
exitcode = cartpole_QP_solver_MAXITREACHED; break; }
cartpole_QP_solver_LA_VVADD3_288(cartpole_QP_solver_grad_cost, cartpole_QP_solver_grad_eq, cartpole_QP_solver_grad_ineq, cartpole_QP_solver_rd);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_6_6(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb00, cartpole_QP_solver_lbIdx00, cartpole_QP_solver_lubbysub00, cartpole_QP_solver_ubIdx00, cartpole_QP_solver_Phi00);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A1, cartpole_QP_solver_lpbysp00, cartpole_QP_solver_Phi00);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi00);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi00, cartpole_QP_solver_C00, cartpole_QP_solver_V00);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi00, cartpole_QP_solver_rd00, cartpole_QP_solver_Lbyrd00);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb01, cartpole_QP_solver_lbIdx01, cartpole_QP_solver_lubbysub01, cartpole_QP_solver_ubIdx01, cartpole_QP_solver_Phi01);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A2, cartpole_QP_solver_lpbysp01, cartpole_QP_solver_Phi01);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi01);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi01, cartpole_QP_solver_C00, cartpole_QP_solver_V01);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi01, cartpole_QP_solver_D01, cartpole_QP_solver_W01);
cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_W01, cartpole_QP_solver_V01, cartpole_QP_solver_Ysd01);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi01, cartpole_QP_solver_rd01, cartpole_QP_solver_Lbyrd01);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb02, cartpole_QP_solver_lbIdx02, cartpole_QP_solver_lubbysub02, cartpole_QP_solver_ubIdx02, cartpole_QP_solver_Phi02);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A3, cartpole_QP_solver_lpbysp02, cartpole_QP_solver_Phi02);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi02);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi02, cartpole_QP_solver_C00, cartpole_QP_solver_V02);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi02, cartpole_QP_solver_D01, cartpole_QP_solver_W02);
cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_W02, cartpole_QP_solver_V02, cartpole_QP_solver_Ysd02);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi02, cartpole_QP_solver_rd02, cartpole_QP_solver_Lbyrd02);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb03, cartpole_QP_solver_lbIdx03, cartpole_QP_solver_lubbysub03, cartpole_QP_solver_ubIdx03, cartpole_QP_solver_Phi03);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A4, cartpole_QP_solver_lpbysp03, cartpole_QP_solver_Phi03);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi03);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi03, cartpole_QP_solver_C00, cartpole_QP_solver_V03);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi03, cartpole_QP_solver_D01, cartpole_QP_solver_W03);
cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_W03, cartpole_QP_solver_V03, cartpole_QP_solver_Ysd03);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi03, cartpole_QP_solver_rd03, cartpole_QP_solver_Lbyrd03);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb04, cartpole_QP_solver_lbIdx04, cartpole_QP_solver_lubbysub04, cartpole_QP_solver_ubIdx04, cartpole_QP_solver_Phi04);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A5, cartpole_QP_solver_lpbysp04, cartpole_QP_solver_Phi04);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi04);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi04, cartpole_QP_solver_C00, cartpole_QP_solver_V04);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi04, cartpole_QP_solver_D01, cartpole_QP_solver_W04);
cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_W04, cartpole_QP_solver_V04, cartpole_QP_solver_Ysd04);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi04, cartpole_QP_solver_rd04, cartpole_QP_solver_Lbyrd04);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb05, cartpole_QP_solver_lbIdx05, cartpole_QP_solver_lubbysub05, cartpole_QP_solver_ubIdx05, cartpole_QP_solver_Phi05);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A6, cartpole_QP_solver_lpbysp05, cartpole_QP_solver_Phi05);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi05);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi05, cartpole_QP_solver_C00, cartpole_QP_solver_V05);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi05, cartpole_QP_solver_D01, cartpole_QP_solver_W05);
cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_W05, cartpole_QP_solver_V05, cartpole_QP_solver_Ysd05);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi05, cartpole_QP_solver_rd05, cartpole_QP_solver_Lbyrd05);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb06, cartpole_QP_solver_lbIdx06, cartpole_QP_solver_lubbysub06, cartpole_QP_solver_ubIdx06, cartpole_QP_solver_Phi06);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A7, cartpole_QP_solver_lpbysp06, cartpole_QP_solver_Phi06);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi06);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi06, cartpole_QP_solver_C00, cartpole_QP_solver_V06);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi06, cartpole_QP_solver_D01, cartpole_QP_solver_W06);
cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_W06, cartpole_QP_solver_V06, cartpole_QP_solver_Ysd06);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi06, cartpole_QP_solver_rd06, cartpole_QP_solver_Lbyrd06);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb07, cartpole_QP_solver_lbIdx07, cartpole_QP_solver_lubbysub07, cartpole_QP_solver_ubIdx07, cartpole_QP_solver_Phi07);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A8, cartpole_QP_solver_lpbysp07, cartpole_QP_solver_Phi07);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi07);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi07, cartpole_QP_solver_C00, cartpole_QP_solver_V07);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi07, cartpole_QP_solver_D01, cartpole_QP_solver_W07);
cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_W07, cartpole_QP_solver_V07, cartpole_QP_solver_Ysd07);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi07, cartpole_QP_solver_rd07, cartpole_QP_solver_Lbyrd07);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb08, cartpole_QP_solver_lbIdx08, cartpole_QP_solver_lubbysub08, cartpole_QP_solver_ubIdx08, cartpole_QP_solver_Phi08);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A9, cartpole_QP_solver_lpbysp08, cartpole_QP_solver_Phi08);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi08);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi08, cartpole_QP_solver_C00, cartpole_QP_solver_V08);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi08, cartpole_QP_solver_D01, cartpole_QP_solver_W08);
cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_W08, cartpole_QP_solver_V08, cartpole_QP_solver_Ysd08);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi08, cartpole_QP_solver_rd08, cartpole_QP_solver_Lbyrd08);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb09, cartpole_QP_solver_lbIdx09, cartpole_QP_solver_lubbysub09, cartpole_QP_solver_ubIdx09, cartpole_QP_solver_Phi09);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A10, cartpole_QP_solver_lpbysp09, cartpole_QP_solver_Phi09);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi09);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi09, cartpole_QP_solver_C00, cartpole_QP_solver_V09);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi09, cartpole_QP_solver_D01, cartpole_QP_solver_W09);
cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_W09, cartpole_QP_solver_V09, cartpole_QP_solver_Ysd09);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi09, cartpole_QP_solver_rd09, cartpole_QP_solver_Lbyrd09);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb10, cartpole_QP_solver_lbIdx10, cartpole_QP_solver_lubbysub10, cartpole_QP_solver_ubIdx10, cartpole_QP_solver_Phi10);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A11, cartpole_QP_solver_lpbysp10, cartpole_QP_solver_Phi10);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi10);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi10, cartpole_QP_solver_C00, cartpole_QP_solver_V10);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi10, cartpole_QP_solver_D01, cartpole_QP_solver_W10);
cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_W10, cartpole_QP_solver_V10, cartpole_QP_solver_Ysd10);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi10, cartpole_QP_solver_rd10, cartpole_QP_solver_Lbyrd10);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb11, cartpole_QP_solver_lbIdx11, cartpole_QP_solver_lubbysub11, cartpole_QP_solver_ubIdx11, cartpole_QP_solver_Phi11);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A12, cartpole_QP_solver_lpbysp11, cartpole_QP_solver_Phi11);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi11);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi11, cartpole_QP_solver_C00, cartpole_QP_solver_V11);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi11, cartpole_QP_solver_D01, cartpole_QP_solver_W11);
cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_W11, cartpole_QP_solver_V11, cartpole_QP_solver_Ysd11);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi11, cartpole_QP_solver_rd11, cartpole_QP_solver_Lbyrd11);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb12, cartpole_QP_solver_lbIdx12, cartpole_QP_solver_lubbysub12, cartpole_QP_solver_ubIdx12, cartpole_QP_solver_Phi12);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A13, cartpole_QP_solver_lpbysp12, cartpole_QP_solver_Phi12);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi12);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi12, cartpole_QP_solver_C00, cartpole_QP_solver_V12);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi12, cartpole_QP_solver_D01, cartpole_QP_solver_W12);
cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_W12, cartpole_QP_solver_V12, cartpole_QP_solver_Ysd12);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi12, cartpole_QP_solver_rd12, cartpole_QP_solver_Lbyrd12);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb13, cartpole_QP_solver_lbIdx13, cartpole_QP_solver_lubbysub13, cartpole_QP_solver_ubIdx13, cartpole_QP_solver_Phi13);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A14, cartpole_QP_solver_lpbysp13, cartpole_QP_solver_Phi13);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi13);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi13, cartpole_QP_solver_C00, cartpole_QP_solver_V13);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi13, cartpole_QP_solver_D01, cartpole_QP_solver_W13);
cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_W13, cartpole_QP_solver_V13, cartpole_QP_solver_Ysd13);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi13, cartpole_QP_solver_rd13, cartpole_QP_solver_Lbyrd13);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb14, cartpole_QP_solver_lbIdx14, cartpole_QP_solver_lubbysub14, cartpole_QP_solver_ubIdx14, cartpole_QP_solver_Phi14);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A15, cartpole_QP_solver_lpbysp14, cartpole_QP_solver_Phi14);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi14);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi14, cartpole_QP_solver_C00, cartpole_QP_solver_V14);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi14, cartpole_QP_solver_D01, cartpole_QP_solver_W14);
cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_W14, cartpole_QP_solver_V14, cartpole_QP_solver_Ysd14);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi14, cartpole_QP_solver_rd14, cartpole_QP_solver_Lbyrd14);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb15, cartpole_QP_solver_lbIdx15, cartpole_QP_solver_lubbysub15, cartpole_QP_solver_ubIdx15, cartpole_QP_solver_Phi15);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A16, cartpole_QP_solver_lpbysp15, cartpole_QP_solver_Phi15);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi15);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi15, cartpole_QP_solver_C00, cartpole_QP_solver_V15);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi15, cartpole_QP_solver_D01, cartpole_QP_solver_W15);
cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_W15, cartpole_QP_solver_V15, cartpole_QP_solver_Ysd15);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi15, cartpole_QP_solver_rd15, cartpole_QP_solver_Lbyrd15);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb16, cartpole_QP_solver_lbIdx16, cartpole_QP_solver_lubbysub16, cartpole_QP_solver_ubIdx16, cartpole_QP_solver_Phi16);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A17, cartpole_QP_solver_lpbysp16, cartpole_QP_solver_Phi16);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi16);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi16, cartpole_QP_solver_C00, cartpole_QP_solver_V16);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi16, cartpole_QP_solver_D01, cartpole_QP_solver_W16);
cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_W16, cartpole_QP_solver_V16, cartpole_QP_solver_Ysd16);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi16, cartpole_QP_solver_rd16, cartpole_QP_solver_Lbyrd16);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_1_1(cartpole_QP_solver_H00, cartpole_QP_solver_llbbyslb17, cartpole_QP_solver_lbIdx17, cartpole_QP_solver_lubbysub17, cartpole_QP_solver_ubIdx17, cartpole_QP_solver_Phi17);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_15(params->A18, cartpole_QP_solver_lpbysp17, cartpole_QP_solver_Phi17);
cartpole_QP_solver_LA_DENSE_CHOL2_15(cartpole_QP_solver_Phi17);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi17, cartpole_QP_solver_C00, cartpole_QP_solver_V17);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_15(cartpole_QP_solver_Phi17, cartpole_QP_solver_D01, cartpole_QP_solver_W17);
cartpole_QP_solver_LA_DENSE_MMTM_6_15_6(cartpole_QP_solver_W17, cartpole_QP_solver_V17, cartpole_QP_solver_Ysd17);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi17, cartpole_QP_solver_rd17, cartpole_QP_solver_Lbyrd17);
cartpole_QP_solver_LA_INEQ_DENSE_DIAG_HESS_14_1_1(cartpole_QP_solver_H18, cartpole_QP_solver_llbbyslb18, cartpole_QP_solver_lbIdx18, cartpole_QP_solver_lubbysub18, cartpole_QP_solver_ubIdx18, cartpole_QP_solver_Phi18);
cartpole_QP_solver_LA_DENSE_ADDMTDM_20_14(params->A19, cartpole_QP_solver_lpbysp18, cartpole_QP_solver_Phi18);
cartpole_QP_solver_LA_DENSE_CHOL2_14(cartpole_QP_solver_Phi18);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_4_14(cartpole_QP_solver_Phi18, cartpole_QP_solver_C18, cartpole_QP_solver_V18);
cartpole_QP_solver_LA_DENSE_MATRIXFORWARDSUB_6_14(cartpole_QP_solver_Phi18, cartpole_QP_solver_D18, cartpole_QP_solver_W18);
cartpole_QP_solver_LA_DENSE_MMTM_6_14_4(cartpole_QP_solver_W18, cartpole_QP_solver_V18, cartpole_QP_solver_Ysd18);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_14(cartpole_QP_solver_Phi18, cartpole_QP_solver_rd18, cartpole_QP_solver_Lbyrd18);
cartpole_QP_solver_LA_DIAG_CHOL_ONELOOP_LBUB_4_4_4(cartpole_QP_solver_H19, cartpole_QP_solver_llbbyslb19, cartpole_QP_solver_lbIdx19, cartpole_QP_solver_lubbysub19, cartpole_QP_solver_ubIdx19, cartpole_QP_solver_Phi19);
cartpole_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_4_4(cartpole_QP_solver_Phi19, cartpole_QP_solver_D19, cartpole_QP_solver_W19);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_4(cartpole_QP_solver_Phi19, cartpole_QP_solver_rd19, cartpole_QP_solver_Lbyrd19);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_V00, cartpole_QP_solver_W01, cartpole_QP_solver_Yd00);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_V00, cartpole_QP_solver_Lbyrd00, cartpole_QP_solver_W01, cartpole_QP_solver_Lbyrd01, cartpole_QP_solver_re00, cartpole_QP_solver_beta00);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_V01, cartpole_QP_solver_W02, cartpole_QP_solver_Yd01);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_V01, cartpole_QP_solver_Lbyrd01, cartpole_QP_solver_W02, cartpole_QP_solver_Lbyrd02, cartpole_QP_solver_re01, cartpole_QP_solver_beta01);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_V02, cartpole_QP_solver_W03, cartpole_QP_solver_Yd02);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_V02, cartpole_QP_solver_Lbyrd02, cartpole_QP_solver_W03, cartpole_QP_solver_Lbyrd03, cartpole_QP_solver_re02, cartpole_QP_solver_beta02);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_V03, cartpole_QP_solver_W04, cartpole_QP_solver_Yd03);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_V03, cartpole_QP_solver_Lbyrd03, cartpole_QP_solver_W04, cartpole_QP_solver_Lbyrd04, cartpole_QP_solver_re03, cartpole_QP_solver_beta03);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_V04, cartpole_QP_solver_W05, cartpole_QP_solver_Yd04);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_V04, cartpole_QP_solver_Lbyrd04, cartpole_QP_solver_W05, cartpole_QP_solver_Lbyrd05, cartpole_QP_solver_re04, cartpole_QP_solver_beta04);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_V05, cartpole_QP_solver_W06, cartpole_QP_solver_Yd05);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_V05, cartpole_QP_solver_Lbyrd05, cartpole_QP_solver_W06, cartpole_QP_solver_Lbyrd06, cartpole_QP_solver_re05, cartpole_QP_solver_beta05);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_V06, cartpole_QP_solver_W07, cartpole_QP_solver_Yd06);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_V06, cartpole_QP_solver_Lbyrd06, cartpole_QP_solver_W07, cartpole_QP_solver_Lbyrd07, cartpole_QP_solver_re06, cartpole_QP_solver_beta06);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_V07, cartpole_QP_solver_W08, cartpole_QP_solver_Yd07);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_V07, cartpole_QP_solver_Lbyrd07, cartpole_QP_solver_W08, cartpole_QP_solver_Lbyrd08, cartpole_QP_solver_re07, cartpole_QP_solver_beta07);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_V08, cartpole_QP_solver_W09, cartpole_QP_solver_Yd08);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_V08, cartpole_QP_solver_Lbyrd08, cartpole_QP_solver_W09, cartpole_QP_solver_Lbyrd09, cartpole_QP_solver_re08, cartpole_QP_solver_beta08);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_V09, cartpole_QP_solver_W10, cartpole_QP_solver_Yd09);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_V09, cartpole_QP_solver_Lbyrd09, cartpole_QP_solver_W10, cartpole_QP_solver_Lbyrd10, cartpole_QP_solver_re09, cartpole_QP_solver_beta09);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_V10, cartpole_QP_solver_W11, cartpole_QP_solver_Yd10);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_V10, cartpole_QP_solver_Lbyrd10, cartpole_QP_solver_W11, cartpole_QP_solver_Lbyrd11, cartpole_QP_solver_re10, cartpole_QP_solver_beta10);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_V11, cartpole_QP_solver_W12, cartpole_QP_solver_Yd11);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_V11, cartpole_QP_solver_Lbyrd11, cartpole_QP_solver_W12, cartpole_QP_solver_Lbyrd12, cartpole_QP_solver_re11, cartpole_QP_solver_beta11);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_V12, cartpole_QP_solver_W13, cartpole_QP_solver_Yd12);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_V12, cartpole_QP_solver_Lbyrd12, cartpole_QP_solver_W13, cartpole_QP_solver_Lbyrd13, cartpole_QP_solver_re12, cartpole_QP_solver_beta12);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_V13, cartpole_QP_solver_W14, cartpole_QP_solver_Yd13);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_V13, cartpole_QP_solver_Lbyrd13, cartpole_QP_solver_W14, cartpole_QP_solver_Lbyrd14, cartpole_QP_solver_re13, cartpole_QP_solver_beta13);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_V14, cartpole_QP_solver_W15, cartpole_QP_solver_Yd14);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_V14, cartpole_QP_solver_Lbyrd14, cartpole_QP_solver_W15, cartpole_QP_solver_Lbyrd15, cartpole_QP_solver_re14, cartpole_QP_solver_beta14);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_V15, cartpole_QP_solver_W16, cartpole_QP_solver_Yd15);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_V15, cartpole_QP_solver_Lbyrd15, cartpole_QP_solver_W16, cartpole_QP_solver_Lbyrd16, cartpole_QP_solver_re15, cartpole_QP_solver_beta15);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_15(cartpole_QP_solver_V16, cartpole_QP_solver_W17, cartpole_QP_solver_Yd16);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_15(cartpole_QP_solver_V16, cartpole_QP_solver_Lbyrd16, cartpole_QP_solver_W17, cartpole_QP_solver_Lbyrd17, cartpole_QP_solver_re16, cartpole_QP_solver_beta16);
cartpole_QP_solver_LA_DENSE_MMT2_6_15_14(cartpole_QP_solver_V17, cartpole_QP_solver_W18, cartpole_QP_solver_Yd17);
cartpole_QP_solver_LA_DENSE_MVMSUB2_6_15_14(cartpole_QP_solver_V17, cartpole_QP_solver_Lbyrd17, cartpole_QP_solver_W18, cartpole_QP_solver_Lbyrd18, cartpole_QP_solver_re17, cartpole_QP_solver_beta17);
cartpole_QP_solver_LA_DENSE_DIAGZERO_MMT2_4_14_4(cartpole_QP_solver_V18, cartpole_QP_solver_W19, cartpole_QP_solver_Yd18);
cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_4_14_4(cartpole_QP_solver_V18, cartpole_QP_solver_Lbyrd18, cartpole_QP_solver_W19, cartpole_QP_solver_Lbyrd19, cartpole_QP_solver_re18, cartpole_QP_solver_beta18);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd00, cartpole_QP_solver_Ld00);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld00, cartpole_QP_solver_beta00, cartpole_QP_solver_yy00);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_Ld00, cartpole_QP_solver_Ysd01, cartpole_QP_solver_Lsd01);
cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_Lsd01, cartpole_QP_solver_Yd01);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd01, cartpole_QP_solver_Ld01);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd01, cartpole_QP_solver_yy00, cartpole_QP_solver_beta01, cartpole_QP_solver_bmy01);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld01, cartpole_QP_solver_bmy01, cartpole_QP_solver_yy01);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_Ld01, cartpole_QP_solver_Ysd02, cartpole_QP_solver_Lsd02);
cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_Lsd02, cartpole_QP_solver_Yd02);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd02, cartpole_QP_solver_Ld02);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd02, cartpole_QP_solver_yy01, cartpole_QP_solver_beta02, cartpole_QP_solver_bmy02);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld02, cartpole_QP_solver_bmy02, cartpole_QP_solver_yy02);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_Ld02, cartpole_QP_solver_Ysd03, cartpole_QP_solver_Lsd03);
cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_Lsd03, cartpole_QP_solver_Yd03);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd03, cartpole_QP_solver_Ld03);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd03, cartpole_QP_solver_yy02, cartpole_QP_solver_beta03, cartpole_QP_solver_bmy03);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld03, cartpole_QP_solver_bmy03, cartpole_QP_solver_yy03);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_Ld03, cartpole_QP_solver_Ysd04, cartpole_QP_solver_Lsd04);
cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_Lsd04, cartpole_QP_solver_Yd04);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd04, cartpole_QP_solver_Ld04);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd04, cartpole_QP_solver_yy03, cartpole_QP_solver_beta04, cartpole_QP_solver_bmy04);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld04, cartpole_QP_solver_bmy04, cartpole_QP_solver_yy04);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_Ld04, cartpole_QP_solver_Ysd05, cartpole_QP_solver_Lsd05);
cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_Lsd05, cartpole_QP_solver_Yd05);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd05, cartpole_QP_solver_Ld05);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd05, cartpole_QP_solver_yy04, cartpole_QP_solver_beta05, cartpole_QP_solver_bmy05);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld05, cartpole_QP_solver_bmy05, cartpole_QP_solver_yy05);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_Ld05, cartpole_QP_solver_Ysd06, cartpole_QP_solver_Lsd06);
cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_Lsd06, cartpole_QP_solver_Yd06);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd06, cartpole_QP_solver_Ld06);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd06, cartpole_QP_solver_yy05, cartpole_QP_solver_beta06, cartpole_QP_solver_bmy06);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld06, cartpole_QP_solver_bmy06, cartpole_QP_solver_yy06);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_Ld06, cartpole_QP_solver_Ysd07, cartpole_QP_solver_Lsd07);
cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_Lsd07, cartpole_QP_solver_Yd07);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd07, cartpole_QP_solver_Ld07);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd07, cartpole_QP_solver_yy06, cartpole_QP_solver_beta07, cartpole_QP_solver_bmy07);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld07, cartpole_QP_solver_bmy07, cartpole_QP_solver_yy07);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_Ld07, cartpole_QP_solver_Ysd08, cartpole_QP_solver_Lsd08);
cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_Lsd08, cartpole_QP_solver_Yd08);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd08, cartpole_QP_solver_Ld08);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd08, cartpole_QP_solver_yy07, cartpole_QP_solver_beta08, cartpole_QP_solver_bmy08);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld08, cartpole_QP_solver_bmy08, cartpole_QP_solver_yy08);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_Ld08, cartpole_QP_solver_Ysd09, cartpole_QP_solver_Lsd09);
cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_Lsd09, cartpole_QP_solver_Yd09);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd09, cartpole_QP_solver_Ld09);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd09, cartpole_QP_solver_yy08, cartpole_QP_solver_beta09, cartpole_QP_solver_bmy09);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld09, cartpole_QP_solver_bmy09, cartpole_QP_solver_yy09);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_Ld09, cartpole_QP_solver_Ysd10, cartpole_QP_solver_Lsd10);
cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_Lsd10, cartpole_QP_solver_Yd10);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd10, cartpole_QP_solver_Ld10);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd10, cartpole_QP_solver_yy09, cartpole_QP_solver_beta10, cartpole_QP_solver_bmy10);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld10, cartpole_QP_solver_bmy10, cartpole_QP_solver_yy10);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_Ld10, cartpole_QP_solver_Ysd11, cartpole_QP_solver_Lsd11);
cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_Lsd11, cartpole_QP_solver_Yd11);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd11, cartpole_QP_solver_Ld11);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd11, cartpole_QP_solver_yy10, cartpole_QP_solver_beta11, cartpole_QP_solver_bmy11);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld11, cartpole_QP_solver_bmy11, cartpole_QP_solver_yy11);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_Ld11, cartpole_QP_solver_Ysd12, cartpole_QP_solver_Lsd12);
cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_Lsd12, cartpole_QP_solver_Yd12);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd12, cartpole_QP_solver_Ld12);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd12, cartpole_QP_solver_yy11, cartpole_QP_solver_beta12, cartpole_QP_solver_bmy12);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld12, cartpole_QP_solver_bmy12, cartpole_QP_solver_yy12);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_Ld12, cartpole_QP_solver_Ysd13, cartpole_QP_solver_Lsd13);
cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_Lsd13, cartpole_QP_solver_Yd13);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd13, cartpole_QP_solver_Ld13);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd13, cartpole_QP_solver_yy12, cartpole_QP_solver_beta13, cartpole_QP_solver_bmy13);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld13, cartpole_QP_solver_bmy13, cartpole_QP_solver_yy13);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_Ld13, cartpole_QP_solver_Ysd14, cartpole_QP_solver_Lsd14);
cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_Lsd14, cartpole_QP_solver_Yd14);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd14, cartpole_QP_solver_Ld14);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd14, cartpole_QP_solver_yy13, cartpole_QP_solver_beta14, cartpole_QP_solver_bmy14);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld14, cartpole_QP_solver_bmy14, cartpole_QP_solver_yy14);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_Ld14, cartpole_QP_solver_Ysd15, cartpole_QP_solver_Lsd15);
cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_Lsd15, cartpole_QP_solver_Yd15);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd15, cartpole_QP_solver_Ld15);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd15, cartpole_QP_solver_yy14, cartpole_QP_solver_beta15, cartpole_QP_solver_bmy15);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld15, cartpole_QP_solver_bmy15, cartpole_QP_solver_yy15);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_Ld15, cartpole_QP_solver_Ysd16, cartpole_QP_solver_Lsd16);
cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_Lsd16, cartpole_QP_solver_Yd16);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd16, cartpole_QP_solver_Ld16);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd16, cartpole_QP_solver_yy15, cartpole_QP_solver_beta16, cartpole_QP_solver_bmy16);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld16, cartpole_QP_solver_bmy16, cartpole_QP_solver_yy16);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_6_6(cartpole_QP_solver_Ld16, cartpole_QP_solver_Ysd17, cartpole_QP_solver_Lsd17);
cartpole_QP_solver_LA_DENSE_MMTSUB_6_6(cartpole_QP_solver_Lsd17, cartpole_QP_solver_Yd17);
cartpole_QP_solver_LA_DENSE_CHOL_6(cartpole_QP_solver_Yd17, cartpole_QP_solver_Ld17);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd17, cartpole_QP_solver_yy16, cartpole_QP_solver_beta17, cartpole_QP_solver_bmy17);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld17, cartpole_QP_solver_bmy17, cartpole_QP_solver_yy17);
cartpole_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_4_6(cartpole_QP_solver_Ld17, cartpole_QP_solver_Ysd18, cartpole_QP_solver_Lsd18);
cartpole_QP_solver_LA_DENSE_MMTSUB_4_6(cartpole_QP_solver_Lsd18, cartpole_QP_solver_Yd18);
cartpole_QP_solver_LA_DENSE_CHOL_4(cartpole_QP_solver_Yd18, cartpole_QP_solver_Ld18);
cartpole_QP_solver_LA_DENSE_MVMSUB1_4_6(cartpole_QP_solver_Lsd18, cartpole_QP_solver_yy17, cartpole_QP_solver_beta18, cartpole_QP_solver_bmy18);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_4(cartpole_QP_solver_Ld18, cartpole_QP_solver_bmy18, cartpole_QP_solver_yy18);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_4(cartpole_QP_solver_Ld18, cartpole_QP_solver_yy18, cartpole_QP_solver_dvaff18);
cartpole_QP_solver_LA_DENSE_MTVMSUB_4_6(cartpole_QP_solver_Lsd18, cartpole_QP_solver_dvaff18, cartpole_QP_solver_yy17, cartpole_QP_solver_bmy17);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld17, cartpole_QP_solver_bmy17, cartpole_QP_solver_dvaff17);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd17, cartpole_QP_solver_dvaff17, cartpole_QP_solver_yy16, cartpole_QP_solver_bmy16);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld16, cartpole_QP_solver_bmy16, cartpole_QP_solver_dvaff16);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd16, cartpole_QP_solver_dvaff16, cartpole_QP_solver_yy15, cartpole_QP_solver_bmy15);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld15, cartpole_QP_solver_bmy15, cartpole_QP_solver_dvaff15);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd15, cartpole_QP_solver_dvaff15, cartpole_QP_solver_yy14, cartpole_QP_solver_bmy14);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld14, cartpole_QP_solver_bmy14, cartpole_QP_solver_dvaff14);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd14, cartpole_QP_solver_dvaff14, cartpole_QP_solver_yy13, cartpole_QP_solver_bmy13);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld13, cartpole_QP_solver_bmy13, cartpole_QP_solver_dvaff13);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd13, cartpole_QP_solver_dvaff13, cartpole_QP_solver_yy12, cartpole_QP_solver_bmy12);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld12, cartpole_QP_solver_bmy12, cartpole_QP_solver_dvaff12);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd12, cartpole_QP_solver_dvaff12, cartpole_QP_solver_yy11, cartpole_QP_solver_bmy11);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld11, cartpole_QP_solver_bmy11, cartpole_QP_solver_dvaff11);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd11, cartpole_QP_solver_dvaff11, cartpole_QP_solver_yy10, cartpole_QP_solver_bmy10);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld10, cartpole_QP_solver_bmy10, cartpole_QP_solver_dvaff10);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd10, cartpole_QP_solver_dvaff10, cartpole_QP_solver_yy09, cartpole_QP_solver_bmy09);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld09, cartpole_QP_solver_bmy09, cartpole_QP_solver_dvaff09);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd09, cartpole_QP_solver_dvaff09, cartpole_QP_solver_yy08, cartpole_QP_solver_bmy08);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld08, cartpole_QP_solver_bmy08, cartpole_QP_solver_dvaff08);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd08, cartpole_QP_solver_dvaff08, cartpole_QP_solver_yy07, cartpole_QP_solver_bmy07);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld07, cartpole_QP_solver_bmy07, cartpole_QP_solver_dvaff07);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd07, cartpole_QP_solver_dvaff07, cartpole_QP_solver_yy06, cartpole_QP_solver_bmy06);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld06, cartpole_QP_solver_bmy06, cartpole_QP_solver_dvaff06);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd06, cartpole_QP_solver_dvaff06, cartpole_QP_solver_yy05, cartpole_QP_solver_bmy05);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld05, cartpole_QP_solver_bmy05, cartpole_QP_solver_dvaff05);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd05, cartpole_QP_solver_dvaff05, cartpole_QP_solver_yy04, cartpole_QP_solver_bmy04);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld04, cartpole_QP_solver_bmy04, cartpole_QP_solver_dvaff04);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd04, cartpole_QP_solver_dvaff04, cartpole_QP_solver_yy03, cartpole_QP_solver_bmy03);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld03, cartpole_QP_solver_bmy03, cartpole_QP_solver_dvaff03);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd03, cartpole_QP_solver_dvaff03, cartpole_QP_solver_yy02, cartpole_QP_solver_bmy02);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld02, cartpole_QP_solver_bmy02, cartpole_QP_solver_dvaff02);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd02, cartpole_QP_solver_dvaff02, cartpole_QP_solver_yy01, cartpole_QP_solver_bmy01);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld01, cartpole_QP_solver_bmy01, cartpole_QP_solver_dvaff01);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd01, cartpole_QP_solver_dvaff01, cartpole_QP_solver_yy00, cartpole_QP_solver_bmy00);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld00, cartpole_QP_solver_bmy00, cartpole_QP_solver_dvaff00);
cartpole_QP_solver_LA_DENSE_MTVM_6_15(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff00, cartpole_QP_solver_grad_eq00);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff01, cartpole_QP_solver_D01, cartpole_QP_solver_dvaff00, cartpole_QP_solver_grad_eq01);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff02, cartpole_QP_solver_D01, cartpole_QP_solver_dvaff01, cartpole_QP_solver_grad_eq02);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff03, cartpole_QP_solver_D01, cartpole_QP_solver_dvaff02, cartpole_QP_solver_grad_eq03);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff04, cartpole_QP_solver_D01, cartpole_QP_solver_dvaff03, cartpole_QP_solver_grad_eq04);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff05, cartpole_QP_solver_D01, cartpole_QP_solver_dvaff04, cartpole_QP_solver_grad_eq05);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff06, cartpole_QP_solver_D01, cartpole_QP_solver_dvaff05, cartpole_QP_solver_grad_eq06);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff07, cartpole_QP_solver_D01, cartpole_QP_solver_dvaff06, cartpole_QP_solver_grad_eq07);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff08, cartpole_QP_solver_D01, cartpole_QP_solver_dvaff07, cartpole_QP_solver_grad_eq08);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff09, cartpole_QP_solver_D01, cartpole_QP_solver_dvaff08, cartpole_QP_solver_grad_eq09);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff10, cartpole_QP_solver_D01, cartpole_QP_solver_dvaff09, cartpole_QP_solver_grad_eq10);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff11, cartpole_QP_solver_D01, cartpole_QP_solver_dvaff10, cartpole_QP_solver_grad_eq11);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff12, cartpole_QP_solver_D01, cartpole_QP_solver_dvaff11, cartpole_QP_solver_grad_eq12);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff13, cartpole_QP_solver_D01, cartpole_QP_solver_dvaff12, cartpole_QP_solver_grad_eq13);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff14, cartpole_QP_solver_D01, cartpole_QP_solver_dvaff13, cartpole_QP_solver_grad_eq14);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff15, cartpole_QP_solver_D01, cartpole_QP_solver_dvaff14, cartpole_QP_solver_grad_eq15);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff16, cartpole_QP_solver_D01, cartpole_QP_solver_dvaff15, cartpole_QP_solver_grad_eq16);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvaff17, cartpole_QP_solver_D01, cartpole_QP_solver_dvaff16, cartpole_QP_solver_grad_eq17);
cartpole_QP_solver_LA_DENSE_MTVM2_4_14_6(cartpole_QP_solver_C18, cartpole_QP_solver_dvaff18, cartpole_QP_solver_D18, cartpole_QP_solver_dvaff17, cartpole_QP_solver_grad_eq18);
cartpole_QP_solver_LA_DIAGZERO_MTVM_4_4(cartpole_QP_solver_D19, cartpole_QP_solver_dvaff18, cartpole_QP_solver_grad_eq19);
cartpole_QP_solver_LA_VSUB2_288(cartpole_QP_solver_rd, cartpole_QP_solver_grad_eq, cartpole_QP_solver_rd);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi00, cartpole_QP_solver_rd00, cartpole_QP_solver_dzaff00);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi01, cartpole_QP_solver_rd01, cartpole_QP_solver_dzaff01);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi02, cartpole_QP_solver_rd02, cartpole_QP_solver_dzaff02);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi03, cartpole_QP_solver_rd03, cartpole_QP_solver_dzaff03);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi04, cartpole_QP_solver_rd04, cartpole_QP_solver_dzaff04);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi05, cartpole_QP_solver_rd05, cartpole_QP_solver_dzaff05);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi06, cartpole_QP_solver_rd06, cartpole_QP_solver_dzaff06);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi07, cartpole_QP_solver_rd07, cartpole_QP_solver_dzaff07);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi08, cartpole_QP_solver_rd08, cartpole_QP_solver_dzaff08);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi09, cartpole_QP_solver_rd09, cartpole_QP_solver_dzaff09);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi10, cartpole_QP_solver_rd10, cartpole_QP_solver_dzaff10);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi11, cartpole_QP_solver_rd11, cartpole_QP_solver_dzaff11);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi12, cartpole_QP_solver_rd12, cartpole_QP_solver_dzaff12);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi13, cartpole_QP_solver_rd13, cartpole_QP_solver_dzaff13);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi14, cartpole_QP_solver_rd14, cartpole_QP_solver_dzaff14);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi15, cartpole_QP_solver_rd15, cartpole_QP_solver_dzaff15);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi16, cartpole_QP_solver_rd16, cartpole_QP_solver_dzaff16);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi17, cartpole_QP_solver_rd17, cartpole_QP_solver_dzaff17);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_14(cartpole_QP_solver_Phi18, cartpole_QP_solver_rd18, cartpole_QP_solver_dzaff18);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_4(cartpole_QP_solver_Phi19, cartpole_QP_solver_rd19, cartpole_QP_solver_dzaff19);
cartpole_QP_solver_LA_VSUB_INDEXED_6(cartpole_QP_solver_dzaff00, cartpole_QP_solver_lbIdx00, cartpole_QP_solver_rilb00, cartpole_QP_solver_dslbaff00);
cartpole_QP_solver_LA_VSUB3_6(cartpole_QP_solver_llbbyslb00, cartpole_QP_solver_dslbaff00, cartpole_QP_solver_llb00, cartpole_QP_solver_dllbaff00);
cartpole_QP_solver_LA_VSUB2_INDEXED_6(cartpole_QP_solver_riub00, cartpole_QP_solver_dzaff00, cartpole_QP_solver_ubIdx00, cartpole_QP_solver_dsubaff00);
cartpole_QP_solver_LA_VSUB3_6(cartpole_QP_solver_lubbysub00, cartpole_QP_solver_dsubaff00, cartpole_QP_solver_lub00, cartpole_QP_solver_dlubaff00);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A1, cartpole_QP_solver_dzaff00, cartpole_QP_solver_rip00, cartpole_QP_solver_dsp_aff00);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp00, cartpole_QP_solver_dsp_aff00, cartpole_QP_solver_lp00, cartpole_QP_solver_dlp_aff00);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff01, cartpole_QP_solver_lbIdx01, cartpole_QP_solver_rilb01, cartpole_QP_solver_dslbaff01);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb01, cartpole_QP_solver_dslbaff01, cartpole_QP_solver_llb01, cartpole_QP_solver_dllbaff01);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub01, cartpole_QP_solver_dzaff01, cartpole_QP_solver_ubIdx01, cartpole_QP_solver_dsubaff01);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub01, cartpole_QP_solver_dsubaff01, cartpole_QP_solver_lub01, cartpole_QP_solver_dlubaff01);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A2, cartpole_QP_solver_dzaff01, cartpole_QP_solver_rip01, cartpole_QP_solver_dsp_aff01);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp01, cartpole_QP_solver_dsp_aff01, cartpole_QP_solver_lp01, cartpole_QP_solver_dlp_aff01);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff02, cartpole_QP_solver_lbIdx02, cartpole_QP_solver_rilb02, cartpole_QP_solver_dslbaff02);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb02, cartpole_QP_solver_dslbaff02, cartpole_QP_solver_llb02, cartpole_QP_solver_dllbaff02);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub02, cartpole_QP_solver_dzaff02, cartpole_QP_solver_ubIdx02, cartpole_QP_solver_dsubaff02);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub02, cartpole_QP_solver_dsubaff02, cartpole_QP_solver_lub02, cartpole_QP_solver_dlubaff02);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A3, cartpole_QP_solver_dzaff02, cartpole_QP_solver_rip02, cartpole_QP_solver_dsp_aff02);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp02, cartpole_QP_solver_dsp_aff02, cartpole_QP_solver_lp02, cartpole_QP_solver_dlp_aff02);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff03, cartpole_QP_solver_lbIdx03, cartpole_QP_solver_rilb03, cartpole_QP_solver_dslbaff03);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb03, cartpole_QP_solver_dslbaff03, cartpole_QP_solver_llb03, cartpole_QP_solver_dllbaff03);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub03, cartpole_QP_solver_dzaff03, cartpole_QP_solver_ubIdx03, cartpole_QP_solver_dsubaff03);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub03, cartpole_QP_solver_dsubaff03, cartpole_QP_solver_lub03, cartpole_QP_solver_dlubaff03);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A4, cartpole_QP_solver_dzaff03, cartpole_QP_solver_rip03, cartpole_QP_solver_dsp_aff03);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp03, cartpole_QP_solver_dsp_aff03, cartpole_QP_solver_lp03, cartpole_QP_solver_dlp_aff03);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff04, cartpole_QP_solver_lbIdx04, cartpole_QP_solver_rilb04, cartpole_QP_solver_dslbaff04);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb04, cartpole_QP_solver_dslbaff04, cartpole_QP_solver_llb04, cartpole_QP_solver_dllbaff04);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub04, cartpole_QP_solver_dzaff04, cartpole_QP_solver_ubIdx04, cartpole_QP_solver_dsubaff04);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub04, cartpole_QP_solver_dsubaff04, cartpole_QP_solver_lub04, cartpole_QP_solver_dlubaff04);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A5, cartpole_QP_solver_dzaff04, cartpole_QP_solver_rip04, cartpole_QP_solver_dsp_aff04);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp04, cartpole_QP_solver_dsp_aff04, cartpole_QP_solver_lp04, cartpole_QP_solver_dlp_aff04);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff05, cartpole_QP_solver_lbIdx05, cartpole_QP_solver_rilb05, cartpole_QP_solver_dslbaff05);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb05, cartpole_QP_solver_dslbaff05, cartpole_QP_solver_llb05, cartpole_QP_solver_dllbaff05);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub05, cartpole_QP_solver_dzaff05, cartpole_QP_solver_ubIdx05, cartpole_QP_solver_dsubaff05);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub05, cartpole_QP_solver_dsubaff05, cartpole_QP_solver_lub05, cartpole_QP_solver_dlubaff05);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A6, cartpole_QP_solver_dzaff05, cartpole_QP_solver_rip05, cartpole_QP_solver_dsp_aff05);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp05, cartpole_QP_solver_dsp_aff05, cartpole_QP_solver_lp05, cartpole_QP_solver_dlp_aff05);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff06, cartpole_QP_solver_lbIdx06, cartpole_QP_solver_rilb06, cartpole_QP_solver_dslbaff06);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb06, cartpole_QP_solver_dslbaff06, cartpole_QP_solver_llb06, cartpole_QP_solver_dllbaff06);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub06, cartpole_QP_solver_dzaff06, cartpole_QP_solver_ubIdx06, cartpole_QP_solver_dsubaff06);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub06, cartpole_QP_solver_dsubaff06, cartpole_QP_solver_lub06, cartpole_QP_solver_dlubaff06);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A7, cartpole_QP_solver_dzaff06, cartpole_QP_solver_rip06, cartpole_QP_solver_dsp_aff06);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp06, cartpole_QP_solver_dsp_aff06, cartpole_QP_solver_lp06, cartpole_QP_solver_dlp_aff06);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff07, cartpole_QP_solver_lbIdx07, cartpole_QP_solver_rilb07, cartpole_QP_solver_dslbaff07);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb07, cartpole_QP_solver_dslbaff07, cartpole_QP_solver_llb07, cartpole_QP_solver_dllbaff07);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub07, cartpole_QP_solver_dzaff07, cartpole_QP_solver_ubIdx07, cartpole_QP_solver_dsubaff07);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub07, cartpole_QP_solver_dsubaff07, cartpole_QP_solver_lub07, cartpole_QP_solver_dlubaff07);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A8, cartpole_QP_solver_dzaff07, cartpole_QP_solver_rip07, cartpole_QP_solver_dsp_aff07);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp07, cartpole_QP_solver_dsp_aff07, cartpole_QP_solver_lp07, cartpole_QP_solver_dlp_aff07);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff08, cartpole_QP_solver_lbIdx08, cartpole_QP_solver_rilb08, cartpole_QP_solver_dslbaff08);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb08, cartpole_QP_solver_dslbaff08, cartpole_QP_solver_llb08, cartpole_QP_solver_dllbaff08);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub08, cartpole_QP_solver_dzaff08, cartpole_QP_solver_ubIdx08, cartpole_QP_solver_dsubaff08);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub08, cartpole_QP_solver_dsubaff08, cartpole_QP_solver_lub08, cartpole_QP_solver_dlubaff08);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A9, cartpole_QP_solver_dzaff08, cartpole_QP_solver_rip08, cartpole_QP_solver_dsp_aff08);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp08, cartpole_QP_solver_dsp_aff08, cartpole_QP_solver_lp08, cartpole_QP_solver_dlp_aff08);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff09, cartpole_QP_solver_lbIdx09, cartpole_QP_solver_rilb09, cartpole_QP_solver_dslbaff09);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb09, cartpole_QP_solver_dslbaff09, cartpole_QP_solver_llb09, cartpole_QP_solver_dllbaff09);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub09, cartpole_QP_solver_dzaff09, cartpole_QP_solver_ubIdx09, cartpole_QP_solver_dsubaff09);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub09, cartpole_QP_solver_dsubaff09, cartpole_QP_solver_lub09, cartpole_QP_solver_dlubaff09);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A10, cartpole_QP_solver_dzaff09, cartpole_QP_solver_rip09, cartpole_QP_solver_dsp_aff09);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp09, cartpole_QP_solver_dsp_aff09, cartpole_QP_solver_lp09, cartpole_QP_solver_dlp_aff09);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff10, cartpole_QP_solver_lbIdx10, cartpole_QP_solver_rilb10, cartpole_QP_solver_dslbaff10);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb10, cartpole_QP_solver_dslbaff10, cartpole_QP_solver_llb10, cartpole_QP_solver_dllbaff10);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub10, cartpole_QP_solver_dzaff10, cartpole_QP_solver_ubIdx10, cartpole_QP_solver_dsubaff10);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub10, cartpole_QP_solver_dsubaff10, cartpole_QP_solver_lub10, cartpole_QP_solver_dlubaff10);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A11, cartpole_QP_solver_dzaff10, cartpole_QP_solver_rip10, cartpole_QP_solver_dsp_aff10);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp10, cartpole_QP_solver_dsp_aff10, cartpole_QP_solver_lp10, cartpole_QP_solver_dlp_aff10);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff11, cartpole_QP_solver_lbIdx11, cartpole_QP_solver_rilb11, cartpole_QP_solver_dslbaff11);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb11, cartpole_QP_solver_dslbaff11, cartpole_QP_solver_llb11, cartpole_QP_solver_dllbaff11);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub11, cartpole_QP_solver_dzaff11, cartpole_QP_solver_ubIdx11, cartpole_QP_solver_dsubaff11);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub11, cartpole_QP_solver_dsubaff11, cartpole_QP_solver_lub11, cartpole_QP_solver_dlubaff11);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A12, cartpole_QP_solver_dzaff11, cartpole_QP_solver_rip11, cartpole_QP_solver_dsp_aff11);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp11, cartpole_QP_solver_dsp_aff11, cartpole_QP_solver_lp11, cartpole_QP_solver_dlp_aff11);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff12, cartpole_QP_solver_lbIdx12, cartpole_QP_solver_rilb12, cartpole_QP_solver_dslbaff12);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb12, cartpole_QP_solver_dslbaff12, cartpole_QP_solver_llb12, cartpole_QP_solver_dllbaff12);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub12, cartpole_QP_solver_dzaff12, cartpole_QP_solver_ubIdx12, cartpole_QP_solver_dsubaff12);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub12, cartpole_QP_solver_dsubaff12, cartpole_QP_solver_lub12, cartpole_QP_solver_dlubaff12);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A13, cartpole_QP_solver_dzaff12, cartpole_QP_solver_rip12, cartpole_QP_solver_dsp_aff12);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp12, cartpole_QP_solver_dsp_aff12, cartpole_QP_solver_lp12, cartpole_QP_solver_dlp_aff12);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff13, cartpole_QP_solver_lbIdx13, cartpole_QP_solver_rilb13, cartpole_QP_solver_dslbaff13);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb13, cartpole_QP_solver_dslbaff13, cartpole_QP_solver_llb13, cartpole_QP_solver_dllbaff13);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub13, cartpole_QP_solver_dzaff13, cartpole_QP_solver_ubIdx13, cartpole_QP_solver_dsubaff13);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub13, cartpole_QP_solver_dsubaff13, cartpole_QP_solver_lub13, cartpole_QP_solver_dlubaff13);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A14, cartpole_QP_solver_dzaff13, cartpole_QP_solver_rip13, cartpole_QP_solver_dsp_aff13);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp13, cartpole_QP_solver_dsp_aff13, cartpole_QP_solver_lp13, cartpole_QP_solver_dlp_aff13);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff14, cartpole_QP_solver_lbIdx14, cartpole_QP_solver_rilb14, cartpole_QP_solver_dslbaff14);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb14, cartpole_QP_solver_dslbaff14, cartpole_QP_solver_llb14, cartpole_QP_solver_dllbaff14);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub14, cartpole_QP_solver_dzaff14, cartpole_QP_solver_ubIdx14, cartpole_QP_solver_dsubaff14);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub14, cartpole_QP_solver_dsubaff14, cartpole_QP_solver_lub14, cartpole_QP_solver_dlubaff14);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A15, cartpole_QP_solver_dzaff14, cartpole_QP_solver_rip14, cartpole_QP_solver_dsp_aff14);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp14, cartpole_QP_solver_dsp_aff14, cartpole_QP_solver_lp14, cartpole_QP_solver_dlp_aff14);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff15, cartpole_QP_solver_lbIdx15, cartpole_QP_solver_rilb15, cartpole_QP_solver_dslbaff15);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb15, cartpole_QP_solver_dslbaff15, cartpole_QP_solver_llb15, cartpole_QP_solver_dllbaff15);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub15, cartpole_QP_solver_dzaff15, cartpole_QP_solver_ubIdx15, cartpole_QP_solver_dsubaff15);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub15, cartpole_QP_solver_dsubaff15, cartpole_QP_solver_lub15, cartpole_QP_solver_dlubaff15);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A16, cartpole_QP_solver_dzaff15, cartpole_QP_solver_rip15, cartpole_QP_solver_dsp_aff15);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp15, cartpole_QP_solver_dsp_aff15, cartpole_QP_solver_lp15, cartpole_QP_solver_dlp_aff15);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff16, cartpole_QP_solver_lbIdx16, cartpole_QP_solver_rilb16, cartpole_QP_solver_dslbaff16);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb16, cartpole_QP_solver_dslbaff16, cartpole_QP_solver_llb16, cartpole_QP_solver_dllbaff16);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub16, cartpole_QP_solver_dzaff16, cartpole_QP_solver_ubIdx16, cartpole_QP_solver_dsubaff16);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub16, cartpole_QP_solver_dsubaff16, cartpole_QP_solver_lub16, cartpole_QP_solver_dlubaff16);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A17, cartpole_QP_solver_dzaff16, cartpole_QP_solver_rip16, cartpole_QP_solver_dsp_aff16);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp16, cartpole_QP_solver_dsp_aff16, cartpole_QP_solver_lp16, cartpole_QP_solver_dlp_aff16);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff17, cartpole_QP_solver_lbIdx17, cartpole_QP_solver_rilb17, cartpole_QP_solver_dslbaff17);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb17, cartpole_QP_solver_dslbaff17, cartpole_QP_solver_llb17, cartpole_QP_solver_dllbaff17);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub17, cartpole_QP_solver_dzaff17, cartpole_QP_solver_ubIdx17, cartpole_QP_solver_dsubaff17);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub17, cartpole_QP_solver_dsubaff17, cartpole_QP_solver_lub17, cartpole_QP_solver_dlubaff17);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_15(params->A18, cartpole_QP_solver_dzaff17, cartpole_QP_solver_rip17, cartpole_QP_solver_dsp_aff17);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp17, cartpole_QP_solver_dsp_aff17, cartpole_QP_solver_lp17, cartpole_QP_solver_dlp_aff17);
cartpole_QP_solver_LA_VSUB_INDEXED_1(cartpole_QP_solver_dzaff18, cartpole_QP_solver_lbIdx18, cartpole_QP_solver_rilb18, cartpole_QP_solver_dslbaff18);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_llbbyslb18, cartpole_QP_solver_dslbaff18, cartpole_QP_solver_llb18, cartpole_QP_solver_dllbaff18);
cartpole_QP_solver_LA_VSUB2_INDEXED_1(cartpole_QP_solver_riub18, cartpole_QP_solver_dzaff18, cartpole_QP_solver_ubIdx18, cartpole_QP_solver_dsubaff18);
cartpole_QP_solver_LA_VSUB3_1(cartpole_QP_solver_lubbysub18, cartpole_QP_solver_dsubaff18, cartpole_QP_solver_lub18, cartpole_QP_solver_dlubaff18);
cartpole_QP_solver_LA_DENSE_MVMSUB4_20_14(params->A19, cartpole_QP_solver_dzaff18, cartpole_QP_solver_rip18, cartpole_QP_solver_dsp_aff18);
cartpole_QP_solver_LA_VSUB3_20(cartpole_QP_solver_lpbysp18, cartpole_QP_solver_dsp_aff18, cartpole_QP_solver_lp18, cartpole_QP_solver_dlp_aff18);
cartpole_QP_solver_LA_VSUB_INDEXED_4(cartpole_QP_solver_dzaff19, cartpole_QP_solver_lbIdx19, cartpole_QP_solver_rilb19, cartpole_QP_solver_dslbaff19);
cartpole_QP_solver_LA_VSUB3_4(cartpole_QP_solver_llbbyslb19, cartpole_QP_solver_dslbaff19, cartpole_QP_solver_llb19, cartpole_QP_solver_dllbaff19);
cartpole_QP_solver_LA_VSUB2_INDEXED_4(cartpole_QP_solver_riub19, cartpole_QP_solver_dzaff19, cartpole_QP_solver_ubIdx19, cartpole_QP_solver_dsubaff19);
cartpole_QP_solver_LA_VSUB3_4(cartpole_QP_solver_lubbysub19, cartpole_QP_solver_dsubaff19, cartpole_QP_solver_lub19, cartpole_QP_solver_dlubaff19);
info->lsit_aff = cartpole_QP_solver_LINESEARCH_BACKTRACKING_AFFINE(cartpole_QP_solver_l, cartpole_QP_solver_s, cartpole_QP_solver_dl_aff, cartpole_QP_solver_ds_aff, &info->step_aff, &info->mu_aff);
if( info->lsit_aff == cartpole_QP_solver_NOPROGRESS ){
PRINTTEXT("Affine line search could not proceed at iteration %d.\nThe problem might be infeasible -- exiting.\n",info->it+1);
exitcode = cartpole_QP_solver_NOPROGRESS; break;
}
sigma_3rdroot = info->mu_aff / info->mu;
info->sigma = sigma_3rdroot*sigma_3rdroot*sigma_3rdroot;
musigma = info->mu * info->sigma;
cartpole_QP_solver_LA_VSUB5_436(cartpole_QP_solver_ds_aff, cartpole_QP_solver_dl_aff, info->mu, info->sigma, cartpole_QP_solver_ccrhs);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_6_6(cartpole_QP_solver_ccrhsub00, cartpole_QP_solver_sub00, cartpole_QP_solver_ubIdx00, cartpole_QP_solver_ccrhsl00, cartpole_QP_solver_slb00, cartpole_QP_solver_lbIdx00, cartpole_QP_solver_rd00);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_ccrhsub01, cartpole_QP_solver_sub01, cartpole_QP_solver_ubIdx01, cartpole_QP_solver_ccrhsl01, cartpole_QP_solver_slb01, cartpole_QP_solver_lbIdx01, cartpole_QP_solver_rd01);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A1, cartpole_QP_solver_ccrhsp00, cartpole_QP_solver_sp00, cartpole_QP_solver_rd00);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A2, cartpole_QP_solver_ccrhsp01, cartpole_QP_solver_sp01, cartpole_QP_solver_rd01);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi00, cartpole_QP_solver_rd00, cartpole_QP_solver_Lbyrd00);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi01, cartpole_QP_solver_rd01, cartpole_QP_solver_Lbyrd01);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_V00, cartpole_QP_solver_Lbyrd00, cartpole_QP_solver_W01, cartpole_QP_solver_Lbyrd01, cartpole_QP_solver_beta00);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld00, cartpole_QP_solver_beta00, cartpole_QP_solver_yy00);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_ccrhsub02, cartpole_QP_solver_sub02, cartpole_QP_solver_ubIdx02, cartpole_QP_solver_ccrhsl02, cartpole_QP_solver_slb02, cartpole_QP_solver_lbIdx02, cartpole_QP_solver_rd02);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A3, cartpole_QP_solver_ccrhsp02, cartpole_QP_solver_sp02, cartpole_QP_solver_rd02);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi02, cartpole_QP_solver_rd02, cartpole_QP_solver_Lbyrd02);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_V01, cartpole_QP_solver_Lbyrd01, cartpole_QP_solver_W02, cartpole_QP_solver_Lbyrd02, cartpole_QP_solver_beta01);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd01, cartpole_QP_solver_yy00, cartpole_QP_solver_beta01, cartpole_QP_solver_bmy01);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld01, cartpole_QP_solver_bmy01, cartpole_QP_solver_yy01);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_ccrhsub03, cartpole_QP_solver_sub03, cartpole_QP_solver_ubIdx03, cartpole_QP_solver_ccrhsl03, cartpole_QP_solver_slb03, cartpole_QP_solver_lbIdx03, cartpole_QP_solver_rd03);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A4, cartpole_QP_solver_ccrhsp03, cartpole_QP_solver_sp03, cartpole_QP_solver_rd03);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi03, cartpole_QP_solver_rd03, cartpole_QP_solver_Lbyrd03);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_V02, cartpole_QP_solver_Lbyrd02, cartpole_QP_solver_W03, cartpole_QP_solver_Lbyrd03, cartpole_QP_solver_beta02);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd02, cartpole_QP_solver_yy01, cartpole_QP_solver_beta02, cartpole_QP_solver_bmy02);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld02, cartpole_QP_solver_bmy02, cartpole_QP_solver_yy02);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_ccrhsub04, cartpole_QP_solver_sub04, cartpole_QP_solver_ubIdx04, cartpole_QP_solver_ccrhsl04, cartpole_QP_solver_slb04, cartpole_QP_solver_lbIdx04, cartpole_QP_solver_rd04);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A5, cartpole_QP_solver_ccrhsp04, cartpole_QP_solver_sp04, cartpole_QP_solver_rd04);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi04, cartpole_QP_solver_rd04, cartpole_QP_solver_Lbyrd04);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_V03, cartpole_QP_solver_Lbyrd03, cartpole_QP_solver_W04, cartpole_QP_solver_Lbyrd04, cartpole_QP_solver_beta03);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd03, cartpole_QP_solver_yy02, cartpole_QP_solver_beta03, cartpole_QP_solver_bmy03);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld03, cartpole_QP_solver_bmy03, cartpole_QP_solver_yy03);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_ccrhsub05, cartpole_QP_solver_sub05, cartpole_QP_solver_ubIdx05, cartpole_QP_solver_ccrhsl05, cartpole_QP_solver_slb05, cartpole_QP_solver_lbIdx05, cartpole_QP_solver_rd05);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A6, cartpole_QP_solver_ccrhsp05, cartpole_QP_solver_sp05, cartpole_QP_solver_rd05);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi05, cartpole_QP_solver_rd05, cartpole_QP_solver_Lbyrd05);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_V04, cartpole_QP_solver_Lbyrd04, cartpole_QP_solver_W05, cartpole_QP_solver_Lbyrd05, cartpole_QP_solver_beta04);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd04, cartpole_QP_solver_yy03, cartpole_QP_solver_beta04, cartpole_QP_solver_bmy04);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld04, cartpole_QP_solver_bmy04, cartpole_QP_solver_yy04);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_ccrhsub06, cartpole_QP_solver_sub06, cartpole_QP_solver_ubIdx06, cartpole_QP_solver_ccrhsl06, cartpole_QP_solver_slb06, cartpole_QP_solver_lbIdx06, cartpole_QP_solver_rd06);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A7, cartpole_QP_solver_ccrhsp06, cartpole_QP_solver_sp06, cartpole_QP_solver_rd06);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi06, cartpole_QP_solver_rd06, cartpole_QP_solver_Lbyrd06);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_V05, cartpole_QP_solver_Lbyrd05, cartpole_QP_solver_W06, cartpole_QP_solver_Lbyrd06, cartpole_QP_solver_beta05);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd05, cartpole_QP_solver_yy04, cartpole_QP_solver_beta05, cartpole_QP_solver_bmy05);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld05, cartpole_QP_solver_bmy05, cartpole_QP_solver_yy05);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_ccrhsub07, cartpole_QP_solver_sub07, cartpole_QP_solver_ubIdx07, cartpole_QP_solver_ccrhsl07, cartpole_QP_solver_slb07, cartpole_QP_solver_lbIdx07, cartpole_QP_solver_rd07);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A8, cartpole_QP_solver_ccrhsp07, cartpole_QP_solver_sp07, cartpole_QP_solver_rd07);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi07, cartpole_QP_solver_rd07, cartpole_QP_solver_Lbyrd07);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_V06, cartpole_QP_solver_Lbyrd06, cartpole_QP_solver_W07, cartpole_QP_solver_Lbyrd07, cartpole_QP_solver_beta06);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd06, cartpole_QP_solver_yy05, cartpole_QP_solver_beta06, cartpole_QP_solver_bmy06);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld06, cartpole_QP_solver_bmy06, cartpole_QP_solver_yy06);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_ccrhsub08, cartpole_QP_solver_sub08, cartpole_QP_solver_ubIdx08, cartpole_QP_solver_ccrhsl08, cartpole_QP_solver_slb08, cartpole_QP_solver_lbIdx08, cartpole_QP_solver_rd08);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A9, cartpole_QP_solver_ccrhsp08, cartpole_QP_solver_sp08, cartpole_QP_solver_rd08);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi08, cartpole_QP_solver_rd08, cartpole_QP_solver_Lbyrd08);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_V07, cartpole_QP_solver_Lbyrd07, cartpole_QP_solver_W08, cartpole_QP_solver_Lbyrd08, cartpole_QP_solver_beta07);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd07, cartpole_QP_solver_yy06, cartpole_QP_solver_beta07, cartpole_QP_solver_bmy07);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld07, cartpole_QP_solver_bmy07, cartpole_QP_solver_yy07);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_ccrhsub09, cartpole_QP_solver_sub09, cartpole_QP_solver_ubIdx09, cartpole_QP_solver_ccrhsl09, cartpole_QP_solver_slb09, cartpole_QP_solver_lbIdx09, cartpole_QP_solver_rd09);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A10, cartpole_QP_solver_ccrhsp09, cartpole_QP_solver_sp09, cartpole_QP_solver_rd09);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi09, cartpole_QP_solver_rd09, cartpole_QP_solver_Lbyrd09);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_V08, cartpole_QP_solver_Lbyrd08, cartpole_QP_solver_W09, cartpole_QP_solver_Lbyrd09, cartpole_QP_solver_beta08);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd08, cartpole_QP_solver_yy07, cartpole_QP_solver_beta08, cartpole_QP_solver_bmy08);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld08, cartpole_QP_solver_bmy08, cartpole_QP_solver_yy08);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_ccrhsub10, cartpole_QP_solver_sub10, cartpole_QP_solver_ubIdx10, cartpole_QP_solver_ccrhsl10, cartpole_QP_solver_slb10, cartpole_QP_solver_lbIdx10, cartpole_QP_solver_rd10);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A11, cartpole_QP_solver_ccrhsp10, cartpole_QP_solver_sp10, cartpole_QP_solver_rd10);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi10, cartpole_QP_solver_rd10, cartpole_QP_solver_Lbyrd10);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_V09, cartpole_QP_solver_Lbyrd09, cartpole_QP_solver_W10, cartpole_QP_solver_Lbyrd10, cartpole_QP_solver_beta09);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd09, cartpole_QP_solver_yy08, cartpole_QP_solver_beta09, cartpole_QP_solver_bmy09);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld09, cartpole_QP_solver_bmy09, cartpole_QP_solver_yy09);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_ccrhsub11, cartpole_QP_solver_sub11, cartpole_QP_solver_ubIdx11, cartpole_QP_solver_ccrhsl11, cartpole_QP_solver_slb11, cartpole_QP_solver_lbIdx11, cartpole_QP_solver_rd11);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A12, cartpole_QP_solver_ccrhsp11, cartpole_QP_solver_sp11, cartpole_QP_solver_rd11);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi11, cartpole_QP_solver_rd11, cartpole_QP_solver_Lbyrd11);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_V10, cartpole_QP_solver_Lbyrd10, cartpole_QP_solver_W11, cartpole_QP_solver_Lbyrd11, cartpole_QP_solver_beta10);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd10, cartpole_QP_solver_yy09, cartpole_QP_solver_beta10, cartpole_QP_solver_bmy10);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld10, cartpole_QP_solver_bmy10, cartpole_QP_solver_yy10);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_ccrhsub12, cartpole_QP_solver_sub12, cartpole_QP_solver_ubIdx12, cartpole_QP_solver_ccrhsl12, cartpole_QP_solver_slb12, cartpole_QP_solver_lbIdx12, cartpole_QP_solver_rd12);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A13, cartpole_QP_solver_ccrhsp12, cartpole_QP_solver_sp12, cartpole_QP_solver_rd12);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi12, cartpole_QP_solver_rd12, cartpole_QP_solver_Lbyrd12);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_V11, cartpole_QP_solver_Lbyrd11, cartpole_QP_solver_W12, cartpole_QP_solver_Lbyrd12, cartpole_QP_solver_beta11);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd11, cartpole_QP_solver_yy10, cartpole_QP_solver_beta11, cartpole_QP_solver_bmy11);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld11, cartpole_QP_solver_bmy11, cartpole_QP_solver_yy11);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_ccrhsub13, cartpole_QP_solver_sub13, cartpole_QP_solver_ubIdx13, cartpole_QP_solver_ccrhsl13, cartpole_QP_solver_slb13, cartpole_QP_solver_lbIdx13, cartpole_QP_solver_rd13);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A14, cartpole_QP_solver_ccrhsp13, cartpole_QP_solver_sp13, cartpole_QP_solver_rd13);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi13, cartpole_QP_solver_rd13, cartpole_QP_solver_Lbyrd13);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_V12, cartpole_QP_solver_Lbyrd12, cartpole_QP_solver_W13, cartpole_QP_solver_Lbyrd13, cartpole_QP_solver_beta12);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd12, cartpole_QP_solver_yy11, cartpole_QP_solver_beta12, cartpole_QP_solver_bmy12);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld12, cartpole_QP_solver_bmy12, cartpole_QP_solver_yy12);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_ccrhsub14, cartpole_QP_solver_sub14, cartpole_QP_solver_ubIdx14, cartpole_QP_solver_ccrhsl14, cartpole_QP_solver_slb14, cartpole_QP_solver_lbIdx14, cartpole_QP_solver_rd14);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A15, cartpole_QP_solver_ccrhsp14, cartpole_QP_solver_sp14, cartpole_QP_solver_rd14);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi14, cartpole_QP_solver_rd14, cartpole_QP_solver_Lbyrd14);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_V13, cartpole_QP_solver_Lbyrd13, cartpole_QP_solver_W14, cartpole_QP_solver_Lbyrd14, cartpole_QP_solver_beta13);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd13, cartpole_QP_solver_yy12, cartpole_QP_solver_beta13, cartpole_QP_solver_bmy13);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld13, cartpole_QP_solver_bmy13, cartpole_QP_solver_yy13);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_ccrhsub15, cartpole_QP_solver_sub15, cartpole_QP_solver_ubIdx15, cartpole_QP_solver_ccrhsl15, cartpole_QP_solver_slb15, cartpole_QP_solver_lbIdx15, cartpole_QP_solver_rd15);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A16, cartpole_QP_solver_ccrhsp15, cartpole_QP_solver_sp15, cartpole_QP_solver_rd15);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi15, cartpole_QP_solver_rd15, cartpole_QP_solver_Lbyrd15);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_V14, cartpole_QP_solver_Lbyrd14, cartpole_QP_solver_W15, cartpole_QP_solver_Lbyrd15, cartpole_QP_solver_beta14);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd14, cartpole_QP_solver_yy13, cartpole_QP_solver_beta14, cartpole_QP_solver_bmy14);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld14, cartpole_QP_solver_bmy14, cartpole_QP_solver_yy14);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_ccrhsub16, cartpole_QP_solver_sub16, cartpole_QP_solver_ubIdx16, cartpole_QP_solver_ccrhsl16, cartpole_QP_solver_slb16, cartpole_QP_solver_lbIdx16, cartpole_QP_solver_rd16);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A17, cartpole_QP_solver_ccrhsp16, cartpole_QP_solver_sp16, cartpole_QP_solver_rd16);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi16, cartpole_QP_solver_rd16, cartpole_QP_solver_Lbyrd16);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_V15, cartpole_QP_solver_Lbyrd15, cartpole_QP_solver_W16, cartpole_QP_solver_Lbyrd16, cartpole_QP_solver_beta15);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd15, cartpole_QP_solver_yy14, cartpole_QP_solver_beta15, cartpole_QP_solver_bmy15);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld15, cartpole_QP_solver_bmy15, cartpole_QP_solver_yy15);
cartpole_QP_solver_LA_VSUB6_INDEXED_15_1_1(cartpole_QP_solver_ccrhsub17, cartpole_QP_solver_sub17, cartpole_QP_solver_ubIdx17, cartpole_QP_solver_ccrhsl17, cartpole_QP_solver_slb17, cartpole_QP_solver_lbIdx17, cartpole_QP_solver_rd17);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_15(params->A18, cartpole_QP_solver_ccrhsp17, cartpole_QP_solver_sp17, cartpole_QP_solver_rd17);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_15(cartpole_QP_solver_Phi17, cartpole_QP_solver_rd17, cartpole_QP_solver_Lbyrd17);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_15(cartpole_QP_solver_V16, cartpole_QP_solver_Lbyrd16, cartpole_QP_solver_W17, cartpole_QP_solver_Lbyrd17, cartpole_QP_solver_beta16);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd16, cartpole_QP_solver_yy15, cartpole_QP_solver_beta16, cartpole_QP_solver_bmy16);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld16, cartpole_QP_solver_bmy16, cartpole_QP_solver_yy16);
cartpole_QP_solver_LA_VSUB6_INDEXED_14_1_1(cartpole_QP_solver_ccrhsub18, cartpole_QP_solver_sub18, cartpole_QP_solver_ubIdx18, cartpole_QP_solver_ccrhsl18, cartpole_QP_solver_slb18, cartpole_QP_solver_lbIdx18, cartpole_QP_solver_rd18);
cartpole_QP_solver_LA_DENSE_MTVMADD2_20_14(params->A19, cartpole_QP_solver_ccrhsp18, cartpole_QP_solver_sp18, cartpole_QP_solver_rd18);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_14(cartpole_QP_solver_Phi18, cartpole_QP_solver_rd18, cartpole_QP_solver_Lbyrd18);
cartpole_QP_solver_LA_DENSE_2MVMADD_6_15_14(cartpole_QP_solver_V17, cartpole_QP_solver_Lbyrd17, cartpole_QP_solver_W18, cartpole_QP_solver_Lbyrd18, cartpole_QP_solver_beta17);
cartpole_QP_solver_LA_DENSE_MVMSUB1_6_6(cartpole_QP_solver_Lsd17, cartpole_QP_solver_yy16, cartpole_QP_solver_beta17, cartpole_QP_solver_bmy17);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_6(cartpole_QP_solver_Ld17, cartpole_QP_solver_bmy17, cartpole_QP_solver_yy17);
cartpole_QP_solver_LA_VSUB6_INDEXED_4_4_4(cartpole_QP_solver_ccrhsub19, cartpole_QP_solver_sub19, cartpole_QP_solver_ubIdx19, cartpole_QP_solver_ccrhsl19, cartpole_QP_solver_slb19, cartpole_QP_solver_lbIdx19, cartpole_QP_solver_rd19);
cartpole_QP_solver_LA_DIAG_FORWARDSUB_4(cartpole_QP_solver_Phi19, cartpole_QP_solver_rd19, cartpole_QP_solver_Lbyrd19);
cartpole_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_4_14_4(cartpole_QP_solver_V18, cartpole_QP_solver_Lbyrd18, cartpole_QP_solver_W19, cartpole_QP_solver_Lbyrd19, cartpole_QP_solver_beta18);
cartpole_QP_solver_LA_DENSE_MVMSUB1_4_6(cartpole_QP_solver_Lsd18, cartpole_QP_solver_yy17, cartpole_QP_solver_beta18, cartpole_QP_solver_bmy18);
cartpole_QP_solver_LA_DENSE_FORWARDSUB_4(cartpole_QP_solver_Ld18, cartpole_QP_solver_bmy18, cartpole_QP_solver_yy18);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_4(cartpole_QP_solver_Ld18, cartpole_QP_solver_yy18, cartpole_QP_solver_dvcc18);
cartpole_QP_solver_LA_DENSE_MTVMSUB_4_6(cartpole_QP_solver_Lsd18, cartpole_QP_solver_dvcc18, cartpole_QP_solver_yy17, cartpole_QP_solver_bmy17);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld17, cartpole_QP_solver_bmy17, cartpole_QP_solver_dvcc17);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd17, cartpole_QP_solver_dvcc17, cartpole_QP_solver_yy16, cartpole_QP_solver_bmy16);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld16, cartpole_QP_solver_bmy16, cartpole_QP_solver_dvcc16);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd16, cartpole_QP_solver_dvcc16, cartpole_QP_solver_yy15, cartpole_QP_solver_bmy15);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld15, cartpole_QP_solver_bmy15, cartpole_QP_solver_dvcc15);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd15, cartpole_QP_solver_dvcc15, cartpole_QP_solver_yy14, cartpole_QP_solver_bmy14);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld14, cartpole_QP_solver_bmy14, cartpole_QP_solver_dvcc14);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd14, cartpole_QP_solver_dvcc14, cartpole_QP_solver_yy13, cartpole_QP_solver_bmy13);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld13, cartpole_QP_solver_bmy13, cartpole_QP_solver_dvcc13);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd13, cartpole_QP_solver_dvcc13, cartpole_QP_solver_yy12, cartpole_QP_solver_bmy12);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld12, cartpole_QP_solver_bmy12, cartpole_QP_solver_dvcc12);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd12, cartpole_QP_solver_dvcc12, cartpole_QP_solver_yy11, cartpole_QP_solver_bmy11);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld11, cartpole_QP_solver_bmy11, cartpole_QP_solver_dvcc11);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd11, cartpole_QP_solver_dvcc11, cartpole_QP_solver_yy10, cartpole_QP_solver_bmy10);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld10, cartpole_QP_solver_bmy10, cartpole_QP_solver_dvcc10);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd10, cartpole_QP_solver_dvcc10, cartpole_QP_solver_yy09, cartpole_QP_solver_bmy09);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld09, cartpole_QP_solver_bmy09, cartpole_QP_solver_dvcc09);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd09, cartpole_QP_solver_dvcc09, cartpole_QP_solver_yy08, cartpole_QP_solver_bmy08);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld08, cartpole_QP_solver_bmy08, cartpole_QP_solver_dvcc08);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd08, cartpole_QP_solver_dvcc08, cartpole_QP_solver_yy07, cartpole_QP_solver_bmy07);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld07, cartpole_QP_solver_bmy07, cartpole_QP_solver_dvcc07);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd07, cartpole_QP_solver_dvcc07, cartpole_QP_solver_yy06, cartpole_QP_solver_bmy06);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld06, cartpole_QP_solver_bmy06, cartpole_QP_solver_dvcc06);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd06, cartpole_QP_solver_dvcc06, cartpole_QP_solver_yy05, cartpole_QP_solver_bmy05);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld05, cartpole_QP_solver_bmy05, cartpole_QP_solver_dvcc05);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd05, cartpole_QP_solver_dvcc05, cartpole_QP_solver_yy04, cartpole_QP_solver_bmy04);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld04, cartpole_QP_solver_bmy04, cartpole_QP_solver_dvcc04);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd04, cartpole_QP_solver_dvcc04, cartpole_QP_solver_yy03, cartpole_QP_solver_bmy03);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld03, cartpole_QP_solver_bmy03, cartpole_QP_solver_dvcc03);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd03, cartpole_QP_solver_dvcc03, cartpole_QP_solver_yy02, cartpole_QP_solver_bmy02);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld02, cartpole_QP_solver_bmy02, cartpole_QP_solver_dvcc02);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd02, cartpole_QP_solver_dvcc02, cartpole_QP_solver_yy01, cartpole_QP_solver_bmy01);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld01, cartpole_QP_solver_bmy01, cartpole_QP_solver_dvcc01);
cartpole_QP_solver_LA_DENSE_MTVMSUB_6_6(cartpole_QP_solver_Lsd01, cartpole_QP_solver_dvcc01, cartpole_QP_solver_yy00, cartpole_QP_solver_bmy00);
cartpole_QP_solver_LA_DENSE_BACKWARDSUB_6(cartpole_QP_solver_Ld00, cartpole_QP_solver_bmy00, cartpole_QP_solver_dvcc00);
cartpole_QP_solver_LA_DENSE_MTVM_6_15(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc00, cartpole_QP_solver_grad_eq00);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc01, cartpole_QP_solver_D01, cartpole_QP_solver_dvcc00, cartpole_QP_solver_grad_eq01);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc02, cartpole_QP_solver_D01, cartpole_QP_solver_dvcc01, cartpole_QP_solver_grad_eq02);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc03, cartpole_QP_solver_D01, cartpole_QP_solver_dvcc02, cartpole_QP_solver_grad_eq03);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc04, cartpole_QP_solver_D01, cartpole_QP_solver_dvcc03, cartpole_QP_solver_grad_eq04);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc05, cartpole_QP_solver_D01, cartpole_QP_solver_dvcc04, cartpole_QP_solver_grad_eq05);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc06, cartpole_QP_solver_D01, cartpole_QP_solver_dvcc05, cartpole_QP_solver_grad_eq06);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc07, cartpole_QP_solver_D01, cartpole_QP_solver_dvcc06, cartpole_QP_solver_grad_eq07);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc08, cartpole_QP_solver_D01, cartpole_QP_solver_dvcc07, cartpole_QP_solver_grad_eq08);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc09, cartpole_QP_solver_D01, cartpole_QP_solver_dvcc08, cartpole_QP_solver_grad_eq09);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc10, cartpole_QP_solver_D01, cartpole_QP_solver_dvcc09, cartpole_QP_solver_grad_eq10);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc11, cartpole_QP_solver_D01, cartpole_QP_solver_dvcc10, cartpole_QP_solver_grad_eq11);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc12, cartpole_QP_solver_D01, cartpole_QP_solver_dvcc11, cartpole_QP_solver_grad_eq12);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc13, cartpole_QP_solver_D01, cartpole_QP_solver_dvcc12, cartpole_QP_solver_grad_eq13);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc14, cartpole_QP_solver_D01, cartpole_QP_solver_dvcc13, cartpole_QP_solver_grad_eq14);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc15, cartpole_QP_solver_D01, cartpole_QP_solver_dvcc14, cartpole_QP_solver_grad_eq15);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc16, cartpole_QP_solver_D01, cartpole_QP_solver_dvcc15, cartpole_QP_solver_grad_eq16);
cartpole_QP_solver_LA_DENSE_MTVM2_6_15_6(cartpole_QP_solver_C00, cartpole_QP_solver_dvcc17, cartpole_QP_solver_D01, cartpole_QP_solver_dvcc16, cartpole_QP_solver_grad_eq17);
cartpole_QP_solver_LA_DENSE_MTVM2_4_14_6(cartpole_QP_solver_C18, cartpole_QP_solver_dvcc18, cartpole_QP_solver_D18, cartpole_QP_solver_dvcc17, cartpole_QP_solver_grad_eq18);
cartpole_QP_solver_LA_DIAGZERO_MTVM_4_4(cartpole_QP_solver_D19, cartpole_QP_solver_dvcc18, cartpole_QP_solver_grad_eq19);
cartpole_QP_solver_LA_VSUB_288(cartpole_QP_solver_rd, cartpole_QP_solver_grad_eq, cartpole_QP_solver_rd);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi00, cartpole_QP_solver_rd00, cartpole_QP_solver_dzcc00);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi01, cartpole_QP_solver_rd01, cartpole_QP_solver_dzcc01);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi02, cartpole_QP_solver_rd02, cartpole_QP_solver_dzcc02);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi03, cartpole_QP_solver_rd03, cartpole_QP_solver_dzcc03);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi04, cartpole_QP_solver_rd04, cartpole_QP_solver_dzcc04);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi05, cartpole_QP_solver_rd05, cartpole_QP_solver_dzcc05);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi06, cartpole_QP_solver_rd06, cartpole_QP_solver_dzcc06);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi07, cartpole_QP_solver_rd07, cartpole_QP_solver_dzcc07);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi08, cartpole_QP_solver_rd08, cartpole_QP_solver_dzcc08);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi09, cartpole_QP_solver_rd09, cartpole_QP_solver_dzcc09);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi10, cartpole_QP_solver_rd10, cartpole_QP_solver_dzcc10);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi11, cartpole_QP_solver_rd11, cartpole_QP_solver_dzcc11);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi12, cartpole_QP_solver_rd12, cartpole_QP_solver_dzcc12);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi13, cartpole_QP_solver_rd13, cartpole_QP_solver_dzcc13);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi14, cartpole_QP_solver_rd14, cartpole_QP_solver_dzcc14);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi15, cartpole_QP_solver_rd15, cartpole_QP_solver_dzcc15);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi16, cartpole_QP_solver_rd16, cartpole_QP_solver_dzcc16);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(cartpole_QP_solver_Phi17, cartpole_QP_solver_rd17, cartpole_QP_solver_dzcc17);
cartpole_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_14(cartpole_QP_solver_Phi18, cartpole_QP_solver_rd18, cartpole_QP_solver_dzcc18);
cartpole_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_4(cartpole_QP_solver_Phi19, cartpole_QP_solver_rd19, cartpole_QP_solver_dzcc19);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(cartpole_QP_solver_ccrhsl00, cartpole_QP_solver_slb00, cartpole_QP_solver_llbbyslb00, cartpole_QP_solver_dzcc00, cartpole_QP_solver_lbIdx00, cartpole_QP_solver_dllbcc00);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(cartpole_QP_solver_ccrhsub00, cartpole_QP_solver_sub00, cartpole_QP_solver_lubbysub00, cartpole_QP_solver_dzcc00, cartpole_QP_solver_ubIdx00, cartpole_QP_solver_dlubcc00);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A1, cartpole_QP_solver_dzcc00, cartpole_QP_solver_ccrhsp00, cartpole_QP_solver_sp00, cartpole_QP_solver_lp00, cartpole_QP_solver_dlp_cc00);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl01, cartpole_QP_solver_slb01, cartpole_QP_solver_llbbyslb01, cartpole_QP_solver_dzcc01, cartpole_QP_solver_lbIdx01, cartpole_QP_solver_dllbcc01);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub01, cartpole_QP_solver_sub01, cartpole_QP_solver_lubbysub01, cartpole_QP_solver_dzcc01, cartpole_QP_solver_ubIdx01, cartpole_QP_solver_dlubcc01);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A2, cartpole_QP_solver_dzcc01, cartpole_QP_solver_ccrhsp01, cartpole_QP_solver_sp01, cartpole_QP_solver_lp01, cartpole_QP_solver_dlp_cc01);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl02, cartpole_QP_solver_slb02, cartpole_QP_solver_llbbyslb02, cartpole_QP_solver_dzcc02, cartpole_QP_solver_lbIdx02, cartpole_QP_solver_dllbcc02);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub02, cartpole_QP_solver_sub02, cartpole_QP_solver_lubbysub02, cartpole_QP_solver_dzcc02, cartpole_QP_solver_ubIdx02, cartpole_QP_solver_dlubcc02);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A3, cartpole_QP_solver_dzcc02, cartpole_QP_solver_ccrhsp02, cartpole_QP_solver_sp02, cartpole_QP_solver_lp02, cartpole_QP_solver_dlp_cc02);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl03, cartpole_QP_solver_slb03, cartpole_QP_solver_llbbyslb03, cartpole_QP_solver_dzcc03, cartpole_QP_solver_lbIdx03, cartpole_QP_solver_dllbcc03);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub03, cartpole_QP_solver_sub03, cartpole_QP_solver_lubbysub03, cartpole_QP_solver_dzcc03, cartpole_QP_solver_ubIdx03, cartpole_QP_solver_dlubcc03);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A4, cartpole_QP_solver_dzcc03, cartpole_QP_solver_ccrhsp03, cartpole_QP_solver_sp03, cartpole_QP_solver_lp03, cartpole_QP_solver_dlp_cc03);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl04, cartpole_QP_solver_slb04, cartpole_QP_solver_llbbyslb04, cartpole_QP_solver_dzcc04, cartpole_QP_solver_lbIdx04, cartpole_QP_solver_dllbcc04);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub04, cartpole_QP_solver_sub04, cartpole_QP_solver_lubbysub04, cartpole_QP_solver_dzcc04, cartpole_QP_solver_ubIdx04, cartpole_QP_solver_dlubcc04);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A5, cartpole_QP_solver_dzcc04, cartpole_QP_solver_ccrhsp04, cartpole_QP_solver_sp04, cartpole_QP_solver_lp04, cartpole_QP_solver_dlp_cc04);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl05, cartpole_QP_solver_slb05, cartpole_QP_solver_llbbyslb05, cartpole_QP_solver_dzcc05, cartpole_QP_solver_lbIdx05, cartpole_QP_solver_dllbcc05);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub05, cartpole_QP_solver_sub05, cartpole_QP_solver_lubbysub05, cartpole_QP_solver_dzcc05, cartpole_QP_solver_ubIdx05, cartpole_QP_solver_dlubcc05);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A6, cartpole_QP_solver_dzcc05, cartpole_QP_solver_ccrhsp05, cartpole_QP_solver_sp05, cartpole_QP_solver_lp05, cartpole_QP_solver_dlp_cc05);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl06, cartpole_QP_solver_slb06, cartpole_QP_solver_llbbyslb06, cartpole_QP_solver_dzcc06, cartpole_QP_solver_lbIdx06, cartpole_QP_solver_dllbcc06);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub06, cartpole_QP_solver_sub06, cartpole_QP_solver_lubbysub06, cartpole_QP_solver_dzcc06, cartpole_QP_solver_ubIdx06, cartpole_QP_solver_dlubcc06);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A7, cartpole_QP_solver_dzcc06, cartpole_QP_solver_ccrhsp06, cartpole_QP_solver_sp06, cartpole_QP_solver_lp06, cartpole_QP_solver_dlp_cc06);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl07, cartpole_QP_solver_slb07, cartpole_QP_solver_llbbyslb07, cartpole_QP_solver_dzcc07, cartpole_QP_solver_lbIdx07, cartpole_QP_solver_dllbcc07);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub07, cartpole_QP_solver_sub07, cartpole_QP_solver_lubbysub07, cartpole_QP_solver_dzcc07, cartpole_QP_solver_ubIdx07, cartpole_QP_solver_dlubcc07);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A8, cartpole_QP_solver_dzcc07, cartpole_QP_solver_ccrhsp07, cartpole_QP_solver_sp07, cartpole_QP_solver_lp07, cartpole_QP_solver_dlp_cc07);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl08, cartpole_QP_solver_slb08, cartpole_QP_solver_llbbyslb08, cartpole_QP_solver_dzcc08, cartpole_QP_solver_lbIdx08, cartpole_QP_solver_dllbcc08);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub08, cartpole_QP_solver_sub08, cartpole_QP_solver_lubbysub08, cartpole_QP_solver_dzcc08, cartpole_QP_solver_ubIdx08, cartpole_QP_solver_dlubcc08);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A9, cartpole_QP_solver_dzcc08, cartpole_QP_solver_ccrhsp08, cartpole_QP_solver_sp08, cartpole_QP_solver_lp08, cartpole_QP_solver_dlp_cc08);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl09, cartpole_QP_solver_slb09, cartpole_QP_solver_llbbyslb09, cartpole_QP_solver_dzcc09, cartpole_QP_solver_lbIdx09, cartpole_QP_solver_dllbcc09);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub09, cartpole_QP_solver_sub09, cartpole_QP_solver_lubbysub09, cartpole_QP_solver_dzcc09, cartpole_QP_solver_ubIdx09, cartpole_QP_solver_dlubcc09);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A10, cartpole_QP_solver_dzcc09, cartpole_QP_solver_ccrhsp09, cartpole_QP_solver_sp09, cartpole_QP_solver_lp09, cartpole_QP_solver_dlp_cc09);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl10, cartpole_QP_solver_slb10, cartpole_QP_solver_llbbyslb10, cartpole_QP_solver_dzcc10, cartpole_QP_solver_lbIdx10, cartpole_QP_solver_dllbcc10);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub10, cartpole_QP_solver_sub10, cartpole_QP_solver_lubbysub10, cartpole_QP_solver_dzcc10, cartpole_QP_solver_ubIdx10, cartpole_QP_solver_dlubcc10);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A11, cartpole_QP_solver_dzcc10, cartpole_QP_solver_ccrhsp10, cartpole_QP_solver_sp10, cartpole_QP_solver_lp10, cartpole_QP_solver_dlp_cc10);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl11, cartpole_QP_solver_slb11, cartpole_QP_solver_llbbyslb11, cartpole_QP_solver_dzcc11, cartpole_QP_solver_lbIdx11, cartpole_QP_solver_dllbcc11);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub11, cartpole_QP_solver_sub11, cartpole_QP_solver_lubbysub11, cartpole_QP_solver_dzcc11, cartpole_QP_solver_ubIdx11, cartpole_QP_solver_dlubcc11);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A12, cartpole_QP_solver_dzcc11, cartpole_QP_solver_ccrhsp11, cartpole_QP_solver_sp11, cartpole_QP_solver_lp11, cartpole_QP_solver_dlp_cc11);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl12, cartpole_QP_solver_slb12, cartpole_QP_solver_llbbyslb12, cartpole_QP_solver_dzcc12, cartpole_QP_solver_lbIdx12, cartpole_QP_solver_dllbcc12);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub12, cartpole_QP_solver_sub12, cartpole_QP_solver_lubbysub12, cartpole_QP_solver_dzcc12, cartpole_QP_solver_ubIdx12, cartpole_QP_solver_dlubcc12);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A13, cartpole_QP_solver_dzcc12, cartpole_QP_solver_ccrhsp12, cartpole_QP_solver_sp12, cartpole_QP_solver_lp12, cartpole_QP_solver_dlp_cc12);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl13, cartpole_QP_solver_slb13, cartpole_QP_solver_llbbyslb13, cartpole_QP_solver_dzcc13, cartpole_QP_solver_lbIdx13, cartpole_QP_solver_dllbcc13);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub13, cartpole_QP_solver_sub13, cartpole_QP_solver_lubbysub13, cartpole_QP_solver_dzcc13, cartpole_QP_solver_ubIdx13, cartpole_QP_solver_dlubcc13);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A14, cartpole_QP_solver_dzcc13, cartpole_QP_solver_ccrhsp13, cartpole_QP_solver_sp13, cartpole_QP_solver_lp13, cartpole_QP_solver_dlp_cc13);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl14, cartpole_QP_solver_slb14, cartpole_QP_solver_llbbyslb14, cartpole_QP_solver_dzcc14, cartpole_QP_solver_lbIdx14, cartpole_QP_solver_dllbcc14);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub14, cartpole_QP_solver_sub14, cartpole_QP_solver_lubbysub14, cartpole_QP_solver_dzcc14, cartpole_QP_solver_ubIdx14, cartpole_QP_solver_dlubcc14);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A15, cartpole_QP_solver_dzcc14, cartpole_QP_solver_ccrhsp14, cartpole_QP_solver_sp14, cartpole_QP_solver_lp14, cartpole_QP_solver_dlp_cc14);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl15, cartpole_QP_solver_slb15, cartpole_QP_solver_llbbyslb15, cartpole_QP_solver_dzcc15, cartpole_QP_solver_lbIdx15, cartpole_QP_solver_dllbcc15);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub15, cartpole_QP_solver_sub15, cartpole_QP_solver_lubbysub15, cartpole_QP_solver_dzcc15, cartpole_QP_solver_ubIdx15, cartpole_QP_solver_dlubcc15);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A16, cartpole_QP_solver_dzcc15, cartpole_QP_solver_ccrhsp15, cartpole_QP_solver_sp15, cartpole_QP_solver_lp15, cartpole_QP_solver_dlp_cc15);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl16, cartpole_QP_solver_slb16, cartpole_QP_solver_llbbyslb16, cartpole_QP_solver_dzcc16, cartpole_QP_solver_lbIdx16, cartpole_QP_solver_dllbcc16);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub16, cartpole_QP_solver_sub16, cartpole_QP_solver_lubbysub16, cartpole_QP_solver_dzcc16, cartpole_QP_solver_ubIdx16, cartpole_QP_solver_dlubcc16);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A17, cartpole_QP_solver_dzcc16, cartpole_QP_solver_ccrhsp16, cartpole_QP_solver_sp16, cartpole_QP_solver_lp16, cartpole_QP_solver_dlp_cc16);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl17, cartpole_QP_solver_slb17, cartpole_QP_solver_llbbyslb17, cartpole_QP_solver_dzcc17, cartpole_QP_solver_lbIdx17, cartpole_QP_solver_dllbcc17);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub17, cartpole_QP_solver_sub17, cartpole_QP_solver_lubbysub17, cartpole_QP_solver_dzcc17, cartpole_QP_solver_ubIdx17, cartpole_QP_solver_dlubcc17);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_15(params->A18, cartpole_QP_solver_dzcc17, cartpole_QP_solver_ccrhsp17, cartpole_QP_solver_sp17, cartpole_QP_solver_lp17, cartpole_QP_solver_dlp_cc17);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_1(cartpole_QP_solver_ccrhsl18, cartpole_QP_solver_slb18, cartpole_QP_solver_llbbyslb18, cartpole_QP_solver_dzcc18, cartpole_QP_solver_lbIdx18, cartpole_QP_solver_dllbcc18);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_1(cartpole_QP_solver_ccrhsub18, cartpole_QP_solver_sub18, cartpole_QP_solver_lubbysub18, cartpole_QP_solver_dzcc18, cartpole_QP_solver_ubIdx18, cartpole_QP_solver_dlubcc18);
cartpole_QP_solver_LA_DENSE_MVMSUB5_20_14(params->A19, cartpole_QP_solver_dzcc18, cartpole_QP_solver_ccrhsp18, cartpole_QP_solver_sp18, cartpole_QP_solver_lp18, cartpole_QP_solver_dlp_cc18);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(cartpole_QP_solver_ccrhsl19, cartpole_QP_solver_slb19, cartpole_QP_solver_llbbyslb19, cartpole_QP_solver_dzcc19, cartpole_QP_solver_lbIdx19, cartpole_QP_solver_dllbcc19);
cartpole_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(cartpole_QP_solver_ccrhsub19, cartpole_QP_solver_sub19, cartpole_QP_solver_lubbysub19, cartpole_QP_solver_dzcc19, cartpole_QP_solver_ubIdx19, cartpole_QP_solver_dlubcc19);
cartpole_QP_solver_LA_VSUB7_436(cartpole_QP_solver_l, cartpole_QP_solver_ccrhs, cartpole_QP_solver_s, cartpole_QP_solver_dl_cc, cartpole_QP_solver_ds_cc);
cartpole_QP_solver_LA_VADD_288(cartpole_QP_solver_dz_cc, cartpole_QP_solver_dz_aff);
cartpole_QP_solver_LA_VADD_112(cartpole_QP_solver_dv_cc, cartpole_QP_solver_dv_aff);
cartpole_QP_solver_LA_VADD_436(cartpole_QP_solver_dl_cc, cartpole_QP_solver_dl_aff);
cartpole_QP_solver_LA_VADD_436(cartpole_QP_solver_ds_cc, cartpole_QP_solver_ds_aff);
info->lsit_cc = cartpole_QP_solver_LINESEARCH_BACKTRACKING_COMBINED(cartpole_QP_solver_z, cartpole_QP_solver_v, cartpole_QP_solver_l, cartpole_QP_solver_s, cartpole_QP_solver_dz_cc, cartpole_QP_solver_dv_cc, cartpole_QP_solver_dl_cc, cartpole_QP_solver_ds_cc, &info->step_cc, &info->mu);
if( info->lsit_cc == cartpole_QP_solver_NOPROGRESS ){
PRINTTEXT("Line search could not proceed at iteration %d, exiting.\n",info->it+1);
exitcode = cartpole_QP_solver_NOPROGRESS; break;
}
info->it++;
}
output->z1[0] = cartpole_QP_solver_z00[0];
output->z1[1] = cartpole_QP_solver_z00[1];
output->z1[2] = cartpole_QP_solver_z00[2];
output->z1[3] = cartpole_QP_solver_z00[3];
output->z1[4] = cartpole_QP_solver_z00[4];
output->z1[5] = cartpole_QP_solver_z00[10];
output->z2[0] = cartpole_QP_solver_z01[0];
output->z2[1] = cartpole_QP_solver_z01[1];
output->z2[2] = cartpole_QP_solver_z01[2];
output->z2[3] = cartpole_QP_solver_z01[3];
output->z2[4] = cartpole_QP_solver_z01[4];
output->z2[5] = cartpole_QP_solver_z01[10];
output->z3[0] = cartpole_QP_solver_z02[0];
output->z3[1] = cartpole_QP_solver_z02[1];
output->z3[2] = cartpole_QP_solver_z02[2];
output->z3[3] = cartpole_QP_solver_z02[3];
output->z3[4] = cartpole_QP_solver_z02[4];
output->z3[5] = cartpole_QP_solver_z02[10];
output->z4[0] = cartpole_QP_solver_z03[0];
output->z4[1] = cartpole_QP_solver_z03[1];
output->z4[2] = cartpole_QP_solver_z03[2];
output->z4[3] = cartpole_QP_solver_z03[3];
output->z4[4] = cartpole_QP_solver_z03[4];
output->z4[5] = cartpole_QP_solver_z03[10];
output->z5[0] = cartpole_QP_solver_z04[0];
output->z5[1] = cartpole_QP_solver_z04[1];
output->z5[2] = cartpole_QP_solver_z04[2];
output->z5[3] = cartpole_QP_solver_z04[3];
output->z5[4] = cartpole_QP_solver_z04[4];
output->z5[5] = cartpole_QP_solver_z04[10];
output->z6[0] = cartpole_QP_solver_z05[0];
output->z6[1] = cartpole_QP_solver_z05[1];
output->z6[2] = cartpole_QP_solver_z05[2];
output->z6[3] = cartpole_QP_solver_z05[3];
output->z6[4] = cartpole_QP_solver_z05[4];
output->z6[5] = cartpole_QP_solver_z05[10];
output->z7[0] = cartpole_QP_solver_z06[0];
output->z7[1] = cartpole_QP_solver_z06[1];
output->z7[2] = cartpole_QP_solver_z06[2];
output->z7[3] = cartpole_QP_solver_z06[3];
output->z7[4] = cartpole_QP_solver_z06[4];
output->z7[5] = cartpole_QP_solver_z06[10];
output->z8[0] = cartpole_QP_solver_z07[0];
output->z8[1] = cartpole_QP_solver_z07[1];
output->z8[2] = cartpole_QP_solver_z07[2];
output->z8[3] = cartpole_QP_solver_z07[3];
output->z8[4] = cartpole_QP_solver_z07[4];
output->z8[5] = cartpole_QP_solver_z07[10];
output->z9[0] = cartpole_QP_solver_z08[0];
output->z9[1] = cartpole_QP_solver_z08[1];
output->z9[2] = cartpole_QP_solver_z08[2];
output->z9[3] = cartpole_QP_solver_z08[3];
output->z9[4] = cartpole_QP_solver_z08[4];
output->z9[5] = cartpole_QP_solver_z08[10];
output->z10[0] = cartpole_QP_solver_z09[0];
output->z10[1] = cartpole_QP_solver_z09[1];
output->z10[2] = cartpole_QP_solver_z09[2];
output->z10[3] = cartpole_QP_solver_z09[3];
output->z10[4] = cartpole_QP_solver_z09[4];
output->z10[5] = cartpole_QP_solver_z09[10];
output->z11[0] = cartpole_QP_solver_z10[0];
output->z11[1] = cartpole_QP_solver_z10[1];
output->z11[2] = cartpole_QP_solver_z10[2];
output->z11[3] = cartpole_QP_solver_z10[3];
output->z11[4] = cartpole_QP_solver_z10[4];
output->z11[5] = cartpole_QP_solver_z10[10];
output->z12[0] = cartpole_QP_solver_z11[0];
output->z12[1] = cartpole_QP_solver_z11[1];
output->z12[2] = cartpole_QP_solver_z11[2];
output->z12[3] = cartpole_QP_solver_z11[3];
output->z12[4] = cartpole_QP_solver_z11[4];
output->z12[5] = cartpole_QP_solver_z11[10];
output->z13[0] = cartpole_QP_solver_z12[0];
output->z13[1] = cartpole_QP_solver_z12[1];
output->z13[2] = cartpole_QP_solver_z12[2];
output->z13[3] = cartpole_QP_solver_z12[3];
output->z13[4] = cartpole_QP_solver_z12[4];
output->z13[5] = cartpole_QP_solver_z12[10];
output->z14[0] = cartpole_QP_solver_z13[0];
output->z14[1] = cartpole_QP_solver_z13[1];
output->z14[2] = cartpole_QP_solver_z13[2];
output->z14[3] = cartpole_QP_solver_z13[3];
output->z14[4] = cartpole_QP_solver_z13[4];
output->z14[5] = cartpole_QP_solver_z13[10];
output->z15[0] = cartpole_QP_solver_z14[0];
output->z15[1] = cartpole_QP_solver_z14[1];
output->z15[2] = cartpole_QP_solver_z14[2];
output->z15[3] = cartpole_QP_solver_z14[3];
output->z15[4] = cartpole_QP_solver_z14[4];
output->z15[5] = cartpole_QP_solver_z14[10];
output->z16[0] = cartpole_QP_solver_z15[0];
output->z16[1] = cartpole_QP_solver_z15[1];
output->z16[2] = cartpole_QP_solver_z15[2];
output->z16[3] = cartpole_QP_solver_z15[3];
output->z16[4] = cartpole_QP_solver_z15[4];
output->z16[5] = cartpole_QP_solver_z15[10];
output->z17[0] = cartpole_QP_solver_z16[0];
output->z17[1] = cartpole_QP_solver_z16[1];
output->z17[2] = cartpole_QP_solver_z16[2];
output->z17[3] = cartpole_QP_solver_z16[3];
output->z17[4] = cartpole_QP_solver_z16[4];
output->z17[5] = cartpole_QP_solver_z16[10];
output->z18[0] = cartpole_QP_solver_z17[0];
output->z18[1] = cartpole_QP_solver_z17[1];
output->z18[2] = cartpole_QP_solver_z17[2];
output->z18[3] = cartpole_QP_solver_z17[3];
output->z18[4] = cartpole_QP_solver_z17[4];
output->z18[5] = cartpole_QP_solver_z17[10];
output->z19[0] = cartpole_QP_solver_z18[0];
output->z19[1] = cartpole_QP_solver_z18[1];
output->z19[2] = cartpole_QP_solver_z18[2];
output->z19[3] = cartpole_QP_solver_z18[3];
output->z19[4] = cartpole_QP_solver_z18[4];
output->z19[5] = cartpole_QP_solver_z18[9];
output->z20[0] = cartpole_QP_solver_z19[0];
output->z20[1] = cartpole_QP_solver_z19[1];
output->z20[2] = cartpole_QP_solver_z19[2];
output->z20[3] = cartpole_QP_solver_z19[3];

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
