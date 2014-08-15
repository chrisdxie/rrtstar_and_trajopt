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

#include "../include/double_integrator_QP_solver_CD.h"

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

typedef struct double_integrator_QP_solver_CD_timer{
	LARGE_INTEGER tic;
	LARGE_INTEGER toc;
	LARGE_INTEGER freq;
} double_integrator_QP_solver_CD_timer;


void double_integrator_QP_solver_CD_tic(double_integrator_QP_solver_CD_timer* t)
{
	QueryPerformanceFrequency(&t->freq);
	QueryPerformanceCounter(&t->tic);
}



double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_toc(double_integrator_QP_solver_CD_timer* t)
{
	QueryPerformanceCounter(&t->toc);
	return ((t->toc.QuadPart - t->tic.QuadPart) / (double_integrator_QP_solver_CD_FLOAT)t->freq.QuadPart);
}


/* WE ARE ON THE MAC */
#elif (defined __APPLE__)
#include <mach/mach_time.h>


/* Use MAC OSX  mach_time for timing */
typedef struct double_integrator_QP_solver_CD_timer{
	uint64_t tic;
	uint64_t toc;
	mach_timebase_info_data_t tinfo;

} double_integrator_QP_solver_CD_timer;


void double_integrator_QP_solver_CD_tic(double_integrator_QP_solver_CD_timer* t)
{
    /* read current clock cycles */
    t->tic = mach_absolute_time();
}



double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_toc(double_integrator_QP_solver_CD_timer* t)
{
    uint64_t duration; /* elapsed time in clock cycles*/
    t->toc = mach_absolute_time();
	duration = t->toc - t->tic;

    /*conversion from clock cycles to nanoseconds*/
    mach_timebase_info(&(t->tinfo));
    duration *= t->tinfo.numer;
    duration /= t->tinfo.denom;

    return (double_integrator_QP_solver_CD_FLOAT)duration / 1000000000;
}

/* WE ARE ON SOME TEXAS INSTRUMENTS PLATFORM */
#elif (defined __TI_COMPILER_VERSION__)

/* TimeStamps */
#include <c6x.h> /* make use of TSCL, TSCH */


typedef struct double_integrator_QP_solver_CD_timer{
	unsigned long long tic;
	unsigned long long toc;
} double_integrator_QP_solver_CD_timer;


void double_integrator_QP_solver_CD_tic(double_integrator_QP_solver_CD_timer* t)
{
	TSCL = 0;	/* Initiate CPU timer by writing any val to TSCL */
	t->tic = _itoll( TSCH, TSCL );
}



double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_toc(double_integrator_QP_solver_CD_timer* t)
{
	t->toc = _itoll( TSCH, TSCL );
	unsigned long long t0;
	unsigned long long overhead;
	t0 = _itoll( TSCH, TSCL );
	overhead = _itoll( TSCH, TSCL )  - t0;

	return (double_integrator_QP_solver_CD_FLOAT)(t->toc - t->tic - overhead) / 1000000000;
}



/* WE ARE ON SOME OTHER UNIX/LINUX SYSTEM */
#else

/* Use POSIX clocl_gettime() for timing on non-Windows machines */
#include <time.h>
typedef struct double_integrator_QP_solver_CD_timer{
	struct timespec tic;
	struct timespec toc;
} double_integrator_QP_solver_CD_timer;


/* read current time */
void double_integrator_QP_solver_CD_tic(double_integrator_QP_solver_CD_timer* t)
{
	clock_gettime(CLOCK_MONOTONIC, &t->tic);
}



/* return time passed since last call to tic on this timer */
double double_integrator_QP_solver_CD_toc(double_integrator_QP_solver_CD_timer* t)
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

	return (double_integrator_QP_solver_CD_FLOAT)temp.tv_sec + (double_integrator_QP_solver_CD_FLOAT)temp.tv_nsec / 1000000000;
}


#endif

/* LINEAR ALGEBRA LIBRARY ---------------------------------------------- */
/*
 * Initializes a vector of length 258 with a value.
 */
void double_integrator_QP_solver_CD_LA_INITIALIZEVECTOR_258(double_integrator_QP_solver_CD_FLOAT* vec, double_integrator_QP_solver_CD_FLOAT value)
{
	int i;
	for( i=0; i<258; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 74 with a value.
 */
void double_integrator_QP_solver_CD_LA_INITIALIZEVECTOR_74(double_integrator_QP_solver_CD_FLOAT* vec, double_integrator_QP_solver_CD_FLOAT value)
{
	int i;
	for( i=0; i<74; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 529 with a value.
 */
void double_integrator_QP_solver_CD_LA_INITIALIZEVECTOR_529(double_integrator_QP_solver_CD_FLOAT* vec, double_integrator_QP_solver_CD_FLOAT value)
{
	int i;
	for( i=0; i<529; i++ )
	{
		vec[i] = value;
	}
}


/* 
 * Calculates a dot product and adds it to a variable: z += x'*y; 
 * This function is for vectors of length 529.
 */
void double_integrator_QP_solver_CD_LA_DOTACC_529(double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *y, double_integrator_QP_solver_CD_FLOAT *z)
{
	int i;
	for( i=0; i<529; i++ ){
		*z += x[i]*y[i];
	}
}


/*
 * Calculates the gradient and the value for a quadratic function 0.5*z'*H*z + f'*z
 *
 * INPUTS:     H  - Symmetric Hessian, diag matrix of size [23 x 23]
 *             f  - column vector of size 23
 *             z  - column vector of size 23
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 23
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void double_integrator_QP_solver_CD_LA_DIAG_QUADFCN_23(double_integrator_QP_solver_CD_FLOAT* H, double_integrator_QP_solver_CD_FLOAT* f, double_integrator_QP_solver_CD_FLOAT* z, double_integrator_QP_solver_CD_FLOAT* grad, double_integrator_QP_solver_CD_FLOAT* value)
{
	int i;
	double_integrator_QP_solver_CD_FLOAT hz;	
	for( i=0; i<23; i++){
		hz = H[i]*z[i];
		grad[i] = hz + f[i];
		*value += 0.5*hz*z[i] + f[i]*z[i];
	}
}


/*
 * Calculates the gradient and the value for a quadratic function 0.5*z'*H*z + f'*z
 *
 * INPUTS:     H  - Symmetric Hessian, diag matrix of size [21 x 21]
 *             f  - column vector of size 21
 *             z  - column vector of size 21
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 21
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void double_integrator_QP_solver_CD_LA_DIAG_QUADFCN_21(double_integrator_QP_solver_CD_FLOAT* H, double_integrator_QP_solver_CD_FLOAT* f, double_integrator_QP_solver_CD_FLOAT* z, double_integrator_QP_solver_CD_FLOAT* grad, double_integrator_QP_solver_CD_FLOAT* value)
{
	int i;
	double_integrator_QP_solver_CD_FLOAT hz;	
	for( i=0; i<21; i++){
		hz = H[i]*z[i];
		grad[i] = hz + f[i];
		*value += 0.5*hz*z[i] + f[i]*z[i];
	}
}


/*
 * Calculates the gradient and the value for a quadratic function 0.5*z'*H*z + f'*z
 *
 * INPUTS:     H  - Symmetric Hessian, diag matrix of size [7 x 7]
 *             f  - column vector of size 7
 *             z  - column vector of size 7
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 7
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void double_integrator_QP_solver_CD_LA_DIAG_QUADFCN_7(double_integrator_QP_solver_CD_FLOAT* H, double_integrator_QP_solver_CD_FLOAT* f, double_integrator_QP_solver_CD_FLOAT* z, double_integrator_QP_solver_CD_FLOAT* grad, double_integrator_QP_solver_CD_FLOAT* value)
{
	int i;
	double_integrator_QP_solver_CD_FLOAT hz;	
	for( i=0; i<7; i++){
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
void double_integrator_QP_solver_CD_LA_DENSE_MVMSUB3_7_23_23(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *l, double_integrator_QP_solver_CD_FLOAT *r, double_integrator_QP_solver_CD_FLOAT *z, double_integrator_QP_solver_CD_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;
	double_integrator_QP_solver_CD_FLOAT AxBu[7];
	double_integrator_QP_solver_CD_FLOAT norm = *y;
	double_integrator_QP_solver_CD_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<7; i++ ){
		AxBu[i] = A[k++]*x[0] + B[m++]*u[0];
	}	
	for( j=1; j<23; j++ ){		
		for( i=0; i<7; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<23; n++ ){
		for( i=0; i<7; i++ ){
			AxBu[i] += B[m++]*u[n];
		}		
	}

	for( i=0; i<7; i++ ){
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
void double_integrator_QP_solver_CD_LA_DENSE_MVMSUB3_7_23_21(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *l, double_integrator_QP_solver_CD_FLOAT *r, double_integrator_QP_solver_CD_FLOAT *z, double_integrator_QP_solver_CD_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;
	double_integrator_QP_solver_CD_FLOAT AxBu[7];
	double_integrator_QP_solver_CD_FLOAT norm = *y;
	double_integrator_QP_solver_CD_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<7; i++ ){
		AxBu[i] = A[k++]*x[0] + B[m++]*u[0];
	}	
	for( j=1; j<23; j++ ){		
		for( i=0; i<7; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<21; n++ ){
		for( i=0; i<7; i++ ){
			AxBu[i] += B[m++]*u[n];
		}		
	}

	for( i=0; i<7; i++ ){
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
void double_integrator_QP_solver_CD_LA_DENSE_DIAGZERO_MVMSUB3_4_21_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *l, double_integrator_QP_solver_CD_FLOAT *r, double_integrator_QP_solver_CD_FLOAT *z, double_integrator_QP_solver_CD_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	double_integrator_QP_solver_CD_FLOAT AxBu[4];
	double_integrator_QP_solver_CD_FLOAT norm = *y;
	double_integrator_QP_solver_CD_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<4; i++ ){
		AxBu[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<21; j++ ){		
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
 * Matrix vector multiplication y = M'*x where M is of size [7 x 23]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_CD_LA_DENSE_MTVM_7_23(double_integrator_QP_solver_CD_FLOAT *M, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *y)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<23; i++ ){
		y[i] = 0;
		for( j=0; j<7; j++ ){
			y[i] += M[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [7 x 23]
 * and B is of size [7 x 23]
 * and stored in column major format. Note the transposes of A and B!
 */
void double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *y, double_integrator_QP_solver_CD_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	int n;
	int m = 0;
	for( i=0; i<23; i++ ){
		z[i] = 0;
		for( j=0; j<7; j++ ){
			z[i] += A[k++]*x[j];
		}
		for( n=0; n<7; n++ ){
			z[i] += B[m++]*y[n];
		}
	}
}


/*
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [4 x 21]
 * and B is of size [7 x 21]
 * and stored in column major format. Note the transposes of A and B!
 */
void double_integrator_QP_solver_CD_LA_DENSE_MTVM2_4_21_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *y, double_integrator_QP_solver_CD_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	int n;
	int m = 0;
	for( i=0; i<21; i++ ){
		z[i] = 0;
		for( j=0; j<4; j++ ){
			z[i] += A[k++]*x[j];
		}
		for( n=0; n<7; n++ ){
			z[i] += B[m++]*y[n];
		}
	}
}


/*
 * Matrix vector multiplication y = M'*x where M is of size [4 x 7]
 * and stored in diagzero format. Note the transpose of M!
 */
void double_integrator_QP_solver_CD_LA_DIAGZERO_MTVM_4_7(double_integrator_QP_solver_CD_FLOAT *M, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *y)
{
	int i;
	for( i=0; i<7; i++ ){
		y[i] = M[i]*x[i];
	}
}


/*
 * Vector subtraction and addition.
 *	 Input: five vectors t, tidx, u, v, w and two scalars z and r
 *	 Output: y = t(tidx) - u + w
 *           z = z - v'*x;
 *           r = max([norm(y,inf), z]);
 * for vectors of length 7. Output z is of course scalar.
 */
void double_integrator_QP_solver_CD_LA_VSUBADD3_7(double_integrator_QP_solver_CD_FLOAT* t, double_integrator_QP_solver_CD_FLOAT* u, int* uidx, double_integrator_QP_solver_CD_FLOAT* v, double_integrator_QP_solver_CD_FLOAT* w, double_integrator_QP_solver_CD_FLOAT* y, double_integrator_QP_solver_CD_FLOAT* z, double_integrator_QP_solver_CD_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_CD_FLOAT norm = *r;
	double_integrator_QP_solver_CD_FLOAT vx = 0;
	double_integrator_QP_solver_CD_FLOAT x;
	for( i=0; i<7; i++){
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
void double_integrator_QP_solver_CD_LA_VSUBADD2_6(double_integrator_QP_solver_CD_FLOAT* t, int* tidx, double_integrator_QP_solver_CD_FLOAT* u, double_integrator_QP_solver_CD_FLOAT* v, double_integrator_QP_solver_CD_FLOAT* w, double_integrator_QP_solver_CD_FLOAT* y, double_integrator_QP_solver_CD_FLOAT* z, double_integrator_QP_solver_CD_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_CD_FLOAT norm = *r;
	double_integrator_QP_solver_CD_FLOAT vx = 0;
	double_integrator_QP_solver_CD_FLOAT x;
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
void double_integrator_QP_solver_CD_LA_MVSUBADD_34_23(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *s, double_integrator_QP_solver_CD_FLOAT *l, double_integrator_QP_solver_CD_FLOAT *r, double_integrator_QP_solver_CD_FLOAT *z, double_integrator_QP_solver_CD_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	double_integrator_QP_solver_CD_FLOAT Ax[34];
	double_integrator_QP_solver_CD_FLOAT Axlessb;
	double_integrator_QP_solver_CD_FLOAT norm = *y;
	double_integrator_QP_solver_CD_FLOAT lAxlessb = 0;

	/* do A*x first */
	for( i=0; i<34; i++ ){
		Ax[i] = A[k++]*x[0];				
	}	
	for( j=1; j<23; j++ ){		
		for( i=0; i<34; i++ ){
			Ax[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<34; i++ ){
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
 * for vectors of length 6. Output z is of course scalar.
 */
void double_integrator_QP_solver_CD_LA_VSUBADD3_6(double_integrator_QP_solver_CD_FLOAT* t, double_integrator_QP_solver_CD_FLOAT* u, int* uidx, double_integrator_QP_solver_CD_FLOAT* v, double_integrator_QP_solver_CD_FLOAT* w, double_integrator_QP_solver_CD_FLOAT* y, double_integrator_QP_solver_CD_FLOAT* z, double_integrator_QP_solver_CD_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_CD_FLOAT norm = *r;
	double_integrator_QP_solver_CD_FLOAT vx = 0;
	double_integrator_QP_solver_CD_FLOAT x;
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
 * Computes r = A*x - b + s
 * and      y = max([norm(r,inf), y])
 * and      z -= l'*(Ax-b)
 * where A is stored in column major format
 */
void double_integrator_QP_solver_CD_LA_MVSUBADD_34_21(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *s, double_integrator_QP_solver_CD_FLOAT *l, double_integrator_QP_solver_CD_FLOAT *r, double_integrator_QP_solver_CD_FLOAT *z, double_integrator_QP_solver_CD_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	double_integrator_QP_solver_CD_FLOAT Ax[34];
	double_integrator_QP_solver_CD_FLOAT Axlessb;
	double_integrator_QP_solver_CD_FLOAT norm = *y;
	double_integrator_QP_solver_CD_FLOAT lAxlessb = 0;

	/* do A*x first */
	for( i=0; i<34; i++ ){
		Ax[i] = A[k++]*x[0];				
	}	
	for( j=1; j<21; j++ ){		
		for( i=0; i<34; i++ ){
			Ax[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<34; i++ ){
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
void double_integrator_QP_solver_CD_LA_VSUBADD3_4(double_integrator_QP_solver_CD_FLOAT* t, double_integrator_QP_solver_CD_FLOAT* u, int* uidx, double_integrator_QP_solver_CD_FLOAT* v, double_integrator_QP_solver_CD_FLOAT* w, double_integrator_QP_solver_CD_FLOAT* y, double_integrator_QP_solver_CD_FLOAT* z, double_integrator_QP_solver_CD_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_CD_FLOAT norm = *r;
	double_integrator_QP_solver_CD_FLOAT vx = 0;
	double_integrator_QP_solver_CD_FLOAT x;
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
void double_integrator_QP_solver_CD_LA_VSUBADD2_4(double_integrator_QP_solver_CD_FLOAT* t, int* tidx, double_integrator_QP_solver_CD_FLOAT* u, double_integrator_QP_solver_CD_FLOAT* v, double_integrator_QP_solver_CD_FLOAT* w, double_integrator_QP_solver_CD_FLOAT* y, double_integrator_QP_solver_CD_FLOAT* z, double_integrator_QP_solver_CD_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_CD_FLOAT norm = *r;
	double_integrator_QP_solver_CD_FLOAT vx = 0;
	double_integrator_QP_solver_CD_FLOAT x;
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
 * Computes r = A*x - b + s
 * and      y = max([norm(r,inf), y])
 * and      z -= l'*(Ax-b)
 * where A is stored in column major format
 */
void double_integrator_QP_solver_CD_LA_MVSUBADD_14_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *s, double_integrator_QP_solver_CD_FLOAT *l, double_integrator_QP_solver_CD_FLOAT *r, double_integrator_QP_solver_CD_FLOAT *z, double_integrator_QP_solver_CD_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	double_integrator_QP_solver_CD_FLOAT Ax[14];
	double_integrator_QP_solver_CD_FLOAT Axlessb;
	double_integrator_QP_solver_CD_FLOAT norm = *y;
	double_integrator_QP_solver_CD_FLOAT lAxlessb = 0;

	/* do A*x first */
	for( i=0; i<14; i++ ){
		Ax[i] = A[k++]*x[0];				
	}	
	for( j=1; j<7; j++ ){		
		for( i=0; i<14; i++ ){
			Ax[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<14; i++ ){
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
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 23
 * Returns also L/S, a value that is often used elsewhere.
 */
void double_integrator_QP_solver_CD_LA_INEQ_B_GRAD_23_7_6(double_integrator_QP_solver_CD_FLOAT *lu, double_integrator_QP_solver_CD_FLOAT *su, double_integrator_QP_solver_CD_FLOAT *ru, double_integrator_QP_solver_CD_FLOAT *ll, double_integrator_QP_solver_CD_FLOAT *sl, double_integrator_QP_solver_CD_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_CD_FLOAT *grad, double_integrator_QP_solver_CD_FLOAT *lubysu, double_integrator_QP_solver_CD_FLOAT *llbysl)
{
	int i;
	for( i=0; i<23; i++ ){
		grad[i] = 0;
	}
	for( i=0; i<7; i++ ){		
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
void double_integrator_QP_solver_CD_LA_INEQ_P_34_23(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *lp, double_integrator_QP_solver_CD_FLOAT *sp, double_integrator_QP_solver_CD_FLOAT *rip, double_integrator_QP_solver_CD_FLOAT *grad, double_integrator_QP_solver_CD_FLOAT *lpbysp)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_CD_FLOAT lsr[34];
	
	/* do (L/S)*ri first */
	for( j=0; j<34; j++ ){
		lpbysp[j] = lp[j] / sp[j];
		lsr[j] = lpbysp[j]*rip[j];
	}

	for( i=0; i<23; i++ ){		
		for( j=0; j<34; j++ ){
			grad[i] += A[k++]*lsr[j];
		}
	}
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 23
 * Returns also L/S, a value that is often used elsewhere.
 */
void double_integrator_QP_solver_CD_LA_INEQ_B_GRAD_23_6_6(double_integrator_QP_solver_CD_FLOAT *lu, double_integrator_QP_solver_CD_FLOAT *su, double_integrator_QP_solver_CD_FLOAT *ru, double_integrator_QP_solver_CD_FLOAT *ll, double_integrator_QP_solver_CD_FLOAT *sl, double_integrator_QP_solver_CD_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_CD_FLOAT *grad, double_integrator_QP_solver_CD_FLOAT *lubysu, double_integrator_QP_solver_CD_FLOAT *llbysl)
{
	int i;
	for( i=0; i<23; i++ ){
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
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 21
 * Returns also L/S, a value that is often used elsewhere.
 */
void double_integrator_QP_solver_CD_LA_INEQ_B_GRAD_21_6_6(double_integrator_QP_solver_CD_FLOAT *lu, double_integrator_QP_solver_CD_FLOAT *su, double_integrator_QP_solver_CD_FLOAT *ru, double_integrator_QP_solver_CD_FLOAT *ll, double_integrator_QP_solver_CD_FLOAT *sl, double_integrator_QP_solver_CD_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_CD_FLOAT *grad, double_integrator_QP_solver_CD_FLOAT *lubysu, double_integrator_QP_solver_CD_FLOAT *llbysl)
{
	int i;
	for( i=0; i<21; i++ ){
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
void double_integrator_QP_solver_CD_LA_INEQ_P_34_21(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *lp, double_integrator_QP_solver_CD_FLOAT *sp, double_integrator_QP_solver_CD_FLOAT *rip, double_integrator_QP_solver_CD_FLOAT *grad, double_integrator_QP_solver_CD_FLOAT *lpbysp)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_CD_FLOAT lsr[34];
	
	/* do (L/S)*ri first */
	for( j=0; j<34; j++ ){
		lpbysp[j] = lp[j] / sp[j];
		lsr[j] = lpbysp[j]*rip[j];
	}

	for( i=0; i<21; i++ ){		
		for( j=0; j<34; j++ ){
			grad[i] += A[k++]*lsr[j];
		}
	}
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 7
 * Returns also L/S, a value that is often used elsewhere.
 */
void double_integrator_QP_solver_CD_LA_INEQ_B_GRAD_7_4_4(double_integrator_QP_solver_CD_FLOAT *lu, double_integrator_QP_solver_CD_FLOAT *su, double_integrator_QP_solver_CD_FLOAT *ru, double_integrator_QP_solver_CD_FLOAT *ll, double_integrator_QP_solver_CD_FLOAT *sl, double_integrator_QP_solver_CD_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_CD_FLOAT *grad, double_integrator_QP_solver_CD_FLOAT *lubysu, double_integrator_QP_solver_CD_FLOAT *llbysl)
{
	int i;
	for( i=0; i<7; i++ ){
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
 * Special function for gradient of inequality constraints
 * Calculates grad += A'*(L/S)*rI
 */
void double_integrator_QP_solver_CD_LA_INEQ_P_14_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *lp, double_integrator_QP_solver_CD_FLOAT *sp, double_integrator_QP_solver_CD_FLOAT *rip, double_integrator_QP_solver_CD_FLOAT *grad, double_integrator_QP_solver_CD_FLOAT *lpbysp)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_CD_FLOAT lsr[14];
	
	/* do (L/S)*ri first */
	for( j=0; j<14; j++ ){
		lpbysp[j] = lp[j] / sp[j];
		lsr[j] = lpbysp[j]*rip[j];
	}

	for( i=0; i<7; i++ ){		
		for( j=0; j<14; j++ ){
			grad[i] += A[k++]*lsr[j];
		}
	}
}


/*
 * Addition of three vectors  z = u + w + v
 * of length 258.
 */
void double_integrator_QP_solver_CD_LA_VVADD3_258(double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *v, double_integrator_QP_solver_CD_FLOAT *w, double_integrator_QP_solver_CD_FLOAT *z)
{
	int i;
	for( i=0; i<258; i++ ){
		z[i] = u[i] + v[i] + w[i];
	}
}


/*
 * Special function to compute the Dense positive definite 
 * augmented Hessian for block size 23.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void double_integrator_QP_solver_CD_LA_INEQ_DENSE_DIAG_HESS_23_7_6(double_integrator_QP_solver_CD_FLOAT *H, double_integrator_QP_solver_CD_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_CD_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_CD_FLOAT *Phi)
{
	int i;
	int j;
	int k = 0;
	
	/* copy diagonal of H into PHI and set lower part of PHI = 0*/
	for( i=0; i<23; i++ ){
		for( j=0; j<i; j++ ){
			Phi[k++] = 0;
		}		
		/* we are on the diagonal */
		Phi[k++] = H[i];
	}

	/* add llbysl onto Phi where necessary */
	for( i=0; i<7; i++ ){
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
 * A is stored in column major format and is of size [34 x 23]
 * Phi is of size [23 x 23].
 */
void double_integrator_QP_solver_CD_LA_DENSE_ADDMTDM_34_23(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *d, double_integrator_QP_solver_CD_FLOAT *X)
{    
    int i,j,k,ii,di;
    double_integrator_QP_solver_CD_FLOAT x;
    
    di = 0; ii = 0;
    for( i=0; i<23; i++ ){        
        for( j=0; j<=i; j++ ){
            x = 0;
            for( k=0; k<34; k++ ){
                x += A[i*34+k]*A[j*34+k]*d[k];
            }
            X[ii+j] += x;
        }
        ii += ++di;
    }
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 23.
 */
void double_integrator_QP_solver_CD_LA_DENSE_CHOL2_23(double_integrator_QP_solver_CD_FLOAT *A)
{
    int i, j, k, di, dj;
	 int ii, jj;
    double_integrator_QP_solver_CD_FLOAT l;
    double_integrator_QP_solver_CD_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<23; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += A[ii+k]*A[ii+k];
        }        
        
        Mii = A[ii+i] - l;
        
#if double_integrator_QP_solver_CD_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
        for( j=i+1; j<23; j++ ){
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
 * where A is to be computed and is of size [7 x 23],
 * B is given and of size [7 x 23], L is a lower tri-
 * angular matrix of size 23 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_FLOAT *L, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    double_integrator_QP_solver_CD_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<23; j++ ){        
        for( i=0; i<7; i++ ){
            a = B[j*7+i];
            for( k=0; k<j; k++ ){
                a -= A[k*7+i]*L[ii+k];
            }

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

            A[j*7+i] = a/L[ii+j];
        }
        ii += ++di;
    }
}


/**
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 23.
 */
void double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_FLOAT *L, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_CD_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<23; i++ ){
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
 * augmented Hessian for block size 23.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void double_integrator_QP_solver_CD_LA_INEQ_DENSE_DIAG_HESS_23_6_6(double_integrator_QP_solver_CD_FLOAT *H, double_integrator_QP_solver_CD_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_CD_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_CD_FLOAT *Phi)
{
	int i;
	int j;
	int k = 0;
	
	/* copy diagonal of H into PHI and set lower part of PHI = 0*/
	for( i=0; i<23; i++ ){
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
 * Compute C = A*B' where 
 *
 *	size(A) = [7 x 23]
 *  size(B) = [7 x 23]
 * 
 * and all matrices are stored in column major format.
 *
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE.  
 * 
 */
void double_integrator_QP_solver_CD_LA_DENSE_MMTM_7_23_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *C)
{
    int i, j, k;
    double_integrator_QP_solver_CD_FLOAT temp;
    
    for( i=0; i<7; i++ ){        
        for( j=0; j<7; j++ ){
            temp = 0; 
            for( k=0; k<23; k++ ){
                temp += A[k*7+i]*B[k*7+j];
            }						
            C[j*7+i] = temp;
        }
    }
}


/*
 * Special function to compute the Dense positive definite 
 * augmented Hessian for block size 21.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void double_integrator_QP_solver_CD_LA_INEQ_DENSE_DIAG_HESS_21_6_6(double_integrator_QP_solver_CD_FLOAT *H, double_integrator_QP_solver_CD_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_CD_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_CD_FLOAT *Phi)
{
	int i;
	int j;
	int k = 0;
	
	/* copy diagonal of H into PHI and set lower part of PHI = 0*/
	for( i=0; i<21; i++ ){
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
 * A is stored in column major format and is of size [34 x 21]
 * Phi is of size [21 x 21].
 */
void double_integrator_QP_solver_CD_LA_DENSE_ADDMTDM_34_21(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *d, double_integrator_QP_solver_CD_FLOAT *X)
{    
    int i,j,k,ii,di;
    double_integrator_QP_solver_CD_FLOAT x;
    
    di = 0; ii = 0;
    for( i=0; i<21; i++ ){        
        for( j=0; j<=i; j++ ){
            x = 0;
            for( k=0; k<34; k++ ){
                x += A[i*34+k]*A[j*34+k]*d[k];
            }
            X[ii+j] += x;
        }
        ii += ++di;
    }
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 21.
 */
void double_integrator_QP_solver_CD_LA_DENSE_CHOL2_21(double_integrator_QP_solver_CD_FLOAT *A)
{
    int i, j, k, di, dj;
	 int ii, jj;
    double_integrator_QP_solver_CD_FLOAT l;
    double_integrator_QP_solver_CD_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<21; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += A[ii+k]*A[ii+k];
        }        
        
        Mii = A[ii+i] - l;
        
#if double_integrator_QP_solver_CD_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
        for( j=i+1; j<21; j++ ){
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
 * where A is to be computed and is of size [4 x 21],
 * B is given and of size [4 x 21], L is a lower tri-
 * angular matrix of size 21 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_4_21(double_integrator_QP_solver_CD_FLOAT *L, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    double_integrator_QP_solver_CD_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<21; j++ ){        
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
 * where A is to be computed and is of size [7 x 21],
 * B is given and of size [7 x 21], L is a lower tri-
 * angular matrix of size 21 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_21(double_integrator_QP_solver_CD_FLOAT *L, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    double_integrator_QP_solver_CD_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<21; j++ ){        
        for( i=0; i<7; i++ ){
            a = B[j*7+i];
            for( k=0; k<j; k++ ){
                a -= A[k*7+i]*L[ii+k];
            }

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

            A[j*7+i] = a/L[ii+j];
        }
        ii += ++di;
    }
}


/**
 * Compute C = A*B' where 
 *
 *	size(A) = [7 x 21]
 *  size(B) = [4 x 21]
 * 
 * and all matrices are stored in column major format.
 *
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE.  
 * 
 */
void double_integrator_QP_solver_CD_LA_DENSE_MMTM_7_21_4(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *C)
{
    int i, j, k;
    double_integrator_QP_solver_CD_FLOAT temp;
    
    for( i=0; i<7; i++ ){        
        for( j=0; j<4; j++ ){
            temp = 0; 
            for( k=0; k<21; k++ ){
                temp += A[k*7+i]*B[k*4+j];
            }						
            C[j*7+i] = temp;
        }
    }
}


/**
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 21.
 */
void double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_21(double_integrator_QP_solver_CD_FLOAT *L, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_CD_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<21; i++ ){
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
 * augmented Hessian for block size 5.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void double_integrator_QP_solver_CD_LA_INEQ_DENSE_DIAG_HESS_5_4_4(double_integrator_QP_solver_CD_FLOAT *H, double_integrator_QP_solver_CD_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_CD_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_CD_FLOAT *Phi)
{
	int i;
	int j;
	int k = 0;
	
	/* copy diagonal of H into PHI and set lower part of PHI = 0*/
	for( i=0; i<5; i++ ){
		for( j=0; j<i; j++ ){
			Phi[k++] = 0;
		}		
		/* we are on the diagonal */
		Phi[k++] = H[i];
	}

	/* add llbysl onto Phi where necessary */
	for( i=0; i<4; i++ ){
		j = lbIdx[i];
		Phi[((j+1)*(j+2))/2-1] += llbysl[i];
	}

	/* add lubysu onto Phi where necessary */
	for( i=0; i<4; i++){
		j = ubIdx[i];
		Phi[((j+1)*(j+2))/2-1] +=  lubysu[i];
	}

}


/**
 * Compute X = X + A'*D*A, where A is a general full matrix, D is
 * is a diagonal matrix stored in the vector d and X is a symmetric
 * positive definite matrix in lower triangular storage format. 
 * A is stored in column major format and is of size [14 x 7]
 * Phi is of size [7 x 7].
 */
void double_integrator_QP_solver_CD_LA_DENSE_ADDMTDM_14_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *d, double_integrator_QP_solver_CD_FLOAT *X)
{    
    int i,j,k,ii,di;
    double_integrator_QP_solver_CD_FLOAT x;
    
    di = 0; ii = 0;
    for( i=0; i<7; i++ ){        
        for( j=0; j<=i; j++ ){
            x = 0;
            for( k=0; k<14; k++ ){
                x += A[i*14+k]*A[j*14+k]*d[k];
            }
            X[ii+j] += x;
        }
        ii += ++di;
    }
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 7.
 */
void double_integrator_QP_solver_CD_LA_DENSE_CHOL2_7(double_integrator_QP_solver_CD_FLOAT *A)
{
    int i, j, k, di, dj;
	 int ii, jj;
    double_integrator_QP_solver_CD_FLOAT l;
    double_integrator_QP_solver_CD_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<7; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += A[ii+k]*A[ii+k];
        }        
        
        Mii = A[ii+i] - l;
        
#if double_integrator_QP_solver_CD_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
        for( j=i+1; j<7; j++ ){
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
 * where A is to be computed and is of size [4 x 7],
 * B is given and of size [4 x 7] stored in 
 * diagzero storage format, L is a lower tri-
 * angular matrix of size 7 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_CD_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_4_7(double_integrator_QP_solver_CD_FLOAT *L, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    double_integrator_QP_solver_CD_FLOAT a;
	
	/*
	* The matrix A has the form
	*
	* d u u u r r r r r 
	* 0 d u u r r r r r 
	* 0 0 d u r r r r r 
	* 0 0 0 d r r r r r
	*
	* |Part1|| Part 2 |
	* 
	* d: diagonal
	* u: upper
	* r: right
	*/
	
	
    /* Part 1 */
    ii=0; di=0;
    for( j=0; j<4; j++ ){        
        for( i=0; i<j; i++ ){
            /* Calculate part of A which is non-zero and not diagonal "u"
             * i < j */
            a = 0;
			
            for( k=i; k<j; k++ ){
                a -= A[k*4+i]*L[ii+k];
            }
            A[j*4+i] = a/L[ii+j];
        }
        /* do the diagonal "d"
         * i = j */
        A[j*4+j] = B[i]/L[ii+j];
        
        /* fill lower triangular part with zeros "0"
         * n > i > j */
        for( i=j+1     ; i < 4; i++ ){
            A[j*4+i] = 0;
        }
        
        /* increment index of L */
        ii += ++di;	
    }
	
	/* Part 2 */ 
	for( j=4; j<7; j++ ){        
        for( i=0; i<4; i++ ){
            /* Calculate part of A which is non-zero and not diagonal "r" */
            a = 0;
			
            for( k=i; k<j; k++ ){
                a -= A[k*4+i]*L[ii+k];
            }
            A[j*4+i] = a/L[ii+j];
        }
        
        /* increment index of L */
        ii += ++di;	
    }
	
	
	
}


/**
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 7.
 */
void double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_FLOAT *L, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_CD_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<7; i++ ){
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
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [7 x 23] in column
 * storage format, and B is of size [7 x 23] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_CD_LA_DENSE_MMT2_7_23_23(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_CD_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<7; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<23; k++ ){
                ltemp += A[k*7+i]*A[k*7+j];
            }			
			for( k=0; k<23; k++ ){
                ltemp += B[k*7+i]*B[k*7+j];
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
void double_integrator_QP_solver_CD_LA_DENSE_MVMSUB2_7_23_23(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<7; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<23; j++ ){		
		for( i=0; i<7; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<23; n++ ){
		for( i=0; i<7; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [7 x 23] in column
 * storage format, and B is of size [7 x 21] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_CD_LA_DENSE_MMT2_7_23_21(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_CD_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<7; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<23; k++ ){
                ltemp += A[k*7+i]*A[k*7+j];
            }			
			for( k=0; k<21; k++ ){
                ltemp += B[k*7+i]*B[k*7+j];
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
void double_integrator_QP_solver_CD_LA_DENSE_MVMSUB2_7_23_21(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<7; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<23; j++ ){		
		for( i=0; i<7; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<21; n++ ){
		for( i=0; i<7; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [4 x 21] in column
 * storage format, and B is of size [4 x 7] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_CD_LA_DENSE_MMT2_4_21_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_CD_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<4; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<21; k++ ){
                ltemp += A[k*4+i]*A[k*4+j];
            }			
			for( k=0; k<7; k++ ){
                ltemp += B[k*4+i]*B[k*4+j];
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
void double_integrator_QP_solver_CD_LA_DENSE_MVMSUB2_4_21_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<4; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<21; j++ ){		
		for( i=0; i<4; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<7; n++ ){
		for( i=0; i<4; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 7 and outputting
 * the Cholesky factor to matrix L in lower triangular format.
 */
void double_integrator_QP_solver_CD_LA_DENSE_CHOL_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    double_integrator_QP_solver_CD_FLOAT l;
    double_integrator_QP_solver_CD_FLOAT Mii;

	/* copy A to L first and then operate on L */
	/* COULD BE OPTIMIZED */
	ii=0; di=0;
	for( i=0; i<7; i++ ){
		for( j=0; j<=i; j++ ){
			L[ii+j] = A[ii+j];
		}
		ii += ++di;
	}    
	
	/* factor L */
	ii=0; di=0;
    for( i=0; i<7; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += L[ii+k]*L[ii+k];
        }        
        
        Mii = L[ii+i] - l;
        
#if double_integrator_QP_solver_CD_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
        for( j=i+1; j<7; j++ ){
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
 * Forward substitution for the matrix equation A*L' = B'
 * where A is to be computed and is of size [7 x 7],
 * B is given and of size [7 x 7], L is a lower tri-
 * angular matrix of size 7 stored in lower triangular 
 * storage format. Note the transpose of L AND B!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_CD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_CD_FLOAT *L, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *A)
{
    int i,j,k,ii,di;
    double_integrator_QP_solver_CD_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<7; j++ ){        
        for( i=0; i<7; i++ ){
            a = B[i*7+j];
            for( k=0; k<j; k++ ){
                a -= A[k*7+i]*L[ii+k];
            }    

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

			A[j*7+i] = a/L[ii+j];			
        }
        ii += ++di;
    }
}


/**
 * Compute L = L - A*A', where L is lower triangular of size 7
 * and A is a dense matrix of size [7 x 7] in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_CD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_CD_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<7; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<7; k++ ){
                ltemp += A[k*7+i]*A[k*7+j];
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
void double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<7; i++ ){
		r[i] = b[i] - A[k++]*x[0];
	}	
	for( j=1; j<7; j++ ){		
		for( i=0; i<7; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/** 
 * Forward substitution for the matrix equation A*L' = B'
 * where A is to be computed and is of size [4 x 7],
 * B is given and of size [4 x 7], L is a lower tri-
 * angular matrix of size 7 stored in lower triangular 
 * storage format. Note the transpose of L AND B!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_CD_LA_DENSE_MATRIXTFORWARDSUB_4_7(double_integrator_QP_solver_CD_FLOAT *L, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *A)
{
    int i,j,k,ii,di;
    double_integrator_QP_solver_CD_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<7; j++ ){        
        for( i=0; i<4; i++ ){
            a = B[i*7+j];
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
 * and A is a dense matrix of size [4 x 7] in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_CD_LA_DENSE_MMTSUB_4_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_CD_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<4; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<7; k++ ){
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
void double_integrator_QP_solver_CD_LA_DENSE_CHOL_4(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    double_integrator_QP_solver_CD_FLOAT l;
    double_integrator_QP_solver_CD_FLOAT Mii;

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
        
#if double_integrator_QP_solver_CD_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
void double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_4_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<4; i++ ){
		r[i] = b[i] - A[k++]*x[0];
	}	
	for( j=1; j<7; j++ ){		
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
void double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_4(double_integrator_QP_solver_CD_FLOAT *L, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_CD_FLOAT yel;
            
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
void double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_4(double_integrator_QP_solver_CD_FLOAT *L, double_integrator_QP_solver_CD_FLOAT *y, double_integrator_QP_solver_CD_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_CD_FLOAT xel;    
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
 * Matrix vector multiplication y = b - M'*x where M is of size [4 x 7]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_4_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *r)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<7; i++ ){
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
 * All involved dimensions are 7.
 */
void double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_FLOAT *L, double_integrator_QP_solver_CD_FLOAT *y, double_integrator_QP_solver_CD_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_CD_FLOAT xel;    
	int start = 21;
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 6;
    for( i=6; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 6;
        for( j=6; j>i; j-- ){
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
 * Matrix vector multiplication y = b - M'*x where M is of size [7 x 7]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *r)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<7; i++ ){
		r[i] = b[i];
		for( j=0; j<7; j++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Vector subtraction z = -x - y for vectors of length 258.
 */
void double_integrator_QP_solver_CD_LA_VSUB2_258(double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *y, double_integrator_QP_solver_CD_FLOAT *z)
{
	int i;
	for( i=0; i<258; i++){
		z[i] = -x[i] - y[i];
	}
}


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * lower triangular matrix of size 23 in lower triangular
 * storage format.
 */
void double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_FLOAT *L, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_CD_FLOAT y[23];
    double_integrator_QP_solver_CD_FLOAT yel,xel;
	int start = 253;
            
    /* first solve Ly = b by forward substitution */
     ii = 0; di = 0;
    for( i=0; i<23; i++ ){
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
    ii = start; di = 22;
    for( i=22; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 22;
        for( j=22; j>i; j-- ){
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
 * lower triangular matrix of size 21 in lower triangular
 * storage format.
 */
void double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_21(double_integrator_QP_solver_CD_FLOAT *L, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_CD_FLOAT y[21];
    double_integrator_QP_solver_CD_FLOAT yel,xel;
	int start = 210;
            
    /* first solve Ly = b by forward substitution */
     ii = 0; di = 0;
    for( i=0; i<21; i++ ){
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
    ii = start; di = 20;
    for( i=20; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 20;
        for( j=20; j>i; j-- ){
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
 * lower triangular matrix of size 7 in lower triangular
 * storage format.
 */
void double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_7(double_integrator_QP_solver_CD_FLOAT *L, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_CD_FLOAT y[7];
    double_integrator_QP_solver_CD_FLOAT yel,xel;
	int start = 21;
            
    /* first solve Ly = b by forward substitution */
     ii = 0; di = 0;
    for( i=0; i<7; i++ ){
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
    ii = start; di = 6;
    for( i=6; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 6;
        for( j=6; j>i; j-- ){
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
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 7,
 * and x has length 23 and is indexed through yidx.
 */
void double_integrator_QP_solver_CD_LA_VSUB_INDEXED_7(double_integrator_QP_solver_CD_FLOAT *x, int* xidx, double_integrator_QP_solver_CD_FLOAT *y, double_integrator_QP_solver_CD_FLOAT *z)
{
	int i;
	for( i=0; i<7; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 7.
 */
void double_integrator_QP_solver_CD_LA_VSUB3_7(double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *v, double_integrator_QP_solver_CD_FLOAT *w, double_integrator_QP_solver_CD_FLOAT *x)
{
	int i;
	for( i=0; i<7; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = -x - y(yidx) where y is of length 23
 * and z, x and yidx are of length 6.
 */
void double_integrator_QP_solver_CD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *y, int* yidx, double_integrator_QP_solver_CD_FLOAT *z)
{
	int i;
	for( i=0; i<6; i++){
		z[i] = -x[i] - y[yidx[i]];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 6.
 */
void double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *v, double_integrator_QP_solver_CD_FLOAT *w, double_integrator_QP_solver_CD_FLOAT *x)
{
	int i;
	for( i=0; i<6; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/* 
 * Computes r = -b - A*x
 * where A is stored in column major format
 */
void double_integrator_QP_solver_CD_LA_DENSE_MVMSUB4_34_23(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<34; i++ ){
		r[i] = -b[i] - A[k++]*x[0];
	}	
	for( j=1; j<23; j++ ){		
		for( i=0; i<34; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 34.
 */
void double_integrator_QP_solver_CD_LA_VSUB3_34(double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *v, double_integrator_QP_solver_CD_FLOAT *w, double_integrator_QP_solver_CD_FLOAT *x)
{
	int i;
	for( i=0; i<34; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 6,
 * and x has length 23 and is indexed through yidx.
 */
void double_integrator_QP_solver_CD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_CD_FLOAT *x, int* xidx, double_integrator_QP_solver_CD_FLOAT *y, double_integrator_QP_solver_CD_FLOAT *z)
{
	int i;
	for( i=0; i<6; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/* 
 * Computes r = -b - A*x
 * where A is stored in column major format
 */
void double_integrator_QP_solver_CD_LA_DENSE_MVMSUB4_34_21(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<34; i++ ){
		r[i] = -b[i] - A[k++]*x[0];
	}	
	for( j=1; j<21; j++ ){		
		for( i=0; i<34; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 4,
 * and x has length 7 and is indexed through yidx.
 */
void double_integrator_QP_solver_CD_LA_VSUB_INDEXED_4(double_integrator_QP_solver_CD_FLOAT *x, int* xidx, double_integrator_QP_solver_CD_FLOAT *y, double_integrator_QP_solver_CD_FLOAT *z)
{
	int i;
	for( i=0; i<4; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 4.
 */
void double_integrator_QP_solver_CD_LA_VSUB3_4(double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *v, double_integrator_QP_solver_CD_FLOAT *w, double_integrator_QP_solver_CD_FLOAT *x)
{
	int i;
	for( i=0; i<4; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = -x - y(yidx) where y is of length 7
 * and z, x and yidx are of length 4.
 */
void double_integrator_QP_solver_CD_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *y, int* yidx, double_integrator_QP_solver_CD_FLOAT *z)
{
	int i;
	for( i=0; i<4; i++){
		z[i] = -x[i] - y[yidx[i]];
	}
}


/* 
 * Computes r = -b - A*x
 * where A is stored in column major format
 */
void double_integrator_QP_solver_CD_LA_DENSE_MVMSUB4_14_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<14; i++ ){
		r[i] = -b[i] - A[k++]*x[0];
	}	
	for( j=1; j<7; j++ ){		
		for( i=0; i<14; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 14.
 */
void double_integrator_QP_solver_CD_LA_VSUB3_14(double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *v, double_integrator_QP_solver_CD_FLOAT *w, double_integrator_QP_solver_CD_FLOAT *x)
{
	int i;
	for( i=0; i<14; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/**
 * Backtracking line search.
 * 
 * First determine the maximum line length by a feasibility line
 * search, i.e. a ~= argmax{ a \in [0...1] s.t. l+a*dl >= 0 and s+a*ds >= 0}.
 *
 * The function returns either the number of iterations or exits the error code
 * double_integrator_QP_solver_CD_NOPROGRESS (should be negative).
 */
int double_integrator_QP_solver_CD_LINESEARCH_BACKTRACKING_AFFINE(double_integrator_QP_solver_CD_FLOAT *l, double_integrator_QP_solver_CD_FLOAT *s, double_integrator_QP_solver_CD_FLOAT *dl, double_integrator_QP_solver_CD_FLOAT *ds, double_integrator_QP_solver_CD_FLOAT *a, double_integrator_QP_solver_CD_FLOAT *mu_aff)
{
    int i;
	int lsIt=1;    
    double_integrator_QP_solver_CD_FLOAT dltemp;
    double_integrator_QP_solver_CD_FLOAT dstemp;
    double_integrator_QP_solver_CD_FLOAT mya = 1.0;
    double_integrator_QP_solver_CD_FLOAT mymu;
        
    while( 1 ){                        

        /* 
         * Compute both snew and wnew together.
         * We compute also mu_affine along the way here, as the
         * values might be in registers, so it should be cheaper.
         */
        mymu = 0;
        for( i=0; i<529; i++ ){
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
        if( i == 529 ){
            break;
        } else {
            mya *= double_integrator_QP_solver_CD_SET_LS_SCALE_AFF;
            if( mya < double_integrator_QP_solver_CD_SET_LS_MINSTEP ){
                return double_integrator_QP_solver_CD_NOPROGRESS;
            }
        }
    }
    
    /* return new values and iteration counter */
    *a = mya;
    *mu_aff = mymu / (double_integrator_QP_solver_CD_FLOAT)529;
    return lsIt;
}


/*
 * Vector subtraction x = (u.*v - mu)*sigma where a is a scalar
*  and x,u,v are vectors of length 529.
 */
void double_integrator_QP_solver_CD_LA_VSUB5_529(double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *v, double_integrator_QP_solver_CD_FLOAT mu,  double_integrator_QP_solver_CD_FLOAT sigma, double_integrator_QP_solver_CD_FLOAT *x)
{
	int i;
	for( i=0; i<529; i++){
		x[i] = u[i]*v[i] - mu;
		x[i] *= sigma;
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 23,
 * u, su, uidx are of length 6 and v, sv, vidx are of length 7.
 */
void double_integrator_QP_solver_CD_LA_VSUB6_INDEXED_23_6_7(double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *su, int* uidx, double_integrator_QP_solver_CD_FLOAT *v, double_integrator_QP_solver_CD_FLOAT *sv, int* vidx, double_integrator_QP_solver_CD_FLOAT *x)
{
	int i;
	for( i=0; i<23; i++ ){
		x[i] = 0;
	}
	for( i=0; i<6; i++){
		x[uidx[i]] += u[i]/su[i];
	}
	for( i=0; i<7; i++){
		x[vidx[i]] -= v[i]/sv[i];
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 23,
 * u, su, uidx are of length 6 and v, sv, vidx are of length 6.
 */
void double_integrator_QP_solver_CD_LA_VSUB6_INDEXED_23_6_6(double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *su, int* uidx, double_integrator_QP_solver_CD_FLOAT *v, double_integrator_QP_solver_CD_FLOAT *sv, int* vidx, double_integrator_QP_solver_CD_FLOAT *x)
{
	int i;
	for( i=0; i<23; i++ ){
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
 * Matrix vector multiplication z = z + A'*(x./s) where A is of size [34 x 23]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_CD_LA_DENSE_MTVMADD2_34_23(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *s, double_integrator_QP_solver_CD_FLOAT *z)
{
	int i;
	int j;
	int k = 0; 
	double_integrator_QP_solver_CD_FLOAT temp[34];

	for( j=0; j<34; j++ ){
		temp[j] = x[j] / s[j];
	}

	for( i=0; i<23; i++ ){
		for( j=0; j<34; j++ ){
			z[i] += A[k++]*temp[j];
		}
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void double_integrator_QP_solver_CD_LA_DENSE_2MVMADD_7_23_23(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<7; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<23; j++ ){		
		for( i=0; i<7; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<23; n++ ){
		for( i=0; i<7; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 21,
 * u, su, uidx are of length 6 and v, sv, vidx are of length 6.
 */
void double_integrator_QP_solver_CD_LA_VSUB6_INDEXED_21_6_6(double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *su, int* uidx, double_integrator_QP_solver_CD_FLOAT *v, double_integrator_QP_solver_CD_FLOAT *sv, int* vidx, double_integrator_QP_solver_CD_FLOAT *x)
{
	int i;
	for( i=0; i<21; i++ ){
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
 * Matrix vector multiplication z = z + A'*(x./s) where A is of size [34 x 21]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_CD_LA_DENSE_MTVMADD2_34_21(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *s, double_integrator_QP_solver_CD_FLOAT *z)
{
	int i;
	int j;
	int k = 0; 
	double_integrator_QP_solver_CD_FLOAT temp[34];

	for( j=0; j<34; j++ ){
		temp[j] = x[j] / s[j];
	}

	for( i=0; i<21; i++ ){
		for( j=0; j<34; j++ ){
			z[i] += A[k++]*temp[j];
		}
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void double_integrator_QP_solver_CD_LA_DENSE_2MVMADD_7_23_21(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<7; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<23; j++ ){		
		for( i=0; i<7; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<21; n++ ){
		for( i=0; i<7; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 7,
 * u, su, uidx are of length 4 and v, sv, vidx are of length 4.
 */
void double_integrator_QP_solver_CD_LA_VSUB6_INDEXED_7_4_4(double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *su, int* uidx, double_integrator_QP_solver_CD_FLOAT *v, double_integrator_QP_solver_CD_FLOAT *sv, int* vidx, double_integrator_QP_solver_CD_FLOAT *x)
{
	int i;
	for( i=0; i<7; i++ ){
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
 * Matrix vector multiplication z = z + A'*(x./s) where A is of size [14 x 7]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_CD_LA_DENSE_MTVMADD2_14_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *s, double_integrator_QP_solver_CD_FLOAT *z)
{
	int i;
	int j;
	int k = 0; 
	double_integrator_QP_solver_CD_FLOAT temp[14];

	for( j=0; j<14; j++ ){
		temp[j] = x[j] / s[j];
	}

	for( i=0; i<7; i++ ){
		for( j=0; j<14; j++ ){
			z[i] += A[k++]*temp[j];
		}
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void double_integrator_QP_solver_CD_LA_DENSE_2MVMADD_4_21_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *B, double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<4; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<21; j++ ){		
		for( i=0; i<4; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<7; n++ ){
		for( i=0; i<4; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/*
 * Vector subtraction z = x - y for vectors of length 258.
 */
void double_integrator_QP_solver_CD_LA_VSUB_258(double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *y, double_integrator_QP_solver_CD_FLOAT *z)
{
	int i;
	for( i=0; i<258; i++){
		z[i] = x[i] - y[i];
	}
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 7 (length of y >= 7).
 */
void double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_CD_FLOAT *r, double_integrator_QP_solver_CD_FLOAT *s, double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *y, int* yidx, double_integrator_QP_solver_CD_FLOAT *z)
{
	int i;
	for( i=0; i<7; i++ ){
		z[i] = -r[i]/s[i] - u[i]*y[yidx[i]];
	}
}


/** 
 * Computes z = -r./s + u.*y(y)
 * where all vectors except of y are of length 6 (length of y >= 6).
 */
void double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_CD_FLOAT *r, double_integrator_QP_solver_CD_FLOAT *s, double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *y, int* yidx, double_integrator_QP_solver_CD_FLOAT *z)
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
void double_integrator_QP_solver_CD_LA_DENSE_MVMSUB5_34_23(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *s, double_integrator_QP_solver_CD_FLOAT *l, double_integrator_QP_solver_CD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_CD_FLOAT temp[34];

	
	for( i=0; i<34; i++ ){
		temp[i] = A[k++]*x[0];
	}
	

	for( j=1; j<23; j++ ){		
		for( i=0; i<34; i++ ){
			temp[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<34; i++ ){
		r[i] = (-b[i] + l[i]*temp[i])/s[i]; 
	}	
	
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 6 (length of y >= 6).
 */
void double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_CD_FLOAT *r, double_integrator_QP_solver_CD_FLOAT *s, double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *y, int* yidx, double_integrator_QP_solver_CD_FLOAT *z)
{
	int i;
	for( i=0; i<6; i++ ){
		z[i] = -r[i]/s[i] - u[i]*y[yidx[i]];
	}
}


/* 
 * Computes r = (-b + l.*(A*x))./s
 * where A is stored in column major format
 */
void double_integrator_QP_solver_CD_LA_DENSE_MVMSUB5_34_21(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *s, double_integrator_QP_solver_CD_FLOAT *l, double_integrator_QP_solver_CD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_CD_FLOAT temp[34];

	
	for( i=0; i<34; i++ ){
		temp[i] = A[k++]*x[0];
	}
	

	for( j=1; j<21; j++ ){		
		for( i=0; i<34; i++ ){
			temp[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<34; i++ ){
		r[i] = (-b[i] + l[i]*temp[i])/s[i]; 
	}	
	
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 4 (length of y >= 4).
 */
void double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_CD_FLOAT *r, double_integrator_QP_solver_CD_FLOAT *s, double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *y, int* yidx, double_integrator_QP_solver_CD_FLOAT *z)
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
void double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_CD_FLOAT *r, double_integrator_QP_solver_CD_FLOAT *s, double_integrator_QP_solver_CD_FLOAT *u, double_integrator_QP_solver_CD_FLOAT *y, int* yidx, double_integrator_QP_solver_CD_FLOAT *z)
{
	int i;
	for( i=0; i<4; i++ ){
		z[i] = -r[i]/s[i] + u[i]*y[yidx[i]];
	}
}


/* 
 * Computes r = (-b + l.*(A*x))./s
 * where A is stored in column major format
 */
void double_integrator_QP_solver_CD_LA_DENSE_MVMSUB5_14_7(double_integrator_QP_solver_CD_FLOAT *A, double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *b, double_integrator_QP_solver_CD_FLOAT *s, double_integrator_QP_solver_CD_FLOAT *l, double_integrator_QP_solver_CD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_CD_FLOAT temp[14];

	
	for( i=0; i<14; i++ ){
		temp[i] = A[k++]*x[0];
	}
	

	for( j=1; j<7; j++ ){		
		for( i=0; i<14; i++ ){
			temp[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<14; i++ ){
		r[i] = (-b[i] + l[i]*temp[i])/s[i]; 
	}	
	
}


/*
 * Computes ds = -l.\(r + s.*dl) for vectors of length 529.
 */
void double_integrator_QP_solver_CD_LA_VSUB7_529(double_integrator_QP_solver_CD_FLOAT *l, double_integrator_QP_solver_CD_FLOAT *r, double_integrator_QP_solver_CD_FLOAT *s, double_integrator_QP_solver_CD_FLOAT *dl, double_integrator_QP_solver_CD_FLOAT *ds)
{
	int i;
	for( i=0; i<529; i++){
		ds[i] = -(r[i] + s[i]*dl[i])/l[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 258.
 */
void double_integrator_QP_solver_CD_LA_VADD_258(double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *y)
{
	int i;
	for( i=0; i<258; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 74.
 */
void double_integrator_QP_solver_CD_LA_VADD_74(double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *y)
{
	int i;
	for( i=0; i<74; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 529.
 */
void double_integrator_QP_solver_CD_LA_VADD_529(double_integrator_QP_solver_CD_FLOAT *x, double_integrator_QP_solver_CD_FLOAT *y)
{
	int i;
	for( i=0; i<529; i++){
		x[i] += y[i];
	}
}


/**
 * Backtracking line search for combined predictor/corrector step.
 * Update on variables with safety factor gamma (to keep us away from
 * boundary).
 */
int double_integrator_QP_solver_CD_LINESEARCH_BACKTRACKING_COMBINED(double_integrator_QP_solver_CD_FLOAT *z, double_integrator_QP_solver_CD_FLOAT *v, double_integrator_QP_solver_CD_FLOAT *l, double_integrator_QP_solver_CD_FLOAT *s, double_integrator_QP_solver_CD_FLOAT *dz, double_integrator_QP_solver_CD_FLOAT *dv, double_integrator_QP_solver_CD_FLOAT *dl, double_integrator_QP_solver_CD_FLOAT *ds, double_integrator_QP_solver_CD_FLOAT *a, double_integrator_QP_solver_CD_FLOAT *mu)
{
    int i, lsIt=1;       
    double_integrator_QP_solver_CD_FLOAT dltemp;
    double_integrator_QP_solver_CD_FLOAT dstemp;    
    double_integrator_QP_solver_CD_FLOAT a_gamma;
            
    *a = 1.0;
    while( 1 ){                        

        /* check whether search criterion is fulfilled */
        for( i=0; i<529; i++ ){
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
        if( i == 529 ){
            break;
        } else {
            *a *= double_integrator_QP_solver_CD_SET_LS_SCALE;
            if( *a < double_integrator_QP_solver_CD_SET_LS_MINSTEP ){
                return double_integrator_QP_solver_CD_NOPROGRESS;
            }
        }
    }
    
    /* update variables with safety margin */
    a_gamma = (*a)*double_integrator_QP_solver_CD_SET_LS_MAXSTEP;
    
    /* primal variables */
    for( i=0; i<258; i++ ){
        z[i] += a_gamma*dz[i];
    }
    
    /* equality constraint multipliers */
    for( i=0; i<74; i++ ){
        v[i] += a_gamma*dv[i];
    }
    
    /* inequality constraint multipliers & slacks, also update mu */
    *mu = 0;
    for( i=0; i<529; i++ ){
        dltemp = l[i] + a_gamma*dl[i]; l[i] = dltemp;
        dstemp = s[i] + a_gamma*ds[i]; s[i] = dstemp;
        *mu += dltemp*dstemp;
    }
    
    *a = a_gamma;
    *mu /= (double_integrator_QP_solver_CD_FLOAT)529;
    return lsIt;
}




/* VARIABLE DEFINITIONS ------------------------------------------------ */
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_z[258];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_v[74];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_dz_aff[258];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_dv_aff[74];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_grad_cost[258];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_grad_eq[258];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rd[258];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_l[529];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_s[529];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_lbys[529];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_dl_aff[529];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_ds_aff[529];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_dz_cc[258];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_dv_cc[74];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_dl_cc[529];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_ds_cc[529];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_ccrhs[529];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_grad_ineq[258];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_H00[23] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_z00 = double_integrator_QP_solver_CD_z + 0;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzaff00 = double_integrator_QP_solver_CD_dz_aff + 0;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzcc00 = double_integrator_QP_solver_CD_dz_cc + 0;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_rd00 = double_integrator_QP_solver_CD_rd + 0;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lbyrd00[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_cost00 = double_integrator_QP_solver_CD_grad_cost + 0;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_eq00 = double_integrator_QP_solver_CD_grad_eq + 0;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_ineq00 = double_integrator_QP_solver_CD_grad_ineq + 0;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_ctv00[23];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_C00[161] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 1.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_v00 = double_integrator_QP_solver_CD_v + 0;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_re00[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_beta00[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_betacc00[7];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvaff00 = double_integrator_QP_solver_CD_dv_aff + 0;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvcc00 = double_integrator_QP_solver_CD_dv_cc + 0;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_V00[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Yd00[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ld00[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_yy00[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_bmy00[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_c00[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_CD_lbIdx00[7] = {0, 1, 2, 3, 4, 5, 12};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llb00 = double_integrator_QP_solver_CD_l + 0;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_slb00 = double_integrator_QP_solver_CD_s + 0;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llbbyslb00 = double_integrator_QP_solver_CD_lbys + 0;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rilb00[7];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbaff00 = double_integrator_QP_solver_CD_dl_aff + 0;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbaff00 = double_integrator_QP_solver_CD_ds_aff + 0;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbcc00 = double_integrator_QP_solver_CD_dl_cc + 0;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbcc00 = double_integrator_QP_solver_CD_ds_cc + 0;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsl00 = double_integrator_QP_solver_CD_ccrhs + 0;
int double_integrator_QP_solver_CD_ubIdx00[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lub00 = double_integrator_QP_solver_CD_l + 7;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sub00 = double_integrator_QP_solver_CD_s + 7;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lubbysub00 = double_integrator_QP_solver_CD_lbys + 7;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_riub00[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubaff00 = double_integrator_QP_solver_CD_dl_aff + 7;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubaff00 = double_integrator_QP_solver_CD_ds_aff + 7;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubcc00 = double_integrator_QP_solver_CD_dl_cc + 7;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubcc00 = double_integrator_QP_solver_CD_ds_cc + 7;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsub00 = double_integrator_QP_solver_CD_ccrhs + 7;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sp00 = double_integrator_QP_solver_CD_s + 13;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lp00 = double_integrator_QP_solver_CD_l + 13;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lpbysp00 = double_integrator_QP_solver_CD_lbys + 13;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_aff00 = double_integrator_QP_solver_CD_dl_aff + 13;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_aff00 = double_integrator_QP_solver_CD_ds_aff + 13;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_cc00 = double_integrator_QP_solver_CD_dl_cc + 13;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_cc00 = double_integrator_QP_solver_CD_ds_cc + 13;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsp00 = double_integrator_QP_solver_CD_ccrhs + 13;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rip00[34];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Phi00[276];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_z01 = double_integrator_QP_solver_CD_z + 23;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzaff01 = double_integrator_QP_solver_CD_dz_aff + 23;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzcc01 = double_integrator_QP_solver_CD_dz_cc + 23;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_rd01 = double_integrator_QP_solver_CD_rd + 23;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lbyrd01[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_cost01 = double_integrator_QP_solver_CD_grad_cost + 23;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_eq01 = double_integrator_QP_solver_CD_grad_eq + 23;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_ineq01 = double_integrator_QP_solver_CD_grad_ineq + 23;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_ctv01[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_v01 = double_integrator_QP_solver_CD_v + 7;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_re01[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_beta01[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_betacc01[7];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvaff01 = double_integrator_QP_solver_CD_dv_aff + 7;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvcc01 = double_integrator_QP_solver_CD_dv_cc + 7;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_V01[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Yd01[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ld01[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_yy01[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_bmy01[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_c01[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_CD_lbIdx01[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llb01 = double_integrator_QP_solver_CD_l + 47;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_slb01 = double_integrator_QP_solver_CD_s + 47;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llbbyslb01 = double_integrator_QP_solver_CD_lbys + 47;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rilb01[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbaff01 = double_integrator_QP_solver_CD_dl_aff + 47;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbaff01 = double_integrator_QP_solver_CD_ds_aff + 47;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbcc01 = double_integrator_QP_solver_CD_dl_cc + 47;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbcc01 = double_integrator_QP_solver_CD_ds_cc + 47;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsl01 = double_integrator_QP_solver_CD_ccrhs + 47;
int double_integrator_QP_solver_CD_ubIdx01[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lub01 = double_integrator_QP_solver_CD_l + 53;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sub01 = double_integrator_QP_solver_CD_s + 53;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lubbysub01 = double_integrator_QP_solver_CD_lbys + 53;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_riub01[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubaff01 = double_integrator_QP_solver_CD_dl_aff + 53;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubaff01 = double_integrator_QP_solver_CD_ds_aff + 53;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubcc01 = double_integrator_QP_solver_CD_dl_cc + 53;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubcc01 = double_integrator_QP_solver_CD_ds_cc + 53;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsub01 = double_integrator_QP_solver_CD_ccrhs + 53;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sp01 = double_integrator_QP_solver_CD_s + 59;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lp01 = double_integrator_QP_solver_CD_l + 59;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lpbysp01 = double_integrator_QP_solver_CD_lbys + 59;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_aff01 = double_integrator_QP_solver_CD_dl_aff + 59;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_aff01 = double_integrator_QP_solver_CD_ds_aff + 59;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_cc01 = double_integrator_QP_solver_CD_dl_cc + 59;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_cc01 = double_integrator_QP_solver_CD_ds_cc + 59;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsp01 = double_integrator_QP_solver_CD_ccrhs + 59;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rip01[34];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Phi01[276];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_D01[161] = {-1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_W01[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ysd01[49];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lsd01[49];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_z02 = double_integrator_QP_solver_CD_z + 46;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzaff02 = double_integrator_QP_solver_CD_dz_aff + 46;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzcc02 = double_integrator_QP_solver_CD_dz_cc + 46;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_rd02 = double_integrator_QP_solver_CD_rd + 46;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lbyrd02[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_cost02 = double_integrator_QP_solver_CD_grad_cost + 46;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_eq02 = double_integrator_QP_solver_CD_grad_eq + 46;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_ineq02 = double_integrator_QP_solver_CD_grad_ineq + 46;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_ctv02[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_v02 = double_integrator_QP_solver_CD_v + 14;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_re02[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_beta02[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_betacc02[7];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvaff02 = double_integrator_QP_solver_CD_dv_aff + 14;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvcc02 = double_integrator_QP_solver_CD_dv_cc + 14;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_V02[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Yd02[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ld02[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_yy02[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_bmy02[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_c02[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_CD_lbIdx02[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llb02 = double_integrator_QP_solver_CD_l + 93;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_slb02 = double_integrator_QP_solver_CD_s + 93;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llbbyslb02 = double_integrator_QP_solver_CD_lbys + 93;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rilb02[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbaff02 = double_integrator_QP_solver_CD_dl_aff + 93;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbaff02 = double_integrator_QP_solver_CD_ds_aff + 93;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbcc02 = double_integrator_QP_solver_CD_dl_cc + 93;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbcc02 = double_integrator_QP_solver_CD_ds_cc + 93;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsl02 = double_integrator_QP_solver_CD_ccrhs + 93;
int double_integrator_QP_solver_CD_ubIdx02[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lub02 = double_integrator_QP_solver_CD_l + 99;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sub02 = double_integrator_QP_solver_CD_s + 99;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lubbysub02 = double_integrator_QP_solver_CD_lbys + 99;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_riub02[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubaff02 = double_integrator_QP_solver_CD_dl_aff + 99;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubaff02 = double_integrator_QP_solver_CD_ds_aff + 99;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubcc02 = double_integrator_QP_solver_CD_dl_cc + 99;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubcc02 = double_integrator_QP_solver_CD_ds_cc + 99;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsub02 = double_integrator_QP_solver_CD_ccrhs + 99;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sp02 = double_integrator_QP_solver_CD_s + 105;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lp02 = double_integrator_QP_solver_CD_l + 105;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lpbysp02 = double_integrator_QP_solver_CD_lbys + 105;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_aff02 = double_integrator_QP_solver_CD_dl_aff + 105;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_aff02 = double_integrator_QP_solver_CD_ds_aff + 105;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_cc02 = double_integrator_QP_solver_CD_dl_cc + 105;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_cc02 = double_integrator_QP_solver_CD_ds_cc + 105;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsp02 = double_integrator_QP_solver_CD_ccrhs + 105;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rip02[34];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Phi02[276];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_W02[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ysd02[49];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lsd02[49];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_z03 = double_integrator_QP_solver_CD_z + 69;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzaff03 = double_integrator_QP_solver_CD_dz_aff + 69;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzcc03 = double_integrator_QP_solver_CD_dz_cc + 69;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_rd03 = double_integrator_QP_solver_CD_rd + 69;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lbyrd03[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_cost03 = double_integrator_QP_solver_CD_grad_cost + 69;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_eq03 = double_integrator_QP_solver_CD_grad_eq + 69;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_ineq03 = double_integrator_QP_solver_CD_grad_ineq + 69;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_ctv03[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_v03 = double_integrator_QP_solver_CD_v + 21;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_re03[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_beta03[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_betacc03[7];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvaff03 = double_integrator_QP_solver_CD_dv_aff + 21;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvcc03 = double_integrator_QP_solver_CD_dv_cc + 21;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_V03[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Yd03[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ld03[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_yy03[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_bmy03[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_c03[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_CD_lbIdx03[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llb03 = double_integrator_QP_solver_CD_l + 139;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_slb03 = double_integrator_QP_solver_CD_s + 139;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llbbyslb03 = double_integrator_QP_solver_CD_lbys + 139;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rilb03[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbaff03 = double_integrator_QP_solver_CD_dl_aff + 139;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbaff03 = double_integrator_QP_solver_CD_ds_aff + 139;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbcc03 = double_integrator_QP_solver_CD_dl_cc + 139;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbcc03 = double_integrator_QP_solver_CD_ds_cc + 139;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsl03 = double_integrator_QP_solver_CD_ccrhs + 139;
int double_integrator_QP_solver_CD_ubIdx03[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lub03 = double_integrator_QP_solver_CD_l + 145;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sub03 = double_integrator_QP_solver_CD_s + 145;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lubbysub03 = double_integrator_QP_solver_CD_lbys + 145;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_riub03[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubaff03 = double_integrator_QP_solver_CD_dl_aff + 145;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubaff03 = double_integrator_QP_solver_CD_ds_aff + 145;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubcc03 = double_integrator_QP_solver_CD_dl_cc + 145;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubcc03 = double_integrator_QP_solver_CD_ds_cc + 145;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsub03 = double_integrator_QP_solver_CD_ccrhs + 145;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sp03 = double_integrator_QP_solver_CD_s + 151;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lp03 = double_integrator_QP_solver_CD_l + 151;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lpbysp03 = double_integrator_QP_solver_CD_lbys + 151;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_aff03 = double_integrator_QP_solver_CD_dl_aff + 151;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_aff03 = double_integrator_QP_solver_CD_ds_aff + 151;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_cc03 = double_integrator_QP_solver_CD_dl_cc + 151;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_cc03 = double_integrator_QP_solver_CD_ds_cc + 151;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsp03 = double_integrator_QP_solver_CD_ccrhs + 151;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rip03[34];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Phi03[276];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_W03[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ysd03[49];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lsd03[49];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_z04 = double_integrator_QP_solver_CD_z + 92;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzaff04 = double_integrator_QP_solver_CD_dz_aff + 92;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzcc04 = double_integrator_QP_solver_CD_dz_cc + 92;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_rd04 = double_integrator_QP_solver_CD_rd + 92;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lbyrd04[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_cost04 = double_integrator_QP_solver_CD_grad_cost + 92;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_eq04 = double_integrator_QP_solver_CD_grad_eq + 92;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_ineq04 = double_integrator_QP_solver_CD_grad_ineq + 92;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_ctv04[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_v04 = double_integrator_QP_solver_CD_v + 28;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_re04[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_beta04[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_betacc04[7];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvaff04 = double_integrator_QP_solver_CD_dv_aff + 28;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvcc04 = double_integrator_QP_solver_CD_dv_cc + 28;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_V04[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Yd04[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ld04[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_yy04[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_bmy04[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_c04[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_CD_lbIdx04[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llb04 = double_integrator_QP_solver_CD_l + 185;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_slb04 = double_integrator_QP_solver_CD_s + 185;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llbbyslb04 = double_integrator_QP_solver_CD_lbys + 185;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rilb04[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbaff04 = double_integrator_QP_solver_CD_dl_aff + 185;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbaff04 = double_integrator_QP_solver_CD_ds_aff + 185;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbcc04 = double_integrator_QP_solver_CD_dl_cc + 185;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbcc04 = double_integrator_QP_solver_CD_ds_cc + 185;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsl04 = double_integrator_QP_solver_CD_ccrhs + 185;
int double_integrator_QP_solver_CD_ubIdx04[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lub04 = double_integrator_QP_solver_CD_l + 191;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sub04 = double_integrator_QP_solver_CD_s + 191;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lubbysub04 = double_integrator_QP_solver_CD_lbys + 191;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_riub04[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubaff04 = double_integrator_QP_solver_CD_dl_aff + 191;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubaff04 = double_integrator_QP_solver_CD_ds_aff + 191;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubcc04 = double_integrator_QP_solver_CD_dl_cc + 191;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubcc04 = double_integrator_QP_solver_CD_ds_cc + 191;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsub04 = double_integrator_QP_solver_CD_ccrhs + 191;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sp04 = double_integrator_QP_solver_CD_s + 197;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lp04 = double_integrator_QP_solver_CD_l + 197;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lpbysp04 = double_integrator_QP_solver_CD_lbys + 197;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_aff04 = double_integrator_QP_solver_CD_dl_aff + 197;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_aff04 = double_integrator_QP_solver_CD_ds_aff + 197;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_cc04 = double_integrator_QP_solver_CD_dl_cc + 197;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_cc04 = double_integrator_QP_solver_CD_ds_cc + 197;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsp04 = double_integrator_QP_solver_CD_ccrhs + 197;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rip04[34];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Phi04[276];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_W04[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ysd04[49];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lsd04[49];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_z05 = double_integrator_QP_solver_CD_z + 115;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzaff05 = double_integrator_QP_solver_CD_dz_aff + 115;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzcc05 = double_integrator_QP_solver_CD_dz_cc + 115;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_rd05 = double_integrator_QP_solver_CD_rd + 115;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lbyrd05[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_cost05 = double_integrator_QP_solver_CD_grad_cost + 115;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_eq05 = double_integrator_QP_solver_CD_grad_eq + 115;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_ineq05 = double_integrator_QP_solver_CD_grad_ineq + 115;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_ctv05[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_v05 = double_integrator_QP_solver_CD_v + 35;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_re05[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_beta05[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_betacc05[7];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvaff05 = double_integrator_QP_solver_CD_dv_aff + 35;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvcc05 = double_integrator_QP_solver_CD_dv_cc + 35;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_V05[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Yd05[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ld05[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_yy05[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_bmy05[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_c05[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_CD_lbIdx05[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llb05 = double_integrator_QP_solver_CD_l + 231;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_slb05 = double_integrator_QP_solver_CD_s + 231;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llbbyslb05 = double_integrator_QP_solver_CD_lbys + 231;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rilb05[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbaff05 = double_integrator_QP_solver_CD_dl_aff + 231;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbaff05 = double_integrator_QP_solver_CD_ds_aff + 231;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbcc05 = double_integrator_QP_solver_CD_dl_cc + 231;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbcc05 = double_integrator_QP_solver_CD_ds_cc + 231;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsl05 = double_integrator_QP_solver_CD_ccrhs + 231;
int double_integrator_QP_solver_CD_ubIdx05[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lub05 = double_integrator_QP_solver_CD_l + 237;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sub05 = double_integrator_QP_solver_CD_s + 237;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lubbysub05 = double_integrator_QP_solver_CD_lbys + 237;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_riub05[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubaff05 = double_integrator_QP_solver_CD_dl_aff + 237;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubaff05 = double_integrator_QP_solver_CD_ds_aff + 237;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubcc05 = double_integrator_QP_solver_CD_dl_cc + 237;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubcc05 = double_integrator_QP_solver_CD_ds_cc + 237;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsub05 = double_integrator_QP_solver_CD_ccrhs + 237;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sp05 = double_integrator_QP_solver_CD_s + 243;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lp05 = double_integrator_QP_solver_CD_l + 243;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lpbysp05 = double_integrator_QP_solver_CD_lbys + 243;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_aff05 = double_integrator_QP_solver_CD_dl_aff + 243;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_aff05 = double_integrator_QP_solver_CD_ds_aff + 243;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_cc05 = double_integrator_QP_solver_CD_dl_cc + 243;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_cc05 = double_integrator_QP_solver_CD_ds_cc + 243;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsp05 = double_integrator_QP_solver_CD_ccrhs + 243;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rip05[34];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Phi05[276];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_W05[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ysd05[49];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lsd05[49];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_z06 = double_integrator_QP_solver_CD_z + 138;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzaff06 = double_integrator_QP_solver_CD_dz_aff + 138;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzcc06 = double_integrator_QP_solver_CD_dz_cc + 138;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_rd06 = double_integrator_QP_solver_CD_rd + 138;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lbyrd06[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_cost06 = double_integrator_QP_solver_CD_grad_cost + 138;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_eq06 = double_integrator_QP_solver_CD_grad_eq + 138;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_ineq06 = double_integrator_QP_solver_CD_grad_ineq + 138;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_ctv06[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_v06 = double_integrator_QP_solver_CD_v + 42;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_re06[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_beta06[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_betacc06[7];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvaff06 = double_integrator_QP_solver_CD_dv_aff + 42;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvcc06 = double_integrator_QP_solver_CD_dv_cc + 42;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_V06[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Yd06[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ld06[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_yy06[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_bmy06[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_c06[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_CD_lbIdx06[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llb06 = double_integrator_QP_solver_CD_l + 277;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_slb06 = double_integrator_QP_solver_CD_s + 277;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llbbyslb06 = double_integrator_QP_solver_CD_lbys + 277;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rilb06[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbaff06 = double_integrator_QP_solver_CD_dl_aff + 277;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbaff06 = double_integrator_QP_solver_CD_ds_aff + 277;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbcc06 = double_integrator_QP_solver_CD_dl_cc + 277;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbcc06 = double_integrator_QP_solver_CD_ds_cc + 277;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsl06 = double_integrator_QP_solver_CD_ccrhs + 277;
int double_integrator_QP_solver_CD_ubIdx06[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lub06 = double_integrator_QP_solver_CD_l + 283;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sub06 = double_integrator_QP_solver_CD_s + 283;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lubbysub06 = double_integrator_QP_solver_CD_lbys + 283;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_riub06[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubaff06 = double_integrator_QP_solver_CD_dl_aff + 283;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubaff06 = double_integrator_QP_solver_CD_ds_aff + 283;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubcc06 = double_integrator_QP_solver_CD_dl_cc + 283;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubcc06 = double_integrator_QP_solver_CD_ds_cc + 283;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsub06 = double_integrator_QP_solver_CD_ccrhs + 283;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sp06 = double_integrator_QP_solver_CD_s + 289;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lp06 = double_integrator_QP_solver_CD_l + 289;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lpbysp06 = double_integrator_QP_solver_CD_lbys + 289;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_aff06 = double_integrator_QP_solver_CD_dl_aff + 289;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_aff06 = double_integrator_QP_solver_CD_ds_aff + 289;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_cc06 = double_integrator_QP_solver_CD_dl_cc + 289;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_cc06 = double_integrator_QP_solver_CD_ds_cc + 289;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsp06 = double_integrator_QP_solver_CD_ccrhs + 289;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rip06[34];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Phi06[276];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_W06[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ysd06[49];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lsd06[49];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_z07 = double_integrator_QP_solver_CD_z + 161;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzaff07 = double_integrator_QP_solver_CD_dz_aff + 161;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzcc07 = double_integrator_QP_solver_CD_dz_cc + 161;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_rd07 = double_integrator_QP_solver_CD_rd + 161;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lbyrd07[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_cost07 = double_integrator_QP_solver_CD_grad_cost + 161;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_eq07 = double_integrator_QP_solver_CD_grad_eq + 161;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_ineq07 = double_integrator_QP_solver_CD_grad_ineq + 161;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_ctv07[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_v07 = double_integrator_QP_solver_CD_v + 49;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_re07[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_beta07[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_betacc07[7];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvaff07 = double_integrator_QP_solver_CD_dv_aff + 49;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvcc07 = double_integrator_QP_solver_CD_dv_cc + 49;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_V07[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Yd07[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ld07[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_yy07[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_bmy07[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_c07[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_CD_lbIdx07[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llb07 = double_integrator_QP_solver_CD_l + 323;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_slb07 = double_integrator_QP_solver_CD_s + 323;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llbbyslb07 = double_integrator_QP_solver_CD_lbys + 323;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rilb07[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbaff07 = double_integrator_QP_solver_CD_dl_aff + 323;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbaff07 = double_integrator_QP_solver_CD_ds_aff + 323;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbcc07 = double_integrator_QP_solver_CD_dl_cc + 323;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbcc07 = double_integrator_QP_solver_CD_ds_cc + 323;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsl07 = double_integrator_QP_solver_CD_ccrhs + 323;
int double_integrator_QP_solver_CD_ubIdx07[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lub07 = double_integrator_QP_solver_CD_l + 329;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sub07 = double_integrator_QP_solver_CD_s + 329;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lubbysub07 = double_integrator_QP_solver_CD_lbys + 329;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_riub07[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubaff07 = double_integrator_QP_solver_CD_dl_aff + 329;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubaff07 = double_integrator_QP_solver_CD_ds_aff + 329;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubcc07 = double_integrator_QP_solver_CD_dl_cc + 329;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubcc07 = double_integrator_QP_solver_CD_ds_cc + 329;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsub07 = double_integrator_QP_solver_CD_ccrhs + 329;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sp07 = double_integrator_QP_solver_CD_s + 335;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lp07 = double_integrator_QP_solver_CD_l + 335;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lpbysp07 = double_integrator_QP_solver_CD_lbys + 335;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_aff07 = double_integrator_QP_solver_CD_dl_aff + 335;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_aff07 = double_integrator_QP_solver_CD_ds_aff + 335;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_cc07 = double_integrator_QP_solver_CD_dl_cc + 335;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_cc07 = double_integrator_QP_solver_CD_ds_cc + 335;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsp07 = double_integrator_QP_solver_CD_ccrhs + 335;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rip07[34];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Phi07[276];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_W07[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ysd07[49];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lsd07[49];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_z08 = double_integrator_QP_solver_CD_z + 184;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzaff08 = double_integrator_QP_solver_CD_dz_aff + 184;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzcc08 = double_integrator_QP_solver_CD_dz_cc + 184;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_rd08 = double_integrator_QP_solver_CD_rd + 184;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lbyrd08[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_cost08 = double_integrator_QP_solver_CD_grad_cost + 184;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_eq08 = double_integrator_QP_solver_CD_grad_eq + 184;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_ineq08 = double_integrator_QP_solver_CD_grad_ineq + 184;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_ctv08[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_v08 = double_integrator_QP_solver_CD_v + 56;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_re08[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_beta08[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_betacc08[7];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvaff08 = double_integrator_QP_solver_CD_dv_aff + 56;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvcc08 = double_integrator_QP_solver_CD_dv_cc + 56;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_V08[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Yd08[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ld08[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_yy08[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_bmy08[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_c08[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_CD_lbIdx08[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llb08 = double_integrator_QP_solver_CD_l + 369;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_slb08 = double_integrator_QP_solver_CD_s + 369;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llbbyslb08 = double_integrator_QP_solver_CD_lbys + 369;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rilb08[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbaff08 = double_integrator_QP_solver_CD_dl_aff + 369;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbaff08 = double_integrator_QP_solver_CD_ds_aff + 369;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbcc08 = double_integrator_QP_solver_CD_dl_cc + 369;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbcc08 = double_integrator_QP_solver_CD_ds_cc + 369;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsl08 = double_integrator_QP_solver_CD_ccrhs + 369;
int double_integrator_QP_solver_CD_ubIdx08[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lub08 = double_integrator_QP_solver_CD_l + 375;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sub08 = double_integrator_QP_solver_CD_s + 375;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lubbysub08 = double_integrator_QP_solver_CD_lbys + 375;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_riub08[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubaff08 = double_integrator_QP_solver_CD_dl_aff + 375;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubaff08 = double_integrator_QP_solver_CD_ds_aff + 375;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubcc08 = double_integrator_QP_solver_CD_dl_cc + 375;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubcc08 = double_integrator_QP_solver_CD_ds_cc + 375;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsub08 = double_integrator_QP_solver_CD_ccrhs + 375;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sp08 = double_integrator_QP_solver_CD_s + 381;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lp08 = double_integrator_QP_solver_CD_l + 381;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lpbysp08 = double_integrator_QP_solver_CD_lbys + 381;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_aff08 = double_integrator_QP_solver_CD_dl_aff + 381;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_aff08 = double_integrator_QP_solver_CD_ds_aff + 381;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_cc08 = double_integrator_QP_solver_CD_dl_cc + 381;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_cc08 = double_integrator_QP_solver_CD_ds_cc + 381;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsp08 = double_integrator_QP_solver_CD_ccrhs + 381;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rip08[34];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Phi08[276];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_W08[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ysd08[49];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lsd08[49];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_z09 = double_integrator_QP_solver_CD_z + 207;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzaff09 = double_integrator_QP_solver_CD_dz_aff + 207;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzcc09 = double_integrator_QP_solver_CD_dz_cc + 207;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_rd09 = double_integrator_QP_solver_CD_rd + 207;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lbyrd09[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_cost09 = double_integrator_QP_solver_CD_grad_cost + 207;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_eq09 = double_integrator_QP_solver_CD_grad_eq + 207;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_ineq09 = double_integrator_QP_solver_CD_grad_ineq + 207;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_ctv09[23];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_v09 = double_integrator_QP_solver_CD_v + 63;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_re09[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_beta09[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_betacc09[7];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvaff09 = double_integrator_QP_solver_CD_dv_aff + 63;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvcc09 = double_integrator_QP_solver_CD_dv_cc + 63;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_V09[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Yd09[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ld09[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_yy09[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_bmy09[7];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_c09[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_CD_lbIdx09[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llb09 = double_integrator_QP_solver_CD_l + 415;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_slb09 = double_integrator_QP_solver_CD_s + 415;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llbbyslb09 = double_integrator_QP_solver_CD_lbys + 415;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rilb09[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbaff09 = double_integrator_QP_solver_CD_dl_aff + 415;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbaff09 = double_integrator_QP_solver_CD_ds_aff + 415;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbcc09 = double_integrator_QP_solver_CD_dl_cc + 415;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbcc09 = double_integrator_QP_solver_CD_ds_cc + 415;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsl09 = double_integrator_QP_solver_CD_ccrhs + 415;
int double_integrator_QP_solver_CD_ubIdx09[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lub09 = double_integrator_QP_solver_CD_l + 421;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sub09 = double_integrator_QP_solver_CD_s + 421;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lubbysub09 = double_integrator_QP_solver_CD_lbys + 421;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_riub09[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubaff09 = double_integrator_QP_solver_CD_dl_aff + 421;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubaff09 = double_integrator_QP_solver_CD_ds_aff + 421;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubcc09 = double_integrator_QP_solver_CD_dl_cc + 421;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubcc09 = double_integrator_QP_solver_CD_ds_cc + 421;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsub09 = double_integrator_QP_solver_CD_ccrhs + 421;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sp09 = double_integrator_QP_solver_CD_s + 427;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lp09 = double_integrator_QP_solver_CD_l + 427;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lpbysp09 = double_integrator_QP_solver_CD_lbys + 427;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_aff09 = double_integrator_QP_solver_CD_dl_aff + 427;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_aff09 = double_integrator_QP_solver_CD_ds_aff + 427;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_cc09 = double_integrator_QP_solver_CD_dl_cc + 427;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_cc09 = double_integrator_QP_solver_CD_ds_cc + 427;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsp09 = double_integrator_QP_solver_CD_ccrhs + 427;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rip09[34];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Phi09[276];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_W09[161];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ysd09[49];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lsd09[49];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_H10[21] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_z10 = double_integrator_QP_solver_CD_z + 230;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzaff10 = double_integrator_QP_solver_CD_dz_aff + 230;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzcc10 = double_integrator_QP_solver_CD_dz_cc + 230;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_rd10 = double_integrator_QP_solver_CD_rd + 230;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lbyrd10[21];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_cost10 = double_integrator_QP_solver_CD_grad_cost + 230;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_eq10 = double_integrator_QP_solver_CD_grad_eq + 230;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_ineq10 = double_integrator_QP_solver_CD_grad_ineq + 230;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_ctv10[21];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_C10[84] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
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
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_v10 = double_integrator_QP_solver_CD_v + 70;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_re10[4];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_beta10[4];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_betacc10[4];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvaff10 = double_integrator_QP_solver_CD_dv_aff + 70;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dvcc10 = double_integrator_QP_solver_CD_dv_cc + 70;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_V10[84];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Yd10[10];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ld10[10];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_yy10[4];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_bmy10[4];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_c10[4] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_CD_lbIdx10[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llb10 = double_integrator_QP_solver_CD_l + 461;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_slb10 = double_integrator_QP_solver_CD_s + 461;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llbbyslb10 = double_integrator_QP_solver_CD_lbys + 461;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rilb10[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbaff10 = double_integrator_QP_solver_CD_dl_aff + 461;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbaff10 = double_integrator_QP_solver_CD_ds_aff + 461;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbcc10 = double_integrator_QP_solver_CD_dl_cc + 461;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbcc10 = double_integrator_QP_solver_CD_ds_cc + 461;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsl10 = double_integrator_QP_solver_CD_ccrhs + 461;
int double_integrator_QP_solver_CD_ubIdx10[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lub10 = double_integrator_QP_solver_CD_l + 467;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sub10 = double_integrator_QP_solver_CD_s + 467;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lubbysub10 = double_integrator_QP_solver_CD_lbys + 467;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_riub10[6];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubaff10 = double_integrator_QP_solver_CD_dl_aff + 467;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubaff10 = double_integrator_QP_solver_CD_ds_aff + 467;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubcc10 = double_integrator_QP_solver_CD_dl_cc + 467;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubcc10 = double_integrator_QP_solver_CD_ds_cc + 467;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsub10 = double_integrator_QP_solver_CD_ccrhs + 467;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sp10 = double_integrator_QP_solver_CD_s + 473;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lp10 = double_integrator_QP_solver_CD_l + 473;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lpbysp10 = double_integrator_QP_solver_CD_lbys + 473;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_aff10 = double_integrator_QP_solver_CD_dl_aff + 473;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_aff10 = double_integrator_QP_solver_CD_ds_aff + 473;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_cc10 = double_integrator_QP_solver_CD_dl_cc + 473;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_cc10 = double_integrator_QP_solver_CD_ds_cc + 473;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsp10 = double_integrator_QP_solver_CD_ccrhs + 473;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rip10[34];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Phi10[231];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_D10[147] = {-1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, -1.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_W10[147];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Ysd10[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lsd10[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_H11[5] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_z11 = double_integrator_QP_solver_CD_z + 251;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzaff11 = double_integrator_QP_solver_CD_dz_aff + 251;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dzcc11 = double_integrator_QP_solver_CD_dz_cc + 251;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_rd11 = double_integrator_QP_solver_CD_rd + 251;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Lbyrd11[7];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_cost11 = double_integrator_QP_solver_CD_grad_cost + 251;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_eq11 = double_integrator_QP_solver_CD_grad_eq + 251;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_grad_ineq11 = double_integrator_QP_solver_CD_grad_ineq + 251;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_ctv11[7];
int double_integrator_QP_solver_CD_lbIdx11[4] = {0, 1, 2, 3};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llb11 = double_integrator_QP_solver_CD_l + 507;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_slb11 = double_integrator_QP_solver_CD_s + 507;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_llbbyslb11 = double_integrator_QP_solver_CD_lbys + 507;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rilb11[4];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbaff11 = double_integrator_QP_solver_CD_dl_aff + 507;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbaff11 = double_integrator_QP_solver_CD_ds_aff + 507;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dllbcc11 = double_integrator_QP_solver_CD_dl_cc + 507;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dslbcc11 = double_integrator_QP_solver_CD_ds_cc + 507;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsl11 = double_integrator_QP_solver_CD_ccrhs + 507;
int double_integrator_QP_solver_CD_ubIdx11[4] = {0, 1, 2, 3};
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lub11 = double_integrator_QP_solver_CD_l + 511;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sub11 = double_integrator_QP_solver_CD_s + 511;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lubbysub11 = double_integrator_QP_solver_CD_lbys + 511;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_riub11[4];
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubaff11 = double_integrator_QP_solver_CD_dl_aff + 511;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubaff11 = double_integrator_QP_solver_CD_ds_aff + 511;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlubcc11 = double_integrator_QP_solver_CD_dl_cc + 511;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsubcc11 = double_integrator_QP_solver_CD_ds_cc + 511;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsub11 = double_integrator_QP_solver_CD_ccrhs + 511;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_sp11 = double_integrator_QP_solver_CD_s + 515;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lp11 = double_integrator_QP_solver_CD_l + 515;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_lpbysp11 = double_integrator_QP_solver_CD_lbys + 515;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_aff11 = double_integrator_QP_solver_CD_dl_aff + 515;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_aff11 = double_integrator_QP_solver_CD_ds_aff + 515;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dlp_cc11 = double_integrator_QP_solver_CD_dl_cc + 515;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_dsp_cc11 = double_integrator_QP_solver_CD_ds_cc + 515;
double_integrator_QP_solver_CD_FLOAT* double_integrator_QP_solver_CD_ccrhsp11 = double_integrator_QP_solver_CD_ccrhs + 515;
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_rip11[14];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_Phi11[28];
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_D11[7] = {-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000};
double_integrator_QP_solver_CD_FLOAT double_integrator_QP_solver_CD_W11[28];
double_integrator_QP_solver_CD_FLOAT musigma;
double_integrator_QP_solver_CD_FLOAT sigma_3rdroot;




/* SOLVER CODE --------------------------------------------------------- */
int double_integrator_QP_solver_CD_solve(double_integrator_QP_solver_CD_params* params, double_integrator_QP_solver_CD_output* output, double_integrator_QP_solver_CD_info* info)
{	
int exitcode;

#if double_integrator_QP_solver_CD_SET_TIMING == 1
	double_integrator_QP_solver_CD_timer solvertimer;
	double_integrator_QP_solver_CD_tic(&solvertimer);
#endif
/* FUNCTION CALLS INTO LA LIBRARY -------------------------------------- */
info->it = 0;
double_integrator_QP_solver_CD_LA_INITIALIZEVECTOR_258(double_integrator_QP_solver_CD_z, 0);
double_integrator_QP_solver_CD_LA_INITIALIZEVECTOR_74(double_integrator_QP_solver_CD_v, 1);
double_integrator_QP_solver_CD_LA_INITIALIZEVECTOR_529(double_integrator_QP_solver_CD_l, 10);
double_integrator_QP_solver_CD_LA_INITIALIZEVECTOR_529(double_integrator_QP_solver_CD_s, 10);
info->mu = 0;
double_integrator_QP_solver_CD_LA_DOTACC_529(double_integrator_QP_solver_CD_l, double_integrator_QP_solver_CD_s, &info->mu);
info->mu /= 529;
PRINTTEXT("This is double_integrator_QP_solver_CD, a solver generated by FORCES (forces.ethz.ch).\n");
PRINTTEXT("(c) Alexander Domahidi, Automatic Control Laboratory, ETH Zurich, 2011-2014.\n");
PRINTTEXT("\n  #it  res_eq   res_ineq     pobj         dobj       dgap     rdgap     mu\n");
PRINTTEXT("  ---------------------------------------------------------------------------\n");
while( 1 ){
info->pobj = 0;
double_integrator_QP_solver_CD_LA_DIAG_QUADFCN_23(double_integrator_QP_solver_CD_H00, params->f1, double_integrator_QP_solver_CD_z00, double_integrator_QP_solver_CD_grad_cost00, &info->pobj);
double_integrator_QP_solver_CD_LA_DIAG_QUADFCN_23(double_integrator_QP_solver_CD_H00, params->f2, double_integrator_QP_solver_CD_z01, double_integrator_QP_solver_CD_grad_cost01, &info->pobj);
double_integrator_QP_solver_CD_LA_DIAG_QUADFCN_23(double_integrator_QP_solver_CD_H00, params->f3, double_integrator_QP_solver_CD_z02, double_integrator_QP_solver_CD_grad_cost02, &info->pobj);
double_integrator_QP_solver_CD_LA_DIAG_QUADFCN_23(double_integrator_QP_solver_CD_H00, params->f4, double_integrator_QP_solver_CD_z03, double_integrator_QP_solver_CD_grad_cost03, &info->pobj);
double_integrator_QP_solver_CD_LA_DIAG_QUADFCN_23(double_integrator_QP_solver_CD_H00, params->f5, double_integrator_QP_solver_CD_z04, double_integrator_QP_solver_CD_grad_cost04, &info->pobj);
double_integrator_QP_solver_CD_LA_DIAG_QUADFCN_23(double_integrator_QP_solver_CD_H00, params->f6, double_integrator_QP_solver_CD_z05, double_integrator_QP_solver_CD_grad_cost05, &info->pobj);
double_integrator_QP_solver_CD_LA_DIAG_QUADFCN_23(double_integrator_QP_solver_CD_H00, params->f7, double_integrator_QP_solver_CD_z06, double_integrator_QP_solver_CD_grad_cost06, &info->pobj);
double_integrator_QP_solver_CD_LA_DIAG_QUADFCN_23(double_integrator_QP_solver_CD_H00, params->f8, double_integrator_QP_solver_CD_z07, double_integrator_QP_solver_CD_grad_cost07, &info->pobj);
double_integrator_QP_solver_CD_LA_DIAG_QUADFCN_23(double_integrator_QP_solver_CD_H00, params->f9, double_integrator_QP_solver_CD_z08, double_integrator_QP_solver_CD_grad_cost08, &info->pobj);
double_integrator_QP_solver_CD_LA_DIAG_QUADFCN_23(double_integrator_QP_solver_CD_H00, params->f10, double_integrator_QP_solver_CD_z09, double_integrator_QP_solver_CD_grad_cost09, &info->pobj);
double_integrator_QP_solver_CD_LA_DIAG_QUADFCN_21(double_integrator_QP_solver_CD_H10, params->f11, double_integrator_QP_solver_CD_z10, double_integrator_QP_solver_CD_grad_cost10, &info->pobj);
double_integrator_QP_solver_CD_LA_DIAG_QUADFCN_7(double_integrator_QP_solver_CD_H11, params->f12, double_integrator_QP_solver_CD_z11, double_integrator_QP_solver_CD_grad_cost11, &info->pobj);
info->res_eq = 0;
info->dgap = 0;
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB3_7_23_23(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_z00, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_z01, double_integrator_QP_solver_CD_c00, double_integrator_QP_solver_CD_v00, double_integrator_QP_solver_CD_re00, &info->dgap, &info->res_eq);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB3_7_23_23(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_z01, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_z02, double_integrator_QP_solver_CD_c01, double_integrator_QP_solver_CD_v01, double_integrator_QP_solver_CD_re01, &info->dgap, &info->res_eq);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB3_7_23_23(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_z02, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_z03, double_integrator_QP_solver_CD_c02, double_integrator_QP_solver_CD_v02, double_integrator_QP_solver_CD_re02, &info->dgap, &info->res_eq);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB3_7_23_23(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_z03, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_z04, double_integrator_QP_solver_CD_c03, double_integrator_QP_solver_CD_v03, double_integrator_QP_solver_CD_re03, &info->dgap, &info->res_eq);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB3_7_23_23(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_z04, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_z05, double_integrator_QP_solver_CD_c04, double_integrator_QP_solver_CD_v04, double_integrator_QP_solver_CD_re04, &info->dgap, &info->res_eq);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB3_7_23_23(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_z05, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_z06, double_integrator_QP_solver_CD_c05, double_integrator_QP_solver_CD_v05, double_integrator_QP_solver_CD_re05, &info->dgap, &info->res_eq);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB3_7_23_23(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_z06, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_z07, double_integrator_QP_solver_CD_c06, double_integrator_QP_solver_CD_v06, double_integrator_QP_solver_CD_re06, &info->dgap, &info->res_eq);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB3_7_23_23(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_z07, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_z08, double_integrator_QP_solver_CD_c07, double_integrator_QP_solver_CD_v07, double_integrator_QP_solver_CD_re07, &info->dgap, &info->res_eq);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB3_7_23_23(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_z08, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_z09, double_integrator_QP_solver_CD_c08, double_integrator_QP_solver_CD_v08, double_integrator_QP_solver_CD_re08, &info->dgap, &info->res_eq);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB3_7_23_21(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_z09, double_integrator_QP_solver_CD_D10, double_integrator_QP_solver_CD_z10, double_integrator_QP_solver_CD_c09, double_integrator_QP_solver_CD_v09, double_integrator_QP_solver_CD_re09, &info->dgap, &info->res_eq);
double_integrator_QP_solver_CD_LA_DENSE_DIAGZERO_MVMSUB3_4_21_7(double_integrator_QP_solver_CD_C10, double_integrator_QP_solver_CD_z10, double_integrator_QP_solver_CD_D11, double_integrator_QP_solver_CD_z11, double_integrator_QP_solver_CD_c10, double_integrator_QP_solver_CD_v10, double_integrator_QP_solver_CD_re10, &info->dgap, &info->res_eq);
double_integrator_QP_solver_CD_LA_DENSE_MTVM_7_23(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_v00, double_integrator_QP_solver_CD_grad_eq00);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_v01, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_v00, double_integrator_QP_solver_CD_grad_eq01);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_v02, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_v01, double_integrator_QP_solver_CD_grad_eq02);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_v03, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_v02, double_integrator_QP_solver_CD_grad_eq03);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_v04, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_v03, double_integrator_QP_solver_CD_grad_eq04);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_v05, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_v04, double_integrator_QP_solver_CD_grad_eq05);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_v06, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_v05, double_integrator_QP_solver_CD_grad_eq06);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_v07, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_v06, double_integrator_QP_solver_CD_grad_eq07);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_v08, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_v07, double_integrator_QP_solver_CD_grad_eq08);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_v09, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_v08, double_integrator_QP_solver_CD_grad_eq09);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_4_21_7(double_integrator_QP_solver_CD_C10, double_integrator_QP_solver_CD_v10, double_integrator_QP_solver_CD_D10, double_integrator_QP_solver_CD_v09, double_integrator_QP_solver_CD_grad_eq10);
double_integrator_QP_solver_CD_LA_DIAGZERO_MTVM_4_7(double_integrator_QP_solver_CD_D11, double_integrator_QP_solver_CD_v10, double_integrator_QP_solver_CD_grad_eq11);
info->res_ineq = 0;
double_integrator_QP_solver_CD_LA_VSUBADD3_7(params->lb1, double_integrator_QP_solver_CD_z00, double_integrator_QP_solver_CD_lbIdx00, double_integrator_QP_solver_CD_llb00, double_integrator_QP_solver_CD_slb00, double_integrator_QP_solver_CD_rilb00, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD2_6(double_integrator_QP_solver_CD_z00, double_integrator_QP_solver_CD_ubIdx00, params->ub1, double_integrator_QP_solver_CD_lub00, double_integrator_QP_solver_CD_sub00, double_integrator_QP_solver_CD_riub00, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_MVSUBADD_34_23(params->A1, double_integrator_QP_solver_CD_z00, params->b1, double_integrator_QP_solver_CD_sp00, double_integrator_QP_solver_CD_lp00, double_integrator_QP_solver_CD_rip00, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD3_6(params->lb2, double_integrator_QP_solver_CD_z01, double_integrator_QP_solver_CD_lbIdx01, double_integrator_QP_solver_CD_llb01, double_integrator_QP_solver_CD_slb01, double_integrator_QP_solver_CD_rilb01, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD2_6(double_integrator_QP_solver_CD_z01, double_integrator_QP_solver_CD_ubIdx01, params->ub2, double_integrator_QP_solver_CD_lub01, double_integrator_QP_solver_CD_sub01, double_integrator_QP_solver_CD_riub01, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_MVSUBADD_34_23(params->A2, double_integrator_QP_solver_CD_z01, params->b2, double_integrator_QP_solver_CD_sp01, double_integrator_QP_solver_CD_lp01, double_integrator_QP_solver_CD_rip01, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD3_6(params->lb3, double_integrator_QP_solver_CD_z02, double_integrator_QP_solver_CD_lbIdx02, double_integrator_QP_solver_CD_llb02, double_integrator_QP_solver_CD_slb02, double_integrator_QP_solver_CD_rilb02, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD2_6(double_integrator_QP_solver_CD_z02, double_integrator_QP_solver_CD_ubIdx02, params->ub3, double_integrator_QP_solver_CD_lub02, double_integrator_QP_solver_CD_sub02, double_integrator_QP_solver_CD_riub02, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_MVSUBADD_34_23(params->A3, double_integrator_QP_solver_CD_z02, params->b3, double_integrator_QP_solver_CD_sp02, double_integrator_QP_solver_CD_lp02, double_integrator_QP_solver_CD_rip02, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD3_6(params->lb4, double_integrator_QP_solver_CD_z03, double_integrator_QP_solver_CD_lbIdx03, double_integrator_QP_solver_CD_llb03, double_integrator_QP_solver_CD_slb03, double_integrator_QP_solver_CD_rilb03, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD2_6(double_integrator_QP_solver_CD_z03, double_integrator_QP_solver_CD_ubIdx03, params->ub4, double_integrator_QP_solver_CD_lub03, double_integrator_QP_solver_CD_sub03, double_integrator_QP_solver_CD_riub03, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_MVSUBADD_34_23(params->A4, double_integrator_QP_solver_CD_z03, params->b4, double_integrator_QP_solver_CD_sp03, double_integrator_QP_solver_CD_lp03, double_integrator_QP_solver_CD_rip03, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD3_6(params->lb5, double_integrator_QP_solver_CD_z04, double_integrator_QP_solver_CD_lbIdx04, double_integrator_QP_solver_CD_llb04, double_integrator_QP_solver_CD_slb04, double_integrator_QP_solver_CD_rilb04, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD2_6(double_integrator_QP_solver_CD_z04, double_integrator_QP_solver_CD_ubIdx04, params->ub5, double_integrator_QP_solver_CD_lub04, double_integrator_QP_solver_CD_sub04, double_integrator_QP_solver_CD_riub04, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_MVSUBADD_34_23(params->A5, double_integrator_QP_solver_CD_z04, params->b5, double_integrator_QP_solver_CD_sp04, double_integrator_QP_solver_CD_lp04, double_integrator_QP_solver_CD_rip04, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD3_6(params->lb6, double_integrator_QP_solver_CD_z05, double_integrator_QP_solver_CD_lbIdx05, double_integrator_QP_solver_CD_llb05, double_integrator_QP_solver_CD_slb05, double_integrator_QP_solver_CD_rilb05, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD2_6(double_integrator_QP_solver_CD_z05, double_integrator_QP_solver_CD_ubIdx05, params->ub6, double_integrator_QP_solver_CD_lub05, double_integrator_QP_solver_CD_sub05, double_integrator_QP_solver_CD_riub05, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_MVSUBADD_34_23(params->A6, double_integrator_QP_solver_CD_z05, params->b6, double_integrator_QP_solver_CD_sp05, double_integrator_QP_solver_CD_lp05, double_integrator_QP_solver_CD_rip05, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD3_6(params->lb7, double_integrator_QP_solver_CD_z06, double_integrator_QP_solver_CD_lbIdx06, double_integrator_QP_solver_CD_llb06, double_integrator_QP_solver_CD_slb06, double_integrator_QP_solver_CD_rilb06, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD2_6(double_integrator_QP_solver_CD_z06, double_integrator_QP_solver_CD_ubIdx06, params->ub7, double_integrator_QP_solver_CD_lub06, double_integrator_QP_solver_CD_sub06, double_integrator_QP_solver_CD_riub06, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_MVSUBADD_34_23(params->A7, double_integrator_QP_solver_CD_z06, params->b7, double_integrator_QP_solver_CD_sp06, double_integrator_QP_solver_CD_lp06, double_integrator_QP_solver_CD_rip06, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD3_6(params->lb8, double_integrator_QP_solver_CD_z07, double_integrator_QP_solver_CD_lbIdx07, double_integrator_QP_solver_CD_llb07, double_integrator_QP_solver_CD_slb07, double_integrator_QP_solver_CD_rilb07, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD2_6(double_integrator_QP_solver_CD_z07, double_integrator_QP_solver_CD_ubIdx07, params->ub8, double_integrator_QP_solver_CD_lub07, double_integrator_QP_solver_CD_sub07, double_integrator_QP_solver_CD_riub07, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_MVSUBADD_34_23(params->A8, double_integrator_QP_solver_CD_z07, params->b8, double_integrator_QP_solver_CD_sp07, double_integrator_QP_solver_CD_lp07, double_integrator_QP_solver_CD_rip07, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD3_6(params->lb9, double_integrator_QP_solver_CD_z08, double_integrator_QP_solver_CD_lbIdx08, double_integrator_QP_solver_CD_llb08, double_integrator_QP_solver_CD_slb08, double_integrator_QP_solver_CD_rilb08, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD2_6(double_integrator_QP_solver_CD_z08, double_integrator_QP_solver_CD_ubIdx08, params->ub9, double_integrator_QP_solver_CD_lub08, double_integrator_QP_solver_CD_sub08, double_integrator_QP_solver_CD_riub08, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_MVSUBADD_34_23(params->A9, double_integrator_QP_solver_CD_z08, params->b9, double_integrator_QP_solver_CD_sp08, double_integrator_QP_solver_CD_lp08, double_integrator_QP_solver_CD_rip08, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD3_6(params->lb10, double_integrator_QP_solver_CD_z09, double_integrator_QP_solver_CD_lbIdx09, double_integrator_QP_solver_CD_llb09, double_integrator_QP_solver_CD_slb09, double_integrator_QP_solver_CD_rilb09, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD2_6(double_integrator_QP_solver_CD_z09, double_integrator_QP_solver_CD_ubIdx09, params->ub10, double_integrator_QP_solver_CD_lub09, double_integrator_QP_solver_CD_sub09, double_integrator_QP_solver_CD_riub09, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_MVSUBADD_34_23(params->A10, double_integrator_QP_solver_CD_z09, params->b10, double_integrator_QP_solver_CD_sp09, double_integrator_QP_solver_CD_lp09, double_integrator_QP_solver_CD_rip09, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD3_6(params->lb11, double_integrator_QP_solver_CD_z10, double_integrator_QP_solver_CD_lbIdx10, double_integrator_QP_solver_CD_llb10, double_integrator_QP_solver_CD_slb10, double_integrator_QP_solver_CD_rilb10, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD2_6(double_integrator_QP_solver_CD_z10, double_integrator_QP_solver_CD_ubIdx10, params->ub11, double_integrator_QP_solver_CD_lub10, double_integrator_QP_solver_CD_sub10, double_integrator_QP_solver_CD_riub10, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_MVSUBADD_34_21(params->A11, double_integrator_QP_solver_CD_z10, params->b11, double_integrator_QP_solver_CD_sp10, double_integrator_QP_solver_CD_lp10, double_integrator_QP_solver_CD_rip10, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD3_4(params->lb12, double_integrator_QP_solver_CD_z11, double_integrator_QP_solver_CD_lbIdx11, double_integrator_QP_solver_CD_llb11, double_integrator_QP_solver_CD_slb11, double_integrator_QP_solver_CD_rilb11, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_VSUBADD2_4(double_integrator_QP_solver_CD_z11, double_integrator_QP_solver_CD_ubIdx11, params->ub12, double_integrator_QP_solver_CD_lub11, double_integrator_QP_solver_CD_sub11, double_integrator_QP_solver_CD_riub11, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_MVSUBADD_14_7(params->A12, double_integrator_QP_solver_CD_z11, params->b12, double_integrator_QP_solver_CD_sp11, double_integrator_QP_solver_CD_lp11, double_integrator_QP_solver_CD_rip11, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_CD_LA_INEQ_B_GRAD_23_7_6(double_integrator_QP_solver_CD_lub00, double_integrator_QP_solver_CD_sub00, double_integrator_QP_solver_CD_riub00, double_integrator_QP_solver_CD_llb00, double_integrator_QP_solver_CD_slb00, double_integrator_QP_solver_CD_rilb00, double_integrator_QP_solver_CD_lbIdx00, double_integrator_QP_solver_CD_ubIdx00, double_integrator_QP_solver_CD_grad_ineq00, double_integrator_QP_solver_CD_lubbysub00, double_integrator_QP_solver_CD_llbbyslb00);
double_integrator_QP_solver_CD_LA_INEQ_P_34_23(params->A1, double_integrator_QP_solver_CD_lp00, double_integrator_QP_solver_CD_sp00, double_integrator_QP_solver_CD_rip00, double_integrator_QP_solver_CD_grad_ineq00, double_integrator_QP_solver_CD_lpbysp00);
double_integrator_QP_solver_CD_LA_INEQ_B_GRAD_23_6_6(double_integrator_QP_solver_CD_lub01, double_integrator_QP_solver_CD_sub01, double_integrator_QP_solver_CD_riub01, double_integrator_QP_solver_CD_llb01, double_integrator_QP_solver_CD_slb01, double_integrator_QP_solver_CD_rilb01, double_integrator_QP_solver_CD_lbIdx01, double_integrator_QP_solver_CD_ubIdx01, double_integrator_QP_solver_CD_grad_ineq01, double_integrator_QP_solver_CD_lubbysub01, double_integrator_QP_solver_CD_llbbyslb01);
double_integrator_QP_solver_CD_LA_INEQ_P_34_23(params->A2, double_integrator_QP_solver_CD_lp01, double_integrator_QP_solver_CD_sp01, double_integrator_QP_solver_CD_rip01, double_integrator_QP_solver_CD_grad_ineq01, double_integrator_QP_solver_CD_lpbysp01);
double_integrator_QP_solver_CD_LA_INEQ_B_GRAD_23_6_6(double_integrator_QP_solver_CD_lub02, double_integrator_QP_solver_CD_sub02, double_integrator_QP_solver_CD_riub02, double_integrator_QP_solver_CD_llb02, double_integrator_QP_solver_CD_slb02, double_integrator_QP_solver_CD_rilb02, double_integrator_QP_solver_CD_lbIdx02, double_integrator_QP_solver_CD_ubIdx02, double_integrator_QP_solver_CD_grad_ineq02, double_integrator_QP_solver_CD_lubbysub02, double_integrator_QP_solver_CD_llbbyslb02);
double_integrator_QP_solver_CD_LA_INEQ_P_34_23(params->A3, double_integrator_QP_solver_CD_lp02, double_integrator_QP_solver_CD_sp02, double_integrator_QP_solver_CD_rip02, double_integrator_QP_solver_CD_grad_ineq02, double_integrator_QP_solver_CD_lpbysp02);
double_integrator_QP_solver_CD_LA_INEQ_B_GRAD_23_6_6(double_integrator_QP_solver_CD_lub03, double_integrator_QP_solver_CD_sub03, double_integrator_QP_solver_CD_riub03, double_integrator_QP_solver_CD_llb03, double_integrator_QP_solver_CD_slb03, double_integrator_QP_solver_CD_rilb03, double_integrator_QP_solver_CD_lbIdx03, double_integrator_QP_solver_CD_ubIdx03, double_integrator_QP_solver_CD_grad_ineq03, double_integrator_QP_solver_CD_lubbysub03, double_integrator_QP_solver_CD_llbbyslb03);
double_integrator_QP_solver_CD_LA_INEQ_P_34_23(params->A4, double_integrator_QP_solver_CD_lp03, double_integrator_QP_solver_CD_sp03, double_integrator_QP_solver_CD_rip03, double_integrator_QP_solver_CD_grad_ineq03, double_integrator_QP_solver_CD_lpbysp03);
double_integrator_QP_solver_CD_LA_INEQ_B_GRAD_23_6_6(double_integrator_QP_solver_CD_lub04, double_integrator_QP_solver_CD_sub04, double_integrator_QP_solver_CD_riub04, double_integrator_QP_solver_CD_llb04, double_integrator_QP_solver_CD_slb04, double_integrator_QP_solver_CD_rilb04, double_integrator_QP_solver_CD_lbIdx04, double_integrator_QP_solver_CD_ubIdx04, double_integrator_QP_solver_CD_grad_ineq04, double_integrator_QP_solver_CD_lubbysub04, double_integrator_QP_solver_CD_llbbyslb04);
double_integrator_QP_solver_CD_LA_INEQ_P_34_23(params->A5, double_integrator_QP_solver_CD_lp04, double_integrator_QP_solver_CD_sp04, double_integrator_QP_solver_CD_rip04, double_integrator_QP_solver_CD_grad_ineq04, double_integrator_QP_solver_CD_lpbysp04);
double_integrator_QP_solver_CD_LA_INEQ_B_GRAD_23_6_6(double_integrator_QP_solver_CD_lub05, double_integrator_QP_solver_CD_sub05, double_integrator_QP_solver_CD_riub05, double_integrator_QP_solver_CD_llb05, double_integrator_QP_solver_CD_slb05, double_integrator_QP_solver_CD_rilb05, double_integrator_QP_solver_CD_lbIdx05, double_integrator_QP_solver_CD_ubIdx05, double_integrator_QP_solver_CD_grad_ineq05, double_integrator_QP_solver_CD_lubbysub05, double_integrator_QP_solver_CD_llbbyslb05);
double_integrator_QP_solver_CD_LA_INEQ_P_34_23(params->A6, double_integrator_QP_solver_CD_lp05, double_integrator_QP_solver_CD_sp05, double_integrator_QP_solver_CD_rip05, double_integrator_QP_solver_CD_grad_ineq05, double_integrator_QP_solver_CD_lpbysp05);
double_integrator_QP_solver_CD_LA_INEQ_B_GRAD_23_6_6(double_integrator_QP_solver_CD_lub06, double_integrator_QP_solver_CD_sub06, double_integrator_QP_solver_CD_riub06, double_integrator_QP_solver_CD_llb06, double_integrator_QP_solver_CD_slb06, double_integrator_QP_solver_CD_rilb06, double_integrator_QP_solver_CD_lbIdx06, double_integrator_QP_solver_CD_ubIdx06, double_integrator_QP_solver_CD_grad_ineq06, double_integrator_QP_solver_CD_lubbysub06, double_integrator_QP_solver_CD_llbbyslb06);
double_integrator_QP_solver_CD_LA_INEQ_P_34_23(params->A7, double_integrator_QP_solver_CD_lp06, double_integrator_QP_solver_CD_sp06, double_integrator_QP_solver_CD_rip06, double_integrator_QP_solver_CD_grad_ineq06, double_integrator_QP_solver_CD_lpbysp06);
double_integrator_QP_solver_CD_LA_INEQ_B_GRAD_23_6_6(double_integrator_QP_solver_CD_lub07, double_integrator_QP_solver_CD_sub07, double_integrator_QP_solver_CD_riub07, double_integrator_QP_solver_CD_llb07, double_integrator_QP_solver_CD_slb07, double_integrator_QP_solver_CD_rilb07, double_integrator_QP_solver_CD_lbIdx07, double_integrator_QP_solver_CD_ubIdx07, double_integrator_QP_solver_CD_grad_ineq07, double_integrator_QP_solver_CD_lubbysub07, double_integrator_QP_solver_CD_llbbyslb07);
double_integrator_QP_solver_CD_LA_INEQ_P_34_23(params->A8, double_integrator_QP_solver_CD_lp07, double_integrator_QP_solver_CD_sp07, double_integrator_QP_solver_CD_rip07, double_integrator_QP_solver_CD_grad_ineq07, double_integrator_QP_solver_CD_lpbysp07);
double_integrator_QP_solver_CD_LA_INEQ_B_GRAD_23_6_6(double_integrator_QP_solver_CD_lub08, double_integrator_QP_solver_CD_sub08, double_integrator_QP_solver_CD_riub08, double_integrator_QP_solver_CD_llb08, double_integrator_QP_solver_CD_slb08, double_integrator_QP_solver_CD_rilb08, double_integrator_QP_solver_CD_lbIdx08, double_integrator_QP_solver_CD_ubIdx08, double_integrator_QP_solver_CD_grad_ineq08, double_integrator_QP_solver_CD_lubbysub08, double_integrator_QP_solver_CD_llbbyslb08);
double_integrator_QP_solver_CD_LA_INEQ_P_34_23(params->A9, double_integrator_QP_solver_CD_lp08, double_integrator_QP_solver_CD_sp08, double_integrator_QP_solver_CD_rip08, double_integrator_QP_solver_CD_grad_ineq08, double_integrator_QP_solver_CD_lpbysp08);
double_integrator_QP_solver_CD_LA_INEQ_B_GRAD_23_6_6(double_integrator_QP_solver_CD_lub09, double_integrator_QP_solver_CD_sub09, double_integrator_QP_solver_CD_riub09, double_integrator_QP_solver_CD_llb09, double_integrator_QP_solver_CD_slb09, double_integrator_QP_solver_CD_rilb09, double_integrator_QP_solver_CD_lbIdx09, double_integrator_QP_solver_CD_ubIdx09, double_integrator_QP_solver_CD_grad_ineq09, double_integrator_QP_solver_CD_lubbysub09, double_integrator_QP_solver_CD_llbbyslb09);
double_integrator_QP_solver_CD_LA_INEQ_P_34_23(params->A10, double_integrator_QP_solver_CD_lp09, double_integrator_QP_solver_CD_sp09, double_integrator_QP_solver_CD_rip09, double_integrator_QP_solver_CD_grad_ineq09, double_integrator_QP_solver_CD_lpbysp09);
double_integrator_QP_solver_CD_LA_INEQ_B_GRAD_21_6_6(double_integrator_QP_solver_CD_lub10, double_integrator_QP_solver_CD_sub10, double_integrator_QP_solver_CD_riub10, double_integrator_QP_solver_CD_llb10, double_integrator_QP_solver_CD_slb10, double_integrator_QP_solver_CD_rilb10, double_integrator_QP_solver_CD_lbIdx10, double_integrator_QP_solver_CD_ubIdx10, double_integrator_QP_solver_CD_grad_ineq10, double_integrator_QP_solver_CD_lubbysub10, double_integrator_QP_solver_CD_llbbyslb10);
double_integrator_QP_solver_CD_LA_INEQ_P_34_21(params->A11, double_integrator_QP_solver_CD_lp10, double_integrator_QP_solver_CD_sp10, double_integrator_QP_solver_CD_rip10, double_integrator_QP_solver_CD_grad_ineq10, double_integrator_QP_solver_CD_lpbysp10);
double_integrator_QP_solver_CD_LA_INEQ_B_GRAD_7_4_4(double_integrator_QP_solver_CD_lub11, double_integrator_QP_solver_CD_sub11, double_integrator_QP_solver_CD_riub11, double_integrator_QP_solver_CD_llb11, double_integrator_QP_solver_CD_slb11, double_integrator_QP_solver_CD_rilb11, double_integrator_QP_solver_CD_lbIdx11, double_integrator_QP_solver_CD_ubIdx11, double_integrator_QP_solver_CD_grad_ineq11, double_integrator_QP_solver_CD_lubbysub11, double_integrator_QP_solver_CD_llbbyslb11);
double_integrator_QP_solver_CD_LA_INEQ_P_14_7(params->A12, double_integrator_QP_solver_CD_lp11, double_integrator_QP_solver_CD_sp11, double_integrator_QP_solver_CD_rip11, double_integrator_QP_solver_CD_grad_ineq11, double_integrator_QP_solver_CD_lpbysp11);
info->dobj = info->pobj - info->dgap;
info->rdgap = info->pobj ? info->dgap / info->pobj : 1e6;
if( info->rdgap < 0 ) info->rdgap = -info->rdgap;
PRINTTEXT("  %3d  %3.1e  %3.1e  %+6.4e  %+6.4e  %+3.1e  %3.1e  %3.1e\n",info->it, info->res_eq, info->res_ineq, info->pobj, info->dobj, info->dgap, info->rdgap, info->mu);
if( info->mu < double_integrator_QP_solver_CD_SET_ACC_KKTCOMPL
    && (info->rdgap < double_integrator_QP_solver_CD_SET_ACC_RDGAP || info->dgap < double_integrator_QP_solver_CD_SET_ACC_KKTCOMPL)
    && info->res_eq < double_integrator_QP_solver_CD_SET_ACC_RESEQ
    && info->res_ineq < double_integrator_QP_solver_CD_SET_ACC_RESINEQ ){
PRINTTEXT("OPTIMAL (within RESEQ=%2.1e, RESINEQ=%2.1e, (R)DGAP=(%2.1e)%2.1e, MU=%2.1e).\n",double_integrator_QP_solver_CD_SET_ACC_RESEQ, double_integrator_QP_solver_CD_SET_ACC_RESINEQ,double_integrator_QP_solver_CD_SET_ACC_KKTCOMPL,double_integrator_QP_solver_CD_SET_ACC_RDGAP,double_integrator_QP_solver_CD_SET_ACC_KKTCOMPL);
exitcode = double_integrator_QP_solver_CD_OPTIMAL; break; }
if( info->it == double_integrator_QP_solver_CD_SET_MAXIT ){
PRINTTEXT("Maximum number of iterations reached, exiting.\n");
exitcode = double_integrator_QP_solver_CD_MAXITREACHED; break; }
double_integrator_QP_solver_CD_LA_VVADD3_258(double_integrator_QP_solver_CD_grad_cost, double_integrator_QP_solver_CD_grad_eq, double_integrator_QP_solver_CD_grad_ineq, double_integrator_QP_solver_CD_rd);
double_integrator_QP_solver_CD_LA_INEQ_DENSE_DIAG_HESS_23_7_6(double_integrator_QP_solver_CD_H00, double_integrator_QP_solver_CD_llbbyslb00, double_integrator_QP_solver_CD_lbIdx00, double_integrator_QP_solver_CD_lubbysub00, double_integrator_QP_solver_CD_ubIdx00, double_integrator_QP_solver_CD_Phi00);
double_integrator_QP_solver_CD_LA_DENSE_ADDMTDM_34_23(params->A1, double_integrator_QP_solver_CD_lpbysp00, double_integrator_QP_solver_CD_Phi00);
double_integrator_QP_solver_CD_LA_DENSE_CHOL2_23(double_integrator_QP_solver_CD_Phi00);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi00, double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_V00);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi00, double_integrator_QP_solver_CD_rd00, double_integrator_QP_solver_CD_Lbyrd00);
double_integrator_QP_solver_CD_LA_INEQ_DENSE_DIAG_HESS_23_6_6(double_integrator_QP_solver_CD_H00, double_integrator_QP_solver_CD_llbbyslb01, double_integrator_QP_solver_CD_lbIdx01, double_integrator_QP_solver_CD_lubbysub01, double_integrator_QP_solver_CD_ubIdx01, double_integrator_QP_solver_CD_Phi01);
double_integrator_QP_solver_CD_LA_DENSE_ADDMTDM_34_23(params->A2, double_integrator_QP_solver_CD_lpbysp01, double_integrator_QP_solver_CD_Phi01);
double_integrator_QP_solver_CD_LA_DENSE_CHOL2_23(double_integrator_QP_solver_CD_Phi01);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi01, double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_V01);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi01, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_W01);
double_integrator_QP_solver_CD_LA_DENSE_MMTM_7_23_7(double_integrator_QP_solver_CD_W01, double_integrator_QP_solver_CD_V01, double_integrator_QP_solver_CD_Ysd01);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi01, double_integrator_QP_solver_CD_rd01, double_integrator_QP_solver_CD_Lbyrd01);
double_integrator_QP_solver_CD_LA_INEQ_DENSE_DIAG_HESS_23_6_6(double_integrator_QP_solver_CD_H00, double_integrator_QP_solver_CD_llbbyslb02, double_integrator_QP_solver_CD_lbIdx02, double_integrator_QP_solver_CD_lubbysub02, double_integrator_QP_solver_CD_ubIdx02, double_integrator_QP_solver_CD_Phi02);
double_integrator_QP_solver_CD_LA_DENSE_ADDMTDM_34_23(params->A3, double_integrator_QP_solver_CD_lpbysp02, double_integrator_QP_solver_CD_Phi02);
double_integrator_QP_solver_CD_LA_DENSE_CHOL2_23(double_integrator_QP_solver_CD_Phi02);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi02, double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_V02);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi02, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_W02);
double_integrator_QP_solver_CD_LA_DENSE_MMTM_7_23_7(double_integrator_QP_solver_CD_W02, double_integrator_QP_solver_CD_V02, double_integrator_QP_solver_CD_Ysd02);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi02, double_integrator_QP_solver_CD_rd02, double_integrator_QP_solver_CD_Lbyrd02);
double_integrator_QP_solver_CD_LA_INEQ_DENSE_DIAG_HESS_23_6_6(double_integrator_QP_solver_CD_H00, double_integrator_QP_solver_CD_llbbyslb03, double_integrator_QP_solver_CD_lbIdx03, double_integrator_QP_solver_CD_lubbysub03, double_integrator_QP_solver_CD_ubIdx03, double_integrator_QP_solver_CD_Phi03);
double_integrator_QP_solver_CD_LA_DENSE_ADDMTDM_34_23(params->A4, double_integrator_QP_solver_CD_lpbysp03, double_integrator_QP_solver_CD_Phi03);
double_integrator_QP_solver_CD_LA_DENSE_CHOL2_23(double_integrator_QP_solver_CD_Phi03);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi03, double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_V03);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi03, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_W03);
double_integrator_QP_solver_CD_LA_DENSE_MMTM_7_23_7(double_integrator_QP_solver_CD_W03, double_integrator_QP_solver_CD_V03, double_integrator_QP_solver_CD_Ysd03);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi03, double_integrator_QP_solver_CD_rd03, double_integrator_QP_solver_CD_Lbyrd03);
double_integrator_QP_solver_CD_LA_INEQ_DENSE_DIAG_HESS_23_6_6(double_integrator_QP_solver_CD_H00, double_integrator_QP_solver_CD_llbbyslb04, double_integrator_QP_solver_CD_lbIdx04, double_integrator_QP_solver_CD_lubbysub04, double_integrator_QP_solver_CD_ubIdx04, double_integrator_QP_solver_CD_Phi04);
double_integrator_QP_solver_CD_LA_DENSE_ADDMTDM_34_23(params->A5, double_integrator_QP_solver_CD_lpbysp04, double_integrator_QP_solver_CD_Phi04);
double_integrator_QP_solver_CD_LA_DENSE_CHOL2_23(double_integrator_QP_solver_CD_Phi04);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi04, double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_V04);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi04, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_W04);
double_integrator_QP_solver_CD_LA_DENSE_MMTM_7_23_7(double_integrator_QP_solver_CD_W04, double_integrator_QP_solver_CD_V04, double_integrator_QP_solver_CD_Ysd04);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi04, double_integrator_QP_solver_CD_rd04, double_integrator_QP_solver_CD_Lbyrd04);
double_integrator_QP_solver_CD_LA_INEQ_DENSE_DIAG_HESS_23_6_6(double_integrator_QP_solver_CD_H00, double_integrator_QP_solver_CD_llbbyslb05, double_integrator_QP_solver_CD_lbIdx05, double_integrator_QP_solver_CD_lubbysub05, double_integrator_QP_solver_CD_ubIdx05, double_integrator_QP_solver_CD_Phi05);
double_integrator_QP_solver_CD_LA_DENSE_ADDMTDM_34_23(params->A6, double_integrator_QP_solver_CD_lpbysp05, double_integrator_QP_solver_CD_Phi05);
double_integrator_QP_solver_CD_LA_DENSE_CHOL2_23(double_integrator_QP_solver_CD_Phi05);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi05, double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_V05);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi05, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_W05);
double_integrator_QP_solver_CD_LA_DENSE_MMTM_7_23_7(double_integrator_QP_solver_CD_W05, double_integrator_QP_solver_CD_V05, double_integrator_QP_solver_CD_Ysd05);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi05, double_integrator_QP_solver_CD_rd05, double_integrator_QP_solver_CD_Lbyrd05);
double_integrator_QP_solver_CD_LA_INEQ_DENSE_DIAG_HESS_23_6_6(double_integrator_QP_solver_CD_H00, double_integrator_QP_solver_CD_llbbyslb06, double_integrator_QP_solver_CD_lbIdx06, double_integrator_QP_solver_CD_lubbysub06, double_integrator_QP_solver_CD_ubIdx06, double_integrator_QP_solver_CD_Phi06);
double_integrator_QP_solver_CD_LA_DENSE_ADDMTDM_34_23(params->A7, double_integrator_QP_solver_CD_lpbysp06, double_integrator_QP_solver_CD_Phi06);
double_integrator_QP_solver_CD_LA_DENSE_CHOL2_23(double_integrator_QP_solver_CD_Phi06);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi06, double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_V06);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi06, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_W06);
double_integrator_QP_solver_CD_LA_DENSE_MMTM_7_23_7(double_integrator_QP_solver_CD_W06, double_integrator_QP_solver_CD_V06, double_integrator_QP_solver_CD_Ysd06);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi06, double_integrator_QP_solver_CD_rd06, double_integrator_QP_solver_CD_Lbyrd06);
double_integrator_QP_solver_CD_LA_INEQ_DENSE_DIAG_HESS_23_6_6(double_integrator_QP_solver_CD_H00, double_integrator_QP_solver_CD_llbbyslb07, double_integrator_QP_solver_CD_lbIdx07, double_integrator_QP_solver_CD_lubbysub07, double_integrator_QP_solver_CD_ubIdx07, double_integrator_QP_solver_CD_Phi07);
double_integrator_QP_solver_CD_LA_DENSE_ADDMTDM_34_23(params->A8, double_integrator_QP_solver_CD_lpbysp07, double_integrator_QP_solver_CD_Phi07);
double_integrator_QP_solver_CD_LA_DENSE_CHOL2_23(double_integrator_QP_solver_CD_Phi07);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi07, double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_V07);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi07, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_W07);
double_integrator_QP_solver_CD_LA_DENSE_MMTM_7_23_7(double_integrator_QP_solver_CD_W07, double_integrator_QP_solver_CD_V07, double_integrator_QP_solver_CD_Ysd07);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi07, double_integrator_QP_solver_CD_rd07, double_integrator_QP_solver_CD_Lbyrd07);
double_integrator_QP_solver_CD_LA_INEQ_DENSE_DIAG_HESS_23_6_6(double_integrator_QP_solver_CD_H00, double_integrator_QP_solver_CD_llbbyslb08, double_integrator_QP_solver_CD_lbIdx08, double_integrator_QP_solver_CD_lubbysub08, double_integrator_QP_solver_CD_ubIdx08, double_integrator_QP_solver_CD_Phi08);
double_integrator_QP_solver_CD_LA_DENSE_ADDMTDM_34_23(params->A9, double_integrator_QP_solver_CD_lpbysp08, double_integrator_QP_solver_CD_Phi08);
double_integrator_QP_solver_CD_LA_DENSE_CHOL2_23(double_integrator_QP_solver_CD_Phi08);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi08, double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_V08);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi08, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_W08);
double_integrator_QP_solver_CD_LA_DENSE_MMTM_7_23_7(double_integrator_QP_solver_CD_W08, double_integrator_QP_solver_CD_V08, double_integrator_QP_solver_CD_Ysd08);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi08, double_integrator_QP_solver_CD_rd08, double_integrator_QP_solver_CD_Lbyrd08);
double_integrator_QP_solver_CD_LA_INEQ_DENSE_DIAG_HESS_23_6_6(double_integrator_QP_solver_CD_H00, double_integrator_QP_solver_CD_llbbyslb09, double_integrator_QP_solver_CD_lbIdx09, double_integrator_QP_solver_CD_lubbysub09, double_integrator_QP_solver_CD_ubIdx09, double_integrator_QP_solver_CD_Phi09);
double_integrator_QP_solver_CD_LA_DENSE_ADDMTDM_34_23(params->A10, double_integrator_QP_solver_CD_lpbysp09, double_integrator_QP_solver_CD_Phi09);
double_integrator_QP_solver_CD_LA_DENSE_CHOL2_23(double_integrator_QP_solver_CD_Phi09);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi09, double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_V09);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_23(double_integrator_QP_solver_CD_Phi09, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_W09);
double_integrator_QP_solver_CD_LA_DENSE_MMTM_7_23_7(double_integrator_QP_solver_CD_W09, double_integrator_QP_solver_CD_V09, double_integrator_QP_solver_CD_Ysd09);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi09, double_integrator_QP_solver_CD_rd09, double_integrator_QP_solver_CD_Lbyrd09);
double_integrator_QP_solver_CD_LA_INEQ_DENSE_DIAG_HESS_21_6_6(double_integrator_QP_solver_CD_H10, double_integrator_QP_solver_CD_llbbyslb10, double_integrator_QP_solver_CD_lbIdx10, double_integrator_QP_solver_CD_lubbysub10, double_integrator_QP_solver_CD_ubIdx10, double_integrator_QP_solver_CD_Phi10);
double_integrator_QP_solver_CD_LA_DENSE_ADDMTDM_34_21(params->A11, double_integrator_QP_solver_CD_lpbysp10, double_integrator_QP_solver_CD_Phi10);
double_integrator_QP_solver_CD_LA_DENSE_CHOL2_21(double_integrator_QP_solver_CD_Phi10);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_4_21(double_integrator_QP_solver_CD_Phi10, double_integrator_QP_solver_CD_C10, double_integrator_QP_solver_CD_V10);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXFORWARDSUB_7_21(double_integrator_QP_solver_CD_Phi10, double_integrator_QP_solver_CD_D10, double_integrator_QP_solver_CD_W10);
double_integrator_QP_solver_CD_LA_DENSE_MMTM_7_21_4(double_integrator_QP_solver_CD_W10, double_integrator_QP_solver_CD_V10, double_integrator_QP_solver_CD_Ysd10);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_21(double_integrator_QP_solver_CD_Phi10, double_integrator_QP_solver_CD_rd10, double_integrator_QP_solver_CD_Lbyrd10);
double_integrator_QP_solver_CD_LA_INEQ_DENSE_DIAG_HESS_5_4_4(double_integrator_QP_solver_CD_H11, double_integrator_QP_solver_CD_llbbyslb11, double_integrator_QP_solver_CD_lbIdx11, double_integrator_QP_solver_CD_lubbysub11, double_integrator_QP_solver_CD_ubIdx11, double_integrator_QP_solver_CD_Phi11);
double_integrator_QP_solver_CD_LA_DENSE_ADDMTDM_14_7(params->A12, double_integrator_QP_solver_CD_lpbysp11, double_integrator_QP_solver_CD_Phi11);
double_integrator_QP_solver_CD_LA_DENSE_CHOL2_7(double_integrator_QP_solver_CD_Phi11);
double_integrator_QP_solver_CD_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_4_7(double_integrator_QP_solver_CD_Phi11, double_integrator_QP_solver_CD_D11, double_integrator_QP_solver_CD_W11);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Phi11, double_integrator_QP_solver_CD_rd11, double_integrator_QP_solver_CD_Lbyrd11);
double_integrator_QP_solver_CD_LA_DENSE_MMT2_7_23_23(double_integrator_QP_solver_CD_V00, double_integrator_QP_solver_CD_W01, double_integrator_QP_solver_CD_Yd00);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB2_7_23_23(double_integrator_QP_solver_CD_V00, double_integrator_QP_solver_CD_Lbyrd00, double_integrator_QP_solver_CD_W01, double_integrator_QP_solver_CD_Lbyrd01, double_integrator_QP_solver_CD_re00, double_integrator_QP_solver_CD_beta00);
double_integrator_QP_solver_CD_LA_DENSE_MMT2_7_23_23(double_integrator_QP_solver_CD_V01, double_integrator_QP_solver_CD_W02, double_integrator_QP_solver_CD_Yd01);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB2_7_23_23(double_integrator_QP_solver_CD_V01, double_integrator_QP_solver_CD_Lbyrd01, double_integrator_QP_solver_CD_W02, double_integrator_QP_solver_CD_Lbyrd02, double_integrator_QP_solver_CD_re01, double_integrator_QP_solver_CD_beta01);
double_integrator_QP_solver_CD_LA_DENSE_MMT2_7_23_23(double_integrator_QP_solver_CD_V02, double_integrator_QP_solver_CD_W03, double_integrator_QP_solver_CD_Yd02);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB2_7_23_23(double_integrator_QP_solver_CD_V02, double_integrator_QP_solver_CD_Lbyrd02, double_integrator_QP_solver_CD_W03, double_integrator_QP_solver_CD_Lbyrd03, double_integrator_QP_solver_CD_re02, double_integrator_QP_solver_CD_beta02);
double_integrator_QP_solver_CD_LA_DENSE_MMT2_7_23_23(double_integrator_QP_solver_CD_V03, double_integrator_QP_solver_CD_W04, double_integrator_QP_solver_CD_Yd03);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB2_7_23_23(double_integrator_QP_solver_CD_V03, double_integrator_QP_solver_CD_Lbyrd03, double_integrator_QP_solver_CD_W04, double_integrator_QP_solver_CD_Lbyrd04, double_integrator_QP_solver_CD_re03, double_integrator_QP_solver_CD_beta03);
double_integrator_QP_solver_CD_LA_DENSE_MMT2_7_23_23(double_integrator_QP_solver_CD_V04, double_integrator_QP_solver_CD_W05, double_integrator_QP_solver_CD_Yd04);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB2_7_23_23(double_integrator_QP_solver_CD_V04, double_integrator_QP_solver_CD_Lbyrd04, double_integrator_QP_solver_CD_W05, double_integrator_QP_solver_CD_Lbyrd05, double_integrator_QP_solver_CD_re04, double_integrator_QP_solver_CD_beta04);
double_integrator_QP_solver_CD_LA_DENSE_MMT2_7_23_23(double_integrator_QP_solver_CD_V05, double_integrator_QP_solver_CD_W06, double_integrator_QP_solver_CD_Yd05);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB2_7_23_23(double_integrator_QP_solver_CD_V05, double_integrator_QP_solver_CD_Lbyrd05, double_integrator_QP_solver_CD_W06, double_integrator_QP_solver_CD_Lbyrd06, double_integrator_QP_solver_CD_re05, double_integrator_QP_solver_CD_beta05);
double_integrator_QP_solver_CD_LA_DENSE_MMT2_7_23_23(double_integrator_QP_solver_CD_V06, double_integrator_QP_solver_CD_W07, double_integrator_QP_solver_CD_Yd06);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB2_7_23_23(double_integrator_QP_solver_CD_V06, double_integrator_QP_solver_CD_Lbyrd06, double_integrator_QP_solver_CD_W07, double_integrator_QP_solver_CD_Lbyrd07, double_integrator_QP_solver_CD_re06, double_integrator_QP_solver_CD_beta06);
double_integrator_QP_solver_CD_LA_DENSE_MMT2_7_23_23(double_integrator_QP_solver_CD_V07, double_integrator_QP_solver_CD_W08, double_integrator_QP_solver_CD_Yd07);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB2_7_23_23(double_integrator_QP_solver_CD_V07, double_integrator_QP_solver_CD_Lbyrd07, double_integrator_QP_solver_CD_W08, double_integrator_QP_solver_CD_Lbyrd08, double_integrator_QP_solver_CD_re07, double_integrator_QP_solver_CD_beta07);
double_integrator_QP_solver_CD_LA_DENSE_MMT2_7_23_23(double_integrator_QP_solver_CD_V08, double_integrator_QP_solver_CD_W09, double_integrator_QP_solver_CD_Yd08);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB2_7_23_23(double_integrator_QP_solver_CD_V08, double_integrator_QP_solver_CD_Lbyrd08, double_integrator_QP_solver_CD_W09, double_integrator_QP_solver_CD_Lbyrd09, double_integrator_QP_solver_CD_re08, double_integrator_QP_solver_CD_beta08);
double_integrator_QP_solver_CD_LA_DENSE_MMT2_7_23_21(double_integrator_QP_solver_CD_V09, double_integrator_QP_solver_CD_W10, double_integrator_QP_solver_CD_Yd09);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB2_7_23_21(double_integrator_QP_solver_CD_V09, double_integrator_QP_solver_CD_Lbyrd09, double_integrator_QP_solver_CD_W10, double_integrator_QP_solver_CD_Lbyrd10, double_integrator_QP_solver_CD_re09, double_integrator_QP_solver_CD_beta09);
double_integrator_QP_solver_CD_LA_DENSE_MMT2_4_21_7(double_integrator_QP_solver_CD_V10, double_integrator_QP_solver_CD_W11, double_integrator_QP_solver_CD_Yd10);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB2_4_21_7(double_integrator_QP_solver_CD_V10, double_integrator_QP_solver_CD_Lbyrd10, double_integrator_QP_solver_CD_W11, double_integrator_QP_solver_CD_Lbyrd11, double_integrator_QP_solver_CD_re10, double_integrator_QP_solver_CD_beta10);
double_integrator_QP_solver_CD_LA_DENSE_CHOL_7(double_integrator_QP_solver_CD_Yd00, double_integrator_QP_solver_CD_Ld00);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld00, double_integrator_QP_solver_CD_beta00, double_integrator_QP_solver_CD_yy00);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_CD_Ld00, double_integrator_QP_solver_CD_Ysd01, double_integrator_QP_solver_CD_Lsd01);
double_integrator_QP_solver_CD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_CD_Lsd01, double_integrator_QP_solver_CD_Yd01);
double_integrator_QP_solver_CD_LA_DENSE_CHOL_7(double_integrator_QP_solver_CD_Yd01, double_integrator_QP_solver_CD_Ld01);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd01, double_integrator_QP_solver_CD_yy00, double_integrator_QP_solver_CD_beta01, double_integrator_QP_solver_CD_bmy01);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld01, double_integrator_QP_solver_CD_bmy01, double_integrator_QP_solver_CD_yy01);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_CD_Ld01, double_integrator_QP_solver_CD_Ysd02, double_integrator_QP_solver_CD_Lsd02);
double_integrator_QP_solver_CD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_CD_Lsd02, double_integrator_QP_solver_CD_Yd02);
double_integrator_QP_solver_CD_LA_DENSE_CHOL_7(double_integrator_QP_solver_CD_Yd02, double_integrator_QP_solver_CD_Ld02);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd02, double_integrator_QP_solver_CD_yy01, double_integrator_QP_solver_CD_beta02, double_integrator_QP_solver_CD_bmy02);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld02, double_integrator_QP_solver_CD_bmy02, double_integrator_QP_solver_CD_yy02);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_CD_Ld02, double_integrator_QP_solver_CD_Ysd03, double_integrator_QP_solver_CD_Lsd03);
double_integrator_QP_solver_CD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_CD_Lsd03, double_integrator_QP_solver_CD_Yd03);
double_integrator_QP_solver_CD_LA_DENSE_CHOL_7(double_integrator_QP_solver_CD_Yd03, double_integrator_QP_solver_CD_Ld03);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd03, double_integrator_QP_solver_CD_yy02, double_integrator_QP_solver_CD_beta03, double_integrator_QP_solver_CD_bmy03);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld03, double_integrator_QP_solver_CD_bmy03, double_integrator_QP_solver_CD_yy03);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_CD_Ld03, double_integrator_QP_solver_CD_Ysd04, double_integrator_QP_solver_CD_Lsd04);
double_integrator_QP_solver_CD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_CD_Lsd04, double_integrator_QP_solver_CD_Yd04);
double_integrator_QP_solver_CD_LA_DENSE_CHOL_7(double_integrator_QP_solver_CD_Yd04, double_integrator_QP_solver_CD_Ld04);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd04, double_integrator_QP_solver_CD_yy03, double_integrator_QP_solver_CD_beta04, double_integrator_QP_solver_CD_bmy04);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld04, double_integrator_QP_solver_CD_bmy04, double_integrator_QP_solver_CD_yy04);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_CD_Ld04, double_integrator_QP_solver_CD_Ysd05, double_integrator_QP_solver_CD_Lsd05);
double_integrator_QP_solver_CD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_CD_Lsd05, double_integrator_QP_solver_CD_Yd05);
double_integrator_QP_solver_CD_LA_DENSE_CHOL_7(double_integrator_QP_solver_CD_Yd05, double_integrator_QP_solver_CD_Ld05);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd05, double_integrator_QP_solver_CD_yy04, double_integrator_QP_solver_CD_beta05, double_integrator_QP_solver_CD_bmy05);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld05, double_integrator_QP_solver_CD_bmy05, double_integrator_QP_solver_CD_yy05);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_CD_Ld05, double_integrator_QP_solver_CD_Ysd06, double_integrator_QP_solver_CD_Lsd06);
double_integrator_QP_solver_CD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_CD_Lsd06, double_integrator_QP_solver_CD_Yd06);
double_integrator_QP_solver_CD_LA_DENSE_CHOL_7(double_integrator_QP_solver_CD_Yd06, double_integrator_QP_solver_CD_Ld06);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd06, double_integrator_QP_solver_CD_yy05, double_integrator_QP_solver_CD_beta06, double_integrator_QP_solver_CD_bmy06);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld06, double_integrator_QP_solver_CD_bmy06, double_integrator_QP_solver_CD_yy06);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_CD_Ld06, double_integrator_QP_solver_CD_Ysd07, double_integrator_QP_solver_CD_Lsd07);
double_integrator_QP_solver_CD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_CD_Lsd07, double_integrator_QP_solver_CD_Yd07);
double_integrator_QP_solver_CD_LA_DENSE_CHOL_7(double_integrator_QP_solver_CD_Yd07, double_integrator_QP_solver_CD_Ld07);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd07, double_integrator_QP_solver_CD_yy06, double_integrator_QP_solver_CD_beta07, double_integrator_QP_solver_CD_bmy07);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld07, double_integrator_QP_solver_CD_bmy07, double_integrator_QP_solver_CD_yy07);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_CD_Ld07, double_integrator_QP_solver_CD_Ysd08, double_integrator_QP_solver_CD_Lsd08);
double_integrator_QP_solver_CD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_CD_Lsd08, double_integrator_QP_solver_CD_Yd08);
double_integrator_QP_solver_CD_LA_DENSE_CHOL_7(double_integrator_QP_solver_CD_Yd08, double_integrator_QP_solver_CD_Ld08);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd08, double_integrator_QP_solver_CD_yy07, double_integrator_QP_solver_CD_beta08, double_integrator_QP_solver_CD_bmy08);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld08, double_integrator_QP_solver_CD_bmy08, double_integrator_QP_solver_CD_yy08);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_CD_Ld08, double_integrator_QP_solver_CD_Ysd09, double_integrator_QP_solver_CD_Lsd09);
double_integrator_QP_solver_CD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_CD_Lsd09, double_integrator_QP_solver_CD_Yd09);
double_integrator_QP_solver_CD_LA_DENSE_CHOL_7(double_integrator_QP_solver_CD_Yd09, double_integrator_QP_solver_CD_Ld09);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd09, double_integrator_QP_solver_CD_yy08, double_integrator_QP_solver_CD_beta09, double_integrator_QP_solver_CD_bmy09);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld09, double_integrator_QP_solver_CD_bmy09, double_integrator_QP_solver_CD_yy09);
double_integrator_QP_solver_CD_LA_DENSE_MATRIXTFORWARDSUB_4_7(double_integrator_QP_solver_CD_Ld09, double_integrator_QP_solver_CD_Ysd10, double_integrator_QP_solver_CD_Lsd10);
double_integrator_QP_solver_CD_LA_DENSE_MMTSUB_4_7(double_integrator_QP_solver_CD_Lsd10, double_integrator_QP_solver_CD_Yd10);
double_integrator_QP_solver_CD_LA_DENSE_CHOL_4(double_integrator_QP_solver_CD_Yd10, double_integrator_QP_solver_CD_Ld10);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_4_7(double_integrator_QP_solver_CD_Lsd10, double_integrator_QP_solver_CD_yy09, double_integrator_QP_solver_CD_beta10, double_integrator_QP_solver_CD_bmy10);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_4(double_integrator_QP_solver_CD_Ld10, double_integrator_QP_solver_CD_bmy10, double_integrator_QP_solver_CD_yy10);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_4(double_integrator_QP_solver_CD_Ld10, double_integrator_QP_solver_CD_yy10, double_integrator_QP_solver_CD_dvaff10);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_4_7(double_integrator_QP_solver_CD_Lsd10, double_integrator_QP_solver_CD_dvaff10, double_integrator_QP_solver_CD_yy09, double_integrator_QP_solver_CD_bmy09);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld09, double_integrator_QP_solver_CD_bmy09, double_integrator_QP_solver_CD_dvaff09);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd09, double_integrator_QP_solver_CD_dvaff09, double_integrator_QP_solver_CD_yy08, double_integrator_QP_solver_CD_bmy08);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld08, double_integrator_QP_solver_CD_bmy08, double_integrator_QP_solver_CD_dvaff08);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd08, double_integrator_QP_solver_CD_dvaff08, double_integrator_QP_solver_CD_yy07, double_integrator_QP_solver_CD_bmy07);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld07, double_integrator_QP_solver_CD_bmy07, double_integrator_QP_solver_CD_dvaff07);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd07, double_integrator_QP_solver_CD_dvaff07, double_integrator_QP_solver_CD_yy06, double_integrator_QP_solver_CD_bmy06);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld06, double_integrator_QP_solver_CD_bmy06, double_integrator_QP_solver_CD_dvaff06);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd06, double_integrator_QP_solver_CD_dvaff06, double_integrator_QP_solver_CD_yy05, double_integrator_QP_solver_CD_bmy05);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld05, double_integrator_QP_solver_CD_bmy05, double_integrator_QP_solver_CD_dvaff05);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd05, double_integrator_QP_solver_CD_dvaff05, double_integrator_QP_solver_CD_yy04, double_integrator_QP_solver_CD_bmy04);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld04, double_integrator_QP_solver_CD_bmy04, double_integrator_QP_solver_CD_dvaff04);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd04, double_integrator_QP_solver_CD_dvaff04, double_integrator_QP_solver_CD_yy03, double_integrator_QP_solver_CD_bmy03);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld03, double_integrator_QP_solver_CD_bmy03, double_integrator_QP_solver_CD_dvaff03);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd03, double_integrator_QP_solver_CD_dvaff03, double_integrator_QP_solver_CD_yy02, double_integrator_QP_solver_CD_bmy02);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld02, double_integrator_QP_solver_CD_bmy02, double_integrator_QP_solver_CD_dvaff02);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd02, double_integrator_QP_solver_CD_dvaff02, double_integrator_QP_solver_CD_yy01, double_integrator_QP_solver_CD_bmy01);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld01, double_integrator_QP_solver_CD_bmy01, double_integrator_QP_solver_CD_dvaff01);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd01, double_integrator_QP_solver_CD_dvaff01, double_integrator_QP_solver_CD_yy00, double_integrator_QP_solver_CD_bmy00);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld00, double_integrator_QP_solver_CD_bmy00, double_integrator_QP_solver_CD_dvaff00);
double_integrator_QP_solver_CD_LA_DENSE_MTVM_7_23(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvaff00, double_integrator_QP_solver_CD_grad_eq00);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvaff01, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvaff00, double_integrator_QP_solver_CD_grad_eq01);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvaff02, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvaff01, double_integrator_QP_solver_CD_grad_eq02);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvaff03, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvaff02, double_integrator_QP_solver_CD_grad_eq03);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvaff04, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvaff03, double_integrator_QP_solver_CD_grad_eq04);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvaff05, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvaff04, double_integrator_QP_solver_CD_grad_eq05);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvaff06, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvaff05, double_integrator_QP_solver_CD_grad_eq06);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvaff07, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvaff06, double_integrator_QP_solver_CD_grad_eq07);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvaff08, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvaff07, double_integrator_QP_solver_CD_grad_eq08);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvaff09, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvaff08, double_integrator_QP_solver_CD_grad_eq09);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_4_21_7(double_integrator_QP_solver_CD_C10, double_integrator_QP_solver_CD_dvaff10, double_integrator_QP_solver_CD_D10, double_integrator_QP_solver_CD_dvaff09, double_integrator_QP_solver_CD_grad_eq10);
double_integrator_QP_solver_CD_LA_DIAGZERO_MTVM_4_7(double_integrator_QP_solver_CD_D11, double_integrator_QP_solver_CD_dvaff10, double_integrator_QP_solver_CD_grad_eq11);
double_integrator_QP_solver_CD_LA_VSUB2_258(double_integrator_QP_solver_CD_rd, double_integrator_QP_solver_CD_grad_eq, double_integrator_QP_solver_CD_rd);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi00, double_integrator_QP_solver_CD_rd00, double_integrator_QP_solver_CD_dzaff00);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi01, double_integrator_QP_solver_CD_rd01, double_integrator_QP_solver_CD_dzaff01);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi02, double_integrator_QP_solver_CD_rd02, double_integrator_QP_solver_CD_dzaff02);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi03, double_integrator_QP_solver_CD_rd03, double_integrator_QP_solver_CD_dzaff03);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi04, double_integrator_QP_solver_CD_rd04, double_integrator_QP_solver_CD_dzaff04);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi05, double_integrator_QP_solver_CD_rd05, double_integrator_QP_solver_CD_dzaff05);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi06, double_integrator_QP_solver_CD_rd06, double_integrator_QP_solver_CD_dzaff06);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi07, double_integrator_QP_solver_CD_rd07, double_integrator_QP_solver_CD_dzaff07);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi08, double_integrator_QP_solver_CD_rd08, double_integrator_QP_solver_CD_dzaff08);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi09, double_integrator_QP_solver_CD_rd09, double_integrator_QP_solver_CD_dzaff09);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_21(double_integrator_QP_solver_CD_Phi10, double_integrator_QP_solver_CD_rd10, double_integrator_QP_solver_CD_dzaff10);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_7(double_integrator_QP_solver_CD_Phi11, double_integrator_QP_solver_CD_rd11, double_integrator_QP_solver_CD_dzaff11);
double_integrator_QP_solver_CD_LA_VSUB_INDEXED_7(double_integrator_QP_solver_CD_dzaff00, double_integrator_QP_solver_CD_lbIdx00, double_integrator_QP_solver_CD_rilb00, double_integrator_QP_solver_CD_dslbaff00);
double_integrator_QP_solver_CD_LA_VSUB3_7(double_integrator_QP_solver_CD_llbbyslb00, double_integrator_QP_solver_CD_dslbaff00, double_integrator_QP_solver_CD_llb00, double_integrator_QP_solver_CD_dllbaff00);
double_integrator_QP_solver_CD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_CD_riub00, double_integrator_QP_solver_CD_dzaff00, double_integrator_QP_solver_CD_ubIdx00, double_integrator_QP_solver_CD_dsubaff00);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_lubbysub00, double_integrator_QP_solver_CD_dsubaff00, double_integrator_QP_solver_CD_lub00, double_integrator_QP_solver_CD_dlubaff00);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB4_34_23(params->A1, double_integrator_QP_solver_CD_dzaff00, double_integrator_QP_solver_CD_rip00, double_integrator_QP_solver_CD_dsp_aff00);
double_integrator_QP_solver_CD_LA_VSUB3_34(double_integrator_QP_solver_CD_lpbysp00, double_integrator_QP_solver_CD_dsp_aff00, double_integrator_QP_solver_CD_lp00, double_integrator_QP_solver_CD_dlp_aff00);
double_integrator_QP_solver_CD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_CD_dzaff01, double_integrator_QP_solver_CD_lbIdx01, double_integrator_QP_solver_CD_rilb01, double_integrator_QP_solver_CD_dslbaff01);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_llbbyslb01, double_integrator_QP_solver_CD_dslbaff01, double_integrator_QP_solver_CD_llb01, double_integrator_QP_solver_CD_dllbaff01);
double_integrator_QP_solver_CD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_CD_riub01, double_integrator_QP_solver_CD_dzaff01, double_integrator_QP_solver_CD_ubIdx01, double_integrator_QP_solver_CD_dsubaff01);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_lubbysub01, double_integrator_QP_solver_CD_dsubaff01, double_integrator_QP_solver_CD_lub01, double_integrator_QP_solver_CD_dlubaff01);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB4_34_23(params->A2, double_integrator_QP_solver_CD_dzaff01, double_integrator_QP_solver_CD_rip01, double_integrator_QP_solver_CD_dsp_aff01);
double_integrator_QP_solver_CD_LA_VSUB3_34(double_integrator_QP_solver_CD_lpbysp01, double_integrator_QP_solver_CD_dsp_aff01, double_integrator_QP_solver_CD_lp01, double_integrator_QP_solver_CD_dlp_aff01);
double_integrator_QP_solver_CD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_CD_dzaff02, double_integrator_QP_solver_CD_lbIdx02, double_integrator_QP_solver_CD_rilb02, double_integrator_QP_solver_CD_dslbaff02);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_llbbyslb02, double_integrator_QP_solver_CD_dslbaff02, double_integrator_QP_solver_CD_llb02, double_integrator_QP_solver_CD_dllbaff02);
double_integrator_QP_solver_CD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_CD_riub02, double_integrator_QP_solver_CD_dzaff02, double_integrator_QP_solver_CD_ubIdx02, double_integrator_QP_solver_CD_dsubaff02);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_lubbysub02, double_integrator_QP_solver_CD_dsubaff02, double_integrator_QP_solver_CD_lub02, double_integrator_QP_solver_CD_dlubaff02);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB4_34_23(params->A3, double_integrator_QP_solver_CD_dzaff02, double_integrator_QP_solver_CD_rip02, double_integrator_QP_solver_CD_dsp_aff02);
double_integrator_QP_solver_CD_LA_VSUB3_34(double_integrator_QP_solver_CD_lpbysp02, double_integrator_QP_solver_CD_dsp_aff02, double_integrator_QP_solver_CD_lp02, double_integrator_QP_solver_CD_dlp_aff02);
double_integrator_QP_solver_CD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_CD_dzaff03, double_integrator_QP_solver_CD_lbIdx03, double_integrator_QP_solver_CD_rilb03, double_integrator_QP_solver_CD_dslbaff03);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_llbbyslb03, double_integrator_QP_solver_CD_dslbaff03, double_integrator_QP_solver_CD_llb03, double_integrator_QP_solver_CD_dllbaff03);
double_integrator_QP_solver_CD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_CD_riub03, double_integrator_QP_solver_CD_dzaff03, double_integrator_QP_solver_CD_ubIdx03, double_integrator_QP_solver_CD_dsubaff03);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_lubbysub03, double_integrator_QP_solver_CD_dsubaff03, double_integrator_QP_solver_CD_lub03, double_integrator_QP_solver_CD_dlubaff03);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB4_34_23(params->A4, double_integrator_QP_solver_CD_dzaff03, double_integrator_QP_solver_CD_rip03, double_integrator_QP_solver_CD_dsp_aff03);
double_integrator_QP_solver_CD_LA_VSUB3_34(double_integrator_QP_solver_CD_lpbysp03, double_integrator_QP_solver_CD_dsp_aff03, double_integrator_QP_solver_CD_lp03, double_integrator_QP_solver_CD_dlp_aff03);
double_integrator_QP_solver_CD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_CD_dzaff04, double_integrator_QP_solver_CD_lbIdx04, double_integrator_QP_solver_CD_rilb04, double_integrator_QP_solver_CD_dslbaff04);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_llbbyslb04, double_integrator_QP_solver_CD_dslbaff04, double_integrator_QP_solver_CD_llb04, double_integrator_QP_solver_CD_dllbaff04);
double_integrator_QP_solver_CD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_CD_riub04, double_integrator_QP_solver_CD_dzaff04, double_integrator_QP_solver_CD_ubIdx04, double_integrator_QP_solver_CD_dsubaff04);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_lubbysub04, double_integrator_QP_solver_CD_dsubaff04, double_integrator_QP_solver_CD_lub04, double_integrator_QP_solver_CD_dlubaff04);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB4_34_23(params->A5, double_integrator_QP_solver_CD_dzaff04, double_integrator_QP_solver_CD_rip04, double_integrator_QP_solver_CD_dsp_aff04);
double_integrator_QP_solver_CD_LA_VSUB3_34(double_integrator_QP_solver_CD_lpbysp04, double_integrator_QP_solver_CD_dsp_aff04, double_integrator_QP_solver_CD_lp04, double_integrator_QP_solver_CD_dlp_aff04);
double_integrator_QP_solver_CD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_CD_dzaff05, double_integrator_QP_solver_CD_lbIdx05, double_integrator_QP_solver_CD_rilb05, double_integrator_QP_solver_CD_dslbaff05);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_llbbyslb05, double_integrator_QP_solver_CD_dslbaff05, double_integrator_QP_solver_CD_llb05, double_integrator_QP_solver_CD_dllbaff05);
double_integrator_QP_solver_CD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_CD_riub05, double_integrator_QP_solver_CD_dzaff05, double_integrator_QP_solver_CD_ubIdx05, double_integrator_QP_solver_CD_dsubaff05);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_lubbysub05, double_integrator_QP_solver_CD_dsubaff05, double_integrator_QP_solver_CD_lub05, double_integrator_QP_solver_CD_dlubaff05);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB4_34_23(params->A6, double_integrator_QP_solver_CD_dzaff05, double_integrator_QP_solver_CD_rip05, double_integrator_QP_solver_CD_dsp_aff05);
double_integrator_QP_solver_CD_LA_VSUB3_34(double_integrator_QP_solver_CD_lpbysp05, double_integrator_QP_solver_CD_dsp_aff05, double_integrator_QP_solver_CD_lp05, double_integrator_QP_solver_CD_dlp_aff05);
double_integrator_QP_solver_CD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_CD_dzaff06, double_integrator_QP_solver_CD_lbIdx06, double_integrator_QP_solver_CD_rilb06, double_integrator_QP_solver_CD_dslbaff06);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_llbbyslb06, double_integrator_QP_solver_CD_dslbaff06, double_integrator_QP_solver_CD_llb06, double_integrator_QP_solver_CD_dllbaff06);
double_integrator_QP_solver_CD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_CD_riub06, double_integrator_QP_solver_CD_dzaff06, double_integrator_QP_solver_CD_ubIdx06, double_integrator_QP_solver_CD_dsubaff06);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_lubbysub06, double_integrator_QP_solver_CD_dsubaff06, double_integrator_QP_solver_CD_lub06, double_integrator_QP_solver_CD_dlubaff06);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB4_34_23(params->A7, double_integrator_QP_solver_CD_dzaff06, double_integrator_QP_solver_CD_rip06, double_integrator_QP_solver_CD_dsp_aff06);
double_integrator_QP_solver_CD_LA_VSUB3_34(double_integrator_QP_solver_CD_lpbysp06, double_integrator_QP_solver_CD_dsp_aff06, double_integrator_QP_solver_CD_lp06, double_integrator_QP_solver_CD_dlp_aff06);
double_integrator_QP_solver_CD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_CD_dzaff07, double_integrator_QP_solver_CD_lbIdx07, double_integrator_QP_solver_CD_rilb07, double_integrator_QP_solver_CD_dslbaff07);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_llbbyslb07, double_integrator_QP_solver_CD_dslbaff07, double_integrator_QP_solver_CD_llb07, double_integrator_QP_solver_CD_dllbaff07);
double_integrator_QP_solver_CD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_CD_riub07, double_integrator_QP_solver_CD_dzaff07, double_integrator_QP_solver_CD_ubIdx07, double_integrator_QP_solver_CD_dsubaff07);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_lubbysub07, double_integrator_QP_solver_CD_dsubaff07, double_integrator_QP_solver_CD_lub07, double_integrator_QP_solver_CD_dlubaff07);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB4_34_23(params->A8, double_integrator_QP_solver_CD_dzaff07, double_integrator_QP_solver_CD_rip07, double_integrator_QP_solver_CD_dsp_aff07);
double_integrator_QP_solver_CD_LA_VSUB3_34(double_integrator_QP_solver_CD_lpbysp07, double_integrator_QP_solver_CD_dsp_aff07, double_integrator_QP_solver_CD_lp07, double_integrator_QP_solver_CD_dlp_aff07);
double_integrator_QP_solver_CD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_CD_dzaff08, double_integrator_QP_solver_CD_lbIdx08, double_integrator_QP_solver_CD_rilb08, double_integrator_QP_solver_CD_dslbaff08);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_llbbyslb08, double_integrator_QP_solver_CD_dslbaff08, double_integrator_QP_solver_CD_llb08, double_integrator_QP_solver_CD_dllbaff08);
double_integrator_QP_solver_CD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_CD_riub08, double_integrator_QP_solver_CD_dzaff08, double_integrator_QP_solver_CD_ubIdx08, double_integrator_QP_solver_CD_dsubaff08);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_lubbysub08, double_integrator_QP_solver_CD_dsubaff08, double_integrator_QP_solver_CD_lub08, double_integrator_QP_solver_CD_dlubaff08);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB4_34_23(params->A9, double_integrator_QP_solver_CD_dzaff08, double_integrator_QP_solver_CD_rip08, double_integrator_QP_solver_CD_dsp_aff08);
double_integrator_QP_solver_CD_LA_VSUB3_34(double_integrator_QP_solver_CD_lpbysp08, double_integrator_QP_solver_CD_dsp_aff08, double_integrator_QP_solver_CD_lp08, double_integrator_QP_solver_CD_dlp_aff08);
double_integrator_QP_solver_CD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_CD_dzaff09, double_integrator_QP_solver_CD_lbIdx09, double_integrator_QP_solver_CD_rilb09, double_integrator_QP_solver_CD_dslbaff09);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_llbbyslb09, double_integrator_QP_solver_CD_dslbaff09, double_integrator_QP_solver_CD_llb09, double_integrator_QP_solver_CD_dllbaff09);
double_integrator_QP_solver_CD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_CD_riub09, double_integrator_QP_solver_CD_dzaff09, double_integrator_QP_solver_CD_ubIdx09, double_integrator_QP_solver_CD_dsubaff09);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_lubbysub09, double_integrator_QP_solver_CD_dsubaff09, double_integrator_QP_solver_CD_lub09, double_integrator_QP_solver_CD_dlubaff09);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB4_34_23(params->A10, double_integrator_QP_solver_CD_dzaff09, double_integrator_QP_solver_CD_rip09, double_integrator_QP_solver_CD_dsp_aff09);
double_integrator_QP_solver_CD_LA_VSUB3_34(double_integrator_QP_solver_CD_lpbysp09, double_integrator_QP_solver_CD_dsp_aff09, double_integrator_QP_solver_CD_lp09, double_integrator_QP_solver_CD_dlp_aff09);
double_integrator_QP_solver_CD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_CD_dzaff10, double_integrator_QP_solver_CD_lbIdx10, double_integrator_QP_solver_CD_rilb10, double_integrator_QP_solver_CD_dslbaff10);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_llbbyslb10, double_integrator_QP_solver_CD_dslbaff10, double_integrator_QP_solver_CD_llb10, double_integrator_QP_solver_CD_dllbaff10);
double_integrator_QP_solver_CD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_CD_riub10, double_integrator_QP_solver_CD_dzaff10, double_integrator_QP_solver_CD_ubIdx10, double_integrator_QP_solver_CD_dsubaff10);
double_integrator_QP_solver_CD_LA_VSUB3_6(double_integrator_QP_solver_CD_lubbysub10, double_integrator_QP_solver_CD_dsubaff10, double_integrator_QP_solver_CD_lub10, double_integrator_QP_solver_CD_dlubaff10);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB4_34_21(params->A11, double_integrator_QP_solver_CD_dzaff10, double_integrator_QP_solver_CD_rip10, double_integrator_QP_solver_CD_dsp_aff10);
double_integrator_QP_solver_CD_LA_VSUB3_34(double_integrator_QP_solver_CD_lpbysp10, double_integrator_QP_solver_CD_dsp_aff10, double_integrator_QP_solver_CD_lp10, double_integrator_QP_solver_CD_dlp_aff10);
double_integrator_QP_solver_CD_LA_VSUB_INDEXED_4(double_integrator_QP_solver_CD_dzaff11, double_integrator_QP_solver_CD_lbIdx11, double_integrator_QP_solver_CD_rilb11, double_integrator_QP_solver_CD_dslbaff11);
double_integrator_QP_solver_CD_LA_VSUB3_4(double_integrator_QP_solver_CD_llbbyslb11, double_integrator_QP_solver_CD_dslbaff11, double_integrator_QP_solver_CD_llb11, double_integrator_QP_solver_CD_dllbaff11);
double_integrator_QP_solver_CD_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_CD_riub11, double_integrator_QP_solver_CD_dzaff11, double_integrator_QP_solver_CD_ubIdx11, double_integrator_QP_solver_CD_dsubaff11);
double_integrator_QP_solver_CD_LA_VSUB3_4(double_integrator_QP_solver_CD_lubbysub11, double_integrator_QP_solver_CD_dsubaff11, double_integrator_QP_solver_CD_lub11, double_integrator_QP_solver_CD_dlubaff11);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB4_14_7(params->A12, double_integrator_QP_solver_CD_dzaff11, double_integrator_QP_solver_CD_rip11, double_integrator_QP_solver_CD_dsp_aff11);
double_integrator_QP_solver_CD_LA_VSUB3_14(double_integrator_QP_solver_CD_lpbysp11, double_integrator_QP_solver_CD_dsp_aff11, double_integrator_QP_solver_CD_lp11, double_integrator_QP_solver_CD_dlp_aff11);
info->lsit_aff = double_integrator_QP_solver_CD_LINESEARCH_BACKTRACKING_AFFINE(double_integrator_QP_solver_CD_l, double_integrator_QP_solver_CD_s, double_integrator_QP_solver_CD_dl_aff, double_integrator_QP_solver_CD_ds_aff, &info->step_aff, &info->mu_aff);
if( info->lsit_aff == double_integrator_QP_solver_CD_NOPROGRESS ){
PRINTTEXT("Affine line search could not proceed at iteration %d.\nThe problem might be infeasible -- exiting.\n",info->it+1);
exitcode = double_integrator_QP_solver_CD_NOPROGRESS; break;
}
sigma_3rdroot = info->mu_aff / info->mu;
info->sigma = sigma_3rdroot*sigma_3rdroot*sigma_3rdroot;
musigma = info->mu * info->sigma;
double_integrator_QP_solver_CD_LA_VSUB5_529(double_integrator_QP_solver_CD_ds_aff, double_integrator_QP_solver_CD_dl_aff, info->mu, info->sigma, double_integrator_QP_solver_CD_ccrhs);
double_integrator_QP_solver_CD_LA_VSUB6_INDEXED_23_6_7(double_integrator_QP_solver_CD_ccrhsub00, double_integrator_QP_solver_CD_sub00, double_integrator_QP_solver_CD_ubIdx00, double_integrator_QP_solver_CD_ccrhsl00, double_integrator_QP_solver_CD_slb00, double_integrator_QP_solver_CD_lbIdx00, double_integrator_QP_solver_CD_rd00);
double_integrator_QP_solver_CD_LA_VSUB6_INDEXED_23_6_6(double_integrator_QP_solver_CD_ccrhsub01, double_integrator_QP_solver_CD_sub01, double_integrator_QP_solver_CD_ubIdx01, double_integrator_QP_solver_CD_ccrhsl01, double_integrator_QP_solver_CD_slb01, double_integrator_QP_solver_CD_lbIdx01, double_integrator_QP_solver_CD_rd01);
double_integrator_QP_solver_CD_LA_DENSE_MTVMADD2_34_23(params->A1, double_integrator_QP_solver_CD_ccrhsp00, double_integrator_QP_solver_CD_sp00, double_integrator_QP_solver_CD_rd00);
double_integrator_QP_solver_CD_LA_DENSE_MTVMADD2_34_23(params->A2, double_integrator_QP_solver_CD_ccrhsp01, double_integrator_QP_solver_CD_sp01, double_integrator_QP_solver_CD_rd01);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi00, double_integrator_QP_solver_CD_rd00, double_integrator_QP_solver_CD_Lbyrd00);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi01, double_integrator_QP_solver_CD_rd01, double_integrator_QP_solver_CD_Lbyrd01);
double_integrator_QP_solver_CD_LA_DENSE_2MVMADD_7_23_23(double_integrator_QP_solver_CD_V00, double_integrator_QP_solver_CD_Lbyrd00, double_integrator_QP_solver_CD_W01, double_integrator_QP_solver_CD_Lbyrd01, double_integrator_QP_solver_CD_beta00);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld00, double_integrator_QP_solver_CD_beta00, double_integrator_QP_solver_CD_yy00);
double_integrator_QP_solver_CD_LA_VSUB6_INDEXED_23_6_6(double_integrator_QP_solver_CD_ccrhsub02, double_integrator_QP_solver_CD_sub02, double_integrator_QP_solver_CD_ubIdx02, double_integrator_QP_solver_CD_ccrhsl02, double_integrator_QP_solver_CD_slb02, double_integrator_QP_solver_CD_lbIdx02, double_integrator_QP_solver_CD_rd02);
double_integrator_QP_solver_CD_LA_DENSE_MTVMADD2_34_23(params->A3, double_integrator_QP_solver_CD_ccrhsp02, double_integrator_QP_solver_CD_sp02, double_integrator_QP_solver_CD_rd02);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi02, double_integrator_QP_solver_CD_rd02, double_integrator_QP_solver_CD_Lbyrd02);
double_integrator_QP_solver_CD_LA_DENSE_2MVMADD_7_23_23(double_integrator_QP_solver_CD_V01, double_integrator_QP_solver_CD_Lbyrd01, double_integrator_QP_solver_CD_W02, double_integrator_QP_solver_CD_Lbyrd02, double_integrator_QP_solver_CD_beta01);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd01, double_integrator_QP_solver_CD_yy00, double_integrator_QP_solver_CD_beta01, double_integrator_QP_solver_CD_bmy01);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld01, double_integrator_QP_solver_CD_bmy01, double_integrator_QP_solver_CD_yy01);
double_integrator_QP_solver_CD_LA_VSUB6_INDEXED_23_6_6(double_integrator_QP_solver_CD_ccrhsub03, double_integrator_QP_solver_CD_sub03, double_integrator_QP_solver_CD_ubIdx03, double_integrator_QP_solver_CD_ccrhsl03, double_integrator_QP_solver_CD_slb03, double_integrator_QP_solver_CD_lbIdx03, double_integrator_QP_solver_CD_rd03);
double_integrator_QP_solver_CD_LA_DENSE_MTVMADD2_34_23(params->A4, double_integrator_QP_solver_CD_ccrhsp03, double_integrator_QP_solver_CD_sp03, double_integrator_QP_solver_CD_rd03);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi03, double_integrator_QP_solver_CD_rd03, double_integrator_QP_solver_CD_Lbyrd03);
double_integrator_QP_solver_CD_LA_DENSE_2MVMADD_7_23_23(double_integrator_QP_solver_CD_V02, double_integrator_QP_solver_CD_Lbyrd02, double_integrator_QP_solver_CD_W03, double_integrator_QP_solver_CD_Lbyrd03, double_integrator_QP_solver_CD_beta02);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd02, double_integrator_QP_solver_CD_yy01, double_integrator_QP_solver_CD_beta02, double_integrator_QP_solver_CD_bmy02);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld02, double_integrator_QP_solver_CD_bmy02, double_integrator_QP_solver_CD_yy02);
double_integrator_QP_solver_CD_LA_VSUB6_INDEXED_23_6_6(double_integrator_QP_solver_CD_ccrhsub04, double_integrator_QP_solver_CD_sub04, double_integrator_QP_solver_CD_ubIdx04, double_integrator_QP_solver_CD_ccrhsl04, double_integrator_QP_solver_CD_slb04, double_integrator_QP_solver_CD_lbIdx04, double_integrator_QP_solver_CD_rd04);
double_integrator_QP_solver_CD_LA_DENSE_MTVMADD2_34_23(params->A5, double_integrator_QP_solver_CD_ccrhsp04, double_integrator_QP_solver_CD_sp04, double_integrator_QP_solver_CD_rd04);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi04, double_integrator_QP_solver_CD_rd04, double_integrator_QP_solver_CD_Lbyrd04);
double_integrator_QP_solver_CD_LA_DENSE_2MVMADD_7_23_23(double_integrator_QP_solver_CD_V03, double_integrator_QP_solver_CD_Lbyrd03, double_integrator_QP_solver_CD_W04, double_integrator_QP_solver_CD_Lbyrd04, double_integrator_QP_solver_CD_beta03);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd03, double_integrator_QP_solver_CD_yy02, double_integrator_QP_solver_CD_beta03, double_integrator_QP_solver_CD_bmy03);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld03, double_integrator_QP_solver_CD_bmy03, double_integrator_QP_solver_CD_yy03);
double_integrator_QP_solver_CD_LA_VSUB6_INDEXED_23_6_6(double_integrator_QP_solver_CD_ccrhsub05, double_integrator_QP_solver_CD_sub05, double_integrator_QP_solver_CD_ubIdx05, double_integrator_QP_solver_CD_ccrhsl05, double_integrator_QP_solver_CD_slb05, double_integrator_QP_solver_CD_lbIdx05, double_integrator_QP_solver_CD_rd05);
double_integrator_QP_solver_CD_LA_DENSE_MTVMADD2_34_23(params->A6, double_integrator_QP_solver_CD_ccrhsp05, double_integrator_QP_solver_CD_sp05, double_integrator_QP_solver_CD_rd05);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi05, double_integrator_QP_solver_CD_rd05, double_integrator_QP_solver_CD_Lbyrd05);
double_integrator_QP_solver_CD_LA_DENSE_2MVMADD_7_23_23(double_integrator_QP_solver_CD_V04, double_integrator_QP_solver_CD_Lbyrd04, double_integrator_QP_solver_CD_W05, double_integrator_QP_solver_CD_Lbyrd05, double_integrator_QP_solver_CD_beta04);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd04, double_integrator_QP_solver_CD_yy03, double_integrator_QP_solver_CD_beta04, double_integrator_QP_solver_CD_bmy04);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld04, double_integrator_QP_solver_CD_bmy04, double_integrator_QP_solver_CD_yy04);
double_integrator_QP_solver_CD_LA_VSUB6_INDEXED_23_6_6(double_integrator_QP_solver_CD_ccrhsub06, double_integrator_QP_solver_CD_sub06, double_integrator_QP_solver_CD_ubIdx06, double_integrator_QP_solver_CD_ccrhsl06, double_integrator_QP_solver_CD_slb06, double_integrator_QP_solver_CD_lbIdx06, double_integrator_QP_solver_CD_rd06);
double_integrator_QP_solver_CD_LA_DENSE_MTVMADD2_34_23(params->A7, double_integrator_QP_solver_CD_ccrhsp06, double_integrator_QP_solver_CD_sp06, double_integrator_QP_solver_CD_rd06);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi06, double_integrator_QP_solver_CD_rd06, double_integrator_QP_solver_CD_Lbyrd06);
double_integrator_QP_solver_CD_LA_DENSE_2MVMADD_7_23_23(double_integrator_QP_solver_CD_V05, double_integrator_QP_solver_CD_Lbyrd05, double_integrator_QP_solver_CD_W06, double_integrator_QP_solver_CD_Lbyrd06, double_integrator_QP_solver_CD_beta05);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd05, double_integrator_QP_solver_CD_yy04, double_integrator_QP_solver_CD_beta05, double_integrator_QP_solver_CD_bmy05);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld05, double_integrator_QP_solver_CD_bmy05, double_integrator_QP_solver_CD_yy05);
double_integrator_QP_solver_CD_LA_VSUB6_INDEXED_23_6_6(double_integrator_QP_solver_CD_ccrhsub07, double_integrator_QP_solver_CD_sub07, double_integrator_QP_solver_CD_ubIdx07, double_integrator_QP_solver_CD_ccrhsl07, double_integrator_QP_solver_CD_slb07, double_integrator_QP_solver_CD_lbIdx07, double_integrator_QP_solver_CD_rd07);
double_integrator_QP_solver_CD_LA_DENSE_MTVMADD2_34_23(params->A8, double_integrator_QP_solver_CD_ccrhsp07, double_integrator_QP_solver_CD_sp07, double_integrator_QP_solver_CD_rd07);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi07, double_integrator_QP_solver_CD_rd07, double_integrator_QP_solver_CD_Lbyrd07);
double_integrator_QP_solver_CD_LA_DENSE_2MVMADD_7_23_23(double_integrator_QP_solver_CD_V06, double_integrator_QP_solver_CD_Lbyrd06, double_integrator_QP_solver_CD_W07, double_integrator_QP_solver_CD_Lbyrd07, double_integrator_QP_solver_CD_beta06);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd06, double_integrator_QP_solver_CD_yy05, double_integrator_QP_solver_CD_beta06, double_integrator_QP_solver_CD_bmy06);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld06, double_integrator_QP_solver_CD_bmy06, double_integrator_QP_solver_CD_yy06);
double_integrator_QP_solver_CD_LA_VSUB6_INDEXED_23_6_6(double_integrator_QP_solver_CD_ccrhsub08, double_integrator_QP_solver_CD_sub08, double_integrator_QP_solver_CD_ubIdx08, double_integrator_QP_solver_CD_ccrhsl08, double_integrator_QP_solver_CD_slb08, double_integrator_QP_solver_CD_lbIdx08, double_integrator_QP_solver_CD_rd08);
double_integrator_QP_solver_CD_LA_DENSE_MTVMADD2_34_23(params->A9, double_integrator_QP_solver_CD_ccrhsp08, double_integrator_QP_solver_CD_sp08, double_integrator_QP_solver_CD_rd08);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi08, double_integrator_QP_solver_CD_rd08, double_integrator_QP_solver_CD_Lbyrd08);
double_integrator_QP_solver_CD_LA_DENSE_2MVMADD_7_23_23(double_integrator_QP_solver_CD_V07, double_integrator_QP_solver_CD_Lbyrd07, double_integrator_QP_solver_CD_W08, double_integrator_QP_solver_CD_Lbyrd08, double_integrator_QP_solver_CD_beta07);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd07, double_integrator_QP_solver_CD_yy06, double_integrator_QP_solver_CD_beta07, double_integrator_QP_solver_CD_bmy07);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld07, double_integrator_QP_solver_CD_bmy07, double_integrator_QP_solver_CD_yy07);
double_integrator_QP_solver_CD_LA_VSUB6_INDEXED_23_6_6(double_integrator_QP_solver_CD_ccrhsub09, double_integrator_QP_solver_CD_sub09, double_integrator_QP_solver_CD_ubIdx09, double_integrator_QP_solver_CD_ccrhsl09, double_integrator_QP_solver_CD_slb09, double_integrator_QP_solver_CD_lbIdx09, double_integrator_QP_solver_CD_rd09);
double_integrator_QP_solver_CD_LA_DENSE_MTVMADD2_34_23(params->A10, double_integrator_QP_solver_CD_ccrhsp09, double_integrator_QP_solver_CD_sp09, double_integrator_QP_solver_CD_rd09);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_23(double_integrator_QP_solver_CD_Phi09, double_integrator_QP_solver_CD_rd09, double_integrator_QP_solver_CD_Lbyrd09);
double_integrator_QP_solver_CD_LA_DENSE_2MVMADD_7_23_23(double_integrator_QP_solver_CD_V08, double_integrator_QP_solver_CD_Lbyrd08, double_integrator_QP_solver_CD_W09, double_integrator_QP_solver_CD_Lbyrd09, double_integrator_QP_solver_CD_beta08);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd08, double_integrator_QP_solver_CD_yy07, double_integrator_QP_solver_CD_beta08, double_integrator_QP_solver_CD_bmy08);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld08, double_integrator_QP_solver_CD_bmy08, double_integrator_QP_solver_CD_yy08);
double_integrator_QP_solver_CD_LA_VSUB6_INDEXED_21_6_6(double_integrator_QP_solver_CD_ccrhsub10, double_integrator_QP_solver_CD_sub10, double_integrator_QP_solver_CD_ubIdx10, double_integrator_QP_solver_CD_ccrhsl10, double_integrator_QP_solver_CD_slb10, double_integrator_QP_solver_CD_lbIdx10, double_integrator_QP_solver_CD_rd10);
double_integrator_QP_solver_CD_LA_DENSE_MTVMADD2_34_21(params->A11, double_integrator_QP_solver_CD_ccrhsp10, double_integrator_QP_solver_CD_sp10, double_integrator_QP_solver_CD_rd10);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_21(double_integrator_QP_solver_CD_Phi10, double_integrator_QP_solver_CD_rd10, double_integrator_QP_solver_CD_Lbyrd10);
double_integrator_QP_solver_CD_LA_DENSE_2MVMADD_7_23_21(double_integrator_QP_solver_CD_V09, double_integrator_QP_solver_CD_Lbyrd09, double_integrator_QP_solver_CD_W10, double_integrator_QP_solver_CD_Lbyrd10, double_integrator_QP_solver_CD_beta09);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_CD_Lsd09, double_integrator_QP_solver_CD_yy08, double_integrator_QP_solver_CD_beta09, double_integrator_QP_solver_CD_bmy09);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Ld09, double_integrator_QP_solver_CD_bmy09, double_integrator_QP_solver_CD_yy09);
double_integrator_QP_solver_CD_LA_VSUB6_INDEXED_7_4_4(double_integrator_QP_solver_CD_ccrhsub11, double_integrator_QP_solver_CD_sub11, double_integrator_QP_solver_CD_ubIdx11, double_integrator_QP_solver_CD_ccrhsl11, double_integrator_QP_solver_CD_slb11, double_integrator_QP_solver_CD_lbIdx11, double_integrator_QP_solver_CD_rd11);
double_integrator_QP_solver_CD_LA_DENSE_MTVMADD2_14_7(params->A12, double_integrator_QP_solver_CD_ccrhsp11, double_integrator_QP_solver_CD_sp11, double_integrator_QP_solver_CD_rd11);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_CD_Phi11, double_integrator_QP_solver_CD_rd11, double_integrator_QP_solver_CD_Lbyrd11);
double_integrator_QP_solver_CD_LA_DENSE_2MVMADD_4_21_7(double_integrator_QP_solver_CD_V10, double_integrator_QP_solver_CD_Lbyrd10, double_integrator_QP_solver_CD_W11, double_integrator_QP_solver_CD_Lbyrd11, double_integrator_QP_solver_CD_beta10);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB1_4_7(double_integrator_QP_solver_CD_Lsd10, double_integrator_QP_solver_CD_yy09, double_integrator_QP_solver_CD_beta10, double_integrator_QP_solver_CD_bmy10);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDSUB_4(double_integrator_QP_solver_CD_Ld10, double_integrator_QP_solver_CD_bmy10, double_integrator_QP_solver_CD_yy10);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_4(double_integrator_QP_solver_CD_Ld10, double_integrator_QP_solver_CD_yy10, double_integrator_QP_solver_CD_dvcc10);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_4_7(double_integrator_QP_solver_CD_Lsd10, double_integrator_QP_solver_CD_dvcc10, double_integrator_QP_solver_CD_yy09, double_integrator_QP_solver_CD_bmy09);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld09, double_integrator_QP_solver_CD_bmy09, double_integrator_QP_solver_CD_dvcc09);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd09, double_integrator_QP_solver_CD_dvcc09, double_integrator_QP_solver_CD_yy08, double_integrator_QP_solver_CD_bmy08);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld08, double_integrator_QP_solver_CD_bmy08, double_integrator_QP_solver_CD_dvcc08);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd08, double_integrator_QP_solver_CD_dvcc08, double_integrator_QP_solver_CD_yy07, double_integrator_QP_solver_CD_bmy07);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld07, double_integrator_QP_solver_CD_bmy07, double_integrator_QP_solver_CD_dvcc07);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd07, double_integrator_QP_solver_CD_dvcc07, double_integrator_QP_solver_CD_yy06, double_integrator_QP_solver_CD_bmy06);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld06, double_integrator_QP_solver_CD_bmy06, double_integrator_QP_solver_CD_dvcc06);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd06, double_integrator_QP_solver_CD_dvcc06, double_integrator_QP_solver_CD_yy05, double_integrator_QP_solver_CD_bmy05);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld05, double_integrator_QP_solver_CD_bmy05, double_integrator_QP_solver_CD_dvcc05);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd05, double_integrator_QP_solver_CD_dvcc05, double_integrator_QP_solver_CD_yy04, double_integrator_QP_solver_CD_bmy04);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld04, double_integrator_QP_solver_CD_bmy04, double_integrator_QP_solver_CD_dvcc04);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd04, double_integrator_QP_solver_CD_dvcc04, double_integrator_QP_solver_CD_yy03, double_integrator_QP_solver_CD_bmy03);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld03, double_integrator_QP_solver_CD_bmy03, double_integrator_QP_solver_CD_dvcc03);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd03, double_integrator_QP_solver_CD_dvcc03, double_integrator_QP_solver_CD_yy02, double_integrator_QP_solver_CD_bmy02);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld02, double_integrator_QP_solver_CD_bmy02, double_integrator_QP_solver_CD_dvcc02);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd02, double_integrator_QP_solver_CD_dvcc02, double_integrator_QP_solver_CD_yy01, double_integrator_QP_solver_CD_bmy01);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld01, double_integrator_QP_solver_CD_bmy01, double_integrator_QP_solver_CD_dvcc01);
double_integrator_QP_solver_CD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_CD_Lsd01, double_integrator_QP_solver_CD_dvcc01, double_integrator_QP_solver_CD_yy00, double_integrator_QP_solver_CD_bmy00);
double_integrator_QP_solver_CD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_CD_Ld00, double_integrator_QP_solver_CD_bmy00, double_integrator_QP_solver_CD_dvcc00);
double_integrator_QP_solver_CD_LA_DENSE_MTVM_7_23(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvcc00, double_integrator_QP_solver_CD_grad_eq00);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvcc01, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvcc00, double_integrator_QP_solver_CD_grad_eq01);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvcc02, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvcc01, double_integrator_QP_solver_CD_grad_eq02);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvcc03, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvcc02, double_integrator_QP_solver_CD_grad_eq03);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvcc04, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvcc03, double_integrator_QP_solver_CD_grad_eq04);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvcc05, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvcc04, double_integrator_QP_solver_CD_grad_eq05);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvcc06, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvcc05, double_integrator_QP_solver_CD_grad_eq06);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvcc07, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvcc06, double_integrator_QP_solver_CD_grad_eq07);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvcc08, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvcc07, double_integrator_QP_solver_CD_grad_eq08);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_7_23_7(double_integrator_QP_solver_CD_C00, double_integrator_QP_solver_CD_dvcc09, double_integrator_QP_solver_CD_D01, double_integrator_QP_solver_CD_dvcc08, double_integrator_QP_solver_CD_grad_eq09);
double_integrator_QP_solver_CD_LA_DENSE_MTVM2_4_21_7(double_integrator_QP_solver_CD_C10, double_integrator_QP_solver_CD_dvcc10, double_integrator_QP_solver_CD_D10, double_integrator_QP_solver_CD_dvcc09, double_integrator_QP_solver_CD_grad_eq10);
double_integrator_QP_solver_CD_LA_DIAGZERO_MTVM_4_7(double_integrator_QP_solver_CD_D11, double_integrator_QP_solver_CD_dvcc10, double_integrator_QP_solver_CD_grad_eq11);
double_integrator_QP_solver_CD_LA_VSUB_258(double_integrator_QP_solver_CD_rd, double_integrator_QP_solver_CD_grad_eq, double_integrator_QP_solver_CD_rd);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi00, double_integrator_QP_solver_CD_rd00, double_integrator_QP_solver_CD_dzcc00);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi01, double_integrator_QP_solver_CD_rd01, double_integrator_QP_solver_CD_dzcc01);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi02, double_integrator_QP_solver_CD_rd02, double_integrator_QP_solver_CD_dzcc02);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi03, double_integrator_QP_solver_CD_rd03, double_integrator_QP_solver_CD_dzcc03);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi04, double_integrator_QP_solver_CD_rd04, double_integrator_QP_solver_CD_dzcc04);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi05, double_integrator_QP_solver_CD_rd05, double_integrator_QP_solver_CD_dzcc05);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi06, double_integrator_QP_solver_CD_rd06, double_integrator_QP_solver_CD_dzcc06);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi07, double_integrator_QP_solver_CD_rd07, double_integrator_QP_solver_CD_dzcc07);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi08, double_integrator_QP_solver_CD_rd08, double_integrator_QP_solver_CD_dzcc08);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_23(double_integrator_QP_solver_CD_Phi09, double_integrator_QP_solver_CD_rd09, double_integrator_QP_solver_CD_dzcc09);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_21(double_integrator_QP_solver_CD_Phi10, double_integrator_QP_solver_CD_rd10, double_integrator_QP_solver_CD_dzcc10);
double_integrator_QP_solver_CD_LA_DENSE_FORWARDBACKWARDSUB_7(double_integrator_QP_solver_CD_Phi11, double_integrator_QP_solver_CD_rd11, double_integrator_QP_solver_CD_dzcc11);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_CD_ccrhsl00, double_integrator_QP_solver_CD_slb00, double_integrator_QP_solver_CD_llbbyslb00, double_integrator_QP_solver_CD_dzcc00, double_integrator_QP_solver_CD_lbIdx00, double_integrator_QP_solver_CD_dllbcc00);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_CD_ccrhsub00, double_integrator_QP_solver_CD_sub00, double_integrator_QP_solver_CD_lubbysub00, double_integrator_QP_solver_CD_dzcc00, double_integrator_QP_solver_CD_ubIdx00, double_integrator_QP_solver_CD_dlubcc00);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB5_34_23(params->A1, double_integrator_QP_solver_CD_dzcc00, double_integrator_QP_solver_CD_ccrhsp00, double_integrator_QP_solver_CD_sp00, double_integrator_QP_solver_CD_lp00, double_integrator_QP_solver_CD_dlp_cc00);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_CD_ccrhsl01, double_integrator_QP_solver_CD_slb01, double_integrator_QP_solver_CD_llbbyslb01, double_integrator_QP_solver_CD_dzcc01, double_integrator_QP_solver_CD_lbIdx01, double_integrator_QP_solver_CD_dllbcc01);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_CD_ccrhsub01, double_integrator_QP_solver_CD_sub01, double_integrator_QP_solver_CD_lubbysub01, double_integrator_QP_solver_CD_dzcc01, double_integrator_QP_solver_CD_ubIdx01, double_integrator_QP_solver_CD_dlubcc01);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB5_34_23(params->A2, double_integrator_QP_solver_CD_dzcc01, double_integrator_QP_solver_CD_ccrhsp01, double_integrator_QP_solver_CD_sp01, double_integrator_QP_solver_CD_lp01, double_integrator_QP_solver_CD_dlp_cc01);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_CD_ccrhsl02, double_integrator_QP_solver_CD_slb02, double_integrator_QP_solver_CD_llbbyslb02, double_integrator_QP_solver_CD_dzcc02, double_integrator_QP_solver_CD_lbIdx02, double_integrator_QP_solver_CD_dllbcc02);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_CD_ccrhsub02, double_integrator_QP_solver_CD_sub02, double_integrator_QP_solver_CD_lubbysub02, double_integrator_QP_solver_CD_dzcc02, double_integrator_QP_solver_CD_ubIdx02, double_integrator_QP_solver_CD_dlubcc02);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB5_34_23(params->A3, double_integrator_QP_solver_CD_dzcc02, double_integrator_QP_solver_CD_ccrhsp02, double_integrator_QP_solver_CD_sp02, double_integrator_QP_solver_CD_lp02, double_integrator_QP_solver_CD_dlp_cc02);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_CD_ccrhsl03, double_integrator_QP_solver_CD_slb03, double_integrator_QP_solver_CD_llbbyslb03, double_integrator_QP_solver_CD_dzcc03, double_integrator_QP_solver_CD_lbIdx03, double_integrator_QP_solver_CD_dllbcc03);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_CD_ccrhsub03, double_integrator_QP_solver_CD_sub03, double_integrator_QP_solver_CD_lubbysub03, double_integrator_QP_solver_CD_dzcc03, double_integrator_QP_solver_CD_ubIdx03, double_integrator_QP_solver_CD_dlubcc03);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB5_34_23(params->A4, double_integrator_QP_solver_CD_dzcc03, double_integrator_QP_solver_CD_ccrhsp03, double_integrator_QP_solver_CD_sp03, double_integrator_QP_solver_CD_lp03, double_integrator_QP_solver_CD_dlp_cc03);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_CD_ccrhsl04, double_integrator_QP_solver_CD_slb04, double_integrator_QP_solver_CD_llbbyslb04, double_integrator_QP_solver_CD_dzcc04, double_integrator_QP_solver_CD_lbIdx04, double_integrator_QP_solver_CD_dllbcc04);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_CD_ccrhsub04, double_integrator_QP_solver_CD_sub04, double_integrator_QP_solver_CD_lubbysub04, double_integrator_QP_solver_CD_dzcc04, double_integrator_QP_solver_CD_ubIdx04, double_integrator_QP_solver_CD_dlubcc04);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB5_34_23(params->A5, double_integrator_QP_solver_CD_dzcc04, double_integrator_QP_solver_CD_ccrhsp04, double_integrator_QP_solver_CD_sp04, double_integrator_QP_solver_CD_lp04, double_integrator_QP_solver_CD_dlp_cc04);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_CD_ccrhsl05, double_integrator_QP_solver_CD_slb05, double_integrator_QP_solver_CD_llbbyslb05, double_integrator_QP_solver_CD_dzcc05, double_integrator_QP_solver_CD_lbIdx05, double_integrator_QP_solver_CD_dllbcc05);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_CD_ccrhsub05, double_integrator_QP_solver_CD_sub05, double_integrator_QP_solver_CD_lubbysub05, double_integrator_QP_solver_CD_dzcc05, double_integrator_QP_solver_CD_ubIdx05, double_integrator_QP_solver_CD_dlubcc05);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB5_34_23(params->A6, double_integrator_QP_solver_CD_dzcc05, double_integrator_QP_solver_CD_ccrhsp05, double_integrator_QP_solver_CD_sp05, double_integrator_QP_solver_CD_lp05, double_integrator_QP_solver_CD_dlp_cc05);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_CD_ccrhsl06, double_integrator_QP_solver_CD_slb06, double_integrator_QP_solver_CD_llbbyslb06, double_integrator_QP_solver_CD_dzcc06, double_integrator_QP_solver_CD_lbIdx06, double_integrator_QP_solver_CD_dllbcc06);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_CD_ccrhsub06, double_integrator_QP_solver_CD_sub06, double_integrator_QP_solver_CD_lubbysub06, double_integrator_QP_solver_CD_dzcc06, double_integrator_QP_solver_CD_ubIdx06, double_integrator_QP_solver_CD_dlubcc06);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB5_34_23(params->A7, double_integrator_QP_solver_CD_dzcc06, double_integrator_QP_solver_CD_ccrhsp06, double_integrator_QP_solver_CD_sp06, double_integrator_QP_solver_CD_lp06, double_integrator_QP_solver_CD_dlp_cc06);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_CD_ccrhsl07, double_integrator_QP_solver_CD_slb07, double_integrator_QP_solver_CD_llbbyslb07, double_integrator_QP_solver_CD_dzcc07, double_integrator_QP_solver_CD_lbIdx07, double_integrator_QP_solver_CD_dllbcc07);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_CD_ccrhsub07, double_integrator_QP_solver_CD_sub07, double_integrator_QP_solver_CD_lubbysub07, double_integrator_QP_solver_CD_dzcc07, double_integrator_QP_solver_CD_ubIdx07, double_integrator_QP_solver_CD_dlubcc07);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB5_34_23(params->A8, double_integrator_QP_solver_CD_dzcc07, double_integrator_QP_solver_CD_ccrhsp07, double_integrator_QP_solver_CD_sp07, double_integrator_QP_solver_CD_lp07, double_integrator_QP_solver_CD_dlp_cc07);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_CD_ccrhsl08, double_integrator_QP_solver_CD_slb08, double_integrator_QP_solver_CD_llbbyslb08, double_integrator_QP_solver_CD_dzcc08, double_integrator_QP_solver_CD_lbIdx08, double_integrator_QP_solver_CD_dllbcc08);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_CD_ccrhsub08, double_integrator_QP_solver_CD_sub08, double_integrator_QP_solver_CD_lubbysub08, double_integrator_QP_solver_CD_dzcc08, double_integrator_QP_solver_CD_ubIdx08, double_integrator_QP_solver_CD_dlubcc08);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB5_34_23(params->A9, double_integrator_QP_solver_CD_dzcc08, double_integrator_QP_solver_CD_ccrhsp08, double_integrator_QP_solver_CD_sp08, double_integrator_QP_solver_CD_lp08, double_integrator_QP_solver_CD_dlp_cc08);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_CD_ccrhsl09, double_integrator_QP_solver_CD_slb09, double_integrator_QP_solver_CD_llbbyslb09, double_integrator_QP_solver_CD_dzcc09, double_integrator_QP_solver_CD_lbIdx09, double_integrator_QP_solver_CD_dllbcc09);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_CD_ccrhsub09, double_integrator_QP_solver_CD_sub09, double_integrator_QP_solver_CD_lubbysub09, double_integrator_QP_solver_CD_dzcc09, double_integrator_QP_solver_CD_ubIdx09, double_integrator_QP_solver_CD_dlubcc09);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB5_34_23(params->A10, double_integrator_QP_solver_CD_dzcc09, double_integrator_QP_solver_CD_ccrhsp09, double_integrator_QP_solver_CD_sp09, double_integrator_QP_solver_CD_lp09, double_integrator_QP_solver_CD_dlp_cc09);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_CD_ccrhsl10, double_integrator_QP_solver_CD_slb10, double_integrator_QP_solver_CD_llbbyslb10, double_integrator_QP_solver_CD_dzcc10, double_integrator_QP_solver_CD_lbIdx10, double_integrator_QP_solver_CD_dllbcc10);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_CD_ccrhsub10, double_integrator_QP_solver_CD_sub10, double_integrator_QP_solver_CD_lubbysub10, double_integrator_QP_solver_CD_dzcc10, double_integrator_QP_solver_CD_ubIdx10, double_integrator_QP_solver_CD_dlubcc10);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB5_34_21(params->A11, double_integrator_QP_solver_CD_dzcc10, double_integrator_QP_solver_CD_ccrhsp10, double_integrator_QP_solver_CD_sp10, double_integrator_QP_solver_CD_lp10, double_integrator_QP_solver_CD_dlp_cc10);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_CD_ccrhsl11, double_integrator_QP_solver_CD_slb11, double_integrator_QP_solver_CD_llbbyslb11, double_integrator_QP_solver_CD_dzcc11, double_integrator_QP_solver_CD_lbIdx11, double_integrator_QP_solver_CD_dllbcc11);
double_integrator_QP_solver_CD_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_CD_ccrhsub11, double_integrator_QP_solver_CD_sub11, double_integrator_QP_solver_CD_lubbysub11, double_integrator_QP_solver_CD_dzcc11, double_integrator_QP_solver_CD_ubIdx11, double_integrator_QP_solver_CD_dlubcc11);
double_integrator_QP_solver_CD_LA_DENSE_MVMSUB5_14_7(params->A12, double_integrator_QP_solver_CD_dzcc11, double_integrator_QP_solver_CD_ccrhsp11, double_integrator_QP_solver_CD_sp11, double_integrator_QP_solver_CD_lp11, double_integrator_QP_solver_CD_dlp_cc11);
double_integrator_QP_solver_CD_LA_VSUB7_529(double_integrator_QP_solver_CD_l, double_integrator_QP_solver_CD_ccrhs, double_integrator_QP_solver_CD_s, double_integrator_QP_solver_CD_dl_cc, double_integrator_QP_solver_CD_ds_cc);
double_integrator_QP_solver_CD_LA_VADD_258(double_integrator_QP_solver_CD_dz_cc, double_integrator_QP_solver_CD_dz_aff);
double_integrator_QP_solver_CD_LA_VADD_74(double_integrator_QP_solver_CD_dv_cc, double_integrator_QP_solver_CD_dv_aff);
double_integrator_QP_solver_CD_LA_VADD_529(double_integrator_QP_solver_CD_dl_cc, double_integrator_QP_solver_CD_dl_aff);
double_integrator_QP_solver_CD_LA_VADD_529(double_integrator_QP_solver_CD_ds_cc, double_integrator_QP_solver_CD_ds_aff);
info->lsit_cc = double_integrator_QP_solver_CD_LINESEARCH_BACKTRACKING_COMBINED(double_integrator_QP_solver_CD_z, double_integrator_QP_solver_CD_v, double_integrator_QP_solver_CD_l, double_integrator_QP_solver_CD_s, double_integrator_QP_solver_CD_dz_cc, double_integrator_QP_solver_CD_dv_cc, double_integrator_QP_solver_CD_dl_cc, double_integrator_QP_solver_CD_ds_cc, &info->step_cc, &info->mu);
if( info->lsit_cc == double_integrator_QP_solver_CD_NOPROGRESS ){
PRINTTEXT("Line search could not proceed at iteration %d, exiting.\n",info->it+1);
exitcode = double_integrator_QP_solver_CD_NOPROGRESS; break;
}
info->it++;
}
output->z1[0] = double_integrator_QP_solver_CD_z00[0];
output->z1[1] = double_integrator_QP_solver_CD_z00[1];
output->z1[2] = double_integrator_QP_solver_CD_z00[2];
output->z1[3] = double_integrator_QP_solver_CD_z00[3];
output->z1[4] = double_integrator_QP_solver_CD_z00[4];
output->z1[5] = double_integrator_QP_solver_CD_z00[5];
output->z1[6] = double_integrator_QP_solver_CD_z00[12];
output->z2[0] = double_integrator_QP_solver_CD_z01[0];
output->z2[1] = double_integrator_QP_solver_CD_z01[1];
output->z2[2] = double_integrator_QP_solver_CD_z01[2];
output->z2[3] = double_integrator_QP_solver_CD_z01[3];
output->z2[4] = double_integrator_QP_solver_CD_z01[4];
output->z2[5] = double_integrator_QP_solver_CD_z01[5];
output->z2[6] = double_integrator_QP_solver_CD_z01[12];
output->z3[0] = double_integrator_QP_solver_CD_z02[0];
output->z3[1] = double_integrator_QP_solver_CD_z02[1];
output->z3[2] = double_integrator_QP_solver_CD_z02[2];
output->z3[3] = double_integrator_QP_solver_CD_z02[3];
output->z3[4] = double_integrator_QP_solver_CD_z02[4];
output->z3[5] = double_integrator_QP_solver_CD_z02[5];
output->z3[6] = double_integrator_QP_solver_CD_z02[12];
output->z4[0] = double_integrator_QP_solver_CD_z03[0];
output->z4[1] = double_integrator_QP_solver_CD_z03[1];
output->z4[2] = double_integrator_QP_solver_CD_z03[2];
output->z4[3] = double_integrator_QP_solver_CD_z03[3];
output->z4[4] = double_integrator_QP_solver_CD_z03[4];
output->z4[5] = double_integrator_QP_solver_CD_z03[5];
output->z4[6] = double_integrator_QP_solver_CD_z03[12];
output->z5[0] = double_integrator_QP_solver_CD_z04[0];
output->z5[1] = double_integrator_QP_solver_CD_z04[1];
output->z5[2] = double_integrator_QP_solver_CD_z04[2];
output->z5[3] = double_integrator_QP_solver_CD_z04[3];
output->z5[4] = double_integrator_QP_solver_CD_z04[4];
output->z5[5] = double_integrator_QP_solver_CD_z04[5];
output->z5[6] = double_integrator_QP_solver_CD_z04[12];
output->z6[0] = double_integrator_QP_solver_CD_z05[0];
output->z6[1] = double_integrator_QP_solver_CD_z05[1];
output->z6[2] = double_integrator_QP_solver_CD_z05[2];
output->z6[3] = double_integrator_QP_solver_CD_z05[3];
output->z6[4] = double_integrator_QP_solver_CD_z05[4];
output->z6[5] = double_integrator_QP_solver_CD_z05[5];
output->z6[6] = double_integrator_QP_solver_CD_z05[12];
output->z7[0] = double_integrator_QP_solver_CD_z06[0];
output->z7[1] = double_integrator_QP_solver_CD_z06[1];
output->z7[2] = double_integrator_QP_solver_CD_z06[2];
output->z7[3] = double_integrator_QP_solver_CD_z06[3];
output->z7[4] = double_integrator_QP_solver_CD_z06[4];
output->z7[5] = double_integrator_QP_solver_CD_z06[5];
output->z7[6] = double_integrator_QP_solver_CD_z06[12];
output->z8[0] = double_integrator_QP_solver_CD_z07[0];
output->z8[1] = double_integrator_QP_solver_CD_z07[1];
output->z8[2] = double_integrator_QP_solver_CD_z07[2];
output->z8[3] = double_integrator_QP_solver_CD_z07[3];
output->z8[4] = double_integrator_QP_solver_CD_z07[4];
output->z8[5] = double_integrator_QP_solver_CD_z07[5];
output->z8[6] = double_integrator_QP_solver_CD_z07[12];
output->z9[0] = double_integrator_QP_solver_CD_z08[0];
output->z9[1] = double_integrator_QP_solver_CD_z08[1];
output->z9[2] = double_integrator_QP_solver_CD_z08[2];
output->z9[3] = double_integrator_QP_solver_CD_z08[3];
output->z9[4] = double_integrator_QP_solver_CD_z08[4];
output->z9[5] = double_integrator_QP_solver_CD_z08[5];
output->z9[6] = double_integrator_QP_solver_CD_z08[12];
output->z10[0] = double_integrator_QP_solver_CD_z09[0];
output->z10[1] = double_integrator_QP_solver_CD_z09[1];
output->z10[2] = double_integrator_QP_solver_CD_z09[2];
output->z10[3] = double_integrator_QP_solver_CD_z09[3];
output->z10[4] = double_integrator_QP_solver_CD_z09[4];
output->z10[5] = double_integrator_QP_solver_CD_z09[5];
output->z10[6] = double_integrator_QP_solver_CD_z09[12];
output->z11[0] = double_integrator_QP_solver_CD_z10[0];
output->z11[1] = double_integrator_QP_solver_CD_z10[1];
output->z11[2] = double_integrator_QP_solver_CD_z10[2];
output->z11[3] = double_integrator_QP_solver_CD_z10[3];
output->z11[4] = double_integrator_QP_solver_CD_z10[4];
output->z11[5] = double_integrator_QP_solver_CD_z10[5];
output->z11[6] = double_integrator_QP_solver_CD_z10[10];
output->z12[0] = double_integrator_QP_solver_CD_z11[0];
output->z12[1] = double_integrator_QP_solver_CD_z11[1];
output->z12[2] = double_integrator_QP_solver_CD_z11[2];
output->z12[3] = double_integrator_QP_solver_CD_z11[3];

#if double_integrator_QP_solver_CD_SET_TIMING == 1
info->solvetime = double_integrator_QP_solver_CD_toc(&solvertimer);
#if double_integrator_QP_solver_CD_SET_PRINTLEVEL > 0 && double_integrator_QP_solver_CD_SET_TIMING == 1
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
