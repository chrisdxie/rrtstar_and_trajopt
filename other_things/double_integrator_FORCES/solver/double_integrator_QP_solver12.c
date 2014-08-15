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
 * Initializes a vector of length 201 with a value.
 */
void double_integrator_QP_solver_LA_INITIALIZEVECTOR_201(double_integrator_QP_solver_FLOAT* vec, double_integrator_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<201; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 74 with a value.
 */
void double_integrator_QP_solver_LA_INITIALIZEVECTOR_74(double_integrator_QP_solver_FLOAT* vec, double_integrator_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<74; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 415 with a value.
 */
void double_integrator_QP_solver_LA_INITIALIZEVECTOR_415(double_integrator_QP_solver_FLOAT* vec, double_integrator_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<415; i++ )
	{
		vec[i] = value;
	}
}


/* 
 * Calculates a dot product and adds it to a variable: z += x'*y; 
 * This function is for vectors of length 415.
 */
void double_integrator_QP_solver_LA_DOTACC_415(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<415; i++ ){
		*z += x[i]*y[i];
	}
}


/*
 * Calculates the gradient and the value for a quadratic function 0.5*z'*H*z + f'*z
 *
 * INPUTS:     H  - Symmetric Hessian, diag matrix of size [18 x 18]
 *             f  - column vector of size 18
 *             z  - column vector of size 18
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 18
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void double_integrator_QP_solver_LA_DIAG_QUADFCN_18(double_integrator_QP_solver_FLOAT* H, double_integrator_QP_solver_FLOAT* f, double_integrator_QP_solver_FLOAT* z, double_integrator_QP_solver_FLOAT* grad, double_integrator_QP_solver_FLOAT* value)
{
	int i;
	double_integrator_QP_solver_FLOAT hz;	
	for( i=0; i<18; i++){
		hz = H[i]*z[i];
		grad[i] = hz + f[i];
		*value += 0.5*hz*z[i] + f[i]*z[i];
	}
}


/*
 * Calculates the gradient and the value for a quadratic function 0.5*z'*H*z + f'*z
 *
 * INPUTS:     H  - Symmetric Hessian, diag matrix of size [16 x 16]
 *             f  - column vector of size 16
 *             z  - column vector of size 16
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 16
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void double_integrator_QP_solver_LA_DIAG_QUADFCN_16(double_integrator_QP_solver_FLOAT* H, double_integrator_QP_solver_FLOAT* f, double_integrator_QP_solver_FLOAT* z, double_integrator_QP_solver_FLOAT* grad, double_integrator_QP_solver_FLOAT* value)
{
	int i;
	double_integrator_QP_solver_FLOAT hz;	
	for( i=0; i<16; i++){
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_18_18(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;
	double_integrator_QP_solver_FLOAT AxBu[7];
	double_integrator_QP_solver_FLOAT norm = *y;
	double_integrator_QP_solver_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<7; i++ ){
		AxBu[i] = A[k++]*x[0] + B[m++]*u[0];
	}	
	for( j=1; j<18; j++ ){		
		for( i=0; i<7; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<18; n++ ){
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_18_16(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;
	double_integrator_QP_solver_FLOAT AxBu[7];
	double_integrator_QP_solver_FLOAT norm = *y;
	double_integrator_QP_solver_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<7; i++ ){
		AxBu[i] = A[k++]*x[0] + B[m++]*u[0];
	}	
	for( j=1; j<18; j++ ){		
		for( i=0; i<7; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<16; n++ ){
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
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_4_16_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	double_integrator_QP_solver_FLOAT AxBu[4];
	double_integrator_QP_solver_FLOAT norm = *y;
	double_integrator_QP_solver_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<4; i++ ){
		AxBu[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<16; j++ ){		
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
 * Matrix vector multiplication y = M'*x where M is of size [7 x 18]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_LA_DENSE_MTVM_7_18(double_integrator_QP_solver_FLOAT *M, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<18; i++ ){
		y[i] = 0;
		for( j=0; j<7; j++ ){
			y[i] += M[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [7 x 18]
 * and B is of size [7 x 18]
 * and stored in column major format. Note the transposes of A and B!
 */
void double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	int n;
	int m = 0;
	for( i=0; i<18; i++ ){
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
 * where A is of size [4 x 16]
 * and B is of size [7 x 16]
 * and stored in column major format. Note the transposes of A and B!
 */
void double_integrator_QP_solver_LA_DENSE_MTVM2_4_16_7(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	int n;
	int m = 0;
	for( i=0; i<16; i++ ){
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
 * Matrix vector multiplication y = M'*x where M is of size [4 x 5]
 * and stored in diagzero format. Note the transpose of M!
 */
void double_integrator_QP_solver_LA_DIAGZERO_MTVM_4_5(double_integrator_QP_solver_FLOAT *M, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
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
 * for vectors of length 7. Output z is of course scalar.
 */
void double_integrator_QP_solver_LA_VSUBADD3_7(double_integrator_QP_solver_FLOAT* t, double_integrator_QP_solver_FLOAT* u, int* uidx, double_integrator_QP_solver_FLOAT* v, double_integrator_QP_solver_FLOAT* w, double_integrator_QP_solver_FLOAT* y, double_integrator_QP_solver_FLOAT* z, double_integrator_QP_solver_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_FLOAT norm = *r;
	double_integrator_QP_solver_FLOAT vx = 0;
	double_integrator_QP_solver_FLOAT x;
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
void double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_FLOAT* t, int* tidx, double_integrator_QP_solver_FLOAT* u, double_integrator_QP_solver_FLOAT* v, double_integrator_QP_solver_FLOAT* w, double_integrator_QP_solver_FLOAT* y, double_integrator_QP_solver_FLOAT* z, double_integrator_QP_solver_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_FLOAT norm = *r;
	double_integrator_QP_solver_FLOAT vx = 0;
	double_integrator_QP_solver_FLOAT x;
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
void double_integrator_QP_solver_LA_MVSUBADD_24_18(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	double_integrator_QP_solver_FLOAT Ax[24];
	double_integrator_QP_solver_FLOAT Axlessb;
	double_integrator_QP_solver_FLOAT norm = *y;
	double_integrator_QP_solver_FLOAT lAxlessb = 0;

	/* do A*x first */
	for( i=0; i<24; i++ ){
		Ax[i] = A[k++]*x[0];				
	}	
	for( j=1; j<18; j++ ){		
		for( i=0; i<24; i++ ){
			Ax[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<24; i++ ){
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
void double_integrator_QP_solver_LA_VSUBADD3_6(double_integrator_QP_solver_FLOAT* t, double_integrator_QP_solver_FLOAT* u, int* uidx, double_integrator_QP_solver_FLOAT* v, double_integrator_QP_solver_FLOAT* w, double_integrator_QP_solver_FLOAT* y, double_integrator_QP_solver_FLOAT* z, double_integrator_QP_solver_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_FLOAT norm = *r;
	double_integrator_QP_solver_FLOAT vx = 0;
	double_integrator_QP_solver_FLOAT x;
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
void double_integrator_QP_solver_LA_MVSUBADD_24_16(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	double_integrator_QP_solver_FLOAT Ax[24];
	double_integrator_QP_solver_FLOAT Axlessb;
	double_integrator_QP_solver_FLOAT norm = *y;
	double_integrator_QP_solver_FLOAT lAxlessb = 0;

	/* do A*x first */
	for( i=0; i<24; i++ ){
		Ax[i] = A[k++]*x[0];				
	}	
	for( j=1; j<16; j++ ){		
		for( i=0; i<24; i++ ){
			Ax[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<24; i++ ){
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
void double_integrator_QP_solver_LA_VSUBADD3_4(double_integrator_QP_solver_FLOAT* t, double_integrator_QP_solver_FLOAT* u, int* uidx, double_integrator_QP_solver_FLOAT* v, double_integrator_QP_solver_FLOAT* w, double_integrator_QP_solver_FLOAT* y, double_integrator_QP_solver_FLOAT* z, double_integrator_QP_solver_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_FLOAT norm = *r;
	double_integrator_QP_solver_FLOAT vx = 0;
	double_integrator_QP_solver_FLOAT x;
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
void double_integrator_QP_solver_LA_VSUBADD2_4(double_integrator_QP_solver_FLOAT* t, int* tidx, double_integrator_QP_solver_FLOAT* u, double_integrator_QP_solver_FLOAT* v, double_integrator_QP_solver_FLOAT* w, double_integrator_QP_solver_FLOAT* y, double_integrator_QP_solver_FLOAT* z, double_integrator_QP_solver_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_FLOAT norm = *r;
	double_integrator_QP_solver_FLOAT vx = 0;
	double_integrator_QP_solver_FLOAT x;
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
void double_integrator_QP_solver_LA_MVSUBADD_10_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	double_integrator_QP_solver_FLOAT Ax[10];
	double_integrator_QP_solver_FLOAT Axlessb;
	double_integrator_QP_solver_FLOAT norm = *y;
	double_integrator_QP_solver_FLOAT lAxlessb = 0;

	/* do A*x first */
	for( i=0; i<10; i++ ){
		Ax[i] = A[k++]*x[0];				
	}	
	for( j=1; j<5; j++ ){		
		for( i=0; i<10; i++ ){
			Ax[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<10; i++ ){
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
 * Special function for box constraints of length 18
 * Returns also L/S, a value that is often used elsewhere.
 */
void double_integrator_QP_solver_LA_INEQ_B_GRAD_18_7_6(double_integrator_QP_solver_FLOAT *lu, double_integrator_QP_solver_FLOAT *su, double_integrator_QP_solver_FLOAT *ru, double_integrator_QP_solver_FLOAT *ll, double_integrator_QP_solver_FLOAT *sl, double_integrator_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_FLOAT *grad, double_integrator_QP_solver_FLOAT *lubysu, double_integrator_QP_solver_FLOAT *llbysl)
{
	int i;
	for( i=0; i<18; i++ ){
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
void double_integrator_QP_solver_LA_INEQ_P_24_18(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *lp, double_integrator_QP_solver_FLOAT *sp, double_integrator_QP_solver_FLOAT *rip, double_integrator_QP_solver_FLOAT *grad, double_integrator_QP_solver_FLOAT *lpbysp)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_FLOAT lsr[24];
	
	/* do (L/S)*ri first */
	for( j=0; j<24; j++ ){
		lpbysp[j] = lp[j] / sp[j];
		lsr[j] = lpbysp[j]*rip[j];
	}

	for( i=0; i<18; i++ ){		
		for( j=0; j<24; j++ ){
			grad[i] += A[k++]*lsr[j];
		}
	}
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 18
 * Returns also L/S, a value that is often used elsewhere.
 */
void double_integrator_QP_solver_LA_INEQ_B_GRAD_18_6_6(double_integrator_QP_solver_FLOAT *lu, double_integrator_QP_solver_FLOAT *su, double_integrator_QP_solver_FLOAT *ru, double_integrator_QP_solver_FLOAT *ll, double_integrator_QP_solver_FLOAT *sl, double_integrator_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_FLOAT *grad, double_integrator_QP_solver_FLOAT *lubysu, double_integrator_QP_solver_FLOAT *llbysl)
{
	int i;
	for( i=0; i<18; i++ ){
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
 * Special function for box constraints of length 16
 * Returns also L/S, a value that is often used elsewhere.
 */
void double_integrator_QP_solver_LA_INEQ_B_GRAD_16_6_6(double_integrator_QP_solver_FLOAT *lu, double_integrator_QP_solver_FLOAT *su, double_integrator_QP_solver_FLOAT *ru, double_integrator_QP_solver_FLOAT *ll, double_integrator_QP_solver_FLOAT *sl, double_integrator_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_FLOAT *grad, double_integrator_QP_solver_FLOAT *lubysu, double_integrator_QP_solver_FLOAT *llbysl)
{
	int i;
	for( i=0; i<16; i++ ){
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
void double_integrator_QP_solver_LA_INEQ_P_24_16(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *lp, double_integrator_QP_solver_FLOAT *sp, double_integrator_QP_solver_FLOAT *rip, double_integrator_QP_solver_FLOAT *grad, double_integrator_QP_solver_FLOAT *lpbysp)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_FLOAT lsr[24];
	
	/* do (L/S)*ri first */
	for( j=0; j<24; j++ ){
		lpbysp[j] = lp[j] / sp[j];
		lsr[j] = lpbysp[j]*rip[j];
	}

	for( i=0; i<16; i++ ){		
		for( j=0; j<24; j++ ){
			grad[i] += A[k++]*lsr[j];
		}
	}
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 5
 * Returns also L/S, a value that is often used elsewhere.
 */
void double_integrator_QP_solver_LA_INEQ_B_GRAD_5_4_4(double_integrator_QP_solver_FLOAT *lu, double_integrator_QP_solver_FLOAT *su, double_integrator_QP_solver_FLOAT *ru, double_integrator_QP_solver_FLOAT *ll, double_integrator_QP_solver_FLOAT *sl, double_integrator_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_FLOAT *grad, double_integrator_QP_solver_FLOAT *lubysu, double_integrator_QP_solver_FLOAT *llbysl)
{
	int i;
	for( i=0; i<5; i++ ){
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
void double_integrator_QP_solver_LA_INEQ_P_10_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *lp, double_integrator_QP_solver_FLOAT *sp, double_integrator_QP_solver_FLOAT *rip, double_integrator_QP_solver_FLOAT *grad, double_integrator_QP_solver_FLOAT *lpbysp)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_FLOAT lsr[10];
	
	/* do (L/S)*ri first */
	for( j=0; j<10; j++ ){
		lpbysp[j] = lp[j] / sp[j];
		lsr[j] = lpbysp[j]*rip[j];
	}

	for( i=0; i<5; i++ ){		
		for( j=0; j<10; j++ ){
			grad[i] += A[k++]*lsr[j];
		}
	}
}


/*
 * Addition of three vectors  z = u + w + v
 * of length 201.
 */
void double_integrator_QP_solver_LA_VVADD3_201(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *w, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<201; i++ ){
		z[i] = u[i] + v[i] + w[i];
	}
}


/*
 * Special function to compute the Dense positive definite 
 * augmented Hessian for block size 18.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_18_7_6(double_integrator_QP_solver_FLOAT *H, double_integrator_QP_solver_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_FLOAT *Phi)
{
	int i;
	int j;
	int k = 0;
	
	/* copy diagonal of H into PHI and set lower part of PHI = 0*/
	for( i=0; i<18; i++ ){
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
 * A is stored in column major format and is of size [24 x 18]
 * Phi is of size [18 x 18].
 */
void double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_18(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *d, double_integrator_QP_solver_FLOAT *X)
{    
    int i,j,k,ii,di;
    double_integrator_QP_solver_FLOAT x;
    
    di = 0; ii = 0;
    for( i=0; i<18; i++ ){        
        for( j=0; j<=i; j++ ){
            x = 0;
            for( k=0; k<24; k++ ){
                x += A[i*24+k]*A[j*24+k]*d[k];
            }
            X[ii+j] += x;
        }
        ii += ++di;
    }
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 18.
 */
void double_integrator_QP_solver_LA_DENSE_CHOL2_18(double_integrator_QP_solver_FLOAT *A)
{
    int i, j, k, di, dj;
	 int ii, jj;
    double_integrator_QP_solver_FLOAT l;
    double_integrator_QP_solver_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<18; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += A[ii+k]*A[ii+k];
        }        
        
        Mii = A[ii+i] - l;
        
#if double_integrator_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
        for( j=i+1; j<18; j++ ){
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
 * where A is to be computed and is of size [7 x 18],
 * B is given and of size [7 x 18], L is a lower tri-
 * angular matrix of size 18 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    double_integrator_QP_solver_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<18; j++ ){        
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
 * The dimensions involved are 18.
 */
void double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<18; i++ ){
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
 * augmented Hessian for block size 18.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_18_6_6(double_integrator_QP_solver_FLOAT *H, double_integrator_QP_solver_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_FLOAT *Phi)
{
	int i;
	int j;
	int k = 0;
	
	/* copy diagonal of H into PHI and set lower part of PHI = 0*/
	for( i=0; i<18; i++ ){
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
 *	size(A) = [7 x 18]
 *  size(B) = [7 x 18]
 * 
 * and all matrices are stored in column major format.
 *
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE.  
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMTM_7_18_7(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *C)
{
    int i, j, k;
    double_integrator_QP_solver_FLOAT temp;
    
    for( i=0; i<7; i++ ){        
        for( j=0; j<7; j++ ){
            temp = 0; 
            for( k=0; k<18; k++ ){
                temp += A[k*7+i]*B[k*7+j];
            }						
            C[j*7+i] = temp;
        }
    }
}


/*
 * Special function to compute the Dense positive definite 
 * augmented Hessian for block size 16.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_16_6_6(double_integrator_QP_solver_FLOAT *H, double_integrator_QP_solver_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_FLOAT *Phi)
{
	int i;
	int j;
	int k = 0;
	
	/* copy diagonal of H into PHI and set lower part of PHI = 0*/
	for( i=0; i<16; i++ ){
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
 * A is stored in column major format and is of size [24 x 16]
 * Phi is of size [16 x 16].
 */
void double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_16(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *d, double_integrator_QP_solver_FLOAT *X)
{    
    int i,j,k,ii,di;
    double_integrator_QP_solver_FLOAT x;
    
    di = 0; ii = 0;
    for( i=0; i<16; i++ ){        
        for( j=0; j<=i; j++ ){
            x = 0;
            for( k=0; k<24; k++ ){
                x += A[i*24+k]*A[j*24+k]*d[k];
            }
            X[ii+j] += x;
        }
        ii += ++di;
    }
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 16.
 */
void double_integrator_QP_solver_LA_DENSE_CHOL2_16(double_integrator_QP_solver_FLOAT *A)
{
    int i, j, k, di, dj;
	 int ii, jj;
    double_integrator_QP_solver_FLOAT l;
    double_integrator_QP_solver_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<16; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += A[ii+k]*A[ii+k];
        }        
        
        Mii = A[ii+i] - l;
        
#if double_integrator_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
        for( j=i+1; j<16; j++ ){
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
 * where A is to be computed and is of size [4 x 16],
 * B is given and of size [4 x 16], L is a lower tri-
 * angular matrix of size 16 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_4_16(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    double_integrator_QP_solver_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<16; j++ ){        
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
 * where A is to be computed and is of size [7 x 16],
 * B is given and of size [7 x 16], L is a lower tri-
 * angular matrix of size 16 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_16(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    double_integrator_QP_solver_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<16; j++ ){        
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
 *	size(A) = [7 x 16]
 *  size(B) = [4 x 16]
 * 
 * and all matrices are stored in column major format.
 *
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE.  
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMTM_7_16_4(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *C)
{
    int i, j, k;
    double_integrator_QP_solver_FLOAT temp;
    
    for( i=0; i<7; i++ ){        
        for( j=0; j<4; j++ ){
            temp = 0; 
            for( k=0; k<16; k++ ){
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
 * The dimensions involved are 16.
 */
void double_integrator_QP_solver_LA_DENSE_FORWARDSUB_16(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<16; i++ ){
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
void double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_5_4_4(double_integrator_QP_solver_FLOAT *H, double_integrator_QP_solver_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_FLOAT *Phi)
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
 * A is stored in column major format and is of size [10 x 5]
 * Phi is of size [5 x 5].
 */
void double_integrator_QP_solver_LA_DENSE_ADDMTDM_10_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *d, double_integrator_QP_solver_FLOAT *X)
{    
    int i,j,k,ii,di;
    double_integrator_QP_solver_FLOAT x;
    
    di = 0; ii = 0;
    for( i=0; i<5; i++ ){        
        for( j=0; j<=i; j++ ){
            x = 0;
            for( k=0; k<10; k++ ){
                x += A[i*10+k]*A[j*10+k]*d[k];
            }
            X[ii+j] += x;
        }
        ii += ++di;
    }
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 5.
 */
void double_integrator_QP_solver_LA_DENSE_CHOL2_5(double_integrator_QP_solver_FLOAT *A)
{
    int i, j, k, di, dj;
	 int ii, jj;
    double_integrator_QP_solver_FLOAT l;
    double_integrator_QP_solver_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<5; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += A[ii+k]*A[ii+k];
        }        
        
        Mii = A[ii+i] - l;
        
#if double_integrator_QP_solver_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
        for( j=i+1; j<5; j++ ){
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
 * where A is to be computed and is of size [4 x 5],
 * B is given and of size [4 x 5] stored in 
 * diagzero storage format, L is a lower tri-
 * angular matrix of size 5 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_4_5(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    double_integrator_QP_solver_FLOAT a;
	
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
	for( j=4; j<5; j++ ){        
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
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [7 x 18] in column
 * storage format, and B is of size [7 x 18] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMT2_7_18_18(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<7; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<18; k++ ){
                ltemp += A[k*7+i]*A[k*7+j];
            }			
			for( k=0; k<18; k++ ){
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_18_18(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<7; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<18; j++ ){		
		for( i=0; i<7; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<18; n++ ){
		for( i=0; i<7; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [7 x 18] in column
 * storage format, and B is of size [7 x 16] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMT2_7_18_16(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<7; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<18; k++ ){
                ltemp += A[k*7+i]*A[k*7+j];
            }			
			for( k=0; k<16; k++ ){
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_18_16(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<7; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<18; j++ ){		
		for( i=0; i<7; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<16; n++ ){
		for( i=0; i<7; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [4 x 16] in column
 * storage format, and B is of size [4 x 5] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMT2_4_16_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<4; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<16; k++ ){
                ltemp += A[k*4+i]*A[k*4+j];
            }			
			for( k=0; k<5; k++ ){
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB2_4_16_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<4; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<16; j++ ){		
		for( i=0; i<4; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<5; n++ ){
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
void double_integrator_QP_solver_LA_DENSE_CHOL_7(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    double_integrator_QP_solver_FLOAT l;
    double_integrator_QP_solver_FLOAT Mii;

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
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 7.
 */
void double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_FLOAT yel;
            
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
 * Forward substitution for the matrix equation A*L' = B'
 * where A is to be computed and is of size [7 x 7],
 * B is given and of size [7 x 7], L is a lower tri-
 * angular matrix of size 7 stored in lower triangular 
 * storage format. Note the transpose of L AND B!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j,k,ii,di;
    double_integrator_QP_solver_FLOAT a;
    
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
void double_integrator_QP_solver_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
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
void double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_4_7(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j,k,ii,di;
    double_integrator_QP_solver_FLOAT a;
    
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
void double_integrator_QP_solver_LA_DENSE_MMTSUB_4_7(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
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
void double_integrator_QP_solver_LA_DENSE_CHOL_4(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    double_integrator_QP_solver_FLOAT l;
    double_integrator_QP_solver_FLOAT Mii;

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
void double_integrator_QP_solver_LA_DENSE_MVMSUB1_4_7(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
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
void double_integrator_QP_solver_LA_DENSE_FORWARDSUB_4(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_FLOAT yel;
            
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
void double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_4(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_FLOAT xel;    
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
void double_integrator_QP_solver_LA_DENSE_MTVMSUB_4_7(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
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
void double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_FLOAT xel;    
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
void double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
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
 * Vector subtraction z = -x - y for vectors of length 201.
 */
void double_integrator_QP_solver_LA_VSUB2_201(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<201; i++){
		z[i] = -x[i] - y[i];
	}
}


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * lower triangular matrix of size 18 in lower triangular
 * storage format.
 */
void double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_FLOAT y[18];
    double_integrator_QP_solver_FLOAT yel,xel;
	int start = 153;
            
    /* first solve Ly = b by forward substitution */
     ii = 0; di = 0;
    for( i=0; i<18; i++ ){
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
    ii = start; di = 17;
    for( i=17; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 17;
        for( j=17; j>i; j-- ){
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
 * lower triangular matrix of size 16 in lower triangular
 * storage format.
 */
void double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_16(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_FLOAT y[16];
    double_integrator_QP_solver_FLOAT yel,xel;
	int start = 120;
            
    /* first solve Ly = b by forward substitution */
     ii = 0; di = 0;
    for( i=0; i<16; i++ ){
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
    ii = start; di = 15;
    for( i=15; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 15;
        for( j=15; j>i; j-- ){
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
 * lower triangular matrix of size 5 in lower triangular
 * storage format.
 */
void double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_5(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_FLOAT y[5];
    double_integrator_QP_solver_FLOAT yel,xel;
	int start = 10;
            
    /* first solve Ly = b by forward substitution */
     ii = 0; di = 0;
    for( i=0; i<5; i++ ){
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
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 7,
 * and x has length 18 and is indexed through yidx.
 */
void double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_FLOAT *x, int* xidx, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<7; i++){
		z[i] = x[xidx[i]] - y[i];
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
 * Vector subtraction z = -x - y(yidx) where y is of length 18
 * and z, x and yidx are of length 6.
 */
void double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y, int* yidx, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<6; i++){
		z[i] = -x[i] - y[yidx[i]];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 6.
 */
void double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *w, double_integrator_QP_solver_FLOAT *x)
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_18(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<24; i++ ){
		r[i] = -b[i] - A[k++]*x[0];
	}	
	for( j=1; j<18; j++ ){		
		for( i=0; i<24; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 24.
 */
void double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *w, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<24; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 6,
 * and x has length 18 and is indexed through yidx.
 */
void double_integrator_QP_solver_LA_VSUB_INDEXED_6(double_integrator_QP_solver_FLOAT *x, int* xidx, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_16(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<24; i++ ){
		r[i] = -b[i] - A[k++]*x[0];
	}	
	for( j=1; j<16; j++ ){		
		for( i=0; i<24; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 4,
 * and x has length 5 and is indexed through yidx.
 */
void double_integrator_QP_solver_LA_VSUB_INDEXED_4(double_integrator_QP_solver_FLOAT *x, int* xidx, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<4; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 4.
 */
void double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *w, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<4; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = -x - y(yidx) where y is of length 5
 * and z, x and yidx are of length 4.
 */
void double_integrator_QP_solver_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y, int* yidx, double_integrator_QP_solver_FLOAT *z)
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB4_10_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<10; i++ ){
		r[i] = -b[i] - A[k++]*x[0];
	}	
	for( j=1; j<5; j++ ){		
		for( i=0; i<10; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 10.
 */
void double_integrator_QP_solver_LA_VSUB3_10(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *w, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<10; i++){
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
        for( i=0; i<415; i++ ){
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
        if( i == 415 ){
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
    *mu_aff = mymu / (double_integrator_QP_solver_FLOAT)415;
    return lsIt;
}


/*
 * Vector subtraction x = (u.*v - mu)*sigma where a is a scalar
*  and x,u,v are vectors of length 415.
 */
void double_integrator_QP_solver_LA_VSUB5_415(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT mu,  double_integrator_QP_solver_FLOAT sigma, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<415; i++){
		x[i] = u[i]*v[i] - mu;
		x[i] *= sigma;
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 18,
 * u, su, uidx are of length 6 and v, sv, vidx are of length 7.
 */
void double_integrator_QP_solver_LA_VSUB6_INDEXED_18_6_7(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *su, int* uidx, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *sv, int* vidx, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<18; i++ ){
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
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 18,
 * u, su, uidx are of length 6 and v, sv, vidx are of length 6.
 */
void double_integrator_QP_solver_LA_VSUB6_INDEXED_18_6_6(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *su, int* uidx, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *sv, int* vidx, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<18; i++ ){
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
 * Matrix vector multiplication z = z + A'*(x./s) where A is of size [24 x 18]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_18(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0; 
	double_integrator_QP_solver_FLOAT temp[24];

	for( j=0; j<24; j++ ){
		temp[j] = x[j] / s[j];
	}

	for( i=0; i<18; i++ ){
		for( j=0; j<24; j++ ){
			z[i] += A[k++]*temp[j];
		}
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void double_integrator_QP_solver_LA_DENSE_2MVMADD_7_18_18(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<7; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<18; j++ ){		
		for( i=0; i<7; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<18; n++ ){
		for( i=0; i<7; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 16,
 * u, su, uidx are of length 6 and v, sv, vidx are of length 6.
 */
void double_integrator_QP_solver_LA_VSUB6_INDEXED_16_6_6(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *su, int* uidx, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *sv, int* vidx, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<16; i++ ){
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
 * Matrix vector multiplication z = z + A'*(x./s) where A is of size [24 x 16]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_16(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0; 
	double_integrator_QP_solver_FLOAT temp[24];

	for( j=0; j<24; j++ ){
		temp[j] = x[j] / s[j];
	}

	for( i=0; i<16; i++ ){
		for( j=0; j<24; j++ ){
			z[i] += A[k++]*temp[j];
		}
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void double_integrator_QP_solver_LA_DENSE_2MVMADD_7_18_16(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<7; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<18; j++ ){		
		for( i=0; i<7; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<16; n++ ){
		for( i=0; i<7; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 5,
 * u, su, uidx are of length 4 and v, sv, vidx are of length 4.
 */
void double_integrator_QP_solver_LA_VSUB6_INDEXED_5_4_4(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *su, int* uidx, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *sv, int* vidx, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<5; i++ ){
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
 * Matrix vector multiplication z = z + A'*(x./s) where A is of size [10 x 5]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_LA_DENSE_MTVMADD2_10_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0; 
	double_integrator_QP_solver_FLOAT temp[10];

	for( j=0; j<10; j++ ){
		temp[j] = x[j] / s[j];
	}

	for( i=0; i<5; i++ ){
		for( j=0; j<10; j++ ){
			z[i] += A[k++]*temp[j];
		}
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void double_integrator_QP_solver_LA_DENSE_2MVMADD_4_16_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<4; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<16; j++ ){		
		for( i=0; i<4; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<5; n++ ){
		for( i=0; i<4; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/*
 * Vector subtraction z = x - y for vectors of length 201.
 */
void double_integrator_QP_solver_LA_VSUB_201(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<201; i++){
		z[i] = x[i] - y[i];
	}
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 7 (length of y >= 7).
 */
void double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *y, int* yidx, double_integrator_QP_solver_FLOAT *z)
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
void double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *y, int* yidx, double_integrator_QP_solver_FLOAT *z)
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_18(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_FLOAT temp[24];

	
	for( i=0; i<24; i++ ){
		temp[i] = A[k++]*x[0];
	}
	

	for( j=1; j<18; j++ ){		
		for( i=0; i<24; i++ ){
			temp[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<24; i++ ){
		r[i] = (-b[i] + l[i]*temp[i])/s[i]; 
	}	
	
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 6 (length of y >= 6).
 */
void double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *y, int* yidx, double_integrator_QP_solver_FLOAT *z)
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_16(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_FLOAT temp[24];

	
	for( i=0; i<24; i++ ){
		temp[i] = A[k++]*x[0];
	}
	

	for( j=1; j<16; j++ ){		
		for( i=0; i<24; i++ ){
			temp[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<24; i++ ){
		r[i] = (-b[i] + l[i]*temp[i])/s[i]; 
	}	
	
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 4 (length of y >= 4).
 */
void double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *y, int* yidx, double_integrator_QP_solver_FLOAT *z)
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
void double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *y, int* yidx, double_integrator_QP_solver_FLOAT *z)
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB5_10_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_FLOAT temp[10];

	
	for( i=0; i<10; i++ ){
		temp[i] = A[k++]*x[0];
	}
	

	for( j=1; j<5; j++ ){		
		for( i=0; i<10; i++ ){
			temp[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<10; i++ ){
		r[i] = (-b[i] + l[i]*temp[i])/s[i]; 
	}	
	
}


/*
 * Computes ds = -l.\(r + s.*dl) for vectors of length 415.
 */
void double_integrator_QP_solver_LA_VSUB7_415(double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *dl, double_integrator_QP_solver_FLOAT *ds)
{
	int i;
	for( i=0; i<415; i++){
		ds[i] = -(r[i] + s[i]*dl[i])/l[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 201.
 */
void double_integrator_QP_solver_LA_VADD_201(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<201; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 74.
 */
void double_integrator_QP_solver_LA_VADD_74(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<74; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 415.
 */
void double_integrator_QP_solver_LA_VADD_415(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<415; i++){
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
        for( i=0; i<415; i++ ){
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
        if( i == 415 ){
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
    for( i=0; i<201; i++ ){
        z[i] += a_gamma*dz[i];
    }
    
    /* equality constraint multipliers */
    for( i=0; i<74; i++ ){
        v[i] += a_gamma*dv[i];
    }
    
    /* inequality constraint multipliers & slacks, also update mu */
    *mu = 0;
    for( i=0; i<415; i++ ){
        dltemp = l[i] + a_gamma*dl[i]; l[i] = dltemp;
        dstemp = s[i] + a_gamma*ds[i]; s[i] = dstemp;
        *mu += dltemp*dstemp;
    }
    
    *a = a_gamma;
    *mu /= (double_integrator_QP_solver_FLOAT)415;
    return lsIt;
}




/* VARIABLE DEFINITIONS ------------------------------------------------ */
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_z[201];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_v[74];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dz_aff[201];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dv_aff[74];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_grad_cost[201];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_grad_eq[201];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rd[201];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_l[415];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_s[415];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_lbys[415];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dl_aff[415];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ds_aff[415];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dz_cc[201];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dv_cc[74];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dl_cc[415];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ds_cc[415];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ccrhs[415];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_grad_ineq[201];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_H00[18] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z00 = double_integrator_QP_solver_z + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff00 = double_integrator_QP_solver_dz_aff + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc00 = double_integrator_QP_solver_dz_cc + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd00 = double_integrator_QP_solver_rd + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd00[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost00 = double_integrator_QP_solver_grad_cost + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq00 = double_integrator_QP_solver_grad_eq + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq00 = double_integrator_QP_solver_grad_ineq + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv00[18];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_C00[126] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
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
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v00 = double_integrator_QP_solver_v + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re00[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta00[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc00[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff00 = double_integrator_QP_solver_dv_aff + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc00 = double_integrator_QP_solver_dv_cc + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V00[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd00[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld00[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy00[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy00[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c00[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx00[7] = {0, 1, 2, 3, 4, 5, 12};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb00 = double_integrator_QP_solver_l + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb00 = double_integrator_QP_solver_s + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb00 = double_integrator_QP_solver_lbys + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb00[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff00 = double_integrator_QP_solver_dl_aff + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff00 = double_integrator_QP_solver_ds_aff + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc00 = double_integrator_QP_solver_dl_cc + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc00 = double_integrator_QP_solver_ds_cc + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl00 = double_integrator_QP_solver_ccrhs + 0;
int double_integrator_QP_solver_ubIdx00[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub00 = double_integrator_QP_solver_l + 7;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub00 = double_integrator_QP_solver_s + 7;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub00 = double_integrator_QP_solver_lbys + 7;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub00[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff00 = double_integrator_QP_solver_dl_aff + 7;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff00 = double_integrator_QP_solver_ds_aff + 7;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc00 = double_integrator_QP_solver_dl_cc + 7;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc00 = double_integrator_QP_solver_ds_cc + 7;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub00 = double_integrator_QP_solver_ccrhs + 7;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp00 = double_integrator_QP_solver_s + 13;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp00 = double_integrator_QP_solver_l + 13;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp00 = double_integrator_QP_solver_lbys + 13;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff00 = double_integrator_QP_solver_dl_aff + 13;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff00 = double_integrator_QP_solver_ds_aff + 13;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc00 = double_integrator_QP_solver_dl_cc + 13;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc00 = double_integrator_QP_solver_ds_cc + 13;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp00 = double_integrator_QP_solver_ccrhs + 13;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip00[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi00[171];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z01 = double_integrator_QP_solver_z + 18;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff01 = double_integrator_QP_solver_dz_aff + 18;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc01 = double_integrator_QP_solver_dz_cc + 18;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd01 = double_integrator_QP_solver_rd + 18;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd01[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost01 = double_integrator_QP_solver_grad_cost + 18;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq01 = double_integrator_QP_solver_grad_eq + 18;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq01 = double_integrator_QP_solver_grad_ineq + 18;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv01[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v01 = double_integrator_QP_solver_v + 7;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re01[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta01[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc01[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff01 = double_integrator_QP_solver_dv_aff + 7;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc01 = double_integrator_QP_solver_dv_cc + 7;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V01[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd01[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld01[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy01[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy01[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c01[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx01[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb01 = double_integrator_QP_solver_l + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb01 = double_integrator_QP_solver_s + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb01 = double_integrator_QP_solver_lbys + 37;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb01[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff01 = double_integrator_QP_solver_dl_aff + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff01 = double_integrator_QP_solver_ds_aff + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc01 = double_integrator_QP_solver_dl_cc + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc01 = double_integrator_QP_solver_ds_cc + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl01 = double_integrator_QP_solver_ccrhs + 37;
int double_integrator_QP_solver_ubIdx01[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub01 = double_integrator_QP_solver_l + 43;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub01 = double_integrator_QP_solver_s + 43;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub01 = double_integrator_QP_solver_lbys + 43;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub01[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff01 = double_integrator_QP_solver_dl_aff + 43;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff01 = double_integrator_QP_solver_ds_aff + 43;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc01 = double_integrator_QP_solver_dl_cc + 43;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc01 = double_integrator_QP_solver_ds_cc + 43;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub01 = double_integrator_QP_solver_ccrhs + 43;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp01 = double_integrator_QP_solver_s + 49;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp01 = double_integrator_QP_solver_l + 49;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp01 = double_integrator_QP_solver_lbys + 49;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff01 = double_integrator_QP_solver_dl_aff + 49;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff01 = double_integrator_QP_solver_ds_aff + 49;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc01 = double_integrator_QP_solver_dl_cc + 49;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc01 = double_integrator_QP_solver_ds_cc + 49;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp01 = double_integrator_QP_solver_ccrhs + 49;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip01[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi01[171];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_D01[126] = {-1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
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
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W01[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd01[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd01[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z02 = double_integrator_QP_solver_z + 36;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff02 = double_integrator_QP_solver_dz_aff + 36;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc02 = double_integrator_QP_solver_dz_cc + 36;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd02 = double_integrator_QP_solver_rd + 36;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd02[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost02 = double_integrator_QP_solver_grad_cost + 36;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq02 = double_integrator_QP_solver_grad_eq + 36;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq02 = double_integrator_QP_solver_grad_ineq + 36;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv02[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v02 = double_integrator_QP_solver_v + 14;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re02[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta02[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc02[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff02 = double_integrator_QP_solver_dv_aff + 14;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc02 = double_integrator_QP_solver_dv_cc + 14;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V02[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd02[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld02[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy02[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy02[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c02[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx02[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb02 = double_integrator_QP_solver_l + 73;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb02 = double_integrator_QP_solver_s + 73;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb02 = double_integrator_QP_solver_lbys + 73;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb02[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff02 = double_integrator_QP_solver_dl_aff + 73;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff02 = double_integrator_QP_solver_ds_aff + 73;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc02 = double_integrator_QP_solver_dl_cc + 73;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc02 = double_integrator_QP_solver_ds_cc + 73;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl02 = double_integrator_QP_solver_ccrhs + 73;
int double_integrator_QP_solver_ubIdx02[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub02 = double_integrator_QP_solver_l + 79;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub02 = double_integrator_QP_solver_s + 79;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub02 = double_integrator_QP_solver_lbys + 79;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub02[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff02 = double_integrator_QP_solver_dl_aff + 79;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff02 = double_integrator_QP_solver_ds_aff + 79;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc02 = double_integrator_QP_solver_dl_cc + 79;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc02 = double_integrator_QP_solver_ds_cc + 79;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub02 = double_integrator_QP_solver_ccrhs + 79;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp02 = double_integrator_QP_solver_s + 85;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp02 = double_integrator_QP_solver_l + 85;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp02 = double_integrator_QP_solver_lbys + 85;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff02 = double_integrator_QP_solver_dl_aff + 85;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff02 = double_integrator_QP_solver_ds_aff + 85;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc02 = double_integrator_QP_solver_dl_cc + 85;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc02 = double_integrator_QP_solver_ds_cc + 85;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp02 = double_integrator_QP_solver_ccrhs + 85;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip02[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi02[171];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W02[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd02[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd02[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z03 = double_integrator_QP_solver_z + 54;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff03 = double_integrator_QP_solver_dz_aff + 54;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc03 = double_integrator_QP_solver_dz_cc + 54;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd03 = double_integrator_QP_solver_rd + 54;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd03[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost03 = double_integrator_QP_solver_grad_cost + 54;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq03 = double_integrator_QP_solver_grad_eq + 54;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq03 = double_integrator_QP_solver_grad_ineq + 54;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv03[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v03 = double_integrator_QP_solver_v + 21;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re03[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta03[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc03[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff03 = double_integrator_QP_solver_dv_aff + 21;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc03 = double_integrator_QP_solver_dv_cc + 21;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V03[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd03[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld03[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy03[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy03[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c03[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx03[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb03 = double_integrator_QP_solver_l + 109;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb03 = double_integrator_QP_solver_s + 109;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb03 = double_integrator_QP_solver_lbys + 109;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb03[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff03 = double_integrator_QP_solver_dl_aff + 109;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff03 = double_integrator_QP_solver_ds_aff + 109;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc03 = double_integrator_QP_solver_dl_cc + 109;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc03 = double_integrator_QP_solver_ds_cc + 109;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl03 = double_integrator_QP_solver_ccrhs + 109;
int double_integrator_QP_solver_ubIdx03[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub03 = double_integrator_QP_solver_l + 115;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub03 = double_integrator_QP_solver_s + 115;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub03 = double_integrator_QP_solver_lbys + 115;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub03[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff03 = double_integrator_QP_solver_dl_aff + 115;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff03 = double_integrator_QP_solver_ds_aff + 115;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc03 = double_integrator_QP_solver_dl_cc + 115;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc03 = double_integrator_QP_solver_ds_cc + 115;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub03 = double_integrator_QP_solver_ccrhs + 115;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp03 = double_integrator_QP_solver_s + 121;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp03 = double_integrator_QP_solver_l + 121;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp03 = double_integrator_QP_solver_lbys + 121;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff03 = double_integrator_QP_solver_dl_aff + 121;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff03 = double_integrator_QP_solver_ds_aff + 121;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc03 = double_integrator_QP_solver_dl_cc + 121;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc03 = double_integrator_QP_solver_ds_cc + 121;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp03 = double_integrator_QP_solver_ccrhs + 121;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip03[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi03[171];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W03[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd03[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd03[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z04 = double_integrator_QP_solver_z + 72;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff04 = double_integrator_QP_solver_dz_aff + 72;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc04 = double_integrator_QP_solver_dz_cc + 72;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd04 = double_integrator_QP_solver_rd + 72;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd04[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost04 = double_integrator_QP_solver_grad_cost + 72;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq04 = double_integrator_QP_solver_grad_eq + 72;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq04 = double_integrator_QP_solver_grad_ineq + 72;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv04[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v04 = double_integrator_QP_solver_v + 28;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re04[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta04[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc04[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff04 = double_integrator_QP_solver_dv_aff + 28;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc04 = double_integrator_QP_solver_dv_cc + 28;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V04[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd04[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld04[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy04[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy04[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c04[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx04[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb04 = double_integrator_QP_solver_l + 145;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb04 = double_integrator_QP_solver_s + 145;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb04 = double_integrator_QP_solver_lbys + 145;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb04[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff04 = double_integrator_QP_solver_dl_aff + 145;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff04 = double_integrator_QP_solver_ds_aff + 145;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc04 = double_integrator_QP_solver_dl_cc + 145;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc04 = double_integrator_QP_solver_ds_cc + 145;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl04 = double_integrator_QP_solver_ccrhs + 145;
int double_integrator_QP_solver_ubIdx04[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub04 = double_integrator_QP_solver_l + 151;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub04 = double_integrator_QP_solver_s + 151;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub04 = double_integrator_QP_solver_lbys + 151;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub04[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff04 = double_integrator_QP_solver_dl_aff + 151;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff04 = double_integrator_QP_solver_ds_aff + 151;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc04 = double_integrator_QP_solver_dl_cc + 151;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc04 = double_integrator_QP_solver_ds_cc + 151;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub04 = double_integrator_QP_solver_ccrhs + 151;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp04 = double_integrator_QP_solver_s + 157;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp04 = double_integrator_QP_solver_l + 157;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp04 = double_integrator_QP_solver_lbys + 157;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff04 = double_integrator_QP_solver_dl_aff + 157;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff04 = double_integrator_QP_solver_ds_aff + 157;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc04 = double_integrator_QP_solver_dl_cc + 157;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc04 = double_integrator_QP_solver_ds_cc + 157;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp04 = double_integrator_QP_solver_ccrhs + 157;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip04[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi04[171];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W04[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd04[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd04[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z05 = double_integrator_QP_solver_z + 90;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff05 = double_integrator_QP_solver_dz_aff + 90;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc05 = double_integrator_QP_solver_dz_cc + 90;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd05 = double_integrator_QP_solver_rd + 90;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd05[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost05 = double_integrator_QP_solver_grad_cost + 90;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq05 = double_integrator_QP_solver_grad_eq + 90;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq05 = double_integrator_QP_solver_grad_ineq + 90;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv05[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v05 = double_integrator_QP_solver_v + 35;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re05[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta05[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc05[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff05 = double_integrator_QP_solver_dv_aff + 35;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc05 = double_integrator_QP_solver_dv_cc + 35;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V05[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd05[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld05[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy05[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy05[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c05[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx05[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb05 = double_integrator_QP_solver_l + 181;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb05 = double_integrator_QP_solver_s + 181;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb05 = double_integrator_QP_solver_lbys + 181;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb05[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff05 = double_integrator_QP_solver_dl_aff + 181;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff05 = double_integrator_QP_solver_ds_aff + 181;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc05 = double_integrator_QP_solver_dl_cc + 181;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc05 = double_integrator_QP_solver_ds_cc + 181;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl05 = double_integrator_QP_solver_ccrhs + 181;
int double_integrator_QP_solver_ubIdx05[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub05 = double_integrator_QP_solver_l + 187;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub05 = double_integrator_QP_solver_s + 187;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub05 = double_integrator_QP_solver_lbys + 187;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub05[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff05 = double_integrator_QP_solver_dl_aff + 187;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff05 = double_integrator_QP_solver_ds_aff + 187;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc05 = double_integrator_QP_solver_dl_cc + 187;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc05 = double_integrator_QP_solver_ds_cc + 187;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub05 = double_integrator_QP_solver_ccrhs + 187;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp05 = double_integrator_QP_solver_s + 193;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp05 = double_integrator_QP_solver_l + 193;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp05 = double_integrator_QP_solver_lbys + 193;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff05 = double_integrator_QP_solver_dl_aff + 193;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff05 = double_integrator_QP_solver_ds_aff + 193;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc05 = double_integrator_QP_solver_dl_cc + 193;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc05 = double_integrator_QP_solver_ds_cc + 193;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp05 = double_integrator_QP_solver_ccrhs + 193;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip05[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi05[171];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W05[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd05[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd05[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z06 = double_integrator_QP_solver_z + 108;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff06 = double_integrator_QP_solver_dz_aff + 108;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc06 = double_integrator_QP_solver_dz_cc + 108;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd06 = double_integrator_QP_solver_rd + 108;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd06[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost06 = double_integrator_QP_solver_grad_cost + 108;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq06 = double_integrator_QP_solver_grad_eq + 108;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq06 = double_integrator_QP_solver_grad_ineq + 108;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv06[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v06 = double_integrator_QP_solver_v + 42;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re06[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta06[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc06[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff06 = double_integrator_QP_solver_dv_aff + 42;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc06 = double_integrator_QP_solver_dv_cc + 42;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V06[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd06[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld06[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy06[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy06[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c06[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx06[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb06 = double_integrator_QP_solver_l + 217;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb06 = double_integrator_QP_solver_s + 217;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb06 = double_integrator_QP_solver_lbys + 217;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb06[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff06 = double_integrator_QP_solver_dl_aff + 217;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff06 = double_integrator_QP_solver_ds_aff + 217;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc06 = double_integrator_QP_solver_dl_cc + 217;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc06 = double_integrator_QP_solver_ds_cc + 217;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl06 = double_integrator_QP_solver_ccrhs + 217;
int double_integrator_QP_solver_ubIdx06[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub06 = double_integrator_QP_solver_l + 223;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub06 = double_integrator_QP_solver_s + 223;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub06 = double_integrator_QP_solver_lbys + 223;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub06[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff06 = double_integrator_QP_solver_dl_aff + 223;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff06 = double_integrator_QP_solver_ds_aff + 223;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc06 = double_integrator_QP_solver_dl_cc + 223;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc06 = double_integrator_QP_solver_ds_cc + 223;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub06 = double_integrator_QP_solver_ccrhs + 223;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp06 = double_integrator_QP_solver_s + 229;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp06 = double_integrator_QP_solver_l + 229;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp06 = double_integrator_QP_solver_lbys + 229;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff06 = double_integrator_QP_solver_dl_aff + 229;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff06 = double_integrator_QP_solver_ds_aff + 229;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc06 = double_integrator_QP_solver_dl_cc + 229;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc06 = double_integrator_QP_solver_ds_cc + 229;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp06 = double_integrator_QP_solver_ccrhs + 229;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip06[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi06[171];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W06[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd06[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd06[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z07 = double_integrator_QP_solver_z + 126;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff07 = double_integrator_QP_solver_dz_aff + 126;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc07 = double_integrator_QP_solver_dz_cc + 126;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd07 = double_integrator_QP_solver_rd + 126;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd07[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost07 = double_integrator_QP_solver_grad_cost + 126;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq07 = double_integrator_QP_solver_grad_eq + 126;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq07 = double_integrator_QP_solver_grad_ineq + 126;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv07[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v07 = double_integrator_QP_solver_v + 49;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re07[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta07[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc07[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff07 = double_integrator_QP_solver_dv_aff + 49;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc07 = double_integrator_QP_solver_dv_cc + 49;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V07[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd07[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld07[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy07[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy07[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c07[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx07[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb07 = double_integrator_QP_solver_l + 253;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb07 = double_integrator_QP_solver_s + 253;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb07 = double_integrator_QP_solver_lbys + 253;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb07[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff07 = double_integrator_QP_solver_dl_aff + 253;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff07 = double_integrator_QP_solver_ds_aff + 253;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc07 = double_integrator_QP_solver_dl_cc + 253;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc07 = double_integrator_QP_solver_ds_cc + 253;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl07 = double_integrator_QP_solver_ccrhs + 253;
int double_integrator_QP_solver_ubIdx07[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub07 = double_integrator_QP_solver_l + 259;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub07 = double_integrator_QP_solver_s + 259;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub07 = double_integrator_QP_solver_lbys + 259;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub07[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff07 = double_integrator_QP_solver_dl_aff + 259;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff07 = double_integrator_QP_solver_ds_aff + 259;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc07 = double_integrator_QP_solver_dl_cc + 259;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc07 = double_integrator_QP_solver_ds_cc + 259;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub07 = double_integrator_QP_solver_ccrhs + 259;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp07 = double_integrator_QP_solver_s + 265;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp07 = double_integrator_QP_solver_l + 265;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp07 = double_integrator_QP_solver_lbys + 265;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff07 = double_integrator_QP_solver_dl_aff + 265;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff07 = double_integrator_QP_solver_ds_aff + 265;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc07 = double_integrator_QP_solver_dl_cc + 265;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc07 = double_integrator_QP_solver_ds_cc + 265;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp07 = double_integrator_QP_solver_ccrhs + 265;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip07[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi07[171];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W07[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd07[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd07[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z08 = double_integrator_QP_solver_z + 144;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff08 = double_integrator_QP_solver_dz_aff + 144;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc08 = double_integrator_QP_solver_dz_cc + 144;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd08 = double_integrator_QP_solver_rd + 144;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd08[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost08 = double_integrator_QP_solver_grad_cost + 144;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq08 = double_integrator_QP_solver_grad_eq + 144;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq08 = double_integrator_QP_solver_grad_ineq + 144;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv08[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v08 = double_integrator_QP_solver_v + 56;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re08[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta08[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc08[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff08 = double_integrator_QP_solver_dv_aff + 56;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc08 = double_integrator_QP_solver_dv_cc + 56;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V08[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd08[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld08[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy08[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy08[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c08[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx08[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb08 = double_integrator_QP_solver_l + 289;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb08 = double_integrator_QP_solver_s + 289;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb08 = double_integrator_QP_solver_lbys + 289;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb08[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff08 = double_integrator_QP_solver_dl_aff + 289;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff08 = double_integrator_QP_solver_ds_aff + 289;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc08 = double_integrator_QP_solver_dl_cc + 289;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc08 = double_integrator_QP_solver_ds_cc + 289;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl08 = double_integrator_QP_solver_ccrhs + 289;
int double_integrator_QP_solver_ubIdx08[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub08 = double_integrator_QP_solver_l + 295;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub08 = double_integrator_QP_solver_s + 295;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub08 = double_integrator_QP_solver_lbys + 295;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub08[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff08 = double_integrator_QP_solver_dl_aff + 295;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff08 = double_integrator_QP_solver_ds_aff + 295;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc08 = double_integrator_QP_solver_dl_cc + 295;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc08 = double_integrator_QP_solver_ds_cc + 295;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub08 = double_integrator_QP_solver_ccrhs + 295;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp08 = double_integrator_QP_solver_s + 301;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp08 = double_integrator_QP_solver_l + 301;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp08 = double_integrator_QP_solver_lbys + 301;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff08 = double_integrator_QP_solver_dl_aff + 301;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff08 = double_integrator_QP_solver_ds_aff + 301;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc08 = double_integrator_QP_solver_dl_cc + 301;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc08 = double_integrator_QP_solver_ds_cc + 301;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp08 = double_integrator_QP_solver_ccrhs + 301;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip08[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi08[171];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W08[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd08[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd08[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z09 = double_integrator_QP_solver_z + 162;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff09 = double_integrator_QP_solver_dz_aff + 162;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc09 = double_integrator_QP_solver_dz_cc + 162;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd09 = double_integrator_QP_solver_rd + 162;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd09[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost09 = double_integrator_QP_solver_grad_cost + 162;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq09 = double_integrator_QP_solver_grad_eq + 162;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq09 = double_integrator_QP_solver_grad_ineq + 162;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv09[18];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v09 = double_integrator_QP_solver_v + 63;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re09[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta09[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc09[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff09 = double_integrator_QP_solver_dv_aff + 63;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc09 = double_integrator_QP_solver_dv_cc + 63;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V09[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd09[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld09[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy09[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy09[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c09[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx09[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb09 = double_integrator_QP_solver_l + 325;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb09 = double_integrator_QP_solver_s + 325;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb09 = double_integrator_QP_solver_lbys + 325;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb09[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff09 = double_integrator_QP_solver_dl_aff + 325;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff09 = double_integrator_QP_solver_ds_aff + 325;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc09 = double_integrator_QP_solver_dl_cc + 325;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc09 = double_integrator_QP_solver_ds_cc + 325;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl09 = double_integrator_QP_solver_ccrhs + 325;
int double_integrator_QP_solver_ubIdx09[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub09 = double_integrator_QP_solver_l + 331;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub09 = double_integrator_QP_solver_s + 331;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub09 = double_integrator_QP_solver_lbys + 331;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub09[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff09 = double_integrator_QP_solver_dl_aff + 331;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff09 = double_integrator_QP_solver_ds_aff + 331;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc09 = double_integrator_QP_solver_dl_cc + 331;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc09 = double_integrator_QP_solver_ds_cc + 331;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub09 = double_integrator_QP_solver_ccrhs + 331;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp09 = double_integrator_QP_solver_s + 337;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp09 = double_integrator_QP_solver_l + 337;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp09 = double_integrator_QP_solver_lbys + 337;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff09 = double_integrator_QP_solver_dl_aff + 337;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff09 = double_integrator_QP_solver_ds_aff + 337;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc09 = double_integrator_QP_solver_dl_cc + 337;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc09 = double_integrator_QP_solver_ds_cc + 337;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp09 = double_integrator_QP_solver_ccrhs + 337;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip09[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi09[171];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W09[126];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd09[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd09[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_H10[16] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z10 = double_integrator_QP_solver_z + 180;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff10 = double_integrator_QP_solver_dz_aff + 180;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc10 = double_integrator_QP_solver_dz_cc + 180;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd10 = double_integrator_QP_solver_rd + 180;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd10[16];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost10 = double_integrator_QP_solver_grad_cost + 180;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq10 = double_integrator_QP_solver_grad_eq + 180;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq10 = double_integrator_QP_solver_grad_ineq + 180;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv10[16];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_C10[64] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
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
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v10 = double_integrator_QP_solver_v + 70;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re10[4];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta10[4];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc10[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff10 = double_integrator_QP_solver_dv_aff + 70;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc10 = double_integrator_QP_solver_dv_cc + 70;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V10[64];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd10[10];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld10[10];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy10[4];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy10[4];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c10[4] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx10[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb10 = double_integrator_QP_solver_l + 361;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb10 = double_integrator_QP_solver_s + 361;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb10 = double_integrator_QP_solver_lbys + 361;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb10[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff10 = double_integrator_QP_solver_dl_aff + 361;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff10 = double_integrator_QP_solver_ds_aff + 361;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc10 = double_integrator_QP_solver_dl_cc + 361;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc10 = double_integrator_QP_solver_ds_cc + 361;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl10 = double_integrator_QP_solver_ccrhs + 361;
int double_integrator_QP_solver_ubIdx10[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub10 = double_integrator_QP_solver_l + 367;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub10 = double_integrator_QP_solver_s + 367;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub10 = double_integrator_QP_solver_lbys + 367;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub10[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff10 = double_integrator_QP_solver_dl_aff + 367;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff10 = double_integrator_QP_solver_ds_aff + 367;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc10 = double_integrator_QP_solver_dl_cc + 367;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc10 = double_integrator_QP_solver_ds_cc + 367;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub10 = double_integrator_QP_solver_ccrhs + 367;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp10 = double_integrator_QP_solver_s + 373;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp10 = double_integrator_QP_solver_l + 373;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp10 = double_integrator_QP_solver_lbys + 373;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff10 = double_integrator_QP_solver_dl_aff + 373;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff10 = double_integrator_QP_solver_ds_aff + 373;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc10 = double_integrator_QP_solver_dl_cc + 373;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc10 = double_integrator_QP_solver_ds_cc + 373;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp10 = double_integrator_QP_solver_ccrhs + 373;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip10[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi10[136];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_D10[112] = {-1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
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
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W10[112];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd10[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd10[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_H11[5] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z11 = double_integrator_QP_solver_z + 196;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff11 = double_integrator_QP_solver_dz_aff + 196;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc11 = double_integrator_QP_solver_dz_cc + 196;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd11 = double_integrator_QP_solver_rd + 196;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd11[5];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost11 = double_integrator_QP_solver_grad_cost + 196;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq11 = double_integrator_QP_solver_grad_eq + 196;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq11 = double_integrator_QP_solver_grad_ineq + 196;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv11[5];
int double_integrator_QP_solver_lbIdx11[4] = {0, 1, 2, 3};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb11 = double_integrator_QP_solver_l + 397;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb11 = double_integrator_QP_solver_s + 397;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb11 = double_integrator_QP_solver_lbys + 397;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb11[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff11 = double_integrator_QP_solver_dl_aff + 397;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff11 = double_integrator_QP_solver_ds_aff + 397;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc11 = double_integrator_QP_solver_dl_cc + 397;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc11 = double_integrator_QP_solver_ds_cc + 397;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl11 = double_integrator_QP_solver_ccrhs + 397;
int double_integrator_QP_solver_ubIdx11[4] = {0, 1, 2, 3};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub11 = double_integrator_QP_solver_l + 401;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub11 = double_integrator_QP_solver_s + 401;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub11 = double_integrator_QP_solver_lbys + 401;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub11[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff11 = double_integrator_QP_solver_dl_aff + 401;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff11 = double_integrator_QP_solver_ds_aff + 401;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc11 = double_integrator_QP_solver_dl_cc + 401;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc11 = double_integrator_QP_solver_ds_cc + 401;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub11 = double_integrator_QP_solver_ccrhs + 401;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp11 = double_integrator_QP_solver_s + 405;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp11 = double_integrator_QP_solver_l + 405;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp11 = double_integrator_QP_solver_lbys + 405;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff11 = double_integrator_QP_solver_dl_aff + 405;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff11 = double_integrator_QP_solver_ds_aff + 405;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc11 = double_integrator_QP_solver_dl_cc + 405;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc11 = double_integrator_QP_solver_ds_cc + 405;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp11 = double_integrator_QP_solver_ccrhs + 405;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip11[10];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi11[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_D11[5] = {-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000};
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W11[20];
double_integrator_QP_solver_FLOAT musigma;
double_integrator_QP_solver_FLOAT sigma_3rdroot;




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
double_integrator_QP_solver_LA_INITIALIZEVECTOR_201(double_integrator_QP_solver_z, 0);
double_integrator_QP_solver_LA_INITIALIZEVECTOR_74(double_integrator_QP_solver_v, 1);
double_integrator_QP_solver_LA_INITIALIZEVECTOR_415(double_integrator_QP_solver_l, 10);
double_integrator_QP_solver_LA_INITIALIZEVECTOR_415(double_integrator_QP_solver_s, 10);
info->mu = 0;
double_integrator_QP_solver_LA_DOTACC_415(double_integrator_QP_solver_l, double_integrator_QP_solver_s, &info->mu);
info->mu /= 415;
PRINTTEXT("This is double_integrator_QP_solver, a solver generated by FORCES (forces.ethz.ch).\n");
PRINTTEXT("(c) Alexander Domahidi, Automatic Control Laboratory, ETH Zurich, 2011-2014.\n");
PRINTTEXT("\n  #it  res_eq   res_ineq     pobj         dobj       dgap     rdgap     mu\n");
PRINTTEXT("  ---------------------------------------------------------------------------\n");
while( 1 ){
info->pobj = 0;
double_integrator_QP_solver_LA_DIAG_QUADFCN_18(double_integrator_QP_solver_H00, params->f1, double_integrator_QP_solver_z00, double_integrator_QP_solver_grad_cost00, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_18(double_integrator_QP_solver_H00, params->f2, double_integrator_QP_solver_z01, double_integrator_QP_solver_grad_cost01, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_18(double_integrator_QP_solver_H00, params->f3, double_integrator_QP_solver_z02, double_integrator_QP_solver_grad_cost02, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_18(double_integrator_QP_solver_H00, params->f4, double_integrator_QP_solver_z03, double_integrator_QP_solver_grad_cost03, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_18(double_integrator_QP_solver_H00, params->f5, double_integrator_QP_solver_z04, double_integrator_QP_solver_grad_cost04, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_18(double_integrator_QP_solver_H00, params->f6, double_integrator_QP_solver_z05, double_integrator_QP_solver_grad_cost05, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_18(double_integrator_QP_solver_H00, params->f7, double_integrator_QP_solver_z06, double_integrator_QP_solver_grad_cost06, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_18(double_integrator_QP_solver_H00, params->f8, double_integrator_QP_solver_z07, double_integrator_QP_solver_grad_cost07, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_18(double_integrator_QP_solver_H00, params->f9, double_integrator_QP_solver_z08, double_integrator_QP_solver_grad_cost08, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_18(double_integrator_QP_solver_H00, params->f10, double_integrator_QP_solver_z09, double_integrator_QP_solver_grad_cost09, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_16(double_integrator_QP_solver_H10, params->f11, double_integrator_QP_solver_z10, double_integrator_QP_solver_grad_cost10, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_5(double_integrator_QP_solver_H11, params->f12, double_integrator_QP_solver_z11, double_integrator_QP_solver_grad_cost11, &info->pobj);
info->res_eq = 0;
info->dgap = 0;
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_18_18(double_integrator_QP_solver_C00, double_integrator_QP_solver_z00, double_integrator_QP_solver_D01, double_integrator_QP_solver_z01, double_integrator_QP_solver_c00, double_integrator_QP_solver_v00, double_integrator_QP_solver_re00, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_18_18(double_integrator_QP_solver_C00, double_integrator_QP_solver_z01, double_integrator_QP_solver_D01, double_integrator_QP_solver_z02, double_integrator_QP_solver_c01, double_integrator_QP_solver_v01, double_integrator_QP_solver_re01, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_18_18(double_integrator_QP_solver_C00, double_integrator_QP_solver_z02, double_integrator_QP_solver_D01, double_integrator_QP_solver_z03, double_integrator_QP_solver_c02, double_integrator_QP_solver_v02, double_integrator_QP_solver_re02, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_18_18(double_integrator_QP_solver_C00, double_integrator_QP_solver_z03, double_integrator_QP_solver_D01, double_integrator_QP_solver_z04, double_integrator_QP_solver_c03, double_integrator_QP_solver_v03, double_integrator_QP_solver_re03, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_18_18(double_integrator_QP_solver_C00, double_integrator_QP_solver_z04, double_integrator_QP_solver_D01, double_integrator_QP_solver_z05, double_integrator_QP_solver_c04, double_integrator_QP_solver_v04, double_integrator_QP_solver_re04, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_18_18(double_integrator_QP_solver_C00, double_integrator_QP_solver_z05, double_integrator_QP_solver_D01, double_integrator_QP_solver_z06, double_integrator_QP_solver_c05, double_integrator_QP_solver_v05, double_integrator_QP_solver_re05, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_18_18(double_integrator_QP_solver_C00, double_integrator_QP_solver_z06, double_integrator_QP_solver_D01, double_integrator_QP_solver_z07, double_integrator_QP_solver_c06, double_integrator_QP_solver_v06, double_integrator_QP_solver_re06, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_18_18(double_integrator_QP_solver_C00, double_integrator_QP_solver_z07, double_integrator_QP_solver_D01, double_integrator_QP_solver_z08, double_integrator_QP_solver_c07, double_integrator_QP_solver_v07, double_integrator_QP_solver_re07, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_18_18(double_integrator_QP_solver_C00, double_integrator_QP_solver_z08, double_integrator_QP_solver_D01, double_integrator_QP_solver_z09, double_integrator_QP_solver_c08, double_integrator_QP_solver_v08, double_integrator_QP_solver_re08, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_18_16(double_integrator_QP_solver_C00, double_integrator_QP_solver_z09, double_integrator_QP_solver_D10, double_integrator_QP_solver_z10, double_integrator_QP_solver_c09, double_integrator_QP_solver_v09, double_integrator_QP_solver_re09, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_4_16_5(double_integrator_QP_solver_C10, double_integrator_QP_solver_z10, double_integrator_QP_solver_D11, double_integrator_QP_solver_z11, double_integrator_QP_solver_c10, double_integrator_QP_solver_v10, double_integrator_QP_solver_re10, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MTVM_7_18(double_integrator_QP_solver_C00, double_integrator_QP_solver_v00, double_integrator_QP_solver_grad_eq00);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v01, double_integrator_QP_solver_D01, double_integrator_QP_solver_v00, double_integrator_QP_solver_grad_eq01);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v02, double_integrator_QP_solver_D01, double_integrator_QP_solver_v01, double_integrator_QP_solver_grad_eq02);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v03, double_integrator_QP_solver_D01, double_integrator_QP_solver_v02, double_integrator_QP_solver_grad_eq03);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v04, double_integrator_QP_solver_D01, double_integrator_QP_solver_v03, double_integrator_QP_solver_grad_eq04);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v05, double_integrator_QP_solver_D01, double_integrator_QP_solver_v04, double_integrator_QP_solver_grad_eq05);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v06, double_integrator_QP_solver_D01, double_integrator_QP_solver_v05, double_integrator_QP_solver_grad_eq06);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v07, double_integrator_QP_solver_D01, double_integrator_QP_solver_v06, double_integrator_QP_solver_grad_eq07);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v08, double_integrator_QP_solver_D01, double_integrator_QP_solver_v07, double_integrator_QP_solver_grad_eq08);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v09, double_integrator_QP_solver_D01, double_integrator_QP_solver_v08, double_integrator_QP_solver_grad_eq09);
double_integrator_QP_solver_LA_DENSE_MTVM2_4_16_7(double_integrator_QP_solver_C10, double_integrator_QP_solver_v10, double_integrator_QP_solver_D10, double_integrator_QP_solver_v09, double_integrator_QP_solver_grad_eq10);
double_integrator_QP_solver_LA_DIAGZERO_MTVM_4_5(double_integrator_QP_solver_D11, double_integrator_QP_solver_v10, double_integrator_QP_solver_grad_eq11);
info->res_ineq = 0;
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb1, double_integrator_QP_solver_z00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_llb00, double_integrator_QP_solver_slb00, double_integrator_QP_solver_rilb00, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z00, double_integrator_QP_solver_ubIdx00, params->ub1, double_integrator_QP_solver_lub00, double_integrator_QP_solver_sub00, double_integrator_QP_solver_riub00, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_24_18(params->A1, double_integrator_QP_solver_z00, params->b1, double_integrator_QP_solver_sp00, double_integrator_QP_solver_lp00, double_integrator_QP_solver_rip00, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_6(params->lb2, double_integrator_QP_solver_z01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_llb01, double_integrator_QP_solver_slb01, double_integrator_QP_solver_rilb01, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z01, double_integrator_QP_solver_ubIdx01, params->ub2, double_integrator_QP_solver_lub01, double_integrator_QP_solver_sub01, double_integrator_QP_solver_riub01, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_24_18(params->A2, double_integrator_QP_solver_z01, params->b2, double_integrator_QP_solver_sp01, double_integrator_QP_solver_lp01, double_integrator_QP_solver_rip01, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_6(params->lb3, double_integrator_QP_solver_z02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_llb02, double_integrator_QP_solver_slb02, double_integrator_QP_solver_rilb02, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z02, double_integrator_QP_solver_ubIdx02, params->ub3, double_integrator_QP_solver_lub02, double_integrator_QP_solver_sub02, double_integrator_QP_solver_riub02, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_24_18(params->A3, double_integrator_QP_solver_z02, params->b3, double_integrator_QP_solver_sp02, double_integrator_QP_solver_lp02, double_integrator_QP_solver_rip02, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_6(params->lb4, double_integrator_QP_solver_z03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_llb03, double_integrator_QP_solver_slb03, double_integrator_QP_solver_rilb03, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z03, double_integrator_QP_solver_ubIdx03, params->ub4, double_integrator_QP_solver_lub03, double_integrator_QP_solver_sub03, double_integrator_QP_solver_riub03, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_24_18(params->A4, double_integrator_QP_solver_z03, params->b4, double_integrator_QP_solver_sp03, double_integrator_QP_solver_lp03, double_integrator_QP_solver_rip03, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_6(params->lb5, double_integrator_QP_solver_z04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_llb04, double_integrator_QP_solver_slb04, double_integrator_QP_solver_rilb04, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z04, double_integrator_QP_solver_ubIdx04, params->ub5, double_integrator_QP_solver_lub04, double_integrator_QP_solver_sub04, double_integrator_QP_solver_riub04, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_24_18(params->A5, double_integrator_QP_solver_z04, params->b5, double_integrator_QP_solver_sp04, double_integrator_QP_solver_lp04, double_integrator_QP_solver_rip04, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_6(params->lb6, double_integrator_QP_solver_z05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_llb05, double_integrator_QP_solver_slb05, double_integrator_QP_solver_rilb05, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z05, double_integrator_QP_solver_ubIdx05, params->ub6, double_integrator_QP_solver_lub05, double_integrator_QP_solver_sub05, double_integrator_QP_solver_riub05, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_24_18(params->A6, double_integrator_QP_solver_z05, params->b6, double_integrator_QP_solver_sp05, double_integrator_QP_solver_lp05, double_integrator_QP_solver_rip05, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_6(params->lb7, double_integrator_QP_solver_z06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_llb06, double_integrator_QP_solver_slb06, double_integrator_QP_solver_rilb06, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z06, double_integrator_QP_solver_ubIdx06, params->ub7, double_integrator_QP_solver_lub06, double_integrator_QP_solver_sub06, double_integrator_QP_solver_riub06, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_24_18(params->A7, double_integrator_QP_solver_z06, params->b7, double_integrator_QP_solver_sp06, double_integrator_QP_solver_lp06, double_integrator_QP_solver_rip06, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_6(params->lb8, double_integrator_QP_solver_z07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_llb07, double_integrator_QP_solver_slb07, double_integrator_QP_solver_rilb07, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z07, double_integrator_QP_solver_ubIdx07, params->ub8, double_integrator_QP_solver_lub07, double_integrator_QP_solver_sub07, double_integrator_QP_solver_riub07, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_24_18(params->A8, double_integrator_QP_solver_z07, params->b8, double_integrator_QP_solver_sp07, double_integrator_QP_solver_lp07, double_integrator_QP_solver_rip07, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_6(params->lb9, double_integrator_QP_solver_z08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_llb08, double_integrator_QP_solver_slb08, double_integrator_QP_solver_rilb08, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z08, double_integrator_QP_solver_ubIdx08, params->ub9, double_integrator_QP_solver_lub08, double_integrator_QP_solver_sub08, double_integrator_QP_solver_riub08, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_24_18(params->A9, double_integrator_QP_solver_z08, params->b9, double_integrator_QP_solver_sp08, double_integrator_QP_solver_lp08, double_integrator_QP_solver_rip08, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_6(params->lb10, double_integrator_QP_solver_z09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_llb09, double_integrator_QP_solver_slb09, double_integrator_QP_solver_rilb09, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z09, double_integrator_QP_solver_ubIdx09, params->ub10, double_integrator_QP_solver_lub09, double_integrator_QP_solver_sub09, double_integrator_QP_solver_riub09, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_24_18(params->A10, double_integrator_QP_solver_z09, params->b10, double_integrator_QP_solver_sp09, double_integrator_QP_solver_lp09, double_integrator_QP_solver_rip09, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_6(params->lb11, double_integrator_QP_solver_z10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_llb10, double_integrator_QP_solver_slb10, double_integrator_QP_solver_rilb10, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z10, double_integrator_QP_solver_ubIdx10, params->ub11, double_integrator_QP_solver_lub10, double_integrator_QP_solver_sub10, double_integrator_QP_solver_riub10, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_24_16(params->A11, double_integrator_QP_solver_z10, params->b11, double_integrator_QP_solver_sp10, double_integrator_QP_solver_lp10, double_integrator_QP_solver_rip10, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_4(params->lb12, double_integrator_QP_solver_z11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_llb11, double_integrator_QP_solver_slb11, double_integrator_QP_solver_rilb11, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_4(double_integrator_QP_solver_z11, double_integrator_QP_solver_ubIdx11, params->ub12, double_integrator_QP_solver_lub11, double_integrator_QP_solver_sub11, double_integrator_QP_solver_riub11, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_10_5(params->A12, double_integrator_QP_solver_z11, params->b12, double_integrator_QP_solver_sp11, double_integrator_QP_solver_lp11, double_integrator_QP_solver_rip11, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_INEQ_B_GRAD_18_7_6(double_integrator_QP_solver_lub00, double_integrator_QP_solver_sub00, double_integrator_QP_solver_riub00, double_integrator_QP_solver_llb00, double_integrator_QP_solver_slb00, double_integrator_QP_solver_rilb00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_grad_ineq00, double_integrator_QP_solver_lubbysub00, double_integrator_QP_solver_llbbyslb00);
double_integrator_QP_solver_LA_INEQ_P_24_18(params->A1, double_integrator_QP_solver_lp00, double_integrator_QP_solver_sp00, double_integrator_QP_solver_rip00, double_integrator_QP_solver_grad_ineq00, double_integrator_QP_solver_lpbysp00);
double_integrator_QP_solver_LA_INEQ_B_GRAD_18_6_6(double_integrator_QP_solver_lub01, double_integrator_QP_solver_sub01, double_integrator_QP_solver_riub01, double_integrator_QP_solver_llb01, double_integrator_QP_solver_slb01, double_integrator_QP_solver_rilb01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_grad_ineq01, double_integrator_QP_solver_lubbysub01, double_integrator_QP_solver_llbbyslb01);
double_integrator_QP_solver_LA_INEQ_P_24_18(params->A2, double_integrator_QP_solver_lp01, double_integrator_QP_solver_sp01, double_integrator_QP_solver_rip01, double_integrator_QP_solver_grad_ineq01, double_integrator_QP_solver_lpbysp01);
double_integrator_QP_solver_LA_INEQ_B_GRAD_18_6_6(double_integrator_QP_solver_lub02, double_integrator_QP_solver_sub02, double_integrator_QP_solver_riub02, double_integrator_QP_solver_llb02, double_integrator_QP_solver_slb02, double_integrator_QP_solver_rilb02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_grad_ineq02, double_integrator_QP_solver_lubbysub02, double_integrator_QP_solver_llbbyslb02);
double_integrator_QP_solver_LA_INEQ_P_24_18(params->A3, double_integrator_QP_solver_lp02, double_integrator_QP_solver_sp02, double_integrator_QP_solver_rip02, double_integrator_QP_solver_grad_ineq02, double_integrator_QP_solver_lpbysp02);
double_integrator_QP_solver_LA_INEQ_B_GRAD_18_6_6(double_integrator_QP_solver_lub03, double_integrator_QP_solver_sub03, double_integrator_QP_solver_riub03, double_integrator_QP_solver_llb03, double_integrator_QP_solver_slb03, double_integrator_QP_solver_rilb03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_grad_ineq03, double_integrator_QP_solver_lubbysub03, double_integrator_QP_solver_llbbyslb03);
double_integrator_QP_solver_LA_INEQ_P_24_18(params->A4, double_integrator_QP_solver_lp03, double_integrator_QP_solver_sp03, double_integrator_QP_solver_rip03, double_integrator_QP_solver_grad_ineq03, double_integrator_QP_solver_lpbysp03);
double_integrator_QP_solver_LA_INEQ_B_GRAD_18_6_6(double_integrator_QP_solver_lub04, double_integrator_QP_solver_sub04, double_integrator_QP_solver_riub04, double_integrator_QP_solver_llb04, double_integrator_QP_solver_slb04, double_integrator_QP_solver_rilb04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_grad_ineq04, double_integrator_QP_solver_lubbysub04, double_integrator_QP_solver_llbbyslb04);
double_integrator_QP_solver_LA_INEQ_P_24_18(params->A5, double_integrator_QP_solver_lp04, double_integrator_QP_solver_sp04, double_integrator_QP_solver_rip04, double_integrator_QP_solver_grad_ineq04, double_integrator_QP_solver_lpbysp04);
double_integrator_QP_solver_LA_INEQ_B_GRAD_18_6_6(double_integrator_QP_solver_lub05, double_integrator_QP_solver_sub05, double_integrator_QP_solver_riub05, double_integrator_QP_solver_llb05, double_integrator_QP_solver_slb05, double_integrator_QP_solver_rilb05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_grad_ineq05, double_integrator_QP_solver_lubbysub05, double_integrator_QP_solver_llbbyslb05);
double_integrator_QP_solver_LA_INEQ_P_24_18(params->A6, double_integrator_QP_solver_lp05, double_integrator_QP_solver_sp05, double_integrator_QP_solver_rip05, double_integrator_QP_solver_grad_ineq05, double_integrator_QP_solver_lpbysp05);
double_integrator_QP_solver_LA_INEQ_B_GRAD_18_6_6(double_integrator_QP_solver_lub06, double_integrator_QP_solver_sub06, double_integrator_QP_solver_riub06, double_integrator_QP_solver_llb06, double_integrator_QP_solver_slb06, double_integrator_QP_solver_rilb06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_grad_ineq06, double_integrator_QP_solver_lubbysub06, double_integrator_QP_solver_llbbyslb06);
double_integrator_QP_solver_LA_INEQ_P_24_18(params->A7, double_integrator_QP_solver_lp06, double_integrator_QP_solver_sp06, double_integrator_QP_solver_rip06, double_integrator_QP_solver_grad_ineq06, double_integrator_QP_solver_lpbysp06);
double_integrator_QP_solver_LA_INEQ_B_GRAD_18_6_6(double_integrator_QP_solver_lub07, double_integrator_QP_solver_sub07, double_integrator_QP_solver_riub07, double_integrator_QP_solver_llb07, double_integrator_QP_solver_slb07, double_integrator_QP_solver_rilb07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_grad_ineq07, double_integrator_QP_solver_lubbysub07, double_integrator_QP_solver_llbbyslb07);
double_integrator_QP_solver_LA_INEQ_P_24_18(params->A8, double_integrator_QP_solver_lp07, double_integrator_QP_solver_sp07, double_integrator_QP_solver_rip07, double_integrator_QP_solver_grad_ineq07, double_integrator_QP_solver_lpbysp07);
double_integrator_QP_solver_LA_INEQ_B_GRAD_18_6_6(double_integrator_QP_solver_lub08, double_integrator_QP_solver_sub08, double_integrator_QP_solver_riub08, double_integrator_QP_solver_llb08, double_integrator_QP_solver_slb08, double_integrator_QP_solver_rilb08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_grad_ineq08, double_integrator_QP_solver_lubbysub08, double_integrator_QP_solver_llbbyslb08);
double_integrator_QP_solver_LA_INEQ_P_24_18(params->A9, double_integrator_QP_solver_lp08, double_integrator_QP_solver_sp08, double_integrator_QP_solver_rip08, double_integrator_QP_solver_grad_ineq08, double_integrator_QP_solver_lpbysp08);
double_integrator_QP_solver_LA_INEQ_B_GRAD_18_6_6(double_integrator_QP_solver_lub09, double_integrator_QP_solver_sub09, double_integrator_QP_solver_riub09, double_integrator_QP_solver_llb09, double_integrator_QP_solver_slb09, double_integrator_QP_solver_rilb09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_grad_ineq09, double_integrator_QP_solver_lubbysub09, double_integrator_QP_solver_llbbyslb09);
double_integrator_QP_solver_LA_INEQ_P_24_18(params->A10, double_integrator_QP_solver_lp09, double_integrator_QP_solver_sp09, double_integrator_QP_solver_rip09, double_integrator_QP_solver_grad_ineq09, double_integrator_QP_solver_lpbysp09);
double_integrator_QP_solver_LA_INEQ_B_GRAD_16_6_6(double_integrator_QP_solver_lub10, double_integrator_QP_solver_sub10, double_integrator_QP_solver_riub10, double_integrator_QP_solver_llb10, double_integrator_QP_solver_slb10, double_integrator_QP_solver_rilb10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_grad_ineq10, double_integrator_QP_solver_lubbysub10, double_integrator_QP_solver_llbbyslb10);
double_integrator_QP_solver_LA_INEQ_P_24_16(params->A11, double_integrator_QP_solver_lp10, double_integrator_QP_solver_sp10, double_integrator_QP_solver_rip10, double_integrator_QP_solver_grad_ineq10, double_integrator_QP_solver_lpbysp10);
double_integrator_QP_solver_LA_INEQ_B_GRAD_5_4_4(double_integrator_QP_solver_lub11, double_integrator_QP_solver_sub11, double_integrator_QP_solver_riub11, double_integrator_QP_solver_llb11, double_integrator_QP_solver_slb11, double_integrator_QP_solver_rilb11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_grad_ineq11, double_integrator_QP_solver_lubbysub11, double_integrator_QP_solver_llbbyslb11);
double_integrator_QP_solver_LA_INEQ_P_10_5(params->A12, double_integrator_QP_solver_lp11, double_integrator_QP_solver_sp11, double_integrator_QP_solver_rip11, double_integrator_QP_solver_grad_ineq11, double_integrator_QP_solver_lpbysp11);
info->dobj = info->pobj - info->dgap;
info->rdgap = info->pobj ? info->dgap / info->pobj : 1e6;
if( info->rdgap < 0 ) info->rdgap = -info->rdgap;
PRINTTEXT("  %3d  %3.1e  %3.1e  %+6.4e  %+6.4e  %+3.1e  %3.1e  %3.1e\n",info->it, info->res_eq, info->res_ineq, info->pobj, info->dobj, info->dgap, info->rdgap, info->mu);
if( info->mu < double_integrator_QP_solver_SET_ACC_KKTCOMPL
    && (info->rdgap < double_integrator_QP_solver_SET_ACC_RDGAP || info->dgap < double_integrator_QP_solver_SET_ACC_KKTCOMPL)
    && info->res_eq < double_integrator_QP_solver_SET_ACC_RESEQ
    && info->res_ineq < double_integrator_QP_solver_SET_ACC_RESINEQ ){
PRINTTEXT("OPTIMAL (within RESEQ=%2.1e, RESINEQ=%2.1e, (R)DGAP=(%2.1e)%2.1e, MU=%2.1e).\n",double_integrator_QP_solver_SET_ACC_RESEQ, double_integrator_QP_solver_SET_ACC_RESINEQ,double_integrator_QP_solver_SET_ACC_KKTCOMPL,double_integrator_QP_solver_SET_ACC_RDGAP,double_integrator_QP_solver_SET_ACC_KKTCOMPL);
exitcode = double_integrator_QP_solver_OPTIMAL; break; }
if( info->it == double_integrator_QP_solver_SET_MAXIT ){
PRINTTEXT("Maximum number of iterations reached, exiting.\n");
exitcode = double_integrator_QP_solver_MAXITREACHED; break; }
double_integrator_QP_solver_LA_VVADD3_201(double_integrator_QP_solver_grad_cost, double_integrator_QP_solver_grad_eq, double_integrator_QP_solver_grad_ineq, double_integrator_QP_solver_rd);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_18_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_lubbysub00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_Phi00);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_18(params->A1, double_integrator_QP_solver_lpbysp00, double_integrator_QP_solver_Phi00);
double_integrator_QP_solver_LA_DENSE_CHOL2_18(double_integrator_QP_solver_Phi00);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_C00, double_integrator_QP_solver_V00);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_rd00, double_integrator_QP_solver_Lbyrd00);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_18_6_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_lubbysub01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_Phi01);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_18(params->A2, double_integrator_QP_solver_lpbysp01, double_integrator_QP_solver_Phi01);
double_integrator_QP_solver_LA_DENSE_CHOL2_18(double_integrator_QP_solver_Phi01);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_C00, double_integrator_QP_solver_V01);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_D01, double_integrator_QP_solver_W01);
double_integrator_QP_solver_LA_DENSE_MMTM_7_18_7(double_integrator_QP_solver_W01, double_integrator_QP_solver_V01, double_integrator_QP_solver_Ysd01);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_rd01, double_integrator_QP_solver_Lbyrd01);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_18_6_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_lubbysub02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_Phi02);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_18(params->A3, double_integrator_QP_solver_lpbysp02, double_integrator_QP_solver_Phi02);
double_integrator_QP_solver_LA_DENSE_CHOL2_18(double_integrator_QP_solver_Phi02);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_C00, double_integrator_QP_solver_V02);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_D01, double_integrator_QP_solver_W02);
double_integrator_QP_solver_LA_DENSE_MMTM_7_18_7(double_integrator_QP_solver_W02, double_integrator_QP_solver_V02, double_integrator_QP_solver_Ysd02);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_rd02, double_integrator_QP_solver_Lbyrd02);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_18_6_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_lubbysub03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_Phi03);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_18(params->A4, double_integrator_QP_solver_lpbysp03, double_integrator_QP_solver_Phi03);
double_integrator_QP_solver_LA_DENSE_CHOL2_18(double_integrator_QP_solver_Phi03);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_C00, double_integrator_QP_solver_V03);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_D01, double_integrator_QP_solver_W03);
double_integrator_QP_solver_LA_DENSE_MMTM_7_18_7(double_integrator_QP_solver_W03, double_integrator_QP_solver_V03, double_integrator_QP_solver_Ysd03);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_rd03, double_integrator_QP_solver_Lbyrd03);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_18_6_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_lubbysub04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_Phi04);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_18(params->A5, double_integrator_QP_solver_lpbysp04, double_integrator_QP_solver_Phi04);
double_integrator_QP_solver_LA_DENSE_CHOL2_18(double_integrator_QP_solver_Phi04);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_C00, double_integrator_QP_solver_V04);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_D01, double_integrator_QP_solver_W04);
double_integrator_QP_solver_LA_DENSE_MMTM_7_18_7(double_integrator_QP_solver_W04, double_integrator_QP_solver_V04, double_integrator_QP_solver_Ysd04);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_rd04, double_integrator_QP_solver_Lbyrd04);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_18_6_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_lubbysub05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_Phi05);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_18(params->A6, double_integrator_QP_solver_lpbysp05, double_integrator_QP_solver_Phi05);
double_integrator_QP_solver_LA_DENSE_CHOL2_18(double_integrator_QP_solver_Phi05);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_C00, double_integrator_QP_solver_V05);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_D01, double_integrator_QP_solver_W05);
double_integrator_QP_solver_LA_DENSE_MMTM_7_18_7(double_integrator_QP_solver_W05, double_integrator_QP_solver_V05, double_integrator_QP_solver_Ysd05);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_rd05, double_integrator_QP_solver_Lbyrd05);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_18_6_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_lubbysub06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_Phi06);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_18(params->A7, double_integrator_QP_solver_lpbysp06, double_integrator_QP_solver_Phi06);
double_integrator_QP_solver_LA_DENSE_CHOL2_18(double_integrator_QP_solver_Phi06);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_C00, double_integrator_QP_solver_V06);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_D01, double_integrator_QP_solver_W06);
double_integrator_QP_solver_LA_DENSE_MMTM_7_18_7(double_integrator_QP_solver_W06, double_integrator_QP_solver_V06, double_integrator_QP_solver_Ysd06);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_rd06, double_integrator_QP_solver_Lbyrd06);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_18_6_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_lubbysub07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_Phi07);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_18(params->A8, double_integrator_QP_solver_lpbysp07, double_integrator_QP_solver_Phi07);
double_integrator_QP_solver_LA_DENSE_CHOL2_18(double_integrator_QP_solver_Phi07);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_C00, double_integrator_QP_solver_V07);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_D01, double_integrator_QP_solver_W07);
double_integrator_QP_solver_LA_DENSE_MMTM_7_18_7(double_integrator_QP_solver_W07, double_integrator_QP_solver_V07, double_integrator_QP_solver_Ysd07);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_rd07, double_integrator_QP_solver_Lbyrd07);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_18_6_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_lubbysub08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_Phi08);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_18(params->A9, double_integrator_QP_solver_lpbysp08, double_integrator_QP_solver_Phi08);
double_integrator_QP_solver_LA_DENSE_CHOL2_18(double_integrator_QP_solver_Phi08);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_C00, double_integrator_QP_solver_V08);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_D01, double_integrator_QP_solver_W08);
double_integrator_QP_solver_LA_DENSE_MMTM_7_18_7(double_integrator_QP_solver_W08, double_integrator_QP_solver_V08, double_integrator_QP_solver_Ysd08);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_rd08, double_integrator_QP_solver_Lbyrd08);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_18_6_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_lubbysub09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_Phi09);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_18(params->A10, double_integrator_QP_solver_lpbysp09, double_integrator_QP_solver_Phi09);
double_integrator_QP_solver_LA_DENSE_CHOL2_18(double_integrator_QP_solver_Phi09);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_C00, double_integrator_QP_solver_V09);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_18(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_D01, double_integrator_QP_solver_W09);
double_integrator_QP_solver_LA_DENSE_MMTM_7_18_7(double_integrator_QP_solver_W09, double_integrator_QP_solver_V09, double_integrator_QP_solver_Ysd09);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_rd09, double_integrator_QP_solver_Lbyrd09);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_16_6_6(double_integrator_QP_solver_H10, double_integrator_QP_solver_llbbyslb10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_lubbysub10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_Phi10);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_16(params->A11, double_integrator_QP_solver_lpbysp10, double_integrator_QP_solver_Phi10);
double_integrator_QP_solver_LA_DENSE_CHOL2_16(double_integrator_QP_solver_Phi10);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_4_16(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_C10, double_integrator_QP_solver_V10);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_16(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_D10, double_integrator_QP_solver_W10);
double_integrator_QP_solver_LA_DENSE_MMTM_7_16_4(double_integrator_QP_solver_W10, double_integrator_QP_solver_V10, double_integrator_QP_solver_Ysd10);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_16(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_rd10, double_integrator_QP_solver_Lbyrd10);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_5_4_4(double_integrator_QP_solver_H11, double_integrator_QP_solver_llbbyslb11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_lubbysub11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_Phi11);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_10_5(params->A12, double_integrator_QP_solver_lpbysp11, double_integrator_QP_solver_Phi11);
double_integrator_QP_solver_LA_DENSE_CHOL2_5(double_integrator_QP_solver_Phi11);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_4_5(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_D11, double_integrator_QP_solver_W11);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_rd11, double_integrator_QP_solver_Lbyrd11);
double_integrator_QP_solver_LA_DENSE_MMT2_7_18_18(double_integrator_QP_solver_V00, double_integrator_QP_solver_W01, double_integrator_QP_solver_Yd00);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_18_18(double_integrator_QP_solver_V00, double_integrator_QP_solver_Lbyrd00, double_integrator_QP_solver_W01, double_integrator_QP_solver_Lbyrd01, double_integrator_QP_solver_re00, double_integrator_QP_solver_beta00);
double_integrator_QP_solver_LA_DENSE_MMT2_7_18_18(double_integrator_QP_solver_V01, double_integrator_QP_solver_W02, double_integrator_QP_solver_Yd01);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_18_18(double_integrator_QP_solver_V01, double_integrator_QP_solver_Lbyrd01, double_integrator_QP_solver_W02, double_integrator_QP_solver_Lbyrd02, double_integrator_QP_solver_re01, double_integrator_QP_solver_beta01);
double_integrator_QP_solver_LA_DENSE_MMT2_7_18_18(double_integrator_QP_solver_V02, double_integrator_QP_solver_W03, double_integrator_QP_solver_Yd02);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_18_18(double_integrator_QP_solver_V02, double_integrator_QP_solver_Lbyrd02, double_integrator_QP_solver_W03, double_integrator_QP_solver_Lbyrd03, double_integrator_QP_solver_re02, double_integrator_QP_solver_beta02);
double_integrator_QP_solver_LA_DENSE_MMT2_7_18_18(double_integrator_QP_solver_V03, double_integrator_QP_solver_W04, double_integrator_QP_solver_Yd03);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_18_18(double_integrator_QP_solver_V03, double_integrator_QP_solver_Lbyrd03, double_integrator_QP_solver_W04, double_integrator_QP_solver_Lbyrd04, double_integrator_QP_solver_re03, double_integrator_QP_solver_beta03);
double_integrator_QP_solver_LA_DENSE_MMT2_7_18_18(double_integrator_QP_solver_V04, double_integrator_QP_solver_W05, double_integrator_QP_solver_Yd04);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_18_18(double_integrator_QP_solver_V04, double_integrator_QP_solver_Lbyrd04, double_integrator_QP_solver_W05, double_integrator_QP_solver_Lbyrd05, double_integrator_QP_solver_re04, double_integrator_QP_solver_beta04);
double_integrator_QP_solver_LA_DENSE_MMT2_7_18_18(double_integrator_QP_solver_V05, double_integrator_QP_solver_W06, double_integrator_QP_solver_Yd05);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_18_18(double_integrator_QP_solver_V05, double_integrator_QP_solver_Lbyrd05, double_integrator_QP_solver_W06, double_integrator_QP_solver_Lbyrd06, double_integrator_QP_solver_re05, double_integrator_QP_solver_beta05);
double_integrator_QP_solver_LA_DENSE_MMT2_7_18_18(double_integrator_QP_solver_V06, double_integrator_QP_solver_W07, double_integrator_QP_solver_Yd06);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_18_18(double_integrator_QP_solver_V06, double_integrator_QP_solver_Lbyrd06, double_integrator_QP_solver_W07, double_integrator_QP_solver_Lbyrd07, double_integrator_QP_solver_re06, double_integrator_QP_solver_beta06);
double_integrator_QP_solver_LA_DENSE_MMT2_7_18_18(double_integrator_QP_solver_V07, double_integrator_QP_solver_W08, double_integrator_QP_solver_Yd07);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_18_18(double_integrator_QP_solver_V07, double_integrator_QP_solver_Lbyrd07, double_integrator_QP_solver_W08, double_integrator_QP_solver_Lbyrd08, double_integrator_QP_solver_re07, double_integrator_QP_solver_beta07);
double_integrator_QP_solver_LA_DENSE_MMT2_7_18_18(double_integrator_QP_solver_V08, double_integrator_QP_solver_W09, double_integrator_QP_solver_Yd08);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_18_18(double_integrator_QP_solver_V08, double_integrator_QP_solver_Lbyrd08, double_integrator_QP_solver_W09, double_integrator_QP_solver_Lbyrd09, double_integrator_QP_solver_re08, double_integrator_QP_solver_beta08);
double_integrator_QP_solver_LA_DENSE_MMT2_7_18_16(double_integrator_QP_solver_V09, double_integrator_QP_solver_W10, double_integrator_QP_solver_Yd09);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_18_16(double_integrator_QP_solver_V09, double_integrator_QP_solver_Lbyrd09, double_integrator_QP_solver_W10, double_integrator_QP_solver_Lbyrd10, double_integrator_QP_solver_re09, double_integrator_QP_solver_beta09);
double_integrator_QP_solver_LA_DENSE_MMT2_4_16_5(double_integrator_QP_solver_V10, double_integrator_QP_solver_W11, double_integrator_QP_solver_Yd10);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_4_16_5(double_integrator_QP_solver_V10, double_integrator_QP_solver_Lbyrd10, double_integrator_QP_solver_W11, double_integrator_QP_solver_Lbyrd11, double_integrator_QP_solver_re10, double_integrator_QP_solver_beta10);
double_integrator_QP_solver_LA_DENSE_CHOL_7(double_integrator_QP_solver_Yd00, double_integrator_QP_solver_Ld00);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld00, double_integrator_QP_solver_beta00, double_integrator_QP_solver_yy00);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_Ld00, double_integrator_QP_solver_Ysd01, double_integrator_QP_solver_Lsd01);
double_integrator_QP_solver_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_Lsd01, double_integrator_QP_solver_Yd01);
double_integrator_QP_solver_LA_DENSE_CHOL_7(double_integrator_QP_solver_Yd01, double_integrator_QP_solver_Ld01);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd01, double_integrator_QP_solver_yy00, double_integrator_QP_solver_beta01, double_integrator_QP_solver_bmy01);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld01, double_integrator_QP_solver_bmy01, double_integrator_QP_solver_yy01);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_Ld01, double_integrator_QP_solver_Ysd02, double_integrator_QP_solver_Lsd02);
double_integrator_QP_solver_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_Lsd02, double_integrator_QP_solver_Yd02);
double_integrator_QP_solver_LA_DENSE_CHOL_7(double_integrator_QP_solver_Yd02, double_integrator_QP_solver_Ld02);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd02, double_integrator_QP_solver_yy01, double_integrator_QP_solver_beta02, double_integrator_QP_solver_bmy02);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld02, double_integrator_QP_solver_bmy02, double_integrator_QP_solver_yy02);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_Ld02, double_integrator_QP_solver_Ysd03, double_integrator_QP_solver_Lsd03);
double_integrator_QP_solver_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_Lsd03, double_integrator_QP_solver_Yd03);
double_integrator_QP_solver_LA_DENSE_CHOL_7(double_integrator_QP_solver_Yd03, double_integrator_QP_solver_Ld03);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd03, double_integrator_QP_solver_yy02, double_integrator_QP_solver_beta03, double_integrator_QP_solver_bmy03);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld03, double_integrator_QP_solver_bmy03, double_integrator_QP_solver_yy03);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_Ld03, double_integrator_QP_solver_Ysd04, double_integrator_QP_solver_Lsd04);
double_integrator_QP_solver_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_Lsd04, double_integrator_QP_solver_Yd04);
double_integrator_QP_solver_LA_DENSE_CHOL_7(double_integrator_QP_solver_Yd04, double_integrator_QP_solver_Ld04);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd04, double_integrator_QP_solver_yy03, double_integrator_QP_solver_beta04, double_integrator_QP_solver_bmy04);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld04, double_integrator_QP_solver_bmy04, double_integrator_QP_solver_yy04);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_Ld04, double_integrator_QP_solver_Ysd05, double_integrator_QP_solver_Lsd05);
double_integrator_QP_solver_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_Lsd05, double_integrator_QP_solver_Yd05);
double_integrator_QP_solver_LA_DENSE_CHOL_7(double_integrator_QP_solver_Yd05, double_integrator_QP_solver_Ld05);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd05, double_integrator_QP_solver_yy04, double_integrator_QP_solver_beta05, double_integrator_QP_solver_bmy05);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld05, double_integrator_QP_solver_bmy05, double_integrator_QP_solver_yy05);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_Ld05, double_integrator_QP_solver_Ysd06, double_integrator_QP_solver_Lsd06);
double_integrator_QP_solver_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_Lsd06, double_integrator_QP_solver_Yd06);
double_integrator_QP_solver_LA_DENSE_CHOL_7(double_integrator_QP_solver_Yd06, double_integrator_QP_solver_Ld06);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd06, double_integrator_QP_solver_yy05, double_integrator_QP_solver_beta06, double_integrator_QP_solver_bmy06);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld06, double_integrator_QP_solver_bmy06, double_integrator_QP_solver_yy06);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_Ld06, double_integrator_QP_solver_Ysd07, double_integrator_QP_solver_Lsd07);
double_integrator_QP_solver_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_Lsd07, double_integrator_QP_solver_Yd07);
double_integrator_QP_solver_LA_DENSE_CHOL_7(double_integrator_QP_solver_Yd07, double_integrator_QP_solver_Ld07);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd07, double_integrator_QP_solver_yy06, double_integrator_QP_solver_beta07, double_integrator_QP_solver_bmy07);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld07, double_integrator_QP_solver_bmy07, double_integrator_QP_solver_yy07);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_Ld07, double_integrator_QP_solver_Ysd08, double_integrator_QP_solver_Lsd08);
double_integrator_QP_solver_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_Lsd08, double_integrator_QP_solver_Yd08);
double_integrator_QP_solver_LA_DENSE_CHOL_7(double_integrator_QP_solver_Yd08, double_integrator_QP_solver_Ld08);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd08, double_integrator_QP_solver_yy07, double_integrator_QP_solver_beta08, double_integrator_QP_solver_bmy08);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld08, double_integrator_QP_solver_bmy08, double_integrator_QP_solver_yy08);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_Ld08, double_integrator_QP_solver_Ysd09, double_integrator_QP_solver_Lsd09);
double_integrator_QP_solver_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_Lsd09, double_integrator_QP_solver_Yd09);
double_integrator_QP_solver_LA_DENSE_CHOL_7(double_integrator_QP_solver_Yd09, double_integrator_QP_solver_Ld09);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd09, double_integrator_QP_solver_yy08, double_integrator_QP_solver_beta09, double_integrator_QP_solver_bmy09);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld09, double_integrator_QP_solver_bmy09, double_integrator_QP_solver_yy09);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_4_7(double_integrator_QP_solver_Ld09, double_integrator_QP_solver_Ysd10, double_integrator_QP_solver_Lsd10);
double_integrator_QP_solver_LA_DENSE_MMTSUB_4_7(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_Yd10);
double_integrator_QP_solver_LA_DENSE_CHOL_4(double_integrator_QP_solver_Yd10, double_integrator_QP_solver_Ld10);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_4_7(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_yy09, double_integrator_QP_solver_beta10, double_integrator_QP_solver_bmy10);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_4(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_bmy10, double_integrator_QP_solver_yy10);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_4(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_yy10, double_integrator_QP_solver_dvaff10);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_4_7(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_dvaff10, double_integrator_QP_solver_yy09, double_integrator_QP_solver_bmy09);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld09, double_integrator_QP_solver_bmy09, double_integrator_QP_solver_dvaff09);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd09, double_integrator_QP_solver_dvaff09, double_integrator_QP_solver_yy08, double_integrator_QP_solver_bmy08);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld08, double_integrator_QP_solver_bmy08, double_integrator_QP_solver_dvaff08);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd08, double_integrator_QP_solver_dvaff08, double_integrator_QP_solver_yy07, double_integrator_QP_solver_bmy07);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld07, double_integrator_QP_solver_bmy07, double_integrator_QP_solver_dvaff07);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd07, double_integrator_QP_solver_dvaff07, double_integrator_QP_solver_yy06, double_integrator_QP_solver_bmy06);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld06, double_integrator_QP_solver_bmy06, double_integrator_QP_solver_dvaff06);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd06, double_integrator_QP_solver_dvaff06, double_integrator_QP_solver_yy05, double_integrator_QP_solver_bmy05);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld05, double_integrator_QP_solver_bmy05, double_integrator_QP_solver_dvaff05);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd05, double_integrator_QP_solver_dvaff05, double_integrator_QP_solver_yy04, double_integrator_QP_solver_bmy04);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld04, double_integrator_QP_solver_bmy04, double_integrator_QP_solver_dvaff04);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd04, double_integrator_QP_solver_dvaff04, double_integrator_QP_solver_yy03, double_integrator_QP_solver_bmy03);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld03, double_integrator_QP_solver_bmy03, double_integrator_QP_solver_dvaff03);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd03, double_integrator_QP_solver_dvaff03, double_integrator_QP_solver_yy02, double_integrator_QP_solver_bmy02);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld02, double_integrator_QP_solver_bmy02, double_integrator_QP_solver_dvaff02);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd02, double_integrator_QP_solver_dvaff02, double_integrator_QP_solver_yy01, double_integrator_QP_solver_bmy01);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld01, double_integrator_QP_solver_bmy01, double_integrator_QP_solver_dvaff01);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd01, double_integrator_QP_solver_dvaff01, double_integrator_QP_solver_yy00, double_integrator_QP_solver_bmy00);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld00, double_integrator_QP_solver_bmy00, double_integrator_QP_solver_dvaff00);
double_integrator_QP_solver_LA_DENSE_MTVM_7_18(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff00, double_integrator_QP_solver_grad_eq00);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff01, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff00, double_integrator_QP_solver_grad_eq01);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff02, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff01, double_integrator_QP_solver_grad_eq02);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff03, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff02, double_integrator_QP_solver_grad_eq03);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff04, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff03, double_integrator_QP_solver_grad_eq04);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff05, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff04, double_integrator_QP_solver_grad_eq05);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff06, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff05, double_integrator_QP_solver_grad_eq06);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff07, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff06, double_integrator_QP_solver_grad_eq07);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff08, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff07, double_integrator_QP_solver_grad_eq08);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff09, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff08, double_integrator_QP_solver_grad_eq09);
double_integrator_QP_solver_LA_DENSE_MTVM2_4_16_7(double_integrator_QP_solver_C10, double_integrator_QP_solver_dvaff10, double_integrator_QP_solver_D10, double_integrator_QP_solver_dvaff09, double_integrator_QP_solver_grad_eq10);
double_integrator_QP_solver_LA_DIAGZERO_MTVM_4_5(double_integrator_QP_solver_D11, double_integrator_QP_solver_dvaff10, double_integrator_QP_solver_grad_eq11);
double_integrator_QP_solver_LA_VSUB2_201(double_integrator_QP_solver_rd, double_integrator_QP_solver_grad_eq, double_integrator_QP_solver_rd);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_rd00, double_integrator_QP_solver_dzaff00);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_rd01, double_integrator_QP_solver_dzaff01);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_rd02, double_integrator_QP_solver_dzaff02);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_rd03, double_integrator_QP_solver_dzaff03);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_rd04, double_integrator_QP_solver_dzaff04);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_rd05, double_integrator_QP_solver_dzaff05);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_rd06, double_integrator_QP_solver_dzaff06);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_rd07, double_integrator_QP_solver_dzaff07);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_rd08, double_integrator_QP_solver_dzaff08);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_rd09, double_integrator_QP_solver_dzaff09);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_16(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_rd10, double_integrator_QP_solver_dzaff10);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_5(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_rd11, double_integrator_QP_solver_dzaff11);
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_rilb00, double_integrator_QP_solver_dslbaff00);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb00, double_integrator_QP_solver_dslbaff00, double_integrator_QP_solver_llb00, double_integrator_QP_solver_dllbaff00);
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub00, double_integrator_QP_solver_dzaff00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_dsubaff00);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub00, double_integrator_QP_solver_dsubaff00, double_integrator_QP_solver_lub00, double_integrator_QP_solver_dlubaff00);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_18(params->A1, double_integrator_QP_solver_dzaff00, double_integrator_QP_solver_rip00, double_integrator_QP_solver_dsp_aff00);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp00, double_integrator_QP_solver_dsp_aff00, double_integrator_QP_solver_lp00, double_integrator_QP_solver_dlp_aff00);
double_integrator_QP_solver_LA_VSUB_INDEXED_6(double_integrator_QP_solver_dzaff01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_rilb01, double_integrator_QP_solver_dslbaff01);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_llbbyslb01, double_integrator_QP_solver_dslbaff01, double_integrator_QP_solver_llb01, double_integrator_QP_solver_dllbaff01);
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub01, double_integrator_QP_solver_dzaff01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_dsubaff01);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub01, double_integrator_QP_solver_dsubaff01, double_integrator_QP_solver_lub01, double_integrator_QP_solver_dlubaff01);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_18(params->A2, double_integrator_QP_solver_dzaff01, double_integrator_QP_solver_rip01, double_integrator_QP_solver_dsp_aff01);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp01, double_integrator_QP_solver_dsp_aff01, double_integrator_QP_solver_lp01, double_integrator_QP_solver_dlp_aff01);
double_integrator_QP_solver_LA_VSUB_INDEXED_6(double_integrator_QP_solver_dzaff02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_rilb02, double_integrator_QP_solver_dslbaff02);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_llbbyslb02, double_integrator_QP_solver_dslbaff02, double_integrator_QP_solver_llb02, double_integrator_QP_solver_dllbaff02);
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub02, double_integrator_QP_solver_dzaff02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_dsubaff02);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub02, double_integrator_QP_solver_dsubaff02, double_integrator_QP_solver_lub02, double_integrator_QP_solver_dlubaff02);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_18(params->A3, double_integrator_QP_solver_dzaff02, double_integrator_QP_solver_rip02, double_integrator_QP_solver_dsp_aff02);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp02, double_integrator_QP_solver_dsp_aff02, double_integrator_QP_solver_lp02, double_integrator_QP_solver_dlp_aff02);
double_integrator_QP_solver_LA_VSUB_INDEXED_6(double_integrator_QP_solver_dzaff03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_rilb03, double_integrator_QP_solver_dslbaff03);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_llbbyslb03, double_integrator_QP_solver_dslbaff03, double_integrator_QP_solver_llb03, double_integrator_QP_solver_dllbaff03);
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub03, double_integrator_QP_solver_dzaff03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_dsubaff03);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub03, double_integrator_QP_solver_dsubaff03, double_integrator_QP_solver_lub03, double_integrator_QP_solver_dlubaff03);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_18(params->A4, double_integrator_QP_solver_dzaff03, double_integrator_QP_solver_rip03, double_integrator_QP_solver_dsp_aff03);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp03, double_integrator_QP_solver_dsp_aff03, double_integrator_QP_solver_lp03, double_integrator_QP_solver_dlp_aff03);
double_integrator_QP_solver_LA_VSUB_INDEXED_6(double_integrator_QP_solver_dzaff04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_rilb04, double_integrator_QP_solver_dslbaff04);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_llbbyslb04, double_integrator_QP_solver_dslbaff04, double_integrator_QP_solver_llb04, double_integrator_QP_solver_dllbaff04);
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub04, double_integrator_QP_solver_dzaff04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_dsubaff04);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub04, double_integrator_QP_solver_dsubaff04, double_integrator_QP_solver_lub04, double_integrator_QP_solver_dlubaff04);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_18(params->A5, double_integrator_QP_solver_dzaff04, double_integrator_QP_solver_rip04, double_integrator_QP_solver_dsp_aff04);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp04, double_integrator_QP_solver_dsp_aff04, double_integrator_QP_solver_lp04, double_integrator_QP_solver_dlp_aff04);
double_integrator_QP_solver_LA_VSUB_INDEXED_6(double_integrator_QP_solver_dzaff05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_rilb05, double_integrator_QP_solver_dslbaff05);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_llbbyslb05, double_integrator_QP_solver_dslbaff05, double_integrator_QP_solver_llb05, double_integrator_QP_solver_dllbaff05);
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub05, double_integrator_QP_solver_dzaff05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_dsubaff05);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub05, double_integrator_QP_solver_dsubaff05, double_integrator_QP_solver_lub05, double_integrator_QP_solver_dlubaff05);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_18(params->A6, double_integrator_QP_solver_dzaff05, double_integrator_QP_solver_rip05, double_integrator_QP_solver_dsp_aff05);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp05, double_integrator_QP_solver_dsp_aff05, double_integrator_QP_solver_lp05, double_integrator_QP_solver_dlp_aff05);
double_integrator_QP_solver_LA_VSUB_INDEXED_6(double_integrator_QP_solver_dzaff06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_rilb06, double_integrator_QP_solver_dslbaff06);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_llbbyslb06, double_integrator_QP_solver_dslbaff06, double_integrator_QP_solver_llb06, double_integrator_QP_solver_dllbaff06);
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub06, double_integrator_QP_solver_dzaff06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_dsubaff06);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub06, double_integrator_QP_solver_dsubaff06, double_integrator_QP_solver_lub06, double_integrator_QP_solver_dlubaff06);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_18(params->A7, double_integrator_QP_solver_dzaff06, double_integrator_QP_solver_rip06, double_integrator_QP_solver_dsp_aff06);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp06, double_integrator_QP_solver_dsp_aff06, double_integrator_QP_solver_lp06, double_integrator_QP_solver_dlp_aff06);
double_integrator_QP_solver_LA_VSUB_INDEXED_6(double_integrator_QP_solver_dzaff07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_rilb07, double_integrator_QP_solver_dslbaff07);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_llbbyslb07, double_integrator_QP_solver_dslbaff07, double_integrator_QP_solver_llb07, double_integrator_QP_solver_dllbaff07);
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub07, double_integrator_QP_solver_dzaff07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_dsubaff07);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub07, double_integrator_QP_solver_dsubaff07, double_integrator_QP_solver_lub07, double_integrator_QP_solver_dlubaff07);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_18(params->A8, double_integrator_QP_solver_dzaff07, double_integrator_QP_solver_rip07, double_integrator_QP_solver_dsp_aff07);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp07, double_integrator_QP_solver_dsp_aff07, double_integrator_QP_solver_lp07, double_integrator_QP_solver_dlp_aff07);
double_integrator_QP_solver_LA_VSUB_INDEXED_6(double_integrator_QP_solver_dzaff08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_rilb08, double_integrator_QP_solver_dslbaff08);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_llbbyslb08, double_integrator_QP_solver_dslbaff08, double_integrator_QP_solver_llb08, double_integrator_QP_solver_dllbaff08);
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub08, double_integrator_QP_solver_dzaff08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_dsubaff08);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub08, double_integrator_QP_solver_dsubaff08, double_integrator_QP_solver_lub08, double_integrator_QP_solver_dlubaff08);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_18(params->A9, double_integrator_QP_solver_dzaff08, double_integrator_QP_solver_rip08, double_integrator_QP_solver_dsp_aff08);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp08, double_integrator_QP_solver_dsp_aff08, double_integrator_QP_solver_lp08, double_integrator_QP_solver_dlp_aff08);
double_integrator_QP_solver_LA_VSUB_INDEXED_6(double_integrator_QP_solver_dzaff09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_rilb09, double_integrator_QP_solver_dslbaff09);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_llbbyslb09, double_integrator_QP_solver_dslbaff09, double_integrator_QP_solver_llb09, double_integrator_QP_solver_dllbaff09);
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub09, double_integrator_QP_solver_dzaff09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_dsubaff09);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub09, double_integrator_QP_solver_dsubaff09, double_integrator_QP_solver_lub09, double_integrator_QP_solver_dlubaff09);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_18(params->A10, double_integrator_QP_solver_dzaff09, double_integrator_QP_solver_rip09, double_integrator_QP_solver_dsp_aff09);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp09, double_integrator_QP_solver_dsp_aff09, double_integrator_QP_solver_lp09, double_integrator_QP_solver_dlp_aff09);
double_integrator_QP_solver_LA_VSUB_INDEXED_6(double_integrator_QP_solver_dzaff10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_rilb10, double_integrator_QP_solver_dslbaff10);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_llbbyslb10, double_integrator_QP_solver_dslbaff10, double_integrator_QP_solver_llb10, double_integrator_QP_solver_dllbaff10);
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub10, double_integrator_QP_solver_dzaff10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_dsubaff10);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub10, double_integrator_QP_solver_dsubaff10, double_integrator_QP_solver_lub10, double_integrator_QP_solver_dlubaff10);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_16(params->A11, double_integrator_QP_solver_dzaff10, double_integrator_QP_solver_rip10, double_integrator_QP_solver_dsp_aff10);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp10, double_integrator_QP_solver_dsp_aff10, double_integrator_QP_solver_lp10, double_integrator_QP_solver_dlp_aff10);
double_integrator_QP_solver_LA_VSUB_INDEXED_4(double_integrator_QP_solver_dzaff11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_rilb11, double_integrator_QP_solver_dslbaff11);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_llbbyslb11, double_integrator_QP_solver_dslbaff11, double_integrator_QP_solver_llb11, double_integrator_QP_solver_dllbaff11);
double_integrator_QP_solver_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_riub11, double_integrator_QP_solver_dzaff11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_dsubaff11);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_lubbysub11, double_integrator_QP_solver_dsubaff11, double_integrator_QP_solver_lub11, double_integrator_QP_solver_dlubaff11);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_10_5(params->A12, double_integrator_QP_solver_dzaff11, double_integrator_QP_solver_rip11, double_integrator_QP_solver_dsp_aff11);
double_integrator_QP_solver_LA_VSUB3_10(double_integrator_QP_solver_lpbysp11, double_integrator_QP_solver_dsp_aff11, double_integrator_QP_solver_lp11, double_integrator_QP_solver_dlp_aff11);
info->lsit_aff = double_integrator_QP_solver_LINESEARCH_BACKTRACKING_AFFINE(double_integrator_QP_solver_l, double_integrator_QP_solver_s, double_integrator_QP_solver_dl_aff, double_integrator_QP_solver_ds_aff, &info->step_aff, &info->mu_aff);
if( info->lsit_aff == double_integrator_QP_solver_NOPROGRESS ){
PRINTTEXT("Affine line search could not proceed at iteration %d.\nThe problem might be infeasible -- exiting.\n",info->it+1);
exitcode = double_integrator_QP_solver_NOPROGRESS; break;
}
sigma_3rdroot = info->mu_aff / info->mu;
info->sigma = sigma_3rdroot*sigma_3rdroot*sigma_3rdroot;
musigma = info->mu * info->sigma;
double_integrator_QP_solver_LA_VSUB5_415(double_integrator_QP_solver_ds_aff, double_integrator_QP_solver_dl_aff, info->mu, info->sigma, double_integrator_QP_solver_ccrhs);
double_integrator_QP_solver_LA_VSUB6_INDEXED_18_6_7(double_integrator_QP_solver_ccrhsub00, double_integrator_QP_solver_sub00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_ccrhsl00, double_integrator_QP_solver_slb00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_rd00);
double_integrator_QP_solver_LA_VSUB6_INDEXED_18_6_6(double_integrator_QP_solver_ccrhsub01, double_integrator_QP_solver_sub01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_ccrhsl01, double_integrator_QP_solver_slb01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_rd01);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_18(params->A1, double_integrator_QP_solver_ccrhsp00, double_integrator_QP_solver_sp00, double_integrator_QP_solver_rd00);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_18(params->A2, double_integrator_QP_solver_ccrhsp01, double_integrator_QP_solver_sp01, double_integrator_QP_solver_rd01);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_rd00, double_integrator_QP_solver_Lbyrd00);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_rd01, double_integrator_QP_solver_Lbyrd01);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_18_18(double_integrator_QP_solver_V00, double_integrator_QP_solver_Lbyrd00, double_integrator_QP_solver_W01, double_integrator_QP_solver_Lbyrd01, double_integrator_QP_solver_beta00);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld00, double_integrator_QP_solver_beta00, double_integrator_QP_solver_yy00);
double_integrator_QP_solver_LA_VSUB6_INDEXED_18_6_6(double_integrator_QP_solver_ccrhsub02, double_integrator_QP_solver_sub02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_ccrhsl02, double_integrator_QP_solver_slb02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_rd02);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_18(params->A3, double_integrator_QP_solver_ccrhsp02, double_integrator_QP_solver_sp02, double_integrator_QP_solver_rd02);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_rd02, double_integrator_QP_solver_Lbyrd02);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_18_18(double_integrator_QP_solver_V01, double_integrator_QP_solver_Lbyrd01, double_integrator_QP_solver_W02, double_integrator_QP_solver_Lbyrd02, double_integrator_QP_solver_beta01);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd01, double_integrator_QP_solver_yy00, double_integrator_QP_solver_beta01, double_integrator_QP_solver_bmy01);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld01, double_integrator_QP_solver_bmy01, double_integrator_QP_solver_yy01);
double_integrator_QP_solver_LA_VSUB6_INDEXED_18_6_6(double_integrator_QP_solver_ccrhsub03, double_integrator_QP_solver_sub03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_ccrhsl03, double_integrator_QP_solver_slb03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_rd03);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_18(params->A4, double_integrator_QP_solver_ccrhsp03, double_integrator_QP_solver_sp03, double_integrator_QP_solver_rd03);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_rd03, double_integrator_QP_solver_Lbyrd03);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_18_18(double_integrator_QP_solver_V02, double_integrator_QP_solver_Lbyrd02, double_integrator_QP_solver_W03, double_integrator_QP_solver_Lbyrd03, double_integrator_QP_solver_beta02);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd02, double_integrator_QP_solver_yy01, double_integrator_QP_solver_beta02, double_integrator_QP_solver_bmy02);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld02, double_integrator_QP_solver_bmy02, double_integrator_QP_solver_yy02);
double_integrator_QP_solver_LA_VSUB6_INDEXED_18_6_6(double_integrator_QP_solver_ccrhsub04, double_integrator_QP_solver_sub04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_ccrhsl04, double_integrator_QP_solver_slb04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_rd04);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_18(params->A5, double_integrator_QP_solver_ccrhsp04, double_integrator_QP_solver_sp04, double_integrator_QP_solver_rd04);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_rd04, double_integrator_QP_solver_Lbyrd04);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_18_18(double_integrator_QP_solver_V03, double_integrator_QP_solver_Lbyrd03, double_integrator_QP_solver_W04, double_integrator_QP_solver_Lbyrd04, double_integrator_QP_solver_beta03);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd03, double_integrator_QP_solver_yy02, double_integrator_QP_solver_beta03, double_integrator_QP_solver_bmy03);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld03, double_integrator_QP_solver_bmy03, double_integrator_QP_solver_yy03);
double_integrator_QP_solver_LA_VSUB6_INDEXED_18_6_6(double_integrator_QP_solver_ccrhsub05, double_integrator_QP_solver_sub05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_ccrhsl05, double_integrator_QP_solver_slb05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_rd05);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_18(params->A6, double_integrator_QP_solver_ccrhsp05, double_integrator_QP_solver_sp05, double_integrator_QP_solver_rd05);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_rd05, double_integrator_QP_solver_Lbyrd05);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_18_18(double_integrator_QP_solver_V04, double_integrator_QP_solver_Lbyrd04, double_integrator_QP_solver_W05, double_integrator_QP_solver_Lbyrd05, double_integrator_QP_solver_beta04);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd04, double_integrator_QP_solver_yy03, double_integrator_QP_solver_beta04, double_integrator_QP_solver_bmy04);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld04, double_integrator_QP_solver_bmy04, double_integrator_QP_solver_yy04);
double_integrator_QP_solver_LA_VSUB6_INDEXED_18_6_6(double_integrator_QP_solver_ccrhsub06, double_integrator_QP_solver_sub06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_ccrhsl06, double_integrator_QP_solver_slb06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_rd06);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_18(params->A7, double_integrator_QP_solver_ccrhsp06, double_integrator_QP_solver_sp06, double_integrator_QP_solver_rd06);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_rd06, double_integrator_QP_solver_Lbyrd06);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_18_18(double_integrator_QP_solver_V05, double_integrator_QP_solver_Lbyrd05, double_integrator_QP_solver_W06, double_integrator_QP_solver_Lbyrd06, double_integrator_QP_solver_beta05);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd05, double_integrator_QP_solver_yy04, double_integrator_QP_solver_beta05, double_integrator_QP_solver_bmy05);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld05, double_integrator_QP_solver_bmy05, double_integrator_QP_solver_yy05);
double_integrator_QP_solver_LA_VSUB6_INDEXED_18_6_6(double_integrator_QP_solver_ccrhsub07, double_integrator_QP_solver_sub07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_ccrhsl07, double_integrator_QP_solver_slb07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_rd07);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_18(params->A8, double_integrator_QP_solver_ccrhsp07, double_integrator_QP_solver_sp07, double_integrator_QP_solver_rd07);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_rd07, double_integrator_QP_solver_Lbyrd07);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_18_18(double_integrator_QP_solver_V06, double_integrator_QP_solver_Lbyrd06, double_integrator_QP_solver_W07, double_integrator_QP_solver_Lbyrd07, double_integrator_QP_solver_beta06);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd06, double_integrator_QP_solver_yy05, double_integrator_QP_solver_beta06, double_integrator_QP_solver_bmy06);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld06, double_integrator_QP_solver_bmy06, double_integrator_QP_solver_yy06);
double_integrator_QP_solver_LA_VSUB6_INDEXED_18_6_6(double_integrator_QP_solver_ccrhsub08, double_integrator_QP_solver_sub08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_ccrhsl08, double_integrator_QP_solver_slb08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_rd08);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_18(params->A9, double_integrator_QP_solver_ccrhsp08, double_integrator_QP_solver_sp08, double_integrator_QP_solver_rd08);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_rd08, double_integrator_QP_solver_Lbyrd08);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_18_18(double_integrator_QP_solver_V07, double_integrator_QP_solver_Lbyrd07, double_integrator_QP_solver_W08, double_integrator_QP_solver_Lbyrd08, double_integrator_QP_solver_beta07);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd07, double_integrator_QP_solver_yy06, double_integrator_QP_solver_beta07, double_integrator_QP_solver_bmy07);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld07, double_integrator_QP_solver_bmy07, double_integrator_QP_solver_yy07);
double_integrator_QP_solver_LA_VSUB6_INDEXED_18_6_6(double_integrator_QP_solver_ccrhsub09, double_integrator_QP_solver_sub09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_ccrhsl09, double_integrator_QP_solver_slb09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_rd09);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_18(params->A10, double_integrator_QP_solver_ccrhsp09, double_integrator_QP_solver_sp09, double_integrator_QP_solver_rd09);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_18(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_rd09, double_integrator_QP_solver_Lbyrd09);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_18_18(double_integrator_QP_solver_V08, double_integrator_QP_solver_Lbyrd08, double_integrator_QP_solver_W09, double_integrator_QP_solver_Lbyrd09, double_integrator_QP_solver_beta08);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd08, double_integrator_QP_solver_yy07, double_integrator_QP_solver_beta08, double_integrator_QP_solver_bmy08);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld08, double_integrator_QP_solver_bmy08, double_integrator_QP_solver_yy08);
double_integrator_QP_solver_LA_VSUB6_INDEXED_16_6_6(double_integrator_QP_solver_ccrhsub10, double_integrator_QP_solver_sub10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_ccrhsl10, double_integrator_QP_solver_slb10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_rd10);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_16(params->A11, double_integrator_QP_solver_ccrhsp10, double_integrator_QP_solver_sp10, double_integrator_QP_solver_rd10);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_16(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_rd10, double_integrator_QP_solver_Lbyrd10);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_18_16(double_integrator_QP_solver_V09, double_integrator_QP_solver_Lbyrd09, double_integrator_QP_solver_W10, double_integrator_QP_solver_Lbyrd10, double_integrator_QP_solver_beta09);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd09, double_integrator_QP_solver_yy08, double_integrator_QP_solver_beta09, double_integrator_QP_solver_bmy09);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld09, double_integrator_QP_solver_bmy09, double_integrator_QP_solver_yy09);
double_integrator_QP_solver_LA_VSUB6_INDEXED_5_4_4(double_integrator_QP_solver_ccrhsub11, double_integrator_QP_solver_sub11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_ccrhsl11, double_integrator_QP_solver_slb11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_rd11);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_10_5(params->A12, double_integrator_QP_solver_ccrhsp11, double_integrator_QP_solver_sp11, double_integrator_QP_solver_rd11);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_rd11, double_integrator_QP_solver_Lbyrd11);
double_integrator_QP_solver_LA_DENSE_2MVMADD_4_16_5(double_integrator_QP_solver_V10, double_integrator_QP_solver_Lbyrd10, double_integrator_QP_solver_W11, double_integrator_QP_solver_Lbyrd11, double_integrator_QP_solver_beta10);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_4_7(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_yy09, double_integrator_QP_solver_beta10, double_integrator_QP_solver_bmy10);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_4(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_bmy10, double_integrator_QP_solver_yy10);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_4(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_yy10, double_integrator_QP_solver_dvcc10);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_4_7(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_dvcc10, double_integrator_QP_solver_yy09, double_integrator_QP_solver_bmy09);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld09, double_integrator_QP_solver_bmy09, double_integrator_QP_solver_dvcc09);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd09, double_integrator_QP_solver_dvcc09, double_integrator_QP_solver_yy08, double_integrator_QP_solver_bmy08);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld08, double_integrator_QP_solver_bmy08, double_integrator_QP_solver_dvcc08);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd08, double_integrator_QP_solver_dvcc08, double_integrator_QP_solver_yy07, double_integrator_QP_solver_bmy07);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld07, double_integrator_QP_solver_bmy07, double_integrator_QP_solver_dvcc07);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd07, double_integrator_QP_solver_dvcc07, double_integrator_QP_solver_yy06, double_integrator_QP_solver_bmy06);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld06, double_integrator_QP_solver_bmy06, double_integrator_QP_solver_dvcc06);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd06, double_integrator_QP_solver_dvcc06, double_integrator_QP_solver_yy05, double_integrator_QP_solver_bmy05);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld05, double_integrator_QP_solver_bmy05, double_integrator_QP_solver_dvcc05);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd05, double_integrator_QP_solver_dvcc05, double_integrator_QP_solver_yy04, double_integrator_QP_solver_bmy04);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld04, double_integrator_QP_solver_bmy04, double_integrator_QP_solver_dvcc04);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd04, double_integrator_QP_solver_dvcc04, double_integrator_QP_solver_yy03, double_integrator_QP_solver_bmy03);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld03, double_integrator_QP_solver_bmy03, double_integrator_QP_solver_dvcc03);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd03, double_integrator_QP_solver_dvcc03, double_integrator_QP_solver_yy02, double_integrator_QP_solver_bmy02);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld02, double_integrator_QP_solver_bmy02, double_integrator_QP_solver_dvcc02);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd02, double_integrator_QP_solver_dvcc02, double_integrator_QP_solver_yy01, double_integrator_QP_solver_bmy01);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld01, double_integrator_QP_solver_bmy01, double_integrator_QP_solver_dvcc01);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd01, double_integrator_QP_solver_dvcc01, double_integrator_QP_solver_yy00, double_integrator_QP_solver_bmy00);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld00, double_integrator_QP_solver_bmy00, double_integrator_QP_solver_dvcc00);
double_integrator_QP_solver_LA_DENSE_MTVM_7_18(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc00, double_integrator_QP_solver_grad_eq00);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc01, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc00, double_integrator_QP_solver_grad_eq01);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc02, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc01, double_integrator_QP_solver_grad_eq02);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc03, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc02, double_integrator_QP_solver_grad_eq03);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc04, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc03, double_integrator_QP_solver_grad_eq04);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc05, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc04, double_integrator_QP_solver_grad_eq05);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc06, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc05, double_integrator_QP_solver_grad_eq06);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc07, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc06, double_integrator_QP_solver_grad_eq07);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc08, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc07, double_integrator_QP_solver_grad_eq08);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_18_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc09, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc08, double_integrator_QP_solver_grad_eq09);
double_integrator_QP_solver_LA_DENSE_MTVM2_4_16_7(double_integrator_QP_solver_C10, double_integrator_QP_solver_dvcc10, double_integrator_QP_solver_D10, double_integrator_QP_solver_dvcc09, double_integrator_QP_solver_grad_eq10);
double_integrator_QP_solver_LA_DIAGZERO_MTVM_4_5(double_integrator_QP_solver_D11, double_integrator_QP_solver_dvcc10, double_integrator_QP_solver_grad_eq11);
double_integrator_QP_solver_LA_VSUB_201(double_integrator_QP_solver_rd, double_integrator_QP_solver_grad_eq, double_integrator_QP_solver_rd);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_rd00, double_integrator_QP_solver_dzcc00);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_rd01, double_integrator_QP_solver_dzcc01);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_rd02, double_integrator_QP_solver_dzcc02);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_rd03, double_integrator_QP_solver_dzcc03);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_rd04, double_integrator_QP_solver_dzcc04);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_rd05, double_integrator_QP_solver_dzcc05);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_rd06, double_integrator_QP_solver_dzcc06);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_rd07, double_integrator_QP_solver_dzcc07);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_rd08, double_integrator_QP_solver_dzcc08);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_18(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_rd09, double_integrator_QP_solver_dzcc09);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_16(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_rd10, double_integrator_QP_solver_dzcc10);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_5(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_rd11, double_integrator_QP_solver_dzcc11);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl00, double_integrator_QP_solver_slb00, double_integrator_QP_solver_llbbyslb00, double_integrator_QP_solver_dzcc00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_dllbcc00);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub00, double_integrator_QP_solver_sub00, double_integrator_QP_solver_lubbysub00, double_integrator_QP_solver_dzcc00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_dlubcc00);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_18(params->A1, double_integrator_QP_solver_dzcc00, double_integrator_QP_solver_ccrhsp00, double_integrator_QP_solver_sp00, double_integrator_QP_solver_lp00, double_integrator_QP_solver_dlp_cc00);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_ccrhsl01, double_integrator_QP_solver_slb01, double_integrator_QP_solver_llbbyslb01, double_integrator_QP_solver_dzcc01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_dllbcc01);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub01, double_integrator_QP_solver_sub01, double_integrator_QP_solver_lubbysub01, double_integrator_QP_solver_dzcc01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_dlubcc01);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_18(params->A2, double_integrator_QP_solver_dzcc01, double_integrator_QP_solver_ccrhsp01, double_integrator_QP_solver_sp01, double_integrator_QP_solver_lp01, double_integrator_QP_solver_dlp_cc01);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_ccrhsl02, double_integrator_QP_solver_slb02, double_integrator_QP_solver_llbbyslb02, double_integrator_QP_solver_dzcc02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_dllbcc02);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub02, double_integrator_QP_solver_sub02, double_integrator_QP_solver_lubbysub02, double_integrator_QP_solver_dzcc02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_dlubcc02);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_18(params->A3, double_integrator_QP_solver_dzcc02, double_integrator_QP_solver_ccrhsp02, double_integrator_QP_solver_sp02, double_integrator_QP_solver_lp02, double_integrator_QP_solver_dlp_cc02);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_ccrhsl03, double_integrator_QP_solver_slb03, double_integrator_QP_solver_llbbyslb03, double_integrator_QP_solver_dzcc03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_dllbcc03);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub03, double_integrator_QP_solver_sub03, double_integrator_QP_solver_lubbysub03, double_integrator_QP_solver_dzcc03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_dlubcc03);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_18(params->A4, double_integrator_QP_solver_dzcc03, double_integrator_QP_solver_ccrhsp03, double_integrator_QP_solver_sp03, double_integrator_QP_solver_lp03, double_integrator_QP_solver_dlp_cc03);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_ccrhsl04, double_integrator_QP_solver_slb04, double_integrator_QP_solver_llbbyslb04, double_integrator_QP_solver_dzcc04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_dllbcc04);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub04, double_integrator_QP_solver_sub04, double_integrator_QP_solver_lubbysub04, double_integrator_QP_solver_dzcc04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_dlubcc04);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_18(params->A5, double_integrator_QP_solver_dzcc04, double_integrator_QP_solver_ccrhsp04, double_integrator_QP_solver_sp04, double_integrator_QP_solver_lp04, double_integrator_QP_solver_dlp_cc04);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_ccrhsl05, double_integrator_QP_solver_slb05, double_integrator_QP_solver_llbbyslb05, double_integrator_QP_solver_dzcc05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_dllbcc05);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub05, double_integrator_QP_solver_sub05, double_integrator_QP_solver_lubbysub05, double_integrator_QP_solver_dzcc05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_dlubcc05);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_18(params->A6, double_integrator_QP_solver_dzcc05, double_integrator_QP_solver_ccrhsp05, double_integrator_QP_solver_sp05, double_integrator_QP_solver_lp05, double_integrator_QP_solver_dlp_cc05);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_ccrhsl06, double_integrator_QP_solver_slb06, double_integrator_QP_solver_llbbyslb06, double_integrator_QP_solver_dzcc06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_dllbcc06);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub06, double_integrator_QP_solver_sub06, double_integrator_QP_solver_lubbysub06, double_integrator_QP_solver_dzcc06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_dlubcc06);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_18(params->A7, double_integrator_QP_solver_dzcc06, double_integrator_QP_solver_ccrhsp06, double_integrator_QP_solver_sp06, double_integrator_QP_solver_lp06, double_integrator_QP_solver_dlp_cc06);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_ccrhsl07, double_integrator_QP_solver_slb07, double_integrator_QP_solver_llbbyslb07, double_integrator_QP_solver_dzcc07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_dllbcc07);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub07, double_integrator_QP_solver_sub07, double_integrator_QP_solver_lubbysub07, double_integrator_QP_solver_dzcc07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_dlubcc07);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_18(params->A8, double_integrator_QP_solver_dzcc07, double_integrator_QP_solver_ccrhsp07, double_integrator_QP_solver_sp07, double_integrator_QP_solver_lp07, double_integrator_QP_solver_dlp_cc07);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_ccrhsl08, double_integrator_QP_solver_slb08, double_integrator_QP_solver_llbbyslb08, double_integrator_QP_solver_dzcc08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_dllbcc08);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub08, double_integrator_QP_solver_sub08, double_integrator_QP_solver_lubbysub08, double_integrator_QP_solver_dzcc08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_dlubcc08);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_18(params->A9, double_integrator_QP_solver_dzcc08, double_integrator_QP_solver_ccrhsp08, double_integrator_QP_solver_sp08, double_integrator_QP_solver_lp08, double_integrator_QP_solver_dlp_cc08);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_ccrhsl09, double_integrator_QP_solver_slb09, double_integrator_QP_solver_llbbyslb09, double_integrator_QP_solver_dzcc09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_dllbcc09);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub09, double_integrator_QP_solver_sub09, double_integrator_QP_solver_lubbysub09, double_integrator_QP_solver_dzcc09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_dlubcc09);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_18(params->A10, double_integrator_QP_solver_dzcc09, double_integrator_QP_solver_ccrhsp09, double_integrator_QP_solver_sp09, double_integrator_QP_solver_lp09, double_integrator_QP_solver_dlp_cc09);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_ccrhsl10, double_integrator_QP_solver_slb10, double_integrator_QP_solver_llbbyslb10, double_integrator_QP_solver_dzcc10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_dllbcc10);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub10, double_integrator_QP_solver_sub10, double_integrator_QP_solver_lubbysub10, double_integrator_QP_solver_dzcc10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_dlubcc10);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_16(params->A11, double_integrator_QP_solver_dzcc10, double_integrator_QP_solver_ccrhsp10, double_integrator_QP_solver_sp10, double_integrator_QP_solver_lp10, double_integrator_QP_solver_dlp_cc10);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_ccrhsl11, double_integrator_QP_solver_slb11, double_integrator_QP_solver_llbbyslb11, double_integrator_QP_solver_dzcc11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_dllbcc11);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_ccrhsub11, double_integrator_QP_solver_sub11, double_integrator_QP_solver_lubbysub11, double_integrator_QP_solver_dzcc11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_dlubcc11);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_10_5(params->A12, double_integrator_QP_solver_dzcc11, double_integrator_QP_solver_ccrhsp11, double_integrator_QP_solver_sp11, double_integrator_QP_solver_lp11, double_integrator_QP_solver_dlp_cc11);
double_integrator_QP_solver_LA_VSUB7_415(double_integrator_QP_solver_l, double_integrator_QP_solver_ccrhs, double_integrator_QP_solver_s, double_integrator_QP_solver_dl_cc, double_integrator_QP_solver_ds_cc);
double_integrator_QP_solver_LA_VADD_201(double_integrator_QP_solver_dz_cc, double_integrator_QP_solver_dz_aff);
double_integrator_QP_solver_LA_VADD_74(double_integrator_QP_solver_dv_cc, double_integrator_QP_solver_dv_aff);
double_integrator_QP_solver_LA_VADD_415(double_integrator_QP_solver_dl_cc, double_integrator_QP_solver_dl_aff);
double_integrator_QP_solver_LA_VADD_415(double_integrator_QP_solver_ds_cc, double_integrator_QP_solver_ds_aff);
info->lsit_cc = double_integrator_QP_solver_LINESEARCH_BACKTRACKING_COMBINED(double_integrator_QP_solver_z, double_integrator_QP_solver_v, double_integrator_QP_solver_l, double_integrator_QP_solver_s, double_integrator_QP_solver_dz_cc, double_integrator_QP_solver_dv_cc, double_integrator_QP_solver_dl_cc, double_integrator_QP_solver_ds_cc, &info->step_cc, &info->mu);
if( info->lsit_cc == double_integrator_QP_solver_NOPROGRESS ){
PRINTTEXT("Line search could not proceed at iteration %d, exiting.\n",info->it+1);
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
output->z1[6] = double_integrator_QP_solver_z00[12];
output->z2[0] = double_integrator_QP_solver_z01[0];
output->z2[1] = double_integrator_QP_solver_z01[1];
output->z2[2] = double_integrator_QP_solver_z01[2];
output->z2[3] = double_integrator_QP_solver_z01[3];
output->z2[4] = double_integrator_QP_solver_z01[4];
output->z2[5] = double_integrator_QP_solver_z01[5];
output->z2[6] = double_integrator_QP_solver_z01[12];
output->z3[0] = double_integrator_QP_solver_z02[0];
output->z3[1] = double_integrator_QP_solver_z02[1];
output->z3[2] = double_integrator_QP_solver_z02[2];
output->z3[3] = double_integrator_QP_solver_z02[3];
output->z3[4] = double_integrator_QP_solver_z02[4];
output->z3[5] = double_integrator_QP_solver_z02[5];
output->z3[6] = double_integrator_QP_solver_z02[12];
output->z4[0] = double_integrator_QP_solver_z03[0];
output->z4[1] = double_integrator_QP_solver_z03[1];
output->z4[2] = double_integrator_QP_solver_z03[2];
output->z4[3] = double_integrator_QP_solver_z03[3];
output->z4[4] = double_integrator_QP_solver_z03[4];
output->z4[5] = double_integrator_QP_solver_z03[5];
output->z4[6] = double_integrator_QP_solver_z03[12];
output->z5[0] = double_integrator_QP_solver_z04[0];
output->z5[1] = double_integrator_QP_solver_z04[1];
output->z5[2] = double_integrator_QP_solver_z04[2];
output->z5[3] = double_integrator_QP_solver_z04[3];
output->z5[4] = double_integrator_QP_solver_z04[4];
output->z5[5] = double_integrator_QP_solver_z04[5];
output->z5[6] = double_integrator_QP_solver_z04[12];
output->z6[0] = double_integrator_QP_solver_z05[0];
output->z6[1] = double_integrator_QP_solver_z05[1];
output->z6[2] = double_integrator_QP_solver_z05[2];
output->z6[3] = double_integrator_QP_solver_z05[3];
output->z6[4] = double_integrator_QP_solver_z05[4];
output->z6[5] = double_integrator_QP_solver_z05[5];
output->z6[6] = double_integrator_QP_solver_z05[12];
output->z7[0] = double_integrator_QP_solver_z06[0];
output->z7[1] = double_integrator_QP_solver_z06[1];
output->z7[2] = double_integrator_QP_solver_z06[2];
output->z7[3] = double_integrator_QP_solver_z06[3];
output->z7[4] = double_integrator_QP_solver_z06[4];
output->z7[5] = double_integrator_QP_solver_z06[5];
output->z7[6] = double_integrator_QP_solver_z06[12];
output->z8[0] = double_integrator_QP_solver_z07[0];
output->z8[1] = double_integrator_QP_solver_z07[1];
output->z8[2] = double_integrator_QP_solver_z07[2];
output->z8[3] = double_integrator_QP_solver_z07[3];
output->z8[4] = double_integrator_QP_solver_z07[4];
output->z8[5] = double_integrator_QP_solver_z07[5];
output->z8[6] = double_integrator_QP_solver_z07[12];
output->z9[0] = double_integrator_QP_solver_z08[0];
output->z9[1] = double_integrator_QP_solver_z08[1];
output->z9[2] = double_integrator_QP_solver_z08[2];
output->z9[3] = double_integrator_QP_solver_z08[3];
output->z9[4] = double_integrator_QP_solver_z08[4];
output->z9[5] = double_integrator_QP_solver_z08[5];
output->z9[6] = double_integrator_QP_solver_z08[12];
output->z10[0] = double_integrator_QP_solver_z09[0];
output->z10[1] = double_integrator_QP_solver_z09[1];
output->z10[2] = double_integrator_QP_solver_z09[2];
output->z10[3] = double_integrator_QP_solver_z09[3];
output->z10[4] = double_integrator_QP_solver_z09[4];
output->z10[5] = double_integrator_QP_solver_z09[5];
output->z10[6] = double_integrator_QP_solver_z09[12];
output->z11[0] = double_integrator_QP_solver_z10[0];
output->z11[1] = double_integrator_QP_solver_z10[1];
output->z11[2] = double_integrator_QP_solver_z10[2];
output->z11[3] = double_integrator_QP_solver_z10[3];
output->z11[4] = double_integrator_QP_solver_z10[4];
output->z11[5] = double_integrator_QP_solver_z10[5];
output->z11[6] = double_integrator_QP_solver_z10[10];
output->z12[0] = double_integrator_QP_solver_z11[0];
output->z12[1] = double_integrator_QP_solver_z11[1];
output->z12[2] = double_integrator_QP_solver_z11[2];
output->z12[3] = double_integrator_QP_solver_z11[3];

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
