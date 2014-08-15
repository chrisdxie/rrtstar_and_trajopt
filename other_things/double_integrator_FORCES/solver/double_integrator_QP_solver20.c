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
 * Initializes a vector of length 233 with a value.
 */
void double_integrator_QP_solver_LA_INITIALIZEVECTOR_233(double_integrator_QP_solver_FLOAT* vec, double_integrator_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<233; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 35 with a value.
 */
void double_integrator_QP_solver_LA_INITIALIZEVECTOR_35(double_integrator_QP_solver_FLOAT* vec, double_integrator_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<35; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 721 with a value.
 */
void double_integrator_QP_solver_LA_INITIALIZEVECTOR_721(double_integrator_QP_solver_FLOAT* vec, double_integrator_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<721; i++ )
	{
		vec[i] = value;
	}
}


/* 
 * Calculates a dot product and adds it to a variable: z += x'*y; 
 * This function is for vectors of length 721.
 */
void double_integrator_QP_solver_LA_DOTACC_721(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<721; i++ ){
		*z += x[i]*y[i];
	}
}


/*
 * Calculates the gradient and the value for a quadratic function 0.5*z'*H*z + f'*z
 *
 * INPUTS:     H  - Symmetric Hessian, diag matrix of size [12 x 12]
 *             f  - column vector of size 12
 *             z  - column vector of size 12
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 12
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_FLOAT* H, double_integrator_QP_solver_FLOAT* f, double_integrator_QP_solver_FLOAT* z, double_integrator_QP_solver_FLOAT* grad, double_integrator_QP_solver_FLOAT* value)
{
	int i;
	double_integrator_QP_solver_FLOAT hz;	
	for( i=0; i<12; i++){
		hz = H[i]*z[i];
		grad[i] = hz + f[i];
		*value += 0.5*hz*z[i] + f[i]*z[i];
	}
}


/*
 * Prints vector of length 12 as row vector.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_FLOAT* vec, char *name)
{
	int i;	
	PRINTTEXT("%s = [",name);
	for( i=0; i<11; i++){
		PRINTTEXT("%6.4e,  ",vec[i]);
	}
	PRINTTEXT("%6.4e]\n",vec[11]);
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
 * Prints vector of length 5 as row vector.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_FLOAT* vec, char *name)
{
	int i;	
	PRINTTEXT("%s = [",name);
	for( i=0; i<4; i++){
		PRINTTEXT("%6.4e,  ",vec[i]);
	}
	PRINTTEXT("%6.4e]\n",vec[4]);
}


/* 
 * Computes r = B*u - b
 * and      y = max([norm(r,inf), y])
 * and      z -= l'*r
 * where B is stored in diabzero format
 */
void double_integrator_QP_solver_LA_DIAGZERO_MVMSUB6_12(double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	double_integrator_QP_solver_FLOAT Bu[12];
	double_integrator_QP_solver_FLOAT norm = *y;
	double_integrator_QP_solver_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<12; i++ ){
		Bu[i] = B[i]*u[i];
	}	

	for( i=0; i<12; i++ ){
		r[i] = Bu[i] - b[i];
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;
	double_integrator_QP_solver_FLOAT AxBu[1];
	double_integrator_QP_solver_FLOAT norm = *y;
	double_integrator_QP_solver_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<1; i++ ){
		AxBu[i] = A[k++]*x[0] + B[m++]*u[0];
	}	
	for( j=1; j<12; j++ ){		
		for( i=0; i<1; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<12; n++ ){
		for( i=0; i<1; i++ ){
			AxBu[i] += B[m++]*u[n];
		}		
	}

	for( i=0; i<1; i++ ){
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
 * Prints vector of length 1 as row vector.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_FLOAT* vec, char *name)
{
	int i;	
	PRINTTEXT("%s = [",name);
	for( i=0; i<0; i++){
		PRINTTEXT("%6.4e,  ",vec[i]);
	}
	PRINTTEXT("%6.4e]\n",vec[0]);
}


/* 
 * Computes r = A*x + B*u - b
 * and      y = max([norm(r,inf), y])
 * and      z -= l'*r
 * where A is stored in column major format
 */
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_12_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
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

	for( j=1; j<12; j++ ){		
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
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [1 x 12] and stored in column major format.
 * and B is of size [12 x 12] and stored in diagzero format
 * Note the transposes of A and B!
 */
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_1_12_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	for( i=0; i<12; i++ ){
		z[i] = 0;
		for( j=0; j<1; j++ ){
			z[i] += A[k++]*x[j];
		}
		z[i] += B[i]*y[i];
	}
	for( i=12 ;i<12; i++ ){
		z[i] = 0;
		for( j=0; j<1; j++ ){
			z[i] += A[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [1 x 12]
 * and B is of size [1 x 12]
 * and stored in column major format. Note the transposes of A and B!
 */
void double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	int n;
	int m = 0;
	for( i=0; i<12; i++ ){
		z[i] = 0;
		for( j=0; j<1; j++ ){
			z[i] += A[k++]*x[j];
		}
		for( n=0; n<1; n++ ){
			z[i] += B[m++]*y[n];
		}
	}
}


/*
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [5 x 12]
 * and B is of size [1 x 12]
 * and stored in column major format. Note the transposes of A and B!
 */
void double_integrator_QP_solver_LA_DENSE_MTVM2_5_12_1(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	int n;
	int m = 0;
	for( i=0; i<12; i++ ){
		z[i] = 0;
		for( j=0; j<5; j++ ){
			z[i] += A[k++]*x[j];
		}
		for( n=0; n<1; n++ ){
			z[i] += B[m++]*y[n];
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
 * Prints vector of length 7 as row vector.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_FLOAT* vec, char *name)
{
	int i;	
	PRINTTEXT("%s = [",name);
	for( i=0; i<6; i++){
		PRINTTEXT("%6.4e,  ",vec[i]);
	}
	PRINTTEXT("%6.4e]\n",vec[6]);
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
 * Prints vector of length 6 as row vector.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_FLOAT* vec, char *name)
{
	int i;	
	PRINTTEXT("%s = [",name);
	for( i=0; i<5; i++){
		PRINTTEXT("%6.4e,  ",vec[i]);
	}
	PRINTTEXT("%6.4e]\n",vec[5]);
}


/* 
 * Computes r = A*x - b + s
 * and      y = max([norm(r,inf), y])
 * and      z -= l'*(Ax-b)
 * where A is stored in column major format
 */
void double_integrator_QP_solver_LA_MVSUBADD_24_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
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
	for( j=1; j<12; j++ ){		
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
 * Prints vector of length 24 as row vector.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_FLOAT* vec, char *name)
{
	int i;	
	PRINTTEXT("%s = [",name);
	for( i=0; i<23; i++){
		PRINTTEXT("%6.4e,  ",vec[i]);
	}
	PRINTTEXT("%6.4e]\n",vec[23]);
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
 * Prints vector of length 4 as row vector.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_4(double_integrator_QP_solver_FLOAT* vec, char *name)
{
	int i;	
	PRINTTEXT("%s = [",name);
	for( i=0; i<3; i++){
		PRINTTEXT("%6.4e,  ",vec[i]);
	}
	PRINTTEXT("%6.4e]\n",vec[3]);
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
 * Prints vector of length 10 as row vector.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_10(double_integrator_QP_solver_FLOAT* vec, char *name)
{
	int i;	
	PRINTTEXT("%s = [",name);
	for( i=0; i<9; i++){
		PRINTTEXT("%6.4e,  ",vec[i]);
	}
	PRINTTEXT("%6.4e]\n",vec[9]);
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 12
 * Returns also L/S, a value that is often used elsewhere.
 */
void double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_FLOAT *lu, double_integrator_QP_solver_FLOAT *su, double_integrator_QP_solver_FLOAT *ru, double_integrator_QP_solver_FLOAT *ll, double_integrator_QP_solver_FLOAT *sl, double_integrator_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_FLOAT *grad, double_integrator_QP_solver_FLOAT *lubysu, double_integrator_QP_solver_FLOAT *llbysl)
{
	int i;
	for( i=0; i<12; i++ ){
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
void double_integrator_QP_solver_LA_INEQ_P_24_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *lp, double_integrator_QP_solver_FLOAT *sp, double_integrator_QP_solver_FLOAT *rip, double_integrator_QP_solver_FLOAT *grad, double_integrator_QP_solver_FLOAT *lpbysp)
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

	for( i=0; i<12; i++ ){		
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
 * of length 233.
 */
void double_integrator_QP_solver_LA_VVADD3_233(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *w, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<233; i++ ){
		z[i] = u[i] + v[i] + w[i];
	}
}


/*
 * Special function to compute the Dense positive definite 
 * augmented Hessian for block size 12.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_FLOAT *H, double_integrator_QP_solver_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_FLOAT *Phi)
{
	int i;
	int j;
	int k = 0;
	
	/* copy diagonal of H into PHI and set lower part of PHI = 0*/
	for( i=0; i<12; i++ ){
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
 * Prints a matrix in triangular storage format.
 * The output can be pasted into MATLAB.
 */
void double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_FLOAT *M, char *name)
{
    int i;
	int j;
	int ii;
    PRINTTEXT("%s = [\n\t",name);
    for( i=0; i<12; i++ ){
        ii = (i*(i+1))/2;
        for( j=0; j<=i; j++ ){
            if( j<12-1 )
                PRINTTEXT("% 6.4e,  ",M[ii+j]);
            else
                PRINTTEXT("% 6.4e;  ",M[ii+j]);
        }
        for( j=i+1; j<12; j++ ){
            if( j<12-1 )
                PRINTTEXT("% 6.4e,  ",0.0);
            else
                PRINTTEXT("% 6.4e;  ",0.0);
        }
        if( i<12-1){
            PRINTTEXT("\n\t");
        }
    }
	PRINTTEXT("];\n");
}


/**
 * Compute X = X + A'*D*A, where A is a general full matrix, D is
 * is a diagonal matrix stored in the vector d and X is a symmetric
 * positive definite matrix in lower triangular storage format. 
 * A is stored in column major format and is of size [24 x 12]
 * Phi is of size [12 x 12].
 */
void double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *d, double_integrator_QP_solver_FLOAT *X)
{    
    int i,j,k,ii,di;
    double_integrator_QP_solver_FLOAT x;
    
    di = 0; ii = 0;
    for( i=0; i<12; i++ ){        
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
 * lower triangular storage format of size 12.
 */
void double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_FLOAT *A)
{
    int i, j, k, di, dj;
	 int ii, jj;
    double_integrator_QP_solver_FLOAT l;
    double_integrator_QP_solver_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<12; i++ ){
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
        for( j=i+1; j<12; j++ ){
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
 * where A is to be computed and is of size [1 x 12],
 * B is given and of size [1 x 12], L is a lower tri-
 * angular matrix of size 12 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    double_integrator_QP_solver_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<12; j++ ){        
        for( i=0; i<1; i++ ){
            a = B[j*1+i];
            for( k=0; k<j; k++ ){
                a -= A[k*1+i]*L[ii+k];
            }

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

            A[j*1+i] = a/L[ii+j];
        }
        ii += ++di;
    }
}


/*
 * Prints a dense matrix of size [12 x 12].
 * The matrix is assumed to be stored in column major format.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_12_12(double_integrator_QP_solver_FLOAT* mat, char *name)
{
	int i;	
	int j;
	int k=0;
	PRINTTEXT("%s = [\n",name);
	for( i=0; i<12; i++){
		PRINTTEXT("    ");
		for( j=0; j<12; j++ ){
			PRINTTEXT("% 6.4e  ",mat[j*12+i]);
		}
		if( i<12-1 )
			PRINTTEXT(";\n");
		else
			PRINTTEXT("];\n");
	}
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [12 x 12],
 * B is given and of size [12 x 12] stored in 
 * diagzero storage format, L is a lower tri-
 * angular matrix of size 12 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_12_12(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
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
    for( j=0; j<12; j++ ){        
        for( i=0; i<j; i++ ){
            /* Calculate part of A which is non-zero and not diagonal "u"
             * i < j */
            a = 0;
			
            for( k=i; k<j; k++ ){
                a -= A[k*12+i]*L[ii+k];
            }
            A[j*12+i] = a/L[ii+j];
        }
        /* do the diagonal "d"
         * i = j */
        A[j*12+j] = B[i]/L[ii+j];
        
        /* fill lower triangular part with zeros "0"
         * n > i > j */
        for( i=j+1     ; i < 12; i++ ){
            A[j*12+i] = 0;
        }
        
        /* increment index of L */
        ii += ++di;	
    }
	
	/* Part 2 */ 
	for( j=12; j<12; j++ ){        
        for( i=0; i<12; i++ ){
            /* Calculate part of A which is non-zero and not diagonal "r" */
            a = 0;
			
            for( k=i; k<j; k++ ){
                a -= A[k*12+i]*L[ii+k];
            }
            A[j*12+i] = a/L[ii+j];
        }
        
        /* increment index of L */
        ii += ++di;	
    }
	
	
	
}


/**
 * Compute C = A*B' where 
 *
 *	size(A) = [12 x 12]
 *  size(B) = [12 x 12]
 * 
 * and all matrices are stored in column major format.
 *
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE.  
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMTM_12_12_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *C)
{
    int i, j, k;
    double_integrator_QP_solver_FLOAT temp;
    
    for( i=0; i<12; i++ ){        
        for( j=0; j<12; j++ ){
            temp = 0; 
            for( k=0; k<12; k++ ){
                temp += A[k*12+i]*B[k*12+j];
            }						
            C[j*12+i] = temp;
        }
    }
}


/*
 * Prints a dense matrix of size [12 x 1].
 * The matrix is assumed to be stored in column major format.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_12_1(double_integrator_QP_solver_FLOAT* mat, char *name)
{
	int i;	
	int j;
	int k=0;
	PRINTTEXT("%s = [\n",name);
	for( i=0; i<12; i++){
		PRINTTEXT("    ");
		for( j=0; j<1; j++ ){
			PRINTTEXT("% 6.4e  ",mat[j*12+i]);
		}
		if( i<12-1 )
			PRINTTEXT(";\n");
		else
			PRINTTEXT("];\n");
	}
}


/**
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 12.
 */
void double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<12; i++ ){
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
 * Prints a dense matrix of size [1 x 12].
 * The matrix is assumed to be stored in column major format.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_FLOAT* mat, char *name)
{
	int i;	
	int j;
	int k=0;
	PRINTTEXT("%s = [\n",name);
	for( i=0; i<1; i++){
		PRINTTEXT("    ");
		for( j=0; j<12; j++ ){
			PRINTTEXT("% 6.4e  ",mat[j*1+i]);
		}
		if( i<1-1 )
			PRINTTEXT(";\n");
		else
			PRINTTEXT("];\n");
	}
}


/**
 * Compute C = A*B' where 
 *
 *	size(A) = [1 x 12]
 *  size(B) = [1 x 12]
 * 
 * and all matrices are stored in column major format.
 *
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE.  
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *C)
{
    int i, j, k;
    double_integrator_QP_solver_FLOAT temp;
    
    for( i=0; i<1; i++ ){        
        for( j=0; j<1; j++ ){
            temp = 0; 
            for( k=0; k<12; k++ ){
                temp += A[k*1+i]*B[k*1+j];
            }						
            C[j*1+i] = temp;
        }
    }
}


/*
 * Prints a dense matrix of size [1 x 1].
 * The matrix is assumed to be stored in column major format.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_FLOAT* mat, char *name)
{
	int i;	
	int j;
	int k=0;
	PRINTTEXT("%s = [\n",name);
	for( i=0; i<1; i++){
		PRINTTEXT("    ");
		for( j=0; j<1; j++ ){
			PRINTTEXT("% 6.4e  ",mat[j*1+i]);
		}
		if( i<1-1 )
			PRINTTEXT(";\n");
		else
			PRINTTEXT("];\n");
	}
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [5 x 12],
 * B is given and of size [5 x 12], L is a lower tri-
 * angular matrix of size 12 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_5_12(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    double_integrator_QP_solver_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<12; j++ ){        
        for( i=0; i<5; i++ ){
            a = B[j*5+i];
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


/*
 * Prints a dense matrix of size [1 x 5].
 * The matrix is assumed to be stored in column major format.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_5(double_integrator_QP_solver_FLOAT* mat, char *name)
{
	int i;	
	int j;
	int k=0;
	PRINTTEXT("%s = [\n",name);
	for( i=0; i<1; i++){
		PRINTTEXT("    ");
		for( j=0; j<5; j++ ){
			PRINTTEXT("% 6.4e  ",mat[j*1+i]);
		}
		if( i<1-1 )
			PRINTTEXT(";\n");
		else
			PRINTTEXT("];\n");
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
 * Prints a matrix in triangular storage format.
 * The output can be pasted into MATLAB.
 */
void double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_5(double_integrator_QP_solver_FLOAT *M, char *name)
{
    int i;
	int j;
	int ii;
    PRINTTEXT("%s = [\n\t",name);
    for( i=0; i<5; i++ ){
        ii = (i*(i+1))/2;
        for( j=0; j<=i; j++ ){
            if( j<5-1 )
                PRINTTEXT("% 6.4e,  ",M[ii+j]);
            else
                PRINTTEXT("% 6.4e;  ",M[ii+j]);
        }
        for( j=i+1; j<5; j++ ){
            if( j<5-1 )
                PRINTTEXT("% 6.4e,  ",0.0);
            else
                PRINTTEXT("% 6.4e;  ",0.0);
        }
        if( i<5-1){
            PRINTTEXT("\n\t");
        }
    }
	PRINTTEXT("];\n");
}


/**
 * Forward substitution for the matrix equation A*L' = B
 * where A is to be computed and is of size [5 x 5],
 * B is given and of size [5 x 5] stored in 
 * diagzero storage format, L is a lower tri-
 * angular matrix of size 5 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_5_5(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
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
    for( j=0; j<5; j++ ){        
        for( i=0; i<j; i++ ){
            /* Calculate part of A which is non-zero and not diagonal "u"
             * i < j */
            a = 0;
			
            for( k=i; k<j; k++ ){
                a -= A[k*5+i]*L[ii+k];
            }
            A[j*5+i] = a/L[ii+j];
        }
        /* do the diagonal "d"
         * i = j */
        A[j*5+j] = B[i]/L[ii+j];
        
        /* fill lower triangular part with zeros "0"
         * n > i > j */
        for( i=j+1     ; i < 5; i++ ){
            A[j*5+i] = 0;
        }
        
        /* increment index of L */
        ii += ++di;	
    }
	
	/* Part 2 */ 
	for( j=5; j<5; j++ ){        
        for( i=0; i<5; i++ ){
            /* Calculate part of A which is non-zero and not diagonal "r" */
            a = 0;
			
            for( k=i; k<j; k++ ){
                a -= A[k*5+i]*L[ii+k];
            }
            A[j*5+i] = a/L[ii+j];
        }
        
        /* increment index of L */
        ii += ++di;	
    }
	
	
	
}


/*
 * Prints a dense matrix of size [5 x 5].
 * The matrix is assumed to be stored in column major format.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_5_5(double_integrator_QP_solver_FLOAT* mat, char *name)
{
	int i;	
	int j;
	int k=0;
	PRINTTEXT("%s = [\n",name);
	for( i=0; i<5; i++){
		PRINTTEXT("    ");
		for( j=0; j<5; j++ ){
			PRINTTEXT("% 6.4e  ",mat[j*5+i]);
		}
		if( i<5-1 )
			PRINTTEXT(";\n");
		else
			PRINTTEXT("];\n");
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
 * Compute L = B*B', where L is lower triangular of size NXp1
 * and B is a diagzero matrix of size [12 x 12] in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMT_12_12(double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<12; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 		
			for( k=0; k<12; k++ ){
                ltemp += B[k*12+i]*B[k*12+j];
            }
            L[ii+j] = ltemp;
        }
        ii += ++di;
    }
}


/* 
 * Computes r = b - B*u
 * where B is stored in column major format
 */
void double_integrator_QP_solver_LA_DENSE_MVMSUB7_12_12(double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int m = 0;
	int n;

	for( i=0; i<12; i++ ){
		r[i] = b[i] - B[m++]*u[0];
	}	
	
	for( n=1; n<12; n++ ){
		for( i=0; i<12; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [12 x 12] in column
 * storage format, and B is of size [12 x 12] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMT2_12_12_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<12; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<12; k++ ){
                ltemp += A[k*12+i]*A[k*12+j];
            }			
			for( k=0; k<12; k++ ){
                ltemp += B[k*12+i]*B[k*12+j];
            }
            L[ii+j] = ltemp;
        }
        ii += ++di;
    }
}


/**
 * Prints a matrix in triangular storage format.
 * The output can be pasted into MATLAB.
 */
void double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_FLOAT *M, char *name)
{
    int i;
	int j;
	int ii;
    PRINTTEXT("%s = [\n\t",name);
    for( i=0; i<1; i++ ){
        ii = (i*(i+1))/2;
        for( j=0; j<=i; j++ ){
            if( j<1-1 )
                PRINTTEXT("% 6.4e,  ",M[ii+j]);
            else
                PRINTTEXT("% 6.4e;  ",M[ii+j]);
        }
        for( j=i+1; j<1; j++ ){
            if( j<1-1 )
                PRINTTEXT("% 6.4e,  ",0.0);
            else
                PRINTTEXT("% 6.4e;  ",0.0);
        }
        if( i<1-1){
            PRINTTEXT("\n\t");
        }
    }
	PRINTTEXT("];\n");
}


/* 
 * Computes r = b - A*x - B*u
 * where A an B are stored in column major format
 */
void double_integrator_QP_solver_LA_DENSE_MVMSUB2_12_12_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<12; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<12; j++ ){		
		for( i=0; i<12; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<12; n++ ){
		for( i=0; i<12; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [1 x 12] in column
 * storage format, and B is of size [1 x 12] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<1; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<12; k++ ){
                ltemp += A[k*1+i]*A[k*1+j];
            }			
			for( k=0; k<12; k++ ){
                ltemp += B[k*1+i]*B[k*1+j];
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<1; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<12; j++ ){		
		for( i=0; i<1; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<12; n++ ){
		for( i=0; i<1; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [1 x 12] in column
 * storage format, and B is of size [1 x 5] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMT2_1_12_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<1; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<12; k++ ){
                ltemp += A[k*1+i]*A[k*1+j];
            }			
			for( k=0; k<5; k++ ){
                ltemp += B[k*1+i]*B[k*1+j];
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<1; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<12; j++ ){		
		for( i=0; i<1; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<5; n++ ){
		for( i=0; i<1; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 12 and outputting
 * the Cholesky factor to matrix L in lower triangular format.
 */
void double_integrator_QP_solver_LA_DENSE_CHOL_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    double_integrator_QP_solver_FLOAT l;
    double_integrator_QP_solver_FLOAT Mii;

	/* copy A to L first and then operate on L */
	/* COULD BE OPTIMIZED */
	ii=0; di=0;
	for( i=0; i<12; i++ ){
		for( j=0; j<=i; j++ ){
			L[ii+j] = A[ii+j];
		}
		ii += ++di;
	}    
	
	/* factor L */
	ii=0; di=0;
    for( i=0; i<12; i++ ){
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
        for( j=i+1; j<12; j++ ){
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
 * where A is to be computed and is of size [1 x 12],
 * B is given and of size [1 x 12], L is a lower tri-
 * angular matrix of size 12 stored in lower triangular 
 * storage format. Note the transpose of L AND B!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_12(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j,k,ii,di;
    double_integrator_QP_solver_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<12; j++ ){        
        for( i=0; i<1; i++ ){
            a = B[i*12+j];
            for( k=0; k<j; k++ ){
                a -= A[k*1+i]*L[ii+k];
            }    

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

			A[j*1+i] = a/L[ii+j];			
        }
        ii += ++di;
    }
}


/**
 * Compute L = L - A*A', where L is lower triangular of size 1
 * and A is a dense matrix of size [1 x 12] in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMTSUB_1_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<1; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<12; k++ ){
                ltemp += A[k*1+i]*A[k*1+j];
            }						
            L[ii+j] -= ltemp;
        }
        ii += ++di;
    }
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 1 and outputting
 * the Cholesky factor to matrix L in lower triangular format.
 */
void double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    double_integrator_QP_solver_FLOAT l;
    double_integrator_QP_solver_FLOAT Mii;

	/* copy A to L first and then operate on L */
	/* COULD BE OPTIMIZED */
	ii=0; di=0;
	for( i=0; i<1; i++ ){
		for( j=0; j<=i; j++ ){
			L[ii+j] = A[ii+j];
		}
		ii += ++di;
	}    
	
	/* factor L */
	ii=0; di=0;
    for( i=0; i<1; i++ ){
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
        for( j=i+1; j<1; j++ ){
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<1; i++ ){
		r[i] = b[i] - A[k++]*x[0];
	}	
	for( j=1; j<12; j++ ){		
		for( i=0; i<1; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/**
 * Forward substitution to solve L*y = b where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * The dimensions involved are 1.
 */
void double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_FLOAT yel;
            
    ii = 0; di = 0;
    for( i=0; i<1; i++ ){
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
 * where A is to be computed and is of size [1 x 1],
 * B is given and of size [1 x 1], L is a lower tri-
 * angular matrix of size 1 stored in lower triangular 
 * storage format. Note the transpose of L AND B!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j,k,ii,di;
    double_integrator_QP_solver_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<1; j++ ){        
        for( i=0; i<1; i++ ){
            a = B[i*1+j];
            for( k=0; k<j; k++ ){
                a -= A[k*1+i]*L[ii+k];
            }    

			/* saturate for numerical stability */
			a = MIN(a, BIGM);
			a = MAX(a, -BIGM); 

			A[j*1+i] = a/L[ii+j];			
        }
        ii += ++di;
    }
}


/**
 * Compute L = L - A*A', where L is lower triangular of size 1
 * and A is a dense matrix of size [1 x 1] in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<1; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<1; k++ ){
                ltemp += A[k*1+i]*A[k*1+j];
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<1; i++ ){
		r[i] = b[i] - A[k++]*x[0];
	}	
	for( j=1; j<1; j++ ){		
		for( i=0; i<1; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/** 
 * Forward substitution for the matrix equation A*L' = B'
 * where A is to be computed and is of size [5 x 1],
 * B is given and of size [5 x 1], L is a lower tri-
 * angular matrix of size 1 stored in lower triangular 
 * storage format. Note the transpose of L AND B!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_1(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j,k,ii,di;
    double_integrator_QP_solver_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<1; j++ ){        
        for( i=0; i<5; i++ ){
            a = B[i*1+j];
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


/*
 * Prints a dense matrix of size [5 x 1].
 * The matrix is assumed to be stored in column major format.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_5_1(double_integrator_QP_solver_FLOAT* mat, char *name)
{
	int i;	
	int j;
	int k=0;
	PRINTTEXT("%s = [\n",name);
	for( i=0; i<5; i++){
		PRINTTEXT("    ");
		for( j=0; j<1; j++ ){
			PRINTTEXT("% 6.4e  ",mat[j*5+i]);
		}
		if( i<5-1 )
			PRINTTEXT(";\n");
		else
			PRINTTEXT("];\n");
	}
}


/**
 * Compute L = L - A*A', where L is lower triangular of size 5
 * and A is a dense matrix of size [5 x 1] in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_LA_DENSE_MMTSUB_5_1(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<5; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<1; k++ ){
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_1(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<5; i++ ){
		r[i] = b[i] - A[k++]*x[0];
	}	
	for( j=1; j<1; j++ ){		
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
 * Matrix vector multiplication y = b - M'*x where M is of size [5 x 1]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_1(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<1; i++ ){
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
 * All involved dimensions are 1.
 */
void double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_FLOAT xel;    
	int start = 0;
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 0;
    for( i=0; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 0;
        for( j=0; j>i; j-- ){
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
 * Matrix vector multiplication y = b - M'*x where M is of size [1 x 1]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<1; i++ ){
		r[i] = b[i];
		for( j=0; j<1; j++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication y = b - M'*x where M is of size [1 x 12]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<12; i++ ){
		r[i] = b[i];
		for( j=0; j<1; j++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/**
 * Backward Substitution to solve L^T*x = y where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * All involved dimensions are 12.
 */
void double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_12(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_FLOAT xel;    
	int start = 66;
    
    /* now solve L^T*x = y by backward substitution */
    ii = start; di = 11;
    for( i=11; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 11;
        for( j=11; j>i; j-- ){
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
 * Vector subtraction z = -x - y for vectors of length 233.
 */
void double_integrator_QP_solver_LA_VSUB2_233(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<233; i++){
		z[i] = -x[i] - y[i];
	}
}


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * lower triangular matrix of size 12 in lower triangular
 * storage format.
 */
void double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_FLOAT y[12];
    double_integrator_QP_solver_FLOAT yel,xel;
	int start = 66;
            
    /* first solve Ly = b by forward substitution */
     ii = 0; di = 0;
    for( i=0; i<12; i++ ){
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
    ii = start; di = 11;
    for( i=11; i>=0; i-- ){        
        xel = y[i];        
        jj = start; dj = 11;
        for( j=11; j>i; j-- ){
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
 * and x has length 12 and is indexed through yidx.
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
 * Vector subtraction z = -x - y(yidx) where y is of length 12
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<24; i++ ){
		r[i] = -b[i] - A[k++]*x[0];
	}	
	for( j=1; j<12; j++ ){		
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
        for( i=0; i<721; i++ ){
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
        if( i == 721 ){
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
    *mu_aff = mymu / (double_integrator_QP_solver_FLOAT)721;
    return lsIt;
}


/*
 * Prints vector of length 721 as row vector.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_721(double_integrator_QP_solver_FLOAT* vec, char *name)
{
	int i;	
	PRINTTEXT("%s = [",name);
	for( i=0; i<720; i++){
		PRINTTEXT("%6.4e,  ",vec[i]);
	}
	PRINTTEXT("%6.4e]\n",vec[720]);
}


/*
 * Vector subtraction x = (u.*v - mu)*sigma where a is a scalar
*  and x,u,v are vectors of length 721.
 */
void double_integrator_QP_solver_LA_VSUB5_721(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT mu,  double_integrator_QP_solver_FLOAT sigma, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<721; i++){
		x[i] = u[i]*v[i] - mu;
		x[i] *= sigma;
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 12,
 * u, su, uidx are of length 6 and v, sv, vidx are of length 7.
 */
void double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *su, int* uidx, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *sv, int* vidx, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<12; i++ ){
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
 * Matrix vector multiplication z = z + A'*(x./s) where A is of size [24 x 12]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0; 
	double_integrator_QP_solver_FLOAT temp[24];

	for( j=0; j<24; j++ ){
		temp[j] = x[j] / s[j];
	}

	for( i=0; i<12; i++ ){
		for( j=0; j<24; j++ ){
			z[i] += A[k++]*temp[j];
		}
	}
}


/* 
 * Computes r = B*u
 * where B is stored in column major format
 */
void double_integrator_QP_solver_LA_DENSE_MVM_12_12(double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int m = 0;
	int n;

	for( i=0; i<12; i++ ){
		r[i] = B[m++]*u[0];
	}	
	
	for( n=1; n<12; n++ ){
		for( i=0; i<12; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void double_integrator_QP_solver_LA_DENSE_2MVMADD_12_12_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<12; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<12; j++ ){		
		for( i=0; i<12; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<12; n++ ){
		for( i=0; i<12; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<1; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<12; j++ ){		
		for( i=0; i<1; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<12; n++ ){
		for( i=0; i<1; i++ ){
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
void double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_5(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<1; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<12; j++ ){		
		for( i=0; i<1; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<5; n++ ){
		for( i=0; i<1; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/*
 * Vector subtraction z = x - y for vectors of length 233.
 */
void double_integrator_QP_solver_LA_VSUB_233(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<233; i++){
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_FLOAT temp[24];

	
	for( i=0; i<24; i++ ){
		temp[i] = A[k++]*x[0];
	}
	

	for( j=1; j<12; j++ ){		
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
 * Computes ds = -l.\(r + s.*dl) for vectors of length 721.
 */
void double_integrator_QP_solver_LA_VSUB7_721(double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *dl, double_integrator_QP_solver_FLOAT *ds)
{
	int i;
	for( i=0; i<721; i++){
		ds[i] = -(r[i] + s[i]*dl[i])/l[i];
	}
}


/*
 * Prints vector of length 233 as row vector.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_233(double_integrator_QP_solver_FLOAT* vec, char *name)
{
	int i;	
	PRINTTEXT("%s = [",name);
	for( i=0; i<232; i++){
		PRINTTEXT("%6.4e,  ",vec[i]);
	}
	PRINTTEXT("%6.4e]\n",vec[232]);
}


/*
 * Prints vector of length 35 as row vector.
 */
void double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_35(double_integrator_QP_solver_FLOAT* vec, char *name)
{
	int i;	
	PRINTTEXT("%s = [",name);
	for( i=0; i<34; i++){
		PRINTTEXT("%6.4e,  ",vec[i]);
	}
	PRINTTEXT("%6.4e]\n",vec[34]);
}


/*
 * Vector addition x = x + y for vectors of length 233.
 */
void double_integrator_QP_solver_LA_VADD_233(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<233; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 35.
 */
void double_integrator_QP_solver_LA_VADD_35(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<35; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 721.
 */
void double_integrator_QP_solver_LA_VADD_721(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<721; i++){
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
        for( i=0; i<721; i++ ){
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
        if( i == 721 ){
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
    for( i=0; i<233; i++ ){
        z[i] += a_gamma*dz[i];
    }
    
    /* equality constraint multipliers */
    for( i=0; i<35; i++ ){
        v[i] += a_gamma*dv[i];
    }
    
    /* inequality constraint multipliers & slacks, also update mu */
    *mu = 0;
    for( i=0; i<721; i++ ){
        dltemp = l[i] + a_gamma*dl[i]; l[i] = dltemp;
        dstemp = s[i] + a_gamma*ds[i]; s[i] = dstemp;
        *mu += dltemp*dstemp;
    }
    
    *a = a_gamma;
    *mu /= (double_integrator_QP_solver_FLOAT)721;
    return lsIt;
}




/* VARIABLE DEFINITIONS ------------------------------------------------ */
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_z[233];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_v[35];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dz_aff[233];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dv_aff[35];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_grad_cost[233];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_grad_eq[233];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rd[233];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_l[721];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_s[721];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_lbys[721];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dl_aff[721];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ds_aff[721];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dz_cc[233];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dv_cc[35];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dl_cc[721];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ds_cc[721];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ccrhs[721];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_grad_ineq[233];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_H00[12] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z00 = double_integrator_QP_solver_z + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff00 = double_integrator_QP_solver_dz_aff + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc00 = double_integrator_QP_solver_dz_cc + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd00 = double_integrator_QP_solver_rd + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd00[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost00 = double_integrator_QP_solver_grad_cost + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq00 = double_integrator_QP_solver_grad_eq + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq00 = double_integrator_QP_solver_grad_ineq + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv00[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_C00[12] = {0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
1.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v00 = double_integrator_QP_solver_v + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re00[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta00[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc00[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff00 = double_integrator_QP_solver_dv_aff + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc00 = double_integrator_QP_solver_dv_cc + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V00[144];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd00[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld00[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy00[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy00[12];
int double_integrator_QP_solver_lbIdx00[7] = {0, 1, 2, 3, 4, 5, 6};
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
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi00[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_D00[12] = {1.0000000000000000E+000, 
1.0000000000000000E+000, 
1.0000000000000000E+000, 
1.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W00[144];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z01 = double_integrator_QP_solver_z + 12;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff01 = double_integrator_QP_solver_dz_aff + 12;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc01 = double_integrator_QP_solver_dz_cc + 12;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd01 = double_integrator_QP_solver_rd + 12;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd01[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost01 = double_integrator_QP_solver_grad_cost + 12;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq01 = double_integrator_QP_solver_grad_eq + 12;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq01 = double_integrator_QP_solver_grad_ineq + 12;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv01[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v01 = double_integrator_QP_solver_v + 12;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re01[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta01[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc01[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff01 = double_integrator_QP_solver_dv_aff + 12;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc01 = double_integrator_QP_solver_dv_cc + 12;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V01[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd01[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld01[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy01[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy01[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c01[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx01[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb01 = double_integrator_QP_solver_l + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb01 = double_integrator_QP_solver_s + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb01 = double_integrator_QP_solver_lbys + 37;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb01[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff01 = double_integrator_QP_solver_dl_aff + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff01 = double_integrator_QP_solver_ds_aff + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc01 = double_integrator_QP_solver_dl_cc + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc01 = double_integrator_QP_solver_ds_cc + 37;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl01 = double_integrator_QP_solver_ccrhs + 37;
int double_integrator_QP_solver_ubIdx01[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub01 = double_integrator_QP_solver_l + 44;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub01 = double_integrator_QP_solver_s + 44;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub01 = double_integrator_QP_solver_lbys + 44;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub01[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff01 = double_integrator_QP_solver_dl_aff + 44;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff01 = double_integrator_QP_solver_ds_aff + 44;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc01 = double_integrator_QP_solver_dl_cc + 44;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc01 = double_integrator_QP_solver_ds_cc + 44;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub01 = double_integrator_QP_solver_ccrhs + 44;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp01 = double_integrator_QP_solver_s + 50;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp01 = double_integrator_QP_solver_l + 50;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp01 = double_integrator_QP_solver_lbys + 50;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff01 = double_integrator_QP_solver_dl_aff + 50;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff01 = double_integrator_QP_solver_ds_aff + 50;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc01 = double_integrator_QP_solver_dl_cc + 50;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc01 = double_integrator_QP_solver_ds_cc + 50;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp01 = double_integrator_QP_solver_ccrhs + 50;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip01[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi01[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_D01[12] = {0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
-1.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000, 
0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W01[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd01[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd01[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z02 = double_integrator_QP_solver_z + 24;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff02 = double_integrator_QP_solver_dz_aff + 24;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc02 = double_integrator_QP_solver_dz_cc + 24;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd02 = double_integrator_QP_solver_rd + 24;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd02[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost02 = double_integrator_QP_solver_grad_cost + 24;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq02 = double_integrator_QP_solver_grad_eq + 24;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq02 = double_integrator_QP_solver_grad_ineq + 24;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv02[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v02 = double_integrator_QP_solver_v + 13;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re02[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta02[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc02[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff02 = double_integrator_QP_solver_dv_aff + 13;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc02 = double_integrator_QP_solver_dv_cc + 13;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V02[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd02[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld02[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy02[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy02[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c02[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx02[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb02 = double_integrator_QP_solver_l + 74;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb02 = double_integrator_QP_solver_s + 74;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb02 = double_integrator_QP_solver_lbys + 74;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb02[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff02 = double_integrator_QP_solver_dl_aff + 74;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff02 = double_integrator_QP_solver_ds_aff + 74;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc02 = double_integrator_QP_solver_dl_cc + 74;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc02 = double_integrator_QP_solver_ds_cc + 74;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl02 = double_integrator_QP_solver_ccrhs + 74;
int double_integrator_QP_solver_ubIdx02[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub02 = double_integrator_QP_solver_l + 81;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub02 = double_integrator_QP_solver_s + 81;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub02 = double_integrator_QP_solver_lbys + 81;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub02[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff02 = double_integrator_QP_solver_dl_aff + 81;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff02 = double_integrator_QP_solver_ds_aff + 81;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc02 = double_integrator_QP_solver_dl_cc + 81;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc02 = double_integrator_QP_solver_ds_cc + 81;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub02 = double_integrator_QP_solver_ccrhs + 81;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp02 = double_integrator_QP_solver_s + 87;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp02 = double_integrator_QP_solver_l + 87;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp02 = double_integrator_QP_solver_lbys + 87;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff02 = double_integrator_QP_solver_dl_aff + 87;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff02 = double_integrator_QP_solver_ds_aff + 87;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc02 = double_integrator_QP_solver_dl_cc + 87;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc02 = double_integrator_QP_solver_ds_cc + 87;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp02 = double_integrator_QP_solver_ccrhs + 87;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip02[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi02[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W02[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd02[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd02[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z03 = double_integrator_QP_solver_z + 36;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff03 = double_integrator_QP_solver_dz_aff + 36;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc03 = double_integrator_QP_solver_dz_cc + 36;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd03 = double_integrator_QP_solver_rd + 36;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd03[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost03 = double_integrator_QP_solver_grad_cost + 36;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq03 = double_integrator_QP_solver_grad_eq + 36;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq03 = double_integrator_QP_solver_grad_ineq + 36;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv03[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v03 = double_integrator_QP_solver_v + 14;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re03[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta03[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc03[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff03 = double_integrator_QP_solver_dv_aff + 14;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc03 = double_integrator_QP_solver_dv_cc + 14;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V03[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd03[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld03[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy03[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy03[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c03[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx03[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb03 = double_integrator_QP_solver_l + 111;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb03 = double_integrator_QP_solver_s + 111;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb03 = double_integrator_QP_solver_lbys + 111;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb03[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff03 = double_integrator_QP_solver_dl_aff + 111;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff03 = double_integrator_QP_solver_ds_aff + 111;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc03 = double_integrator_QP_solver_dl_cc + 111;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc03 = double_integrator_QP_solver_ds_cc + 111;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl03 = double_integrator_QP_solver_ccrhs + 111;
int double_integrator_QP_solver_ubIdx03[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub03 = double_integrator_QP_solver_l + 118;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub03 = double_integrator_QP_solver_s + 118;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub03 = double_integrator_QP_solver_lbys + 118;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub03[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff03 = double_integrator_QP_solver_dl_aff + 118;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff03 = double_integrator_QP_solver_ds_aff + 118;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc03 = double_integrator_QP_solver_dl_cc + 118;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc03 = double_integrator_QP_solver_ds_cc + 118;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub03 = double_integrator_QP_solver_ccrhs + 118;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp03 = double_integrator_QP_solver_s + 124;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp03 = double_integrator_QP_solver_l + 124;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp03 = double_integrator_QP_solver_lbys + 124;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff03 = double_integrator_QP_solver_dl_aff + 124;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff03 = double_integrator_QP_solver_ds_aff + 124;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc03 = double_integrator_QP_solver_dl_cc + 124;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc03 = double_integrator_QP_solver_ds_cc + 124;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp03 = double_integrator_QP_solver_ccrhs + 124;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip03[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi03[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W03[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd03[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd03[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z04 = double_integrator_QP_solver_z + 48;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff04 = double_integrator_QP_solver_dz_aff + 48;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc04 = double_integrator_QP_solver_dz_cc + 48;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd04 = double_integrator_QP_solver_rd + 48;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd04[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost04 = double_integrator_QP_solver_grad_cost + 48;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq04 = double_integrator_QP_solver_grad_eq + 48;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq04 = double_integrator_QP_solver_grad_ineq + 48;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv04[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v04 = double_integrator_QP_solver_v + 15;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re04[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta04[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc04[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff04 = double_integrator_QP_solver_dv_aff + 15;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc04 = double_integrator_QP_solver_dv_cc + 15;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V04[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd04[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld04[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy04[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy04[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c04[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx04[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb04 = double_integrator_QP_solver_l + 148;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb04 = double_integrator_QP_solver_s + 148;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb04 = double_integrator_QP_solver_lbys + 148;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb04[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff04 = double_integrator_QP_solver_dl_aff + 148;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff04 = double_integrator_QP_solver_ds_aff + 148;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc04 = double_integrator_QP_solver_dl_cc + 148;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc04 = double_integrator_QP_solver_ds_cc + 148;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl04 = double_integrator_QP_solver_ccrhs + 148;
int double_integrator_QP_solver_ubIdx04[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub04 = double_integrator_QP_solver_l + 155;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub04 = double_integrator_QP_solver_s + 155;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub04 = double_integrator_QP_solver_lbys + 155;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub04[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff04 = double_integrator_QP_solver_dl_aff + 155;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff04 = double_integrator_QP_solver_ds_aff + 155;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc04 = double_integrator_QP_solver_dl_cc + 155;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc04 = double_integrator_QP_solver_ds_cc + 155;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub04 = double_integrator_QP_solver_ccrhs + 155;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp04 = double_integrator_QP_solver_s + 161;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp04 = double_integrator_QP_solver_l + 161;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp04 = double_integrator_QP_solver_lbys + 161;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff04 = double_integrator_QP_solver_dl_aff + 161;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff04 = double_integrator_QP_solver_ds_aff + 161;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc04 = double_integrator_QP_solver_dl_cc + 161;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc04 = double_integrator_QP_solver_ds_cc + 161;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp04 = double_integrator_QP_solver_ccrhs + 161;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip04[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi04[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W04[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd04[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd04[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z05 = double_integrator_QP_solver_z + 60;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff05 = double_integrator_QP_solver_dz_aff + 60;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc05 = double_integrator_QP_solver_dz_cc + 60;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd05 = double_integrator_QP_solver_rd + 60;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd05[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost05 = double_integrator_QP_solver_grad_cost + 60;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq05 = double_integrator_QP_solver_grad_eq + 60;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq05 = double_integrator_QP_solver_grad_ineq + 60;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv05[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v05 = double_integrator_QP_solver_v + 16;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re05[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta05[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc05[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff05 = double_integrator_QP_solver_dv_aff + 16;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc05 = double_integrator_QP_solver_dv_cc + 16;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V05[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd05[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld05[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy05[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy05[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c05[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx05[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb05 = double_integrator_QP_solver_l + 185;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb05 = double_integrator_QP_solver_s + 185;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb05 = double_integrator_QP_solver_lbys + 185;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb05[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff05 = double_integrator_QP_solver_dl_aff + 185;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff05 = double_integrator_QP_solver_ds_aff + 185;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc05 = double_integrator_QP_solver_dl_cc + 185;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc05 = double_integrator_QP_solver_ds_cc + 185;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl05 = double_integrator_QP_solver_ccrhs + 185;
int double_integrator_QP_solver_ubIdx05[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub05 = double_integrator_QP_solver_l + 192;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub05 = double_integrator_QP_solver_s + 192;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub05 = double_integrator_QP_solver_lbys + 192;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub05[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff05 = double_integrator_QP_solver_dl_aff + 192;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff05 = double_integrator_QP_solver_ds_aff + 192;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc05 = double_integrator_QP_solver_dl_cc + 192;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc05 = double_integrator_QP_solver_ds_cc + 192;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub05 = double_integrator_QP_solver_ccrhs + 192;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp05 = double_integrator_QP_solver_s + 198;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp05 = double_integrator_QP_solver_l + 198;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp05 = double_integrator_QP_solver_lbys + 198;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff05 = double_integrator_QP_solver_dl_aff + 198;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff05 = double_integrator_QP_solver_ds_aff + 198;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc05 = double_integrator_QP_solver_dl_cc + 198;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc05 = double_integrator_QP_solver_ds_cc + 198;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp05 = double_integrator_QP_solver_ccrhs + 198;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip05[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi05[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W05[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd05[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd05[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z06 = double_integrator_QP_solver_z + 72;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff06 = double_integrator_QP_solver_dz_aff + 72;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc06 = double_integrator_QP_solver_dz_cc + 72;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd06 = double_integrator_QP_solver_rd + 72;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd06[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost06 = double_integrator_QP_solver_grad_cost + 72;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq06 = double_integrator_QP_solver_grad_eq + 72;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq06 = double_integrator_QP_solver_grad_ineq + 72;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv06[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v06 = double_integrator_QP_solver_v + 17;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re06[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta06[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc06[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff06 = double_integrator_QP_solver_dv_aff + 17;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc06 = double_integrator_QP_solver_dv_cc + 17;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V06[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd06[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld06[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy06[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy06[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c06[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx06[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb06 = double_integrator_QP_solver_l + 222;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb06 = double_integrator_QP_solver_s + 222;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb06 = double_integrator_QP_solver_lbys + 222;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb06[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff06 = double_integrator_QP_solver_dl_aff + 222;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff06 = double_integrator_QP_solver_ds_aff + 222;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc06 = double_integrator_QP_solver_dl_cc + 222;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc06 = double_integrator_QP_solver_ds_cc + 222;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl06 = double_integrator_QP_solver_ccrhs + 222;
int double_integrator_QP_solver_ubIdx06[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub06 = double_integrator_QP_solver_l + 229;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub06 = double_integrator_QP_solver_s + 229;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub06 = double_integrator_QP_solver_lbys + 229;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub06[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff06 = double_integrator_QP_solver_dl_aff + 229;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff06 = double_integrator_QP_solver_ds_aff + 229;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc06 = double_integrator_QP_solver_dl_cc + 229;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc06 = double_integrator_QP_solver_ds_cc + 229;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub06 = double_integrator_QP_solver_ccrhs + 229;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp06 = double_integrator_QP_solver_s + 235;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp06 = double_integrator_QP_solver_l + 235;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp06 = double_integrator_QP_solver_lbys + 235;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff06 = double_integrator_QP_solver_dl_aff + 235;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff06 = double_integrator_QP_solver_ds_aff + 235;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc06 = double_integrator_QP_solver_dl_cc + 235;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc06 = double_integrator_QP_solver_ds_cc + 235;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp06 = double_integrator_QP_solver_ccrhs + 235;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip06[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi06[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W06[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd06[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd06[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z07 = double_integrator_QP_solver_z + 84;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff07 = double_integrator_QP_solver_dz_aff + 84;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc07 = double_integrator_QP_solver_dz_cc + 84;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd07 = double_integrator_QP_solver_rd + 84;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd07[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost07 = double_integrator_QP_solver_grad_cost + 84;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq07 = double_integrator_QP_solver_grad_eq + 84;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq07 = double_integrator_QP_solver_grad_ineq + 84;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv07[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v07 = double_integrator_QP_solver_v + 18;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re07[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta07[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc07[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff07 = double_integrator_QP_solver_dv_aff + 18;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc07 = double_integrator_QP_solver_dv_cc + 18;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V07[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd07[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld07[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy07[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy07[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c07[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx07[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb07 = double_integrator_QP_solver_l + 259;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb07 = double_integrator_QP_solver_s + 259;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb07 = double_integrator_QP_solver_lbys + 259;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb07[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff07 = double_integrator_QP_solver_dl_aff + 259;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff07 = double_integrator_QP_solver_ds_aff + 259;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc07 = double_integrator_QP_solver_dl_cc + 259;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc07 = double_integrator_QP_solver_ds_cc + 259;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl07 = double_integrator_QP_solver_ccrhs + 259;
int double_integrator_QP_solver_ubIdx07[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub07 = double_integrator_QP_solver_l + 266;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub07 = double_integrator_QP_solver_s + 266;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub07 = double_integrator_QP_solver_lbys + 266;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub07[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff07 = double_integrator_QP_solver_dl_aff + 266;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff07 = double_integrator_QP_solver_ds_aff + 266;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc07 = double_integrator_QP_solver_dl_cc + 266;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc07 = double_integrator_QP_solver_ds_cc + 266;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub07 = double_integrator_QP_solver_ccrhs + 266;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp07 = double_integrator_QP_solver_s + 272;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp07 = double_integrator_QP_solver_l + 272;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp07 = double_integrator_QP_solver_lbys + 272;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff07 = double_integrator_QP_solver_dl_aff + 272;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff07 = double_integrator_QP_solver_ds_aff + 272;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc07 = double_integrator_QP_solver_dl_cc + 272;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc07 = double_integrator_QP_solver_ds_cc + 272;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp07 = double_integrator_QP_solver_ccrhs + 272;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip07[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi07[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W07[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd07[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd07[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z08 = double_integrator_QP_solver_z + 96;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff08 = double_integrator_QP_solver_dz_aff + 96;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc08 = double_integrator_QP_solver_dz_cc + 96;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd08 = double_integrator_QP_solver_rd + 96;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd08[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost08 = double_integrator_QP_solver_grad_cost + 96;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq08 = double_integrator_QP_solver_grad_eq + 96;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq08 = double_integrator_QP_solver_grad_ineq + 96;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv08[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v08 = double_integrator_QP_solver_v + 19;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re08[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta08[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc08[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff08 = double_integrator_QP_solver_dv_aff + 19;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc08 = double_integrator_QP_solver_dv_cc + 19;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V08[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd08[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld08[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy08[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy08[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c08[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx08[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb08 = double_integrator_QP_solver_l + 296;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb08 = double_integrator_QP_solver_s + 296;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb08 = double_integrator_QP_solver_lbys + 296;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb08[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff08 = double_integrator_QP_solver_dl_aff + 296;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff08 = double_integrator_QP_solver_ds_aff + 296;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc08 = double_integrator_QP_solver_dl_cc + 296;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc08 = double_integrator_QP_solver_ds_cc + 296;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl08 = double_integrator_QP_solver_ccrhs + 296;
int double_integrator_QP_solver_ubIdx08[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub08 = double_integrator_QP_solver_l + 303;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub08 = double_integrator_QP_solver_s + 303;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub08 = double_integrator_QP_solver_lbys + 303;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub08[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff08 = double_integrator_QP_solver_dl_aff + 303;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff08 = double_integrator_QP_solver_ds_aff + 303;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc08 = double_integrator_QP_solver_dl_cc + 303;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc08 = double_integrator_QP_solver_ds_cc + 303;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub08 = double_integrator_QP_solver_ccrhs + 303;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp08 = double_integrator_QP_solver_s + 309;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp08 = double_integrator_QP_solver_l + 309;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp08 = double_integrator_QP_solver_lbys + 309;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff08 = double_integrator_QP_solver_dl_aff + 309;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff08 = double_integrator_QP_solver_ds_aff + 309;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc08 = double_integrator_QP_solver_dl_cc + 309;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc08 = double_integrator_QP_solver_ds_cc + 309;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp08 = double_integrator_QP_solver_ccrhs + 309;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip08[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi08[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W08[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd08[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd08[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z09 = double_integrator_QP_solver_z + 108;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff09 = double_integrator_QP_solver_dz_aff + 108;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc09 = double_integrator_QP_solver_dz_cc + 108;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd09 = double_integrator_QP_solver_rd + 108;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd09[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost09 = double_integrator_QP_solver_grad_cost + 108;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq09 = double_integrator_QP_solver_grad_eq + 108;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq09 = double_integrator_QP_solver_grad_ineq + 108;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv09[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v09 = double_integrator_QP_solver_v + 20;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re09[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta09[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc09[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff09 = double_integrator_QP_solver_dv_aff + 20;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc09 = double_integrator_QP_solver_dv_cc + 20;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V09[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd09[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld09[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy09[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy09[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c09[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx09[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb09 = double_integrator_QP_solver_l + 333;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb09 = double_integrator_QP_solver_s + 333;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb09 = double_integrator_QP_solver_lbys + 333;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb09[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff09 = double_integrator_QP_solver_dl_aff + 333;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff09 = double_integrator_QP_solver_ds_aff + 333;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc09 = double_integrator_QP_solver_dl_cc + 333;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc09 = double_integrator_QP_solver_ds_cc + 333;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl09 = double_integrator_QP_solver_ccrhs + 333;
int double_integrator_QP_solver_ubIdx09[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub09 = double_integrator_QP_solver_l + 340;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub09 = double_integrator_QP_solver_s + 340;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub09 = double_integrator_QP_solver_lbys + 340;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub09[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff09 = double_integrator_QP_solver_dl_aff + 340;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff09 = double_integrator_QP_solver_ds_aff + 340;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc09 = double_integrator_QP_solver_dl_cc + 340;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc09 = double_integrator_QP_solver_ds_cc + 340;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub09 = double_integrator_QP_solver_ccrhs + 340;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp09 = double_integrator_QP_solver_s + 346;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp09 = double_integrator_QP_solver_l + 346;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp09 = double_integrator_QP_solver_lbys + 346;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff09 = double_integrator_QP_solver_dl_aff + 346;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff09 = double_integrator_QP_solver_ds_aff + 346;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc09 = double_integrator_QP_solver_dl_cc + 346;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc09 = double_integrator_QP_solver_ds_cc + 346;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp09 = double_integrator_QP_solver_ccrhs + 346;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip09[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi09[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W09[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd09[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd09[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z10 = double_integrator_QP_solver_z + 120;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff10 = double_integrator_QP_solver_dz_aff + 120;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc10 = double_integrator_QP_solver_dz_cc + 120;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd10 = double_integrator_QP_solver_rd + 120;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd10[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost10 = double_integrator_QP_solver_grad_cost + 120;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq10 = double_integrator_QP_solver_grad_eq + 120;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq10 = double_integrator_QP_solver_grad_ineq + 120;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv10[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v10 = double_integrator_QP_solver_v + 21;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re10[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta10[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc10[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff10 = double_integrator_QP_solver_dv_aff + 21;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc10 = double_integrator_QP_solver_dv_cc + 21;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V10[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd10[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld10[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy10[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy10[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c10[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx10[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb10 = double_integrator_QP_solver_l + 370;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb10 = double_integrator_QP_solver_s + 370;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb10 = double_integrator_QP_solver_lbys + 370;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb10[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff10 = double_integrator_QP_solver_dl_aff + 370;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff10 = double_integrator_QP_solver_ds_aff + 370;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc10 = double_integrator_QP_solver_dl_cc + 370;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc10 = double_integrator_QP_solver_ds_cc + 370;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl10 = double_integrator_QP_solver_ccrhs + 370;
int double_integrator_QP_solver_ubIdx10[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub10 = double_integrator_QP_solver_l + 377;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub10 = double_integrator_QP_solver_s + 377;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub10 = double_integrator_QP_solver_lbys + 377;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub10[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff10 = double_integrator_QP_solver_dl_aff + 377;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff10 = double_integrator_QP_solver_ds_aff + 377;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc10 = double_integrator_QP_solver_dl_cc + 377;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc10 = double_integrator_QP_solver_ds_cc + 377;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub10 = double_integrator_QP_solver_ccrhs + 377;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp10 = double_integrator_QP_solver_s + 383;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp10 = double_integrator_QP_solver_l + 383;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp10 = double_integrator_QP_solver_lbys + 383;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff10 = double_integrator_QP_solver_dl_aff + 383;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff10 = double_integrator_QP_solver_ds_aff + 383;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc10 = double_integrator_QP_solver_dl_cc + 383;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc10 = double_integrator_QP_solver_ds_cc + 383;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp10 = double_integrator_QP_solver_ccrhs + 383;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip10[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi10[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W10[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd10[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd10[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z11 = double_integrator_QP_solver_z + 132;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff11 = double_integrator_QP_solver_dz_aff + 132;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc11 = double_integrator_QP_solver_dz_cc + 132;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd11 = double_integrator_QP_solver_rd + 132;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd11[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost11 = double_integrator_QP_solver_grad_cost + 132;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq11 = double_integrator_QP_solver_grad_eq + 132;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq11 = double_integrator_QP_solver_grad_ineq + 132;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv11[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v11 = double_integrator_QP_solver_v + 22;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re11[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta11[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc11[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff11 = double_integrator_QP_solver_dv_aff + 22;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc11 = double_integrator_QP_solver_dv_cc + 22;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V11[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd11[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld11[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy11[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy11[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c11[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx11[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb11 = double_integrator_QP_solver_l + 407;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb11 = double_integrator_QP_solver_s + 407;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb11 = double_integrator_QP_solver_lbys + 407;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb11[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff11 = double_integrator_QP_solver_dl_aff + 407;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff11 = double_integrator_QP_solver_ds_aff + 407;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc11 = double_integrator_QP_solver_dl_cc + 407;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc11 = double_integrator_QP_solver_ds_cc + 407;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl11 = double_integrator_QP_solver_ccrhs + 407;
int double_integrator_QP_solver_ubIdx11[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub11 = double_integrator_QP_solver_l + 414;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub11 = double_integrator_QP_solver_s + 414;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub11 = double_integrator_QP_solver_lbys + 414;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub11[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff11 = double_integrator_QP_solver_dl_aff + 414;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff11 = double_integrator_QP_solver_ds_aff + 414;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc11 = double_integrator_QP_solver_dl_cc + 414;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc11 = double_integrator_QP_solver_ds_cc + 414;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub11 = double_integrator_QP_solver_ccrhs + 414;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp11 = double_integrator_QP_solver_s + 420;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp11 = double_integrator_QP_solver_l + 420;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp11 = double_integrator_QP_solver_lbys + 420;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff11 = double_integrator_QP_solver_dl_aff + 420;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff11 = double_integrator_QP_solver_ds_aff + 420;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc11 = double_integrator_QP_solver_dl_cc + 420;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc11 = double_integrator_QP_solver_ds_cc + 420;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp11 = double_integrator_QP_solver_ccrhs + 420;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip11[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi11[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W11[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd11[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd11[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z12 = double_integrator_QP_solver_z + 144;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff12 = double_integrator_QP_solver_dz_aff + 144;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc12 = double_integrator_QP_solver_dz_cc + 144;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd12 = double_integrator_QP_solver_rd + 144;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd12[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost12 = double_integrator_QP_solver_grad_cost + 144;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq12 = double_integrator_QP_solver_grad_eq + 144;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq12 = double_integrator_QP_solver_grad_ineq + 144;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv12[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v12 = double_integrator_QP_solver_v + 23;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re12[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta12[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc12[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff12 = double_integrator_QP_solver_dv_aff + 23;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc12 = double_integrator_QP_solver_dv_cc + 23;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V12[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd12[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld12[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy12[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy12[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c12[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx12[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb12 = double_integrator_QP_solver_l + 444;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb12 = double_integrator_QP_solver_s + 444;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb12 = double_integrator_QP_solver_lbys + 444;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb12[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff12 = double_integrator_QP_solver_dl_aff + 444;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff12 = double_integrator_QP_solver_ds_aff + 444;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc12 = double_integrator_QP_solver_dl_cc + 444;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc12 = double_integrator_QP_solver_ds_cc + 444;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl12 = double_integrator_QP_solver_ccrhs + 444;
int double_integrator_QP_solver_ubIdx12[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub12 = double_integrator_QP_solver_l + 451;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub12 = double_integrator_QP_solver_s + 451;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub12 = double_integrator_QP_solver_lbys + 451;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub12[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff12 = double_integrator_QP_solver_dl_aff + 451;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff12 = double_integrator_QP_solver_ds_aff + 451;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc12 = double_integrator_QP_solver_dl_cc + 451;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc12 = double_integrator_QP_solver_ds_cc + 451;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub12 = double_integrator_QP_solver_ccrhs + 451;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp12 = double_integrator_QP_solver_s + 457;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp12 = double_integrator_QP_solver_l + 457;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp12 = double_integrator_QP_solver_lbys + 457;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff12 = double_integrator_QP_solver_dl_aff + 457;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff12 = double_integrator_QP_solver_ds_aff + 457;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc12 = double_integrator_QP_solver_dl_cc + 457;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc12 = double_integrator_QP_solver_ds_cc + 457;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp12 = double_integrator_QP_solver_ccrhs + 457;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip12[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi12[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W12[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd12[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd12[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z13 = double_integrator_QP_solver_z + 156;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff13 = double_integrator_QP_solver_dz_aff + 156;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc13 = double_integrator_QP_solver_dz_cc + 156;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd13 = double_integrator_QP_solver_rd + 156;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd13[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost13 = double_integrator_QP_solver_grad_cost + 156;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq13 = double_integrator_QP_solver_grad_eq + 156;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq13 = double_integrator_QP_solver_grad_ineq + 156;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv13[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v13 = double_integrator_QP_solver_v + 24;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re13[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta13[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc13[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff13 = double_integrator_QP_solver_dv_aff + 24;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc13 = double_integrator_QP_solver_dv_cc + 24;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V13[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd13[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld13[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy13[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy13[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c13[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx13[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb13 = double_integrator_QP_solver_l + 481;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb13 = double_integrator_QP_solver_s + 481;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb13 = double_integrator_QP_solver_lbys + 481;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb13[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff13 = double_integrator_QP_solver_dl_aff + 481;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff13 = double_integrator_QP_solver_ds_aff + 481;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc13 = double_integrator_QP_solver_dl_cc + 481;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc13 = double_integrator_QP_solver_ds_cc + 481;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl13 = double_integrator_QP_solver_ccrhs + 481;
int double_integrator_QP_solver_ubIdx13[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub13 = double_integrator_QP_solver_l + 488;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub13 = double_integrator_QP_solver_s + 488;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub13 = double_integrator_QP_solver_lbys + 488;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub13[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff13 = double_integrator_QP_solver_dl_aff + 488;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff13 = double_integrator_QP_solver_ds_aff + 488;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc13 = double_integrator_QP_solver_dl_cc + 488;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc13 = double_integrator_QP_solver_ds_cc + 488;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub13 = double_integrator_QP_solver_ccrhs + 488;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp13 = double_integrator_QP_solver_s + 494;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp13 = double_integrator_QP_solver_l + 494;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp13 = double_integrator_QP_solver_lbys + 494;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff13 = double_integrator_QP_solver_dl_aff + 494;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff13 = double_integrator_QP_solver_ds_aff + 494;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc13 = double_integrator_QP_solver_dl_cc + 494;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc13 = double_integrator_QP_solver_ds_cc + 494;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp13 = double_integrator_QP_solver_ccrhs + 494;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip13[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi13[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W13[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd13[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd13[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z14 = double_integrator_QP_solver_z + 168;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff14 = double_integrator_QP_solver_dz_aff + 168;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc14 = double_integrator_QP_solver_dz_cc + 168;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd14 = double_integrator_QP_solver_rd + 168;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd14[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost14 = double_integrator_QP_solver_grad_cost + 168;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq14 = double_integrator_QP_solver_grad_eq + 168;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq14 = double_integrator_QP_solver_grad_ineq + 168;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv14[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v14 = double_integrator_QP_solver_v + 25;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re14[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta14[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc14[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff14 = double_integrator_QP_solver_dv_aff + 25;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc14 = double_integrator_QP_solver_dv_cc + 25;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V14[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd14[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld14[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy14[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy14[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c14[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx14[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb14 = double_integrator_QP_solver_l + 518;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb14 = double_integrator_QP_solver_s + 518;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb14 = double_integrator_QP_solver_lbys + 518;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb14[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff14 = double_integrator_QP_solver_dl_aff + 518;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff14 = double_integrator_QP_solver_ds_aff + 518;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc14 = double_integrator_QP_solver_dl_cc + 518;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc14 = double_integrator_QP_solver_ds_cc + 518;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl14 = double_integrator_QP_solver_ccrhs + 518;
int double_integrator_QP_solver_ubIdx14[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub14 = double_integrator_QP_solver_l + 525;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub14 = double_integrator_QP_solver_s + 525;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub14 = double_integrator_QP_solver_lbys + 525;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub14[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff14 = double_integrator_QP_solver_dl_aff + 525;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff14 = double_integrator_QP_solver_ds_aff + 525;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc14 = double_integrator_QP_solver_dl_cc + 525;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc14 = double_integrator_QP_solver_ds_cc + 525;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub14 = double_integrator_QP_solver_ccrhs + 525;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp14 = double_integrator_QP_solver_s + 531;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp14 = double_integrator_QP_solver_l + 531;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp14 = double_integrator_QP_solver_lbys + 531;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff14 = double_integrator_QP_solver_dl_aff + 531;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff14 = double_integrator_QP_solver_ds_aff + 531;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc14 = double_integrator_QP_solver_dl_cc + 531;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc14 = double_integrator_QP_solver_ds_cc + 531;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp14 = double_integrator_QP_solver_ccrhs + 531;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip14[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi14[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W14[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd14[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd14[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z15 = double_integrator_QP_solver_z + 180;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff15 = double_integrator_QP_solver_dz_aff + 180;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc15 = double_integrator_QP_solver_dz_cc + 180;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd15 = double_integrator_QP_solver_rd + 180;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd15[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost15 = double_integrator_QP_solver_grad_cost + 180;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq15 = double_integrator_QP_solver_grad_eq + 180;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq15 = double_integrator_QP_solver_grad_ineq + 180;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv15[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v15 = double_integrator_QP_solver_v + 26;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re15[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta15[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc15[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff15 = double_integrator_QP_solver_dv_aff + 26;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc15 = double_integrator_QP_solver_dv_cc + 26;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V15[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd15[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld15[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy15[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy15[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c15[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx15[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb15 = double_integrator_QP_solver_l + 555;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb15 = double_integrator_QP_solver_s + 555;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb15 = double_integrator_QP_solver_lbys + 555;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb15[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff15 = double_integrator_QP_solver_dl_aff + 555;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff15 = double_integrator_QP_solver_ds_aff + 555;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc15 = double_integrator_QP_solver_dl_cc + 555;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc15 = double_integrator_QP_solver_ds_cc + 555;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl15 = double_integrator_QP_solver_ccrhs + 555;
int double_integrator_QP_solver_ubIdx15[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub15 = double_integrator_QP_solver_l + 562;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub15 = double_integrator_QP_solver_s + 562;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub15 = double_integrator_QP_solver_lbys + 562;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub15[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff15 = double_integrator_QP_solver_dl_aff + 562;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff15 = double_integrator_QP_solver_ds_aff + 562;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc15 = double_integrator_QP_solver_dl_cc + 562;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc15 = double_integrator_QP_solver_ds_cc + 562;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub15 = double_integrator_QP_solver_ccrhs + 562;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp15 = double_integrator_QP_solver_s + 568;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp15 = double_integrator_QP_solver_l + 568;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp15 = double_integrator_QP_solver_lbys + 568;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff15 = double_integrator_QP_solver_dl_aff + 568;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff15 = double_integrator_QP_solver_ds_aff + 568;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc15 = double_integrator_QP_solver_dl_cc + 568;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc15 = double_integrator_QP_solver_ds_cc + 568;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp15 = double_integrator_QP_solver_ccrhs + 568;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip15[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi15[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W15[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd15[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd15[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z16 = double_integrator_QP_solver_z + 192;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff16 = double_integrator_QP_solver_dz_aff + 192;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc16 = double_integrator_QP_solver_dz_cc + 192;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd16 = double_integrator_QP_solver_rd + 192;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd16[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost16 = double_integrator_QP_solver_grad_cost + 192;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq16 = double_integrator_QP_solver_grad_eq + 192;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq16 = double_integrator_QP_solver_grad_ineq + 192;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv16[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v16 = double_integrator_QP_solver_v + 27;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re16[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta16[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc16[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff16 = double_integrator_QP_solver_dv_aff + 27;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc16 = double_integrator_QP_solver_dv_cc + 27;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V16[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd16[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld16[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy16[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy16[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c16[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx16[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb16 = double_integrator_QP_solver_l + 592;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb16 = double_integrator_QP_solver_s + 592;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb16 = double_integrator_QP_solver_lbys + 592;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb16[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff16 = double_integrator_QP_solver_dl_aff + 592;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff16 = double_integrator_QP_solver_ds_aff + 592;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc16 = double_integrator_QP_solver_dl_cc + 592;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc16 = double_integrator_QP_solver_ds_cc + 592;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl16 = double_integrator_QP_solver_ccrhs + 592;
int double_integrator_QP_solver_ubIdx16[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub16 = double_integrator_QP_solver_l + 599;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub16 = double_integrator_QP_solver_s + 599;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub16 = double_integrator_QP_solver_lbys + 599;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub16[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff16 = double_integrator_QP_solver_dl_aff + 599;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff16 = double_integrator_QP_solver_ds_aff + 599;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc16 = double_integrator_QP_solver_dl_cc + 599;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc16 = double_integrator_QP_solver_ds_cc + 599;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub16 = double_integrator_QP_solver_ccrhs + 599;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp16 = double_integrator_QP_solver_s + 605;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp16 = double_integrator_QP_solver_l + 605;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp16 = double_integrator_QP_solver_lbys + 605;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff16 = double_integrator_QP_solver_dl_aff + 605;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff16 = double_integrator_QP_solver_ds_aff + 605;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc16 = double_integrator_QP_solver_dl_cc + 605;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc16 = double_integrator_QP_solver_ds_cc + 605;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp16 = double_integrator_QP_solver_ccrhs + 605;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip16[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi16[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W16[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd16[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd16[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z17 = double_integrator_QP_solver_z + 204;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff17 = double_integrator_QP_solver_dz_aff + 204;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc17 = double_integrator_QP_solver_dz_cc + 204;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd17 = double_integrator_QP_solver_rd + 204;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd17[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost17 = double_integrator_QP_solver_grad_cost + 204;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq17 = double_integrator_QP_solver_grad_eq + 204;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq17 = double_integrator_QP_solver_grad_ineq + 204;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv17[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v17 = double_integrator_QP_solver_v + 28;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re17[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta17[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc17[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff17 = double_integrator_QP_solver_dv_aff + 28;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc17 = double_integrator_QP_solver_dv_cc + 28;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V17[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd17[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld17[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy17[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy17[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c17[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx17[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb17 = double_integrator_QP_solver_l + 629;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb17 = double_integrator_QP_solver_s + 629;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb17 = double_integrator_QP_solver_lbys + 629;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb17[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff17 = double_integrator_QP_solver_dl_aff + 629;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff17 = double_integrator_QP_solver_ds_aff + 629;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc17 = double_integrator_QP_solver_dl_cc + 629;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc17 = double_integrator_QP_solver_ds_cc + 629;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl17 = double_integrator_QP_solver_ccrhs + 629;
int double_integrator_QP_solver_ubIdx17[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub17 = double_integrator_QP_solver_l + 636;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub17 = double_integrator_QP_solver_s + 636;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub17 = double_integrator_QP_solver_lbys + 636;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub17[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff17 = double_integrator_QP_solver_dl_aff + 636;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff17 = double_integrator_QP_solver_ds_aff + 636;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc17 = double_integrator_QP_solver_dl_cc + 636;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc17 = double_integrator_QP_solver_ds_cc + 636;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub17 = double_integrator_QP_solver_ccrhs + 636;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp17 = double_integrator_QP_solver_s + 642;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp17 = double_integrator_QP_solver_l + 642;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp17 = double_integrator_QP_solver_lbys + 642;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff17 = double_integrator_QP_solver_dl_aff + 642;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff17 = double_integrator_QP_solver_ds_aff + 642;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc17 = double_integrator_QP_solver_dl_cc + 642;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc17 = double_integrator_QP_solver_ds_cc + 642;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp17 = double_integrator_QP_solver_ccrhs + 642;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip17[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi17[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W17[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd17[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd17[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z18 = double_integrator_QP_solver_z + 216;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff18 = double_integrator_QP_solver_dz_aff + 216;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc18 = double_integrator_QP_solver_dz_cc + 216;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd18 = double_integrator_QP_solver_rd + 216;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd18[12];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost18 = double_integrator_QP_solver_grad_cost + 216;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq18 = double_integrator_QP_solver_grad_eq + 216;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq18 = double_integrator_QP_solver_grad_ineq + 216;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv18[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_C18[60] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v18 = double_integrator_QP_solver_v + 29;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re18[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta18[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc18[1];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff18 = double_integrator_QP_solver_dv_aff + 29;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc18 = double_integrator_QP_solver_dv_cc + 29;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V18[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd18[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld18[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy18[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy18[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c18[1] = {0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx18[7] = {0, 1, 2, 3, 4, 5, 6};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb18 = double_integrator_QP_solver_l + 666;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb18 = double_integrator_QP_solver_s + 666;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb18 = double_integrator_QP_solver_lbys + 666;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb18[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff18 = double_integrator_QP_solver_dl_aff + 666;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff18 = double_integrator_QP_solver_ds_aff + 666;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc18 = double_integrator_QP_solver_dl_cc + 666;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc18 = double_integrator_QP_solver_ds_cc + 666;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl18 = double_integrator_QP_solver_ccrhs + 666;
int double_integrator_QP_solver_ubIdx18[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub18 = double_integrator_QP_solver_l + 673;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub18 = double_integrator_QP_solver_s + 673;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub18 = double_integrator_QP_solver_lbys + 673;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub18[6];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff18 = double_integrator_QP_solver_dl_aff + 673;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff18 = double_integrator_QP_solver_ds_aff + 673;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc18 = double_integrator_QP_solver_dl_cc + 673;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc18 = double_integrator_QP_solver_ds_cc + 673;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub18 = double_integrator_QP_solver_ccrhs + 673;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp18 = double_integrator_QP_solver_s + 679;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp18 = double_integrator_QP_solver_l + 679;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp18 = double_integrator_QP_solver_lbys + 679;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff18 = double_integrator_QP_solver_dl_aff + 679;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff18 = double_integrator_QP_solver_ds_aff + 679;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc18 = double_integrator_QP_solver_dl_cc + 679;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc18 = double_integrator_QP_solver_ds_cc + 679;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp18 = double_integrator_QP_solver_ccrhs + 679;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip18[24];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi18[78];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W18[12];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd18[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd18[1];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_H19[5] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z19 = double_integrator_QP_solver_z + 228;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff19 = double_integrator_QP_solver_dz_aff + 228;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc19 = double_integrator_QP_solver_dz_cc + 228;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd19 = double_integrator_QP_solver_rd + 228;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd19[5];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost19 = double_integrator_QP_solver_grad_cost + 228;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq19 = double_integrator_QP_solver_grad_eq + 228;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq19 = double_integrator_QP_solver_grad_ineq + 228;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv19[5];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v19 = double_integrator_QP_solver_v + 30;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re19[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta19[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc19[5];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff19 = double_integrator_QP_solver_dv_aff + 30;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc19 = double_integrator_QP_solver_dv_cc + 30;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V19[25];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd19[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld19[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy19[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy19[5];
int double_integrator_QP_solver_lbIdx19[4] = {0, 1, 2, 3};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb19 = double_integrator_QP_solver_l + 703;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb19 = double_integrator_QP_solver_s + 703;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb19 = double_integrator_QP_solver_lbys + 703;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb19[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff19 = double_integrator_QP_solver_dl_aff + 703;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff19 = double_integrator_QP_solver_ds_aff + 703;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc19 = double_integrator_QP_solver_dl_cc + 703;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc19 = double_integrator_QP_solver_ds_cc + 703;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl19 = double_integrator_QP_solver_ccrhs + 703;
int double_integrator_QP_solver_ubIdx19[4] = {0, 1, 2, 3};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub19 = double_integrator_QP_solver_l + 707;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub19 = double_integrator_QP_solver_s + 707;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub19 = double_integrator_QP_solver_lbys + 707;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub19[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff19 = double_integrator_QP_solver_dl_aff + 707;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff19 = double_integrator_QP_solver_ds_aff + 707;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc19 = double_integrator_QP_solver_dl_cc + 707;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc19 = double_integrator_QP_solver_ds_cc + 707;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub19 = double_integrator_QP_solver_ccrhs + 707;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp19 = double_integrator_QP_solver_s + 711;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp19 = double_integrator_QP_solver_l + 711;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp19 = double_integrator_QP_solver_lbys + 711;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff19 = double_integrator_QP_solver_dl_aff + 711;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff19 = double_integrator_QP_solver_ds_aff + 711;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc19 = double_integrator_QP_solver_dl_cc + 711;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc19 = double_integrator_QP_solver_ds_cc + 711;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp19 = double_integrator_QP_solver_ccrhs + 711;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip19[10];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi19[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_D19[5] = {1.0000000000000000E+000, 
1.0000000000000000E+000, 
1.0000000000000000E+000, 
1.0000000000000000E+000, 
0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W19[25];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd19[5];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd19[5];
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
double_integrator_QP_solver_LA_INITIALIZEVECTOR_233(double_integrator_QP_solver_z, 0);
double_integrator_QP_solver_LA_INITIALIZEVECTOR_35(double_integrator_QP_solver_v, 1);
double_integrator_QP_solver_LA_INITIALIZEVECTOR_721(double_integrator_QP_solver_l, 10);
double_integrator_QP_solver_LA_INITIALIZEVECTOR_721(double_integrator_QP_solver_s, 10);
info->mu = 0;
double_integrator_QP_solver_LA_DOTACC_721(double_integrator_QP_solver_l, double_integrator_QP_solver_s, &info->mu);
info->mu /= 721;
PRINTTEXT("This is double_integrator_QP_solver, a solver generated by FORCES (forces.ethz.ch).\n");
PRINTTEXT("(c) Alexander Domahidi, Automatic Control Laboratory, ETH Zurich, 2011-2014.\n");
PRINTTEXT("\n  #it  res_eq   res_ineq     pobj         dobj       dgap     rdgap     mu\n");
PRINTTEXT("  ---------------------------------------------------------------------------\n");
while( 1 ){
info->pobj = 0;
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f1, double_integrator_QP_solver_z00, double_integrator_QP_solver_grad_cost00, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost00, "double_integrator_QP_solver_grad_cost00");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f2, double_integrator_QP_solver_z01, double_integrator_QP_solver_grad_cost01, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost01, "double_integrator_QP_solver_grad_cost01");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f3, double_integrator_QP_solver_z02, double_integrator_QP_solver_grad_cost02, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost02, "double_integrator_QP_solver_grad_cost02");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f4, double_integrator_QP_solver_z03, double_integrator_QP_solver_grad_cost03, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost03, "double_integrator_QP_solver_grad_cost03");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f5, double_integrator_QP_solver_z04, double_integrator_QP_solver_grad_cost04, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost04, "double_integrator_QP_solver_grad_cost04");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f6, double_integrator_QP_solver_z05, double_integrator_QP_solver_grad_cost05, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost05, "double_integrator_QP_solver_grad_cost05");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f7, double_integrator_QP_solver_z06, double_integrator_QP_solver_grad_cost06, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost06, "double_integrator_QP_solver_grad_cost06");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f8, double_integrator_QP_solver_z07, double_integrator_QP_solver_grad_cost07, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost07, "double_integrator_QP_solver_grad_cost07");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f9, double_integrator_QP_solver_z08, double_integrator_QP_solver_grad_cost08, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost08, "double_integrator_QP_solver_grad_cost08");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f10, double_integrator_QP_solver_z09, double_integrator_QP_solver_grad_cost09, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost09, "double_integrator_QP_solver_grad_cost09");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f11, double_integrator_QP_solver_z10, double_integrator_QP_solver_grad_cost10, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost10, "double_integrator_QP_solver_grad_cost10");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f12, double_integrator_QP_solver_z11, double_integrator_QP_solver_grad_cost11, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost11, "double_integrator_QP_solver_grad_cost11");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f13, double_integrator_QP_solver_z12, double_integrator_QP_solver_grad_cost12, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost12, "double_integrator_QP_solver_grad_cost12");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f14, double_integrator_QP_solver_z13, double_integrator_QP_solver_grad_cost13, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost13, "double_integrator_QP_solver_grad_cost13");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f15, double_integrator_QP_solver_z14, double_integrator_QP_solver_grad_cost14, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost14, "double_integrator_QP_solver_grad_cost14");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f16, double_integrator_QP_solver_z15, double_integrator_QP_solver_grad_cost15, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost15, "double_integrator_QP_solver_grad_cost15");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f17, double_integrator_QP_solver_z16, double_integrator_QP_solver_grad_cost16, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost16, "double_integrator_QP_solver_grad_cost16");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f18, double_integrator_QP_solver_z17, double_integrator_QP_solver_grad_cost17, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost17, "double_integrator_QP_solver_grad_cost17");
double_integrator_QP_solver_LA_DIAG_QUADFCN_12(double_integrator_QP_solver_H00, params->f19, double_integrator_QP_solver_z18, double_integrator_QP_solver_grad_cost18, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_cost18, "double_integrator_QP_solver_grad_cost18");
double_integrator_QP_solver_LA_DIAG_QUADFCN_5(double_integrator_QP_solver_H19, params->f20, double_integrator_QP_solver_z19, double_integrator_QP_solver_grad_cost19, &info->pobj);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_grad_cost19, "double_integrator_QP_solver_grad_cost19");
PRINTTEXT("pobj = %6.4f\n", info->pobj);
info->res_eq = 0;
info->dgap = 0;
double_integrator_QP_solver_LA_DIAGZERO_MVMSUB6_12(double_integrator_QP_solver_D00, double_integrator_QP_solver_z00, params->c1, double_integrator_QP_solver_v00, double_integrator_QP_solver_re00, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_v00, "double_integrator_QP_solver_v00");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_re00, "double_integrator_QP_solver_re00");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z00, double_integrator_QP_solver_D01, double_integrator_QP_solver_z01, double_integrator_QP_solver_c01, double_integrator_QP_solver_v01, double_integrator_QP_solver_re01, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v01, "double_integrator_QP_solver_v01");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re01, "double_integrator_QP_solver_re01");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z01, double_integrator_QP_solver_D01, double_integrator_QP_solver_z02, double_integrator_QP_solver_c02, double_integrator_QP_solver_v02, double_integrator_QP_solver_re02, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v02, "double_integrator_QP_solver_v02");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re02, "double_integrator_QP_solver_re02");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z02, double_integrator_QP_solver_D01, double_integrator_QP_solver_z03, double_integrator_QP_solver_c03, double_integrator_QP_solver_v03, double_integrator_QP_solver_re03, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v03, "double_integrator_QP_solver_v03");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re03, "double_integrator_QP_solver_re03");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z03, double_integrator_QP_solver_D01, double_integrator_QP_solver_z04, double_integrator_QP_solver_c04, double_integrator_QP_solver_v04, double_integrator_QP_solver_re04, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v04, "double_integrator_QP_solver_v04");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re04, "double_integrator_QP_solver_re04");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z04, double_integrator_QP_solver_D01, double_integrator_QP_solver_z05, double_integrator_QP_solver_c05, double_integrator_QP_solver_v05, double_integrator_QP_solver_re05, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v05, "double_integrator_QP_solver_v05");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re05, "double_integrator_QP_solver_re05");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z05, double_integrator_QP_solver_D01, double_integrator_QP_solver_z06, double_integrator_QP_solver_c06, double_integrator_QP_solver_v06, double_integrator_QP_solver_re06, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v06, "double_integrator_QP_solver_v06");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re06, "double_integrator_QP_solver_re06");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z06, double_integrator_QP_solver_D01, double_integrator_QP_solver_z07, double_integrator_QP_solver_c07, double_integrator_QP_solver_v07, double_integrator_QP_solver_re07, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v07, "double_integrator_QP_solver_v07");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re07, "double_integrator_QP_solver_re07");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z07, double_integrator_QP_solver_D01, double_integrator_QP_solver_z08, double_integrator_QP_solver_c08, double_integrator_QP_solver_v08, double_integrator_QP_solver_re08, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v08, "double_integrator_QP_solver_v08");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re08, "double_integrator_QP_solver_re08");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z08, double_integrator_QP_solver_D01, double_integrator_QP_solver_z09, double_integrator_QP_solver_c09, double_integrator_QP_solver_v09, double_integrator_QP_solver_re09, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v09, "double_integrator_QP_solver_v09");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re09, "double_integrator_QP_solver_re09");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z09, double_integrator_QP_solver_D01, double_integrator_QP_solver_z10, double_integrator_QP_solver_c10, double_integrator_QP_solver_v10, double_integrator_QP_solver_re10, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v10, "double_integrator_QP_solver_v10");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re10, "double_integrator_QP_solver_re10");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z10, double_integrator_QP_solver_D01, double_integrator_QP_solver_z11, double_integrator_QP_solver_c11, double_integrator_QP_solver_v11, double_integrator_QP_solver_re11, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v11, "double_integrator_QP_solver_v11");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re11, "double_integrator_QP_solver_re11");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z11, double_integrator_QP_solver_D01, double_integrator_QP_solver_z12, double_integrator_QP_solver_c12, double_integrator_QP_solver_v12, double_integrator_QP_solver_re12, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v12, "double_integrator_QP_solver_v12");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re12, "double_integrator_QP_solver_re12");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z12, double_integrator_QP_solver_D01, double_integrator_QP_solver_z13, double_integrator_QP_solver_c13, double_integrator_QP_solver_v13, double_integrator_QP_solver_re13, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v13, "double_integrator_QP_solver_v13");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re13, "double_integrator_QP_solver_re13");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z13, double_integrator_QP_solver_D01, double_integrator_QP_solver_z14, double_integrator_QP_solver_c14, double_integrator_QP_solver_v14, double_integrator_QP_solver_re14, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v14, "double_integrator_QP_solver_v14");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re14, "double_integrator_QP_solver_re14");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z14, double_integrator_QP_solver_D01, double_integrator_QP_solver_z15, double_integrator_QP_solver_c15, double_integrator_QP_solver_v15, double_integrator_QP_solver_re15, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v15, "double_integrator_QP_solver_v15");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re15, "double_integrator_QP_solver_re15");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z15, double_integrator_QP_solver_D01, double_integrator_QP_solver_z16, double_integrator_QP_solver_c16, double_integrator_QP_solver_v16, double_integrator_QP_solver_re16, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v16, "double_integrator_QP_solver_v16");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re16, "double_integrator_QP_solver_re16");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z16, double_integrator_QP_solver_D01, double_integrator_QP_solver_z17, double_integrator_QP_solver_c17, double_integrator_QP_solver_v17, double_integrator_QP_solver_re17, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v17, "double_integrator_QP_solver_v17");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re17, "double_integrator_QP_solver_re17");
double_integrator_QP_solver_LA_DENSE_MVMSUB3_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_z17, double_integrator_QP_solver_D01, double_integrator_QP_solver_z18, double_integrator_QP_solver_c18, double_integrator_QP_solver_v18, double_integrator_QP_solver_re18, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_v18, "double_integrator_QP_solver_v18");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_re18, "double_integrator_QP_solver_re18");
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_5_12_5(double_integrator_QP_solver_C18, double_integrator_QP_solver_z18, double_integrator_QP_solver_D19, double_integrator_QP_solver_z19, params->c20, double_integrator_QP_solver_v19, double_integrator_QP_solver_re19, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_v19, "double_integrator_QP_solver_v19");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_re19, "double_integrator_QP_solver_re19");
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_v01, double_integrator_QP_solver_D00, double_integrator_QP_solver_v00, double_integrator_QP_solver_grad_eq00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq00, "double_integrator_QP_solver_grad_eq00");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_v02, double_integrator_QP_solver_D01, double_integrator_QP_solver_v01, double_integrator_QP_solver_grad_eq01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq01, "double_integrator_QP_solver_grad_eq01");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_v03, double_integrator_QP_solver_D01, double_integrator_QP_solver_v02, double_integrator_QP_solver_grad_eq02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq02, "double_integrator_QP_solver_grad_eq02");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_v04, double_integrator_QP_solver_D01, double_integrator_QP_solver_v03, double_integrator_QP_solver_grad_eq03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq03, "double_integrator_QP_solver_grad_eq03");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_v05, double_integrator_QP_solver_D01, double_integrator_QP_solver_v04, double_integrator_QP_solver_grad_eq04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq04, "double_integrator_QP_solver_grad_eq04");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_v06, double_integrator_QP_solver_D01, double_integrator_QP_solver_v05, double_integrator_QP_solver_grad_eq05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq05, "double_integrator_QP_solver_grad_eq05");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_v07, double_integrator_QP_solver_D01, double_integrator_QP_solver_v06, double_integrator_QP_solver_grad_eq06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq06, "double_integrator_QP_solver_grad_eq06");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_v08, double_integrator_QP_solver_D01, double_integrator_QP_solver_v07, double_integrator_QP_solver_grad_eq07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq07, "double_integrator_QP_solver_grad_eq07");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_v09, double_integrator_QP_solver_D01, double_integrator_QP_solver_v08, double_integrator_QP_solver_grad_eq08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq08, "double_integrator_QP_solver_grad_eq08");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_v10, double_integrator_QP_solver_D01, double_integrator_QP_solver_v09, double_integrator_QP_solver_grad_eq09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq09, "double_integrator_QP_solver_grad_eq09");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_v11, double_integrator_QP_solver_D01, double_integrator_QP_solver_v10, double_integrator_QP_solver_grad_eq10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq10, "double_integrator_QP_solver_grad_eq10");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_v12, double_integrator_QP_solver_D01, double_integrator_QP_solver_v11, double_integrator_QP_solver_grad_eq11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq11, "double_integrator_QP_solver_grad_eq11");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_v13, double_integrator_QP_solver_D01, double_integrator_QP_solver_v12, double_integrator_QP_solver_grad_eq12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq12, "double_integrator_QP_solver_grad_eq12");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_v14, double_integrator_QP_solver_D01, double_integrator_QP_solver_v13, double_integrator_QP_solver_grad_eq13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq13, "double_integrator_QP_solver_grad_eq13");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_v15, double_integrator_QP_solver_D01, double_integrator_QP_solver_v14, double_integrator_QP_solver_grad_eq14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq14, "double_integrator_QP_solver_grad_eq14");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_v16, double_integrator_QP_solver_D01, double_integrator_QP_solver_v15, double_integrator_QP_solver_grad_eq15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq15, "double_integrator_QP_solver_grad_eq15");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_v17, double_integrator_QP_solver_D01, double_integrator_QP_solver_v16, double_integrator_QP_solver_grad_eq16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq16, "double_integrator_QP_solver_grad_eq16");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_v18, double_integrator_QP_solver_D01, double_integrator_QP_solver_v17, double_integrator_QP_solver_grad_eq17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq17, "double_integrator_QP_solver_grad_eq17");
double_integrator_QP_solver_LA_DENSE_MTVM2_5_12_1(double_integrator_QP_solver_C18, double_integrator_QP_solver_v19, double_integrator_QP_solver_D01, double_integrator_QP_solver_v18, double_integrator_QP_solver_grad_eq18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq18, "double_integrator_QP_solver_grad_eq18");
double_integrator_QP_solver_LA_DIAGZERO_MTVM_5_5(double_integrator_QP_solver_D19, double_integrator_QP_solver_v19, double_integrator_QP_solver_grad_eq19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_grad_eq19, "double_integrator_QP_solver_grad_eq19");
info->res_ineq = 0;
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb1, double_integrator_QP_solver_z00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_llb00, double_integrator_QP_solver_slb00, double_integrator_QP_solver_rilb00, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb00, "double_integrator_QP_solver_rilb00");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z00, double_integrator_QP_solver_ubIdx00, params->ub1, double_integrator_QP_solver_lub00, double_integrator_QP_solver_sub00, double_integrator_QP_solver_riub00, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub00, "double_integrator_QP_solver_riub00");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A1, double_integrator_QP_solver_z00, params->b1, double_integrator_QP_solver_sp00, double_integrator_QP_solver_lp00, double_integrator_QP_solver_rip00, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip00, "double_integrator_QP_solver_rip00");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb2, double_integrator_QP_solver_z01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_llb01, double_integrator_QP_solver_slb01, double_integrator_QP_solver_rilb01, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb01, "double_integrator_QP_solver_rilb01");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z01, double_integrator_QP_solver_ubIdx01, params->ub2, double_integrator_QP_solver_lub01, double_integrator_QP_solver_sub01, double_integrator_QP_solver_riub01, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub01, "double_integrator_QP_solver_riub01");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A2, double_integrator_QP_solver_z01, params->b2, double_integrator_QP_solver_sp01, double_integrator_QP_solver_lp01, double_integrator_QP_solver_rip01, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip01, "double_integrator_QP_solver_rip01");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb3, double_integrator_QP_solver_z02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_llb02, double_integrator_QP_solver_slb02, double_integrator_QP_solver_rilb02, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb02, "double_integrator_QP_solver_rilb02");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z02, double_integrator_QP_solver_ubIdx02, params->ub3, double_integrator_QP_solver_lub02, double_integrator_QP_solver_sub02, double_integrator_QP_solver_riub02, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub02, "double_integrator_QP_solver_riub02");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A3, double_integrator_QP_solver_z02, params->b3, double_integrator_QP_solver_sp02, double_integrator_QP_solver_lp02, double_integrator_QP_solver_rip02, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip02, "double_integrator_QP_solver_rip02");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb4, double_integrator_QP_solver_z03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_llb03, double_integrator_QP_solver_slb03, double_integrator_QP_solver_rilb03, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb03, "double_integrator_QP_solver_rilb03");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z03, double_integrator_QP_solver_ubIdx03, params->ub4, double_integrator_QP_solver_lub03, double_integrator_QP_solver_sub03, double_integrator_QP_solver_riub03, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub03, "double_integrator_QP_solver_riub03");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A4, double_integrator_QP_solver_z03, params->b4, double_integrator_QP_solver_sp03, double_integrator_QP_solver_lp03, double_integrator_QP_solver_rip03, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip03, "double_integrator_QP_solver_rip03");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb5, double_integrator_QP_solver_z04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_llb04, double_integrator_QP_solver_slb04, double_integrator_QP_solver_rilb04, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb04, "double_integrator_QP_solver_rilb04");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z04, double_integrator_QP_solver_ubIdx04, params->ub5, double_integrator_QP_solver_lub04, double_integrator_QP_solver_sub04, double_integrator_QP_solver_riub04, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub04, "double_integrator_QP_solver_riub04");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A5, double_integrator_QP_solver_z04, params->b5, double_integrator_QP_solver_sp04, double_integrator_QP_solver_lp04, double_integrator_QP_solver_rip04, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip04, "double_integrator_QP_solver_rip04");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb6, double_integrator_QP_solver_z05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_llb05, double_integrator_QP_solver_slb05, double_integrator_QP_solver_rilb05, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb05, "double_integrator_QP_solver_rilb05");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z05, double_integrator_QP_solver_ubIdx05, params->ub6, double_integrator_QP_solver_lub05, double_integrator_QP_solver_sub05, double_integrator_QP_solver_riub05, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub05, "double_integrator_QP_solver_riub05");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A6, double_integrator_QP_solver_z05, params->b6, double_integrator_QP_solver_sp05, double_integrator_QP_solver_lp05, double_integrator_QP_solver_rip05, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip05, "double_integrator_QP_solver_rip05");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb7, double_integrator_QP_solver_z06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_llb06, double_integrator_QP_solver_slb06, double_integrator_QP_solver_rilb06, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb06, "double_integrator_QP_solver_rilb06");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z06, double_integrator_QP_solver_ubIdx06, params->ub7, double_integrator_QP_solver_lub06, double_integrator_QP_solver_sub06, double_integrator_QP_solver_riub06, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub06, "double_integrator_QP_solver_riub06");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A7, double_integrator_QP_solver_z06, params->b7, double_integrator_QP_solver_sp06, double_integrator_QP_solver_lp06, double_integrator_QP_solver_rip06, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip06, "double_integrator_QP_solver_rip06");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb8, double_integrator_QP_solver_z07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_llb07, double_integrator_QP_solver_slb07, double_integrator_QP_solver_rilb07, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb07, "double_integrator_QP_solver_rilb07");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z07, double_integrator_QP_solver_ubIdx07, params->ub8, double_integrator_QP_solver_lub07, double_integrator_QP_solver_sub07, double_integrator_QP_solver_riub07, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub07, "double_integrator_QP_solver_riub07");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A8, double_integrator_QP_solver_z07, params->b8, double_integrator_QP_solver_sp07, double_integrator_QP_solver_lp07, double_integrator_QP_solver_rip07, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip07, "double_integrator_QP_solver_rip07");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb9, double_integrator_QP_solver_z08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_llb08, double_integrator_QP_solver_slb08, double_integrator_QP_solver_rilb08, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb08, "double_integrator_QP_solver_rilb08");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z08, double_integrator_QP_solver_ubIdx08, params->ub9, double_integrator_QP_solver_lub08, double_integrator_QP_solver_sub08, double_integrator_QP_solver_riub08, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub08, "double_integrator_QP_solver_riub08");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A9, double_integrator_QP_solver_z08, params->b9, double_integrator_QP_solver_sp08, double_integrator_QP_solver_lp08, double_integrator_QP_solver_rip08, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip08, "double_integrator_QP_solver_rip08");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb10, double_integrator_QP_solver_z09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_llb09, double_integrator_QP_solver_slb09, double_integrator_QP_solver_rilb09, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb09, "double_integrator_QP_solver_rilb09");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z09, double_integrator_QP_solver_ubIdx09, params->ub10, double_integrator_QP_solver_lub09, double_integrator_QP_solver_sub09, double_integrator_QP_solver_riub09, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub09, "double_integrator_QP_solver_riub09");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A10, double_integrator_QP_solver_z09, params->b10, double_integrator_QP_solver_sp09, double_integrator_QP_solver_lp09, double_integrator_QP_solver_rip09, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip09, "double_integrator_QP_solver_rip09");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb11, double_integrator_QP_solver_z10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_llb10, double_integrator_QP_solver_slb10, double_integrator_QP_solver_rilb10, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb10, "double_integrator_QP_solver_rilb10");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z10, double_integrator_QP_solver_ubIdx10, params->ub11, double_integrator_QP_solver_lub10, double_integrator_QP_solver_sub10, double_integrator_QP_solver_riub10, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub10, "double_integrator_QP_solver_riub10");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A11, double_integrator_QP_solver_z10, params->b11, double_integrator_QP_solver_sp10, double_integrator_QP_solver_lp10, double_integrator_QP_solver_rip10, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip10, "double_integrator_QP_solver_rip10");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb12, double_integrator_QP_solver_z11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_llb11, double_integrator_QP_solver_slb11, double_integrator_QP_solver_rilb11, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb11, "double_integrator_QP_solver_rilb11");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z11, double_integrator_QP_solver_ubIdx11, params->ub12, double_integrator_QP_solver_lub11, double_integrator_QP_solver_sub11, double_integrator_QP_solver_riub11, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub11, "double_integrator_QP_solver_riub11");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A12, double_integrator_QP_solver_z11, params->b12, double_integrator_QP_solver_sp11, double_integrator_QP_solver_lp11, double_integrator_QP_solver_rip11, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip11, "double_integrator_QP_solver_rip11");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb13, double_integrator_QP_solver_z12, double_integrator_QP_solver_lbIdx12, double_integrator_QP_solver_llb12, double_integrator_QP_solver_slb12, double_integrator_QP_solver_rilb12, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb12, "double_integrator_QP_solver_rilb12");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z12, double_integrator_QP_solver_ubIdx12, params->ub13, double_integrator_QP_solver_lub12, double_integrator_QP_solver_sub12, double_integrator_QP_solver_riub12, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub12, "double_integrator_QP_solver_riub12");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A13, double_integrator_QP_solver_z12, params->b13, double_integrator_QP_solver_sp12, double_integrator_QP_solver_lp12, double_integrator_QP_solver_rip12, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip12, "double_integrator_QP_solver_rip12");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb14, double_integrator_QP_solver_z13, double_integrator_QP_solver_lbIdx13, double_integrator_QP_solver_llb13, double_integrator_QP_solver_slb13, double_integrator_QP_solver_rilb13, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb13, "double_integrator_QP_solver_rilb13");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z13, double_integrator_QP_solver_ubIdx13, params->ub14, double_integrator_QP_solver_lub13, double_integrator_QP_solver_sub13, double_integrator_QP_solver_riub13, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub13, "double_integrator_QP_solver_riub13");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A14, double_integrator_QP_solver_z13, params->b14, double_integrator_QP_solver_sp13, double_integrator_QP_solver_lp13, double_integrator_QP_solver_rip13, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip13, "double_integrator_QP_solver_rip13");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb15, double_integrator_QP_solver_z14, double_integrator_QP_solver_lbIdx14, double_integrator_QP_solver_llb14, double_integrator_QP_solver_slb14, double_integrator_QP_solver_rilb14, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb14, "double_integrator_QP_solver_rilb14");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z14, double_integrator_QP_solver_ubIdx14, params->ub15, double_integrator_QP_solver_lub14, double_integrator_QP_solver_sub14, double_integrator_QP_solver_riub14, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub14, "double_integrator_QP_solver_riub14");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A15, double_integrator_QP_solver_z14, params->b15, double_integrator_QP_solver_sp14, double_integrator_QP_solver_lp14, double_integrator_QP_solver_rip14, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip14, "double_integrator_QP_solver_rip14");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb16, double_integrator_QP_solver_z15, double_integrator_QP_solver_lbIdx15, double_integrator_QP_solver_llb15, double_integrator_QP_solver_slb15, double_integrator_QP_solver_rilb15, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb15, "double_integrator_QP_solver_rilb15");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z15, double_integrator_QP_solver_ubIdx15, params->ub16, double_integrator_QP_solver_lub15, double_integrator_QP_solver_sub15, double_integrator_QP_solver_riub15, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub15, "double_integrator_QP_solver_riub15");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A16, double_integrator_QP_solver_z15, params->b16, double_integrator_QP_solver_sp15, double_integrator_QP_solver_lp15, double_integrator_QP_solver_rip15, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip15, "double_integrator_QP_solver_rip15");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb17, double_integrator_QP_solver_z16, double_integrator_QP_solver_lbIdx16, double_integrator_QP_solver_llb16, double_integrator_QP_solver_slb16, double_integrator_QP_solver_rilb16, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb16, "double_integrator_QP_solver_rilb16");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z16, double_integrator_QP_solver_ubIdx16, params->ub17, double_integrator_QP_solver_lub16, double_integrator_QP_solver_sub16, double_integrator_QP_solver_riub16, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub16, "double_integrator_QP_solver_riub16");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A17, double_integrator_QP_solver_z16, params->b17, double_integrator_QP_solver_sp16, double_integrator_QP_solver_lp16, double_integrator_QP_solver_rip16, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip16, "double_integrator_QP_solver_rip16");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb18, double_integrator_QP_solver_z17, double_integrator_QP_solver_lbIdx17, double_integrator_QP_solver_llb17, double_integrator_QP_solver_slb17, double_integrator_QP_solver_rilb17, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb17, "double_integrator_QP_solver_rilb17");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z17, double_integrator_QP_solver_ubIdx17, params->ub18, double_integrator_QP_solver_lub17, double_integrator_QP_solver_sub17, double_integrator_QP_solver_riub17, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub17, "double_integrator_QP_solver_riub17");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A18, double_integrator_QP_solver_z17, params->b18, double_integrator_QP_solver_sp17, double_integrator_QP_solver_lp17, double_integrator_QP_solver_rip17, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip17, "double_integrator_QP_solver_rip17");
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb19, double_integrator_QP_solver_z18, double_integrator_QP_solver_lbIdx18, double_integrator_QP_solver_llb18, double_integrator_QP_solver_slb18, double_integrator_QP_solver_rilb18, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_rilb18, "double_integrator_QP_solver_rilb18");
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z18, double_integrator_QP_solver_ubIdx18, params->ub19, double_integrator_QP_solver_lub18, double_integrator_QP_solver_sub18, double_integrator_QP_solver_riub18, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_riub18, "double_integrator_QP_solver_riub18");
double_integrator_QP_solver_LA_MVSUBADD_24_12(params->A19, double_integrator_QP_solver_z18, params->b19, double_integrator_QP_solver_sp18, double_integrator_QP_solver_lp18, double_integrator_QP_solver_rip18, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_rip18, "double_integrator_QP_solver_rip18");
double_integrator_QP_solver_LA_VSUBADD3_4(params->lb20, double_integrator_QP_solver_z19, double_integrator_QP_solver_lbIdx19, double_integrator_QP_solver_llb19, double_integrator_QP_solver_slb19, double_integrator_QP_solver_rilb19, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_4(double_integrator_QP_solver_rilb19, "double_integrator_QP_solver_rilb19");
double_integrator_QP_solver_LA_VSUBADD2_4(double_integrator_QP_solver_z19, double_integrator_QP_solver_ubIdx19, params->ub20, double_integrator_QP_solver_lub19, double_integrator_QP_solver_sub19, double_integrator_QP_solver_riub19, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_4(double_integrator_QP_solver_riub19, "double_integrator_QP_solver_riub19");
double_integrator_QP_solver_LA_MVSUBADD_10_5(params->A20, double_integrator_QP_solver_z19, params->b20, double_integrator_QP_solver_sp19, double_integrator_QP_solver_lp19, double_integrator_QP_solver_rip19, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_10(double_integrator_QP_solver_rip19, "double_integrator_QP_solver_rip19");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub00, double_integrator_QP_solver_sub00, double_integrator_QP_solver_riub00, double_integrator_QP_solver_llb00, double_integrator_QP_solver_slb00, double_integrator_QP_solver_rilb00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_grad_ineq00, double_integrator_QP_solver_lubbysub00, double_integrator_QP_solver_llbbyslb00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub00, "double_integrator_QP_solver_lubbysub00");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb00, "double_integrator_QP_solver_llbbyslb00");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A1, double_integrator_QP_solver_lp00, double_integrator_QP_solver_sp00, double_integrator_QP_solver_rip00, double_integrator_QP_solver_grad_ineq00, double_integrator_QP_solver_lpbysp00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp00, "double_integrator_QP_solver_lpbysp00");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq00, "double_integrator_QP_solver_grad_ineq00");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub01, double_integrator_QP_solver_sub01, double_integrator_QP_solver_riub01, double_integrator_QP_solver_llb01, double_integrator_QP_solver_slb01, double_integrator_QP_solver_rilb01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_grad_ineq01, double_integrator_QP_solver_lubbysub01, double_integrator_QP_solver_llbbyslb01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub01, "double_integrator_QP_solver_lubbysub01");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb01, "double_integrator_QP_solver_llbbyslb01");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A2, double_integrator_QP_solver_lp01, double_integrator_QP_solver_sp01, double_integrator_QP_solver_rip01, double_integrator_QP_solver_grad_ineq01, double_integrator_QP_solver_lpbysp01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp01, "double_integrator_QP_solver_lpbysp01");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq01, "double_integrator_QP_solver_grad_ineq01");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub02, double_integrator_QP_solver_sub02, double_integrator_QP_solver_riub02, double_integrator_QP_solver_llb02, double_integrator_QP_solver_slb02, double_integrator_QP_solver_rilb02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_grad_ineq02, double_integrator_QP_solver_lubbysub02, double_integrator_QP_solver_llbbyslb02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub02, "double_integrator_QP_solver_lubbysub02");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb02, "double_integrator_QP_solver_llbbyslb02");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A3, double_integrator_QP_solver_lp02, double_integrator_QP_solver_sp02, double_integrator_QP_solver_rip02, double_integrator_QP_solver_grad_ineq02, double_integrator_QP_solver_lpbysp02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp02, "double_integrator_QP_solver_lpbysp02");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq02, "double_integrator_QP_solver_grad_ineq02");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub03, double_integrator_QP_solver_sub03, double_integrator_QP_solver_riub03, double_integrator_QP_solver_llb03, double_integrator_QP_solver_slb03, double_integrator_QP_solver_rilb03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_grad_ineq03, double_integrator_QP_solver_lubbysub03, double_integrator_QP_solver_llbbyslb03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub03, "double_integrator_QP_solver_lubbysub03");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb03, "double_integrator_QP_solver_llbbyslb03");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A4, double_integrator_QP_solver_lp03, double_integrator_QP_solver_sp03, double_integrator_QP_solver_rip03, double_integrator_QP_solver_grad_ineq03, double_integrator_QP_solver_lpbysp03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp03, "double_integrator_QP_solver_lpbysp03");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq03, "double_integrator_QP_solver_grad_ineq03");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub04, double_integrator_QP_solver_sub04, double_integrator_QP_solver_riub04, double_integrator_QP_solver_llb04, double_integrator_QP_solver_slb04, double_integrator_QP_solver_rilb04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_grad_ineq04, double_integrator_QP_solver_lubbysub04, double_integrator_QP_solver_llbbyslb04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub04, "double_integrator_QP_solver_lubbysub04");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb04, "double_integrator_QP_solver_llbbyslb04");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A5, double_integrator_QP_solver_lp04, double_integrator_QP_solver_sp04, double_integrator_QP_solver_rip04, double_integrator_QP_solver_grad_ineq04, double_integrator_QP_solver_lpbysp04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp04, "double_integrator_QP_solver_lpbysp04");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq04, "double_integrator_QP_solver_grad_ineq04");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub05, double_integrator_QP_solver_sub05, double_integrator_QP_solver_riub05, double_integrator_QP_solver_llb05, double_integrator_QP_solver_slb05, double_integrator_QP_solver_rilb05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_grad_ineq05, double_integrator_QP_solver_lubbysub05, double_integrator_QP_solver_llbbyslb05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub05, "double_integrator_QP_solver_lubbysub05");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb05, "double_integrator_QP_solver_llbbyslb05");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A6, double_integrator_QP_solver_lp05, double_integrator_QP_solver_sp05, double_integrator_QP_solver_rip05, double_integrator_QP_solver_grad_ineq05, double_integrator_QP_solver_lpbysp05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp05, "double_integrator_QP_solver_lpbysp05");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq05, "double_integrator_QP_solver_grad_ineq05");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub06, double_integrator_QP_solver_sub06, double_integrator_QP_solver_riub06, double_integrator_QP_solver_llb06, double_integrator_QP_solver_slb06, double_integrator_QP_solver_rilb06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_grad_ineq06, double_integrator_QP_solver_lubbysub06, double_integrator_QP_solver_llbbyslb06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub06, "double_integrator_QP_solver_lubbysub06");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb06, "double_integrator_QP_solver_llbbyslb06");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A7, double_integrator_QP_solver_lp06, double_integrator_QP_solver_sp06, double_integrator_QP_solver_rip06, double_integrator_QP_solver_grad_ineq06, double_integrator_QP_solver_lpbysp06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp06, "double_integrator_QP_solver_lpbysp06");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq06, "double_integrator_QP_solver_grad_ineq06");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub07, double_integrator_QP_solver_sub07, double_integrator_QP_solver_riub07, double_integrator_QP_solver_llb07, double_integrator_QP_solver_slb07, double_integrator_QP_solver_rilb07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_grad_ineq07, double_integrator_QP_solver_lubbysub07, double_integrator_QP_solver_llbbyslb07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub07, "double_integrator_QP_solver_lubbysub07");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb07, "double_integrator_QP_solver_llbbyslb07");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A8, double_integrator_QP_solver_lp07, double_integrator_QP_solver_sp07, double_integrator_QP_solver_rip07, double_integrator_QP_solver_grad_ineq07, double_integrator_QP_solver_lpbysp07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp07, "double_integrator_QP_solver_lpbysp07");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq07, "double_integrator_QP_solver_grad_ineq07");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub08, double_integrator_QP_solver_sub08, double_integrator_QP_solver_riub08, double_integrator_QP_solver_llb08, double_integrator_QP_solver_slb08, double_integrator_QP_solver_rilb08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_grad_ineq08, double_integrator_QP_solver_lubbysub08, double_integrator_QP_solver_llbbyslb08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub08, "double_integrator_QP_solver_lubbysub08");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb08, "double_integrator_QP_solver_llbbyslb08");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A9, double_integrator_QP_solver_lp08, double_integrator_QP_solver_sp08, double_integrator_QP_solver_rip08, double_integrator_QP_solver_grad_ineq08, double_integrator_QP_solver_lpbysp08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp08, "double_integrator_QP_solver_lpbysp08");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq08, "double_integrator_QP_solver_grad_ineq08");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub09, double_integrator_QP_solver_sub09, double_integrator_QP_solver_riub09, double_integrator_QP_solver_llb09, double_integrator_QP_solver_slb09, double_integrator_QP_solver_rilb09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_grad_ineq09, double_integrator_QP_solver_lubbysub09, double_integrator_QP_solver_llbbyslb09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub09, "double_integrator_QP_solver_lubbysub09");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb09, "double_integrator_QP_solver_llbbyslb09");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A10, double_integrator_QP_solver_lp09, double_integrator_QP_solver_sp09, double_integrator_QP_solver_rip09, double_integrator_QP_solver_grad_ineq09, double_integrator_QP_solver_lpbysp09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp09, "double_integrator_QP_solver_lpbysp09");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq09, "double_integrator_QP_solver_grad_ineq09");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub10, double_integrator_QP_solver_sub10, double_integrator_QP_solver_riub10, double_integrator_QP_solver_llb10, double_integrator_QP_solver_slb10, double_integrator_QP_solver_rilb10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_grad_ineq10, double_integrator_QP_solver_lubbysub10, double_integrator_QP_solver_llbbyslb10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub10, "double_integrator_QP_solver_lubbysub10");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb10, "double_integrator_QP_solver_llbbyslb10");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A11, double_integrator_QP_solver_lp10, double_integrator_QP_solver_sp10, double_integrator_QP_solver_rip10, double_integrator_QP_solver_grad_ineq10, double_integrator_QP_solver_lpbysp10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp10, "double_integrator_QP_solver_lpbysp10");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq10, "double_integrator_QP_solver_grad_ineq10");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub11, double_integrator_QP_solver_sub11, double_integrator_QP_solver_riub11, double_integrator_QP_solver_llb11, double_integrator_QP_solver_slb11, double_integrator_QP_solver_rilb11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_grad_ineq11, double_integrator_QP_solver_lubbysub11, double_integrator_QP_solver_llbbyslb11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub11, "double_integrator_QP_solver_lubbysub11");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb11, "double_integrator_QP_solver_llbbyslb11");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A12, double_integrator_QP_solver_lp11, double_integrator_QP_solver_sp11, double_integrator_QP_solver_rip11, double_integrator_QP_solver_grad_ineq11, double_integrator_QP_solver_lpbysp11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp11, "double_integrator_QP_solver_lpbysp11");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq11, "double_integrator_QP_solver_grad_ineq11");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub12, double_integrator_QP_solver_sub12, double_integrator_QP_solver_riub12, double_integrator_QP_solver_llb12, double_integrator_QP_solver_slb12, double_integrator_QP_solver_rilb12, double_integrator_QP_solver_lbIdx12, double_integrator_QP_solver_ubIdx12, double_integrator_QP_solver_grad_ineq12, double_integrator_QP_solver_lubbysub12, double_integrator_QP_solver_llbbyslb12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub12, "double_integrator_QP_solver_lubbysub12");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb12, "double_integrator_QP_solver_llbbyslb12");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A13, double_integrator_QP_solver_lp12, double_integrator_QP_solver_sp12, double_integrator_QP_solver_rip12, double_integrator_QP_solver_grad_ineq12, double_integrator_QP_solver_lpbysp12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp12, "double_integrator_QP_solver_lpbysp12");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq12, "double_integrator_QP_solver_grad_ineq12");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub13, double_integrator_QP_solver_sub13, double_integrator_QP_solver_riub13, double_integrator_QP_solver_llb13, double_integrator_QP_solver_slb13, double_integrator_QP_solver_rilb13, double_integrator_QP_solver_lbIdx13, double_integrator_QP_solver_ubIdx13, double_integrator_QP_solver_grad_ineq13, double_integrator_QP_solver_lubbysub13, double_integrator_QP_solver_llbbyslb13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub13, "double_integrator_QP_solver_lubbysub13");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb13, "double_integrator_QP_solver_llbbyslb13");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A14, double_integrator_QP_solver_lp13, double_integrator_QP_solver_sp13, double_integrator_QP_solver_rip13, double_integrator_QP_solver_grad_ineq13, double_integrator_QP_solver_lpbysp13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp13, "double_integrator_QP_solver_lpbysp13");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq13, "double_integrator_QP_solver_grad_ineq13");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub14, double_integrator_QP_solver_sub14, double_integrator_QP_solver_riub14, double_integrator_QP_solver_llb14, double_integrator_QP_solver_slb14, double_integrator_QP_solver_rilb14, double_integrator_QP_solver_lbIdx14, double_integrator_QP_solver_ubIdx14, double_integrator_QP_solver_grad_ineq14, double_integrator_QP_solver_lubbysub14, double_integrator_QP_solver_llbbyslb14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub14, "double_integrator_QP_solver_lubbysub14");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb14, "double_integrator_QP_solver_llbbyslb14");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A15, double_integrator_QP_solver_lp14, double_integrator_QP_solver_sp14, double_integrator_QP_solver_rip14, double_integrator_QP_solver_grad_ineq14, double_integrator_QP_solver_lpbysp14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp14, "double_integrator_QP_solver_lpbysp14");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq14, "double_integrator_QP_solver_grad_ineq14");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub15, double_integrator_QP_solver_sub15, double_integrator_QP_solver_riub15, double_integrator_QP_solver_llb15, double_integrator_QP_solver_slb15, double_integrator_QP_solver_rilb15, double_integrator_QP_solver_lbIdx15, double_integrator_QP_solver_ubIdx15, double_integrator_QP_solver_grad_ineq15, double_integrator_QP_solver_lubbysub15, double_integrator_QP_solver_llbbyslb15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub15, "double_integrator_QP_solver_lubbysub15");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb15, "double_integrator_QP_solver_llbbyslb15");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A16, double_integrator_QP_solver_lp15, double_integrator_QP_solver_sp15, double_integrator_QP_solver_rip15, double_integrator_QP_solver_grad_ineq15, double_integrator_QP_solver_lpbysp15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp15, "double_integrator_QP_solver_lpbysp15");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq15, "double_integrator_QP_solver_grad_ineq15");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub16, double_integrator_QP_solver_sub16, double_integrator_QP_solver_riub16, double_integrator_QP_solver_llb16, double_integrator_QP_solver_slb16, double_integrator_QP_solver_rilb16, double_integrator_QP_solver_lbIdx16, double_integrator_QP_solver_ubIdx16, double_integrator_QP_solver_grad_ineq16, double_integrator_QP_solver_lubbysub16, double_integrator_QP_solver_llbbyslb16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub16, "double_integrator_QP_solver_lubbysub16");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb16, "double_integrator_QP_solver_llbbyslb16");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A17, double_integrator_QP_solver_lp16, double_integrator_QP_solver_sp16, double_integrator_QP_solver_rip16, double_integrator_QP_solver_grad_ineq16, double_integrator_QP_solver_lpbysp16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp16, "double_integrator_QP_solver_lpbysp16");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq16, "double_integrator_QP_solver_grad_ineq16");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub17, double_integrator_QP_solver_sub17, double_integrator_QP_solver_riub17, double_integrator_QP_solver_llb17, double_integrator_QP_solver_slb17, double_integrator_QP_solver_rilb17, double_integrator_QP_solver_lbIdx17, double_integrator_QP_solver_ubIdx17, double_integrator_QP_solver_grad_ineq17, double_integrator_QP_solver_lubbysub17, double_integrator_QP_solver_llbbyslb17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub17, "double_integrator_QP_solver_lubbysub17");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb17, "double_integrator_QP_solver_llbbyslb17");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A18, double_integrator_QP_solver_lp17, double_integrator_QP_solver_sp17, double_integrator_QP_solver_rip17, double_integrator_QP_solver_grad_ineq17, double_integrator_QP_solver_lpbysp17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp17, "double_integrator_QP_solver_lpbysp17");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq17, "double_integrator_QP_solver_grad_ineq17");
double_integrator_QP_solver_LA_INEQ_B_GRAD_12_7_6(double_integrator_QP_solver_lub18, double_integrator_QP_solver_sub18, double_integrator_QP_solver_riub18, double_integrator_QP_solver_llb18, double_integrator_QP_solver_slb18, double_integrator_QP_solver_rilb18, double_integrator_QP_solver_lbIdx18, double_integrator_QP_solver_ubIdx18, double_integrator_QP_solver_grad_ineq18, double_integrator_QP_solver_lubbysub18, double_integrator_QP_solver_llbbyslb18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_lubbysub18, "double_integrator_QP_solver_lubbysub18");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_llbbyslb18, "double_integrator_QP_solver_llbbyslb18");
double_integrator_QP_solver_LA_INEQ_P_24_12(params->A19, double_integrator_QP_solver_lp18, double_integrator_QP_solver_sp18, double_integrator_QP_solver_rip18, double_integrator_QP_solver_grad_ineq18, double_integrator_QP_solver_lpbysp18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_lpbysp18, "double_integrator_QP_solver_lpbysp18");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_ineq18, "double_integrator_QP_solver_grad_ineq18");
double_integrator_QP_solver_LA_INEQ_B_GRAD_5_4_4(double_integrator_QP_solver_lub19, double_integrator_QP_solver_sub19, double_integrator_QP_solver_riub19, double_integrator_QP_solver_llb19, double_integrator_QP_solver_slb19, double_integrator_QP_solver_rilb19, double_integrator_QP_solver_lbIdx19, double_integrator_QP_solver_ubIdx19, double_integrator_QP_solver_grad_ineq19, double_integrator_QP_solver_lubbysub19, double_integrator_QP_solver_llbbyslb19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_4(double_integrator_QP_solver_lubbysub19, "double_integrator_QP_solver_lubbysub19");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_4(double_integrator_QP_solver_llbbyslb19, "double_integrator_QP_solver_llbbyslb19");
double_integrator_QP_solver_LA_INEQ_P_10_5(params->A20, double_integrator_QP_solver_lp19, double_integrator_QP_solver_sp19, double_integrator_QP_solver_rip19, double_integrator_QP_solver_grad_ineq19, double_integrator_QP_solver_lpbysp19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_10(double_integrator_QP_solver_lpbysp19, "double_integrator_QP_solver_lpbysp19");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_grad_ineq19, "double_integrator_QP_solver_grad_ineq19");
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
double_integrator_QP_solver_LA_VVADD3_233(double_integrator_QP_solver_grad_cost, double_integrator_QP_solver_grad_eq, double_integrator_QP_solver_grad_ineq, double_integrator_QP_solver_rd);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd00, "double_integrator_QP_solver_rd00");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd01, "double_integrator_QP_solver_rd01");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd02, "double_integrator_QP_solver_rd02");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd03, "double_integrator_QP_solver_rd03");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd04, "double_integrator_QP_solver_rd04");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd05, "double_integrator_QP_solver_rd05");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd06, "double_integrator_QP_solver_rd06");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd07, "double_integrator_QP_solver_rd07");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd08, "double_integrator_QP_solver_rd08");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd09, "double_integrator_QP_solver_rd09");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd10, "double_integrator_QP_solver_rd10");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd11, "double_integrator_QP_solver_rd11");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd12, "double_integrator_QP_solver_rd12");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd13, "double_integrator_QP_solver_rd13");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd14, "double_integrator_QP_solver_rd14");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd15, "double_integrator_QP_solver_rd15");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd16, "double_integrator_QP_solver_rd16");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd17, "double_integrator_QP_solver_rd17");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd18, "double_integrator_QP_solver_rd18");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_rd19, "double_integrator_QP_solver_rd19");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_lubbysub00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_Phi00);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A1, double_integrator_QP_solver_lpbysp00, double_integrator_QP_solver_Phi00);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi00);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_C00, double_integrator_QP_solver_V00);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_12_12(double_integrator_QP_solver_V00, "double_integrator_QP_solver_V00");
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_12_12(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_D00, double_integrator_QP_solver_W00);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_12_12(double_integrator_QP_solver_W00, "double_integrator_QP_solver_W00");
double_integrator_QP_solver_LA_DENSE_MMTM_12_12_12(double_integrator_QP_solver_W00, double_integrator_QP_solver_V00, double_integrator_QP_solver_Ysd01);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_12_1(double_integrator_QP_solver_Ysd01, "double_integrator_QP_solver_Ysd01");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_rd00, double_integrator_QP_solver_Lbyrd00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd00, "double_integrator_QP_solver_Lbyrd00");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_lubbysub01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_Phi01);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A2, double_integrator_QP_solver_lpbysp01, double_integrator_QP_solver_Phi01);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi01);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi01, "double_integrator_QP_solver_Phi01");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_C00, double_integrator_QP_solver_V01);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_D01, double_integrator_QP_solver_W01);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V01, "double_integrator_QP_solver_V01");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W01, "double_integrator_QP_solver_W01");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W01, double_integrator_QP_solver_V01, double_integrator_QP_solver_Ysd02);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Ysd02, "double_integrator_QP_solver_Ysd02");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_rd01, double_integrator_QP_solver_Lbyrd01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd01, "double_integrator_QP_solver_Lbyrd01");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_lubbysub02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_Phi02);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A3, double_integrator_QP_solver_lpbysp02, double_integrator_QP_solver_Phi02);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi02);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi02, "double_integrator_QP_solver_Phi02");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_C00, double_integrator_QP_solver_V02);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_D01, double_integrator_QP_solver_W02);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V02, "double_integrator_QP_solver_V02");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W02, "double_integrator_QP_solver_W02");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W02, double_integrator_QP_solver_V02, double_integrator_QP_solver_Ysd03);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Ysd03, "double_integrator_QP_solver_Ysd03");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_rd02, double_integrator_QP_solver_Lbyrd02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd02, "double_integrator_QP_solver_Lbyrd02");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_lubbysub03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_Phi03);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A4, double_integrator_QP_solver_lpbysp03, double_integrator_QP_solver_Phi03);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi03);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi03, "double_integrator_QP_solver_Phi03");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_C00, double_integrator_QP_solver_V03);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_D01, double_integrator_QP_solver_W03);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V03, "double_integrator_QP_solver_V03");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W03, "double_integrator_QP_solver_W03");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W03, double_integrator_QP_solver_V03, double_integrator_QP_solver_Ysd04);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Ysd04, "double_integrator_QP_solver_Ysd04");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_rd03, double_integrator_QP_solver_Lbyrd03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd03, "double_integrator_QP_solver_Lbyrd03");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_lubbysub04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_Phi04);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A5, double_integrator_QP_solver_lpbysp04, double_integrator_QP_solver_Phi04);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi04);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi04, "double_integrator_QP_solver_Phi04");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_C00, double_integrator_QP_solver_V04);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_D01, double_integrator_QP_solver_W04);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V04, "double_integrator_QP_solver_V04");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W04, "double_integrator_QP_solver_W04");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W04, double_integrator_QP_solver_V04, double_integrator_QP_solver_Ysd05);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Ysd05, "double_integrator_QP_solver_Ysd05");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_rd04, double_integrator_QP_solver_Lbyrd04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd04, "double_integrator_QP_solver_Lbyrd04");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_lubbysub05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_Phi05);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A6, double_integrator_QP_solver_lpbysp05, double_integrator_QP_solver_Phi05);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi05);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi05, "double_integrator_QP_solver_Phi05");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_C00, double_integrator_QP_solver_V05);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_D01, double_integrator_QP_solver_W05);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V05, "double_integrator_QP_solver_V05");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W05, "double_integrator_QP_solver_W05");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W05, double_integrator_QP_solver_V05, double_integrator_QP_solver_Ysd06);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Ysd06, "double_integrator_QP_solver_Ysd06");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_rd05, double_integrator_QP_solver_Lbyrd05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd05, "double_integrator_QP_solver_Lbyrd05");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_lubbysub06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_Phi06);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A7, double_integrator_QP_solver_lpbysp06, double_integrator_QP_solver_Phi06);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi06);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi06, "double_integrator_QP_solver_Phi06");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_C00, double_integrator_QP_solver_V06);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_D01, double_integrator_QP_solver_W06);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V06, "double_integrator_QP_solver_V06");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W06, "double_integrator_QP_solver_W06");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W06, double_integrator_QP_solver_V06, double_integrator_QP_solver_Ysd07);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Ysd07, "double_integrator_QP_solver_Ysd07");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_rd06, double_integrator_QP_solver_Lbyrd06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd06, "double_integrator_QP_solver_Lbyrd06");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_lubbysub07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_Phi07);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A8, double_integrator_QP_solver_lpbysp07, double_integrator_QP_solver_Phi07);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi07);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi07, "double_integrator_QP_solver_Phi07");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_C00, double_integrator_QP_solver_V07);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_D01, double_integrator_QP_solver_W07);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V07, "double_integrator_QP_solver_V07");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W07, "double_integrator_QP_solver_W07");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W07, double_integrator_QP_solver_V07, double_integrator_QP_solver_Ysd08);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Ysd08, "double_integrator_QP_solver_Ysd08");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_rd07, double_integrator_QP_solver_Lbyrd07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd07, "double_integrator_QP_solver_Lbyrd07");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_lubbysub08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_Phi08);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A9, double_integrator_QP_solver_lpbysp08, double_integrator_QP_solver_Phi08);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi08);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi08, "double_integrator_QP_solver_Phi08");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_C00, double_integrator_QP_solver_V08);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_D01, double_integrator_QP_solver_W08);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V08, "double_integrator_QP_solver_V08");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W08, "double_integrator_QP_solver_W08");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W08, double_integrator_QP_solver_V08, double_integrator_QP_solver_Ysd09);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Ysd09, "double_integrator_QP_solver_Ysd09");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_rd08, double_integrator_QP_solver_Lbyrd08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd08, "double_integrator_QP_solver_Lbyrd08");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_lubbysub09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_Phi09);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A10, double_integrator_QP_solver_lpbysp09, double_integrator_QP_solver_Phi09);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi09);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi09, "double_integrator_QP_solver_Phi09");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_C00, double_integrator_QP_solver_V09);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_D01, double_integrator_QP_solver_W09);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V09, "double_integrator_QP_solver_V09");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W09, "double_integrator_QP_solver_W09");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W09, double_integrator_QP_solver_V09, double_integrator_QP_solver_Ysd10);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Ysd10, "double_integrator_QP_solver_Ysd10");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_rd09, double_integrator_QP_solver_Lbyrd09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd09, "double_integrator_QP_solver_Lbyrd09");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_lubbysub10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_Phi10);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A11, double_integrator_QP_solver_lpbysp10, double_integrator_QP_solver_Phi10);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi10);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi10, "double_integrator_QP_solver_Phi10");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_C00, double_integrator_QP_solver_V10);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_D01, double_integrator_QP_solver_W10);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V10, "double_integrator_QP_solver_V10");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W10, "double_integrator_QP_solver_W10");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W10, double_integrator_QP_solver_V10, double_integrator_QP_solver_Ysd11);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Ysd11, "double_integrator_QP_solver_Ysd11");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_rd10, double_integrator_QP_solver_Lbyrd10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd10, "double_integrator_QP_solver_Lbyrd10");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_lubbysub11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_Phi11);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A12, double_integrator_QP_solver_lpbysp11, double_integrator_QP_solver_Phi11);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi11);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi11, "double_integrator_QP_solver_Phi11");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_C00, double_integrator_QP_solver_V11);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_D01, double_integrator_QP_solver_W11);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V11, "double_integrator_QP_solver_V11");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W11, "double_integrator_QP_solver_W11");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W11, double_integrator_QP_solver_V11, double_integrator_QP_solver_Ysd12);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Ysd12, "double_integrator_QP_solver_Ysd12");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_rd11, double_integrator_QP_solver_Lbyrd11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd11, "double_integrator_QP_solver_Lbyrd11");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb12, double_integrator_QP_solver_lbIdx12, double_integrator_QP_solver_lubbysub12, double_integrator_QP_solver_ubIdx12, double_integrator_QP_solver_Phi12);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A13, double_integrator_QP_solver_lpbysp12, double_integrator_QP_solver_Phi12);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi12);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi12, "double_integrator_QP_solver_Phi12");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi12, double_integrator_QP_solver_C00, double_integrator_QP_solver_V12);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi12, double_integrator_QP_solver_D01, double_integrator_QP_solver_W12);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V12, "double_integrator_QP_solver_V12");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W12, "double_integrator_QP_solver_W12");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W12, double_integrator_QP_solver_V12, double_integrator_QP_solver_Ysd13);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Ysd13, "double_integrator_QP_solver_Ysd13");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi12, double_integrator_QP_solver_rd12, double_integrator_QP_solver_Lbyrd12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd12, "double_integrator_QP_solver_Lbyrd12");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb13, double_integrator_QP_solver_lbIdx13, double_integrator_QP_solver_lubbysub13, double_integrator_QP_solver_ubIdx13, double_integrator_QP_solver_Phi13);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A14, double_integrator_QP_solver_lpbysp13, double_integrator_QP_solver_Phi13);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi13);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi13, "double_integrator_QP_solver_Phi13");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi13, double_integrator_QP_solver_C00, double_integrator_QP_solver_V13);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi13, double_integrator_QP_solver_D01, double_integrator_QP_solver_W13);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V13, "double_integrator_QP_solver_V13");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W13, "double_integrator_QP_solver_W13");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W13, double_integrator_QP_solver_V13, double_integrator_QP_solver_Ysd14);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Ysd14, "double_integrator_QP_solver_Ysd14");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi13, double_integrator_QP_solver_rd13, double_integrator_QP_solver_Lbyrd13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd13, "double_integrator_QP_solver_Lbyrd13");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb14, double_integrator_QP_solver_lbIdx14, double_integrator_QP_solver_lubbysub14, double_integrator_QP_solver_ubIdx14, double_integrator_QP_solver_Phi14);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A15, double_integrator_QP_solver_lpbysp14, double_integrator_QP_solver_Phi14);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi14);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi14, "double_integrator_QP_solver_Phi14");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi14, double_integrator_QP_solver_C00, double_integrator_QP_solver_V14);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi14, double_integrator_QP_solver_D01, double_integrator_QP_solver_W14);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V14, "double_integrator_QP_solver_V14");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W14, "double_integrator_QP_solver_W14");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W14, double_integrator_QP_solver_V14, double_integrator_QP_solver_Ysd15);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Ysd15, "double_integrator_QP_solver_Ysd15");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi14, double_integrator_QP_solver_rd14, double_integrator_QP_solver_Lbyrd14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd14, "double_integrator_QP_solver_Lbyrd14");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb15, double_integrator_QP_solver_lbIdx15, double_integrator_QP_solver_lubbysub15, double_integrator_QP_solver_ubIdx15, double_integrator_QP_solver_Phi15);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A16, double_integrator_QP_solver_lpbysp15, double_integrator_QP_solver_Phi15);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi15);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi15, "double_integrator_QP_solver_Phi15");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi15, double_integrator_QP_solver_C00, double_integrator_QP_solver_V15);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi15, double_integrator_QP_solver_D01, double_integrator_QP_solver_W15);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V15, "double_integrator_QP_solver_V15");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W15, "double_integrator_QP_solver_W15");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W15, double_integrator_QP_solver_V15, double_integrator_QP_solver_Ysd16);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Ysd16, "double_integrator_QP_solver_Ysd16");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi15, double_integrator_QP_solver_rd15, double_integrator_QP_solver_Lbyrd15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd15, "double_integrator_QP_solver_Lbyrd15");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb16, double_integrator_QP_solver_lbIdx16, double_integrator_QP_solver_lubbysub16, double_integrator_QP_solver_ubIdx16, double_integrator_QP_solver_Phi16);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A17, double_integrator_QP_solver_lpbysp16, double_integrator_QP_solver_Phi16);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi16);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi16, "double_integrator_QP_solver_Phi16");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi16, double_integrator_QP_solver_C00, double_integrator_QP_solver_V16);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi16, double_integrator_QP_solver_D01, double_integrator_QP_solver_W16);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V16, "double_integrator_QP_solver_V16");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W16, "double_integrator_QP_solver_W16");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W16, double_integrator_QP_solver_V16, double_integrator_QP_solver_Ysd17);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Ysd17, "double_integrator_QP_solver_Ysd17");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi16, double_integrator_QP_solver_rd16, double_integrator_QP_solver_Lbyrd16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd16, "double_integrator_QP_solver_Lbyrd16");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb17, double_integrator_QP_solver_lbIdx17, double_integrator_QP_solver_lubbysub17, double_integrator_QP_solver_ubIdx17, double_integrator_QP_solver_Phi17);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A18, double_integrator_QP_solver_lpbysp17, double_integrator_QP_solver_Phi17);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi17);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi17, "double_integrator_QP_solver_Phi17");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi17, double_integrator_QP_solver_C00, double_integrator_QP_solver_V17);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi17, double_integrator_QP_solver_D01, double_integrator_QP_solver_W17);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V17, "double_integrator_QP_solver_V17");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W17, "double_integrator_QP_solver_W17");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W17, double_integrator_QP_solver_V17, double_integrator_QP_solver_Ysd18);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Ysd18, "double_integrator_QP_solver_Ysd18");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi17, double_integrator_QP_solver_rd17, double_integrator_QP_solver_Lbyrd17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd17, "double_integrator_QP_solver_Lbyrd17");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_12_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb18, double_integrator_QP_solver_lbIdx18, double_integrator_QP_solver_lubbysub18, double_integrator_QP_solver_ubIdx18, double_integrator_QP_solver_Phi18);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_24_12(params->A19, double_integrator_QP_solver_lpbysp18, double_integrator_QP_solver_Phi18);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_12(double_integrator_QP_solver_Phi18);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi18, "double_integrator_QP_solver_Phi18");
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_5_12(double_integrator_QP_solver_Phi18, double_integrator_QP_solver_C18, double_integrator_QP_solver_V18);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_1_12(double_integrator_QP_solver_Phi18, double_integrator_QP_solver_D01, double_integrator_QP_solver_W18);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_V18, "double_integrator_QP_solver_V18");
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_W18, "double_integrator_QP_solver_W18");
double_integrator_QP_solver_LA_DENSE_MMTM_1_12_1(double_integrator_QP_solver_W18, double_integrator_QP_solver_V18, double_integrator_QP_solver_Ysd19);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_5(double_integrator_QP_solver_Ysd19, "double_integrator_QP_solver_Ysd19");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi18, double_integrator_QP_solver_rd18, double_integrator_QP_solver_Lbyrd18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_Lbyrd18, "double_integrator_QP_solver_Lbyrd18");
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_5_4_4(double_integrator_QP_solver_H19, double_integrator_QP_solver_llbbyslb19, double_integrator_QP_solver_lbIdx19, double_integrator_QP_solver_lubbysub19, double_integrator_QP_solver_ubIdx19, double_integrator_QP_solver_Phi19);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_ADDMTDM_10_5(params->A20, double_integrator_QP_solver_lpbysp19, double_integrator_QP_solver_Phi19);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Phi00, "double_integrator_QP_solver_Phi00");
double_integrator_QP_solver_LA_DENSE_CHOL2_5(double_integrator_QP_solver_Phi19);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_5(double_integrator_QP_solver_Phi19, "double_integrator_QP_solver_Phi19");
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_5_5(double_integrator_QP_solver_Phi19, double_integrator_QP_solver_D19, double_integrator_QP_solver_W19);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_5_5(double_integrator_QP_solver_W19, "double_integrator_QP_solver_W19");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Phi19, double_integrator_QP_solver_rd19, double_integrator_QP_solver_Lbyrd19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_Lbyrd19, "double_integrator_QP_solver_Lbyrd19");
double_integrator_QP_solver_LA_DENSE_MMT_12_12(double_integrator_QP_solver_W00, double_integrator_QP_solver_Yd00);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Yd00, "double_integrator_QP_solver_Yd00");
double_integrator_QP_solver_LA_DENSE_MVMSUB7_12_12(double_integrator_QP_solver_W00, double_integrator_QP_solver_Lbyrd00, double_integrator_QP_solver_re00, double_integrator_QP_solver_beta00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_beta00, "double_integrator_QP_solver_beta00");
double_integrator_QP_solver_LA_DENSE_MMT2_12_12_12(double_integrator_QP_solver_V00, double_integrator_QP_solver_W01, double_integrator_QP_solver_Yd01);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd01, "double_integrator_QP_solver_Yd01");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_12_12_12(double_integrator_QP_solver_V00, double_integrator_QP_solver_Lbyrd00, double_integrator_QP_solver_W01, double_integrator_QP_solver_Lbyrd01, double_integrator_QP_solver_re01, double_integrator_QP_solver_beta01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta01, "double_integrator_QP_solver_beta01");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_V01, double_integrator_QP_solver_W02, double_integrator_QP_solver_Yd02);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd02, "double_integrator_QP_solver_Yd02");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_V01, double_integrator_QP_solver_Lbyrd01, double_integrator_QP_solver_W02, double_integrator_QP_solver_Lbyrd02, double_integrator_QP_solver_re02, double_integrator_QP_solver_beta02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta02, "double_integrator_QP_solver_beta02");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_V02, double_integrator_QP_solver_W03, double_integrator_QP_solver_Yd03);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd03, "double_integrator_QP_solver_Yd03");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_V02, double_integrator_QP_solver_Lbyrd02, double_integrator_QP_solver_W03, double_integrator_QP_solver_Lbyrd03, double_integrator_QP_solver_re03, double_integrator_QP_solver_beta03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta03, "double_integrator_QP_solver_beta03");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_V03, double_integrator_QP_solver_W04, double_integrator_QP_solver_Yd04);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd04, "double_integrator_QP_solver_Yd04");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_V03, double_integrator_QP_solver_Lbyrd03, double_integrator_QP_solver_W04, double_integrator_QP_solver_Lbyrd04, double_integrator_QP_solver_re04, double_integrator_QP_solver_beta04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta04, "double_integrator_QP_solver_beta04");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_V04, double_integrator_QP_solver_W05, double_integrator_QP_solver_Yd05);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd05, "double_integrator_QP_solver_Yd05");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_V04, double_integrator_QP_solver_Lbyrd04, double_integrator_QP_solver_W05, double_integrator_QP_solver_Lbyrd05, double_integrator_QP_solver_re05, double_integrator_QP_solver_beta05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta05, "double_integrator_QP_solver_beta05");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_V05, double_integrator_QP_solver_W06, double_integrator_QP_solver_Yd06);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd06, "double_integrator_QP_solver_Yd06");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_V05, double_integrator_QP_solver_Lbyrd05, double_integrator_QP_solver_W06, double_integrator_QP_solver_Lbyrd06, double_integrator_QP_solver_re06, double_integrator_QP_solver_beta06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta06, "double_integrator_QP_solver_beta06");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_V06, double_integrator_QP_solver_W07, double_integrator_QP_solver_Yd07);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd07, "double_integrator_QP_solver_Yd07");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_V06, double_integrator_QP_solver_Lbyrd06, double_integrator_QP_solver_W07, double_integrator_QP_solver_Lbyrd07, double_integrator_QP_solver_re07, double_integrator_QP_solver_beta07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta07, "double_integrator_QP_solver_beta07");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_V07, double_integrator_QP_solver_W08, double_integrator_QP_solver_Yd08);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd08, "double_integrator_QP_solver_Yd08");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_V07, double_integrator_QP_solver_Lbyrd07, double_integrator_QP_solver_W08, double_integrator_QP_solver_Lbyrd08, double_integrator_QP_solver_re08, double_integrator_QP_solver_beta08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta08, "double_integrator_QP_solver_beta08");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_V08, double_integrator_QP_solver_W09, double_integrator_QP_solver_Yd09);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd09, "double_integrator_QP_solver_Yd09");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_V08, double_integrator_QP_solver_Lbyrd08, double_integrator_QP_solver_W09, double_integrator_QP_solver_Lbyrd09, double_integrator_QP_solver_re09, double_integrator_QP_solver_beta09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta09, "double_integrator_QP_solver_beta09");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_V09, double_integrator_QP_solver_W10, double_integrator_QP_solver_Yd10);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd10, "double_integrator_QP_solver_Yd10");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_V09, double_integrator_QP_solver_Lbyrd09, double_integrator_QP_solver_W10, double_integrator_QP_solver_Lbyrd10, double_integrator_QP_solver_re10, double_integrator_QP_solver_beta10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta10, "double_integrator_QP_solver_beta10");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_V10, double_integrator_QP_solver_W11, double_integrator_QP_solver_Yd11);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd11, "double_integrator_QP_solver_Yd11");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_V10, double_integrator_QP_solver_Lbyrd10, double_integrator_QP_solver_W11, double_integrator_QP_solver_Lbyrd11, double_integrator_QP_solver_re11, double_integrator_QP_solver_beta11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta11, "double_integrator_QP_solver_beta11");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_V11, double_integrator_QP_solver_W12, double_integrator_QP_solver_Yd12);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd12, "double_integrator_QP_solver_Yd12");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_V11, double_integrator_QP_solver_Lbyrd11, double_integrator_QP_solver_W12, double_integrator_QP_solver_Lbyrd12, double_integrator_QP_solver_re12, double_integrator_QP_solver_beta12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta12, "double_integrator_QP_solver_beta12");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_V12, double_integrator_QP_solver_W13, double_integrator_QP_solver_Yd13);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd13, "double_integrator_QP_solver_Yd13");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_V12, double_integrator_QP_solver_Lbyrd12, double_integrator_QP_solver_W13, double_integrator_QP_solver_Lbyrd13, double_integrator_QP_solver_re13, double_integrator_QP_solver_beta13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta13, "double_integrator_QP_solver_beta13");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_V13, double_integrator_QP_solver_W14, double_integrator_QP_solver_Yd14);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd14, "double_integrator_QP_solver_Yd14");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_V13, double_integrator_QP_solver_Lbyrd13, double_integrator_QP_solver_W14, double_integrator_QP_solver_Lbyrd14, double_integrator_QP_solver_re14, double_integrator_QP_solver_beta14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta14, "double_integrator_QP_solver_beta14");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_V14, double_integrator_QP_solver_W15, double_integrator_QP_solver_Yd15);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd15, "double_integrator_QP_solver_Yd15");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_V14, double_integrator_QP_solver_Lbyrd14, double_integrator_QP_solver_W15, double_integrator_QP_solver_Lbyrd15, double_integrator_QP_solver_re15, double_integrator_QP_solver_beta15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta15, "double_integrator_QP_solver_beta15");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_V15, double_integrator_QP_solver_W16, double_integrator_QP_solver_Yd16);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd16, "double_integrator_QP_solver_Yd16");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_V15, double_integrator_QP_solver_Lbyrd15, double_integrator_QP_solver_W16, double_integrator_QP_solver_Lbyrd16, double_integrator_QP_solver_re16, double_integrator_QP_solver_beta16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta16, "double_integrator_QP_solver_beta16");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_V16, double_integrator_QP_solver_W17, double_integrator_QP_solver_Yd17);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd17, "double_integrator_QP_solver_Yd17");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_V16, double_integrator_QP_solver_Lbyrd16, double_integrator_QP_solver_W17, double_integrator_QP_solver_Lbyrd17, double_integrator_QP_solver_re17, double_integrator_QP_solver_beta17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta17, "double_integrator_QP_solver_beta17");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_12(double_integrator_QP_solver_V17, double_integrator_QP_solver_W18, double_integrator_QP_solver_Yd18);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Yd18, "double_integrator_QP_solver_Yd18");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_12(double_integrator_QP_solver_V17, double_integrator_QP_solver_Lbyrd17, double_integrator_QP_solver_W18, double_integrator_QP_solver_Lbyrd18, double_integrator_QP_solver_re18, double_integrator_QP_solver_beta18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta18, "double_integrator_QP_solver_beta18");
double_integrator_QP_solver_LA_DENSE_MMT2_1_12_5(double_integrator_QP_solver_V18, double_integrator_QP_solver_W19, double_integrator_QP_solver_Yd19);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_5(double_integrator_QP_solver_Yd19, "double_integrator_QP_solver_Yd19");
double_integrator_QP_solver_LA_DENSE_MVMSUB2_1_12_5(double_integrator_QP_solver_V18, double_integrator_QP_solver_Lbyrd18, double_integrator_QP_solver_W19, double_integrator_QP_solver_Lbyrd19, double_integrator_QP_solver_re19, double_integrator_QP_solver_beta19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_beta19, "double_integrator_QP_solver_beta19");
double_integrator_QP_solver_LA_DENSE_CHOL_12(double_integrator_QP_solver_Yd00, double_integrator_QP_solver_Ld00);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_12(double_integrator_QP_solver_Ld00, "double_integrator_QP_solver_Ld00");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Ld00, double_integrator_QP_solver_beta00, double_integrator_QP_solver_yy00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_yy00, "double_integrator_QP_solver_yy00");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_12(double_integrator_QP_solver_Ld00, double_integrator_QP_solver_Ysd01, double_integrator_QP_solver_Lsd01);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_12(double_integrator_QP_solver_Lsd01, "double_integrator_QP_solver_Lsd01");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_12(double_integrator_QP_solver_Lsd01, double_integrator_QP_solver_Yd01);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd01, double_integrator_QP_solver_Ld01);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld01, "double_integrator_QP_solver_Ld01");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_12(double_integrator_QP_solver_Lsd01, double_integrator_QP_solver_yy00, double_integrator_QP_solver_beta01, double_integrator_QP_solver_bmy01);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld01, double_integrator_QP_solver_bmy01, double_integrator_QP_solver_yy01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy01, "double_integrator_QP_solver_yy01");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_Ld01, double_integrator_QP_solver_Ysd02, double_integrator_QP_solver_Lsd02);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Lsd02, "double_integrator_QP_solver_Lsd02");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_Lsd02, double_integrator_QP_solver_Yd02);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd02, double_integrator_QP_solver_Ld02);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld02, "double_integrator_QP_solver_Ld02");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd02, double_integrator_QP_solver_yy01, double_integrator_QP_solver_beta02, double_integrator_QP_solver_bmy02);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld02, double_integrator_QP_solver_bmy02, double_integrator_QP_solver_yy02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy02, "double_integrator_QP_solver_yy02");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_Ld02, double_integrator_QP_solver_Ysd03, double_integrator_QP_solver_Lsd03);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Lsd03, "double_integrator_QP_solver_Lsd03");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_Lsd03, double_integrator_QP_solver_Yd03);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd03, double_integrator_QP_solver_Ld03);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld03, "double_integrator_QP_solver_Ld03");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd03, double_integrator_QP_solver_yy02, double_integrator_QP_solver_beta03, double_integrator_QP_solver_bmy03);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld03, double_integrator_QP_solver_bmy03, double_integrator_QP_solver_yy03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy03, "double_integrator_QP_solver_yy03");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_Ld03, double_integrator_QP_solver_Ysd04, double_integrator_QP_solver_Lsd04);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Lsd04, "double_integrator_QP_solver_Lsd04");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_Lsd04, double_integrator_QP_solver_Yd04);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd04, double_integrator_QP_solver_Ld04);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld04, "double_integrator_QP_solver_Ld04");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd04, double_integrator_QP_solver_yy03, double_integrator_QP_solver_beta04, double_integrator_QP_solver_bmy04);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld04, double_integrator_QP_solver_bmy04, double_integrator_QP_solver_yy04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy04, "double_integrator_QP_solver_yy04");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_Ld04, double_integrator_QP_solver_Ysd05, double_integrator_QP_solver_Lsd05);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Lsd05, "double_integrator_QP_solver_Lsd05");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_Lsd05, double_integrator_QP_solver_Yd05);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd05, double_integrator_QP_solver_Ld05);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld05, "double_integrator_QP_solver_Ld05");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd05, double_integrator_QP_solver_yy04, double_integrator_QP_solver_beta05, double_integrator_QP_solver_bmy05);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld05, double_integrator_QP_solver_bmy05, double_integrator_QP_solver_yy05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy05, "double_integrator_QP_solver_yy05");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_Ld05, double_integrator_QP_solver_Ysd06, double_integrator_QP_solver_Lsd06);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Lsd06, "double_integrator_QP_solver_Lsd06");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_Lsd06, double_integrator_QP_solver_Yd06);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd06, double_integrator_QP_solver_Ld06);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld06, "double_integrator_QP_solver_Ld06");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd06, double_integrator_QP_solver_yy05, double_integrator_QP_solver_beta06, double_integrator_QP_solver_bmy06);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld06, double_integrator_QP_solver_bmy06, double_integrator_QP_solver_yy06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy06, "double_integrator_QP_solver_yy06");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_Ld06, double_integrator_QP_solver_Ysd07, double_integrator_QP_solver_Lsd07);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Lsd07, "double_integrator_QP_solver_Lsd07");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_Lsd07, double_integrator_QP_solver_Yd07);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd07, double_integrator_QP_solver_Ld07);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld07, "double_integrator_QP_solver_Ld07");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd07, double_integrator_QP_solver_yy06, double_integrator_QP_solver_beta07, double_integrator_QP_solver_bmy07);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld07, double_integrator_QP_solver_bmy07, double_integrator_QP_solver_yy07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy07, "double_integrator_QP_solver_yy07");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_Ld07, double_integrator_QP_solver_Ysd08, double_integrator_QP_solver_Lsd08);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Lsd08, "double_integrator_QP_solver_Lsd08");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_Lsd08, double_integrator_QP_solver_Yd08);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd08, double_integrator_QP_solver_Ld08);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld08, "double_integrator_QP_solver_Ld08");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd08, double_integrator_QP_solver_yy07, double_integrator_QP_solver_beta08, double_integrator_QP_solver_bmy08);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld08, double_integrator_QP_solver_bmy08, double_integrator_QP_solver_yy08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy08, "double_integrator_QP_solver_yy08");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_Ld08, double_integrator_QP_solver_Ysd09, double_integrator_QP_solver_Lsd09);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Lsd09, "double_integrator_QP_solver_Lsd09");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_Lsd09, double_integrator_QP_solver_Yd09);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd09, double_integrator_QP_solver_Ld09);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld09, "double_integrator_QP_solver_Ld09");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd09, double_integrator_QP_solver_yy08, double_integrator_QP_solver_beta09, double_integrator_QP_solver_bmy09);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld09, double_integrator_QP_solver_bmy09, double_integrator_QP_solver_yy09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy09, "double_integrator_QP_solver_yy09");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_Ld09, double_integrator_QP_solver_Ysd10, double_integrator_QP_solver_Lsd10);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Lsd10, "double_integrator_QP_solver_Lsd10");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_Yd10);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd10, double_integrator_QP_solver_Ld10);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld10, "double_integrator_QP_solver_Ld10");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_yy09, double_integrator_QP_solver_beta10, double_integrator_QP_solver_bmy10);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_bmy10, double_integrator_QP_solver_yy10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy10, "double_integrator_QP_solver_yy10");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_Ysd11, double_integrator_QP_solver_Lsd11);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Lsd11, "double_integrator_QP_solver_Lsd11");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_Lsd11, double_integrator_QP_solver_Yd11);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd11, double_integrator_QP_solver_Ld11);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld11, "double_integrator_QP_solver_Ld11");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd11, double_integrator_QP_solver_yy10, double_integrator_QP_solver_beta11, double_integrator_QP_solver_bmy11);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld11, double_integrator_QP_solver_bmy11, double_integrator_QP_solver_yy11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy11, "double_integrator_QP_solver_yy11");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_Ld11, double_integrator_QP_solver_Ysd12, double_integrator_QP_solver_Lsd12);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Lsd12, "double_integrator_QP_solver_Lsd12");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_Lsd12, double_integrator_QP_solver_Yd12);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd12, double_integrator_QP_solver_Ld12);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld12, "double_integrator_QP_solver_Ld12");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd12, double_integrator_QP_solver_yy11, double_integrator_QP_solver_beta12, double_integrator_QP_solver_bmy12);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld12, double_integrator_QP_solver_bmy12, double_integrator_QP_solver_yy12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy12, "double_integrator_QP_solver_yy12");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_Ld12, double_integrator_QP_solver_Ysd13, double_integrator_QP_solver_Lsd13);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Lsd13, "double_integrator_QP_solver_Lsd13");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_Lsd13, double_integrator_QP_solver_Yd13);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd13, double_integrator_QP_solver_Ld13);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld13, "double_integrator_QP_solver_Ld13");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd13, double_integrator_QP_solver_yy12, double_integrator_QP_solver_beta13, double_integrator_QP_solver_bmy13);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld13, double_integrator_QP_solver_bmy13, double_integrator_QP_solver_yy13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy13, "double_integrator_QP_solver_yy13");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_Ld13, double_integrator_QP_solver_Ysd14, double_integrator_QP_solver_Lsd14);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Lsd14, "double_integrator_QP_solver_Lsd14");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_Lsd14, double_integrator_QP_solver_Yd14);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd14, double_integrator_QP_solver_Ld14);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld14, "double_integrator_QP_solver_Ld14");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd14, double_integrator_QP_solver_yy13, double_integrator_QP_solver_beta14, double_integrator_QP_solver_bmy14);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld14, double_integrator_QP_solver_bmy14, double_integrator_QP_solver_yy14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy14, "double_integrator_QP_solver_yy14");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_Ld14, double_integrator_QP_solver_Ysd15, double_integrator_QP_solver_Lsd15);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Lsd15, "double_integrator_QP_solver_Lsd15");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_Lsd15, double_integrator_QP_solver_Yd15);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd15, double_integrator_QP_solver_Ld15);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld15, "double_integrator_QP_solver_Ld15");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd15, double_integrator_QP_solver_yy14, double_integrator_QP_solver_beta15, double_integrator_QP_solver_bmy15);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld15, double_integrator_QP_solver_bmy15, double_integrator_QP_solver_yy15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy15, "double_integrator_QP_solver_yy15");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_Ld15, double_integrator_QP_solver_Ysd16, double_integrator_QP_solver_Lsd16);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Lsd16, "double_integrator_QP_solver_Lsd16");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_Lsd16, double_integrator_QP_solver_Yd16);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd16, double_integrator_QP_solver_Ld16);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld16, "double_integrator_QP_solver_Ld16");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd16, double_integrator_QP_solver_yy15, double_integrator_QP_solver_beta16, double_integrator_QP_solver_bmy16);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld16, double_integrator_QP_solver_bmy16, double_integrator_QP_solver_yy16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy16, "double_integrator_QP_solver_yy16");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_Ld16, double_integrator_QP_solver_Ysd17, double_integrator_QP_solver_Lsd17);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Lsd17, "double_integrator_QP_solver_Lsd17");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_Lsd17, double_integrator_QP_solver_Yd17);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd17, double_integrator_QP_solver_Ld17);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld17, "double_integrator_QP_solver_Ld17");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd17, double_integrator_QP_solver_yy16, double_integrator_QP_solver_beta17, double_integrator_QP_solver_bmy17);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld17, double_integrator_QP_solver_bmy17, double_integrator_QP_solver_yy17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy17, "double_integrator_QP_solver_yy17");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_1_1(double_integrator_QP_solver_Ld17, double_integrator_QP_solver_Ysd18, double_integrator_QP_solver_Lsd18);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_1_1(double_integrator_QP_solver_Lsd18, "double_integrator_QP_solver_Lsd18");
double_integrator_QP_solver_LA_DENSE_MMTSUB_1_1(double_integrator_QP_solver_Lsd18, double_integrator_QP_solver_Yd18);
double_integrator_QP_solver_LA_DENSE_CHOL_1(double_integrator_QP_solver_Yd18, double_integrator_QP_solver_Ld18);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_1(double_integrator_QP_solver_Ld18, "double_integrator_QP_solver_Ld18");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd18, double_integrator_QP_solver_yy17, double_integrator_QP_solver_beta18, double_integrator_QP_solver_bmy18);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld18, double_integrator_QP_solver_bmy18, double_integrator_QP_solver_yy18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy18, "double_integrator_QP_solver_yy18");
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_5_1(double_integrator_QP_solver_Ld18, double_integrator_QP_solver_Ysd19, double_integrator_QP_solver_Lsd19);
double_integrator_QP_solver_LA_DENSE_PRINT_MATRIX_CM_5_1(double_integrator_QP_solver_Lsd19, "double_integrator_QP_solver_Lsd19");
double_integrator_QP_solver_LA_DENSE_MMTSUB_5_1(double_integrator_QP_solver_Lsd19, double_integrator_QP_solver_Yd19);
double_integrator_QP_solver_LA_DENSE_CHOL_5(double_integrator_QP_solver_Yd19, double_integrator_QP_solver_Ld19);
double_integrator_QP_solver_PRINT_TRIANGULAR_MATRIX_5(double_integrator_QP_solver_Ld19, "double_integrator_QP_solver_Ld19");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_1(double_integrator_QP_solver_Lsd19, double_integrator_QP_solver_yy18, double_integrator_QP_solver_beta19, double_integrator_QP_solver_bmy19);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld19, double_integrator_QP_solver_bmy19, double_integrator_QP_solver_yy19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_yy19, "double_integrator_QP_solver_yy19");
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld19, double_integrator_QP_solver_yy19, double_integrator_QP_solver_dvaff19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_dvaff19, "double_integrator_QP_solver_dvaff19");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_1(double_integrator_QP_solver_Lsd19, double_integrator_QP_solver_dvaff19, double_integrator_QP_solver_yy18, double_integrator_QP_solver_bmy18);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld18, double_integrator_QP_solver_bmy18, double_integrator_QP_solver_dvaff18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff18, "double_integrator_QP_solver_dvaff18");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd18, double_integrator_QP_solver_dvaff18, double_integrator_QP_solver_yy17, double_integrator_QP_solver_bmy17);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld17, double_integrator_QP_solver_bmy17, double_integrator_QP_solver_dvaff17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff17, "double_integrator_QP_solver_dvaff17");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd17, double_integrator_QP_solver_dvaff17, double_integrator_QP_solver_yy16, double_integrator_QP_solver_bmy16);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld16, double_integrator_QP_solver_bmy16, double_integrator_QP_solver_dvaff16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff16, "double_integrator_QP_solver_dvaff16");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd16, double_integrator_QP_solver_dvaff16, double_integrator_QP_solver_yy15, double_integrator_QP_solver_bmy15);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld15, double_integrator_QP_solver_bmy15, double_integrator_QP_solver_dvaff15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff15, "double_integrator_QP_solver_dvaff15");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd15, double_integrator_QP_solver_dvaff15, double_integrator_QP_solver_yy14, double_integrator_QP_solver_bmy14);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld14, double_integrator_QP_solver_bmy14, double_integrator_QP_solver_dvaff14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff14, "double_integrator_QP_solver_dvaff14");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd14, double_integrator_QP_solver_dvaff14, double_integrator_QP_solver_yy13, double_integrator_QP_solver_bmy13);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld13, double_integrator_QP_solver_bmy13, double_integrator_QP_solver_dvaff13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff13, "double_integrator_QP_solver_dvaff13");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd13, double_integrator_QP_solver_dvaff13, double_integrator_QP_solver_yy12, double_integrator_QP_solver_bmy12);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld12, double_integrator_QP_solver_bmy12, double_integrator_QP_solver_dvaff12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff12, "double_integrator_QP_solver_dvaff12");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd12, double_integrator_QP_solver_dvaff12, double_integrator_QP_solver_yy11, double_integrator_QP_solver_bmy11);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld11, double_integrator_QP_solver_bmy11, double_integrator_QP_solver_dvaff11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff11, "double_integrator_QP_solver_dvaff11");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd11, double_integrator_QP_solver_dvaff11, double_integrator_QP_solver_yy10, double_integrator_QP_solver_bmy10);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_bmy10, double_integrator_QP_solver_dvaff10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff10, "double_integrator_QP_solver_dvaff10");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_dvaff10, double_integrator_QP_solver_yy09, double_integrator_QP_solver_bmy09);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld09, double_integrator_QP_solver_bmy09, double_integrator_QP_solver_dvaff09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff09, "double_integrator_QP_solver_dvaff09");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd09, double_integrator_QP_solver_dvaff09, double_integrator_QP_solver_yy08, double_integrator_QP_solver_bmy08);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld08, double_integrator_QP_solver_bmy08, double_integrator_QP_solver_dvaff08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff08, "double_integrator_QP_solver_dvaff08");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd08, double_integrator_QP_solver_dvaff08, double_integrator_QP_solver_yy07, double_integrator_QP_solver_bmy07);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld07, double_integrator_QP_solver_bmy07, double_integrator_QP_solver_dvaff07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff07, "double_integrator_QP_solver_dvaff07");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd07, double_integrator_QP_solver_dvaff07, double_integrator_QP_solver_yy06, double_integrator_QP_solver_bmy06);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld06, double_integrator_QP_solver_bmy06, double_integrator_QP_solver_dvaff06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff06, "double_integrator_QP_solver_dvaff06");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd06, double_integrator_QP_solver_dvaff06, double_integrator_QP_solver_yy05, double_integrator_QP_solver_bmy05);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld05, double_integrator_QP_solver_bmy05, double_integrator_QP_solver_dvaff05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff05, "double_integrator_QP_solver_dvaff05");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd05, double_integrator_QP_solver_dvaff05, double_integrator_QP_solver_yy04, double_integrator_QP_solver_bmy04);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld04, double_integrator_QP_solver_bmy04, double_integrator_QP_solver_dvaff04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff04, "double_integrator_QP_solver_dvaff04");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd04, double_integrator_QP_solver_dvaff04, double_integrator_QP_solver_yy03, double_integrator_QP_solver_bmy03);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld03, double_integrator_QP_solver_bmy03, double_integrator_QP_solver_dvaff03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff03, "double_integrator_QP_solver_dvaff03");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd03, double_integrator_QP_solver_dvaff03, double_integrator_QP_solver_yy02, double_integrator_QP_solver_bmy02);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld02, double_integrator_QP_solver_bmy02, double_integrator_QP_solver_dvaff02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff02, "double_integrator_QP_solver_dvaff02");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd02, double_integrator_QP_solver_dvaff02, double_integrator_QP_solver_yy01, double_integrator_QP_solver_bmy01);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld01, double_integrator_QP_solver_bmy01, double_integrator_QP_solver_dvaff01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvaff01, "double_integrator_QP_solver_dvaff01");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_12(double_integrator_QP_solver_Lsd01, double_integrator_QP_solver_dvaff01, double_integrator_QP_solver_yy00, double_integrator_QP_solver_bmy00);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_12(double_integrator_QP_solver_Ld00, double_integrator_QP_solver_bmy00, double_integrator_QP_solver_dvaff00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dvaff00, "double_integrator_QP_solver_dvaff00");
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff01, double_integrator_QP_solver_D00, double_integrator_QP_solver_dvaff00, double_integrator_QP_solver_grad_eq00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq00, "double_integrator_QP_solver_grad_eq00");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff02, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff01, double_integrator_QP_solver_grad_eq01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq01, "double_integrator_QP_solver_grad_eq01");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff03, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff02, double_integrator_QP_solver_grad_eq02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq02, "double_integrator_QP_solver_grad_eq02");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff04, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff03, double_integrator_QP_solver_grad_eq03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq03, "double_integrator_QP_solver_grad_eq03");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff05, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff04, double_integrator_QP_solver_grad_eq04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq04, "double_integrator_QP_solver_grad_eq04");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff06, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff05, double_integrator_QP_solver_grad_eq05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq05, "double_integrator_QP_solver_grad_eq05");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff07, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff06, double_integrator_QP_solver_grad_eq06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq06, "double_integrator_QP_solver_grad_eq06");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff08, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff07, double_integrator_QP_solver_grad_eq07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq07, "double_integrator_QP_solver_grad_eq07");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff09, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff08, double_integrator_QP_solver_grad_eq08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq08, "double_integrator_QP_solver_grad_eq08");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff10, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff09, double_integrator_QP_solver_grad_eq09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq09, "double_integrator_QP_solver_grad_eq09");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff11, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff10, double_integrator_QP_solver_grad_eq10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq10, "double_integrator_QP_solver_grad_eq10");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff12, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff11, double_integrator_QP_solver_grad_eq11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq11, "double_integrator_QP_solver_grad_eq11");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff13, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff12, double_integrator_QP_solver_grad_eq12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq12, "double_integrator_QP_solver_grad_eq12");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff14, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff13, double_integrator_QP_solver_grad_eq13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq13, "double_integrator_QP_solver_grad_eq13");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff15, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff14, double_integrator_QP_solver_grad_eq14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq14, "double_integrator_QP_solver_grad_eq14");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff16, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff15, double_integrator_QP_solver_grad_eq15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq15, "double_integrator_QP_solver_grad_eq15");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff17, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff16, double_integrator_QP_solver_grad_eq16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq16, "double_integrator_QP_solver_grad_eq16");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff18, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff17, double_integrator_QP_solver_grad_eq17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq17, "double_integrator_QP_solver_grad_eq17");
double_integrator_QP_solver_LA_DENSE_MTVM2_5_12_1(double_integrator_QP_solver_C18, double_integrator_QP_solver_dvaff19, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff18, double_integrator_QP_solver_grad_eq18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq18, "double_integrator_QP_solver_grad_eq18");
double_integrator_QP_solver_LA_DIAGZERO_MTVM_5_5(double_integrator_QP_solver_D19, double_integrator_QP_solver_dvaff19, double_integrator_QP_solver_grad_eq19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_grad_eq19, "double_integrator_QP_solver_grad_eq19");
double_integrator_QP_solver_LA_VSUB2_233(double_integrator_QP_solver_rd, double_integrator_QP_solver_grad_eq, double_integrator_QP_solver_rd);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_rd00, double_integrator_QP_solver_dzaff00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff00, "double_integrator_QP_solver_dzaff00");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_rd01, double_integrator_QP_solver_dzaff01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff01, "double_integrator_QP_solver_dzaff01");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_rd02, double_integrator_QP_solver_dzaff02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff02, "double_integrator_QP_solver_dzaff02");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_rd03, double_integrator_QP_solver_dzaff03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff03, "double_integrator_QP_solver_dzaff03");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_rd04, double_integrator_QP_solver_dzaff04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff04, "double_integrator_QP_solver_dzaff04");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_rd05, double_integrator_QP_solver_dzaff05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff05, "double_integrator_QP_solver_dzaff05");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_rd06, double_integrator_QP_solver_dzaff06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff06, "double_integrator_QP_solver_dzaff06");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_rd07, double_integrator_QP_solver_dzaff07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff07, "double_integrator_QP_solver_dzaff07");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_rd08, double_integrator_QP_solver_dzaff08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff08, "double_integrator_QP_solver_dzaff08");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_rd09, double_integrator_QP_solver_dzaff09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff09, "double_integrator_QP_solver_dzaff09");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_rd10, double_integrator_QP_solver_dzaff10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff10, "double_integrator_QP_solver_dzaff10");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_rd11, double_integrator_QP_solver_dzaff11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff11, "double_integrator_QP_solver_dzaff11");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi12, double_integrator_QP_solver_rd12, double_integrator_QP_solver_dzaff12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff12, "double_integrator_QP_solver_dzaff12");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi13, double_integrator_QP_solver_rd13, double_integrator_QP_solver_dzaff13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff13, "double_integrator_QP_solver_dzaff13");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi14, double_integrator_QP_solver_rd14, double_integrator_QP_solver_dzaff14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff14, "double_integrator_QP_solver_dzaff14");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi15, double_integrator_QP_solver_rd15, double_integrator_QP_solver_dzaff15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff15, "double_integrator_QP_solver_dzaff15");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi16, double_integrator_QP_solver_rd16, double_integrator_QP_solver_dzaff16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff16, "double_integrator_QP_solver_dzaff16");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi17, double_integrator_QP_solver_rd17, double_integrator_QP_solver_dzaff17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff17, "double_integrator_QP_solver_dzaff17");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi18, double_integrator_QP_solver_rd18, double_integrator_QP_solver_dzaff18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzaff18, "double_integrator_QP_solver_dzaff18");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_5(double_integrator_QP_solver_Phi19, double_integrator_QP_solver_rd19, double_integrator_QP_solver_dzaff19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_dzaff19, "double_integrator_QP_solver_dzaff19");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_rilb00, double_integrator_QP_solver_dslbaff00);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb00, double_integrator_QP_solver_dslbaff00, double_integrator_QP_solver_llb00, double_integrator_QP_solver_dllbaff00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff00, "double_integrator_QP_solver_dslbaff00");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff00, "double_integrator_QP_solver_dllbaff00");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub00, double_integrator_QP_solver_dzaff00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_dsubaff00);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub00, double_integrator_QP_solver_dsubaff00, double_integrator_QP_solver_lub00, double_integrator_QP_solver_dlubaff00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff00, "double_integrator_QP_solver_dsubaff00");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff00, "double_integrator_QP_solver_dlubaff00");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A1, double_integrator_QP_solver_dzaff00, double_integrator_QP_solver_rip00, double_integrator_QP_solver_dsp_aff00);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp00, double_integrator_QP_solver_dsp_aff00, double_integrator_QP_solver_lp00, double_integrator_QP_solver_dlp_aff00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff00, "double_integrator_QP_solver_dsp_aff00");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff00, "double_integrator_QP_solver_dlp_aff00");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_rilb01, double_integrator_QP_solver_dslbaff01);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb01, double_integrator_QP_solver_dslbaff01, double_integrator_QP_solver_llb01, double_integrator_QP_solver_dllbaff01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff01, "double_integrator_QP_solver_dslbaff01");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff01, "double_integrator_QP_solver_dllbaff01");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub01, double_integrator_QP_solver_dzaff01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_dsubaff01);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub01, double_integrator_QP_solver_dsubaff01, double_integrator_QP_solver_lub01, double_integrator_QP_solver_dlubaff01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff01, "double_integrator_QP_solver_dsubaff01");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff01, "double_integrator_QP_solver_dlubaff01");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A2, double_integrator_QP_solver_dzaff01, double_integrator_QP_solver_rip01, double_integrator_QP_solver_dsp_aff01);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp01, double_integrator_QP_solver_dsp_aff01, double_integrator_QP_solver_lp01, double_integrator_QP_solver_dlp_aff01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff01, "double_integrator_QP_solver_dsp_aff01");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff01, "double_integrator_QP_solver_dlp_aff01");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_rilb02, double_integrator_QP_solver_dslbaff02);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb02, double_integrator_QP_solver_dslbaff02, double_integrator_QP_solver_llb02, double_integrator_QP_solver_dllbaff02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff02, "double_integrator_QP_solver_dslbaff02");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff02, "double_integrator_QP_solver_dllbaff02");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub02, double_integrator_QP_solver_dzaff02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_dsubaff02);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub02, double_integrator_QP_solver_dsubaff02, double_integrator_QP_solver_lub02, double_integrator_QP_solver_dlubaff02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff02, "double_integrator_QP_solver_dsubaff02");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff02, "double_integrator_QP_solver_dlubaff02");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A3, double_integrator_QP_solver_dzaff02, double_integrator_QP_solver_rip02, double_integrator_QP_solver_dsp_aff02);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp02, double_integrator_QP_solver_dsp_aff02, double_integrator_QP_solver_lp02, double_integrator_QP_solver_dlp_aff02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff02, "double_integrator_QP_solver_dsp_aff02");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff02, "double_integrator_QP_solver_dlp_aff02");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_rilb03, double_integrator_QP_solver_dslbaff03);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb03, double_integrator_QP_solver_dslbaff03, double_integrator_QP_solver_llb03, double_integrator_QP_solver_dllbaff03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff03, "double_integrator_QP_solver_dslbaff03");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff03, "double_integrator_QP_solver_dllbaff03");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub03, double_integrator_QP_solver_dzaff03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_dsubaff03);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub03, double_integrator_QP_solver_dsubaff03, double_integrator_QP_solver_lub03, double_integrator_QP_solver_dlubaff03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff03, "double_integrator_QP_solver_dsubaff03");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff03, "double_integrator_QP_solver_dlubaff03");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A4, double_integrator_QP_solver_dzaff03, double_integrator_QP_solver_rip03, double_integrator_QP_solver_dsp_aff03);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp03, double_integrator_QP_solver_dsp_aff03, double_integrator_QP_solver_lp03, double_integrator_QP_solver_dlp_aff03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff03, "double_integrator_QP_solver_dsp_aff03");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff03, "double_integrator_QP_solver_dlp_aff03");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_rilb04, double_integrator_QP_solver_dslbaff04);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb04, double_integrator_QP_solver_dslbaff04, double_integrator_QP_solver_llb04, double_integrator_QP_solver_dllbaff04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff04, "double_integrator_QP_solver_dslbaff04");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff04, "double_integrator_QP_solver_dllbaff04");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub04, double_integrator_QP_solver_dzaff04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_dsubaff04);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub04, double_integrator_QP_solver_dsubaff04, double_integrator_QP_solver_lub04, double_integrator_QP_solver_dlubaff04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff04, "double_integrator_QP_solver_dsubaff04");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff04, "double_integrator_QP_solver_dlubaff04");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A5, double_integrator_QP_solver_dzaff04, double_integrator_QP_solver_rip04, double_integrator_QP_solver_dsp_aff04);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp04, double_integrator_QP_solver_dsp_aff04, double_integrator_QP_solver_lp04, double_integrator_QP_solver_dlp_aff04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff04, "double_integrator_QP_solver_dsp_aff04");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff04, "double_integrator_QP_solver_dlp_aff04");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_rilb05, double_integrator_QP_solver_dslbaff05);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb05, double_integrator_QP_solver_dslbaff05, double_integrator_QP_solver_llb05, double_integrator_QP_solver_dllbaff05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff05, "double_integrator_QP_solver_dslbaff05");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff05, "double_integrator_QP_solver_dllbaff05");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub05, double_integrator_QP_solver_dzaff05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_dsubaff05);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub05, double_integrator_QP_solver_dsubaff05, double_integrator_QP_solver_lub05, double_integrator_QP_solver_dlubaff05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff05, "double_integrator_QP_solver_dsubaff05");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff05, "double_integrator_QP_solver_dlubaff05");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A6, double_integrator_QP_solver_dzaff05, double_integrator_QP_solver_rip05, double_integrator_QP_solver_dsp_aff05);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp05, double_integrator_QP_solver_dsp_aff05, double_integrator_QP_solver_lp05, double_integrator_QP_solver_dlp_aff05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff05, "double_integrator_QP_solver_dsp_aff05");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff05, "double_integrator_QP_solver_dlp_aff05");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_rilb06, double_integrator_QP_solver_dslbaff06);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb06, double_integrator_QP_solver_dslbaff06, double_integrator_QP_solver_llb06, double_integrator_QP_solver_dllbaff06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff06, "double_integrator_QP_solver_dslbaff06");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff06, "double_integrator_QP_solver_dllbaff06");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub06, double_integrator_QP_solver_dzaff06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_dsubaff06);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub06, double_integrator_QP_solver_dsubaff06, double_integrator_QP_solver_lub06, double_integrator_QP_solver_dlubaff06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff06, "double_integrator_QP_solver_dsubaff06");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff06, "double_integrator_QP_solver_dlubaff06");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A7, double_integrator_QP_solver_dzaff06, double_integrator_QP_solver_rip06, double_integrator_QP_solver_dsp_aff06);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp06, double_integrator_QP_solver_dsp_aff06, double_integrator_QP_solver_lp06, double_integrator_QP_solver_dlp_aff06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff06, "double_integrator_QP_solver_dsp_aff06");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff06, "double_integrator_QP_solver_dlp_aff06");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_rilb07, double_integrator_QP_solver_dslbaff07);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb07, double_integrator_QP_solver_dslbaff07, double_integrator_QP_solver_llb07, double_integrator_QP_solver_dllbaff07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff07, "double_integrator_QP_solver_dslbaff07");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff07, "double_integrator_QP_solver_dllbaff07");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub07, double_integrator_QP_solver_dzaff07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_dsubaff07);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub07, double_integrator_QP_solver_dsubaff07, double_integrator_QP_solver_lub07, double_integrator_QP_solver_dlubaff07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff07, "double_integrator_QP_solver_dsubaff07");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff07, "double_integrator_QP_solver_dlubaff07");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A8, double_integrator_QP_solver_dzaff07, double_integrator_QP_solver_rip07, double_integrator_QP_solver_dsp_aff07);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp07, double_integrator_QP_solver_dsp_aff07, double_integrator_QP_solver_lp07, double_integrator_QP_solver_dlp_aff07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff07, "double_integrator_QP_solver_dsp_aff07");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff07, "double_integrator_QP_solver_dlp_aff07");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_rilb08, double_integrator_QP_solver_dslbaff08);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb08, double_integrator_QP_solver_dslbaff08, double_integrator_QP_solver_llb08, double_integrator_QP_solver_dllbaff08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff08, "double_integrator_QP_solver_dslbaff08");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff08, "double_integrator_QP_solver_dllbaff08");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub08, double_integrator_QP_solver_dzaff08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_dsubaff08);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub08, double_integrator_QP_solver_dsubaff08, double_integrator_QP_solver_lub08, double_integrator_QP_solver_dlubaff08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff08, "double_integrator_QP_solver_dsubaff08");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff08, "double_integrator_QP_solver_dlubaff08");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A9, double_integrator_QP_solver_dzaff08, double_integrator_QP_solver_rip08, double_integrator_QP_solver_dsp_aff08);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp08, double_integrator_QP_solver_dsp_aff08, double_integrator_QP_solver_lp08, double_integrator_QP_solver_dlp_aff08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff08, "double_integrator_QP_solver_dsp_aff08");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff08, "double_integrator_QP_solver_dlp_aff08");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_rilb09, double_integrator_QP_solver_dslbaff09);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb09, double_integrator_QP_solver_dslbaff09, double_integrator_QP_solver_llb09, double_integrator_QP_solver_dllbaff09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff09, "double_integrator_QP_solver_dslbaff09");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff09, "double_integrator_QP_solver_dllbaff09");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub09, double_integrator_QP_solver_dzaff09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_dsubaff09);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub09, double_integrator_QP_solver_dsubaff09, double_integrator_QP_solver_lub09, double_integrator_QP_solver_dlubaff09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff09, "double_integrator_QP_solver_dsubaff09");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff09, "double_integrator_QP_solver_dlubaff09");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A10, double_integrator_QP_solver_dzaff09, double_integrator_QP_solver_rip09, double_integrator_QP_solver_dsp_aff09);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp09, double_integrator_QP_solver_dsp_aff09, double_integrator_QP_solver_lp09, double_integrator_QP_solver_dlp_aff09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff09, "double_integrator_QP_solver_dsp_aff09");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff09, "double_integrator_QP_solver_dlp_aff09");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_rilb10, double_integrator_QP_solver_dslbaff10);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb10, double_integrator_QP_solver_dslbaff10, double_integrator_QP_solver_llb10, double_integrator_QP_solver_dllbaff10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff10, "double_integrator_QP_solver_dslbaff10");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff10, "double_integrator_QP_solver_dllbaff10");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub10, double_integrator_QP_solver_dzaff10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_dsubaff10);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub10, double_integrator_QP_solver_dsubaff10, double_integrator_QP_solver_lub10, double_integrator_QP_solver_dlubaff10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff10, "double_integrator_QP_solver_dsubaff10");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff10, "double_integrator_QP_solver_dlubaff10");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A11, double_integrator_QP_solver_dzaff10, double_integrator_QP_solver_rip10, double_integrator_QP_solver_dsp_aff10);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp10, double_integrator_QP_solver_dsp_aff10, double_integrator_QP_solver_lp10, double_integrator_QP_solver_dlp_aff10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff10, "double_integrator_QP_solver_dsp_aff10");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff10, "double_integrator_QP_solver_dlp_aff10");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_rilb11, double_integrator_QP_solver_dslbaff11);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb11, double_integrator_QP_solver_dslbaff11, double_integrator_QP_solver_llb11, double_integrator_QP_solver_dllbaff11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff11, "double_integrator_QP_solver_dslbaff11");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff11, "double_integrator_QP_solver_dllbaff11");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub11, double_integrator_QP_solver_dzaff11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_dsubaff11);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub11, double_integrator_QP_solver_dsubaff11, double_integrator_QP_solver_lub11, double_integrator_QP_solver_dlubaff11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff11, "double_integrator_QP_solver_dsubaff11");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff11, "double_integrator_QP_solver_dlubaff11");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A12, double_integrator_QP_solver_dzaff11, double_integrator_QP_solver_rip11, double_integrator_QP_solver_dsp_aff11);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp11, double_integrator_QP_solver_dsp_aff11, double_integrator_QP_solver_lp11, double_integrator_QP_solver_dlp_aff11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff11, "double_integrator_QP_solver_dsp_aff11");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff11, "double_integrator_QP_solver_dlp_aff11");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff12, double_integrator_QP_solver_lbIdx12, double_integrator_QP_solver_rilb12, double_integrator_QP_solver_dslbaff12);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb12, double_integrator_QP_solver_dslbaff12, double_integrator_QP_solver_llb12, double_integrator_QP_solver_dllbaff12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff12, "double_integrator_QP_solver_dslbaff12");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff12, "double_integrator_QP_solver_dllbaff12");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub12, double_integrator_QP_solver_dzaff12, double_integrator_QP_solver_ubIdx12, double_integrator_QP_solver_dsubaff12);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub12, double_integrator_QP_solver_dsubaff12, double_integrator_QP_solver_lub12, double_integrator_QP_solver_dlubaff12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff12, "double_integrator_QP_solver_dsubaff12");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff12, "double_integrator_QP_solver_dlubaff12");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A13, double_integrator_QP_solver_dzaff12, double_integrator_QP_solver_rip12, double_integrator_QP_solver_dsp_aff12);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp12, double_integrator_QP_solver_dsp_aff12, double_integrator_QP_solver_lp12, double_integrator_QP_solver_dlp_aff12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff12, "double_integrator_QP_solver_dsp_aff12");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff12, "double_integrator_QP_solver_dlp_aff12");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff13, double_integrator_QP_solver_lbIdx13, double_integrator_QP_solver_rilb13, double_integrator_QP_solver_dslbaff13);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb13, double_integrator_QP_solver_dslbaff13, double_integrator_QP_solver_llb13, double_integrator_QP_solver_dllbaff13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff13, "double_integrator_QP_solver_dslbaff13");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff13, "double_integrator_QP_solver_dllbaff13");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub13, double_integrator_QP_solver_dzaff13, double_integrator_QP_solver_ubIdx13, double_integrator_QP_solver_dsubaff13);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub13, double_integrator_QP_solver_dsubaff13, double_integrator_QP_solver_lub13, double_integrator_QP_solver_dlubaff13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff13, "double_integrator_QP_solver_dsubaff13");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff13, "double_integrator_QP_solver_dlubaff13");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A14, double_integrator_QP_solver_dzaff13, double_integrator_QP_solver_rip13, double_integrator_QP_solver_dsp_aff13);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp13, double_integrator_QP_solver_dsp_aff13, double_integrator_QP_solver_lp13, double_integrator_QP_solver_dlp_aff13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff13, "double_integrator_QP_solver_dsp_aff13");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff13, "double_integrator_QP_solver_dlp_aff13");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff14, double_integrator_QP_solver_lbIdx14, double_integrator_QP_solver_rilb14, double_integrator_QP_solver_dslbaff14);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb14, double_integrator_QP_solver_dslbaff14, double_integrator_QP_solver_llb14, double_integrator_QP_solver_dllbaff14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff14, "double_integrator_QP_solver_dslbaff14");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff14, "double_integrator_QP_solver_dllbaff14");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub14, double_integrator_QP_solver_dzaff14, double_integrator_QP_solver_ubIdx14, double_integrator_QP_solver_dsubaff14);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub14, double_integrator_QP_solver_dsubaff14, double_integrator_QP_solver_lub14, double_integrator_QP_solver_dlubaff14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff14, "double_integrator_QP_solver_dsubaff14");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff14, "double_integrator_QP_solver_dlubaff14");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A15, double_integrator_QP_solver_dzaff14, double_integrator_QP_solver_rip14, double_integrator_QP_solver_dsp_aff14);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp14, double_integrator_QP_solver_dsp_aff14, double_integrator_QP_solver_lp14, double_integrator_QP_solver_dlp_aff14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff14, "double_integrator_QP_solver_dsp_aff14");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff14, "double_integrator_QP_solver_dlp_aff14");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff15, double_integrator_QP_solver_lbIdx15, double_integrator_QP_solver_rilb15, double_integrator_QP_solver_dslbaff15);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb15, double_integrator_QP_solver_dslbaff15, double_integrator_QP_solver_llb15, double_integrator_QP_solver_dllbaff15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff15, "double_integrator_QP_solver_dslbaff15");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff15, "double_integrator_QP_solver_dllbaff15");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub15, double_integrator_QP_solver_dzaff15, double_integrator_QP_solver_ubIdx15, double_integrator_QP_solver_dsubaff15);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub15, double_integrator_QP_solver_dsubaff15, double_integrator_QP_solver_lub15, double_integrator_QP_solver_dlubaff15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff15, "double_integrator_QP_solver_dsubaff15");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff15, "double_integrator_QP_solver_dlubaff15");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A16, double_integrator_QP_solver_dzaff15, double_integrator_QP_solver_rip15, double_integrator_QP_solver_dsp_aff15);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp15, double_integrator_QP_solver_dsp_aff15, double_integrator_QP_solver_lp15, double_integrator_QP_solver_dlp_aff15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff15, "double_integrator_QP_solver_dsp_aff15");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff15, "double_integrator_QP_solver_dlp_aff15");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff16, double_integrator_QP_solver_lbIdx16, double_integrator_QP_solver_rilb16, double_integrator_QP_solver_dslbaff16);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb16, double_integrator_QP_solver_dslbaff16, double_integrator_QP_solver_llb16, double_integrator_QP_solver_dllbaff16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff16, "double_integrator_QP_solver_dslbaff16");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff16, "double_integrator_QP_solver_dllbaff16");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub16, double_integrator_QP_solver_dzaff16, double_integrator_QP_solver_ubIdx16, double_integrator_QP_solver_dsubaff16);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub16, double_integrator_QP_solver_dsubaff16, double_integrator_QP_solver_lub16, double_integrator_QP_solver_dlubaff16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff16, "double_integrator_QP_solver_dsubaff16");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff16, "double_integrator_QP_solver_dlubaff16");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A17, double_integrator_QP_solver_dzaff16, double_integrator_QP_solver_rip16, double_integrator_QP_solver_dsp_aff16);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp16, double_integrator_QP_solver_dsp_aff16, double_integrator_QP_solver_lp16, double_integrator_QP_solver_dlp_aff16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff16, "double_integrator_QP_solver_dsp_aff16");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff16, "double_integrator_QP_solver_dlp_aff16");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff17, double_integrator_QP_solver_lbIdx17, double_integrator_QP_solver_rilb17, double_integrator_QP_solver_dslbaff17);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb17, double_integrator_QP_solver_dslbaff17, double_integrator_QP_solver_llb17, double_integrator_QP_solver_dllbaff17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff17, "double_integrator_QP_solver_dslbaff17");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff17, "double_integrator_QP_solver_dllbaff17");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub17, double_integrator_QP_solver_dzaff17, double_integrator_QP_solver_ubIdx17, double_integrator_QP_solver_dsubaff17);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub17, double_integrator_QP_solver_dsubaff17, double_integrator_QP_solver_lub17, double_integrator_QP_solver_dlubaff17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff17, "double_integrator_QP_solver_dsubaff17");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff17, "double_integrator_QP_solver_dlubaff17");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A18, double_integrator_QP_solver_dzaff17, double_integrator_QP_solver_rip17, double_integrator_QP_solver_dsp_aff17);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp17, double_integrator_QP_solver_dsp_aff17, double_integrator_QP_solver_lp17, double_integrator_QP_solver_dlp_aff17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff17, "double_integrator_QP_solver_dsp_aff17");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff17, "double_integrator_QP_solver_dlp_aff17");
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff18, double_integrator_QP_solver_lbIdx18, double_integrator_QP_solver_rilb18, double_integrator_QP_solver_dslbaff18);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb18, double_integrator_QP_solver_dslbaff18, double_integrator_QP_solver_llb18, double_integrator_QP_solver_dllbaff18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbaff18, "double_integrator_QP_solver_dslbaff18");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbaff18, "double_integrator_QP_solver_dllbaff18");
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub18, double_integrator_QP_solver_dzaff18, double_integrator_QP_solver_ubIdx18, double_integrator_QP_solver_dsubaff18);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub18, double_integrator_QP_solver_dsubaff18, double_integrator_QP_solver_lub18, double_integrator_QP_solver_dlubaff18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubaff18, "double_integrator_QP_solver_dsubaff18");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubaff18, "double_integrator_QP_solver_dlubaff18");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_24_12(params->A19, double_integrator_QP_solver_dzaff18, double_integrator_QP_solver_rip18, double_integrator_QP_solver_dsp_aff18);
double_integrator_QP_solver_LA_VSUB3_24(double_integrator_QP_solver_lpbysp18, double_integrator_QP_solver_dsp_aff18, double_integrator_QP_solver_lp18, double_integrator_QP_solver_dlp_aff18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_aff18, "double_integrator_QP_solver_dsp_aff18");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_aff18, "double_integrator_QP_solver_dlp_aff18");
double_integrator_QP_solver_LA_VSUB_INDEXED_4(double_integrator_QP_solver_dzaff19, double_integrator_QP_solver_lbIdx19, double_integrator_QP_solver_rilb19, double_integrator_QP_solver_dslbaff19);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_llbbyslb19, double_integrator_QP_solver_dslbaff19, double_integrator_QP_solver_llb19, double_integrator_QP_solver_dllbaff19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_4(double_integrator_QP_solver_dslbaff19, "double_integrator_QP_solver_dslbaff19");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_4(double_integrator_QP_solver_dllbaff19, "double_integrator_QP_solver_dllbaff19");
double_integrator_QP_solver_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_riub19, double_integrator_QP_solver_dzaff19, double_integrator_QP_solver_ubIdx19, double_integrator_QP_solver_dsubaff19);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_lubbysub19, double_integrator_QP_solver_dsubaff19, double_integrator_QP_solver_lub19, double_integrator_QP_solver_dlubaff19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_4(double_integrator_QP_solver_dsubaff19, "double_integrator_QP_solver_dsubaff19");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_4(double_integrator_QP_solver_dlubaff19, "double_integrator_QP_solver_dlubaff19");
double_integrator_QP_solver_LA_DENSE_MVMSUB4_10_5(params->A20, double_integrator_QP_solver_dzaff19, double_integrator_QP_solver_rip19, double_integrator_QP_solver_dsp_aff19);
double_integrator_QP_solver_LA_VSUB3_10(double_integrator_QP_solver_lpbysp19, double_integrator_QP_solver_dsp_aff19, double_integrator_QP_solver_lp19, double_integrator_QP_solver_dlp_aff19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_10(double_integrator_QP_solver_dsp_aff19, "double_integrator_QP_solver_dsp_aff19");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_10(double_integrator_QP_solver_dlp_aff19, "double_integrator_QP_solver_dlp_aff19");
info->lsit_aff = double_integrator_QP_solver_LINESEARCH_BACKTRACKING_AFFINE(double_integrator_QP_solver_l, double_integrator_QP_solver_s, double_integrator_QP_solver_dl_aff, double_integrator_QP_solver_ds_aff, &info->step_aff, &info->mu_aff);
if( info->lsit_aff == double_integrator_QP_solver_NOPROGRESS ){
PRINTTEXT("Affine line search could not proceed at iteration %d.\nThe problem might be infeasible -- exiting.\n",info->it+1);
exitcode = double_integrator_QP_solver_NOPROGRESS; break;
}
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_721(double_integrator_QP_solver_l, "double_integrator_QP_solver_l");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_721(double_integrator_QP_solver_s, "double_integrator_QP_solver_s");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_721(double_integrator_QP_solver_dl_aff, "double_integrator_QP_solver_dl_aff");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_721(double_integrator_QP_solver_ds_aff, "double_integrator_QP_solver_ds_aff");
sigma_3rdroot = info->mu_aff / info->mu;
info->sigma = sigma_3rdroot*sigma_3rdroot*sigma_3rdroot;
musigma = info->mu * info->sigma;
double_integrator_QP_solver_LA_VSUB5_721(double_integrator_QP_solver_ds_aff, double_integrator_QP_solver_dl_aff, info->mu, info->sigma, double_integrator_QP_solver_ccrhs);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_721(double_integrator_QP_solver_ccrhs, "double_integrator_QP_solver_ccrhs");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub00, double_integrator_QP_solver_sub00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_ccrhsl00, double_integrator_QP_solver_slb00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_rd00);
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub01, double_integrator_QP_solver_sub01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_ccrhsl01, double_integrator_QP_solver_slb01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_rd01);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A1, double_integrator_QP_solver_ccrhsp00, double_integrator_QP_solver_sp00, double_integrator_QP_solver_rd00);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A2, double_integrator_QP_solver_ccrhsp01, double_integrator_QP_solver_sp01, double_integrator_QP_solver_rd01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd00, "double_integrator_QP_solver_rd00");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd01, "double_integrator_QP_solver_rd01");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_rd00, double_integrator_QP_solver_Lbyrd00);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_rd01, double_integrator_QP_solver_Lbyrd01);
double_integrator_QP_solver_LA_DENSE_MVM_12_12(double_integrator_QP_solver_W00, double_integrator_QP_solver_Lbyrd00, double_integrator_QP_solver_beta00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_beta00, "double_integrator_QP_solver_beta00");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Ld00, double_integrator_QP_solver_beta00, double_integrator_QP_solver_yy00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_yy00, "double_integrator_QP_solver_yy00");
double_integrator_QP_solver_LA_DENSE_2MVMADD_12_12_12(double_integrator_QP_solver_V00, double_integrator_QP_solver_Lbyrd00, double_integrator_QP_solver_W01, double_integrator_QP_solver_Lbyrd01, double_integrator_QP_solver_beta01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta01, "double_integrator_QP_solver_beta01");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_12(double_integrator_QP_solver_Lsd01, double_integrator_QP_solver_yy00, double_integrator_QP_solver_beta01, double_integrator_QP_solver_bmy01);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld01, double_integrator_QP_solver_bmy01, double_integrator_QP_solver_yy01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy01, "double_integrator_QP_solver_yy01");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub02, double_integrator_QP_solver_sub02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_ccrhsl02, double_integrator_QP_solver_slb02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_rd02);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A3, double_integrator_QP_solver_ccrhsp02, double_integrator_QP_solver_sp02, double_integrator_QP_solver_rd02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd02, "double_integrator_QP_solver_rd02");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_rd02, double_integrator_QP_solver_Lbyrd02);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_V01, double_integrator_QP_solver_Lbyrd01, double_integrator_QP_solver_W02, double_integrator_QP_solver_Lbyrd02, double_integrator_QP_solver_beta02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta02, "double_integrator_QP_solver_beta02");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd02, double_integrator_QP_solver_yy01, double_integrator_QP_solver_beta02, double_integrator_QP_solver_bmy02);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld02, double_integrator_QP_solver_bmy02, double_integrator_QP_solver_yy02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy02, "double_integrator_QP_solver_yy02");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub03, double_integrator_QP_solver_sub03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_ccrhsl03, double_integrator_QP_solver_slb03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_rd03);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A4, double_integrator_QP_solver_ccrhsp03, double_integrator_QP_solver_sp03, double_integrator_QP_solver_rd03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd03, "double_integrator_QP_solver_rd03");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_rd03, double_integrator_QP_solver_Lbyrd03);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_V02, double_integrator_QP_solver_Lbyrd02, double_integrator_QP_solver_W03, double_integrator_QP_solver_Lbyrd03, double_integrator_QP_solver_beta03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta03, "double_integrator_QP_solver_beta03");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd03, double_integrator_QP_solver_yy02, double_integrator_QP_solver_beta03, double_integrator_QP_solver_bmy03);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld03, double_integrator_QP_solver_bmy03, double_integrator_QP_solver_yy03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy03, "double_integrator_QP_solver_yy03");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub04, double_integrator_QP_solver_sub04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_ccrhsl04, double_integrator_QP_solver_slb04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_rd04);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A5, double_integrator_QP_solver_ccrhsp04, double_integrator_QP_solver_sp04, double_integrator_QP_solver_rd04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd04, "double_integrator_QP_solver_rd04");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_rd04, double_integrator_QP_solver_Lbyrd04);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_V03, double_integrator_QP_solver_Lbyrd03, double_integrator_QP_solver_W04, double_integrator_QP_solver_Lbyrd04, double_integrator_QP_solver_beta04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta04, "double_integrator_QP_solver_beta04");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd04, double_integrator_QP_solver_yy03, double_integrator_QP_solver_beta04, double_integrator_QP_solver_bmy04);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld04, double_integrator_QP_solver_bmy04, double_integrator_QP_solver_yy04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy04, "double_integrator_QP_solver_yy04");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub05, double_integrator_QP_solver_sub05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_ccrhsl05, double_integrator_QP_solver_slb05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_rd05);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A6, double_integrator_QP_solver_ccrhsp05, double_integrator_QP_solver_sp05, double_integrator_QP_solver_rd05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd05, "double_integrator_QP_solver_rd05");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_rd05, double_integrator_QP_solver_Lbyrd05);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_V04, double_integrator_QP_solver_Lbyrd04, double_integrator_QP_solver_W05, double_integrator_QP_solver_Lbyrd05, double_integrator_QP_solver_beta05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta05, "double_integrator_QP_solver_beta05");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd05, double_integrator_QP_solver_yy04, double_integrator_QP_solver_beta05, double_integrator_QP_solver_bmy05);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld05, double_integrator_QP_solver_bmy05, double_integrator_QP_solver_yy05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy05, "double_integrator_QP_solver_yy05");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub06, double_integrator_QP_solver_sub06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_ccrhsl06, double_integrator_QP_solver_slb06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_rd06);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A7, double_integrator_QP_solver_ccrhsp06, double_integrator_QP_solver_sp06, double_integrator_QP_solver_rd06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd06, "double_integrator_QP_solver_rd06");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_rd06, double_integrator_QP_solver_Lbyrd06);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_V05, double_integrator_QP_solver_Lbyrd05, double_integrator_QP_solver_W06, double_integrator_QP_solver_Lbyrd06, double_integrator_QP_solver_beta06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta06, "double_integrator_QP_solver_beta06");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd06, double_integrator_QP_solver_yy05, double_integrator_QP_solver_beta06, double_integrator_QP_solver_bmy06);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld06, double_integrator_QP_solver_bmy06, double_integrator_QP_solver_yy06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy06, "double_integrator_QP_solver_yy06");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub07, double_integrator_QP_solver_sub07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_ccrhsl07, double_integrator_QP_solver_slb07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_rd07);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A8, double_integrator_QP_solver_ccrhsp07, double_integrator_QP_solver_sp07, double_integrator_QP_solver_rd07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd07, "double_integrator_QP_solver_rd07");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_rd07, double_integrator_QP_solver_Lbyrd07);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_V06, double_integrator_QP_solver_Lbyrd06, double_integrator_QP_solver_W07, double_integrator_QP_solver_Lbyrd07, double_integrator_QP_solver_beta07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta07, "double_integrator_QP_solver_beta07");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd07, double_integrator_QP_solver_yy06, double_integrator_QP_solver_beta07, double_integrator_QP_solver_bmy07);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld07, double_integrator_QP_solver_bmy07, double_integrator_QP_solver_yy07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy07, "double_integrator_QP_solver_yy07");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub08, double_integrator_QP_solver_sub08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_ccrhsl08, double_integrator_QP_solver_slb08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_rd08);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A9, double_integrator_QP_solver_ccrhsp08, double_integrator_QP_solver_sp08, double_integrator_QP_solver_rd08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd08, "double_integrator_QP_solver_rd08");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_rd08, double_integrator_QP_solver_Lbyrd08);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_V07, double_integrator_QP_solver_Lbyrd07, double_integrator_QP_solver_W08, double_integrator_QP_solver_Lbyrd08, double_integrator_QP_solver_beta08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta08, "double_integrator_QP_solver_beta08");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd08, double_integrator_QP_solver_yy07, double_integrator_QP_solver_beta08, double_integrator_QP_solver_bmy08);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld08, double_integrator_QP_solver_bmy08, double_integrator_QP_solver_yy08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy08, "double_integrator_QP_solver_yy08");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub09, double_integrator_QP_solver_sub09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_ccrhsl09, double_integrator_QP_solver_slb09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_rd09);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A10, double_integrator_QP_solver_ccrhsp09, double_integrator_QP_solver_sp09, double_integrator_QP_solver_rd09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd09, "double_integrator_QP_solver_rd09");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_rd09, double_integrator_QP_solver_Lbyrd09);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_V08, double_integrator_QP_solver_Lbyrd08, double_integrator_QP_solver_W09, double_integrator_QP_solver_Lbyrd09, double_integrator_QP_solver_beta09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta09, "double_integrator_QP_solver_beta09");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd09, double_integrator_QP_solver_yy08, double_integrator_QP_solver_beta09, double_integrator_QP_solver_bmy09);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld09, double_integrator_QP_solver_bmy09, double_integrator_QP_solver_yy09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy09, "double_integrator_QP_solver_yy09");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub10, double_integrator_QP_solver_sub10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_ccrhsl10, double_integrator_QP_solver_slb10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_rd10);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A11, double_integrator_QP_solver_ccrhsp10, double_integrator_QP_solver_sp10, double_integrator_QP_solver_rd10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd10, "double_integrator_QP_solver_rd10");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_rd10, double_integrator_QP_solver_Lbyrd10);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_V09, double_integrator_QP_solver_Lbyrd09, double_integrator_QP_solver_W10, double_integrator_QP_solver_Lbyrd10, double_integrator_QP_solver_beta10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta10, "double_integrator_QP_solver_beta10");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_yy09, double_integrator_QP_solver_beta10, double_integrator_QP_solver_bmy10);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_bmy10, double_integrator_QP_solver_yy10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy10, "double_integrator_QP_solver_yy10");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub11, double_integrator_QP_solver_sub11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_ccrhsl11, double_integrator_QP_solver_slb11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_rd11);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A12, double_integrator_QP_solver_ccrhsp11, double_integrator_QP_solver_sp11, double_integrator_QP_solver_rd11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd11, "double_integrator_QP_solver_rd11");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_rd11, double_integrator_QP_solver_Lbyrd11);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_V10, double_integrator_QP_solver_Lbyrd10, double_integrator_QP_solver_W11, double_integrator_QP_solver_Lbyrd11, double_integrator_QP_solver_beta11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta11, "double_integrator_QP_solver_beta11");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd11, double_integrator_QP_solver_yy10, double_integrator_QP_solver_beta11, double_integrator_QP_solver_bmy11);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld11, double_integrator_QP_solver_bmy11, double_integrator_QP_solver_yy11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy11, "double_integrator_QP_solver_yy11");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub12, double_integrator_QP_solver_sub12, double_integrator_QP_solver_ubIdx12, double_integrator_QP_solver_ccrhsl12, double_integrator_QP_solver_slb12, double_integrator_QP_solver_lbIdx12, double_integrator_QP_solver_rd12);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A13, double_integrator_QP_solver_ccrhsp12, double_integrator_QP_solver_sp12, double_integrator_QP_solver_rd12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd12, "double_integrator_QP_solver_rd12");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi12, double_integrator_QP_solver_rd12, double_integrator_QP_solver_Lbyrd12);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_V11, double_integrator_QP_solver_Lbyrd11, double_integrator_QP_solver_W12, double_integrator_QP_solver_Lbyrd12, double_integrator_QP_solver_beta12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta12, "double_integrator_QP_solver_beta12");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd12, double_integrator_QP_solver_yy11, double_integrator_QP_solver_beta12, double_integrator_QP_solver_bmy12);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld12, double_integrator_QP_solver_bmy12, double_integrator_QP_solver_yy12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy12, "double_integrator_QP_solver_yy12");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub13, double_integrator_QP_solver_sub13, double_integrator_QP_solver_ubIdx13, double_integrator_QP_solver_ccrhsl13, double_integrator_QP_solver_slb13, double_integrator_QP_solver_lbIdx13, double_integrator_QP_solver_rd13);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A14, double_integrator_QP_solver_ccrhsp13, double_integrator_QP_solver_sp13, double_integrator_QP_solver_rd13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd13, "double_integrator_QP_solver_rd13");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi13, double_integrator_QP_solver_rd13, double_integrator_QP_solver_Lbyrd13);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_V12, double_integrator_QP_solver_Lbyrd12, double_integrator_QP_solver_W13, double_integrator_QP_solver_Lbyrd13, double_integrator_QP_solver_beta13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta13, "double_integrator_QP_solver_beta13");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd13, double_integrator_QP_solver_yy12, double_integrator_QP_solver_beta13, double_integrator_QP_solver_bmy13);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld13, double_integrator_QP_solver_bmy13, double_integrator_QP_solver_yy13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy13, "double_integrator_QP_solver_yy13");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub14, double_integrator_QP_solver_sub14, double_integrator_QP_solver_ubIdx14, double_integrator_QP_solver_ccrhsl14, double_integrator_QP_solver_slb14, double_integrator_QP_solver_lbIdx14, double_integrator_QP_solver_rd14);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A15, double_integrator_QP_solver_ccrhsp14, double_integrator_QP_solver_sp14, double_integrator_QP_solver_rd14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd14, "double_integrator_QP_solver_rd14");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi14, double_integrator_QP_solver_rd14, double_integrator_QP_solver_Lbyrd14);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_V13, double_integrator_QP_solver_Lbyrd13, double_integrator_QP_solver_W14, double_integrator_QP_solver_Lbyrd14, double_integrator_QP_solver_beta14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta14, "double_integrator_QP_solver_beta14");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd14, double_integrator_QP_solver_yy13, double_integrator_QP_solver_beta14, double_integrator_QP_solver_bmy14);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld14, double_integrator_QP_solver_bmy14, double_integrator_QP_solver_yy14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy14, "double_integrator_QP_solver_yy14");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub15, double_integrator_QP_solver_sub15, double_integrator_QP_solver_ubIdx15, double_integrator_QP_solver_ccrhsl15, double_integrator_QP_solver_slb15, double_integrator_QP_solver_lbIdx15, double_integrator_QP_solver_rd15);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A16, double_integrator_QP_solver_ccrhsp15, double_integrator_QP_solver_sp15, double_integrator_QP_solver_rd15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd15, "double_integrator_QP_solver_rd15");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi15, double_integrator_QP_solver_rd15, double_integrator_QP_solver_Lbyrd15);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_V14, double_integrator_QP_solver_Lbyrd14, double_integrator_QP_solver_W15, double_integrator_QP_solver_Lbyrd15, double_integrator_QP_solver_beta15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta15, "double_integrator_QP_solver_beta15");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd15, double_integrator_QP_solver_yy14, double_integrator_QP_solver_beta15, double_integrator_QP_solver_bmy15);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld15, double_integrator_QP_solver_bmy15, double_integrator_QP_solver_yy15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy15, "double_integrator_QP_solver_yy15");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub16, double_integrator_QP_solver_sub16, double_integrator_QP_solver_ubIdx16, double_integrator_QP_solver_ccrhsl16, double_integrator_QP_solver_slb16, double_integrator_QP_solver_lbIdx16, double_integrator_QP_solver_rd16);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A17, double_integrator_QP_solver_ccrhsp16, double_integrator_QP_solver_sp16, double_integrator_QP_solver_rd16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd16, "double_integrator_QP_solver_rd16");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi16, double_integrator_QP_solver_rd16, double_integrator_QP_solver_Lbyrd16);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_V15, double_integrator_QP_solver_Lbyrd15, double_integrator_QP_solver_W16, double_integrator_QP_solver_Lbyrd16, double_integrator_QP_solver_beta16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta16, "double_integrator_QP_solver_beta16");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd16, double_integrator_QP_solver_yy15, double_integrator_QP_solver_beta16, double_integrator_QP_solver_bmy16);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld16, double_integrator_QP_solver_bmy16, double_integrator_QP_solver_yy16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy16, "double_integrator_QP_solver_yy16");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub17, double_integrator_QP_solver_sub17, double_integrator_QP_solver_ubIdx17, double_integrator_QP_solver_ccrhsl17, double_integrator_QP_solver_slb17, double_integrator_QP_solver_lbIdx17, double_integrator_QP_solver_rd17);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A18, double_integrator_QP_solver_ccrhsp17, double_integrator_QP_solver_sp17, double_integrator_QP_solver_rd17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd17, "double_integrator_QP_solver_rd17");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi17, double_integrator_QP_solver_rd17, double_integrator_QP_solver_Lbyrd17);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_V16, double_integrator_QP_solver_Lbyrd16, double_integrator_QP_solver_W17, double_integrator_QP_solver_Lbyrd17, double_integrator_QP_solver_beta17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta17, "double_integrator_QP_solver_beta17");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd17, double_integrator_QP_solver_yy16, double_integrator_QP_solver_beta17, double_integrator_QP_solver_bmy17);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld17, double_integrator_QP_solver_bmy17, double_integrator_QP_solver_yy17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy17, "double_integrator_QP_solver_yy17");
double_integrator_QP_solver_LA_VSUB6_INDEXED_12_6_7(double_integrator_QP_solver_ccrhsub18, double_integrator_QP_solver_sub18, double_integrator_QP_solver_ubIdx18, double_integrator_QP_solver_ccrhsl18, double_integrator_QP_solver_slb18, double_integrator_QP_solver_lbIdx18, double_integrator_QP_solver_rd18);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_24_12(params->A19, double_integrator_QP_solver_ccrhsp18, double_integrator_QP_solver_sp18, double_integrator_QP_solver_rd18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_rd18, "double_integrator_QP_solver_rd18");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_12(double_integrator_QP_solver_Phi18, double_integrator_QP_solver_rd18, double_integrator_QP_solver_Lbyrd18);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_12(double_integrator_QP_solver_V17, double_integrator_QP_solver_Lbyrd17, double_integrator_QP_solver_W18, double_integrator_QP_solver_Lbyrd18, double_integrator_QP_solver_beta18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_beta18, "double_integrator_QP_solver_beta18");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_1_1(double_integrator_QP_solver_Lsd18, double_integrator_QP_solver_yy17, double_integrator_QP_solver_beta18, double_integrator_QP_solver_bmy18);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_1(double_integrator_QP_solver_Ld18, double_integrator_QP_solver_bmy18, double_integrator_QP_solver_yy18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_yy18, "double_integrator_QP_solver_yy18");
double_integrator_QP_solver_LA_VSUB6_INDEXED_5_4_4(double_integrator_QP_solver_ccrhsub19, double_integrator_QP_solver_sub19, double_integrator_QP_solver_ubIdx19, double_integrator_QP_solver_ccrhsl19, double_integrator_QP_solver_slb19, double_integrator_QP_solver_lbIdx19, double_integrator_QP_solver_rd19);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_10_5(params->A20, double_integrator_QP_solver_ccrhsp19, double_integrator_QP_solver_sp19, double_integrator_QP_solver_rd19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_rd19, "double_integrator_QP_solver_rd19");
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Phi19, double_integrator_QP_solver_rd19, double_integrator_QP_solver_Lbyrd19);
double_integrator_QP_solver_LA_DENSE_2MVMADD_1_12_5(double_integrator_QP_solver_V18, double_integrator_QP_solver_Lbyrd18, double_integrator_QP_solver_W19, double_integrator_QP_solver_Lbyrd19, double_integrator_QP_solver_beta19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_beta19, "double_integrator_QP_solver_beta19");
double_integrator_QP_solver_LA_DENSE_MVMSUB1_5_1(double_integrator_QP_solver_Lsd19, double_integrator_QP_solver_yy18, double_integrator_QP_solver_beta19, double_integrator_QP_solver_bmy19);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_5(double_integrator_QP_solver_Ld19, double_integrator_QP_solver_bmy19, double_integrator_QP_solver_yy19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_yy19, "double_integrator_QP_solver_yy19");
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_5(double_integrator_QP_solver_Ld19, double_integrator_QP_solver_yy19, double_integrator_QP_solver_dvcc19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_dvcc19, "double_integrator_QP_solver_dvcc19");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_5_1(double_integrator_QP_solver_Lsd19, double_integrator_QP_solver_dvcc19, double_integrator_QP_solver_yy18, double_integrator_QP_solver_bmy18);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld18, double_integrator_QP_solver_bmy18, double_integrator_QP_solver_dvcc18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc18, "double_integrator_QP_solver_dvcc18");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd18, double_integrator_QP_solver_dvcc18, double_integrator_QP_solver_yy17, double_integrator_QP_solver_bmy17);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld17, double_integrator_QP_solver_bmy17, double_integrator_QP_solver_dvcc17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc17, "double_integrator_QP_solver_dvcc17");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd17, double_integrator_QP_solver_dvcc17, double_integrator_QP_solver_yy16, double_integrator_QP_solver_bmy16);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld16, double_integrator_QP_solver_bmy16, double_integrator_QP_solver_dvcc16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc16, "double_integrator_QP_solver_dvcc16");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd16, double_integrator_QP_solver_dvcc16, double_integrator_QP_solver_yy15, double_integrator_QP_solver_bmy15);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld15, double_integrator_QP_solver_bmy15, double_integrator_QP_solver_dvcc15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc15, "double_integrator_QP_solver_dvcc15");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd15, double_integrator_QP_solver_dvcc15, double_integrator_QP_solver_yy14, double_integrator_QP_solver_bmy14);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld14, double_integrator_QP_solver_bmy14, double_integrator_QP_solver_dvcc14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc14, "double_integrator_QP_solver_dvcc14");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd14, double_integrator_QP_solver_dvcc14, double_integrator_QP_solver_yy13, double_integrator_QP_solver_bmy13);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld13, double_integrator_QP_solver_bmy13, double_integrator_QP_solver_dvcc13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc13, "double_integrator_QP_solver_dvcc13");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd13, double_integrator_QP_solver_dvcc13, double_integrator_QP_solver_yy12, double_integrator_QP_solver_bmy12);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld12, double_integrator_QP_solver_bmy12, double_integrator_QP_solver_dvcc12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc12, "double_integrator_QP_solver_dvcc12");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd12, double_integrator_QP_solver_dvcc12, double_integrator_QP_solver_yy11, double_integrator_QP_solver_bmy11);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld11, double_integrator_QP_solver_bmy11, double_integrator_QP_solver_dvcc11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc11, "double_integrator_QP_solver_dvcc11");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd11, double_integrator_QP_solver_dvcc11, double_integrator_QP_solver_yy10, double_integrator_QP_solver_bmy10);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_bmy10, double_integrator_QP_solver_dvcc10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc10, "double_integrator_QP_solver_dvcc10");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_dvcc10, double_integrator_QP_solver_yy09, double_integrator_QP_solver_bmy09);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld09, double_integrator_QP_solver_bmy09, double_integrator_QP_solver_dvcc09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc09, "double_integrator_QP_solver_dvcc09");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd09, double_integrator_QP_solver_dvcc09, double_integrator_QP_solver_yy08, double_integrator_QP_solver_bmy08);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld08, double_integrator_QP_solver_bmy08, double_integrator_QP_solver_dvcc08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc08, "double_integrator_QP_solver_dvcc08");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd08, double_integrator_QP_solver_dvcc08, double_integrator_QP_solver_yy07, double_integrator_QP_solver_bmy07);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld07, double_integrator_QP_solver_bmy07, double_integrator_QP_solver_dvcc07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc07, "double_integrator_QP_solver_dvcc07");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd07, double_integrator_QP_solver_dvcc07, double_integrator_QP_solver_yy06, double_integrator_QP_solver_bmy06);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld06, double_integrator_QP_solver_bmy06, double_integrator_QP_solver_dvcc06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc06, "double_integrator_QP_solver_dvcc06");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd06, double_integrator_QP_solver_dvcc06, double_integrator_QP_solver_yy05, double_integrator_QP_solver_bmy05);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld05, double_integrator_QP_solver_bmy05, double_integrator_QP_solver_dvcc05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc05, "double_integrator_QP_solver_dvcc05");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd05, double_integrator_QP_solver_dvcc05, double_integrator_QP_solver_yy04, double_integrator_QP_solver_bmy04);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld04, double_integrator_QP_solver_bmy04, double_integrator_QP_solver_dvcc04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc04, "double_integrator_QP_solver_dvcc04");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd04, double_integrator_QP_solver_dvcc04, double_integrator_QP_solver_yy03, double_integrator_QP_solver_bmy03);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld03, double_integrator_QP_solver_bmy03, double_integrator_QP_solver_dvcc03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc03, "double_integrator_QP_solver_dvcc03");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd03, double_integrator_QP_solver_dvcc03, double_integrator_QP_solver_yy02, double_integrator_QP_solver_bmy02);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld02, double_integrator_QP_solver_bmy02, double_integrator_QP_solver_dvcc02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc02, "double_integrator_QP_solver_dvcc02");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_1(double_integrator_QP_solver_Lsd02, double_integrator_QP_solver_dvcc02, double_integrator_QP_solver_yy01, double_integrator_QP_solver_bmy01);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_1(double_integrator_QP_solver_Ld01, double_integrator_QP_solver_bmy01, double_integrator_QP_solver_dvcc01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_1(double_integrator_QP_solver_dvcc01, "double_integrator_QP_solver_dvcc01");
double_integrator_QP_solver_LA_DENSE_MTVMSUB_1_12(double_integrator_QP_solver_Lsd01, double_integrator_QP_solver_dvcc01, double_integrator_QP_solver_yy00, double_integrator_QP_solver_bmy00);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_12(double_integrator_QP_solver_Ld00, double_integrator_QP_solver_bmy00, double_integrator_QP_solver_dvcc00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dvcc00, "double_integrator_QP_solver_dvcc00");
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MTVM2_1_12_12(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc01, double_integrator_QP_solver_D00, double_integrator_QP_solver_dvcc00, double_integrator_QP_solver_grad_eq00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq00, "double_integrator_QP_solver_grad_eq00");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc02, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc01, double_integrator_QP_solver_grad_eq01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq01, "double_integrator_QP_solver_grad_eq01");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc03, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc02, double_integrator_QP_solver_grad_eq02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq02, "double_integrator_QP_solver_grad_eq02");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc04, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc03, double_integrator_QP_solver_grad_eq03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq03, "double_integrator_QP_solver_grad_eq03");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc05, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc04, double_integrator_QP_solver_grad_eq04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq04, "double_integrator_QP_solver_grad_eq04");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc06, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc05, double_integrator_QP_solver_grad_eq05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq05, "double_integrator_QP_solver_grad_eq05");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc07, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc06, double_integrator_QP_solver_grad_eq06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq06, "double_integrator_QP_solver_grad_eq06");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc08, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc07, double_integrator_QP_solver_grad_eq07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq07, "double_integrator_QP_solver_grad_eq07");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc09, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc08, double_integrator_QP_solver_grad_eq08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq08, "double_integrator_QP_solver_grad_eq08");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc10, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc09, double_integrator_QP_solver_grad_eq09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq09, "double_integrator_QP_solver_grad_eq09");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc11, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc10, double_integrator_QP_solver_grad_eq10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq10, "double_integrator_QP_solver_grad_eq10");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc12, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc11, double_integrator_QP_solver_grad_eq11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq11, "double_integrator_QP_solver_grad_eq11");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc13, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc12, double_integrator_QP_solver_grad_eq12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq12, "double_integrator_QP_solver_grad_eq12");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc14, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc13, double_integrator_QP_solver_grad_eq13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq13, "double_integrator_QP_solver_grad_eq13");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc15, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc14, double_integrator_QP_solver_grad_eq14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq14, "double_integrator_QP_solver_grad_eq14");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc16, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc15, double_integrator_QP_solver_grad_eq15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq15, "double_integrator_QP_solver_grad_eq15");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc17, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc16, double_integrator_QP_solver_grad_eq16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq16, "double_integrator_QP_solver_grad_eq16");
double_integrator_QP_solver_LA_DENSE_MTVM2_1_12_1(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc18, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc17, double_integrator_QP_solver_grad_eq17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq17, "double_integrator_QP_solver_grad_eq17");
double_integrator_QP_solver_LA_DENSE_MTVM2_5_12_1(double_integrator_QP_solver_C18, double_integrator_QP_solver_dvcc19, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc18, double_integrator_QP_solver_grad_eq18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_grad_eq18, "double_integrator_QP_solver_grad_eq18");
double_integrator_QP_solver_LA_DIAGZERO_MTVM_5_5(double_integrator_QP_solver_D19, double_integrator_QP_solver_dvcc19, double_integrator_QP_solver_grad_eq19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_grad_eq19, "double_integrator_QP_solver_grad_eq19");
double_integrator_QP_solver_LA_VSUB_233(double_integrator_QP_solver_rd, double_integrator_QP_solver_grad_eq, double_integrator_QP_solver_rd);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_rd00, double_integrator_QP_solver_dzcc00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc00, "double_integrator_QP_solver_dzcc00");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_rd01, double_integrator_QP_solver_dzcc01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc01, "double_integrator_QP_solver_dzcc01");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_rd02, double_integrator_QP_solver_dzcc02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc02, "double_integrator_QP_solver_dzcc02");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_rd03, double_integrator_QP_solver_dzcc03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc03, "double_integrator_QP_solver_dzcc03");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_rd04, double_integrator_QP_solver_dzcc04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc04, "double_integrator_QP_solver_dzcc04");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_rd05, double_integrator_QP_solver_dzcc05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc05, "double_integrator_QP_solver_dzcc05");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_rd06, double_integrator_QP_solver_dzcc06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc06, "double_integrator_QP_solver_dzcc06");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_rd07, double_integrator_QP_solver_dzcc07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc07, "double_integrator_QP_solver_dzcc07");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_rd08, double_integrator_QP_solver_dzcc08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc08, "double_integrator_QP_solver_dzcc08");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_rd09, double_integrator_QP_solver_dzcc09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc09, "double_integrator_QP_solver_dzcc09");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_rd10, double_integrator_QP_solver_dzcc10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc10, "double_integrator_QP_solver_dzcc10");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_rd11, double_integrator_QP_solver_dzcc11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc11, "double_integrator_QP_solver_dzcc11");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi12, double_integrator_QP_solver_rd12, double_integrator_QP_solver_dzcc12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc12, "double_integrator_QP_solver_dzcc12");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi13, double_integrator_QP_solver_rd13, double_integrator_QP_solver_dzcc13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc13, "double_integrator_QP_solver_dzcc13");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi14, double_integrator_QP_solver_rd14, double_integrator_QP_solver_dzcc14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc14, "double_integrator_QP_solver_dzcc14");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi15, double_integrator_QP_solver_rd15, double_integrator_QP_solver_dzcc15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc15, "double_integrator_QP_solver_dzcc15");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi16, double_integrator_QP_solver_rd16, double_integrator_QP_solver_dzcc16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc16, "double_integrator_QP_solver_dzcc16");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi17, double_integrator_QP_solver_rd17, double_integrator_QP_solver_dzcc17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc17, "double_integrator_QP_solver_dzcc17");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_12(double_integrator_QP_solver_Phi18, double_integrator_QP_solver_rd18, double_integrator_QP_solver_dzcc18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_12(double_integrator_QP_solver_dzcc18, "double_integrator_QP_solver_dzcc18");
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_5(double_integrator_QP_solver_Phi19, double_integrator_QP_solver_rd19, double_integrator_QP_solver_dzcc19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_5(double_integrator_QP_solver_dzcc19, "double_integrator_QP_solver_dzcc19");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl00, double_integrator_QP_solver_slb00, double_integrator_QP_solver_llbbyslb00, double_integrator_QP_solver_dzcc00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_dllbcc00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc00, "double_integrator_QP_solver_dllbcc00");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub00, double_integrator_QP_solver_sub00, double_integrator_QP_solver_lubbysub00, double_integrator_QP_solver_dzcc00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_dlubcc00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc00, "double_integrator_QP_solver_dlubcc00");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A1, double_integrator_QP_solver_dzcc00, double_integrator_QP_solver_ccrhsp00, double_integrator_QP_solver_sp00, double_integrator_QP_solver_lp00, double_integrator_QP_solver_dlp_cc00);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc00, "double_integrator_QP_solver_dlp_cc00");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl01, double_integrator_QP_solver_slb01, double_integrator_QP_solver_llbbyslb01, double_integrator_QP_solver_dzcc01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_dllbcc01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc01, "double_integrator_QP_solver_dllbcc01");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub01, double_integrator_QP_solver_sub01, double_integrator_QP_solver_lubbysub01, double_integrator_QP_solver_dzcc01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_dlubcc01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc01, "double_integrator_QP_solver_dlubcc01");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A2, double_integrator_QP_solver_dzcc01, double_integrator_QP_solver_ccrhsp01, double_integrator_QP_solver_sp01, double_integrator_QP_solver_lp01, double_integrator_QP_solver_dlp_cc01);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc01, "double_integrator_QP_solver_dlp_cc01");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl02, double_integrator_QP_solver_slb02, double_integrator_QP_solver_llbbyslb02, double_integrator_QP_solver_dzcc02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_dllbcc02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc02, "double_integrator_QP_solver_dllbcc02");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub02, double_integrator_QP_solver_sub02, double_integrator_QP_solver_lubbysub02, double_integrator_QP_solver_dzcc02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_dlubcc02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc02, "double_integrator_QP_solver_dlubcc02");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A3, double_integrator_QP_solver_dzcc02, double_integrator_QP_solver_ccrhsp02, double_integrator_QP_solver_sp02, double_integrator_QP_solver_lp02, double_integrator_QP_solver_dlp_cc02);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc02, "double_integrator_QP_solver_dlp_cc02");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl03, double_integrator_QP_solver_slb03, double_integrator_QP_solver_llbbyslb03, double_integrator_QP_solver_dzcc03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_dllbcc03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc03, "double_integrator_QP_solver_dllbcc03");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub03, double_integrator_QP_solver_sub03, double_integrator_QP_solver_lubbysub03, double_integrator_QP_solver_dzcc03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_dlubcc03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc03, "double_integrator_QP_solver_dlubcc03");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A4, double_integrator_QP_solver_dzcc03, double_integrator_QP_solver_ccrhsp03, double_integrator_QP_solver_sp03, double_integrator_QP_solver_lp03, double_integrator_QP_solver_dlp_cc03);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc03, "double_integrator_QP_solver_dlp_cc03");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl04, double_integrator_QP_solver_slb04, double_integrator_QP_solver_llbbyslb04, double_integrator_QP_solver_dzcc04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_dllbcc04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc04, "double_integrator_QP_solver_dllbcc04");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub04, double_integrator_QP_solver_sub04, double_integrator_QP_solver_lubbysub04, double_integrator_QP_solver_dzcc04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_dlubcc04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc04, "double_integrator_QP_solver_dlubcc04");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A5, double_integrator_QP_solver_dzcc04, double_integrator_QP_solver_ccrhsp04, double_integrator_QP_solver_sp04, double_integrator_QP_solver_lp04, double_integrator_QP_solver_dlp_cc04);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc04, "double_integrator_QP_solver_dlp_cc04");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl05, double_integrator_QP_solver_slb05, double_integrator_QP_solver_llbbyslb05, double_integrator_QP_solver_dzcc05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_dllbcc05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc05, "double_integrator_QP_solver_dllbcc05");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub05, double_integrator_QP_solver_sub05, double_integrator_QP_solver_lubbysub05, double_integrator_QP_solver_dzcc05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_dlubcc05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc05, "double_integrator_QP_solver_dlubcc05");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A6, double_integrator_QP_solver_dzcc05, double_integrator_QP_solver_ccrhsp05, double_integrator_QP_solver_sp05, double_integrator_QP_solver_lp05, double_integrator_QP_solver_dlp_cc05);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc05, "double_integrator_QP_solver_dlp_cc05");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl06, double_integrator_QP_solver_slb06, double_integrator_QP_solver_llbbyslb06, double_integrator_QP_solver_dzcc06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_dllbcc06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc06, "double_integrator_QP_solver_dllbcc06");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub06, double_integrator_QP_solver_sub06, double_integrator_QP_solver_lubbysub06, double_integrator_QP_solver_dzcc06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_dlubcc06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc06, "double_integrator_QP_solver_dlubcc06");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A7, double_integrator_QP_solver_dzcc06, double_integrator_QP_solver_ccrhsp06, double_integrator_QP_solver_sp06, double_integrator_QP_solver_lp06, double_integrator_QP_solver_dlp_cc06);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc06, "double_integrator_QP_solver_dlp_cc06");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl07, double_integrator_QP_solver_slb07, double_integrator_QP_solver_llbbyslb07, double_integrator_QP_solver_dzcc07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_dllbcc07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc07, "double_integrator_QP_solver_dllbcc07");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub07, double_integrator_QP_solver_sub07, double_integrator_QP_solver_lubbysub07, double_integrator_QP_solver_dzcc07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_dlubcc07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc07, "double_integrator_QP_solver_dlubcc07");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A8, double_integrator_QP_solver_dzcc07, double_integrator_QP_solver_ccrhsp07, double_integrator_QP_solver_sp07, double_integrator_QP_solver_lp07, double_integrator_QP_solver_dlp_cc07);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc07, "double_integrator_QP_solver_dlp_cc07");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl08, double_integrator_QP_solver_slb08, double_integrator_QP_solver_llbbyslb08, double_integrator_QP_solver_dzcc08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_dllbcc08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc08, "double_integrator_QP_solver_dllbcc08");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub08, double_integrator_QP_solver_sub08, double_integrator_QP_solver_lubbysub08, double_integrator_QP_solver_dzcc08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_dlubcc08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc08, "double_integrator_QP_solver_dlubcc08");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A9, double_integrator_QP_solver_dzcc08, double_integrator_QP_solver_ccrhsp08, double_integrator_QP_solver_sp08, double_integrator_QP_solver_lp08, double_integrator_QP_solver_dlp_cc08);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc08, "double_integrator_QP_solver_dlp_cc08");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl09, double_integrator_QP_solver_slb09, double_integrator_QP_solver_llbbyslb09, double_integrator_QP_solver_dzcc09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_dllbcc09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc09, "double_integrator_QP_solver_dllbcc09");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub09, double_integrator_QP_solver_sub09, double_integrator_QP_solver_lubbysub09, double_integrator_QP_solver_dzcc09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_dlubcc09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc09, "double_integrator_QP_solver_dlubcc09");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A10, double_integrator_QP_solver_dzcc09, double_integrator_QP_solver_ccrhsp09, double_integrator_QP_solver_sp09, double_integrator_QP_solver_lp09, double_integrator_QP_solver_dlp_cc09);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc09, "double_integrator_QP_solver_dlp_cc09");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl10, double_integrator_QP_solver_slb10, double_integrator_QP_solver_llbbyslb10, double_integrator_QP_solver_dzcc10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_dllbcc10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc10, "double_integrator_QP_solver_dllbcc10");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub10, double_integrator_QP_solver_sub10, double_integrator_QP_solver_lubbysub10, double_integrator_QP_solver_dzcc10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_dlubcc10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc10, "double_integrator_QP_solver_dlubcc10");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A11, double_integrator_QP_solver_dzcc10, double_integrator_QP_solver_ccrhsp10, double_integrator_QP_solver_sp10, double_integrator_QP_solver_lp10, double_integrator_QP_solver_dlp_cc10);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc10, "double_integrator_QP_solver_dlp_cc10");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl11, double_integrator_QP_solver_slb11, double_integrator_QP_solver_llbbyslb11, double_integrator_QP_solver_dzcc11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_dllbcc11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc11, "double_integrator_QP_solver_dllbcc11");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub11, double_integrator_QP_solver_sub11, double_integrator_QP_solver_lubbysub11, double_integrator_QP_solver_dzcc11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_dlubcc11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc11, "double_integrator_QP_solver_dlubcc11");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A12, double_integrator_QP_solver_dzcc11, double_integrator_QP_solver_ccrhsp11, double_integrator_QP_solver_sp11, double_integrator_QP_solver_lp11, double_integrator_QP_solver_dlp_cc11);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc11, "double_integrator_QP_solver_dlp_cc11");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl12, double_integrator_QP_solver_slb12, double_integrator_QP_solver_llbbyslb12, double_integrator_QP_solver_dzcc12, double_integrator_QP_solver_lbIdx12, double_integrator_QP_solver_dllbcc12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc12, "double_integrator_QP_solver_dllbcc12");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub12, double_integrator_QP_solver_sub12, double_integrator_QP_solver_lubbysub12, double_integrator_QP_solver_dzcc12, double_integrator_QP_solver_ubIdx12, double_integrator_QP_solver_dlubcc12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc12, "double_integrator_QP_solver_dlubcc12");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A13, double_integrator_QP_solver_dzcc12, double_integrator_QP_solver_ccrhsp12, double_integrator_QP_solver_sp12, double_integrator_QP_solver_lp12, double_integrator_QP_solver_dlp_cc12);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc12, "double_integrator_QP_solver_dlp_cc12");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl13, double_integrator_QP_solver_slb13, double_integrator_QP_solver_llbbyslb13, double_integrator_QP_solver_dzcc13, double_integrator_QP_solver_lbIdx13, double_integrator_QP_solver_dllbcc13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc13, "double_integrator_QP_solver_dllbcc13");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub13, double_integrator_QP_solver_sub13, double_integrator_QP_solver_lubbysub13, double_integrator_QP_solver_dzcc13, double_integrator_QP_solver_ubIdx13, double_integrator_QP_solver_dlubcc13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc13, "double_integrator_QP_solver_dlubcc13");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A14, double_integrator_QP_solver_dzcc13, double_integrator_QP_solver_ccrhsp13, double_integrator_QP_solver_sp13, double_integrator_QP_solver_lp13, double_integrator_QP_solver_dlp_cc13);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc13, "double_integrator_QP_solver_dlp_cc13");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl14, double_integrator_QP_solver_slb14, double_integrator_QP_solver_llbbyslb14, double_integrator_QP_solver_dzcc14, double_integrator_QP_solver_lbIdx14, double_integrator_QP_solver_dllbcc14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc14, "double_integrator_QP_solver_dllbcc14");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub14, double_integrator_QP_solver_sub14, double_integrator_QP_solver_lubbysub14, double_integrator_QP_solver_dzcc14, double_integrator_QP_solver_ubIdx14, double_integrator_QP_solver_dlubcc14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc14, "double_integrator_QP_solver_dlubcc14");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A15, double_integrator_QP_solver_dzcc14, double_integrator_QP_solver_ccrhsp14, double_integrator_QP_solver_sp14, double_integrator_QP_solver_lp14, double_integrator_QP_solver_dlp_cc14);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc14, "double_integrator_QP_solver_dlp_cc14");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl15, double_integrator_QP_solver_slb15, double_integrator_QP_solver_llbbyslb15, double_integrator_QP_solver_dzcc15, double_integrator_QP_solver_lbIdx15, double_integrator_QP_solver_dllbcc15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc15, "double_integrator_QP_solver_dllbcc15");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub15, double_integrator_QP_solver_sub15, double_integrator_QP_solver_lubbysub15, double_integrator_QP_solver_dzcc15, double_integrator_QP_solver_ubIdx15, double_integrator_QP_solver_dlubcc15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc15, "double_integrator_QP_solver_dlubcc15");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A16, double_integrator_QP_solver_dzcc15, double_integrator_QP_solver_ccrhsp15, double_integrator_QP_solver_sp15, double_integrator_QP_solver_lp15, double_integrator_QP_solver_dlp_cc15);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc15, "double_integrator_QP_solver_dlp_cc15");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl16, double_integrator_QP_solver_slb16, double_integrator_QP_solver_llbbyslb16, double_integrator_QP_solver_dzcc16, double_integrator_QP_solver_lbIdx16, double_integrator_QP_solver_dllbcc16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc16, "double_integrator_QP_solver_dllbcc16");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub16, double_integrator_QP_solver_sub16, double_integrator_QP_solver_lubbysub16, double_integrator_QP_solver_dzcc16, double_integrator_QP_solver_ubIdx16, double_integrator_QP_solver_dlubcc16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc16, "double_integrator_QP_solver_dlubcc16");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A17, double_integrator_QP_solver_dzcc16, double_integrator_QP_solver_ccrhsp16, double_integrator_QP_solver_sp16, double_integrator_QP_solver_lp16, double_integrator_QP_solver_dlp_cc16);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc16, "double_integrator_QP_solver_dlp_cc16");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl17, double_integrator_QP_solver_slb17, double_integrator_QP_solver_llbbyslb17, double_integrator_QP_solver_dzcc17, double_integrator_QP_solver_lbIdx17, double_integrator_QP_solver_dllbcc17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc17, "double_integrator_QP_solver_dllbcc17");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub17, double_integrator_QP_solver_sub17, double_integrator_QP_solver_lubbysub17, double_integrator_QP_solver_dzcc17, double_integrator_QP_solver_ubIdx17, double_integrator_QP_solver_dlubcc17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc17, "double_integrator_QP_solver_dlubcc17");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A18, double_integrator_QP_solver_dzcc17, double_integrator_QP_solver_ccrhsp17, double_integrator_QP_solver_sp17, double_integrator_QP_solver_lp17, double_integrator_QP_solver_dlp_cc17);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc17, "double_integrator_QP_solver_dlp_cc17");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl18, double_integrator_QP_solver_slb18, double_integrator_QP_solver_llbbyslb18, double_integrator_QP_solver_dzcc18, double_integrator_QP_solver_lbIdx18, double_integrator_QP_solver_dllbcc18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dllbcc18, "double_integrator_QP_solver_dllbcc18");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub18, double_integrator_QP_solver_sub18, double_integrator_QP_solver_lubbysub18, double_integrator_QP_solver_dzcc18, double_integrator_QP_solver_ubIdx18, double_integrator_QP_solver_dlubcc18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dlubcc18, "double_integrator_QP_solver_dlubcc18");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_24_12(params->A19, double_integrator_QP_solver_dzcc18, double_integrator_QP_solver_ccrhsp18, double_integrator_QP_solver_sp18, double_integrator_QP_solver_lp18, double_integrator_QP_solver_dlp_cc18);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dlp_cc18, "double_integrator_QP_solver_dlp_cc18");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_ccrhsl19, double_integrator_QP_solver_slb19, double_integrator_QP_solver_llbbyslb19, double_integrator_QP_solver_dzcc19, double_integrator_QP_solver_lbIdx19, double_integrator_QP_solver_dllbcc19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_4(double_integrator_QP_solver_dllbcc19, "double_integrator_QP_solver_dllbcc19");
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_ccrhsub19, double_integrator_QP_solver_sub19, double_integrator_QP_solver_lubbysub19, double_integrator_QP_solver_dzcc19, double_integrator_QP_solver_ubIdx19, double_integrator_QP_solver_dlubcc19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_4(double_integrator_QP_solver_dlubcc19, "double_integrator_QP_solver_dlubcc19");
double_integrator_QP_solver_LA_DENSE_MVMSUB5_10_5(params->A20, double_integrator_QP_solver_dzcc19, double_integrator_QP_solver_ccrhsp19, double_integrator_QP_solver_sp19, double_integrator_QP_solver_lp19, double_integrator_QP_solver_dlp_cc19);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_10(double_integrator_QP_solver_dlp_cc19, "double_integrator_QP_solver_dlp_cc19");
double_integrator_QP_solver_LA_VSUB7_721(double_integrator_QP_solver_l, double_integrator_QP_solver_ccrhs, double_integrator_QP_solver_s, double_integrator_QP_solver_dl_cc, double_integrator_QP_solver_ds_cc);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc00, "double_integrator_QP_solver_dslbcc00");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc00, "double_integrator_QP_solver_dsubcc00");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc00, "double_integrator_QP_solver_dsp_cc00");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc01, "double_integrator_QP_solver_dslbcc01");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc01, "double_integrator_QP_solver_dsubcc01");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc01, "double_integrator_QP_solver_dsp_cc01");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc02, "double_integrator_QP_solver_dslbcc02");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc02, "double_integrator_QP_solver_dsubcc02");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc02, "double_integrator_QP_solver_dsp_cc02");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc03, "double_integrator_QP_solver_dslbcc03");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc03, "double_integrator_QP_solver_dsubcc03");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc03, "double_integrator_QP_solver_dsp_cc03");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc04, "double_integrator_QP_solver_dslbcc04");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc04, "double_integrator_QP_solver_dsubcc04");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc04, "double_integrator_QP_solver_dsp_cc04");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc05, "double_integrator_QP_solver_dslbcc05");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc05, "double_integrator_QP_solver_dsubcc05");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc05, "double_integrator_QP_solver_dsp_cc05");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc06, "double_integrator_QP_solver_dslbcc06");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc06, "double_integrator_QP_solver_dsubcc06");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc06, "double_integrator_QP_solver_dsp_cc06");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc07, "double_integrator_QP_solver_dslbcc07");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc07, "double_integrator_QP_solver_dsubcc07");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc07, "double_integrator_QP_solver_dsp_cc07");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc08, "double_integrator_QP_solver_dslbcc08");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc08, "double_integrator_QP_solver_dsubcc08");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc08, "double_integrator_QP_solver_dsp_cc08");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc09, "double_integrator_QP_solver_dslbcc09");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc09, "double_integrator_QP_solver_dsubcc09");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc09, "double_integrator_QP_solver_dsp_cc09");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc10, "double_integrator_QP_solver_dslbcc10");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc10, "double_integrator_QP_solver_dsubcc10");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc10, "double_integrator_QP_solver_dsp_cc10");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc11, "double_integrator_QP_solver_dslbcc11");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc11, "double_integrator_QP_solver_dsubcc11");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc11, "double_integrator_QP_solver_dsp_cc11");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc12, "double_integrator_QP_solver_dslbcc12");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc12, "double_integrator_QP_solver_dsubcc12");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc12, "double_integrator_QP_solver_dsp_cc12");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc13, "double_integrator_QP_solver_dslbcc13");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc13, "double_integrator_QP_solver_dsubcc13");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc13, "double_integrator_QP_solver_dsp_cc13");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc14, "double_integrator_QP_solver_dslbcc14");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc14, "double_integrator_QP_solver_dsubcc14");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc14, "double_integrator_QP_solver_dsp_cc14");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc15, "double_integrator_QP_solver_dslbcc15");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc15, "double_integrator_QP_solver_dsubcc15");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc15, "double_integrator_QP_solver_dsp_cc15");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc16, "double_integrator_QP_solver_dslbcc16");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc16, "double_integrator_QP_solver_dsubcc16");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc16, "double_integrator_QP_solver_dsp_cc16");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc17, "double_integrator_QP_solver_dslbcc17");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc17, "double_integrator_QP_solver_dsubcc17");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc17, "double_integrator_QP_solver_dsp_cc17");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_7(double_integrator_QP_solver_dslbcc18, "double_integrator_QP_solver_dslbcc18");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_6(double_integrator_QP_solver_dsubcc18, "double_integrator_QP_solver_dsubcc18");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_24(double_integrator_QP_solver_dsp_cc18, "double_integrator_QP_solver_dsp_cc18");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_4(double_integrator_QP_solver_dslbcc19, "double_integrator_QP_solver_dslbcc19");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_4(double_integrator_QP_solver_dsubcc19, "double_integrator_QP_solver_dsubcc19");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_10(double_integrator_QP_solver_dsp_cc19, "double_integrator_QP_solver_dsp_cc19");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_233(double_integrator_QP_solver_dz_aff, "double_integrator_QP_solver_dz_aff");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_35(double_integrator_QP_solver_dv_aff, "double_integrator_QP_solver_dv_aff");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_721(double_integrator_QP_solver_dl_aff, "double_integrator_QP_solver_dl_aff");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_721(double_integrator_QP_solver_ds_aff, "double_integrator_QP_solver_ds_aff");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_233(double_integrator_QP_solver_dz_cc, "double_integrator_QP_solver_dz_cc");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_35(double_integrator_QP_solver_dv_cc, "double_integrator_QP_solver_dv_cc");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_721(double_integrator_QP_solver_dl_cc, "double_integrator_QP_solver_dl_cc");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_721(double_integrator_QP_solver_ds_cc, "double_integrator_QP_solver_ds_cc");
double_integrator_QP_solver_LA_VADD_233(double_integrator_QP_solver_dz_cc, double_integrator_QP_solver_dz_aff);
double_integrator_QP_solver_LA_VADD_35(double_integrator_QP_solver_dv_cc, double_integrator_QP_solver_dv_aff);
double_integrator_QP_solver_LA_VADD_721(double_integrator_QP_solver_dl_cc, double_integrator_QP_solver_dl_aff);
double_integrator_QP_solver_LA_VADD_721(double_integrator_QP_solver_ds_cc, double_integrator_QP_solver_ds_aff);
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_233(double_integrator_QP_solver_z, "double_integrator_QP_solver_z");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_35(double_integrator_QP_solver_v, "double_integrator_QP_solver_v");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_721(double_integrator_QP_solver_l, "double_integrator_QP_solver_l");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_721(double_integrator_QP_solver_s, "double_integrator_QP_solver_s");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_233(double_integrator_QP_solver_dz_cc, "double_integrator_QP_solver_dz_cc");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_35(double_integrator_QP_solver_dv_cc, "double_integrator_QP_solver_dv_cc");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_721(double_integrator_QP_solver_dl_cc, "double_integrator_QP_solver_dl_cc");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_721(double_integrator_QP_solver_ds_cc, "double_integrator_QP_solver_ds_cc");
info->lsit_cc = double_integrator_QP_solver_LINESEARCH_BACKTRACKING_COMBINED(double_integrator_QP_solver_z, double_integrator_QP_solver_v, double_integrator_QP_solver_l, double_integrator_QP_solver_s, double_integrator_QP_solver_dz_cc, double_integrator_QP_solver_dv_cc, double_integrator_QP_solver_dl_cc, double_integrator_QP_solver_ds_cc, &info->step_cc, &info->mu);
if( info->lsit_cc == double_integrator_QP_solver_NOPROGRESS ){
PRINTTEXT("Line search could not proceed at iteration %d, exiting.\n",info->it+1);
exitcode = double_integrator_QP_solver_NOPROGRESS; break;
}
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_233(double_integrator_QP_solver_z, "double_integrator_QP_solver_z");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_35(double_integrator_QP_solver_v, "double_integrator_QP_solver_v");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_721(double_integrator_QP_solver_l, "double_integrator_QP_solver_l");
double_integrator_QP_solver_LA_DENSE_PRINT_VECTOR_721(double_integrator_QP_solver_s, "double_integrator_QP_solver_s");
info->it++;
}
output->z1[0] = double_integrator_QP_solver_z00[0];
output->z1[1] = double_integrator_QP_solver_z00[1];
output->z1[2] = double_integrator_QP_solver_z00[2];
output->z1[3] = double_integrator_QP_solver_z00[3];
output->z1[4] = double_integrator_QP_solver_z00[4];
output->z1[5] = double_integrator_QP_solver_z00[5];
output->z1[6] = double_integrator_QP_solver_z00[6];
output->z1[7] = double_integrator_QP_solver_z00[7];
output->z1[8] = double_integrator_QP_solver_z00[8];
output->z1[9] = double_integrator_QP_solver_z00[9];
output->z1[10] = double_integrator_QP_solver_z00[10];
output->z1[11] = double_integrator_QP_solver_z00[11];
output->z2[0] = double_integrator_QP_solver_z01[0];
output->z2[1] = double_integrator_QP_solver_z01[1];
output->z2[2] = double_integrator_QP_solver_z01[2];
output->z2[3] = double_integrator_QP_solver_z01[3];
output->z2[4] = double_integrator_QP_solver_z01[4];
output->z2[5] = double_integrator_QP_solver_z01[5];
output->z2[6] = double_integrator_QP_solver_z01[6];
output->z2[7] = double_integrator_QP_solver_z01[7];
output->z2[8] = double_integrator_QP_solver_z01[8];
output->z2[9] = double_integrator_QP_solver_z01[9];
output->z2[10] = double_integrator_QP_solver_z01[10];
output->z2[11] = double_integrator_QP_solver_z01[11];
output->z3[0] = double_integrator_QP_solver_z02[0];
output->z3[1] = double_integrator_QP_solver_z02[1];
output->z3[2] = double_integrator_QP_solver_z02[2];
output->z3[3] = double_integrator_QP_solver_z02[3];
output->z3[4] = double_integrator_QP_solver_z02[4];
output->z3[5] = double_integrator_QP_solver_z02[5];
output->z3[6] = double_integrator_QP_solver_z02[6];
output->z3[7] = double_integrator_QP_solver_z02[7];
output->z3[8] = double_integrator_QP_solver_z02[8];
output->z3[9] = double_integrator_QP_solver_z02[9];
output->z3[10] = double_integrator_QP_solver_z02[10];
output->z3[11] = double_integrator_QP_solver_z02[11];
output->z4[0] = double_integrator_QP_solver_z03[0];
output->z4[1] = double_integrator_QP_solver_z03[1];
output->z4[2] = double_integrator_QP_solver_z03[2];
output->z4[3] = double_integrator_QP_solver_z03[3];
output->z4[4] = double_integrator_QP_solver_z03[4];
output->z4[5] = double_integrator_QP_solver_z03[5];
output->z4[6] = double_integrator_QP_solver_z03[6];
output->z4[7] = double_integrator_QP_solver_z03[7];
output->z4[8] = double_integrator_QP_solver_z03[8];
output->z4[9] = double_integrator_QP_solver_z03[9];
output->z4[10] = double_integrator_QP_solver_z03[10];
output->z4[11] = double_integrator_QP_solver_z03[11];
output->z5[0] = double_integrator_QP_solver_z04[0];
output->z5[1] = double_integrator_QP_solver_z04[1];
output->z5[2] = double_integrator_QP_solver_z04[2];
output->z5[3] = double_integrator_QP_solver_z04[3];
output->z5[4] = double_integrator_QP_solver_z04[4];
output->z5[5] = double_integrator_QP_solver_z04[5];
output->z5[6] = double_integrator_QP_solver_z04[6];
output->z5[7] = double_integrator_QP_solver_z04[7];
output->z5[8] = double_integrator_QP_solver_z04[8];
output->z5[9] = double_integrator_QP_solver_z04[9];
output->z5[10] = double_integrator_QP_solver_z04[10];
output->z5[11] = double_integrator_QP_solver_z04[11];
output->z6[0] = double_integrator_QP_solver_z05[0];
output->z6[1] = double_integrator_QP_solver_z05[1];
output->z6[2] = double_integrator_QP_solver_z05[2];
output->z6[3] = double_integrator_QP_solver_z05[3];
output->z6[4] = double_integrator_QP_solver_z05[4];
output->z6[5] = double_integrator_QP_solver_z05[5];
output->z6[6] = double_integrator_QP_solver_z05[6];
output->z6[7] = double_integrator_QP_solver_z05[7];
output->z6[8] = double_integrator_QP_solver_z05[8];
output->z6[9] = double_integrator_QP_solver_z05[9];
output->z6[10] = double_integrator_QP_solver_z05[10];
output->z6[11] = double_integrator_QP_solver_z05[11];
output->z7[0] = double_integrator_QP_solver_z06[0];
output->z7[1] = double_integrator_QP_solver_z06[1];
output->z7[2] = double_integrator_QP_solver_z06[2];
output->z7[3] = double_integrator_QP_solver_z06[3];
output->z7[4] = double_integrator_QP_solver_z06[4];
output->z7[5] = double_integrator_QP_solver_z06[5];
output->z7[6] = double_integrator_QP_solver_z06[6];
output->z7[7] = double_integrator_QP_solver_z06[7];
output->z7[8] = double_integrator_QP_solver_z06[8];
output->z7[9] = double_integrator_QP_solver_z06[9];
output->z7[10] = double_integrator_QP_solver_z06[10];
output->z7[11] = double_integrator_QP_solver_z06[11];
output->z8[0] = double_integrator_QP_solver_z07[0];
output->z8[1] = double_integrator_QP_solver_z07[1];
output->z8[2] = double_integrator_QP_solver_z07[2];
output->z8[3] = double_integrator_QP_solver_z07[3];
output->z8[4] = double_integrator_QP_solver_z07[4];
output->z8[5] = double_integrator_QP_solver_z07[5];
output->z8[6] = double_integrator_QP_solver_z07[6];
output->z8[7] = double_integrator_QP_solver_z07[7];
output->z8[8] = double_integrator_QP_solver_z07[8];
output->z8[9] = double_integrator_QP_solver_z07[9];
output->z8[10] = double_integrator_QP_solver_z07[10];
output->z8[11] = double_integrator_QP_solver_z07[11];
output->z9[0] = double_integrator_QP_solver_z08[0];
output->z9[1] = double_integrator_QP_solver_z08[1];
output->z9[2] = double_integrator_QP_solver_z08[2];
output->z9[3] = double_integrator_QP_solver_z08[3];
output->z9[4] = double_integrator_QP_solver_z08[4];
output->z9[5] = double_integrator_QP_solver_z08[5];
output->z9[6] = double_integrator_QP_solver_z08[6];
output->z9[7] = double_integrator_QP_solver_z08[7];
output->z9[8] = double_integrator_QP_solver_z08[8];
output->z9[9] = double_integrator_QP_solver_z08[9];
output->z9[10] = double_integrator_QP_solver_z08[10];
output->z9[11] = double_integrator_QP_solver_z08[11];
output->z10[0] = double_integrator_QP_solver_z09[0];
output->z10[1] = double_integrator_QP_solver_z09[1];
output->z10[2] = double_integrator_QP_solver_z09[2];
output->z10[3] = double_integrator_QP_solver_z09[3];
output->z10[4] = double_integrator_QP_solver_z09[4];
output->z10[5] = double_integrator_QP_solver_z09[5];
output->z10[6] = double_integrator_QP_solver_z09[6];
output->z10[7] = double_integrator_QP_solver_z09[7];
output->z10[8] = double_integrator_QP_solver_z09[8];
output->z10[9] = double_integrator_QP_solver_z09[9];
output->z10[10] = double_integrator_QP_solver_z09[10];
output->z10[11] = double_integrator_QP_solver_z09[11];
output->z11[0] = double_integrator_QP_solver_z10[0];
output->z11[1] = double_integrator_QP_solver_z10[1];
output->z11[2] = double_integrator_QP_solver_z10[2];
output->z11[3] = double_integrator_QP_solver_z10[3];
output->z11[4] = double_integrator_QP_solver_z10[4];
output->z11[5] = double_integrator_QP_solver_z10[5];
output->z11[6] = double_integrator_QP_solver_z10[6];
output->z11[7] = double_integrator_QP_solver_z10[7];
output->z11[8] = double_integrator_QP_solver_z10[8];
output->z11[9] = double_integrator_QP_solver_z10[9];
output->z11[10] = double_integrator_QP_solver_z10[10];
output->z11[11] = double_integrator_QP_solver_z10[11];
output->z12[0] = double_integrator_QP_solver_z11[0];
output->z12[1] = double_integrator_QP_solver_z11[1];
output->z12[2] = double_integrator_QP_solver_z11[2];
output->z12[3] = double_integrator_QP_solver_z11[3];
output->z12[4] = double_integrator_QP_solver_z11[4];
output->z12[5] = double_integrator_QP_solver_z11[5];
output->z12[6] = double_integrator_QP_solver_z11[6];
output->z12[7] = double_integrator_QP_solver_z11[7];
output->z12[8] = double_integrator_QP_solver_z11[8];
output->z12[9] = double_integrator_QP_solver_z11[9];
output->z12[10] = double_integrator_QP_solver_z11[10];
output->z12[11] = double_integrator_QP_solver_z11[11];
output->z13[0] = double_integrator_QP_solver_z12[0];
output->z13[1] = double_integrator_QP_solver_z12[1];
output->z13[2] = double_integrator_QP_solver_z12[2];
output->z13[3] = double_integrator_QP_solver_z12[3];
output->z13[4] = double_integrator_QP_solver_z12[4];
output->z13[5] = double_integrator_QP_solver_z12[5];
output->z13[6] = double_integrator_QP_solver_z12[6];
output->z13[7] = double_integrator_QP_solver_z12[7];
output->z13[8] = double_integrator_QP_solver_z12[8];
output->z13[9] = double_integrator_QP_solver_z12[9];
output->z13[10] = double_integrator_QP_solver_z12[10];
output->z13[11] = double_integrator_QP_solver_z12[11];
output->z14[0] = double_integrator_QP_solver_z13[0];
output->z14[1] = double_integrator_QP_solver_z13[1];
output->z14[2] = double_integrator_QP_solver_z13[2];
output->z14[3] = double_integrator_QP_solver_z13[3];
output->z14[4] = double_integrator_QP_solver_z13[4];
output->z14[5] = double_integrator_QP_solver_z13[5];
output->z14[6] = double_integrator_QP_solver_z13[6];
output->z14[7] = double_integrator_QP_solver_z13[7];
output->z14[8] = double_integrator_QP_solver_z13[8];
output->z14[9] = double_integrator_QP_solver_z13[9];
output->z14[10] = double_integrator_QP_solver_z13[10];
output->z14[11] = double_integrator_QP_solver_z13[11];
output->z15[0] = double_integrator_QP_solver_z14[0];
output->z15[1] = double_integrator_QP_solver_z14[1];
output->z15[2] = double_integrator_QP_solver_z14[2];
output->z15[3] = double_integrator_QP_solver_z14[3];
output->z15[4] = double_integrator_QP_solver_z14[4];
output->z15[5] = double_integrator_QP_solver_z14[5];
output->z15[6] = double_integrator_QP_solver_z14[6];
output->z15[7] = double_integrator_QP_solver_z14[7];
output->z15[8] = double_integrator_QP_solver_z14[8];
output->z15[9] = double_integrator_QP_solver_z14[9];
output->z15[10] = double_integrator_QP_solver_z14[10];
output->z15[11] = double_integrator_QP_solver_z14[11];
output->z16[0] = double_integrator_QP_solver_z15[0];
output->z16[1] = double_integrator_QP_solver_z15[1];
output->z16[2] = double_integrator_QP_solver_z15[2];
output->z16[3] = double_integrator_QP_solver_z15[3];
output->z16[4] = double_integrator_QP_solver_z15[4];
output->z16[5] = double_integrator_QP_solver_z15[5];
output->z16[6] = double_integrator_QP_solver_z15[6];
output->z16[7] = double_integrator_QP_solver_z15[7];
output->z16[8] = double_integrator_QP_solver_z15[8];
output->z16[9] = double_integrator_QP_solver_z15[9];
output->z16[10] = double_integrator_QP_solver_z15[10];
output->z16[11] = double_integrator_QP_solver_z15[11];
output->z17[0] = double_integrator_QP_solver_z16[0];
output->z17[1] = double_integrator_QP_solver_z16[1];
output->z17[2] = double_integrator_QP_solver_z16[2];
output->z17[3] = double_integrator_QP_solver_z16[3];
output->z17[4] = double_integrator_QP_solver_z16[4];
output->z17[5] = double_integrator_QP_solver_z16[5];
output->z17[6] = double_integrator_QP_solver_z16[6];
output->z17[7] = double_integrator_QP_solver_z16[7];
output->z17[8] = double_integrator_QP_solver_z16[8];
output->z17[9] = double_integrator_QP_solver_z16[9];
output->z17[10] = double_integrator_QP_solver_z16[10];
output->z17[11] = double_integrator_QP_solver_z16[11];
output->z18[0] = double_integrator_QP_solver_z17[0];
output->z18[1] = double_integrator_QP_solver_z17[1];
output->z18[2] = double_integrator_QP_solver_z17[2];
output->z18[3] = double_integrator_QP_solver_z17[3];
output->z18[4] = double_integrator_QP_solver_z17[4];
output->z18[5] = double_integrator_QP_solver_z17[5];
output->z18[6] = double_integrator_QP_solver_z17[6];
output->z18[7] = double_integrator_QP_solver_z17[7];
output->z18[8] = double_integrator_QP_solver_z17[8];
output->z18[9] = double_integrator_QP_solver_z17[9];
output->z18[10] = double_integrator_QP_solver_z17[10];
output->z18[11] = double_integrator_QP_solver_z17[11];
output->z19[0] = double_integrator_QP_solver_z18[0];
output->z19[1] = double_integrator_QP_solver_z18[1];
output->z19[2] = double_integrator_QP_solver_z18[2];
output->z19[3] = double_integrator_QP_solver_z18[3];
output->z19[4] = double_integrator_QP_solver_z18[4];
output->z19[5] = double_integrator_QP_solver_z18[5];
output->z19[6] = double_integrator_QP_solver_z18[6];
output->z19[7] = double_integrator_QP_solver_z18[7];
output->z19[8] = double_integrator_QP_solver_z18[8];
output->z19[9] = double_integrator_QP_solver_z18[9];
output->z19[10] = double_integrator_QP_solver_z18[10];
output->z19[11] = double_integrator_QP_solver_z18[11];
output->z20[0] = double_integrator_QP_solver_z19[0];
output->z20[1] = double_integrator_QP_solver_z19[1];
output->z20[2] = double_integrator_QP_solver_z19[2];
output->z20[3] = double_integrator_QP_solver_z19[3];
output->z20[4] = double_integrator_QP_solver_z19[4];

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
