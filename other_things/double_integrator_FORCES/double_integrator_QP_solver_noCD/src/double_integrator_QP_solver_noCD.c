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

#include "../include/double_integrator_QP_solver_noCD.h"

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

typedef struct double_integrator_QP_solver_noCD_timer{
	LARGE_INTEGER tic;
	LARGE_INTEGER toc;
	LARGE_INTEGER freq;
} double_integrator_QP_solver_noCD_timer;


void double_integrator_QP_solver_noCD_tic(double_integrator_QP_solver_noCD_timer* t)
{
	QueryPerformanceFrequency(&t->freq);
	QueryPerformanceCounter(&t->tic);
}



double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_toc(double_integrator_QP_solver_noCD_timer* t)
{
	QueryPerformanceCounter(&t->toc);
	return ((t->toc.QuadPart - t->tic.QuadPart) / (double_integrator_QP_solver_noCD_FLOAT)t->freq.QuadPart);
}


/* WE ARE ON THE MAC */
#elif (defined __APPLE__)
#include <mach/mach_time.h>


/* Use MAC OSX  mach_time for timing */
typedef struct double_integrator_QP_solver_noCD_timer{
	uint64_t tic;
	uint64_t toc;
	mach_timebase_info_data_t tinfo;

} double_integrator_QP_solver_noCD_timer;


void double_integrator_QP_solver_noCD_tic(double_integrator_QP_solver_noCD_timer* t)
{
    /* read current clock cycles */
    t->tic = mach_absolute_time();
}



double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_toc(double_integrator_QP_solver_noCD_timer* t)
{
    uint64_t duration; /* elapsed time in clock cycles*/
    t->toc = mach_absolute_time();
	duration = t->toc - t->tic;

    /*conversion from clock cycles to nanoseconds*/
    mach_timebase_info(&(t->tinfo));
    duration *= t->tinfo.numer;
    duration /= t->tinfo.denom;

    return (double_integrator_QP_solver_noCD_FLOAT)duration / 1000000000;
}

/* WE ARE ON SOME TEXAS INSTRUMENTS PLATFORM */
#elif (defined __TI_COMPILER_VERSION__)

/* TimeStamps */
#include <c6x.h> /* make use of TSCL, TSCH */


typedef struct double_integrator_QP_solver_noCD_timer{
	unsigned long long tic;
	unsigned long long toc;
} double_integrator_QP_solver_noCD_timer;


void double_integrator_QP_solver_noCD_tic(double_integrator_QP_solver_noCD_timer* t)
{
	TSCL = 0;	/* Initiate CPU timer by writing any val to TSCL */
	t->tic = _itoll( TSCH, TSCL );
}



double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_toc(double_integrator_QP_solver_noCD_timer* t)
{
	t->toc = _itoll( TSCH, TSCL );
	unsigned long long t0;
	unsigned long long overhead;
	t0 = _itoll( TSCH, TSCL );
	overhead = _itoll( TSCH, TSCL )  - t0;

	return (double_integrator_QP_solver_noCD_FLOAT)(t->toc - t->tic - overhead) / 1000000000;
}



/* WE ARE ON SOME OTHER UNIX/LINUX SYSTEM */
#else

/* Use POSIX clocl_gettime() for timing on non-Windows machines */
#include <time.h>
typedef struct double_integrator_QP_solver_noCD_timer{
	struct timespec tic;
	struct timespec toc;
} double_integrator_QP_solver_noCD_timer;


/* read current time */
void double_integrator_QP_solver_noCD_tic(double_integrator_QP_solver_noCD_timer* t)
{
	clock_gettime(CLOCK_MONOTONIC, &t->tic);
}



/* return time passed since last call to tic on this timer */
double double_integrator_QP_solver_noCD_toc(double_integrator_QP_solver_noCD_timer* t)
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

	return (double_integrator_QP_solver_noCD_FLOAT)temp.tv_sec + (double_integrator_QP_solver_noCD_FLOAT)temp.tv_nsec / 1000000000;
}


#endif

/* LINEAR ALGEBRA LIBRARY ---------------------------------------------- */
/*
 * Initializes a vector of length 189 with a value.
 */
void double_integrator_QP_solver_noCD_LA_INITIALIZEVECTOR_189(double_integrator_QP_solver_noCD_FLOAT* vec, double_integrator_QP_solver_noCD_FLOAT value)
{
	int i;
	for( i=0; i<189; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 74 with a value.
 */
void double_integrator_QP_solver_noCD_LA_INITIALIZEVECTOR_74(double_integrator_QP_solver_noCD_FLOAT* vec, double_integrator_QP_solver_noCD_FLOAT value)
{
	int i;
	for( i=0; i<74; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 391 with a value.
 */
void double_integrator_QP_solver_noCD_LA_INITIALIZEVECTOR_391(double_integrator_QP_solver_noCD_FLOAT* vec, double_integrator_QP_solver_noCD_FLOAT value)
{
	int i;
	for( i=0; i<391; i++ )
	{
		vec[i] = value;
	}
}


/* 
 * Calculates a dot product and adds it to a variable: z += x'*y; 
 * This function is for vectors of length 391.
 */
void double_integrator_QP_solver_noCD_LA_DOTACC_391(double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *y, double_integrator_QP_solver_noCD_FLOAT *z)
{
	int i;
	for( i=0; i<391; i++ ){
		*z += x[i]*y[i];
	}
}


/*
 * Calculates the gradient and the value for a quadratic function 0.5*z'*H*z + f'*z
 *
 * INPUTS:     H  - Symmetric Hessian, diag matrix of size [17 x 17]
 *             f  - column vector of size 17
 *             z  - column vector of size 17
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 17
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void double_integrator_QP_solver_noCD_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_noCD_FLOAT* H, double_integrator_QP_solver_noCD_FLOAT* f, double_integrator_QP_solver_noCD_FLOAT* z, double_integrator_QP_solver_noCD_FLOAT* grad, double_integrator_QP_solver_noCD_FLOAT* value)
{
	int i;
	double_integrator_QP_solver_noCD_FLOAT hz;	
	for( i=0; i<17; i++){
		hz = H[i]*z[i];
		grad[i] = hz + f[i];
		*value += 0.5*hz*z[i] + f[i]*z[i];
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
void double_integrator_QP_solver_noCD_LA_DIAG_QUADFCN_15(double_integrator_QP_solver_noCD_FLOAT* H, double_integrator_QP_solver_noCD_FLOAT* f, double_integrator_QP_solver_noCD_FLOAT* z, double_integrator_QP_solver_noCD_FLOAT* grad, double_integrator_QP_solver_noCD_FLOAT* value)
{
	int i;
	double_integrator_QP_solver_noCD_FLOAT hz;	
	for( i=0; i<15; i++){
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
void double_integrator_QP_solver_noCD_LA_DIAG_QUADFCN_4(double_integrator_QP_solver_noCD_FLOAT* H, double_integrator_QP_solver_noCD_FLOAT* f, double_integrator_QP_solver_noCD_FLOAT* z, double_integrator_QP_solver_noCD_FLOAT* grad, double_integrator_QP_solver_noCD_FLOAT* value)
{
	int i;
	double_integrator_QP_solver_noCD_FLOAT hz;	
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
void double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *l, double_integrator_QP_solver_noCD_FLOAT *r, double_integrator_QP_solver_noCD_FLOAT *z, double_integrator_QP_solver_noCD_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;
	double_integrator_QP_solver_noCD_FLOAT AxBu[7];
	double_integrator_QP_solver_noCD_FLOAT norm = *y;
	double_integrator_QP_solver_noCD_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<7; i++ ){
		AxBu[i] = A[k++]*x[0] + B[m++]*u[0];
	}	
	for( j=1; j<17; j++ ){		
		for( i=0; i<7; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<17; n++ ){
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
void double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB3_7_17_15(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *l, double_integrator_QP_solver_noCD_FLOAT *r, double_integrator_QP_solver_noCD_FLOAT *z, double_integrator_QP_solver_noCD_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;
	double_integrator_QP_solver_noCD_FLOAT AxBu[7];
	double_integrator_QP_solver_noCD_FLOAT norm = *y;
	double_integrator_QP_solver_noCD_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<7; i++ ){
		AxBu[i] = A[k++]*x[0] + B[m++]*u[0];
	}	
	for( j=1; j<17; j++ ){		
		for( i=0; i<7; i++ ){
			AxBu[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<15; n++ ){
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
void double_integrator_QP_solver_noCD_LA_DENSE_DIAGZERO_MVMSUB3_4_15_4(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *l, double_integrator_QP_solver_noCD_FLOAT *r, double_integrator_QP_solver_noCD_FLOAT *z, double_integrator_QP_solver_noCD_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	double_integrator_QP_solver_noCD_FLOAT AxBu[4];
	double_integrator_QP_solver_noCD_FLOAT norm = *y;
	double_integrator_QP_solver_noCD_FLOAT lr = 0;

	/* do A*x + B*u first */
	for( i=0; i<4; i++ ){
		AxBu[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<15; j++ ){		
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
 * Matrix vector multiplication y = M'*x where M is of size [7 x 17]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_noCD_LA_DENSE_MTVM_7_17(double_integrator_QP_solver_noCD_FLOAT *M, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *y)
{
	int i;
	int j;
	int k = 0; 
	for( i=0; i<17; i++ ){
		y[i] = 0;
		for( j=0; j<7; j++ ){
			y[i] += M[k++]*x[j];
		}
	}
}


/*
 * Matrix vector multiplication z = A'*x + B'*y 
 * where A is of size [7 x 17]
 * and B is of size [7 x 17]
 * and stored in column major format. Note the transposes of A and B!
 */
void double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *y, double_integrator_QP_solver_noCD_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	int n;
	int m = 0;
	for( i=0; i<17; i++ ){
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
 * where A is of size [4 x 15]
 * and B is of size [7 x 15]
 * and stored in column major format. Note the transposes of A and B!
 */
void double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_4_15_7(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *y, double_integrator_QP_solver_noCD_FLOAT *z)
{
	int i;
	int j;
	int k = 0;
	int n;
	int m = 0;
	for( i=0; i<15; i++ ){
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
 * Matrix vector multiplication y = M'*x where M is of size [4 x 4]
 * and stored in diagzero format. Note the transpose of M!
 */
void double_integrator_QP_solver_noCD_LA_DIAGZERO_MTVM_4_4(double_integrator_QP_solver_noCD_FLOAT *M, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *y)
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
 * for vectors of length 7. Output z is of course scalar.
 */
void double_integrator_QP_solver_noCD_LA_VSUBADD3_7(double_integrator_QP_solver_noCD_FLOAT* t, double_integrator_QP_solver_noCD_FLOAT* u, int* uidx, double_integrator_QP_solver_noCD_FLOAT* v, double_integrator_QP_solver_noCD_FLOAT* w, double_integrator_QP_solver_noCD_FLOAT* y, double_integrator_QP_solver_noCD_FLOAT* z, double_integrator_QP_solver_noCD_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_noCD_FLOAT norm = *r;
	double_integrator_QP_solver_noCD_FLOAT vx = 0;
	double_integrator_QP_solver_noCD_FLOAT x;
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
void double_integrator_QP_solver_noCD_LA_VSUBADD2_6(double_integrator_QP_solver_noCD_FLOAT* t, int* tidx, double_integrator_QP_solver_noCD_FLOAT* u, double_integrator_QP_solver_noCD_FLOAT* v, double_integrator_QP_solver_noCD_FLOAT* w, double_integrator_QP_solver_noCD_FLOAT* y, double_integrator_QP_solver_noCD_FLOAT* z, double_integrator_QP_solver_noCD_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_noCD_FLOAT norm = *r;
	double_integrator_QP_solver_noCD_FLOAT vx = 0;
	double_integrator_QP_solver_noCD_FLOAT x;
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
void double_integrator_QP_solver_noCD_LA_MVSUBADD_22_17(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *s, double_integrator_QP_solver_noCD_FLOAT *l, double_integrator_QP_solver_noCD_FLOAT *r, double_integrator_QP_solver_noCD_FLOAT *z, double_integrator_QP_solver_noCD_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	double_integrator_QP_solver_noCD_FLOAT Ax[22];
	double_integrator_QP_solver_noCD_FLOAT Axlessb;
	double_integrator_QP_solver_noCD_FLOAT norm = *y;
	double_integrator_QP_solver_noCD_FLOAT lAxlessb = 0;

	/* do A*x first */
	for( i=0; i<22; i++ ){
		Ax[i] = A[k++]*x[0];				
	}	
	for( j=1; j<17; j++ ){		
		for( i=0; i<22; i++ ){
			Ax[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<22; i++ ){
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
void double_integrator_QP_solver_noCD_LA_VSUBADD3_6(double_integrator_QP_solver_noCD_FLOAT* t, double_integrator_QP_solver_noCD_FLOAT* u, int* uidx, double_integrator_QP_solver_noCD_FLOAT* v, double_integrator_QP_solver_noCD_FLOAT* w, double_integrator_QP_solver_noCD_FLOAT* y, double_integrator_QP_solver_noCD_FLOAT* z, double_integrator_QP_solver_noCD_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_noCD_FLOAT norm = *r;
	double_integrator_QP_solver_noCD_FLOAT vx = 0;
	double_integrator_QP_solver_noCD_FLOAT x;
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
void double_integrator_QP_solver_noCD_LA_MVSUBADD_22_15(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *s, double_integrator_QP_solver_noCD_FLOAT *l, double_integrator_QP_solver_noCD_FLOAT *r, double_integrator_QP_solver_noCD_FLOAT *z, double_integrator_QP_solver_noCD_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	double_integrator_QP_solver_noCD_FLOAT Ax[22];
	double_integrator_QP_solver_noCD_FLOAT Axlessb;
	double_integrator_QP_solver_noCD_FLOAT norm = *y;
	double_integrator_QP_solver_noCD_FLOAT lAxlessb = 0;

	/* do A*x first */
	for( i=0; i<22; i++ ){
		Ax[i] = A[k++]*x[0];				
	}	
	for( j=1; j<15; j++ ){		
		for( i=0; i<22; i++ ){
			Ax[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<22; i++ ){
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
void double_integrator_QP_solver_noCD_LA_VSUBADD3_4(double_integrator_QP_solver_noCD_FLOAT* t, double_integrator_QP_solver_noCD_FLOAT* u, int* uidx, double_integrator_QP_solver_noCD_FLOAT* v, double_integrator_QP_solver_noCD_FLOAT* w, double_integrator_QP_solver_noCD_FLOAT* y, double_integrator_QP_solver_noCD_FLOAT* z, double_integrator_QP_solver_noCD_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_noCD_FLOAT norm = *r;
	double_integrator_QP_solver_noCD_FLOAT vx = 0;
	double_integrator_QP_solver_noCD_FLOAT x;
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
void double_integrator_QP_solver_noCD_LA_VSUBADD2_4(double_integrator_QP_solver_noCD_FLOAT* t, int* tidx, double_integrator_QP_solver_noCD_FLOAT* u, double_integrator_QP_solver_noCD_FLOAT* v, double_integrator_QP_solver_noCD_FLOAT* w, double_integrator_QP_solver_noCD_FLOAT* y, double_integrator_QP_solver_noCD_FLOAT* z, double_integrator_QP_solver_noCD_FLOAT* r)
{
	int i;
	double_integrator_QP_solver_noCD_FLOAT norm = *r;
	double_integrator_QP_solver_noCD_FLOAT vx = 0;
	double_integrator_QP_solver_noCD_FLOAT x;
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
void double_integrator_QP_solver_noCD_LA_MVSUBADD_8_4(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *s, double_integrator_QP_solver_noCD_FLOAT *l, double_integrator_QP_solver_noCD_FLOAT *r, double_integrator_QP_solver_noCD_FLOAT *z, double_integrator_QP_solver_noCD_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	double_integrator_QP_solver_noCD_FLOAT Ax[8];
	double_integrator_QP_solver_noCD_FLOAT Axlessb;
	double_integrator_QP_solver_noCD_FLOAT norm = *y;
	double_integrator_QP_solver_noCD_FLOAT lAxlessb = 0;

	/* do A*x first */
	for( i=0; i<8; i++ ){
		Ax[i] = A[k++]*x[0];				
	}	
	for( j=1; j<4; j++ ){		
		for( i=0; i<8; i++ ){
			Ax[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<8; i++ ){
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
 * Special function for box constraints of length 17
 * Returns also L/S, a value that is often used elsewhere.
 */
void double_integrator_QP_solver_noCD_LA_INEQ_B_GRAD_17_7_6(double_integrator_QP_solver_noCD_FLOAT *lu, double_integrator_QP_solver_noCD_FLOAT *su, double_integrator_QP_solver_noCD_FLOAT *ru, double_integrator_QP_solver_noCD_FLOAT *ll, double_integrator_QP_solver_noCD_FLOAT *sl, double_integrator_QP_solver_noCD_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_noCD_FLOAT *grad, double_integrator_QP_solver_noCD_FLOAT *lubysu, double_integrator_QP_solver_noCD_FLOAT *llbysl)
{
	int i;
	for( i=0; i<17; i++ ){
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
void double_integrator_QP_solver_noCD_LA_INEQ_P_22_17(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *lp, double_integrator_QP_solver_noCD_FLOAT *sp, double_integrator_QP_solver_noCD_FLOAT *rip, double_integrator_QP_solver_noCD_FLOAT *grad, double_integrator_QP_solver_noCD_FLOAT *lpbysp)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_noCD_FLOAT lsr[22];
	
	/* do (L/S)*ri first */
	for( j=0; j<22; j++ ){
		lpbysp[j] = lp[j] / sp[j];
		lsr[j] = lpbysp[j]*rip[j];
	}

	for( i=0; i<17; i++ ){		
		for( j=0; j<22; j++ ){
			grad[i] += A[k++]*lsr[j];
		}
	}
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 17
 * Returns also L/S, a value that is often used elsewhere.
 */
void double_integrator_QP_solver_noCD_LA_INEQ_B_GRAD_17_6_6(double_integrator_QP_solver_noCD_FLOAT *lu, double_integrator_QP_solver_noCD_FLOAT *su, double_integrator_QP_solver_noCD_FLOAT *ru, double_integrator_QP_solver_noCD_FLOAT *ll, double_integrator_QP_solver_noCD_FLOAT *sl, double_integrator_QP_solver_noCD_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_noCD_FLOAT *grad, double_integrator_QP_solver_noCD_FLOAT *lubysu, double_integrator_QP_solver_noCD_FLOAT *llbysl)
{
	int i;
	for( i=0; i<17; i++ ){
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
 * Special function for box constraints of length 15
 * Returns also L/S, a value that is often used elsewhere.
 */
void double_integrator_QP_solver_noCD_LA_INEQ_B_GRAD_15_6_6(double_integrator_QP_solver_noCD_FLOAT *lu, double_integrator_QP_solver_noCD_FLOAT *su, double_integrator_QP_solver_noCD_FLOAT *ru, double_integrator_QP_solver_noCD_FLOAT *ll, double_integrator_QP_solver_noCD_FLOAT *sl, double_integrator_QP_solver_noCD_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_noCD_FLOAT *grad, double_integrator_QP_solver_noCD_FLOAT *lubysu, double_integrator_QP_solver_noCD_FLOAT *llbysl)
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
void double_integrator_QP_solver_noCD_LA_INEQ_P_22_15(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *lp, double_integrator_QP_solver_noCD_FLOAT *sp, double_integrator_QP_solver_noCD_FLOAT *rip, double_integrator_QP_solver_noCD_FLOAT *grad, double_integrator_QP_solver_noCD_FLOAT *lpbysp)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_noCD_FLOAT lsr[22];
	
	/* do (L/S)*ri first */
	for( j=0; j<22; j++ ){
		lpbysp[j] = lp[j] / sp[j];
		lsr[j] = lpbysp[j]*rip[j];
	}

	for( i=0; i<15; i++ ){		
		for( j=0; j<22; j++ ){
			grad[i] += A[k++]*lsr[j];
		}
	}
}


/*
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 4
 * Returns also L/S, a value that is often used elsewhere.
 */
void double_integrator_QP_solver_noCD_LA_INEQ_B_GRAD_4_4_4(double_integrator_QP_solver_noCD_FLOAT *lu, double_integrator_QP_solver_noCD_FLOAT *su, double_integrator_QP_solver_noCD_FLOAT *ru, double_integrator_QP_solver_noCD_FLOAT *ll, double_integrator_QP_solver_noCD_FLOAT *sl, double_integrator_QP_solver_noCD_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_noCD_FLOAT *grad, double_integrator_QP_solver_noCD_FLOAT *lubysu, double_integrator_QP_solver_noCD_FLOAT *llbysl)
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
 * Special function for gradient of inequality constraints
 * Calculates grad += A'*(L/S)*rI
 */
void double_integrator_QP_solver_noCD_LA_INEQ_P_8_4(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *lp, double_integrator_QP_solver_noCD_FLOAT *sp, double_integrator_QP_solver_noCD_FLOAT *rip, double_integrator_QP_solver_noCD_FLOAT *grad, double_integrator_QP_solver_noCD_FLOAT *lpbysp)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_noCD_FLOAT lsr[8];
	
	/* do (L/S)*ri first */
	for( j=0; j<8; j++ ){
		lpbysp[j] = lp[j] / sp[j];
		lsr[j] = lpbysp[j]*rip[j];
	}

	for( i=0; i<4; i++ ){		
		for( j=0; j<8; j++ ){
			grad[i] += A[k++]*lsr[j];
		}
	}
}


/*
 * Addition of three vectors  z = u + w + v
 * of length 189.
 */
void double_integrator_QP_solver_noCD_LA_VVADD3_189(double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *v, double_integrator_QP_solver_noCD_FLOAT *w, double_integrator_QP_solver_noCD_FLOAT *z)
{
	int i;
	for( i=0; i<189; i++ ){
		z[i] = u[i] + v[i] + w[i];
	}
}


/*
 * Special function to compute the Dense positive definite 
 * augmented Hessian for block size 17.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void double_integrator_QP_solver_noCD_LA_INEQ_DENSE_DIAG_HESS_17_7_6(double_integrator_QP_solver_noCD_FLOAT *H, double_integrator_QP_solver_noCD_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_noCD_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_noCD_FLOAT *Phi)
{
	int i;
	int j;
	int k = 0;
	
	/* copy diagonal of H into PHI and set lower part of PHI = 0*/
	for( i=0; i<17; i++ ){
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
 * A is stored in column major format and is of size [22 x 17]
 * Phi is of size [17 x 17].
 */
void double_integrator_QP_solver_noCD_LA_DENSE_ADDMTDM_22_17(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *d, double_integrator_QP_solver_noCD_FLOAT *X)
{    
    int i,j,k,ii,di;
    double_integrator_QP_solver_noCD_FLOAT x;
    
    di = 0; ii = 0;
    for( i=0; i<17; i++ ){        
        for( j=0; j<=i; j++ ){
            x = 0;
            for( k=0; k<22; k++ ){
                x += A[i*22+k]*A[j*22+k]*d[k];
            }
            X[ii+j] += x;
        }
        ii += ++di;
    }
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 17.
 */
void double_integrator_QP_solver_noCD_LA_DENSE_CHOL2_17(double_integrator_QP_solver_noCD_FLOAT *A)
{
    int i, j, k, di, dj;
	 int ii, jj;
    double_integrator_QP_solver_noCD_FLOAT l;
    double_integrator_QP_solver_noCD_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<17; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += A[ii+k]*A[ii+k];
        }        
        
        Mii = A[ii+i] - l;
        
#if double_integrator_QP_solver_noCD_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
        for( j=i+1; j<17; j++ ){
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
 * where A is to be computed and is of size [7 x 17],
 * B is given and of size [7 x 17], L is a lower tri-
 * angular matrix of size 17 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_FLOAT *L, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    double_integrator_QP_solver_noCD_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<17; j++ ){        
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
 * The dimensions involved are 17.
 */
void double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_FLOAT *L, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_noCD_FLOAT yel;
            
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


/*
 * Special function to compute the Dense positive definite 
 * augmented Hessian for block size 17.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void double_integrator_QP_solver_noCD_LA_INEQ_DENSE_DIAG_HESS_17_6_6(double_integrator_QP_solver_noCD_FLOAT *H, double_integrator_QP_solver_noCD_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_noCD_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_noCD_FLOAT *Phi)
{
	int i;
	int j;
	int k = 0;
	
	/* copy diagonal of H into PHI and set lower part of PHI = 0*/
	for( i=0; i<17; i++ ){
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
 *	size(A) = [7 x 17]
 *  size(B) = [7 x 17]
 * 
 * and all matrices are stored in column major format.
 *
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE.  
 * 
 */
void double_integrator_QP_solver_noCD_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *C)
{
    int i, j, k;
    double_integrator_QP_solver_noCD_FLOAT temp;
    
    for( i=0; i<7; i++ ){        
        for( j=0; j<7; j++ ){
            temp = 0; 
            for( k=0; k<17; k++ ){
                temp += A[k*7+i]*B[k*7+j];
            }						
            C[j*7+i] = temp;
        }
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
void double_integrator_QP_solver_noCD_LA_INEQ_DENSE_DIAG_HESS_15_6_6(double_integrator_QP_solver_noCD_FLOAT *H, double_integrator_QP_solver_noCD_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_noCD_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_noCD_FLOAT *Phi)
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
 * A is stored in column major format and is of size [22 x 15]
 * Phi is of size [15 x 15].
 */
void double_integrator_QP_solver_noCD_LA_DENSE_ADDMTDM_22_15(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *d, double_integrator_QP_solver_noCD_FLOAT *X)
{    
    int i,j,k,ii,di;
    double_integrator_QP_solver_noCD_FLOAT x;
    
    di = 0; ii = 0;
    for( i=0; i<15; i++ ){        
        for( j=0; j<=i; j++ ){
            x = 0;
            for( k=0; k<22; k++ ){
                x += A[i*22+k]*A[j*22+k]*d[k];
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
void double_integrator_QP_solver_noCD_LA_DENSE_CHOL2_15(double_integrator_QP_solver_noCD_FLOAT *A)
{
    int i, j, k, di, dj;
	 int ii, jj;
    double_integrator_QP_solver_noCD_FLOAT l;
    double_integrator_QP_solver_noCD_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<15; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += A[ii+k]*A[ii+k];
        }        
        
        Mii = A[ii+i] - l;
        
#if double_integrator_QP_solver_noCD_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
 * where A is to be computed and is of size [4 x 15],
 * B is given and of size [4 x 15], L is a lower tri-
 * angular matrix of size 15 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_4_15(double_integrator_QP_solver_noCD_FLOAT *L, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    double_integrator_QP_solver_noCD_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<15; j++ ){        
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
 * where A is to be computed and is of size [7 x 15],
 * B is given and of size [7 x 15], L is a lower tri-
 * angular matrix of size 15 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_15(double_integrator_QP_solver_noCD_FLOAT *L, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    double_integrator_QP_solver_noCD_FLOAT a;
    
    ii=0; di=0;
    for( j=0; j<15; j++ ){        
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
 *	size(A) = [7 x 15]
 *  size(B) = [4 x 15]
 * 
 * and all matrices are stored in column major format.
 *
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE.  
 * 
 */
void double_integrator_QP_solver_noCD_LA_DENSE_MMTM_7_15_4(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *C)
{
    int i, j, k;
    double_integrator_QP_solver_noCD_FLOAT temp;
    
    for( i=0; i<7; i++ ){        
        for( j=0; j<4; j++ ){
            temp = 0; 
            for( k=0; k<15; k++ ){
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
 * The dimensions involved are 15.
 */
void double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_15(double_integrator_QP_solver_noCD_FLOAT *L, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_noCD_FLOAT yel;
            
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
 * augmented Hessian for block size 5.
 *
 * Inputs: - H = diagonal cost Hessian in diagonal storage format
 *         - llbysl = L / S of lower bounds
 *         - lubysu = L / S of upper bounds
 *
 * Output: Phi = H + diag(llbysl) + diag(lubysu)
 * where Phi is stored in lower triangular row major format
 */
void double_integrator_QP_solver_noCD_LA_INEQ_DENSE_DIAG_HESS_5_4_4(double_integrator_QP_solver_noCD_FLOAT *H, double_integrator_QP_solver_noCD_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_noCD_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_noCD_FLOAT *Phi)
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
 * A is stored in column major format and is of size [8 x 4]
 * Phi is of size [4 x 4].
 */
void double_integrator_QP_solver_noCD_LA_DENSE_ADDMTDM_8_4(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *d, double_integrator_QP_solver_noCD_FLOAT *X)
{    
    int i,j,k,ii,di;
    double_integrator_QP_solver_noCD_FLOAT x;
    
    di = 0; ii = 0;
    for( i=0; i<4; i++ ){        
        for( j=0; j<=i; j++ ){
            x = 0;
            for( k=0; k<8; k++ ){
                x += A[i*8+k]*A[j*8+k]*d[k];
            }
            X[ii+j] += x;
        }
        ii += ++di;
    }
}


/**
 * Cholesky factorization as above, but working on a matrix in 
 * lower triangular storage format of size 4.
 */
void double_integrator_QP_solver_noCD_LA_DENSE_CHOL2_4(double_integrator_QP_solver_noCD_FLOAT *A)
{
    int i, j, k, di, dj;
	 int ii, jj;
    double_integrator_QP_solver_noCD_FLOAT l;
    double_integrator_QP_solver_noCD_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<4; i++ ){
        l = 0;
        for( k=0; k<i; k++ ){
            l += A[ii+k]*A[ii+k];
        }        
        
        Mii = A[ii+i] - l;
        
#if double_integrator_QP_solver_noCD_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
        for( j=i+1; j<4; j++ ){
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
 * where A is to be computed and is of size [4 x 4],
 * B is given and of size [4 x 4] stored in 
 * diagzero storage format, L is a lower tri-
 * angular matrix of size 4 stored in lower triangular 
 * storage format. Note the transpose of L!
 *
 * Result: A in column major storage format.
 *
 */
void double_integrator_QP_solver_noCD_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_4_4(double_integrator_QP_solver_noCD_FLOAT *L, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    double_integrator_QP_solver_noCD_FLOAT a;
	
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
	for( j=4; j<4; j++ ){        
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
 * The dimensions involved are 4.
 */
void double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_4(double_integrator_QP_solver_noCD_FLOAT *L, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_noCD_FLOAT yel;
            
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
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [7 x 17] in column
 * storage format, and B is of size [7 x 17] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_noCD_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_noCD_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<7; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<17; k++ ){
                ltemp += A[k*7+i]*A[k*7+j];
            }			
			for( k=0; k<17; k++ ){
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
void double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<7; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<17; j++ ){		
		for( i=0; i<7; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<17; n++ ){
		for( i=0; i<7; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [7 x 17] in column
 * storage format, and B is of size [7 x 15] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_noCD_LA_DENSE_MMT2_7_17_15(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_noCD_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<7; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<17; k++ ){
                ltemp += A[k*7+i]*A[k*7+j];
            }			
			for( k=0; k<15; k++ ){
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
void double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB2_7_17_15(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<7; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<17; j++ ){		
		for( i=0; i<7; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<15; n++ ){
		for( i=0; i<7; i++ ){
			r[i] -= B[m++]*u[n];
		}		
	}
}


/**
 * Compute L = A*A' + B*B', where L is lower triangular of size NXp1
 * and A is a dense matrix of size [4 x 15] in column
 * storage format, and B is of size [4 x 4] also in column
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_noCD_LA_DENSE_MMT2_4_15_4(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_noCD_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<4; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<15; k++ ){
                ltemp += A[k*4+i]*A[k*4+j];
            }			
			for( k=0; k<4; k++ ){
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
void double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB2_4_15_4(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<4; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[m++]*u[0];
	}	
	for( j=1; j<15; j++ ){		
		for( i=0; i<4; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
	
	for( n=1; n<4; n++ ){
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
void double_integrator_QP_solver_noCD_LA_DENSE_CHOL_7(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    double_integrator_QP_solver_noCD_FLOAT l;
    double_integrator_QP_solver_noCD_FLOAT Mii;

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
        
#if double_integrator_QP_solver_noCD_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
void double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_FLOAT *L, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_noCD_FLOAT yel;
            
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
void double_integrator_QP_solver_noCD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_noCD_FLOAT *L, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *A)
{
    int i,j,k,ii,di;
    double_integrator_QP_solver_noCD_FLOAT a;
    
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
void double_integrator_QP_solver_noCD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_noCD_FLOAT ltemp;
    
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
void double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *r)
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
void double_integrator_QP_solver_noCD_LA_DENSE_MATRIXTFORWARDSUB_4_7(double_integrator_QP_solver_noCD_FLOAT *L, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *A)
{
    int i,j,k,ii,di;
    double_integrator_QP_solver_noCD_FLOAT a;
    
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
void double_integrator_QP_solver_noCD_LA_DENSE_MMTSUB_4_7(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_noCD_FLOAT ltemp;
    
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
void double_integrator_QP_solver_noCD_LA_DENSE_CHOL_4(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *L)
{
    int i, j, k, di, dj;
	 int ii, jj;

    double_integrator_QP_solver_noCD_FLOAT l;
    double_integrator_QP_solver_noCD_FLOAT Mii;

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
        
#if double_integrator_QP_solver_noCD_SET_PRINTLEVEL > 0 && defined PRINTNUMERICALWARNINGS
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
void double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_4_7(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *r)
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
 * Backward Substitution to solve L^T*x = y where L is a
 * lower triangular matrix in triangular storage format.
 * 
 * All involved dimensions are 4.
 */
void double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_4(double_integrator_QP_solver_noCD_FLOAT *L, double_integrator_QP_solver_noCD_FLOAT *y, double_integrator_QP_solver_noCD_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_noCD_FLOAT xel;    
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
void double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_4_7(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *r)
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
void double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_FLOAT *L, double_integrator_QP_solver_noCD_FLOAT *y, double_integrator_QP_solver_noCD_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_noCD_FLOAT xel;    
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
void double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *r)
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
 * Vector subtraction z = -x - y for vectors of length 189.
 */
void double_integrator_QP_solver_noCD_LA_VSUB2_189(double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *y, double_integrator_QP_solver_noCD_FLOAT *z)
{
	int i;
	for( i=0; i<189; i++){
		z[i] = -x[i] - y[i];
	}
}


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * lower triangular matrix of size 17 in lower triangular
 * storage format.
 */
void double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_FLOAT *L, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_noCD_FLOAT y[17];
    double_integrator_QP_solver_noCD_FLOAT yel,xel;
	int start = 136;
            
    /* first solve Ly = b by forward substitution */
     ii = 0; di = 0;
    for( i=0; i<17; i++ ){
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


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * lower triangular matrix of size 15 in lower triangular
 * storage format.
 */
void double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_noCD_FLOAT *L, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_noCD_FLOAT y[15];
    double_integrator_QP_solver_noCD_FLOAT yel,xel;
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
 * lower triangular matrix of size 4 in lower triangular
 * storage format.
 */
void double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_4(double_integrator_QP_solver_noCD_FLOAT *L, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_noCD_FLOAT y[4];
    double_integrator_QP_solver_noCD_FLOAT yel,xel;
	int start = 6;
            
    /* first solve Ly = b by forward substitution */
     ii = 0; di = 0;
    for( i=0; i<4; i++ ){
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
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 7,
 * and x has length 17 and is indexed through yidx.
 */
void double_integrator_QP_solver_noCD_LA_VSUB_INDEXED_7(double_integrator_QP_solver_noCD_FLOAT *x, int* xidx, double_integrator_QP_solver_noCD_FLOAT *y, double_integrator_QP_solver_noCD_FLOAT *z)
{
	int i;
	for( i=0; i<7; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 7.
 */
void double_integrator_QP_solver_noCD_LA_VSUB3_7(double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *v, double_integrator_QP_solver_noCD_FLOAT *w, double_integrator_QP_solver_noCD_FLOAT *x)
{
	int i;
	for( i=0; i<7; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = -x - y(yidx) where y is of length 17
 * and z, x and yidx are of length 6.
 */
void double_integrator_QP_solver_noCD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *y, int* yidx, double_integrator_QP_solver_noCD_FLOAT *z)
{
	int i;
	for( i=0; i<6; i++){
		z[i] = -x[i] - y[yidx[i]];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 6.
 */
void double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *v, double_integrator_QP_solver_noCD_FLOAT *w, double_integrator_QP_solver_noCD_FLOAT *x)
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
void double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB4_22_17(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<22; i++ ){
		r[i] = -b[i] - A[k++]*x[0];
	}	
	for( j=1; j<17; j++ ){		
		for( i=0; i<22; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 22.
 */
void double_integrator_QP_solver_noCD_LA_VSUB3_22(double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *v, double_integrator_QP_solver_noCD_FLOAT *w, double_integrator_QP_solver_noCD_FLOAT *x)
{
	int i;
	for( i=0; i<22; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 6,
 * and x has length 17 and is indexed through yidx.
 */
void double_integrator_QP_solver_noCD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_noCD_FLOAT *x, int* xidx, double_integrator_QP_solver_noCD_FLOAT *y, double_integrator_QP_solver_noCD_FLOAT *z)
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
void double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB4_22_15(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<22; i++ ){
		r[i] = -b[i] - A[k++]*x[0];
	}	
	for( j=1; j<15; j++ ){		
		for( i=0; i<22; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 4,
 * and x has length 4 and is indexed through yidx.
 */
void double_integrator_QP_solver_noCD_LA_VSUB_INDEXED_4(double_integrator_QP_solver_noCD_FLOAT *x, int* xidx, double_integrator_QP_solver_noCD_FLOAT *y, double_integrator_QP_solver_noCD_FLOAT *z)
{
	int i;
	for( i=0; i<4; i++){
		z[i] = x[xidx[i]] - y[i];
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 4.
 */
void double_integrator_QP_solver_noCD_LA_VSUB3_4(double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *v, double_integrator_QP_solver_noCD_FLOAT *w, double_integrator_QP_solver_noCD_FLOAT *x)
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
void double_integrator_QP_solver_noCD_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *y, int* yidx, double_integrator_QP_solver_noCD_FLOAT *z)
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
void double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB4_8_4(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<8; i++ ){
		r[i] = -b[i] - A[k++]*x[0];
	}	
	for( j=1; j<4; j++ ){		
		for( i=0; i<8; i++ ){
			r[i] -= A[k++]*x[j];
		}
	}
}


/*
 * Vector subtraction x = -u.*v - w for vectors of length 8.
 */
void double_integrator_QP_solver_noCD_LA_VSUB3_8(double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *v, double_integrator_QP_solver_noCD_FLOAT *w, double_integrator_QP_solver_noCD_FLOAT *x)
{
	int i;
	for( i=0; i<8; i++){
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
 * double_integrator_QP_solver_noCD_NOPROGRESS (should be negative).
 */
int double_integrator_QP_solver_noCD_LINESEARCH_BACKTRACKING_AFFINE(double_integrator_QP_solver_noCD_FLOAT *l, double_integrator_QP_solver_noCD_FLOAT *s, double_integrator_QP_solver_noCD_FLOAT *dl, double_integrator_QP_solver_noCD_FLOAT *ds, double_integrator_QP_solver_noCD_FLOAT *a, double_integrator_QP_solver_noCD_FLOAT *mu_aff)
{
    int i;
	int lsIt=1;    
    double_integrator_QP_solver_noCD_FLOAT dltemp;
    double_integrator_QP_solver_noCD_FLOAT dstemp;
    double_integrator_QP_solver_noCD_FLOAT mya = 1.0;
    double_integrator_QP_solver_noCD_FLOAT mymu;
        
    while( 1 ){                        

        /* 
         * Compute both snew and wnew together.
         * We compute also mu_affine along the way here, as the
         * values might be in registers, so it should be cheaper.
         */
        mymu = 0;
        for( i=0; i<391; i++ ){
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
        if( i == 391 ){
            break;
        } else {
            mya *= double_integrator_QP_solver_noCD_SET_LS_SCALE_AFF;
            if( mya < double_integrator_QP_solver_noCD_SET_LS_MINSTEP ){
                return double_integrator_QP_solver_noCD_NOPROGRESS;
            }
        }
    }
    
    /* return new values and iteration counter */
    *a = mya;
    *mu_aff = mymu / (double_integrator_QP_solver_noCD_FLOAT)391;
    return lsIt;
}


/*
 * Vector subtraction x = (u.*v - mu)*sigma where a is a scalar
*  and x,u,v are vectors of length 391.
 */
void double_integrator_QP_solver_noCD_LA_VSUB5_391(double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *v, double_integrator_QP_solver_noCD_FLOAT mu,  double_integrator_QP_solver_noCD_FLOAT sigma, double_integrator_QP_solver_noCD_FLOAT *x)
{
	int i;
	for( i=0; i<391; i++){
		x[i] = u[i]*v[i] - mu;
		x[i] *= sigma;
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 17,
 * u, su, uidx are of length 6 and v, sv, vidx are of length 7.
 */
void double_integrator_QP_solver_noCD_LA_VSUB6_INDEXED_17_6_7(double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *su, int* uidx, double_integrator_QP_solver_noCD_FLOAT *v, double_integrator_QP_solver_noCD_FLOAT *sv, int* vidx, double_integrator_QP_solver_noCD_FLOAT *x)
{
	int i;
	for( i=0; i<17; i++ ){
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
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 17,
 * u, su, uidx are of length 6 and v, sv, vidx are of length 6.
 */
void double_integrator_QP_solver_noCD_LA_VSUB6_INDEXED_17_6_6(double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *su, int* uidx, double_integrator_QP_solver_noCD_FLOAT *v, double_integrator_QP_solver_noCD_FLOAT *sv, int* vidx, double_integrator_QP_solver_noCD_FLOAT *x)
{
	int i;
	for( i=0; i<17; i++ ){
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
 * Matrix vector multiplication z = z + A'*(x./s) where A is of size [22 x 17]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_noCD_LA_DENSE_MTVMADD2_22_17(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *s, double_integrator_QP_solver_noCD_FLOAT *z)
{
	int i;
	int j;
	int k = 0; 
	double_integrator_QP_solver_noCD_FLOAT temp[22];

	for( j=0; j<22; j++ ){
		temp[j] = x[j] / s[j];
	}

	for( i=0; i<17; i++ ){
		for( j=0; j<22; j++ ){
			z[i] += A[k++]*temp[j];
		}
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void double_integrator_QP_solver_noCD_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<7; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<17; j++ ){		
		for( i=0; i<7; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<17; n++ ){
		for( i=0; i<7; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 15,
 * u, su, uidx are of length 6 and v, sv, vidx are of length 6.
 */
void double_integrator_QP_solver_noCD_LA_VSUB6_INDEXED_15_6_6(double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *su, int* uidx, double_integrator_QP_solver_noCD_FLOAT *v, double_integrator_QP_solver_noCD_FLOAT *sv, int* vidx, double_integrator_QP_solver_noCD_FLOAT *x)
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
 * Matrix vector multiplication z = z + A'*(x./s) where A is of size [22 x 15]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_noCD_LA_DENSE_MTVMADD2_22_15(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *s, double_integrator_QP_solver_noCD_FLOAT *z)
{
	int i;
	int j;
	int k = 0; 
	double_integrator_QP_solver_noCD_FLOAT temp[22];

	for( j=0; j<22; j++ ){
		temp[j] = x[j] / s[j];
	}

	for( i=0; i<15; i++ ){
		for( j=0; j<22; j++ ){
			z[i] += A[k++]*temp[j];
		}
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void double_integrator_QP_solver_noCD_LA_DENSE_2MVMADD_7_17_15(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<7; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<17; j++ ){		
		for( i=0; i<7; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<15; n++ ){
		for( i=0; i<7; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 4,
 * u, su, uidx are of length 4 and v, sv, vidx are of length 4.
 */
void double_integrator_QP_solver_noCD_LA_VSUB6_INDEXED_4_4_4(double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *su, int* uidx, double_integrator_QP_solver_noCD_FLOAT *v, double_integrator_QP_solver_noCD_FLOAT *sv, int* vidx, double_integrator_QP_solver_noCD_FLOAT *x)
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
 * Matrix vector multiplication z = z + A'*(x./s) where A is of size [8 x 4]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_noCD_LA_DENSE_MTVMADD2_8_4(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *s, double_integrator_QP_solver_noCD_FLOAT *z)
{
	int i;
	int j;
	int k = 0; 
	double_integrator_QP_solver_noCD_FLOAT temp[8];

	for( j=0; j<8; j++ ){
		temp[j] = x[j] / s[j];
	}

	for( i=0; i<4; i++ ){
		for( j=0; j<8; j++ ){
			z[i] += A[k++]*temp[j];
		}
	}
}


/* 
 * Computes r = A*x + B*u
 * where A an B are stored in column major format
 */
void double_integrator_QP_solver_noCD_LA_DENSE_2MVMADD_4_15_4(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *B, double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;
	int m = 0;
	int n;

	for( i=0; i<4; i++ ){
		r[i] = A[k++]*x[0] + B[m++]*u[0];
	}	

	for( j=1; j<15; j++ ){		
		for( i=0; i<4; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
	for( n=1; n<4; n++ ){
		for( i=0; i<4; i++ ){
			r[i] += B[m++]*u[n];
		}		
	}
}


/*
 * Vector subtraction z = x - y for vectors of length 189.
 */
void double_integrator_QP_solver_noCD_LA_VSUB_189(double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *y, double_integrator_QP_solver_noCD_FLOAT *z)
{
	int i;
	for( i=0; i<189; i++){
		z[i] = x[i] - y[i];
	}
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 7 (length of y >= 7).
 */
void double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_noCD_FLOAT *r, double_integrator_QP_solver_noCD_FLOAT *s, double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *y, int* yidx, double_integrator_QP_solver_noCD_FLOAT *z)
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
void double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_noCD_FLOAT *r, double_integrator_QP_solver_noCD_FLOAT *s, double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *y, int* yidx, double_integrator_QP_solver_noCD_FLOAT *z)
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
void double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB5_22_17(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *s, double_integrator_QP_solver_noCD_FLOAT *l, double_integrator_QP_solver_noCD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_noCD_FLOAT temp[22];

	
	for( i=0; i<22; i++ ){
		temp[i] = A[k++]*x[0];
	}
	

	for( j=1; j<17; j++ ){		
		for( i=0; i<22; i++ ){
			temp[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<22; i++ ){
		r[i] = (-b[i] + l[i]*temp[i])/s[i]; 
	}	
	
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 6 (length of y >= 6).
 */
void double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_noCD_FLOAT *r, double_integrator_QP_solver_noCD_FLOAT *s, double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *y, int* yidx, double_integrator_QP_solver_noCD_FLOAT *z)
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
void double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB5_22_15(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *s, double_integrator_QP_solver_noCD_FLOAT *l, double_integrator_QP_solver_noCD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_noCD_FLOAT temp[22];

	
	for( i=0; i<22; i++ ){
		temp[i] = A[k++]*x[0];
	}
	

	for( j=1; j<15; j++ ){		
		for( i=0; i<22; i++ ){
			temp[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<22; i++ ){
		r[i] = (-b[i] + l[i]*temp[i])/s[i]; 
	}	
	
}


/** 
 * Computes z = -r./s - u.*y(y)
 * where all vectors except of y are of length 4 (length of y >= 4).
 */
void double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_noCD_FLOAT *r, double_integrator_QP_solver_noCD_FLOAT *s, double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *y, int* yidx, double_integrator_QP_solver_noCD_FLOAT *z)
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
void double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_noCD_FLOAT *r, double_integrator_QP_solver_noCD_FLOAT *s, double_integrator_QP_solver_noCD_FLOAT *u, double_integrator_QP_solver_noCD_FLOAT *y, int* yidx, double_integrator_QP_solver_noCD_FLOAT *z)
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
void double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB5_8_4(double_integrator_QP_solver_noCD_FLOAT *A, double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *b, double_integrator_QP_solver_noCD_FLOAT *s, double_integrator_QP_solver_noCD_FLOAT *l, double_integrator_QP_solver_noCD_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_noCD_FLOAT temp[8];

	
	for( i=0; i<8; i++ ){
		temp[i] = A[k++]*x[0];
	}
	

	for( j=1; j<4; j++ ){		
		for( i=0; i<8; i++ ){
			temp[i] += A[k++]*x[j];
		}
	}

	for( i=0; i<8; i++ ){
		r[i] = (-b[i] + l[i]*temp[i])/s[i]; 
	}	
	
}


/*
 * Computes ds = -l.\(r + s.*dl) for vectors of length 391.
 */
void double_integrator_QP_solver_noCD_LA_VSUB7_391(double_integrator_QP_solver_noCD_FLOAT *l, double_integrator_QP_solver_noCD_FLOAT *r, double_integrator_QP_solver_noCD_FLOAT *s, double_integrator_QP_solver_noCD_FLOAT *dl, double_integrator_QP_solver_noCD_FLOAT *ds)
{
	int i;
	for( i=0; i<391; i++){
		ds[i] = -(r[i] + s[i]*dl[i])/l[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 189.
 */
void double_integrator_QP_solver_noCD_LA_VADD_189(double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *y)
{
	int i;
	for( i=0; i<189; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 74.
 */
void double_integrator_QP_solver_noCD_LA_VADD_74(double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *y)
{
	int i;
	for( i=0; i<74; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 391.
 */
void double_integrator_QP_solver_noCD_LA_VADD_391(double_integrator_QP_solver_noCD_FLOAT *x, double_integrator_QP_solver_noCD_FLOAT *y)
{
	int i;
	for( i=0; i<391; i++){
		x[i] += y[i];
	}
}


/**
 * Backtracking line search for combined predictor/corrector step.
 * Update on variables with safety factor gamma (to keep us away from
 * boundary).
 */
int double_integrator_QP_solver_noCD_LINESEARCH_BACKTRACKING_COMBINED(double_integrator_QP_solver_noCD_FLOAT *z, double_integrator_QP_solver_noCD_FLOAT *v, double_integrator_QP_solver_noCD_FLOAT *l, double_integrator_QP_solver_noCD_FLOAT *s, double_integrator_QP_solver_noCD_FLOAT *dz, double_integrator_QP_solver_noCD_FLOAT *dv, double_integrator_QP_solver_noCD_FLOAT *dl, double_integrator_QP_solver_noCD_FLOAT *ds, double_integrator_QP_solver_noCD_FLOAT *a, double_integrator_QP_solver_noCD_FLOAT *mu)
{
    int i, lsIt=1;       
    double_integrator_QP_solver_noCD_FLOAT dltemp;
    double_integrator_QP_solver_noCD_FLOAT dstemp;    
    double_integrator_QP_solver_noCD_FLOAT a_gamma;
            
    *a = 1.0;
    while( 1 ){                        

        /* check whether search criterion is fulfilled */
        for( i=0; i<391; i++ ){
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
        if( i == 391 ){
            break;
        } else {
            *a *= double_integrator_QP_solver_noCD_SET_LS_SCALE;
            if( *a < double_integrator_QP_solver_noCD_SET_LS_MINSTEP ){
                return double_integrator_QP_solver_noCD_NOPROGRESS;
            }
        }
    }
    
    /* update variables with safety margin */
    a_gamma = (*a)*double_integrator_QP_solver_noCD_SET_LS_MAXSTEP;
    
    /* primal variables */
    for( i=0; i<189; i++ ){
        z[i] += a_gamma*dz[i];
    }
    
    /* equality constraint multipliers */
    for( i=0; i<74; i++ ){
        v[i] += a_gamma*dv[i];
    }
    
    /* inequality constraint multipliers & slacks, also update mu */
    *mu = 0;
    for( i=0; i<391; i++ ){
        dltemp = l[i] + a_gamma*dl[i]; l[i] = dltemp;
        dstemp = s[i] + a_gamma*ds[i]; s[i] = dstemp;
        *mu += dltemp*dstemp;
    }
    
    *a = a_gamma;
    *mu /= (double_integrator_QP_solver_noCD_FLOAT)391;
    return lsIt;
}




/* VARIABLE DEFINITIONS ------------------------------------------------ */
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_z[189];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_v[74];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_dz_aff[189];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_dv_aff[74];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_grad_cost[189];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_grad_eq[189];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rd[189];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_l[391];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_s[391];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_lbys[391];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_dl_aff[391];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_ds_aff[391];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_dz_cc[189];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_dv_cc[74];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_dl_cc[391];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_ds_cc[391];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_ccrhs[391];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_grad_ineq[189];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_H00[17] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_z00 = double_integrator_QP_solver_noCD_z + 0;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzaff00 = double_integrator_QP_solver_noCD_dz_aff + 0;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzcc00 = double_integrator_QP_solver_noCD_dz_cc + 0;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_rd00 = double_integrator_QP_solver_noCD_rd + 0;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lbyrd00[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_cost00 = double_integrator_QP_solver_noCD_grad_cost + 0;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_eq00 = double_integrator_QP_solver_noCD_grad_eq + 0;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_ineq00 = double_integrator_QP_solver_noCD_grad_ineq + 0;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_ctv00[17];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_C00[119] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
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
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_v00 = double_integrator_QP_solver_noCD_v + 0;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_re00[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_beta00[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_betacc00[7];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvaff00 = double_integrator_QP_solver_noCD_dv_aff + 0;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvcc00 = double_integrator_QP_solver_noCD_dv_cc + 0;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_V00[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Yd00[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ld00[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_yy00[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_bmy00[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_c00[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_noCD_lbIdx00[7] = {0, 1, 2, 3, 4, 5, 12};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llb00 = double_integrator_QP_solver_noCD_l + 0;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_slb00 = double_integrator_QP_solver_noCD_s + 0;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llbbyslb00 = double_integrator_QP_solver_noCD_lbys + 0;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rilb00[7];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbaff00 = double_integrator_QP_solver_noCD_dl_aff + 0;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbaff00 = double_integrator_QP_solver_noCD_ds_aff + 0;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbcc00 = double_integrator_QP_solver_noCD_dl_cc + 0;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbcc00 = double_integrator_QP_solver_noCD_ds_cc + 0;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsl00 = double_integrator_QP_solver_noCD_ccrhs + 0;
int double_integrator_QP_solver_noCD_ubIdx00[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lub00 = double_integrator_QP_solver_noCD_l + 7;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sub00 = double_integrator_QP_solver_noCD_s + 7;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lubbysub00 = double_integrator_QP_solver_noCD_lbys + 7;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_riub00[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubaff00 = double_integrator_QP_solver_noCD_dl_aff + 7;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubaff00 = double_integrator_QP_solver_noCD_ds_aff + 7;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubcc00 = double_integrator_QP_solver_noCD_dl_cc + 7;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubcc00 = double_integrator_QP_solver_noCD_ds_cc + 7;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsub00 = double_integrator_QP_solver_noCD_ccrhs + 7;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sp00 = double_integrator_QP_solver_noCD_s + 13;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lp00 = double_integrator_QP_solver_noCD_l + 13;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lpbysp00 = double_integrator_QP_solver_noCD_lbys + 13;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_aff00 = double_integrator_QP_solver_noCD_dl_aff + 13;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_aff00 = double_integrator_QP_solver_noCD_ds_aff + 13;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_cc00 = double_integrator_QP_solver_noCD_dl_cc + 13;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_cc00 = double_integrator_QP_solver_noCD_ds_cc + 13;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsp00 = double_integrator_QP_solver_noCD_ccrhs + 13;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rip00[22];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Phi00[153];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_z01 = double_integrator_QP_solver_noCD_z + 17;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzaff01 = double_integrator_QP_solver_noCD_dz_aff + 17;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzcc01 = double_integrator_QP_solver_noCD_dz_cc + 17;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_rd01 = double_integrator_QP_solver_noCD_rd + 17;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lbyrd01[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_cost01 = double_integrator_QP_solver_noCD_grad_cost + 17;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_eq01 = double_integrator_QP_solver_noCD_grad_eq + 17;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_ineq01 = double_integrator_QP_solver_noCD_grad_ineq + 17;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_ctv01[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_v01 = double_integrator_QP_solver_noCD_v + 7;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_re01[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_beta01[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_betacc01[7];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvaff01 = double_integrator_QP_solver_noCD_dv_aff + 7;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvcc01 = double_integrator_QP_solver_noCD_dv_cc + 7;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_V01[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Yd01[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ld01[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_yy01[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_bmy01[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_c01[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_noCD_lbIdx01[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llb01 = double_integrator_QP_solver_noCD_l + 35;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_slb01 = double_integrator_QP_solver_noCD_s + 35;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llbbyslb01 = double_integrator_QP_solver_noCD_lbys + 35;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rilb01[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbaff01 = double_integrator_QP_solver_noCD_dl_aff + 35;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbaff01 = double_integrator_QP_solver_noCD_ds_aff + 35;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbcc01 = double_integrator_QP_solver_noCD_dl_cc + 35;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbcc01 = double_integrator_QP_solver_noCD_ds_cc + 35;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsl01 = double_integrator_QP_solver_noCD_ccrhs + 35;
int double_integrator_QP_solver_noCD_ubIdx01[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lub01 = double_integrator_QP_solver_noCD_l + 41;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sub01 = double_integrator_QP_solver_noCD_s + 41;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lubbysub01 = double_integrator_QP_solver_noCD_lbys + 41;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_riub01[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubaff01 = double_integrator_QP_solver_noCD_dl_aff + 41;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubaff01 = double_integrator_QP_solver_noCD_ds_aff + 41;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubcc01 = double_integrator_QP_solver_noCD_dl_cc + 41;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubcc01 = double_integrator_QP_solver_noCD_ds_cc + 41;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsub01 = double_integrator_QP_solver_noCD_ccrhs + 41;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sp01 = double_integrator_QP_solver_noCD_s + 47;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lp01 = double_integrator_QP_solver_noCD_l + 47;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lpbysp01 = double_integrator_QP_solver_noCD_lbys + 47;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_aff01 = double_integrator_QP_solver_noCD_dl_aff + 47;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_aff01 = double_integrator_QP_solver_noCD_ds_aff + 47;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_cc01 = double_integrator_QP_solver_noCD_dl_cc + 47;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_cc01 = double_integrator_QP_solver_noCD_ds_cc + 47;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsp01 = double_integrator_QP_solver_noCD_ccrhs + 47;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rip01[22];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Phi01[153];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_D01[119] = {-1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
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
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_W01[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ysd01[49];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lsd01[49];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_z02 = double_integrator_QP_solver_noCD_z + 34;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzaff02 = double_integrator_QP_solver_noCD_dz_aff + 34;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzcc02 = double_integrator_QP_solver_noCD_dz_cc + 34;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_rd02 = double_integrator_QP_solver_noCD_rd + 34;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lbyrd02[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_cost02 = double_integrator_QP_solver_noCD_grad_cost + 34;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_eq02 = double_integrator_QP_solver_noCD_grad_eq + 34;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_ineq02 = double_integrator_QP_solver_noCD_grad_ineq + 34;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_ctv02[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_v02 = double_integrator_QP_solver_noCD_v + 14;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_re02[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_beta02[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_betacc02[7];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvaff02 = double_integrator_QP_solver_noCD_dv_aff + 14;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvcc02 = double_integrator_QP_solver_noCD_dv_cc + 14;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_V02[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Yd02[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ld02[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_yy02[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_bmy02[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_c02[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_noCD_lbIdx02[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llb02 = double_integrator_QP_solver_noCD_l + 69;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_slb02 = double_integrator_QP_solver_noCD_s + 69;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llbbyslb02 = double_integrator_QP_solver_noCD_lbys + 69;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rilb02[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbaff02 = double_integrator_QP_solver_noCD_dl_aff + 69;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbaff02 = double_integrator_QP_solver_noCD_ds_aff + 69;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbcc02 = double_integrator_QP_solver_noCD_dl_cc + 69;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbcc02 = double_integrator_QP_solver_noCD_ds_cc + 69;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsl02 = double_integrator_QP_solver_noCD_ccrhs + 69;
int double_integrator_QP_solver_noCD_ubIdx02[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lub02 = double_integrator_QP_solver_noCD_l + 75;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sub02 = double_integrator_QP_solver_noCD_s + 75;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lubbysub02 = double_integrator_QP_solver_noCD_lbys + 75;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_riub02[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubaff02 = double_integrator_QP_solver_noCD_dl_aff + 75;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubaff02 = double_integrator_QP_solver_noCD_ds_aff + 75;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubcc02 = double_integrator_QP_solver_noCD_dl_cc + 75;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubcc02 = double_integrator_QP_solver_noCD_ds_cc + 75;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsub02 = double_integrator_QP_solver_noCD_ccrhs + 75;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sp02 = double_integrator_QP_solver_noCD_s + 81;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lp02 = double_integrator_QP_solver_noCD_l + 81;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lpbysp02 = double_integrator_QP_solver_noCD_lbys + 81;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_aff02 = double_integrator_QP_solver_noCD_dl_aff + 81;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_aff02 = double_integrator_QP_solver_noCD_ds_aff + 81;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_cc02 = double_integrator_QP_solver_noCD_dl_cc + 81;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_cc02 = double_integrator_QP_solver_noCD_ds_cc + 81;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsp02 = double_integrator_QP_solver_noCD_ccrhs + 81;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rip02[22];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Phi02[153];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_W02[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ysd02[49];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lsd02[49];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_z03 = double_integrator_QP_solver_noCD_z + 51;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzaff03 = double_integrator_QP_solver_noCD_dz_aff + 51;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzcc03 = double_integrator_QP_solver_noCD_dz_cc + 51;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_rd03 = double_integrator_QP_solver_noCD_rd + 51;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lbyrd03[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_cost03 = double_integrator_QP_solver_noCD_grad_cost + 51;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_eq03 = double_integrator_QP_solver_noCD_grad_eq + 51;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_ineq03 = double_integrator_QP_solver_noCD_grad_ineq + 51;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_ctv03[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_v03 = double_integrator_QP_solver_noCD_v + 21;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_re03[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_beta03[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_betacc03[7];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvaff03 = double_integrator_QP_solver_noCD_dv_aff + 21;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvcc03 = double_integrator_QP_solver_noCD_dv_cc + 21;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_V03[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Yd03[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ld03[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_yy03[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_bmy03[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_c03[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_noCD_lbIdx03[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llb03 = double_integrator_QP_solver_noCD_l + 103;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_slb03 = double_integrator_QP_solver_noCD_s + 103;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llbbyslb03 = double_integrator_QP_solver_noCD_lbys + 103;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rilb03[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbaff03 = double_integrator_QP_solver_noCD_dl_aff + 103;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbaff03 = double_integrator_QP_solver_noCD_ds_aff + 103;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbcc03 = double_integrator_QP_solver_noCD_dl_cc + 103;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbcc03 = double_integrator_QP_solver_noCD_ds_cc + 103;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsl03 = double_integrator_QP_solver_noCD_ccrhs + 103;
int double_integrator_QP_solver_noCD_ubIdx03[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lub03 = double_integrator_QP_solver_noCD_l + 109;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sub03 = double_integrator_QP_solver_noCD_s + 109;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lubbysub03 = double_integrator_QP_solver_noCD_lbys + 109;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_riub03[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubaff03 = double_integrator_QP_solver_noCD_dl_aff + 109;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubaff03 = double_integrator_QP_solver_noCD_ds_aff + 109;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubcc03 = double_integrator_QP_solver_noCD_dl_cc + 109;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubcc03 = double_integrator_QP_solver_noCD_ds_cc + 109;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsub03 = double_integrator_QP_solver_noCD_ccrhs + 109;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sp03 = double_integrator_QP_solver_noCD_s + 115;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lp03 = double_integrator_QP_solver_noCD_l + 115;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lpbysp03 = double_integrator_QP_solver_noCD_lbys + 115;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_aff03 = double_integrator_QP_solver_noCD_dl_aff + 115;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_aff03 = double_integrator_QP_solver_noCD_ds_aff + 115;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_cc03 = double_integrator_QP_solver_noCD_dl_cc + 115;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_cc03 = double_integrator_QP_solver_noCD_ds_cc + 115;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsp03 = double_integrator_QP_solver_noCD_ccrhs + 115;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rip03[22];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Phi03[153];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_W03[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ysd03[49];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lsd03[49];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_z04 = double_integrator_QP_solver_noCD_z + 68;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzaff04 = double_integrator_QP_solver_noCD_dz_aff + 68;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzcc04 = double_integrator_QP_solver_noCD_dz_cc + 68;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_rd04 = double_integrator_QP_solver_noCD_rd + 68;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lbyrd04[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_cost04 = double_integrator_QP_solver_noCD_grad_cost + 68;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_eq04 = double_integrator_QP_solver_noCD_grad_eq + 68;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_ineq04 = double_integrator_QP_solver_noCD_grad_ineq + 68;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_ctv04[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_v04 = double_integrator_QP_solver_noCD_v + 28;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_re04[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_beta04[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_betacc04[7];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvaff04 = double_integrator_QP_solver_noCD_dv_aff + 28;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvcc04 = double_integrator_QP_solver_noCD_dv_cc + 28;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_V04[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Yd04[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ld04[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_yy04[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_bmy04[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_c04[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_noCD_lbIdx04[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llb04 = double_integrator_QP_solver_noCD_l + 137;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_slb04 = double_integrator_QP_solver_noCD_s + 137;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llbbyslb04 = double_integrator_QP_solver_noCD_lbys + 137;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rilb04[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbaff04 = double_integrator_QP_solver_noCD_dl_aff + 137;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbaff04 = double_integrator_QP_solver_noCD_ds_aff + 137;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbcc04 = double_integrator_QP_solver_noCD_dl_cc + 137;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbcc04 = double_integrator_QP_solver_noCD_ds_cc + 137;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsl04 = double_integrator_QP_solver_noCD_ccrhs + 137;
int double_integrator_QP_solver_noCD_ubIdx04[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lub04 = double_integrator_QP_solver_noCD_l + 143;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sub04 = double_integrator_QP_solver_noCD_s + 143;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lubbysub04 = double_integrator_QP_solver_noCD_lbys + 143;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_riub04[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubaff04 = double_integrator_QP_solver_noCD_dl_aff + 143;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubaff04 = double_integrator_QP_solver_noCD_ds_aff + 143;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubcc04 = double_integrator_QP_solver_noCD_dl_cc + 143;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubcc04 = double_integrator_QP_solver_noCD_ds_cc + 143;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsub04 = double_integrator_QP_solver_noCD_ccrhs + 143;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sp04 = double_integrator_QP_solver_noCD_s + 149;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lp04 = double_integrator_QP_solver_noCD_l + 149;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lpbysp04 = double_integrator_QP_solver_noCD_lbys + 149;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_aff04 = double_integrator_QP_solver_noCD_dl_aff + 149;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_aff04 = double_integrator_QP_solver_noCD_ds_aff + 149;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_cc04 = double_integrator_QP_solver_noCD_dl_cc + 149;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_cc04 = double_integrator_QP_solver_noCD_ds_cc + 149;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsp04 = double_integrator_QP_solver_noCD_ccrhs + 149;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rip04[22];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Phi04[153];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_W04[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ysd04[49];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lsd04[49];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_z05 = double_integrator_QP_solver_noCD_z + 85;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzaff05 = double_integrator_QP_solver_noCD_dz_aff + 85;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzcc05 = double_integrator_QP_solver_noCD_dz_cc + 85;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_rd05 = double_integrator_QP_solver_noCD_rd + 85;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lbyrd05[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_cost05 = double_integrator_QP_solver_noCD_grad_cost + 85;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_eq05 = double_integrator_QP_solver_noCD_grad_eq + 85;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_ineq05 = double_integrator_QP_solver_noCD_grad_ineq + 85;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_ctv05[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_v05 = double_integrator_QP_solver_noCD_v + 35;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_re05[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_beta05[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_betacc05[7];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvaff05 = double_integrator_QP_solver_noCD_dv_aff + 35;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvcc05 = double_integrator_QP_solver_noCD_dv_cc + 35;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_V05[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Yd05[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ld05[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_yy05[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_bmy05[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_c05[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_noCD_lbIdx05[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llb05 = double_integrator_QP_solver_noCD_l + 171;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_slb05 = double_integrator_QP_solver_noCD_s + 171;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llbbyslb05 = double_integrator_QP_solver_noCD_lbys + 171;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rilb05[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbaff05 = double_integrator_QP_solver_noCD_dl_aff + 171;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbaff05 = double_integrator_QP_solver_noCD_ds_aff + 171;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbcc05 = double_integrator_QP_solver_noCD_dl_cc + 171;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbcc05 = double_integrator_QP_solver_noCD_ds_cc + 171;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsl05 = double_integrator_QP_solver_noCD_ccrhs + 171;
int double_integrator_QP_solver_noCD_ubIdx05[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lub05 = double_integrator_QP_solver_noCD_l + 177;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sub05 = double_integrator_QP_solver_noCD_s + 177;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lubbysub05 = double_integrator_QP_solver_noCD_lbys + 177;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_riub05[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubaff05 = double_integrator_QP_solver_noCD_dl_aff + 177;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubaff05 = double_integrator_QP_solver_noCD_ds_aff + 177;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubcc05 = double_integrator_QP_solver_noCD_dl_cc + 177;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubcc05 = double_integrator_QP_solver_noCD_ds_cc + 177;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsub05 = double_integrator_QP_solver_noCD_ccrhs + 177;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sp05 = double_integrator_QP_solver_noCD_s + 183;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lp05 = double_integrator_QP_solver_noCD_l + 183;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lpbysp05 = double_integrator_QP_solver_noCD_lbys + 183;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_aff05 = double_integrator_QP_solver_noCD_dl_aff + 183;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_aff05 = double_integrator_QP_solver_noCD_ds_aff + 183;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_cc05 = double_integrator_QP_solver_noCD_dl_cc + 183;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_cc05 = double_integrator_QP_solver_noCD_ds_cc + 183;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsp05 = double_integrator_QP_solver_noCD_ccrhs + 183;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rip05[22];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Phi05[153];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_W05[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ysd05[49];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lsd05[49];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_z06 = double_integrator_QP_solver_noCD_z + 102;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzaff06 = double_integrator_QP_solver_noCD_dz_aff + 102;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzcc06 = double_integrator_QP_solver_noCD_dz_cc + 102;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_rd06 = double_integrator_QP_solver_noCD_rd + 102;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lbyrd06[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_cost06 = double_integrator_QP_solver_noCD_grad_cost + 102;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_eq06 = double_integrator_QP_solver_noCD_grad_eq + 102;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_ineq06 = double_integrator_QP_solver_noCD_grad_ineq + 102;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_ctv06[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_v06 = double_integrator_QP_solver_noCD_v + 42;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_re06[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_beta06[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_betacc06[7];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvaff06 = double_integrator_QP_solver_noCD_dv_aff + 42;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvcc06 = double_integrator_QP_solver_noCD_dv_cc + 42;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_V06[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Yd06[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ld06[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_yy06[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_bmy06[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_c06[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_noCD_lbIdx06[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llb06 = double_integrator_QP_solver_noCD_l + 205;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_slb06 = double_integrator_QP_solver_noCD_s + 205;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llbbyslb06 = double_integrator_QP_solver_noCD_lbys + 205;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rilb06[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbaff06 = double_integrator_QP_solver_noCD_dl_aff + 205;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbaff06 = double_integrator_QP_solver_noCD_ds_aff + 205;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbcc06 = double_integrator_QP_solver_noCD_dl_cc + 205;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbcc06 = double_integrator_QP_solver_noCD_ds_cc + 205;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsl06 = double_integrator_QP_solver_noCD_ccrhs + 205;
int double_integrator_QP_solver_noCD_ubIdx06[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lub06 = double_integrator_QP_solver_noCD_l + 211;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sub06 = double_integrator_QP_solver_noCD_s + 211;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lubbysub06 = double_integrator_QP_solver_noCD_lbys + 211;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_riub06[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubaff06 = double_integrator_QP_solver_noCD_dl_aff + 211;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubaff06 = double_integrator_QP_solver_noCD_ds_aff + 211;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubcc06 = double_integrator_QP_solver_noCD_dl_cc + 211;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubcc06 = double_integrator_QP_solver_noCD_ds_cc + 211;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsub06 = double_integrator_QP_solver_noCD_ccrhs + 211;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sp06 = double_integrator_QP_solver_noCD_s + 217;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lp06 = double_integrator_QP_solver_noCD_l + 217;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lpbysp06 = double_integrator_QP_solver_noCD_lbys + 217;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_aff06 = double_integrator_QP_solver_noCD_dl_aff + 217;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_aff06 = double_integrator_QP_solver_noCD_ds_aff + 217;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_cc06 = double_integrator_QP_solver_noCD_dl_cc + 217;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_cc06 = double_integrator_QP_solver_noCD_ds_cc + 217;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsp06 = double_integrator_QP_solver_noCD_ccrhs + 217;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rip06[22];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Phi06[153];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_W06[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ysd06[49];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lsd06[49];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_z07 = double_integrator_QP_solver_noCD_z + 119;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzaff07 = double_integrator_QP_solver_noCD_dz_aff + 119;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzcc07 = double_integrator_QP_solver_noCD_dz_cc + 119;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_rd07 = double_integrator_QP_solver_noCD_rd + 119;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lbyrd07[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_cost07 = double_integrator_QP_solver_noCD_grad_cost + 119;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_eq07 = double_integrator_QP_solver_noCD_grad_eq + 119;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_ineq07 = double_integrator_QP_solver_noCD_grad_ineq + 119;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_ctv07[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_v07 = double_integrator_QP_solver_noCD_v + 49;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_re07[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_beta07[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_betacc07[7];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvaff07 = double_integrator_QP_solver_noCD_dv_aff + 49;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvcc07 = double_integrator_QP_solver_noCD_dv_cc + 49;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_V07[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Yd07[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ld07[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_yy07[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_bmy07[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_c07[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_noCD_lbIdx07[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llb07 = double_integrator_QP_solver_noCD_l + 239;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_slb07 = double_integrator_QP_solver_noCD_s + 239;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llbbyslb07 = double_integrator_QP_solver_noCD_lbys + 239;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rilb07[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbaff07 = double_integrator_QP_solver_noCD_dl_aff + 239;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbaff07 = double_integrator_QP_solver_noCD_ds_aff + 239;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbcc07 = double_integrator_QP_solver_noCD_dl_cc + 239;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbcc07 = double_integrator_QP_solver_noCD_ds_cc + 239;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsl07 = double_integrator_QP_solver_noCD_ccrhs + 239;
int double_integrator_QP_solver_noCD_ubIdx07[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lub07 = double_integrator_QP_solver_noCD_l + 245;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sub07 = double_integrator_QP_solver_noCD_s + 245;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lubbysub07 = double_integrator_QP_solver_noCD_lbys + 245;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_riub07[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubaff07 = double_integrator_QP_solver_noCD_dl_aff + 245;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubaff07 = double_integrator_QP_solver_noCD_ds_aff + 245;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubcc07 = double_integrator_QP_solver_noCD_dl_cc + 245;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubcc07 = double_integrator_QP_solver_noCD_ds_cc + 245;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsub07 = double_integrator_QP_solver_noCD_ccrhs + 245;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sp07 = double_integrator_QP_solver_noCD_s + 251;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lp07 = double_integrator_QP_solver_noCD_l + 251;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lpbysp07 = double_integrator_QP_solver_noCD_lbys + 251;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_aff07 = double_integrator_QP_solver_noCD_dl_aff + 251;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_aff07 = double_integrator_QP_solver_noCD_ds_aff + 251;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_cc07 = double_integrator_QP_solver_noCD_dl_cc + 251;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_cc07 = double_integrator_QP_solver_noCD_ds_cc + 251;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsp07 = double_integrator_QP_solver_noCD_ccrhs + 251;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rip07[22];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Phi07[153];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_W07[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ysd07[49];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lsd07[49];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_z08 = double_integrator_QP_solver_noCD_z + 136;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzaff08 = double_integrator_QP_solver_noCD_dz_aff + 136;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzcc08 = double_integrator_QP_solver_noCD_dz_cc + 136;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_rd08 = double_integrator_QP_solver_noCD_rd + 136;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lbyrd08[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_cost08 = double_integrator_QP_solver_noCD_grad_cost + 136;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_eq08 = double_integrator_QP_solver_noCD_grad_eq + 136;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_ineq08 = double_integrator_QP_solver_noCD_grad_ineq + 136;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_ctv08[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_v08 = double_integrator_QP_solver_noCD_v + 56;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_re08[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_beta08[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_betacc08[7];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvaff08 = double_integrator_QP_solver_noCD_dv_aff + 56;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvcc08 = double_integrator_QP_solver_noCD_dv_cc + 56;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_V08[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Yd08[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ld08[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_yy08[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_bmy08[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_c08[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_noCD_lbIdx08[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llb08 = double_integrator_QP_solver_noCD_l + 273;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_slb08 = double_integrator_QP_solver_noCD_s + 273;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llbbyslb08 = double_integrator_QP_solver_noCD_lbys + 273;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rilb08[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbaff08 = double_integrator_QP_solver_noCD_dl_aff + 273;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbaff08 = double_integrator_QP_solver_noCD_ds_aff + 273;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbcc08 = double_integrator_QP_solver_noCD_dl_cc + 273;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbcc08 = double_integrator_QP_solver_noCD_ds_cc + 273;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsl08 = double_integrator_QP_solver_noCD_ccrhs + 273;
int double_integrator_QP_solver_noCD_ubIdx08[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lub08 = double_integrator_QP_solver_noCD_l + 279;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sub08 = double_integrator_QP_solver_noCD_s + 279;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lubbysub08 = double_integrator_QP_solver_noCD_lbys + 279;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_riub08[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubaff08 = double_integrator_QP_solver_noCD_dl_aff + 279;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubaff08 = double_integrator_QP_solver_noCD_ds_aff + 279;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubcc08 = double_integrator_QP_solver_noCD_dl_cc + 279;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubcc08 = double_integrator_QP_solver_noCD_ds_cc + 279;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsub08 = double_integrator_QP_solver_noCD_ccrhs + 279;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sp08 = double_integrator_QP_solver_noCD_s + 285;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lp08 = double_integrator_QP_solver_noCD_l + 285;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lpbysp08 = double_integrator_QP_solver_noCD_lbys + 285;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_aff08 = double_integrator_QP_solver_noCD_dl_aff + 285;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_aff08 = double_integrator_QP_solver_noCD_ds_aff + 285;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_cc08 = double_integrator_QP_solver_noCD_dl_cc + 285;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_cc08 = double_integrator_QP_solver_noCD_ds_cc + 285;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsp08 = double_integrator_QP_solver_noCD_ccrhs + 285;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rip08[22];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Phi08[153];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_W08[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ysd08[49];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lsd08[49];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_z09 = double_integrator_QP_solver_noCD_z + 153;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzaff09 = double_integrator_QP_solver_noCD_dz_aff + 153;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzcc09 = double_integrator_QP_solver_noCD_dz_cc + 153;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_rd09 = double_integrator_QP_solver_noCD_rd + 153;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lbyrd09[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_cost09 = double_integrator_QP_solver_noCD_grad_cost + 153;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_eq09 = double_integrator_QP_solver_noCD_grad_eq + 153;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_ineq09 = double_integrator_QP_solver_noCD_grad_ineq + 153;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_ctv09[17];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_v09 = double_integrator_QP_solver_noCD_v + 63;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_re09[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_beta09[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_betacc09[7];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvaff09 = double_integrator_QP_solver_noCD_dv_aff + 63;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvcc09 = double_integrator_QP_solver_noCD_dv_cc + 63;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_V09[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Yd09[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ld09[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_yy09[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_bmy09[7];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_c09[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_noCD_lbIdx09[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llb09 = double_integrator_QP_solver_noCD_l + 307;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_slb09 = double_integrator_QP_solver_noCD_s + 307;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llbbyslb09 = double_integrator_QP_solver_noCD_lbys + 307;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rilb09[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbaff09 = double_integrator_QP_solver_noCD_dl_aff + 307;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbaff09 = double_integrator_QP_solver_noCD_ds_aff + 307;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbcc09 = double_integrator_QP_solver_noCD_dl_cc + 307;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbcc09 = double_integrator_QP_solver_noCD_ds_cc + 307;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsl09 = double_integrator_QP_solver_noCD_ccrhs + 307;
int double_integrator_QP_solver_noCD_ubIdx09[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lub09 = double_integrator_QP_solver_noCD_l + 313;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sub09 = double_integrator_QP_solver_noCD_s + 313;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lubbysub09 = double_integrator_QP_solver_noCD_lbys + 313;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_riub09[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubaff09 = double_integrator_QP_solver_noCD_dl_aff + 313;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubaff09 = double_integrator_QP_solver_noCD_ds_aff + 313;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubcc09 = double_integrator_QP_solver_noCD_dl_cc + 313;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubcc09 = double_integrator_QP_solver_noCD_ds_cc + 313;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsub09 = double_integrator_QP_solver_noCD_ccrhs + 313;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sp09 = double_integrator_QP_solver_noCD_s + 319;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lp09 = double_integrator_QP_solver_noCD_l + 319;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lpbysp09 = double_integrator_QP_solver_noCD_lbys + 319;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_aff09 = double_integrator_QP_solver_noCD_dl_aff + 319;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_aff09 = double_integrator_QP_solver_noCD_ds_aff + 319;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_cc09 = double_integrator_QP_solver_noCD_dl_cc + 319;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_cc09 = double_integrator_QP_solver_noCD_ds_cc + 319;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsp09 = double_integrator_QP_solver_noCD_ccrhs + 319;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rip09[22];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Phi09[153];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_W09[119];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ysd09[49];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lsd09[49];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_H10[15] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_z10 = double_integrator_QP_solver_noCD_z + 170;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzaff10 = double_integrator_QP_solver_noCD_dz_aff + 170;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzcc10 = double_integrator_QP_solver_noCD_dz_cc + 170;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_rd10 = double_integrator_QP_solver_noCD_rd + 170;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lbyrd10[15];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_cost10 = double_integrator_QP_solver_noCD_grad_cost + 170;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_eq10 = double_integrator_QP_solver_noCD_grad_eq + 170;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_ineq10 = double_integrator_QP_solver_noCD_grad_ineq + 170;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_ctv10[15];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_C10[60] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
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
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_v10 = double_integrator_QP_solver_noCD_v + 70;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_re10[4];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_beta10[4];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_betacc10[4];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvaff10 = double_integrator_QP_solver_noCD_dv_aff + 70;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dvcc10 = double_integrator_QP_solver_noCD_dv_cc + 70;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_V10[60];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Yd10[10];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ld10[10];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_yy10[4];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_bmy10[4];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_c10[4] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_noCD_lbIdx10[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llb10 = double_integrator_QP_solver_noCD_l + 341;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_slb10 = double_integrator_QP_solver_noCD_s + 341;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llbbyslb10 = double_integrator_QP_solver_noCD_lbys + 341;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rilb10[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbaff10 = double_integrator_QP_solver_noCD_dl_aff + 341;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbaff10 = double_integrator_QP_solver_noCD_ds_aff + 341;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbcc10 = double_integrator_QP_solver_noCD_dl_cc + 341;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbcc10 = double_integrator_QP_solver_noCD_ds_cc + 341;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsl10 = double_integrator_QP_solver_noCD_ccrhs + 341;
int double_integrator_QP_solver_noCD_ubIdx10[6] = {0, 1, 2, 3, 4, 5};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lub10 = double_integrator_QP_solver_noCD_l + 347;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sub10 = double_integrator_QP_solver_noCD_s + 347;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lubbysub10 = double_integrator_QP_solver_noCD_lbys + 347;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_riub10[6];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubaff10 = double_integrator_QP_solver_noCD_dl_aff + 347;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubaff10 = double_integrator_QP_solver_noCD_ds_aff + 347;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubcc10 = double_integrator_QP_solver_noCD_dl_cc + 347;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubcc10 = double_integrator_QP_solver_noCD_ds_cc + 347;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsub10 = double_integrator_QP_solver_noCD_ccrhs + 347;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sp10 = double_integrator_QP_solver_noCD_s + 353;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lp10 = double_integrator_QP_solver_noCD_l + 353;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lpbysp10 = double_integrator_QP_solver_noCD_lbys + 353;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_aff10 = double_integrator_QP_solver_noCD_dl_aff + 353;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_aff10 = double_integrator_QP_solver_noCD_ds_aff + 353;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_cc10 = double_integrator_QP_solver_noCD_dl_cc + 353;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_cc10 = double_integrator_QP_solver_noCD_ds_cc + 353;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsp10 = double_integrator_QP_solver_noCD_ccrhs + 353;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rip10[22];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Phi10[120];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_D10[105] = {-1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
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
0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_W10[105];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Ysd10[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lsd10[28];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_H11[5] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_z11 = double_integrator_QP_solver_noCD_z + 185;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzaff11 = double_integrator_QP_solver_noCD_dz_aff + 185;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dzcc11 = double_integrator_QP_solver_noCD_dz_cc + 185;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_rd11 = double_integrator_QP_solver_noCD_rd + 185;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Lbyrd11[4];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_cost11 = double_integrator_QP_solver_noCD_grad_cost + 185;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_eq11 = double_integrator_QP_solver_noCD_grad_eq + 185;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_grad_ineq11 = double_integrator_QP_solver_noCD_grad_ineq + 185;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_ctv11[4];
int double_integrator_QP_solver_noCD_lbIdx11[4] = {0, 1, 2, 3};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llb11 = double_integrator_QP_solver_noCD_l + 375;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_slb11 = double_integrator_QP_solver_noCD_s + 375;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_llbbyslb11 = double_integrator_QP_solver_noCD_lbys + 375;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rilb11[4];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbaff11 = double_integrator_QP_solver_noCD_dl_aff + 375;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbaff11 = double_integrator_QP_solver_noCD_ds_aff + 375;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dllbcc11 = double_integrator_QP_solver_noCD_dl_cc + 375;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dslbcc11 = double_integrator_QP_solver_noCD_ds_cc + 375;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsl11 = double_integrator_QP_solver_noCD_ccrhs + 375;
int double_integrator_QP_solver_noCD_ubIdx11[4] = {0, 1, 2, 3};
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lub11 = double_integrator_QP_solver_noCD_l + 379;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sub11 = double_integrator_QP_solver_noCD_s + 379;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lubbysub11 = double_integrator_QP_solver_noCD_lbys + 379;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_riub11[4];
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubaff11 = double_integrator_QP_solver_noCD_dl_aff + 379;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubaff11 = double_integrator_QP_solver_noCD_ds_aff + 379;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlubcc11 = double_integrator_QP_solver_noCD_dl_cc + 379;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsubcc11 = double_integrator_QP_solver_noCD_ds_cc + 379;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsub11 = double_integrator_QP_solver_noCD_ccrhs + 379;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_sp11 = double_integrator_QP_solver_noCD_s + 383;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lp11 = double_integrator_QP_solver_noCD_l + 383;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_lpbysp11 = double_integrator_QP_solver_noCD_lbys + 383;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_aff11 = double_integrator_QP_solver_noCD_dl_aff + 383;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_aff11 = double_integrator_QP_solver_noCD_ds_aff + 383;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dlp_cc11 = double_integrator_QP_solver_noCD_dl_cc + 383;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_dsp_cc11 = double_integrator_QP_solver_noCD_ds_cc + 383;
double_integrator_QP_solver_noCD_FLOAT* double_integrator_QP_solver_noCD_ccrhsp11 = double_integrator_QP_solver_noCD_ccrhs + 383;
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_rip11[8];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_Phi11[10];
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_D11[4] = {-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000};
double_integrator_QP_solver_noCD_FLOAT double_integrator_QP_solver_noCD_W11[16];
double_integrator_QP_solver_noCD_FLOAT musigma;
double_integrator_QP_solver_noCD_FLOAT sigma_3rdroot;




/* SOLVER CODE --------------------------------------------------------- */
int double_integrator_QP_solver_noCD_solve(double_integrator_QP_solver_noCD_params* params, double_integrator_QP_solver_noCD_output* output, double_integrator_QP_solver_noCD_info* info)
{	
int exitcode;

#if double_integrator_QP_solver_noCD_SET_TIMING == 1
	double_integrator_QP_solver_noCD_timer solvertimer;
	double_integrator_QP_solver_noCD_tic(&solvertimer);
#endif
/* FUNCTION CALLS INTO LA LIBRARY -------------------------------------- */
info->it = 0;
double_integrator_QP_solver_noCD_LA_INITIALIZEVECTOR_189(double_integrator_QP_solver_noCD_z, 0);
double_integrator_QP_solver_noCD_LA_INITIALIZEVECTOR_74(double_integrator_QP_solver_noCD_v, 1);
double_integrator_QP_solver_noCD_LA_INITIALIZEVECTOR_391(double_integrator_QP_solver_noCD_l, 10);
double_integrator_QP_solver_noCD_LA_INITIALIZEVECTOR_391(double_integrator_QP_solver_noCD_s, 10);
info->mu = 0;
double_integrator_QP_solver_noCD_LA_DOTACC_391(double_integrator_QP_solver_noCD_l, double_integrator_QP_solver_noCD_s, &info->mu);
info->mu /= 391;
while( 1 ){
info->pobj = 0;
double_integrator_QP_solver_noCD_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_noCD_H00, params->f1, double_integrator_QP_solver_noCD_z00, double_integrator_QP_solver_noCD_grad_cost00, &info->pobj);
double_integrator_QP_solver_noCD_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_noCD_H00, params->f2, double_integrator_QP_solver_noCD_z01, double_integrator_QP_solver_noCD_grad_cost01, &info->pobj);
double_integrator_QP_solver_noCD_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_noCD_H00, params->f3, double_integrator_QP_solver_noCD_z02, double_integrator_QP_solver_noCD_grad_cost02, &info->pobj);
double_integrator_QP_solver_noCD_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_noCD_H00, params->f4, double_integrator_QP_solver_noCD_z03, double_integrator_QP_solver_noCD_grad_cost03, &info->pobj);
double_integrator_QP_solver_noCD_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_noCD_H00, params->f5, double_integrator_QP_solver_noCD_z04, double_integrator_QP_solver_noCD_grad_cost04, &info->pobj);
double_integrator_QP_solver_noCD_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_noCD_H00, params->f6, double_integrator_QP_solver_noCD_z05, double_integrator_QP_solver_noCD_grad_cost05, &info->pobj);
double_integrator_QP_solver_noCD_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_noCD_H00, params->f7, double_integrator_QP_solver_noCD_z06, double_integrator_QP_solver_noCD_grad_cost06, &info->pobj);
double_integrator_QP_solver_noCD_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_noCD_H00, params->f8, double_integrator_QP_solver_noCD_z07, double_integrator_QP_solver_noCD_grad_cost07, &info->pobj);
double_integrator_QP_solver_noCD_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_noCD_H00, params->f9, double_integrator_QP_solver_noCD_z08, double_integrator_QP_solver_noCD_grad_cost08, &info->pobj);
double_integrator_QP_solver_noCD_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_noCD_H00, params->f10, double_integrator_QP_solver_noCD_z09, double_integrator_QP_solver_noCD_grad_cost09, &info->pobj);
double_integrator_QP_solver_noCD_LA_DIAG_QUADFCN_15(double_integrator_QP_solver_noCD_H10, params->f11, double_integrator_QP_solver_noCD_z10, double_integrator_QP_solver_noCD_grad_cost10, &info->pobj);
double_integrator_QP_solver_noCD_LA_DIAG_QUADFCN_4(double_integrator_QP_solver_noCD_H11, params->f12, double_integrator_QP_solver_noCD_z11, double_integrator_QP_solver_noCD_grad_cost11, &info->pobj);
info->res_eq = 0;
info->dgap = 0;
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_z00, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_z01, double_integrator_QP_solver_noCD_c00, double_integrator_QP_solver_noCD_v00, double_integrator_QP_solver_noCD_re00, &info->dgap, &info->res_eq);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_z01, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_z02, double_integrator_QP_solver_noCD_c01, double_integrator_QP_solver_noCD_v01, double_integrator_QP_solver_noCD_re01, &info->dgap, &info->res_eq);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_z02, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_z03, double_integrator_QP_solver_noCD_c02, double_integrator_QP_solver_noCD_v02, double_integrator_QP_solver_noCD_re02, &info->dgap, &info->res_eq);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_z03, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_z04, double_integrator_QP_solver_noCD_c03, double_integrator_QP_solver_noCD_v03, double_integrator_QP_solver_noCD_re03, &info->dgap, &info->res_eq);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_z04, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_z05, double_integrator_QP_solver_noCD_c04, double_integrator_QP_solver_noCD_v04, double_integrator_QP_solver_noCD_re04, &info->dgap, &info->res_eq);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_z05, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_z06, double_integrator_QP_solver_noCD_c05, double_integrator_QP_solver_noCD_v05, double_integrator_QP_solver_noCD_re05, &info->dgap, &info->res_eq);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_z06, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_z07, double_integrator_QP_solver_noCD_c06, double_integrator_QP_solver_noCD_v06, double_integrator_QP_solver_noCD_re06, &info->dgap, &info->res_eq);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_z07, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_z08, double_integrator_QP_solver_noCD_c07, double_integrator_QP_solver_noCD_v07, double_integrator_QP_solver_noCD_re07, &info->dgap, &info->res_eq);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_z08, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_z09, double_integrator_QP_solver_noCD_c08, double_integrator_QP_solver_noCD_v08, double_integrator_QP_solver_noCD_re08, &info->dgap, &info->res_eq);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB3_7_17_15(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_z09, double_integrator_QP_solver_noCD_D10, double_integrator_QP_solver_noCD_z10, double_integrator_QP_solver_noCD_c09, double_integrator_QP_solver_noCD_v09, double_integrator_QP_solver_noCD_re09, &info->dgap, &info->res_eq);
double_integrator_QP_solver_noCD_LA_DENSE_DIAGZERO_MVMSUB3_4_15_4(double_integrator_QP_solver_noCD_C10, double_integrator_QP_solver_noCD_z10, double_integrator_QP_solver_noCD_D11, double_integrator_QP_solver_noCD_z11, double_integrator_QP_solver_noCD_c10, double_integrator_QP_solver_noCD_v10, double_integrator_QP_solver_noCD_re10, &info->dgap, &info->res_eq);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM_7_17(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_v00, double_integrator_QP_solver_noCD_grad_eq00);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_v01, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_v00, double_integrator_QP_solver_noCD_grad_eq01);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_v02, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_v01, double_integrator_QP_solver_noCD_grad_eq02);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_v03, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_v02, double_integrator_QP_solver_noCD_grad_eq03);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_v04, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_v03, double_integrator_QP_solver_noCD_grad_eq04);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_v05, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_v04, double_integrator_QP_solver_noCD_grad_eq05);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_v06, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_v05, double_integrator_QP_solver_noCD_grad_eq06);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_v07, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_v06, double_integrator_QP_solver_noCD_grad_eq07);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_v08, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_v07, double_integrator_QP_solver_noCD_grad_eq08);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_v09, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_v08, double_integrator_QP_solver_noCD_grad_eq09);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_4_15_7(double_integrator_QP_solver_noCD_C10, double_integrator_QP_solver_noCD_v10, double_integrator_QP_solver_noCD_D10, double_integrator_QP_solver_noCD_v09, double_integrator_QP_solver_noCD_grad_eq10);
double_integrator_QP_solver_noCD_LA_DIAGZERO_MTVM_4_4(double_integrator_QP_solver_noCD_D11, double_integrator_QP_solver_noCD_v10, double_integrator_QP_solver_noCD_grad_eq11);
info->res_ineq = 0;
double_integrator_QP_solver_noCD_LA_VSUBADD3_7(params->lb1, double_integrator_QP_solver_noCD_z00, double_integrator_QP_solver_noCD_lbIdx00, double_integrator_QP_solver_noCD_llb00, double_integrator_QP_solver_noCD_slb00, double_integrator_QP_solver_noCD_rilb00, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD2_6(double_integrator_QP_solver_noCD_z00, double_integrator_QP_solver_noCD_ubIdx00, params->ub1, double_integrator_QP_solver_noCD_lub00, double_integrator_QP_solver_noCD_sub00, double_integrator_QP_solver_noCD_riub00, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_MVSUBADD_22_17(params->A1, double_integrator_QP_solver_noCD_z00, params->b1, double_integrator_QP_solver_noCD_sp00, double_integrator_QP_solver_noCD_lp00, double_integrator_QP_solver_noCD_rip00, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD3_6(params->lb2, double_integrator_QP_solver_noCD_z01, double_integrator_QP_solver_noCD_lbIdx01, double_integrator_QP_solver_noCD_llb01, double_integrator_QP_solver_noCD_slb01, double_integrator_QP_solver_noCD_rilb01, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD2_6(double_integrator_QP_solver_noCD_z01, double_integrator_QP_solver_noCD_ubIdx01, params->ub2, double_integrator_QP_solver_noCD_lub01, double_integrator_QP_solver_noCD_sub01, double_integrator_QP_solver_noCD_riub01, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_MVSUBADD_22_17(params->A2, double_integrator_QP_solver_noCD_z01, params->b2, double_integrator_QP_solver_noCD_sp01, double_integrator_QP_solver_noCD_lp01, double_integrator_QP_solver_noCD_rip01, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD3_6(params->lb3, double_integrator_QP_solver_noCD_z02, double_integrator_QP_solver_noCD_lbIdx02, double_integrator_QP_solver_noCD_llb02, double_integrator_QP_solver_noCD_slb02, double_integrator_QP_solver_noCD_rilb02, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD2_6(double_integrator_QP_solver_noCD_z02, double_integrator_QP_solver_noCD_ubIdx02, params->ub3, double_integrator_QP_solver_noCD_lub02, double_integrator_QP_solver_noCD_sub02, double_integrator_QP_solver_noCD_riub02, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_MVSUBADD_22_17(params->A3, double_integrator_QP_solver_noCD_z02, params->b3, double_integrator_QP_solver_noCD_sp02, double_integrator_QP_solver_noCD_lp02, double_integrator_QP_solver_noCD_rip02, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD3_6(params->lb4, double_integrator_QP_solver_noCD_z03, double_integrator_QP_solver_noCD_lbIdx03, double_integrator_QP_solver_noCD_llb03, double_integrator_QP_solver_noCD_slb03, double_integrator_QP_solver_noCD_rilb03, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD2_6(double_integrator_QP_solver_noCD_z03, double_integrator_QP_solver_noCD_ubIdx03, params->ub4, double_integrator_QP_solver_noCD_lub03, double_integrator_QP_solver_noCD_sub03, double_integrator_QP_solver_noCD_riub03, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_MVSUBADD_22_17(params->A4, double_integrator_QP_solver_noCD_z03, params->b4, double_integrator_QP_solver_noCD_sp03, double_integrator_QP_solver_noCD_lp03, double_integrator_QP_solver_noCD_rip03, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD3_6(params->lb5, double_integrator_QP_solver_noCD_z04, double_integrator_QP_solver_noCD_lbIdx04, double_integrator_QP_solver_noCD_llb04, double_integrator_QP_solver_noCD_slb04, double_integrator_QP_solver_noCD_rilb04, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD2_6(double_integrator_QP_solver_noCD_z04, double_integrator_QP_solver_noCD_ubIdx04, params->ub5, double_integrator_QP_solver_noCD_lub04, double_integrator_QP_solver_noCD_sub04, double_integrator_QP_solver_noCD_riub04, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_MVSUBADD_22_17(params->A5, double_integrator_QP_solver_noCD_z04, params->b5, double_integrator_QP_solver_noCD_sp04, double_integrator_QP_solver_noCD_lp04, double_integrator_QP_solver_noCD_rip04, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD3_6(params->lb6, double_integrator_QP_solver_noCD_z05, double_integrator_QP_solver_noCD_lbIdx05, double_integrator_QP_solver_noCD_llb05, double_integrator_QP_solver_noCD_slb05, double_integrator_QP_solver_noCD_rilb05, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD2_6(double_integrator_QP_solver_noCD_z05, double_integrator_QP_solver_noCD_ubIdx05, params->ub6, double_integrator_QP_solver_noCD_lub05, double_integrator_QP_solver_noCD_sub05, double_integrator_QP_solver_noCD_riub05, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_MVSUBADD_22_17(params->A6, double_integrator_QP_solver_noCD_z05, params->b6, double_integrator_QP_solver_noCD_sp05, double_integrator_QP_solver_noCD_lp05, double_integrator_QP_solver_noCD_rip05, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD3_6(params->lb7, double_integrator_QP_solver_noCD_z06, double_integrator_QP_solver_noCD_lbIdx06, double_integrator_QP_solver_noCD_llb06, double_integrator_QP_solver_noCD_slb06, double_integrator_QP_solver_noCD_rilb06, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD2_6(double_integrator_QP_solver_noCD_z06, double_integrator_QP_solver_noCD_ubIdx06, params->ub7, double_integrator_QP_solver_noCD_lub06, double_integrator_QP_solver_noCD_sub06, double_integrator_QP_solver_noCD_riub06, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_MVSUBADD_22_17(params->A7, double_integrator_QP_solver_noCD_z06, params->b7, double_integrator_QP_solver_noCD_sp06, double_integrator_QP_solver_noCD_lp06, double_integrator_QP_solver_noCD_rip06, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD3_6(params->lb8, double_integrator_QP_solver_noCD_z07, double_integrator_QP_solver_noCD_lbIdx07, double_integrator_QP_solver_noCD_llb07, double_integrator_QP_solver_noCD_slb07, double_integrator_QP_solver_noCD_rilb07, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD2_6(double_integrator_QP_solver_noCD_z07, double_integrator_QP_solver_noCD_ubIdx07, params->ub8, double_integrator_QP_solver_noCD_lub07, double_integrator_QP_solver_noCD_sub07, double_integrator_QP_solver_noCD_riub07, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_MVSUBADD_22_17(params->A8, double_integrator_QP_solver_noCD_z07, params->b8, double_integrator_QP_solver_noCD_sp07, double_integrator_QP_solver_noCD_lp07, double_integrator_QP_solver_noCD_rip07, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD3_6(params->lb9, double_integrator_QP_solver_noCD_z08, double_integrator_QP_solver_noCD_lbIdx08, double_integrator_QP_solver_noCD_llb08, double_integrator_QP_solver_noCD_slb08, double_integrator_QP_solver_noCD_rilb08, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD2_6(double_integrator_QP_solver_noCD_z08, double_integrator_QP_solver_noCD_ubIdx08, params->ub9, double_integrator_QP_solver_noCD_lub08, double_integrator_QP_solver_noCD_sub08, double_integrator_QP_solver_noCD_riub08, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_MVSUBADD_22_17(params->A9, double_integrator_QP_solver_noCD_z08, params->b9, double_integrator_QP_solver_noCD_sp08, double_integrator_QP_solver_noCD_lp08, double_integrator_QP_solver_noCD_rip08, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD3_6(params->lb10, double_integrator_QP_solver_noCD_z09, double_integrator_QP_solver_noCD_lbIdx09, double_integrator_QP_solver_noCD_llb09, double_integrator_QP_solver_noCD_slb09, double_integrator_QP_solver_noCD_rilb09, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD2_6(double_integrator_QP_solver_noCD_z09, double_integrator_QP_solver_noCD_ubIdx09, params->ub10, double_integrator_QP_solver_noCD_lub09, double_integrator_QP_solver_noCD_sub09, double_integrator_QP_solver_noCD_riub09, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_MVSUBADD_22_17(params->A10, double_integrator_QP_solver_noCD_z09, params->b10, double_integrator_QP_solver_noCD_sp09, double_integrator_QP_solver_noCD_lp09, double_integrator_QP_solver_noCD_rip09, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD3_6(params->lb11, double_integrator_QP_solver_noCD_z10, double_integrator_QP_solver_noCD_lbIdx10, double_integrator_QP_solver_noCD_llb10, double_integrator_QP_solver_noCD_slb10, double_integrator_QP_solver_noCD_rilb10, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD2_6(double_integrator_QP_solver_noCD_z10, double_integrator_QP_solver_noCD_ubIdx10, params->ub11, double_integrator_QP_solver_noCD_lub10, double_integrator_QP_solver_noCD_sub10, double_integrator_QP_solver_noCD_riub10, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_MVSUBADD_22_15(params->A11, double_integrator_QP_solver_noCD_z10, params->b11, double_integrator_QP_solver_noCD_sp10, double_integrator_QP_solver_noCD_lp10, double_integrator_QP_solver_noCD_rip10, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD3_4(params->lb12, double_integrator_QP_solver_noCD_z11, double_integrator_QP_solver_noCD_lbIdx11, double_integrator_QP_solver_noCD_llb11, double_integrator_QP_solver_noCD_slb11, double_integrator_QP_solver_noCD_rilb11, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_VSUBADD2_4(double_integrator_QP_solver_noCD_z11, double_integrator_QP_solver_noCD_ubIdx11, params->ub12, double_integrator_QP_solver_noCD_lub11, double_integrator_QP_solver_noCD_sub11, double_integrator_QP_solver_noCD_riub11, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_MVSUBADD_8_4(params->A12, double_integrator_QP_solver_noCD_z11, params->b12, double_integrator_QP_solver_noCD_sp11, double_integrator_QP_solver_noCD_lp11, double_integrator_QP_solver_noCD_rip11, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_noCD_LA_INEQ_B_GRAD_17_7_6(double_integrator_QP_solver_noCD_lub00, double_integrator_QP_solver_noCD_sub00, double_integrator_QP_solver_noCD_riub00, double_integrator_QP_solver_noCD_llb00, double_integrator_QP_solver_noCD_slb00, double_integrator_QP_solver_noCD_rilb00, double_integrator_QP_solver_noCD_lbIdx00, double_integrator_QP_solver_noCD_ubIdx00, double_integrator_QP_solver_noCD_grad_ineq00, double_integrator_QP_solver_noCD_lubbysub00, double_integrator_QP_solver_noCD_llbbyslb00);
double_integrator_QP_solver_noCD_LA_INEQ_P_22_17(params->A1, double_integrator_QP_solver_noCD_lp00, double_integrator_QP_solver_noCD_sp00, double_integrator_QP_solver_noCD_rip00, double_integrator_QP_solver_noCD_grad_ineq00, double_integrator_QP_solver_noCD_lpbysp00);
double_integrator_QP_solver_noCD_LA_INEQ_B_GRAD_17_6_6(double_integrator_QP_solver_noCD_lub01, double_integrator_QP_solver_noCD_sub01, double_integrator_QP_solver_noCD_riub01, double_integrator_QP_solver_noCD_llb01, double_integrator_QP_solver_noCD_slb01, double_integrator_QP_solver_noCD_rilb01, double_integrator_QP_solver_noCD_lbIdx01, double_integrator_QP_solver_noCD_ubIdx01, double_integrator_QP_solver_noCD_grad_ineq01, double_integrator_QP_solver_noCD_lubbysub01, double_integrator_QP_solver_noCD_llbbyslb01);
double_integrator_QP_solver_noCD_LA_INEQ_P_22_17(params->A2, double_integrator_QP_solver_noCD_lp01, double_integrator_QP_solver_noCD_sp01, double_integrator_QP_solver_noCD_rip01, double_integrator_QP_solver_noCD_grad_ineq01, double_integrator_QP_solver_noCD_lpbysp01);
double_integrator_QP_solver_noCD_LA_INEQ_B_GRAD_17_6_6(double_integrator_QP_solver_noCD_lub02, double_integrator_QP_solver_noCD_sub02, double_integrator_QP_solver_noCD_riub02, double_integrator_QP_solver_noCD_llb02, double_integrator_QP_solver_noCD_slb02, double_integrator_QP_solver_noCD_rilb02, double_integrator_QP_solver_noCD_lbIdx02, double_integrator_QP_solver_noCD_ubIdx02, double_integrator_QP_solver_noCD_grad_ineq02, double_integrator_QP_solver_noCD_lubbysub02, double_integrator_QP_solver_noCD_llbbyslb02);
double_integrator_QP_solver_noCD_LA_INEQ_P_22_17(params->A3, double_integrator_QP_solver_noCD_lp02, double_integrator_QP_solver_noCD_sp02, double_integrator_QP_solver_noCD_rip02, double_integrator_QP_solver_noCD_grad_ineq02, double_integrator_QP_solver_noCD_lpbysp02);
double_integrator_QP_solver_noCD_LA_INEQ_B_GRAD_17_6_6(double_integrator_QP_solver_noCD_lub03, double_integrator_QP_solver_noCD_sub03, double_integrator_QP_solver_noCD_riub03, double_integrator_QP_solver_noCD_llb03, double_integrator_QP_solver_noCD_slb03, double_integrator_QP_solver_noCD_rilb03, double_integrator_QP_solver_noCD_lbIdx03, double_integrator_QP_solver_noCD_ubIdx03, double_integrator_QP_solver_noCD_grad_ineq03, double_integrator_QP_solver_noCD_lubbysub03, double_integrator_QP_solver_noCD_llbbyslb03);
double_integrator_QP_solver_noCD_LA_INEQ_P_22_17(params->A4, double_integrator_QP_solver_noCD_lp03, double_integrator_QP_solver_noCD_sp03, double_integrator_QP_solver_noCD_rip03, double_integrator_QP_solver_noCD_grad_ineq03, double_integrator_QP_solver_noCD_lpbysp03);
double_integrator_QP_solver_noCD_LA_INEQ_B_GRAD_17_6_6(double_integrator_QP_solver_noCD_lub04, double_integrator_QP_solver_noCD_sub04, double_integrator_QP_solver_noCD_riub04, double_integrator_QP_solver_noCD_llb04, double_integrator_QP_solver_noCD_slb04, double_integrator_QP_solver_noCD_rilb04, double_integrator_QP_solver_noCD_lbIdx04, double_integrator_QP_solver_noCD_ubIdx04, double_integrator_QP_solver_noCD_grad_ineq04, double_integrator_QP_solver_noCD_lubbysub04, double_integrator_QP_solver_noCD_llbbyslb04);
double_integrator_QP_solver_noCD_LA_INEQ_P_22_17(params->A5, double_integrator_QP_solver_noCD_lp04, double_integrator_QP_solver_noCD_sp04, double_integrator_QP_solver_noCD_rip04, double_integrator_QP_solver_noCD_grad_ineq04, double_integrator_QP_solver_noCD_lpbysp04);
double_integrator_QP_solver_noCD_LA_INEQ_B_GRAD_17_6_6(double_integrator_QP_solver_noCD_lub05, double_integrator_QP_solver_noCD_sub05, double_integrator_QP_solver_noCD_riub05, double_integrator_QP_solver_noCD_llb05, double_integrator_QP_solver_noCD_slb05, double_integrator_QP_solver_noCD_rilb05, double_integrator_QP_solver_noCD_lbIdx05, double_integrator_QP_solver_noCD_ubIdx05, double_integrator_QP_solver_noCD_grad_ineq05, double_integrator_QP_solver_noCD_lubbysub05, double_integrator_QP_solver_noCD_llbbyslb05);
double_integrator_QP_solver_noCD_LA_INEQ_P_22_17(params->A6, double_integrator_QP_solver_noCD_lp05, double_integrator_QP_solver_noCD_sp05, double_integrator_QP_solver_noCD_rip05, double_integrator_QP_solver_noCD_grad_ineq05, double_integrator_QP_solver_noCD_lpbysp05);
double_integrator_QP_solver_noCD_LA_INEQ_B_GRAD_17_6_6(double_integrator_QP_solver_noCD_lub06, double_integrator_QP_solver_noCD_sub06, double_integrator_QP_solver_noCD_riub06, double_integrator_QP_solver_noCD_llb06, double_integrator_QP_solver_noCD_slb06, double_integrator_QP_solver_noCD_rilb06, double_integrator_QP_solver_noCD_lbIdx06, double_integrator_QP_solver_noCD_ubIdx06, double_integrator_QP_solver_noCD_grad_ineq06, double_integrator_QP_solver_noCD_lubbysub06, double_integrator_QP_solver_noCD_llbbyslb06);
double_integrator_QP_solver_noCD_LA_INEQ_P_22_17(params->A7, double_integrator_QP_solver_noCD_lp06, double_integrator_QP_solver_noCD_sp06, double_integrator_QP_solver_noCD_rip06, double_integrator_QP_solver_noCD_grad_ineq06, double_integrator_QP_solver_noCD_lpbysp06);
double_integrator_QP_solver_noCD_LA_INEQ_B_GRAD_17_6_6(double_integrator_QP_solver_noCD_lub07, double_integrator_QP_solver_noCD_sub07, double_integrator_QP_solver_noCD_riub07, double_integrator_QP_solver_noCD_llb07, double_integrator_QP_solver_noCD_slb07, double_integrator_QP_solver_noCD_rilb07, double_integrator_QP_solver_noCD_lbIdx07, double_integrator_QP_solver_noCD_ubIdx07, double_integrator_QP_solver_noCD_grad_ineq07, double_integrator_QP_solver_noCD_lubbysub07, double_integrator_QP_solver_noCD_llbbyslb07);
double_integrator_QP_solver_noCD_LA_INEQ_P_22_17(params->A8, double_integrator_QP_solver_noCD_lp07, double_integrator_QP_solver_noCD_sp07, double_integrator_QP_solver_noCD_rip07, double_integrator_QP_solver_noCD_grad_ineq07, double_integrator_QP_solver_noCD_lpbysp07);
double_integrator_QP_solver_noCD_LA_INEQ_B_GRAD_17_6_6(double_integrator_QP_solver_noCD_lub08, double_integrator_QP_solver_noCD_sub08, double_integrator_QP_solver_noCD_riub08, double_integrator_QP_solver_noCD_llb08, double_integrator_QP_solver_noCD_slb08, double_integrator_QP_solver_noCD_rilb08, double_integrator_QP_solver_noCD_lbIdx08, double_integrator_QP_solver_noCD_ubIdx08, double_integrator_QP_solver_noCD_grad_ineq08, double_integrator_QP_solver_noCD_lubbysub08, double_integrator_QP_solver_noCD_llbbyslb08);
double_integrator_QP_solver_noCD_LA_INEQ_P_22_17(params->A9, double_integrator_QP_solver_noCD_lp08, double_integrator_QP_solver_noCD_sp08, double_integrator_QP_solver_noCD_rip08, double_integrator_QP_solver_noCD_grad_ineq08, double_integrator_QP_solver_noCD_lpbysp08);
double_integrator_QP_solver_noCD_LA_INEQ_B_GRAD_17_6_6(double_integrator_QP_solver_noCD_lub09, double_integrator_QP_solver_noCD_sub09, double_integrator_QP_solver_noCD_riub09, double_integrator_QP_solver_noCD_llb09, double_integrator_QP_solver_noCD_slb09, double_integrator_QP_solver_noCD_rilb09, double_integrator_QP_solver_noCD_lbIdx09, double_integrator_QP_solver_noCD_ubIdx09, double_integrator_QP_solver_noCD_grad_ineq09, double_integrator_QP_solver_noCD_lubbysub09, double_integrator_QP_solver_noCD_llbbyslb09);
double_integrator_QP_solver_noCD_LA_INEQ_P_22_17(params->A10, double_integrator_QP_solver_noCD_lp09, double_integrator_QP_solver_noCD_sp09, double_integrator_QP_solver_noCD_rip09, double_integrator_QP_solver_noCD_grad_ineq09, double_integrator_QP_solver_noCD_lpbysp09);
double_integrator_QP_solver_noCD_LA_INEQ_B_GRAD_15_6_6(double_integrator_QP_solver_noCD_lub10, double_integrator_QP_solver_noCD_sub10, double_integrator_QP_solver_noCD_riub10, double_integrator_QP_solver_noCD_llb10, double_integrator_QP_solver_noCD_slb10, double_integrator_QP_solver_noCD_rilb10, double_integrator_QP_solver_noCD_lbIdx10, double_integrator_QP_solver_noCD_ubIdx10, double_integrator_QP_solver_noCD_grad_ineq10, double_integrator_QP_solver_noCD_lubbysub10, double_integrator_QP_solver_noCD_llbbyslb10);
double_integrator_QP_solver_noCD_LA_INEQ_P_22_15(params->A11, double_integrator_QP_solver_noCD_lp10, double_integrator_QP_solver_noCD_sp10, double_integrator_QP_solver_noCD_rip10, double_integrator_QP_solver_noCD_grad_ineq10, double_integrator_QP_solver_noCD_lpbysp10);
double_integrator_QP_solver_noCD_LA_INEQ_B_GRAD_4_4_4(double_integrator_QP_solver_noCD_lub11, double_integrator_QP_solver_noCD_sub11, double_integrator_QP_solver_noCD_riub11, double_integrator_QP_solver_noCD_llb11, double_integrator_QP_solver_noCD_slb11, double_integrator_QP_solver_noCD_rilb11, double_integrator_QP_solver_noCD_lbIdx11, double_integrator_QP_solver_noCD_ubIdx11, double_integrator_QP_solver_noCD_grad_ineq11, double_integrator_QP_solver_noCD_lubbysub11, double_integrator_QP_solver_noCD_llbbyslb11);
double_integrator_QP_solver_noCD_LA_INEQ_P_8_4(params->A12, double_integrator_QP_solver_noCD_lp11, double_integrator_QP_solver_noCD_sp11, double_integrator_QP_solver_noCD_rip11, double_integrator_QP_solver_noCD_grad_ineq11, double_integrator_QP_solver_noCD_lpbysp11);
info->dobj = info->pobj - info->dgap;
info->rdgap = info->pobj ? info->dgap / info->pobj : 1e6;
if( info->rdgap < 0 ) info->rdgap = -info->rdgap;
if( info->mu < double_integrator_QP_solver_noCD_SET_ACC_KKTCOMPL
    && (info->rdgap < double_integrator_QP_solver_noCD_SET_ACC_RDGAP || info->dgap < double_integrator_QP_solver_noCD_SET_ACC_KKTCOMPL)
    && info->res_eq < double_integrator_QP_solver_noCD_SET_ACC_RESEQ
    && info->res_ineq < double_integrator_QP_solver_noCD_SET_ACC_RESINEQ ){
exitcode = double_integrator_QP_solver_noCD_OPTIMAL; break; }
if( info->it == double_integrator_QP_solver_noCD_SET_MAXIT ){
exitcode = double_integrator_QP_solver_noCD_MAXITREACHED; break; }
double_integrator_QP_solver_noCD_LA_VVADD3_189(double_integrator_QP_solver_noCD_grad_cost, double_integrator_QP_solver_noCD_grad_eq, double_integrator_QP_solver_noCD_grad_ineq, double_integrator_QP_solver_noCD_rd);
double_integrator_QP_solver_noCD_LA_INEQ_DENSE_DIAG_HESS_17_7_6(double_integrator_QP_solver_noCD_H00, double_integrator_QP_solver_noCD_llbbyslb00, double_integrator_QP_solver_noCD_lbIdx00, double_integrator_QP_solver_noCD_lubbysub00, double_integrator_QP_solver_noCD_ubIdx00, double_integrator_QP_solver_noCD_Phi00);
double_integrator_QP_solver_noCD_LA_DENSE_ADDMTDM_22_17(params->A1, double_integrator_QP_solver_noCD_lpbysp00, double_integrator_QP_solver_noCD_Phi00);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL2_17(double_integrator_QP_solver_noCD_Phi00);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi00, double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_V00);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi00, double_integrator_QP_solver_noCD_rd00, double_integrator_QP_solver_noCD_Lbyrd00);
double_integrator_QP_solver_noCD_LA_INEQ_DENSE_DIAG_HESS_17_6_6(double_integrator_QP_solver_noCD_H00, double_integrator_QP_solver_noCD_llbbyslb01, double_integrator_QP_solver_noCD_lbIdx01, double_integrator_QP_solver_noCD_lubbysub01, double_integrator_QP_solver_noCD_ubIdx01, double_integrator_QP_solver_noCD_Phi01);
double_integrator_QP_solver_noCD_LA_DENSE_ADDMTDM_22_17(params->A2, double_integrator_QP_solver_noCD_lpbysp01, double_integrator_QP_solver_noCD_Phi01);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL2_17(double_integrator_QP_solver_noCD_Phi01);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi01, double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_V01);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi01, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_W01);
double_integrator_QP_solver_noCD_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_noCD_W01, double_integrator_QP_solver_noCD_V01, double_integrator_QP_solver_noCD_Ysd01);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi01, double_integrator_QP_solver_noCD_rd01, double_integrator_QP_solver_noCD_Lbyrd01);
double_integrator_QP_solver_noCD_LA_INEQ_DENSE_DIAG_HESS_17_6_6(double_integrator_QP_solver_noCD_H00, double_integrator_QP_solver_noCD_llbbyslb02, double_integrator_QP_solver_noCD_lbIdx02, double_integrator_QP_solver_noCD_lubbysub02, double_integrator_QP_solver_noCD_ubIdx02, double_integrator_QP_solver_noCD_Phi02);
double_integrator_QP_solver_noCD_LA_DENSE_ADDMTDM_22_17(params->A3, double_integrator_QP_solver_noCD_lpbysp02, double_integrator_QP_solver_noCD_Phi02);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL2_17(double_integrator_QP_solver_noCD_Phi02);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi02, double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_V02);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi02, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_W02);
double_integrator_QP_solver_noCD_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_noCD_W02, double_integrator_QP_solver_noCD_V02, double_integrator_QP_solver_noCD_Ysd02);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi02, double_integrator_QP_solver_noCD_rd02, double_integrator_QP_solver_noCD_Lbyrd02);
double_integrator_QP_solver_noCD_LA_INEQ_DENSE_DIAG_HESS_17_6_6(double_integrator_QP_solver_noCD_H00, double_integrator_QP_solver_noCD_llbbyslb03, double_integrator_QP_solver_noCD_lbIdx03, double_integrator_QP_solver_noCD_lubbysub03, double_integrator_QP_solver_noCD_ubIdx03, double_integrator_QP_solver_noCD_Phi03);
double_integrator_QP_solver_noCD_LA_DENSE_ADDMTDM_22_17(params->A4, double_integrator_QP_solver_noCD_lpbysp03, double_integrator_QP_solver_noCD_Phi03);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL2_17(double_integrator_QP_solver_noCD_Phi03);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi03, double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_V03);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi03, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_W03);
double_integrator_QP_solver_noCD_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_noCD_W03, double_integrator_QP_solver_noCD_V03, double_integrator_QP_solver_noCD_Ysd03);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi03, double_integrator_QP_solver_noCD_rd03, double_integrator_QP_solver_noCD_Lbyrd03);
double_integrator_QP_solver_noCD_LA_INEQ_DENSE_DIAG_HESS_17_6_6(double_integrator_QP_solver_noCD_H00, double_integrator_QP_solver_noCD_llbbyslb04, double_integrator_QP_solver_noCD_lbIdx04, double_integrator_QP_solver_noCD_lubbysub04, double_integrator_QP_solver_noCD_ubIdx04, double_integrator_QP_solver_noCD_Phi04);
double_integrator_QP_solver_noCD_LA_DENSE_ADDMTDM_22_17(params->A5, double_integrator_QP_solver_noCD_lpbysp04, double_integrator_QP_solver_noCD_Phi04);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL2_17(double_integrator_QP_solver_noCD_Phi04);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi04, double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_V04);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi04, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_W04);
double_integrator_QP_solver_noCD_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_noCD_W04, double_integrator_QP_solver_noCD_V04, double_integrator_QP_solver_noCD_Ysd04);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi04, double_integrator_QP_solver_noCD_rd04, double_integrator_QP_solver_noCD_Lbyrd04);
double_integrator_QP_solver_noCD_LA_INEQ_DENSE_DIAG_HESS_17_6_6(double_integrator_QP_solver_noCD_H00, double_integrator_QP_solver_noCD_llbbyslb05, double_integrator_QP_solver_noCD_lbIdx05, double_integrator_QP_solver_noCD_lubbysub05, double_integrator_QP_solver_noCD_ubIdx05, double_integrator_QP_solver_noCD_Phi05);
double_integrator_QP_solver_noCD_LA_DENSE_ADDMTDM_22_17(params->A6, double_integrator_QP_solver_noCD_lpbysp05, double_integrator_QP_solver_noCD_Phi05);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL2_17(double_integrator_QP_solver_noCD_Phi05);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi05, double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_V05);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi05, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_W05);
double_integrator_QP_solver_noCD_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_noCD_W05, double_integrator_QP_solver_noCD_V05, double_integrator_QP_solver_noCD_Ysd05);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi05, double_integrator_QP_solver_noCD_rd05, double_integrator_QP_solver_noCD_Lbyrd05);
double_integrator_QP_solver_noCD_LA_INEQ_DENSE_DIAG_HESS_17_6_6(double_integrator_QP_solver_noCD_H00, double_integrator_QP_solver_noCD_llbbyslb06, double_integrator_QP_solver_noCD_lbIdx06, double_integrator_QP_solver_noCD_lubbysub06, double_integrator_QP_solver_noCD_ubIdx06, double_integrator_QP_solver_noCD_Phi06);
double_integrator_QP_solver_noCD_LA_DENSE_ADDMTDM_22_17(params->A7, double_integrator_QP_solver_noCD_lpbysp06, double_integrator_QP_solver_noCD_Phi06);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL2_17(double_integrator_QP_solver_noCD_Phi06);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi06, double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_V06);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi06, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_W06);
double_integrator_QP_solver_noCD_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_noCD_W06, double_integrator_QP_solver_noCD_V06, double_integrator_QP_solver_noCD_Ysd06);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi06, double_integrator_QP_solver_noCD_rd06, double_integrator_QP_solver_noCD_Lbyrd06);
double_integrator_QP_solver_noCD_LA_INEQ_DENSE_DIAG_HESS_17_6_6(double_integrator_QP_solver_noCD_H00, double_integrator_QP_solver_noCD_llbbyslb07, double_integrator_QP_solver_noCD_lbIdx07, double_integrator_QP_solver_noCD_lubbysub07, double_integrator_QP_solver_noCD_ubIdx07, double_integrator_QP_solver_noCD_Phi07);
double_integrator_QP_solver_noCD_LA_DENSE_ADDMTDM_22_17(params->A8, double_integrator_QP_solver_noCD_lpbysp07, double_integrator_QP_solver_noCD_Phi07);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL2_17(double_integrator_QP_solver_noCD_Phi07);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi07, double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_V07);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi07, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_W07);
double_integrator_QP_solver_noCD_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_noCD_W07, double_integrator_QP_solver_noCD_V07, double_integrator_QP_solver_noCD_Ysd07);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi07, double_integrator_QP_solver_noCD_rd07, double_integrator_QP_solver_noCD_Lbyrd07);
double_integrator_QP_solver_noCD_LA_INEQ_DENSE_DIAG_HESS_17_6_6(double_integrator_QP_solver_noCD_H00, double_integrator_QP_solver_noCD_llbbyslb08, double_integrator_QP_solver_noCD_lbIdx08, double_integrator_QP_solver_noCD_lubbysub08, double_integrator_QP_solver_noCD_ubIdx08, double_integrator_QP_solver_noCD_Phi08);
double_integrator_QP_solver_noCD_LA_DENSE_ADDMTDM_22_17(params->A9, double_integrator_QP_solver_noCD_lpbysp08, double_integrator_QP_solver_noCD_Phi08);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL2_17(double_integrator_QP_solver_noCD_Phi08);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi08, double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_V08);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi08, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_W08);
double_integrator_QP_solver_noCD_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_noCD_W08, double_integrator_QP_solver_noCD_V08, double_integrator_QP_solver_noCD_Ysd08);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi08, double_integrator_QP_solver_noCD_rd08, double_integrator_QP_solver_noCD_Lbyrd08);
double_integrator_QP_solver_noCD_LA_INEQ_DENSE_DIAG_HESS_17_6_6(double_integrator_QP_solver_noCD_H00, double_integrator_QP_solver_noCD_llbbyslb09, double_integrator_QP_solver_noCD_lbIdx09, double_integrator_QP_solver_noCD_lubbysub09, double_integrator_QP_solver_noCD_ubIdx09, double_integrator_QP_solver_noCD_Phi09);
double_integrator_QP_solver_noCD_LA_DENSE_ADDMTDM_22_17(params->A10, double_integrator_QP_solver_noCD_lpbysp09, double_integrator_QP_solver_noCD_Phi09);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL2_17(double_integrator_QP_solver_noCD_Phi09);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi09, double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_V09);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_noCD_Phi09, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_W09);
double_integrator_QP_solver_noCD_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_noCD_W09, double_integrator_QP_solver_noCD_V09, double_integrator_QP_solver_noCD_Ysd09);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi09, double_integrator_QP_solver_noCD_rd09, double_integrator_QP_solver_noCD_Lbyrd09);
double_integrator_QP_solver_noCD_LA_INEQ_DENSE_DIAG_HESS_15_6_6(double_integrator_QP_solver_noCD_H10, double_integrator_QP_solver_noCD_llbbyslb10, double_integrator_QP_solver_noCD_lbIdx10, double_integrator_QP_solver_noCD_lubbysub10, double_integrator_QP_solver_noCD_ubIdx10, double_integrator_QP_solver_noCD_Phi10);
double_integrator_QP_solver_noCD_LA_DENSE_ADDMTDM_22_15(params->A11, double_integrator_QP_solver_noCD_lpbysp10, double_integrator_QP_solver_noCD_Phi10);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL2_15(double_integrator_QP_solver_noCD_Phi10);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_4_15(double_integrator_QP_solver_noCD_Phi10, double_integrator_QP_solver_noCD_C10, double_integrator_QP_solver_noCD_V10);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXFORWARDSUB_7_15(double_integrator_QP_solver_noCD_Phi10, double_integrator_QP_solver_noCD_D10, double_integrator_QP_solver_noCD_W10);
double_integrator_QP_solver_noCD_LA_DENSE_MMTM_7_15_4(double_integrator_QP_solver_noCD_W10, double_integrator_QP_solver_noCD_V10, double_integrator_QP_solver_noCD_Ysd10);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_15(double_integrator_QP_solver_noCD_Phi10, double_integrator_QP_solver_noCD_rd10, double_integrator_QP_solver_noCD_Lbyrd10);
double_integrator_QP_solver_noCD_LA_INEQ_DENSE_DIAG_HESS_5_4_4(double_integrator_QP_solver_noCD_H11, double_integrator_QP_solver_noCD_llbbyslb11, double_integrator_QP_solver_noCD_lbIdx11, double_integrator_QP_solver_noCD_lubbysub11, double_integrator_QP_solver_noCD_ubIdx11, double_integrator_QP_solver_noCD_Phi11);
double_integrator_QP_solver_noCD_LA_DENSE_ADDMTDM_8_4(params->A12, double_integrator_QP_solver_noCD_lpbysp11, double_integrator_QP_solver_noCD_Phi11);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL2_4(double_integrator_QP_solver_noCD_Phi11);
double_integrator_QP_solver_noCD_LA_DENSE_DIAGZERO_MATRIXFORWARDSUB_4_4(double_integrator_QP_solver_noCD_Phi11, double_integrator_QP_solver_noCD_D11, double_integrator_QP_solver_noCD_W11);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_4(double_integrator_QP_solver_noCD_Phi11, double_integrator_QP_solver_noCD_rd11, double_integrator_QP_solver_noCD_Lbyrd11);
double_integrator_QP_solver_noCD_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_noCD_V00, double_integrator_QP_solver_noCD_W01, double_integrator_QP_solver_noCD_Yd00);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_noCD_V00, double_integrator_QP_solver_noCD_Lbyrd00, double_integrator_QP_solver_noCD_W01, double_integrator_QP_solver_noCD_Lbyrd01, double_integrator_QP_solver_noCD_re00, double_integrator_QP_solver_noCD_beta00);
double_integrator_QP_solver_noCD_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_noCD_V01, double_integrator_QP_solver_noCD_W02, double_integrator_QP_solver_noCD_Yd01);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_noCD_V01, double_integrator_QP_solver_noCD_Lbyrd01, double_integrator_QP_solver_noCD_W02, double_integrator_QP_solver_noCD_Lbyrd02, double_integrator_QP_solver_noCD_re01, double_integrator_QP_solver_noCD_beta01);
double_integrator_QP_solver_noCD_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_noCD_V02, double_integrator_QP_solver_noCD_W03, double_integrator_QP_solver_noCD_Yd02);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_noCD_V02, double_integrator_QP_solver_noCD_Lbyrd02, double_integrator_QP_solver_noCD_W03, double_integrator_QP_solver_noCD_Lbyrd03, double_integrator_QP_solver_noCD_re02, double_integrator_QP_solver_noCD_beta02);
double_integrator_QP_solver_noCD_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_noCD_V03, double_integrator_QP_solver_noCD_W04, double_integrator_QP_solver_noCD_Yd03);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_noCD_V03, double_integrator_QP_solver_noCD_Lbyrd03, double_integrator_QP_solver_noCD_W04, double_integrator_QP_solver_noCD_Lbyrd04, double_integrator_QP_solver_noCD_re03, double_integrator_QP_solver_noCD_beta03);
double_integrator_QP_solver_noCD_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_noCD_V04, double_integrator_QP_solver_noCD_W05, double_integrator_QP_solver_noCD_Yd04);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_noCD_V04, double_integrator_QP_solver_noCD_Lbyrd04, double_integrator_QP_solver_noCD_W05, double_integrator_QP_solver_noCD_Lbyrd05, double_integrator_QP_solver_noCD_re04, double_integrator_QP_solver_noCD_beta04);
double_integrator_QP_solver_noCD_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_noCD_V05, double_integrator_QP_solver_noCD_W06, double_integrator_QP_solver_noCD_Yd05);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_noCD_V05, double_integrator_QP_solver_noCD_Lbyrd05, double_integrator_QP_solver_noCD_W06, double_integrator_QP_solver_noCD_Lbyrd06, double_integrator_QP_solver_noCD_re05, double_integrator_QP_solver_noCD_beta05);
double_integrator_QP_solver_noCD_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_noCD_V06, double_integrator_QP_solver_noCD_W07, double_integrator_QP_solver_noCD_Yd06);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_noCD_V06, double_integrator_QP_solver_noCD_Lbyrd06, double_integrator_QP_solver_noCD_W07, double_integrator_QP_solver_noCD_Lbyrd07, double_integrator_QP_solver_noCD_re06, double_integrator_QP_solver_noCD_beta06);
double_integrator_QP_solver_noCD_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_noCD_V07, double_integrator_QP_solver_noCD_W08, double_integrator_QP_solver_noCD_Yd07);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_noCD_V07, double_integrator_QP_solver_noCD_Lbyrd07, double_integrator_QP_solver_noCD_W08, double_integrator_QP_solver_noCD_Lbyrd08, double_integrator_QP_solver_noCD_re07, double_integrator_QP_solver_noCD_beta07);
double_integrator_QP_solver_noCD_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_noCD_V08, double_integrator_QP_solver_noCD_W09, double_integrator_QP_solver_noCD_Yd08);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_noCD_V08, double_integrator_QP_solver_noCD_Lbyrd08, double_integrator_QP_solver_noCD_W09, double_integrator_QP_solver_noCD_Lbyrd09, double_integrator_QP_solver_noCD_re08, double_integrator_QP_solver_noCD_beta08);
double_integrator_QP_solver_noCD_LA_DENSE_MMT2_7_17_15(double_integrator_QP_solver_noCD_V09, double_integrator_QP_solver_noCD_W10, double_integrator_QP_solver_noCD_Yd09);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB2_7_17_15(double_integrator_QP_solver_noCD_V09, double_integrator_QP_solver_noCD_Lbyrd09, double_integrator_QP_solver_noCD_W10, double_integrator_QP_solver_noCD_Lbyrd10, double_integrator_QP_solver_noCD_re09, double_integrator_QP_solver_noCD_beta09);
double_integrator_QP_solver_noCD_LA_DENSE_MMT2_4_15_4(double_integrator_QP_solver_noCD_V10, double_integrator_QP_solver_noCD_W11, double_integrator_QP_solver_noCD_Yd10);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB2_4_15_4(double_integrator_QP_solver_noCD_V10, double_integrator_QP_solver_noCD_Lbyrd10, double_integrator_QP_solver_noCD_W11, double_integrator_QP_solver_noCD_Lbyrd11, double_integrator_QP_solver_noCD_re10, double_integrator_QP_solver_noCD_beta10);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL_7(double_integrator_QP_solver_noCD_Yd00, double_integrator_QP_solver_noCD_Ld00);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld00, double_integrator_QP_solver_noCD_beta00, double_integrator_QP_solver_noCD_yy00);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_noCD_Ld00, double_integrator_QP_solver_noCD_Ysd01, double_integrator_QP_solver_noCD_Lsd01);
double_integrator_QP_solver_noCD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_noCD_Lsd01, double_integrator_QP_solver_noCD_Yd01);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL_7(double_integrator_QP_solver_noCD_Yd01, double_integrator_QP_solver_noCD_Ld01);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd01, double_integrator_QP_solver_noCD_yy00, double_integrator_QP_solver_noCD_beta01, double_integrator_QP_solver_noCD_bmy01);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld01, double_integrator_QP_solver_noCD_bmy01, double_integrator_QP_solver_noCD_yy01);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_noCD_Ld01, double_integrator_QP_solver_noCD_Ysd02, double_integrator_QP_solver_noCD_Lsd02);
double_integrator_QP_solver_noCD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_noCD_Lsd02, double_integrator_QP_solver_noCD_Yd02);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL_7(double_integrator_QP_solver_noCD_Yd02, double_integrator_QP_solver_noCD_Ld02);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd02, double_integrator_QP_solver_noCD_yy01, double_integrator_QP_solver_noCD_beta02, double_integrator_QP_solver_noCD_bmy02);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld02, double_integrator_QP_solver_noCD_bmy02, double_integrator_QP_solver_noCD_yy02);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_noCD_Ld02, double_integrator_QP_solver_noCD_Ysd03, double_integrator_QP_solver_noCD_Lsd03);
double_integrator_QP_solver_noCD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_noCD_Lsd03, double_integrator_QP_solver_noCD_Yd03);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL_7(double_integrator_QP_solver_noCD_Yd03, double_integrator_QP_solver_noCD_Ld03);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd03, double_integrator_QP_solver_noCD_yy02, double_integrator_QP_solver_noCD_beta03, double_integrator_QP_solver_noCD_bmy03);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld03, double_integrator_QP_solver_noCD_bmy03, double_integrator_QP_solver_noCD_yy03);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_noCD_Ld03, double_integrator_QP_solver_noCD_Ysd04, double_integrator_QP_solver_noCD_Lsd04);
double_integrator_QP_solver_noCD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_noCD_Lsd04, double_integrator_QP_solver_noCD_Yd04);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL_7(double_integrator_QP_solver_noCD_Yd04, double_integrator_QP_solver_noCD_Ld04);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd04, double_integrator_QP_solver_noCD_yy03, double_integrator_QP_solver_noCD_beta04, double_integrator_QP_solver_noCD_bmy04);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld04, double_integrator_QP_solver_noCD_bmy04, double_integrator_QP_solver_noCD_yy04);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_noCD_Ld04, double_integrator_QP_solver_noCD_Ysd05, double_integrator_QP_solver_noCD_Lsd05);
double_integrator_QP_solver_noCD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_noCD_Lsd05, double_integrator_QP_solver_noCD_Yd05);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL_7(double_integrator_QP_solver_noCD_Yd05, double_integrator_QP_solver_noCD_Ld05);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd05, double_integrator_QP_solver_noCD_yy04, double_integrator_QP_solver_noCD_beta05, double_integrator_QP_solver_noCD_bmy05);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld05, double_integrator_QP_solver_noCD_bmy05, double_integrator_QP_solver_noCD_yy05);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_noCD_Ld05, double_integrator_QP_solver_noCD_Ysd06, double_integrator_QP_solver_noCD_Lsd06);
double_integrator_QP_solver_noCD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_noCD_Lsd06, double_integrator_QP_solver_noCD_Yd06);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL_7(double_integrator_QP_solver_noCD_Yd06, double_integrator_QP_solver_noCD_Ld06);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd06, double_integrator_QP_solver_noCD_yy05, double_integrator_QP_solver_noCD_beta06, double_integrator_QP_solver_noCD_bmy06);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld06, double_integrator_QP_solver_noCD_bmy06, double_integrator_QP_solver_noCD_yy06);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_noCD_Ld06, double_integrator_QP_solver_noCD_Ysd07, double_integrator_QP_solver_noCD_Lsd07);
double_integrator_QP_solver_noCD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_noCD_Lsd07, double_integrator_QP_solver_noCD_Yd07);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL_7(double_integrator_QP_solver_noCD_Yd07, double_integrator_QP_solver_noCD_Ld07);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd07, double_integrator_QP_solver_noCD_yy06, double_integrator_QP_solver_noCD_beta07, double_integrator_QP_solver_noCD_bmy07);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld07, double_integrator_QP_solver_noCD_bmy07, double_integrator_QP_solver_noCD_yy07);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_noCD_Ld07, double_integrator_QP_solver_noCD_Ysd08, double_integrator_QP_solver_noCD_Lsd08);
double_integrator_QP_solver_noCD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_noCD_Lsd08, double_integrator_QP_solver_noCD_Yd08);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL_7(double_integrator_QP_solver_noCD_Yd08, double_integrator_QP_solver_noCD_Ld08);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd08, double_integrator_QP_solver_noCD_yy07, double_integrator_QP_solver_noCD_beta08, double_integrator_QP_solver_noCD_bmy08);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld08, double_integrator_QP_solver_noCD_bmy08, double_integrator_QP_solver_noCD_yy08);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_noCD_Ld08, double_integrator_QP_solver_noCD_Ysd09, double_integrator_QP_solver_noCD_Lsd09);
double_integrator_QP_solver_noCD_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_noCD_Lsd09, double_integrator_QP_solver_noCD_Yd09);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL_7(double_integrator_QP_solver_noCD_Yd09, double_integrator_QP_solver_noCD_Ld09);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd09, double_integrator_QP_solver_noCD_yy08, double_integrator_QP_solver_noCD_beta09, double_integrator_QP_solver_noCD_bmy09);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld09, double_integrator_QP_solver_noCD_bmy09, double_integrator_QP_solver_noCD_yy09);
double_integrator_QP_solver_noCD_LA_DENSE_MATRIXTFORWARDSUB_4_7(double_integrator_QP_solver_noCD_Ld09, double_integrator_QP_solver_noCD_Ysd10, double_integrator_QP_solver_noCD_Lsd10);
double_integrator_QP_solver_noCD_LA_DENSE_MMTSUB_4_7(double_integrator_QP_solver_noCD_Lsd10, double_integrator_QP_solver_noCD_Yd10);
double_integrator_QP_solver_noCD_LA_DENSE_CHOL_4(double_integrator_QP_solver_noCD_Yd10, double_integrator_QP_solver_noCD_Ld10);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_4_7(double_integrator_QP_solver_noCD_Lsd10, double_integrator_QP_solver_noCD_yy09, double_integrator_QP_solver_noCD_beta10, double_integrator_QP_solver_noCD_bmy10);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_4(double_integrator_QP_solver_noCD_Ld10, double_integrator_QP_solver_noCD_bmy10, double_integrator_QP_solver_noCD_yy10);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_4(double_integrator_QP_solver_noCD_Ld10, double_integrator_QP_solver_noCD_yy10, double_integrator_QP_solver_noCD_dvaff10);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_4_7(double_integrator_QP_solver_noCD_Lsd10, double_integrator_QP_solver_noCD_dvaff10, double_integrator_QP_solver_noCD_yy09, double_integrator_QP_solver_noCD_bmy09);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld09, double_integrator_QP_solver_noCD_bmy09, double_integrator_QP_solver_noCD_dvaff09);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd09, double_integrator_QP_solver_noCD_dvaff09, double_integrator_QP_solver_noCD_yy08, double_integrator_QP_solver_noCD_bmy08);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld08, double_integrator_QP_solver_noCD_bmy08, double_integrator_QP_solver_noCD_dvaff08);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd08, double_integrator_QP_solver_noCD_dvaff08, double_integrator_QP_solver_noCD_yy07, double_integrator_QP_solver_noCD_bmy07);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld07, double_integrator_QP_solver_noCD_bmy07, double_integrator_QP_solver_noCD_dvaff07);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd07, double_integrator_QP_solver_noCD_dvaff07, double_integrator_QP_solver_noCD_yy06, double_integrator_QP_solver_noCD_bmy06);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld06, double_integrator_QP_solver_noCD_bmy06, double_integrator_QP_solver_noCD_dvaff06);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd06, double_integrator_QP_solver_noCD_dvaff06, double_integrator_QP_solver_noCD_yy05, double_integrator_QP_solver_noCD_bmy05);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld05, double_integrator_QP_solver_noCD_bmy05, double_integrator_QP_solver_noCD_dvaff05);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd05, double_integrator_QP_solver_noCD_dvaff05, double_integrator_QP_solver_noCD_yy04, double_integrator_QP_solver_noCD_bmy04);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld04, double_integrator_QP_solver_noCD_bmy04, double_integrator_QP_solver_noCD_dvaff04);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd04, double_integrator_QP_solver_noCD_dvaff04, double_integrator_QP_solver_noCD_yy03, double_integrator_QP_solver_noCD_bmy03);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld03, double_integrator_QP_solver_noCD_bmy03, double_integrator_QP_solver_noCD_dvaff03);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd03, double_integrator_QP_solver_noCD_dvaff03, double_integrator_QP_solver_noCD_yy02, double_integrator_QP_solver_noCD_bmy02);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld02, double_integrator_QP_solver_noCD_bmy02, double_integrator_QP_solver_noCD_dvaff02);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd02, double_integrator_QP_solver_noCD_dvaff02, double_integrator_QP_solver_noCD_yy01, double_integrator_QP_solver_noCD_bmy01);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld01, double_integrator_QP_solver_noCD_bmy01, double_integrator_QP_solver_noCD_dvaff01);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd01, double_integrator_QP_solver_noCD_dvaff01, double_integrator_QP_solver_noCD_yy00, double_integrator_QP_solver_noCD_bmy00);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld00, double_integrator_QP_solver_noCD_bmy00, double_integrator_QP_solver_noCD_dvaff00);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM_7_17(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvaff00, double_integrator_QP_solver_noCD_grad_eq00);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvaff01, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvaff00, double_integrator_QP_solver_noCD_grad_eq01);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvaff02, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvaff01, double_integrator_QP_solver_noCD_grad_eq02);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvaff03, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvaff02, double_integrator_QP_solver_noCD_grad_eq03);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvaff04, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvaff03, double_integrator_QP_solver_noCD_grad_eq04);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvaff05, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvaff04, double_integrator_QP_solver_noCD_grad_eq05);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvaff06, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvaff05, double_integrator_QP_solver_noCD_grad_eq06);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvaff07, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvaff06, double_integrator_QP_solver_noCD_grad_eq07);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvaff08, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvaff07, double_integrator_QP_solver_noCD_grad_eq08);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvaff09, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvaff08, double_integrator_QP_solver_noCD_grad_eq09);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_4_15_7(double_integrator_QP_solver_noCD_C10, double_integrator_QP_solver_noCD_dvaff10, double_integrator_QP_solver_noCD_D10, double_integrator_QP_solver_noCD_dvaff09, double_integrator_QP_solver_noCD_grad_eq10);
double_integrator_QP_solver_noCD_LA_DIAGZERO_MTVM_4_4(double_integrator_QP_solver_noCD_D11, double_integrator_QP_solver_noCD_dvaff10, double_integrator_QP_solver_noCD_grad_eq11);
double_integrator_QP_solver_noCD_LA_VSUB2_189(double_integrator_QP_solver_noCD_rd, double_integrator_QP_solver_noCD_grad_eq, double_integrator_QP_solver_noCD_rd);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi00, double_integrator_QP_solver_noCD_rd00, double_integrator_QP_solver_noCD_dzaff00);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi01, double_integrator_QP_solver_noCD_rd01, double_integrator_QP_solver_noCD_dzaff01);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi02, double_integrator_QP_solver_noCD_rd02, double_integrator_QP_solver_noCD_dzaff02);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi03, double_integrator_QP_solver_noCD_rd03, double_integrator_QP_solver_noCD_dzaff03);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi04, double_integrator_QP_solver_noCD_rd04, double_integrator_QP_solver_noCD_dzaff04);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi05, double_integrator_QP_solver_noCD_rd05, double_integrator_QP_solver_noCD_dzaff05);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi06, double_integrator_QP_solver_noCD_rd06, double_integrator_QP_solver_noCD_dzaff06);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi07, double_integrator_QP_solver_noCD_rd07, double_integrator_QP_solver_noCD_dzaff07);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi08, double_integrator_QP_solver_noCD_rd08, double_integrator_QP_solver_noCD_dzaff08);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi09, double_integrator_QP_solver_noCD_rd09, double_integrator_QP_solver_noCD_dzaff09);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_noCD_Phi10, double_integrator_QP_solver_noCD_rd10, double_integrator_QP_solver_noCD_dzaff10);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_4(double_integrator_QP_solver_noCD_Phi11, double_integrator_QP_solver_noCD_rd11, double_integrator_QP_solver_noCD_dzaff11);
double_integrator_QP_solver_noCD_LA_VSUB_INDEXED_7(double_integrator_QP_solver_noCD_dzaff00, double_integrator_QP_solver_noCD_lbIdx00, double_integrator_QP_solver_noCD_rilb00, double_integrator_QP_solver_noCD_dslbaff00);
double_integrator_QP_solver_noCD_LA_VSUB3_7(double_integrator_QP_solver_noCD_llbbyslb00, double_integrator_QP_solver_noCD_dslbaff00, double_integrator_QP_solver_noCD_llb00, double_integrator_QP_solver_noCD_dllbaff00);
double_integrator_QP_solver_noCD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_noCD_riub00, double_integrator_QP_solver_noCD_dzaff00, double_integrator_QP_solver_noCD_ubIdx00, double_integrator_QP_solver_noCD_dsubaff00);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_lubbysub00, double_integrator_QP_solver_noCD_dsubaff00, double_integrator_QP_solver_noCD_lub00, double_integrator_QP_solver_noCD_dlubaff00);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB4_22_17(params->A1, double_integrator_QP_solver_noCD_dzaff00, double_integrator_QP_solver_noCD_rip00, double_integrator_QP_solver_noCD_dsp_aff00);
double_integrator_QP_solver_noCD_LA_VSUB3_22(double_integrator_QP_solver_noCD_lpbysp00, double_integrator_QP_solver_noCD_dsp_aff00, double_integrator_QP_solver_noCD_lp00, double_integrator_QP_solver_noCD_dlp_aff00);
double_integrator_QP_solver_noCD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_noCD_dzaff01, double_integrator_QP_solver_noCD_lbIdx01, double_integrator_QP_solver_noCD_rilb01, double_integrator_QP_solver_noCD_dslbaff01);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_llbbyslb01, double_integrator_QP_solver_noCD_dslbaff01, double_integrator_QP_solver_noCD_llb01, double_integrator_QP_solver_noCD_dllbaff01);
double_integrator_QP_solver_noCD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_noCD_riub01, double_integrator_QP_solver_noCD_dzaff01, double_integrator_QP_solver_noCD_ubIdx01, double_integrator_QP_solver_noCD_dsubaff01);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_lubbysub01, double_integrator_QP_solver_noCD_dsubaff01, double_integrator_QP_solver_noCD_lub01, double_integrator_QP_solver_noCD_dlubaff01);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB4_22_17(params->A2, double_integrator_QP_solver_noCD_dzaff01, double_integrator_QP_solver_noCD_rip01, double_integrator_QP_solver_noCD_dsp_aff01);
double_integrator_QP_solver_noCD_LA_VSUB3_22(double_integrator_QP_solver_noCD_lpbysp01, double_integrator_QP_solver_noCD_dsp_aff01, double_integrator_QP_solver_noCD_lp01, double_integrator_QP_solver_noCD_dlp_aff01);
double_integrator_QP_solver_noCD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_noCD_dzaff02, double_integrator_QP_solver_noCD_lbIdx02, double_integrator_QP_solver_noCD_rilb02, double_integrator_QP_solver_noCD_dslbaff02);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_llbbyslb02, double_integrator_QP_solver_noCD_dslbaff02, double_integrator_QP_solver_noCD_llb02, double_integrator_QP_solver_noCD_dllbaff02);
double_integrator_QP_solver_noCD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_noCD_riub02, double_integrator_QP_solver_noCD_dzaff02, double_integrator_QP_solver_noCD_ubIdx02, double_integrator_QP_solver_noCD_dsubaff02);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_lubbysub02, double_integrator_QP_solver_noCD_dsubaff02, double_integrator_QP_solver_noCD_lub02, double_integrator_QP_solver_noCD_dlubaff02);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB4_22_17(params->A3, double_integrator_QP_solver_noCD_dzaff02, double_integrator_QP_solver_noCD_rip02, double_integrator_QP_solver_noCD_dsp_aff02);
double_integrator_QP_solver_noCD_LA_VSUB3_22(double_integrator_QP_solver_noCD_lpbysp02, double_integrator_QP_solver_noCD_dsp_aff02, double_integrator_QP_solver_noCD_lp02, double_integrator_QP_solver_noCD_dlp_aff02);
double_integrator_QP_solver_noCD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_noCD_dzaff03, double_integrator_QP_solver_noCD_lbIdx03, double_integrator_QP_solver_noCD_rilb03, double_integrator_QP_solver_noCD_dslbaff03);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_llbbyslb03, double_integrator_QP_solver_noCD_dslbaff03, double_integrator_QP_solver_noCD_llb03, double_integrator_QP_solver_noCD_dllbaff03);
double_integrator_QP_solver_noCD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_noCD_riub03, double_integrator_QP_solver_noCD_dzaff03, double_integrator_QP_solver_noCD_ubIdx03, double_integrator_QP_solver_noCD_dsubaff03);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_lubbysub03, double_integrator_QP_solver_noCD_dsubaff03, double_integrator_QP_solver_noCD_lub03, double_integrator_QP_solver_noCD_dlubaff03);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB4_22_17(params->A4, double_integrator_QP_solver_noCD_dzaff03, double_integrator_QP_solver_noCD_rip03, double_integrator_QP_solver_noCD_dsp_aff03);
double_integrator_QP_solver_noCD_LA_VSUB3_22(double_integrator_QP_solver_noCD_lpbysp03, double_integrator_QP_solver_noCD_dsp_aff03, double_integrator_QP_solver_noCD_lp03, double_integrator_QP_solver_noCD_dlp_aff03);
double_integrator_QP_solver_noCD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_noCD_dzaff04, double_integrator_QP_solver_noCD_lbIdx04, double_integrator_QP_solver_noCD_rilb04, double_integrator_QP_solver_noCD_dslbaff04);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_llbbyslb04, double_integrator_QP_solver_noCD_dslbaff04, double_integrator_QP_solver_noCD_llb04, double_integrator_QP_solver_noCD_dllbaff04);
double_integrator_QP_solver_noCD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_noCD_riub04, double_integrator_QP_solver_noCD_dzaff04, double_integrator_QP_solver_noCD_ubIdx04, double_integrator_QP_solver_noCD_dsubaff04);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_lubbysub04, double_integrator_QP_solver_noCD_dsubaff04, double_integrator_QP_solver_noCD_lub04, double_integrator_QP_solver_noCD_dlubaff04);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB4_22_17(params->A5, double_integrator_QP_solver_noCD_dzaff04, double_integrator_QP_solver_noCD_rip04, double_integrator_QP_solver_noCD_dsp_aff04);
double_integrator_QP_solver_noCD_LA_VSUB3_22(double_integrator_QP_solver_noCD_lpbysp04, double_integrator_QP_solver_noCD_dsp_aff04, double_integrator_QP_solver_noCD_lp04, double_integrator_QP_solver_noCD_dlp_aff04);
double_integrator_QP_solver_noCD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_noCD_dzaff05, double_integrator_QP_solver_noCD_lbIdx05, double_integrator_QP_solver_noCD_rilb05, double_integrator_QP_solver_noCD_dslbaff05);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_llbbyslb05, double_integrator_QP_solver_noCD_dslbaff05, double_integrator_QP_solver_noCD_llb05, double_integrator_QP_solver_noCD_dllbaff05);
double_integrator_QP_solver_noCD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_noCD_riub05, double_integrator_QP_solver_noCD_dzaff05, double_integrator_QP_solver_noCD_ubIdx05, double_integrator_QP_solver_noCD_dsubaff05);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_lubbysub05, double_integrator_QP_solver_noCD_dsubaff05, double_integrator_QP_solver_noCD_lub05, double_integrator_QP_solver_noCD_dlubaff05);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB4_22_17(params->A6, double_integrator_QP_solver_noCD_dzaff05, double_integrator_QP_solver_noCD_rip05, double_integrator_QP_solver_noCD_dsp_aff05);
double_integrator_QP_solver_noCD_LA_VSUB3_22(double_integrator_QP_solver_noCD_lpbysp05, double_integrator_QP_solver_noCD_dsp_aff05, double_integrator_QP_solver_noCD_lp05, double_integrator_QP_solver_noCD_dlp_aff05);
double_integrator_QP_solver_noCD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_noCD_dzaff06, double_integrator_QP_solver_noCD_lbIdx06, double_integrator_QP_solver_noCD_rilb06, double_integrator_QP_solver_noCD_dslbaff06);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_llbbyslb06, double_integrator_QP_solver_noCD_dslbaff06, double_integrator_QP_solver_noCD_llb06, double_integrator_QP_solver_noCD_dllbaff06);
double_integrator_QP_solver_noCD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_noCD_riub06, double_integrator_QP_solver_noCD_dzaff06, double_integrator_QP_solver_noCD_ubIdx06, double_integrator_QP_solver_noCD_dsubaff06);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_lubbysub06, double_integrator_QP_solver_noCD_dsubaff06, double_integrator_QP_solver_noCD_lub06, double_integrator_QP_solver_noCD_dlubaff06);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB4_22_17(params->A7, double_integrator_QP_solver_noCD_dzaff06, double_integrator_QP_solver_noCD_rip06, double_integrator_QP_solver_noCD_dsp_aff06);
double_integrator_QP_solver_noCD_LA_VSUB3_22(double_integrator_QP_solver_noCD_lpbysp06, double_integrator_QP_solver_noCD_dsp_aff06, double_integrator_QP_solver_noCD_lp06, double_integrator_QP_solver_noCD_dlp_aff06);
double_integrator_QP_solver_noCD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_noCD_dzaff07, double_integrator_QP_solver_noCD_lbIdx07, double_integrator_QP_solver_noCD_rilb07, double_integrator_QP_solver_noCD_dslbaff07);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_llbbyslb07, double_integrator_QP_solver_noCD_dslbaff07, double_integrator_QP_solver_noCD_llb07, double_integrator_QP_solver_noCD_dllbaff07);
double_integrator_QP_solver_noCD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_noCD_riub07, double_integrator_QP_solver_noCD_dzaff07, double_integrator_QP_solver_noCD_ubIdx07, double_integrator_QP_solver_noCD_dsubaff07);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_lubbysub07, double_integrator_QP_solver_noCD_dsubaff07, double_integrator_QP_solver_noCD_lub07, double_integrator_QP_solver_noCD_dlubaff07);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB4_22_17(params->A8, double_integrator_QP_solver_noCD_dzaff07, double_integrator_QP_solver_noCD_rip07, double_integrator_QP_solver_noCD_dsp_aff07);
double_integrator_QP_solver_noCD_LA_VSUB3_22(double_integrator_QP_solver_noCD_lpbysp07, double_integrator_QP_solver_noCD_dsp_aff07, double_integrator_QP_solver_noCD_lp07, double_integrator_QP_solver_noCD_dlp_aff07);
double_integrator_QP_solver_noCD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_noCD_dzaff08, double_integrator_QP_solver_noCD_lbIdx08, double_integrator_QP_solver_noCD_rilb08, double_integrator_QP_solver_noCD_dslbaff08);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_llbbyslb08, double_integrator_QP_solver_noCD_dslbaff08, double_integrator_QP_solver_noCD_llb08, double_integrator_QP_solver_noCD_dllbaff08);
double_integrator_QP_solver_noCD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_noCD_riub08, double_integrator_QP_solver_noCD_dzaff08, double_integrator_QP_solver_noCD_ubIdx08, double_integrator_QP_solver_noCD_dsubaff08);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_lubbysub08, double_integrator_QP_solver_noCD_dsubaff08, double_integrator_QP_solver_noCD_lub08, double_integrator_QP_solver_noCD_dlubaff08);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB4_22_17(params->A9, double_integrator_QP_solver_noCD_dzaff08, double_integrator_QP_solver_noCD_rip08, double_integrator_QP_solver_noCD_dsp_aff08);
double_integrator_QP_solver_noCD_LA_VSUB3_22(double_integrator_QP_solver_noCD_lpbysp08, double_integrator_QP_solver_noCD_dsp_aff08, double_integrator_QP_solver_noCD_lp08, double_integrator_QP_solver_noCD_dlp_aff08);
double_integrator_QP_solver_noCD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_noCD_dzaff09, double_integrator_QP_solver_noCD_lbIdx09, double_integrator_QP_solver_noCD_rilb09, double_integrator_QP_solver_noCD_dslbaff09);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_llbbyslb09, double_integrator_QP_solver_noCD_dslbaff09, double_integrator_QP_solver_noCD_llb09, double_integrator_QP_solver_noCD_dllbaff09);
double_integrator_QP_solver_noCD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_noCD_riub09, double_integrator_QP_solver_noCD_dzaff09, double_integrator_QP_solver_noCD_ubIdx09, double_integrator_QP_solver_noCD_dsubaff09);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_lubbysub09, double_integrator_QP_solver_noCD_dsubaff09, double_integrator_QP_solver_noCD_lub09, double_integrator_QP_solver_noCD_dlubaff09);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB4_22_17(params->A10, double_integrator_QP_solver_noCD_dzaff09, double_integrator_QP_solver_noCD_rip09, double_integrator_QP_solver_noCD_dsp_aff09);
double_integrator_QP_solver_noCD_LA_VSUB3_22(double_integrator_QP_solver_noCD_lpbysp09, double_integrator_QP_solver_noCD_dsp_aff09, double_integrator_QP_solver_noCD_lp09, double_integrator_QP_solver_noCD_dlp_aff09);
double_integrator_QP_solver_noCD_LA_VSUB_INDEXED_6(double_integrator_QP_solver_noCD_dzaff10, double_integrator_QP_solver_noCD_lbIdx10, double_integrator_QP_solver_noCD_rilb10, double_integrator_QP_solver_noCD_dslbaff10);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_llbbyslb10, double_integrator_QP_solver_noCD_dslbaff10, double_integrator_QP_solver_noCD_llb10, double_integrator_QP_solver_noCD_dllbaff10);
double_integrator_QP_solver_noCD_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_noCD_riub10, double_integrator_QP_solver_noCD_dzaff10, double_integrator_QP_solver_noCD_ubIdx10, double_integrator_QP_solver_noCD_dsubaff10);
double_integrator_QP_solver_noCD_LA_VSUB3_6(double_integrator_QP_solver_noCD_lubbysub10, double_integrator_QP_solver_noCD_dsubaff10, double_integrator_QP_solver_noCD_lub10, double_integrator_QP_solver_noCD_dlubaff10);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB4_22_15(params->A11, double_integrator_QP_solver_noCD_dzaff10, double_integrator_QP_solver_noCD_rip10, double_integrator_QP_solver_noCD_dsp_aff10);
double_integrator_QP_solver_noCD_LA_VSUB3_22(double_integrator_QP_solver_noCD_lpbysp10, double_integrator_QP_solver_noCD_dsp_aff10, double_integrator_QP_solver_noCD_lp10, double_integrator_QP_solver_noCD_dlp_aff10);
double_integrator_QP_solver_noCD_LA_VSUB_INDEXED_4(double_integrator_QP_solver_noCD_dzaff11, double_integrator_QP_solver_noCD_lbIdx11, double_integrator_QP_solver_noCD_rilb11, double_integrator_QP_solver_noCD_dslbaff11);
double_integrator_QP_solver_noCD_LA_VSUB3_4(double_integrator_QP_solver_noCD_llbbyslb11, double_integrator_QP_solver_noCD_dslbaff11, double_integrator_QP_solver_noCD_llb11, double_integrator_QP_solver_noCD_dllbaff11);
double_integrator_QP_solver_noCD_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_noCD_riub11, double_integrator_QP_solver_noCD_dzaff11, double_integrator_QP_solver_noCD_ubIdx11, double_integrator_QP_solver_noCD_dsubaff11);
double_integrator_QP_solver_noCD_LA_VSUB3_4(double_integrator_QP_solver_noCD_lubbysub11, double_integrator_QP_solver_noCD_dsubaff11, double_integrator_QP_solver_noCD_lub11, double_integrator_QP_solver_noCD_dlubaff11);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB4_8_4(params->A12, double_integrator_QP_solver_noCD_dzaff11, double_integrator_QP_solver_noCD_rip11, double_integrator_QP_solver_noCD_dsp_aff11);
double_integrator_QP_solver_noCD_LA_VSUB3_8(double_integrator_QP_solver_noCD_lpbysp11, double_integrator_QP_solver_noCD_dsp_aff11, double_integrator_QP_solver_noCD_lp11, double_integrator_QP_solver_noCD_dlp_aff11);
info->lsit_aff = double_integrator_QP_solver_noCD_LINESEARCH_BACKTRACKING_AFFINE(double_integrator_QP_solver_noCD_l, double_integrator_QP_solver_noCD_s, double_integrator_QP_solver_noCD_dl_aff, double_integrator_QP_solver_noCD_ds_aff, &info->step_aff, &info->mu_aff);
if( info->lsit_aff == double_integrator_QP_solver_noCD_NOPROGRESS ){
exitcode = double_integrator_QP_solver_noCD_NOPROGRESS; break;
}
sigma_3rdroot = info->mu_aff / info->mu;
info->sigma = sigma_3rdroot*sigma_3rdroot*sigma_3rdroot;
musigma = info->mu * info->sigma;
double_integrator_QP_solver_noCD_LA_VSUB5_391(double_integrator_QP_solver_noCD_ds_aff, double_integrator_QP_solver_noCD_dl_aff, info->mu, info->sigma, double_integrator_QP_solver_noCD_ccrhs);
double_integrator_QP_solver_noCD_LA_VSUB6_INDEXED_17_6_7(double_integrator_QP_solver_noCD_ccrhsub00, double_integrator_QP_solver_noCD_sub00, double_integrator_QP_solver_noCD_ubIdx00, double_integrator_QP_solver_noCD_ccrhsl00, double_integrator_QP_solver_noCD_slb00, double_integrator_QP_solver_noCD_lbIdx00, double_integrator_QP_solver_noCD_rd00);
double_integrator_QP_solver_noCD_LA_VSUB6_INDEXED_17_6_6(double_integrator_QP_solver_noCD_ccrhsub01, double_integrator_QP_solver_noCD_sub01, double_integrator_QP_solver_noCD_ubIdx01, double_integrator_QP_solver_noCD_ccrhsl01, double_integrator_QP_solver_noCD_slb01, double_integrator_QP_solver_noCD_lbIdx01, double_integrator_QP_solver_noCD_rd01);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMADD2_22_17(params->A1, double_integrator_QP_solver_noCD_ccrhsp00, double_integrator_QP_solver_noCD_sp00, double_integrator_QP_solver_noCD_rd00);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMADD2_22_17(params->A2, double_integrator_QP_solver_noCD_ccrhsp01, double_integrator_QP_solver_noCD_sp01, double_integrator_QP_solver_noCD_rd01);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi00, double_integrator_QP_solver_noCD_rd00, double_integrator_QP_solver_noCD_Lbyrd00);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi01, double_integrator_QP_solver_noCD_rd01, double_integrator_QP_solver_noCD_Lbyrd01);
double_integrator_QP_solver_noCD_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_noCD_V00, double_integrator_QP_solver_noCD_Lbyrd00, double_integrator_QP_solver_noCD_W01, double_integrator_QP_solver_noCD_Lbyrd01, double_integrator_QP_solver_noCD_beta00);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld00, double_integrator_QP_solver_noCD_beta00, double_integrator_QP_solver_noCD_yy00);
double_integrator_QP_solver_noCD_LA_VSUB6_INDEXED_17_6_6(double_integrator_QP_solver_noCD_ccrhsub02, double_integrator_QP_solver_noCD_sub02, double_integrator_QP_solver_noCD_ubIdx02, double_integrator_QP_solver_noCD_ccrhsl02, double_integrator_QP_solver_noCD_slb02, double_integrator_QP_solver_noCD_lbIdx02, double_integrator_QP_solver_noCD_rd02);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMADD2_22_17(params->A3, double_integrator_QP_solver_noCD_ccrhsp02, double_integrator_QP_solver_noCD_sp02, double_integrator_QP_solver_noCD_rd02);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi02, double_integrator_QP_solver_noCD_rd02, double_integrator_QP_solver_noCD_Lbyrd02);
double_integrator_QP_solver_noCD_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_noCD_V01, double_integrator_QP_solver_noCD_Lbyrd01, double_integrator_QP_solver_noCD_W02, double_integrator_QP_solver_noCD_Lbyrd02, double_integrator_QP_solver_noCD_beta01);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd01, double_integrator_QP_solver_noCD_yy00, double_integrator_QP_solver_noCD_beta01, double_integrator_QP_solver_noCD_bmy01);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld01, double_integrator_QP_solver_noCD_bmy01, double_integrator_QP_solver_noCD_yy01);
double_integrator_QP_solver_noCD_LA_VSUB6_INDEXED_17_6_6(double_integrator_QP_solver_noCD_ccrhsub03, double_integrator_QP_solver_noCD_sub03, double_integrator_QP_solver_noCD_ubIdx03, double_integrator_QP_solver_noCD_ccrhsl03, double_integrator_QP_solver_noCD_slb03, double_integrator_QP_solver_noCD_lbIdx03, double_integrator_QP_solver_noCD_rd03);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMADD2_22_17(params->A4, double_integrator_QP_solver_noCD_ccrhsp03, double_integrator_QP_solver_noCD_sp03, double_integrator_QP_solver_noCD_rd03);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi03, double_integrator_QP_solver_noCD_rd03, double_integrator_QP_solver_noCD_Lbyrd03);
double_integrator_QP_solver_noCD_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_noCD_V02, double_integrator_QP_solver_noCD_Lbyrd02, double_integrator_QP_solver_noCD_W03, double_integrator_QP_solver_noCD_Lbyrd03, double_integrator_QP_solver_noCD_beta02);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd02, double_integrator_QP_solver_noCD_yy01, double_integrator_QP_solver_noCD_beta02, double_integrator_QP_solver_noCD_bmy02);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld02, double_integrator_QP_solver_noCD_bmy02, double_integrator_QP_solver_noCD_yy02);
double_integrator_QP_solver_noCD_LA_VSUB6_INDEXED_17_6_6(double_integrator_QP_solver_noCD_ccrhsub04, double_integrator_QP_solver_noCD_sub04, double_integrator_QP_solver_noCD_ubIdx04, double_integrator_QP_solver_noCD_ccrhsl04, double_integrator_QP_solver_noCD_slb04, double_integrator_QP_solver_noCD_lbIdx04, double_integrator_QP_solver_noCD_rd04);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMADD2_22_17(params->A5, double_integrator_QP_solver_noCD_ccrhsp04, double_integrator_QP_solver_noCD_sp04, double_integrator_QP_solver_noCD_rd04);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi04, double_integrator_QP_solver_noCD_rd04, double_integrator_QP_solver_noCD_Lbyrd04);
double_integrator_QP_solver_noCD_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_noCD_V03, double_integrator_QP_solver_noCD_Lbyrd03, double_integrator_QP_solver_noCD_W04, double_integrator_QP_solver_noCD_Lbyrd04, double_integrator_QP_solver_noCD_beta03);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd03, double_integrator_QP_solver_noCD_yy02, double_integrator_QP_solver_noCD_beta03, double_integrator_QP_solver_noCD_bmy03);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld03, double_integrator_QP_solver_noCD_bmy03, double_integrator_QP_solver_noCD_yy03);
double_integrator_QP_solver_noCD_LA_VSUB6_INDEXED_17_6_6(double_integrator_QP_solver_noCD_ccrhsub05, double_integrator_QP_solver_noCD_sub05, double_integrator_QP_solver_noCD_ubIdx05, double_integrator_QP_solver_noCD_ccrhsl05, double_integrator_QP_solver_noCD_slb05, double_integrator_QP_solver_noCD_lbIdx05, double_integrator_QP_solver_noCD_rd05);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMADD2_22_17(params->A6, double_integrator_QP_solver_noCD_ccrhsp05, double_integrator_QP_solver_noCD_sp05, double_integrator_QP_solver_noCD_rd05);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi05, double_integrator_QP_solver_noCD_rd05, double_integrator_QP_solver_noCD_Lbyrd05);
double_integrator_QP_solver_noCD_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_noCD_V04, double_integrator_QP_solver_noCD_Lbyrd04, double_integrator_QP_solver_noCD_W05, double_integrator_QP_solver_noCD_Lbyrd05, double_integrator_QP_solver_noCD_beta04);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd04, double_integrator_QP_solver_noCD_yy03, double_integrator_QP_solver_noCD_beta04, double_integrator_QP_solver_noCD_bmy04);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld04, double_integrator_QP_solver_noCD_bmy04, double_integrator_QP_solver_noCD_yy04);
double_integrator_QP_solver_noCD_LA_VSUB6_INDEXED_17_6_6(double_integrator_QP_solver_noCD_ccrhsub06, double_integrator_QP_solver_noCD_sub06, double_integrator_QP_solver_noCD_ubIdx06, double_integrator_QP_solver_noCD_ccrhsl06, double_integrator_QP_solver_noCD_slb06, double_integrator_QP_solver_noCD_lbIdx06, double_integrator_QP_solver_noCD_rd06);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMADD2_22_17(params->A7, double_integrator_QP_solver_noCD_ccrhsp06, double_integrator_QP_solver_noCD_sp06, double_integrator_QP_solver_noCD_rd06);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi06, double_integrator_QP_solver_noCD_rd06, double_integrator_QP_solver_noCD_Lbyrd06);
double_integrator_QP_solver_noCD_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_noCD_V05, double_integrator_QP_solver_noCD_Lbyrd05, double_integrator_QP_solver_noCD_W06, double_integrator_QP_solver_noCD_Lbyrd06, double_integrator_QP_solver_noCD_beta05);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd05, double_integrator_QP_solver_noCD_yy04, double_integrator_QP_solver_noCD_beta05, double_integrator_QP_solver_noCD_bmy05);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld05, double_integrator_QP_solver_noCD_bmy05, double_integrator_QP_solver_noCD_yy05);
double_integrator_QP_solver_noCD_LA_VSUB6_INDEXED_17_6_6(double_integrator_QP_solver_noCD_ccrhsub07, double_integrator_QP_solver_noCD_sub07, double_integrator_QP_solver_noCD_ubIdx07, double_integrator_QP_solver_noCD_ccrhsl07, double_integrator_QP_solver_noCD_slb07, double_integrator_QP_solver_noCD_lbIdx07, double_integrator_QP_solver_noCD_rd07);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMADD2_22_17(params->A8, double_integrator_QP_solver_noCD_ccrhsp07, double_integrator_QP_solver_noCD_sp07, double_integrator_QP_solver_noCD_rd07);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi07, double_integrator_QP_solver_noCD_rd07, double_integrator_QP_solver_noCD_Lbyrd07);
double_integrator_QP_solver_noCD_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_noCD_V06, double_integrator_QP_solver_noCD_Lbyrd06, double_integrator_QP_solver_noCD_W07, double_integrator_QP_solver_noCD_Lbyrd07, double_integrator_QP_solver_noCD_beta06);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd06, double_integrator_QP_solver_noCD_yy05, double_integrator_QP_solver_noCD_beta06, double_integrator_QP_solver_noCD_bmy06);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld06, double_integrator_QP_solver_noCD_bmy06, double_integrator_QP_solver_noCD_yy06);
double_integrator_QP_solver_noCD_LA_VSUB6_INDEXED_17_6_6(double_integrator_QP_solver_noCD_ccrhsub08, double_integrator_QP_solver_noCD_sub08, double_integrator_QP_solver_noCD_ubIdx08, double_integrator_QP_solver_noCD_ccrhsl08, double_integrator_QP_solver_noCD_slb08, double_integrator_QP_solver_noCD_lbIdx08, double_integrator_QP_solver_noCD_rd08);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMADD2_22_17(params->A9, double_integrator_QP_solver_noCD_ccrhsp08, double_integrator_QP_solver_noCD_sp08, double_integrator_QP_solver_noCD_rd08);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi08, double_integrator_QP_solver_noCD_rd08, double_integrator_QP_solver_noCD_Lbyrd08);
double_integrator_QP_solver_noCD_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_noCD_V07, double_integrator_QP_solver_noCD_Lbyrd07, double_integrator_QP_solver_noCD_W08, double_integrator_QP_solver_noCD_Lbyrd08, double_integrator_QP_solver_noCD_beta07);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd07, double_integrator_QP_solver_noCD_yy06, double_integrator_QP_solver_noCD_beta07, double_integrator_QP_solver_noCD_bmy07);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld07, double_integrator_QP_solver_noCD_bmy07, double_integrator_QP_solver_noCD_yy07);
double_integrator_QP_solver_noCD_LA_VSUB6_INDEXED_17_6_6(double_integrator_QP_solver_noCD_ccrhsub09, double_integrator_QP_solver_noCD_sub09, double_integrator_QP_solver_noCD_ubIdx09, double_integrator_QP_solver_noCD_ccrhsl09, double_integrator_QP_solver_noCD_slb09, double_integrator_QP_solver_noCD_lbIdx09, double_integrator_QP_solver_noCD_rd09);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMADD2_22_17(params->A10, double_integrator_QP_solver_noCD_ccrhsp09, double_integrator_QP_solver_noCD_sp09, double_integrator_QP_solver_noCD_rd09);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_noCD_Phi09, double_integrator_QP_solver_noCD_rd09, double_integrator_QP_solver_noCD_Lbyrd09);
double_integrator_QP_solver_noCD_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_noCD_V08, double_integrator_QP_solver_noCD_Lbyrd08, double_integrator_QP_solver_noCD_W09, double_integrator_QP_solver_noCD_Lbyrd09, double_integrator_QP_solver_noCD_beta08);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd08, double_integrator_QP_solver_noCD_yy07, double_integrator_QP_solver_noCD_beta08, double_integrator_QP_solver_noCD_bmy08);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld08, double_integrator_QP_solver_noCD_bmy08, double_integrator_QP_solver_noCD_yy08);
double_integrator_QP_solver_noCD_LA_VSUB6_INDEXED_15_6_6(double_integrator_QP_solver_noCD_ccrhsub10, double_integrator_QP_solver_noCD_sub10, double_integrator_QP_solver_noCD_ubIdx10, double_integrator_QP_solver_noCD_ccrhsl10, double_integrator_QP_solver_noCD_slb10, double_integrator_QP_solver_noCD_lbIdx10, double_integrator_QP_solver_noCD_rd10);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMADD2_22_15(params->A11, double_integrator_QP_solver_noCD_ccrhsp10, double_integrator_QP_solver_noCD_sp10, double_integrator_QP_solver_noCD_rd10);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_15(double_integrator_QP_solver_noCD_Phi10, double_integrator_QP_solver_noCD_rd10, double_integrator_QP_solver_noCD_Lbyrd10);
double_integrator_QP_solver_noCD_LA_DENSE_2MVMADD_7_17_15(double_integrator_QP_solver_noCD_V09, double_integrator_QP_solver_noCD_Lbyrd09, double_integrator_QP_solver_noCD_W10, double_integrator_QP_solver_noCD_Lbyrd10, double_integrator_QP_solver_noCD_beta09);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_noCD_Lsd09, double_integrator_QP_solver_noCD_yy08, double_integrator_QP_solver_noCD_beta09, double_integrator_QP_solver_noCD_bmy09);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_noCD_Ld09, double_integrator_QP_solver_noCD_bmy09, double_integrator_QP_solver_noCD_yy09);
double_integrator_QP_solver_noCD_LA_VSUB6_INDEXED_4_4_4(double_integrator_QP_solver_noCD_ccrhsub11, double_integrator_QP_solver_noCD_sub11, double_integrator_QP_solver_noCD_ubIdx11, double_integrator_QP_solver_noCD_ccrhsl11, double_integrator_QP_solver_noCD_slb11, double_integrator_QP_solver_noCD_lbIdx11, double_integrator_QP_solver_noCD_rd11);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMADD2_8_4(params->A12, double_integrator_QP_solver_noCD_ccrhsp11, double_integrator_QP_solver_noCD_sp11, double_integrator_QP_solver_noCD_rd11);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_4(double_integrator_QP_solver_noCD_Phi11, double_integrator_QP_solver_noCD_rd11, double_integrator_QP_solver_noCD_Lbyrd11);
double_integrator_QP_solver_noCD_LA_DENSE_2MVMADD_4_15_4(double_integrator_QP_solver_noCD_V10, double_integrator_QP_solver_noCD_Lbyrd10, double_integrator_QP_solver_noCD_W11, double_integrator_QP_solver_noCD_Lbyrd11, double_integrator_QP_solver_noCD_beta10);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB1_4_7(double_integrator_QP_solver_noCD_Lsd10, double_integrator_QP_solver_noCD_yy09, double_integrator_QP_solver_noCD_beta10, double_integrator_QP_solver_noCD_bmy10);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDSUB_4(double_integrator_QP_solver_noCD_Ld10, double_integrator_QP_solver_noCD_bmy10, double_integrator_QP_solver_noCD_yy10);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_4(double_integrator_QP_solver_noCD_Ld10, double_integrator_QP_solver_noCD_yy10, double_integrator_QP_solver_noCD_dvcc10);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_4_7(double_integrator_QP_solver_noCD_Lsd10, double_integrator_QP_solver_noCD_dvcc10, double_integrator_QP_solver_noCD_yy09, double_integrator_QP_solver_noCD_bmy09);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld09, double_integrator_QP_solver_noCD_bmy09, double_integrator_QP_solver_noCD_dvcc09);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd09, double_integrator_QP_solver_noCD_dvcc09, double_integrator_QP_solver_noCD_yy08, double_integrator_QP_solver_noCD_bmy08);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld08, double_integrator_QP_solver_noCD_bmy08, double_integrator_QP_solver_noCD_dvcc08);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd08, double_integrator_QP_solver_noCD_dvcc08, double_integrator_QP_solver_noCD_yy07, double_integrator_QP_solver_noCD_bmy07);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld07, double_integrator_QP_solver_noCD_bmy07, double_integrator_QP_solver_noCD_dvcc07);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd07, double_integrator_QP_solver_noCD_dvcc07, double_integrator_QP_solver_noCD_yy06, double_integrator_QP_solver_noCD_bmy06);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld06, double_integrator_QP_solver_noCD_bmy06, double_integrator_QP_solver_noCD_dvcc06);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd06, double_integrator_QP_solver_noCD_dvcc06, double_integrator_QP_solver_noCD_yy05, double_integrator_QP_solver_noCD_bmy05);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld05, double_integrator_QP_solver_noCD_bmy05, double_integrator_QP_solver_noCD_dvcc05);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd05, double_integrator_QP_solver_noCD_dvcc05, double_integrator_QP_solver_noCD_yy04, double_integrator_QP_solver_noCD_bmy04);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld04, double_integrator_QP_solver_noCD_bmy04, double_integrator_QP_solver_noCD_dvcc04);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd04, double_integrator_QP_solver_noCD_dvcc04, double_integrator_QP_solver_noCD_yy03, double_integrator_QP_solver_noCD_bmy03);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld03, double_integrator_QP_solver_noCD_bmy03, double_integrator_QP_solver_noCD_dvcc03);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd03, double_integrator_QP_solver_noCD_dvcc03, double_integrator_QP_solver_noCD_yy02, double_integrator_QP_solver_noCD_bmy02);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld02, double_integrator_QP_solver_noCD_bmy02, double_integrator_QP_solver_noCD_dvcc02);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd02, double_integrator_QP_solver_noCD_dvcc02, double_integrator_QP_solver_noCD_yy01, double_integrator_QP_solver_noCD_bmy01);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld01, double_integrator_QP_solver_noCD_bmy01, double_integrator_QP_solver_noCD_dvcc01);
double_integrator_QP_solver_noCD_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_noCD_Lsd01, double_integrator_QP_solver_noCD_dvcc01, double_integrator_QP_solver_noCD_yy00, double_integrator_QP_solver_noCD_bmy00);
double_integrator_QP_solver_noCD_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_noCD_Ld00, double_integrator_QP_solver_noCD_bmy00, double_integrator_QP_solver_noCD_dvcc00);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM_7_17(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvcc00, double_integrator_QP_solver_noCD_grad_eq00);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvcc01, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvcc00, double_integrator_QP_solver_noCD_grad_eq01);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvcc02, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvcc01, double_integrator_QP_solver_noCD_grad_eq02);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvcc03, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvcc02, double_integrator_QP_solver_noCD_grad_eq03);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvcc04, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvcc03, double_integrator_QP_solver_noCD_grad_eq04);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvcc05, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvcc04, double_integrator_QP_solver_noCD_grad_eq05);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvcc06, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvcc05, double_integrator_QP_solver_noCD_grad_eq06);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvcc07, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvcc06, double_integrator_QP_solver_noCD_grad_eq07);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvcc08, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvcc07, double_integrator_QP_solver_noCD_grad_eq08);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_noCD_C00, double_integrator_QP_solver_noCD_dvcc09, double_integrator_QP_solver_noCD_D01, double_integrator_QP_solver_noCD_dvcc08, double_integrator_QP_solver_noCD_grad_eq09);
double_integrator_QP_solver_noCD_LA_DENSE_MTVM2_4_15_7(double_integrator_QP_solver_noCD_C10, double_integrator_QP_solver_noCD_dvcc10, double_integrator_QP_solver_noCD_D10, double_integrator_QP_solver_noCD_dvcc09, double_integrator_QP_solver_noCD_grad_eq10);
double_integrator_QP_solver_noCD_LA_DIAGZERO_MTVM_4_4(double_integrator_QP_solver_noCD_D11, double_integrator_QP_solver_noCD_dvcc10, double_integrator_QP_solver_noCD_grad_eq11);
double_integrator_QP_solver_noCD_LA_VSUB_189(double_integrator_QP_solver_noCD_rd, double_integrator_QP_solver_noCD_grad_eq, double_integrator_QP_solver_noCD_rd);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi00, double_integrator_QP_solver_noCD_rd00, double_integrator_QP_solver_noCD_dzcc00);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi01, double_integrator_QP_solver_noCD_rd01, double_integrator_QP_solver_noCD_dzcc01);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi02, double_integrator_QP_solver_noCD_rd02, double_integrator_QP_solver_noCD_dzcc02);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi03, double_integrator_QP_solver_noCD_rd03, double_integrator_QP_solver_noCD_dzcc03);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi04, double_integrator_QP_solver_noCD_rd04, double_integrator_QP_solver_noCD_dzcc04);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi05, double_integrator_QP_solver_noCD_rd05, double_integrator_QP_solver_noCD_dzcc05);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi06, double_integrator_QP_solver_noCD_rd06, double_integrator_QP_solver_noCD_dzcc06);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi07, double_integrator_QP_solver_noCD_rd07, double_integrator_QP_solver_noCD_dzcc07);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi08, double_integrator_QP_solver_noCD_rd08, double_integrator_QP_solver_noCD_dzcc08);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_noCD_Phi09, double_integrator_QP_solver_noCD_rd09, double_integrator_QP_solver_noCD_dzcc09);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_noCD_Phi10, double_integrator_QP_solver_noCD_rd10, double_integrator_QP_solver_noCD_dzcc10);
double_integrator_QP_solver_noCD_LA_DENSE_FORWARDBACKWARDSUB_4(double_integrator_QP_solver_noCD_Phi11, double_integrator_QP_solver_noCD_rd11, double_integrator_QP_solver_noCD_dzcc11);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_noCD_ccrhsl00, double_integrator_QP_solver_noCD_slb00, double_integrator_QP_solver_noCD_llbbyslb00, double_integrator_QP_solver_noCD_dzcc00, double_integrator_QP_solver_noCD_lbIdx00, double_integrator_QP_solver_noCD_dllbcc00);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsub00, double_integrator_QP_solver_noCD_sub00, double_integrator_QP_solver_noCD_lubbysub00, double_integrator_QP_solver_noCD_dzcc00, double_integrator_QP_solver_noCD_ubIdx00, double_integrator_QP_solver_noCD_dlubcc00);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB5_22_17(params->A1, double_integrator_QP_solver_noCD_dzcc00, double_integrator_QP_solver_noCD_ccrhsp00, double_integrator_QP_solver_noCD_sp00, double_integrator_QP_solver_noCD_lp00, double_integrator_QP_solver_noCD_dlp_cc00);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsl01, double_integrator_QP_solver_noCD_slb01, double_integrator_QP_solver_noCD_llbbyslb01, double_integrator_QP_solver_noCD_dzcc01, double_integrator_QP_solver_noCD_lbIdx01, double_integrator_QP_solver_noCD_dllbcc01);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsub01, double_integrator_QP_solver_noCD_sub01, double_integrator_QP_solver_noCD_lubbysub01, double_integrator_QP_solver_noCD_dzcc01, double_integrator_QP_solver_noCD_ubIdx01, double_integrator_QP_solver_noCD_dlubcc01);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB5_22_17(params->A2, double_integrator_QP_solver_noCD_dzcc01, double_integrator_QP_solver_noCD_ccrhsp01, double_integrator_QP_solver_noCD_sp01, double_integrator_QP_solver_noCD_lp01, double_integrator_QP_solver_noCD_dlp_cc01);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsl02, double_integrator_QP_solver_noCD_slb02, double_integrator_QP_solver_noCD_llbbyslb02, double_integrator_QP_solver_noCD_dzcc02, double_integrator_QP_solver_noCD_lbIdx02, double_integrator_QP_solver_noCD_dllbcc02);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsub02, double_integrator_QP_solver_noCD_sub02, double_integrator_QP_solver_noCD_lubbysub02, double_integrator_QP_solver_noCD_dzcc02, double_integrator_QP_solver_noCD_ubIdx02, double_integrator_QP_solver_noCD_dlubcc02);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB5_22_17(params->A3, double_integrator_QP_solver_noCD_dzcc02, double_integrator_QP_solver_noCD_ccrhsp02, double_integrator_QP_solver_noCD_sp02, double_integrator_QP_solver_noCD_lp02, double_integrator_QP_solver_noCD_dlp_cc02);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsl03, double_integrator_QP_solver_noCD_slb03, double_integrator_QP_solver_noCD_llbbyslb03, double_integrator_QP_solver_noCD_dzcc03, double_integrator_QP_solver_noCD_lbIdx03, double_integrator_QP_solver_noCD_dllbcc03);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsub03, double_integrator_QP_solver_noCD_sub03, double_integrator_QP_solver_noCD_lubbysub03, double_integrator_QP_solver_noCD_dzcc03, double_integrator_QP_solver_noCD_ubIdx03, double_integrator_QP_solver_noCD_dlubcc03);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB5_22_17(params->A4, double_integrator_QP_solver_noCD_dzcc03, double_integrator_QP_solver_noCD_ccrhsp03, double_integrator_QP_solver_noCD_sp03, double_integrator_QP_solver_noCD_lp03, double_integrator_QP_solver_noCD_dlp_cc03);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsl04, double_integrator_QP_solver_noCD_slb04, double_integrator_QP_solver_noCD_llbbyslb04, double_integrator_QP_solver_noCD_dzcc04, double_integrator_QP_solver_noCD_lbIdx04, double_integrator_QP_solver_noCD_dllbcc04);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsub04, double_integrator_QP_solver_noCD_sub04, double_integrator_QP_solver_noCD_lubbysub04, double_integrator_QP_solver_noCD_dzcc04, double_integrator_QP_solver_noCD_ubIdx04, double_integrator_QP_solver_noCD_dlubcc04);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB5_22_17(params->A5, double_integrator_QP_solver_noCD_dzcc04, double_integrator_QP_solver_noCD_ccrhsp04, double_integrator_QP_solver_noCD_sp04, double_integrator_QP_solver_noCD_lp04, double_integrator_QP_solver_noCD_dlp_cc04);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsl05, double_integrator_QP_solver_noCD_slb05, double_integrator_QP_solver_noCD_llbbyslb05, double_integrator_QP_solver_noCD_dzcc05, double_integrator_QP_solver_noCD_lbIdx05, double_integrator_QP_solver_noCD_dllbcc05);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsub05, double_integrator_QP_solver_noCD_sub05, double_integrator_QP_solver_noCD_lubbysub05, double_integrator_QP_solver_noCD_dzcc05, double_integrator_QP_solver_noCD_ubIdx05, double_integrator_QP_solver_noCD_dlubcc05);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB5_22_17(params->A6, double_integrator_QP_solver_noCD_dzcc05, double_integrator_QP_solver_noCD_ccrhsp05, double_integrator_QP_solver_noCD_sp05, double_integrator_QP_solver_noCD_lp05, double_integrator_QP_solver_noCD_dlp_cc05);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsl06, double_integrator_QP_solver_noCD_slb06, double_integrator_QP_solver_noCD_llbbyslb06, double_integrator_QP_solver_noCD_dzcc06, double_integrator_QP_solver_noCD_lbIdx06, double_integrator_QP_solver_noCD_dllbcc06);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsub06, double_integrator_QP_solver_noCD_sub06, double_integrator_QP_solver_noCD_lubbysub06, double_integrator_QP_solver_noCD_dzcc06, double_integrator_QP_solver_noCD_ubIdx06, double_integrator_QP_solver_noCD_dlubcc06);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB5_22_17(params->A7, double_integrator_QP_solver_noCD_dzcc06, double_integrator_QP_solver_noCD_ccrhsp06, double_integrator_QP_solver_noCD_sp06, double_integrator_QP_solver_noCD_lp06, double_integrator_QP_solver_noCD_dlp_cc06);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsl07, double_integrator_QP_solver_noCD_slb07, double_integrator_QP_solver_noCD_llbbyslb07, double_integrator_QP_solver_noCD_dzcc07, double_integrator_QP_solver_noCD_lbIdx07, double_integrator_QP_solver_noCD_dllbcc07);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsub07, double_integrator_QP_solver_noCD_sub07, double_integrator_QP_solver_noCD_lubbysub07, double_integrator_QP_solver_noCD_dzcc07, double_integrator_QP_solver_noCD_ubIdx07, double_integrator_QP_solver_noCD_dlubcc07);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB5_22_17(params->A8, double_integrator_QP_solver_noCD_dzcc07, double_integrator_QP_solver_noCD_ccrhsp07, double_integrator_QP_solver_noCD_sp07, double_integrator_QP_solver_noCD_lp07, double_integrator_QP_solver_noCD_dlp_cc07);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsl08, double_integrator_QP_solver_noCD_slb08, double_integrator_QP_solver_noCD_llbbyslb08, double_integrator_QP_solver_noCD_dzcc08, double_integrator_QP_solver_noCD_lbIdx08, double_integrator_QP_solver_noCD_dllbcc08);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsub08, double_integrator_QP_solver_noCD_sub08, double_integrator_QP_solver_noCD_lubbysub08, double_integrator_QP_solver_noCD_dzcc08, double_integrator_QP_solver_noCD_ubIdx08, double_integrator_QP_solver_noCD_dlubcc08);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB5_22_17(params->A9, double_integrator_QP_solver_noCD_dzcc08, double_integrator_QP_solver_noCD_ccrhsp08, double_integrator_QP_solver_noCD_sp08, double_integrator_QP_solver_noCD_lp08, double_integrator_QP_solver_noCD_dlp_cc08);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsl09, double_integrator_QP_solver_noCD_slb09, double_integrator_QP_solver_noCD_llbbyslb09, double_integrator_QP_solver_noCD_dzcc09, double_integrator_QP_solver_noCD_lbIdx09, double_integrator_QP_solver_noCD_dllbcc09);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsub09, double_integrator_QP_solver_noCD_sub09, double_integrator_QP_solver_noCD_lubbysub09, double_integrator_QP_solver_noCD_dzcc09, double_integrator_QP_solver_noCD_ubIdx09, double_integrator_QP_solver_noCD_dlubcc09);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB5_22_17(params->A10, double_integrator_QP_solver_noCD_dzcc09, double_integrator_QP_solver_noCD_ccrhsp09, double_integrator_QP_solver_noCD_sp09, double_integrator_QP_solver_noCD_lp09, double_integrator_QP_solver_noCD_dlp_cc09);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTSUB_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsl10, double_integrator_QP_solver_noCD_slb10, double_integrator_QP_solver_noCD_llbbyslb10, double_integrator_QP_solver_noCD_dzcc10, double_integrator_QP_solver_noCD_lbIdx10, double_integrator_QP_solver_noCD_dllbcc10);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_noCD_ccrhsub10, double_integrator_QP_solver_noCD_sub10, double_integrator_QP_solver_noCD_lubbysub10, double_integrator_QP_solver_noCD_dzcc10, double_integrator_QP_solver_noCD_ubIdx10, double_integrator_QP_solver_noCD_dlubcc10);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB5_22_15(params->A11, double_integrator_QP_solver_noCD_dzcc10, double_integrator_QP_solver_noCD_ccrhsp10, double_integrator_QP_solver_noCD_sp10, double_integrator_QP_solver_noCD_lp10, double_integrator_QP_solver_noCD_dlp_cc10);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_noCD_ccrhsl11, double_integrator_QP_solver_noCD_slb11, double_integrator_QP_solver_noCD_llbbyslb11, double_integrator_QP_solver_noCD_dzcc11, double_integrator_QP_solver_noCD_lbIdx11, double_integrator_QP_solver_noCD_dllbcc11);
double_integrator_QP_solver_noCD_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_noCD_ccrhsub11, double_integrator_QP_solver_noCD_sub11, double_integrator_QP_solver_noCD_lubbysub11, double_integrator_QP_solver_noCD_dzcc11, double_integrator_QP_solver_noCD_ubIdx11, double_integrator_QP_solver_noCD_dlubcc11);
double_integrator_QP_solver_noCD_LA_DENSE_MVMSUB5_8_4(params->A12, double_integrator_QP_solver_noCD_dzcc11, double_integrator_QP_solver_noCD_ccrhsp11, double_integrator_QP_solver_noCD_sp11, double_integrator_QP_solver_noCD_lp11, double_integrator_QP_solver_noCD_dlp_cc11);
double_integrator_QP_solver_noCD_LA_VSUB7_391(double_integrator_QP_solver_noCD_l, double_integrator_QP_solver_noCD_ccrhs, double_integrator_QP_solver_noCD_s, double_integrator_QP_solver_noCD_dl_cc, double_integrator_QP_solver_noCD_ds_cc);
double_integrator_QP_solver_noCD_LA_VADD_189(double_integrator_QP_solver_noCD_dz_cc, double_integrator_QP_solver_noCD_dz_aff);
double_integrator_QP_solver_noCD_LA_VADD_74(double_integrator_QP_solver_noCD_dv_cc, double_integrator_QP_solver_noCD_dv_aff);
double_integrator_QP_solver_noCD_LA_VADD_391(double_integrator_QP_solver_noCD_dl_cc, double_integrator_QP_solver_noCD_dl_aff);
double_integrator_QP_solver_noCD_LA_VADD_391(double_integrator_QP_solver_noCD_ds_cc, double_integrator_QP_solver_noCD_ds_aff);
info->lsit_cc = double_integrator_QP_solver_noCD_LINESEARCH_BACKTRACKING_COMBINED(double_integrator_QP_solver_noCD_z, double_integrator_QP_solver_noCD_v, double_integrator_QP_solver_noCD_l, double_integrator_QP_solver_noCD_s, double_integrator_QP_solver_noCD_dz_cc, double_integrator_QP_solver_noCD_dv_cc, double_integrator_QP_solver_noCD_dl_cc, double_integrator_QP_solver_noCD_ds_cc, &info->step_cc, &info->mu);
if( info->lsit_cc == double_integrator_QP_solver_noCD_NOPROGRESS ){
exitcode = double_integrator_QP_solver_noCD_NOPROGRESS; break;
}
info->it++;
}
output->z1[0] = double_integrator_QP_solver_noCD_z00[0];
output->z1[1] = double_integrator_QP_solver_noCD_z00[1];
output->z1[2] = double_integrator_QP_solver_noCD_z00[2];
output->z1[3] = double_integrator_QP_solver_noCD_z00[3];
output->z1[4] = double_integrator_QP_solver_noCD_z00[4];
output->z1[5] = double_integrator_QP_solver_noCD_z00[5];
output->z1[6] = double_integrator_QP_solver_noCD_z00[12];
output->z2[0] = double_integrator_QP_solver_noCD_z01[0];
output->z2[1] = double_integrator_QP_solver_noCD_z01[1];
output->z2[2] = double_integrator_QP_solver_noCD_z01[2];
output->z2[3] = double_integrator_QP_solver_noCD_z01[3];
output->z2[4] = double_integrator_QP_solver_noCD_z01[4];
output->z2[5] = double_integrator_QP_solver_noCD_z01[5];
output->z2[6] = double_integrator_QP_solver_noCD_z01[12];
output->z3[0] = double_integrator_QP_solver_noCD_z02[0];
output->z3[1] = double_integrator_QP_solver_noCD_z02[1];
output->z3[2] = double_integrator_QP_solver_noCD_z02[2];
output->z3[3] = double_integrator_QP_solver_noCD_z02[3];
output->z3[4] = double_integrator_QP_solver_noCD_z02[4];
output->z3[5] = double_integrator_QP_solver_noCD_z02[5];
output->z3[6] = double_integrator_QP_solver_noCD_z02[12];
output->z4[0] = double_integrator_QP_solver_noCD_z03[0];
output->z4[1] = double_integrator_QP_solver_noCD_z03[1];
output->z4[2] = double_integrator_QP_solver_noCD_z03[2];
output->z4[3] = double_integrator_QP_solver_noCD_z03[3];
output->z4[4] = double_integrator_QP_solver_noCD_z03[4];
output->z4[5] = double_integrator_QP_solver_noCD_z03[5];
output->z4[6] = double_integrator_QP_solver_noCD_z03[12];
output->z5[0] = double_integrator_QP_solver_noCD_z04[0];
output->z5[1] = double_integrator_QP_solver_noCD_z04[1];
output->z5[2] = double_integrator_QP_solver_noCD_z04[2];
output->z5[3] = double_integrator_QP_solver_noCD_z04[3];
output->z5[4] = double_integrator_QP_solver_noCD_z04[4];
output->z5[5] = double_integrator_QP_solver_noCD_z04[5];
output->z5[6] = double_integrator_QP_solver_noCD_z04[12];
output->z6[0] = double_integrator_QP_solver_noCD_z05[0];
output->z6[1] = double_integrator_QP_solver_noCD_z05[1];
output->z6[2] = double_integrator_QP_solver_noCD_z05[2];
output->z6[3] = double_integrator_QP_solver_noCD_z05[3];
output->z6[4] = double_integrator_QP_solver_noCD_z05[4];
output->z6[5] = double_integrator_QP_solver_noCD_z05[5];
output->z6[6] = double_integrator_QP_solver_noCD_z05[12];
output->z7[0] = double_integrator_QP_solver_noCD_z06[0];
output->z7[1] = double_integrator_QP_solver_noCD_z06[1];
output->z7[2] = double_integrator_QP_solver_noCD_z06[2];
output->z7[3] = double_integrator_QP_solver_noCD_z06[3];
output->z7[4] = double_integrator_QP_solver_noCD_z06[4];
output->z7[5] = double_integrator_QP_solver_noCD_z06[5];
output->z7[6] = double_integrator_QP_solver_noCD_z06[12];
output->z8[0] = double_integrator_QP_solver_noCD_z07[0];
output->z8[1] = double_integrator_QP_solver_noCD_z07[1];
output->z8[2] = double_integrator_QP_solver_noCD_z07[2];
output->z8[3] = double_integrator_QP_solver_noCD_z07[3];
output->z8[4] = double_integrator_QP_solver_noCD_z07[4];
output->z8[5] = double_integrator_QP_solver_noCD_z07[5];
output->z8[6] = double_integrator_QP_solver_noCD_z07[12];
output->z9[0] = double_integrator_QP_solver_noCD_z08[0];
output->z9[1] = double_integrator_QP_solver_noCD_z08[1];
output->z9[2] = double_integrator_QP_solver_noCD_z08[2];
output->z9[3] = double_integrator_QP_solver_noCD_z08[3];
output->z9[4] = double_integrator_QP_solver_noCD_z08[4];
output->z9[5] = double_integrator_QP_solver_noCD_z08[5];
output->z9[6] = double_integrator_QP_solver_noCD_z08[12];
output->z10[0] = double_integrator_QP_solver_noCD_z09[0];
output->z10[1] = double_integrator_QP_solver_noCD_z09[1];
output->z10[2] = double_integrator_QP_solver_noCD_z09[2];
output->z10[3] = double_integrator_QP_solver_noCD_z09[3];
output->z10[4] = double_integrator_QP_solver_noCD_z09[4];
output->z10[5] = double_integrator_QP_solver_noCD_z09[5];
output->z10[6] = double_integrator_QP_solver_noCD_z09[12];
output->z11[0] = double_integrator_QP_solver_noCD_z10[0];
output->z11[1] = double_integrator_QP_solver_noCD_z10[1];
output->z11[2] = double_integrator_QP_solver_noCD_z10[2];
output->z11[3] = double_integrator_QP_solver_noCD_z10[3];
output->z11[4] = double_integrator_QP_solver_noCD_z10[4];
output->z11[5] = double_integrator_QP_solver_noCD_z10[5];
output->z11[6] = double_integrator_QP_solver_noCD_z10[10];
output->z12[0] = double_integrator_QP_solver_noCD_z11[0];
output->z12[1] = double_integrator_QP_solver_noCD_z11[1];
output->z12[2] = double_integrator_QP_solver_noCD_z11[2];
output->z12[3] = double_integrator_QP_solver_noCD_z11[3];

#if double_integrator_QP_solver_noCD_SET_TIMING == 1
info->solvetime = double_integrator_QP_solver_noCD_toc(&solvertimer);
#if double_integrator_QP_solver_noCD_SET_PRINTLEVEL > 0 && double_integrator_QP_solver_noCD_SET_TIMING == 1
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
