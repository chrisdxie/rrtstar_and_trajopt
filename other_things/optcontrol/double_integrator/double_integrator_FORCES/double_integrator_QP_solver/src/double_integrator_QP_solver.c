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

#include "../include/double_integrator_QP_solver.h"

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
 * Initializes a vector of length 240 with a value.
 */
void double_integrator_QP_solver_LA_INITIALIZEVECTOR_240(double_integrator_QP_solver_FLOAT* vec, double_integrator_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<240; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 95 with a value.
 */
void double_integrator_QP_solver_LA_INITIALIZEVECTOR_95(double_integrator_QP_solver_FLOAT* vec, double_integrator_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<95; i++ )
	{
		vec[i] = value;
	}
}


/*
 * Initializes a vector of length 433 with a value.
 */
void double_integrator_QP_solver_LA_INITIALIZEVECTOR_433(double_integrator_QP_solver_FLOAT* vec, double_integrator_QP_solver_FLOAT value)
{
	int i;
	for( i=0; i<433; i++ )
	{
		vec[i] = value;
	}
}


/* 
 * Calculates a dot product and adds it to a variable: z += x'*y; 
 * This function is for vectors of length 433.
 */
void double_integrator_QP_solver_LA_DOTACC_433(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<433; i++ ){
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
void double_integrator_QP_solver_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_FLOAT* H, double_integrator_QP_solver_FLOAT* f, double_integrator_QP_solver_FLOAT* z, double_integrator_QP_solver_FLOAT* grad, double_integrator_QP_solver_FLOAT* value)
{
	int i;
	double_integrator_QP_solver_FLOAT hz;	
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
 * INPUTS:     H  - Symmetric Hessian, diag matrix of size [4 x 4]
 *             f  - column vector of size 4
 *             z  - column vector of size 4
 *
 * OUTPUTS: grad  - gradient at z (= H*z + f), column vector of size 4
 *          value <-- value + 0.5*z'*H*z + f'*z (value will be modified)
 */
void double_integrator_QP_solver_LA_DIAG_QUADFCN_4(double_integrator_QP_solver_FLOAT* H, double_integrator_QP_solver_FLOAT* f, double_integrator_QP_solver_FLOAT* z, double_integrator_QP_solver_FLOAT* grad, double_integrator_QP_solver_FLOAT* value)
{
	int i;
	double_integrator_QP_solver_FLOAT hz;	
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_17_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
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
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_4_15_4(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
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
void double_integrator_QP_solver_LA_DENSE_MTVM_7_17(double_integrator_QP_solver_FLOAT *M, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
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
void double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
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
void double_integrator_QP_solver_LA_DENSE_MTVM2_4_15_7(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
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
void double_integrator_QP_solver_LA_DIAGZERO_MTVM_4_4(double_integrator_QP_solver_FLOAT *M, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
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
void double_integrator_QP_solver_LA_MVSUBADD_22_17(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	double_integrator_QP_solver_FLOAT Ax[22];
	double_integrator_QP_solver_FLOAT Axlessb;
	double_integrator_QP_solver_FLOAT norm = *y;
	double_integrator_QP_solver_FLOAT lAxlessb = 0;

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
void double_integrator_QP_solver_LA_MVSUBADD_22_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *z, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	int j;
	int k = 0;
	double_integrator_QP_solver_FLOAT Ax[22];
	double_integrator_QP_solver_FLOAT Axlessb;
	double_integrator_QP_solver_FLOAT norm = *y;
	double_integrator_QP_solver_FLOAT lAxlessb = 0;

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
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 17
 * Returns also L/S, a value that is often used elsewhere.
 */
void double_integrator_QP_solver_LA_INEQ_B_GRAD_17_7_6(double_integrator_QP_solver_FLOAT *lu, double_integrator_QP_solver_FLOAT *su, double_integrator_QP_solver_FLOAT *ru, double_integrator_QP_solver_FLOAT *ll, double_integrator_QP_solver_FLOAT *sl, double_integrator_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_FLOAT *grad, double_integrator_QP_solver_FLOAT *lubysu, double_integrator_QP_solver_FLOAT *llbysl)
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
void double_integrator_QP_solver_LA_INEQ_P_22_17(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *lp, double_integrator_QP_solver_FLOAT *sp, double_integrator_QP_solver_FLOAT *rip, double_integrator_QP_solver_FLOAT *grad, double_integrator_QP_solver_FLOAT *lpbysp)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_FLOAT lsr[22];
	
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
void double_integrator_QP_solver_LA_INEQ_B_GRAD_17_4_4(double_integrator_QP_solver_FLOAT *lu, double_integrator_QP_solver_FLOAT *su, double_integrator_QP_solver_FLOAT *ru, double_integrator_QP_solver_FLOAT *ll, double_integrator_QP_solver_FLOAT *sl, double_integrator_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_FLOAT *grad, double_integrator_QP_solver_FLOAT *lubysu, double_integrator_QP_solver_FLOAT *llbysl)
{
	int i;
	for( i=0; i<17; i++ ){
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
 * Computes inequality constraints gradient-
 * Special function for box constraints of length 15
 * Returns also L/S, a value that is often used elsewhere.
 */
void double_integrator_QP_solver_LA_INEQ_B_GRAD_15_4_4(double_integrator_QP_solver_FLOAT *lu, double_integrator_QP_solver_FLOAT *su, double_integrator_QP_solver_FLOAT *ru, double_integrator_QP_solver_FLOAT *ll, double_integrator_QP_solver_FLOAT *sl, double_integrator_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_FLOAT *grad, double_integrator_QP_solver_FLOAT *lubysu, double_integrator_QP_solver_FLOAT *llbysl)
{
	int i;
	for( i=0; i<15; i++ ){
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
void double_integrator_QP_solver_LA_INEQ_P_22_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *lp, double_integrator_QP_solver_FLOAT *sp, double_integrator_QP_solver_FLOAT *rip, double_integrator_QP_solver_FLOAT *grad, double_integrator_QP_solver_FLOAT *lpbysp)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_FLOAT lsr[22];
	
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
void double_integrator_QP_solver_LA_INEQ_B_GRAD_4_4_4(double_integrator_QP_solver_FLOAT *lu, double_integrator_QP_solver_FLOAT *su, double_integrator_QP_solver_FLOAT *ru, double_integrator_QP_solver_FLOAT *ll, double_integrator_QP_solver_FLOAT *sl, double_integrator_QP_solver_FLOAT *rl, int* lbIdx, int* ubIdx, double_integrator_QP_solver_FLOAT *grad, double_integrator_QP_solver_FLOAT *lubysu, double_integrator_QP_solver_FLOAT *llbysl)
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
 * of length 240.
 */
void double_integrator_QP_solver_LA_VVADD3_240(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *w, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<240; i++ ){
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
void double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_17_7_6(double_integrator_QP_solver_FLOAT *H, double_integrator_QP_solver_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_FLOAT *Phi)
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
void double_integrator_QP_solver_LA_DENSE_ADDMTDM_22_17(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *d, double_integrator_QP_solver_FLOAT *X)
{    
    int i,j,k,ii,di;
    double_integrator_QP_solver_FLOAT x;
    
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
void double_integrator_QP_solver_LA_DENSE_CHOL2_17(double_integrator_QP_solver_FLOAT *A)
{
    int i, j, k, di, dj;
	 int ii, jj;
    double_integrator_QP_solver_FLOAT l;
    double_integrator_QP_solver_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<17; i++ ){
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
void double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    double_integrator_QP_solver_FLOAT a;
    
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
void double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_FLOAT yel;
            
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
void double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_17_4_4(double_integrator_QP_solver_FLOAT *H, double_integrator_QP_solver_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_FLOAT *Phi)
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
void double_integrator_QP_solver_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *C)
{
    int i, j, k;
    double_integrator_QP_solver_FLOAT temp;
    
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
void double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_4_4(double_integrator_QP_solver_FLOAT *H, double_integrator_QP_solver_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_FLOAT *Phi)
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
 * A is stored in column major format and is of size [22 x 15]
 * Phi is of size [15 x 15].
 */
void double_integrator_QP_solver_LA_DENSE_ADDMTDM_22_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *d, double_integrator_QP_solver_FLOAT *X)
{    
    int i,j,k,ii,di;
    double_integrator_QP_solver_FLOAT x;
    
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
void double_integrator_QP_solver_LA_DENSE_CHOL2_15(double_integrator_QP_solver_FLOAT *A)
{
    int i, j, k, di, dj;
	 int ii, jj;
    double_integrator_QP_solver_FLOAT l;
    double_integrator_QP_solver_FLOAT Mii;
    
	ii=0; di=0;
    for( i=0; i<15; i++ ){
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
void double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_4_15(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    double_integrator_QP_solver_FLOAT a;
    
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
void double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_15(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
{
    int i,j,k,di;
	 int ii;
    double_integrator_QP_solver_FLOAT a;
    
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
void double_integrator_QP_solver_LA_DENSE_MMTM_7_15_4(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *C)
{
    int i, j, k;
    double_integrator_QP_solver_FLOAT temp;
    
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
void double_integrator_QP_solver_LA_DENSE_FORWARDSUB_15(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *y)
{
    int i,j,ii,di;
    double_integrator_QP_solver_FLOAT yel;
            
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
void double_integrator_QP_solver_LA_DIAG_CHOL_LBUB_5_4_4(double_integrator_QP_solver_FLOAT *H, double_integrator_QP_solver_FLOAT *llbysl, int* lbIdx, double_integrator_QP_solver_FLOAT *lubysu, int* ubIdx, double_integrator_QP_solver_FLOAT *Phi)


{
	int i;
	
	/* copy  H into PHI */
	for( i=0; i<5; i++ ){
		Phi[i] = H[i];
	}

	/* add llbysl onto Phi where necessary */
	for( i=0; i<4; i++ ){
		Phi[lbIdx[i]] += llbysl[i];
	}

	/* add lubysu onto Phi where necessary */
	for( i=0; i<4; i++){
		Phi[ubIdx[i]] +=  lubysu[i];
	}
	
	/* compute cholesky */
	for(i=0; i<5; i++)
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
 * where A is to be computed and is of size [4 x 4],
 * B is given and of size [4 x 4], L is a diagonal
 *  matrix of size 4 stored in diagonal 
 * storage format. Note the transpose of L!
 *
 * Result: A in diagonalzero storage format.
 *
 */
void double_integrator_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_4_4(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *A)
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
void double_integrator_QP_solver_LA_DIAG_FORWARDSUB_4(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *y)
{
    int i;

    for( i=0; i<4; i++ ){
		y[i] = b[i]/L[i];
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
void double_integrator_QP_solver_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
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
void double_integrator_QP_solver_LA_DENSE_MMT2_7_17_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_17_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
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
 * storage format, and B is of size [4 x 4] diagonalzero
 * storage format.
 * 
 * THIS ONE HAS THE WORST ACCES PATTERN POSSIBLE. 
 * POSSIBKE FIX: PUT A AND B INTO ROW MAJOR FORMAT FIRST.
 * 
 */
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMT2_4_15_4(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *L)
{
    int i, j, k, ii, di;
    double_integrator_QP_solver_FLOAT ltemp;
    
    ii = 0; di = 0;
    for( i=0; i<4; i++ ){        
        for( j=0; j<=i; j++ ){
            ltemp = 0; 
            for( k=0; k<15; k++ ){
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
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_4_15_4(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<4; i++ ){
		r[i] = b[i] - A[k++]*x[0] - B[i]*u[i];
	}	

	for( j=1; j<15; j++ ){		
		for( i=0; i<4; i++ ){
			r[i] -= A[k++]*x[j];
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
 * Vector subtraction z = -x - y for vectors of length 240.
 */
void double_integrator_QP_solver_LA_VSUB2_240(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<240; i++){
		z[i] = -x[i] - y[i];
	}
}


/**
 * Forward-Backward-Substitution to solve L*L^T*x = b where L is a
 * lower triangular matrix of size 17 in lower triangular
 * storage format.
 */
void double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_FLOAT y[17];
    double_integrator_QP_solver_FLOAT yel,xel;
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
void double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *x)
{
    int i, ii, di, j, jj, dj;
    double_integrator_QP_solver_FLOAT y[15];
    double_integrator_QP_solver_FLOAT yel,xel;
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
 * diagonal matrix of size 4 in vector
 * storage format.
 */
void double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_4(double_integrator_QP_solver_FLOAT *L, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *x)
{
    int i;
            
    /* solve Ly = b by forward and backward substitution */
    for( i=0; i<4; i++ ){
		x[i] = b[i]/(L[i]*L[i]);
    }
    
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 7,
 * and x has length 17 and is indexed through yidx.
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
 * Vector subtraction z = -x - y(yidx) where y is of length 17
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB4_22_17(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
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
void double_integrator_QP_solver_LA_VSUB3_22(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *w, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<22; i++){
		x[i] = -u[i]*v[i] - w[i];
	}
}


/*
 * Vector subtraction z = x(xidx) - y where y, z and xidx are of length 4,
 * and x has length 17 and is indexed through yidx.
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
 * Vector subtraction z = -x - y(yidx) where y is of length 17
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB4_22_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *r)
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
        for( i=0; i<433; i++ ){
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
        if( i == 433 ){
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
    *mu_aff = mymu / (double_integrator_QP_solver_FLOAT)433;
    return lsIt;
}


/*
 * Vector subtraction x = (u.*v - mu)*sigma where a is a scalar
*  and x,u,v are vectors of length 433.
 */
void double_integrator_QP_solver_LA_VSUB5_433(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT mu,  double_integrator_QP_solver_FLOAT sigma, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<433; i++){
		x[i] = u[i]*v[i] - mu;
		x[i] *= sigma;
	}
}


/*
 * Computes x=0; x(uidx) += u/su; x(vidx) -= v/sv where x is of length 17,
 * u, su, uidx are of length 6 and v, sv, vidx are of length 7.
 */
void double_integrator_QP_solver_LA_VSUB6_INDEXED_17_6_7(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *su, int* uidx, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *sv, int* vidx, double_integrator_QP_solver_FLOAT *x)
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
 * u, su, uidx are of length 4 and v, sv, vidx are of length 4.
 */
void double_integrator_QP_solver_LA_VSUB6_INDEXED_17_4_4(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *su, int* uidx, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *sv, int* vidx, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<17; i++ ){
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
 * Matrix vector multiplication z = z + A'*(x./s) where A is of size [22 x 17]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_LA_DENSE_MTVMADD2_22_17(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0; 
	double_integrator_QP_solver_FLOAT temp[22];

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
void double_integrator_QP_solver_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *r)
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
 * u, su, uidx are of length 4 and v, sv, vidx are of length 4.
 */
void double_integrator_QP_solver_LA_VSUB6_INDEXED_15_4_4(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *su, int* uidx, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *sv, int* vidx, double_integrator_QP_solver_FLOAT *x)
{
	int i;
	for( i=0; i<15; i++ ){
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
 * Matrix vector multiplication z = z + A'*(x./s) where A is of size [22 x 15]
 * and stored in column major format. Note the transpose of M!
 */
void double_integrator_QP_solver_LA_DENSE_MTVMADD2_22_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	int j;
	int k = 0; 
	double_integrator_QP_solver_FLOAT temp[22];

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
void double_integrator_QP_solver_LA_DENSE_2MVMADD_7_17_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *r)
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
void double_integrator_QP_solver_LA_VSUB6_INDEXED_4_4_4(double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *su, int* uidx, double_integrator_QP_solver_FLOAT *v, double_integrator_QP_solver_FLOAT *sv, int* vidx, double_integrator_QP_solver_FLOAT *x)
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
void double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_4_15_4(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *B, double_integrator_QP_solver_FLOAT *u, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	for( i=0; i<4; i++ ){
		r[i] = A[k++]*x[0] + B[i]*u[i];
	}	

	for( j=1; j<15; j++ ){		
		for( i=0; i<4; i++ ){
			r[i] += A[k++]*x[j];
		}
	}
	
}


/*
 * Vector subtraction z = x - y for vectors of length 240.
 */
void double_integrator_QP_solver_LA_VSUB_240(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y, double_integrator_QP_solver_FLOAT *z)
{
	int i;
	for( i=0; i<240; i++){
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB5_22_17(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_FLOAT temp[22];

	
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
void double_integrator_QP_solver_LA_DENSE_MVMSUB5_22_15(double_integrator_QP_solver_FLOAT *A, double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *b, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r)
{
	int i;
	int j;
	int k = 0;

	double_integrator_QP_solver_FLOAT temp[22];

	
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


/*
 * Computes ds = -l.\(r + s.*dl) for vectors of length 433.
 */
void double_integrator_QP_solver_LA_VSUB7_433(double_integrator_QP_solver_FLOAT *l, double_integrator_QP_solver_FLOAT *r, double_integrator_QP_solver_FLOAT *s, double_integrator_QP_solver_FLOAT *dl, double_integrator_QP_solver_FLOAT *ds)
{
	int i;
	for( i=0; i<433; i++){
		ds[i] = -(r[i] + s[i]*dl[i])/l[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 240.
 */
void double_integrator_QP_solver_LA_VADD_240(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<240; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 95.
 */
void double_integrator_QP_solver_LA_VADD_95(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<95; i++){
		x[i] += y[i];
	}
}


/*
 * Vector addition x = x + y for vectors of length 433.
 */
void double_integrator_QP_solver_LA_VADD_433(double_integrator_QP_solver_FLOAT *x, double_integrator_QP_solver_FLOAT *y)
{
	int i;
	for( i=0; i<433; i++){
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
        for( i=0; i<433; i++ ){
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
        if( i == 433 ){
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
    for( i=0; i<240; i++ ){
        z[i] += a_gamma*dz[i];
    }
    
    /* equality constraint multipliers */
    for( i=0; i<95; i++ ){
        v[i] += a_gamma*dv[i];
    }
    
    /* inequality constraint multipliers & slacks, also update mu */
    *mu = 0;
    for( i=0; i<433; i++ ){
        dltemp = l[i] + a_gamma*dl[i]; l[i] = dltemp;
        dstemp = s[i] + a_gamma*ds[i]; s[i] = dstemp;
        *mu += dltemp*dstemp;
    }
    
    *a = a_gamma;
    *mu /= (double_integrator_QP_solver_FLOAT)433;
    return lsIt;
}




/* VARIABLE DEFINITIONS ------------------------------------------------ */
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_z[240];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_v[95];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dz_aff[240];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dv_aff[95];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_grad_cost[240];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_grad_eq[240];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rd[240];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_l[433];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_s[433];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_lbys[433];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dl_aff[433];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ds_aff[433];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dz_cc[240];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dv_cc[95];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_dl_cc[433];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ds_cc[433];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ccrhs[433];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_grad_ineq[240];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_H00[17] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z00 = double_integrator_QP_solver_z + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff00 = double_integrator_QP_solver_dz_aff + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc00 = double_integrator_QP_solver_dz_cc + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd00 = double_integrator_QP_solver_rd + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd00[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost00 = double_integrator_QP_solver_grad_cost + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq00 = double_integrator_QP_solver_grad_eq + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq00 = double_integrator_QP_solver_grad_ineq + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv00[17];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_C00[119] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
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
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v00 = double_integrator_QP_solver_v + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re00[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta00[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc00[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff00 = double_integrator_QP_solver_dv_aff + 0;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc00 = double_integrator_QP_solver_dv_cc + 0;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V00[119];
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
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip00[22];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi00[153];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z01 = double_integrator_QP_solver_z + 17;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff01 = double_integrator_QP_solver_dz_aff + 17;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc01 = double_integrator_QP_solver_dz_cc + 17;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd01 = double_integrator_QP_solver_rd + 17;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd01[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost01 = double_integrator_QP_solver_grad_cost + 17;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq01 = double_integrator_QP_solver_grad_eq + 17;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq01 = double_integrator_QP_solver_grad_ineq + 17;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv01[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v01 = double_integrator_QP_solver_v + 7;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re01[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta01[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc01[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff01 = double_integrator_QP_solver_dv_aff + 7;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc01 = double_integrator_QP_solver_dv_cc + 7;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V01[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd01[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld01[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy01[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy01[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c01[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx01[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb01 = double_integrator_QP_solver_l + 35;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb01 = double_integrator_QP_solver_s + 35;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb01 = double_integrator_QP_solver_lbys + 35;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb01[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff01 = double_integrator_QP_solver_dl_aff + 35;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff01 = double_integrator_QP_solver_ds_aff + 35;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc01 = double_integrator_QP_solver_dl_cc + 35;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc01 = double_integrator_QP_solver_ds_cc + 35;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl01 = double_integrator_QP_solver_ccrhs + 35;
int double_integrator_QP_solver_ubIdx01[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub01 = double_integrator_QP_solver_l + 39;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub01 = double_integrator_QP_solver_s + 39;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub01 = double_integrator_QP_solver_lbys + 39;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub01[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff01 = double_integrator_QP_solver_dl_aff + 39;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff01 = double_integrator_QP_solver_ds_aff + 39;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc01 = double_integrator_QP_solver_dl_cc + 39;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc01 = double_integrator_QP_solver_ds_cc + 39;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub01 = double_integrator_QP_solver_ccrhs + 39;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp01 = double_integrator_QP_solver_s + 43;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp01 = double_integrator_QP_solver_l + 43;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp01 = double_integrator_QP_solver_lbys + 43;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff01 = double_integrator_QP_solver_dl_aff + 43;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff01 = double_integrator_QP_solver_ds_aff + 43;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc01 = double_integrator_QP_solver_dl_cc + 43;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc01 = double_integrator_QP_solver_ds_cc + 43;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp01 = double_integrator_QP_solver_ccrhs + 43;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip01[22];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi01[153];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_D01[119] = {-1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
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
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W01[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd01[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd01[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z02 = double_integrator_QP_solver_z + 34;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff02 = double_integrator_QP_solver_dz_aff + 34;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc02 = double_integrator_QP_solver_dz_cc + 34;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd02 = double_integrator_QP_solver_rd + 34;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd02[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost02 = double_integrator_QP_solver_grad_cost + 34;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq02 = double_integrator_QP_solver_grad_eq + 34;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq02 = double_integrator_QP_solver_grad_ineq + 34;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv02[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v02 = double_integrator_QP_solver_v + 14;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re02[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta02[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc02[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff02 = double_integrator_QP_solver_dv_aff + 14;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc02 = double_integrator_QP_solver_dv_cc + 14;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V02[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd02[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld02[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy02[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy02[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c02[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx02[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb02 = double_integrator_QP_solver_l + 65;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb02 = double_integrator_QP_solver_s + 65;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb02 = double_integrator_QP_solver_lbys + 65;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb02[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff02 = double_integrator_QP_solver_dl_aff + 65;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff02 = double_integrator_QP_solver_ds_aff + 65;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc02 = double_integrator_QP_solver_dl_cc + 65;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc02 = double_integrator_QP_solver_ds_cc + 65;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl02 = double_integrator_QP_solver_ccrhs + 65;
int double_integrator_QP_solver_ubIdx02[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub02 = double_integrator_QP_solver_l + 69;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub02 = double_integrator_QP_solver_s + 69;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub02 = double_integrator_QP_solver_lbys + 69;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub02[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff02 = double_integrator_QP_solver_dl_aff + 69;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff02 = double_integrator_QP_solver_ds_aff + 69;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc02 = double_integrator_QP_solver_dl_cc + 69;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc02 = double_integrator_QP_solver_ds_cc + 69;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub02 = double_integrator_QP_solver_ccrhs + 69;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp02 = double_integrator_QP_solver_s + 73;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp02 = double_integrator_QP_solver_l + 73;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp02 = double_integrator_QP_solver_lbys + 73;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff02 = double_integrator_QP_solver_dl_aff + 73;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff02 = double_integrator_QP_solver_ds_aff + 73;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc02 = double_integrator_QP_solver_dl_cc + 73;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc02 = double_integrator_QP_solver_ds_cc + 73;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp02 = double_integrator_QP_solver_ccrhs + 73;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip02[22];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi02[153];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W02[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd02[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd02[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z03 = double_integrator_QP_solver_z + 51;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff03 = double_integrator_QP_solver_dz_aff + 51;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc03 = double_integrator_QP_solver_dz_cc + 51;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd03 = double_integrator_QP_solver_rd + 51;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd03[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost03 = double_integrator_QP_solver_grad_cost + 51;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq03 = double_integrator_QP_solver_grad_eq + 51;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq03 = double_integrator_QP_solver_grad_ineq + 51;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv03[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v03 = double_integrator_QP_solver_v + 21;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re03[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta03[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc03[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff03 = double_integrator_QP_solver_dv_aff + 21;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc03 = double_integrator_QP_solver_dv_cc + 21;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V03[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd03[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld03[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy03[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy03[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c03[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx03[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb03 = double_integrator_QP_solver_l + 95;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb03 = double_integrator_QP_solver_s + 95;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb03 = double_integrator_QP_solver_lbys + 95;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb03[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff03 = double_integrator_QP_solver_dl_aff + 95;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff03 = double_integrator_QP_solver_ds_aff + 95;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc03 = double_integrator_QP_solver_dl_cc + 95;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc03 = double_integrator_QP_solver_ds_cc + 95;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl03 = double_integrator_QP_solver_ccrhs + 95;
int double_integrator_QP_solver_ubIdx03[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub03 = double_integrator_QP_solver_l + 99;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub03 = double_integrator_QP_solver_s + 99;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub03 = double_integrator_QP_solver_lbys + 99;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub03[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff03 = double_integrator_QP_solver_dl_aff + 99;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff03 = double_integrator_QP_solver_ds_aff + 99;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc03 = double_integrator_QP_solver_dl_cc + 99;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc03 = double_integrator_QP_solver_ds_cc + 99;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub03 = double_integrator_QP_solver_ccrhs + 99;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp03 = double_integrator_QP_solver_s + 103;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp03 = double_integrator_QP_solver_l + 103;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp03 = double_integrator_QP_solver_lbys + 103;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff03 = double_integrator_QP_solver_dl_aff + 103;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff03 = double_integrator_QP_solver_ds_aff + 103;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc03 = double_integrator_QP_solver_dl_cc + 103;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc03 = double_integrator_QP_solver_ds_cc + 103;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp03 = double_integrator_QP_solver_ccrhs + 103;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip03[22];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi03[153];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W03[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd03[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd03[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z04 = double_integrator_QP_solver_z + 68;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff04 = double_integrator_QP_solver_dz_aff + 68;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc04 = double_integrator_QP_solver_dz_cc + 68;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd04 = double_integrator_QP_solver_rd + 68;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd04[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost04 = double_integrator_QP_solver_grad_cost + 68;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq04 = double_integrator_QP_solver_grad_eq + 68;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq04 = double_integrator_QP_solver_grad_ineq + 68;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv04[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v04 = double_integrator_QP_solver_v + 28;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re04[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta04[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc04[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff04 = double_integrator_QP_solver_dv_aff + 28;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc04 = double_integrator_QP_solver_dv_cc + 28;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V04[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd04[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld04[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy04[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy04[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c04[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx04[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb04 = double_integrator_QP_solver_l + 125;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb04 = double_integrator_QP_solver_s + 125;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb04 = double_integrator_QP_solver_lbys + 125;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb04[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff04 = double_integrator_QP_solver_dl_aff + 125;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff04 = double_integrator_QP_solver_ds_aff + 125;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc04 = double_integrator_QP_solver_dl_cc + 125;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc04 = double_integrator_QP_solver_ds_cc + 125;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl04 = double_integrator_QP_solver_ccrhs + 125;
int double_integrator_QP_solver_ubIdx04[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub04 = double_integrator_QP_solver_l + 129;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub04 = double_integrator_QP_solver_s + 129;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub04 = double_integrator_QP_solver_lbys + 129;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub04[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff04 = double_integrator_QP_solver_dl_aff + 129;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff04 = double_integrator_QP_solver_ds_aff + 129;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc04 = double_integrator_QP_solver_dl_cc + 129;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc04 = double_integrator_QP_solver_ds_cc + 129;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub04 = double_integrator_QP_solver_ccrhs + 129;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp04 = double_integrator_QP_solver_s + 133;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp04 = double_integrator_QP_solver_l + 133;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp04 = double_integrator_QP_solver_lbys + 133;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff04 = double_integrator_QP_solver_dl_aff + 133;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff04 = double_integrator_QP_solver_ds_aff + 133;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc04 = double_integrator_QP_solver_dl_cc + 133;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc04 = double_integrator_QP_solver_ds_cc + 133;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp04 = double_integrator_QP_solver_ccrhs + 133;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip04[22];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi04[153];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W04[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd04[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd04[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z05 = double_integrator_QP_solver_z + 85;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff05 = double_integrator_QP_solver_dz_aff + 85;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc05 = double_integrator_QP_solver_dz_cc + 85;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd05 = double_integrator_QP_solver_rd + 85;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd05[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost05 = double_integrator_QP_solver_grad_cost + 85;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq05 = double_integrator_QP_solver_grad_eq + 85;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq05 = double_integrator_QP_solver_grad_ineq + 85;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv05[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v05 = double_integrator_QP_solver_v + 35;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re05[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta05[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc05[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff05 = double_integrator_QP_solver_dv_aff + 35;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc05 = double_integrator_QP_solver_dv_cc + 35;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V05[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd05[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld05[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy05[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy05[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c05[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx05[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb05 = double_integrator_QP_solver_l + 155;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb05 = double_integrator_QP_solver_s + 155;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb05 = double_integrator_QP_solver_lbys + 155;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb05[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff05 = double_integrator_QP_solver_dl_aff + 155;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff05 = double_integrator_QP_solver_ds_aff + 155;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc05 = double_integrator_QP_solver_dl_cc + 155;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc05 = double_integrator_QP_solver_ds_cc + 155;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl05 = double_integrator_QP_solver_ccrhs + 155;
int double_integrator_QP_solver_ubIdx05[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub05 = double_integrator_QP_solver_l + 159;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub05 = double_integrator_QP_solver_s + 159;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub05 = double_integrator_QP_solver_lbys + 159;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub05[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff05 = double_integrator_QP_solver_dl_aff + 159;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff05 = double_integrator_QP_solver_ds_aff + 159;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc05 = double_integrator_QP_solver_dl_cc + 159;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc05 = double_integrator_QP_solver_ds_cc + 159;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub05 = double_integrator_QP_solver_ccrhs + 159;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp05 = double_integrator_QP_solver_s + 163;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp05 = double_integrator_QP_solver_l + 163;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp05 = double_integrator_QP_solver_lbys + 163;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff05 = double_integrator_QP_solver_dl_aff + 163;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff05 = double_integrator_QP_solver_ds_aff + 163;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc05 = double_integrator_QP_solver_dl_cc + 163;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc05 = double_integrator_QP_solver_ds_cc + 163;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp05 = double_integrator_QP_solver_ccrhs + 163;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip05[22];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi05[153];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W05[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd05[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd05[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z06 = double_integrator_QP_solver_z + 102;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff06 = double_integrator_QP_solver_dz_aff + 102;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc06 = double_integrator_QP_solver_dz_cc + 102;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd06 = double_integrator_QP_solver_rd + 102;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd06[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost06 = double_integrator_QP_solver_grad_cost + 102;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq06 = double_integrator_QP_solver_grad_eq + 102;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq06 = double_integrator_QP_solver_grad_ineq + 102;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv06[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v06 = double_integrator_QP_solver_v + 42;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re06[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta06[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc06[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff06 = double_integrator_QP_solver_dv_aff + 42;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc06 = double_integrator_QP_solver_dv_cc + 42;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V06[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd06[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld06[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy06[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy06[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c06[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx06[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb06 = double_integrator_QP_solver_l + 185;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb06 = double_integrator_QP_solver_s + 185;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb06 = double_integrator_QP_solver_lbys + 185;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb06[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff06 = double_integrator_QP_solver_dl_aff + 185;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff06 = double_integrator_QP_solver_ds_aff + 185;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc06 = double_integrator_QP_solver_dl_cc + 185;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc06 = double_integrator_QP_solver_ds_cc + 185;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl06 = double_integrator_QP_solver_ccrhs + 185;
int double_integrator_QP_solver_ubIdx06[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub06 = double_integrator_QP_solver_l + 189;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub06 = double_integrator_QP_solver_s + 189;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub06 = double_integrator_QP_solver_lbys + 189;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub06[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff06 = double_integrator_QP_solver_dl_aff + 189;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff06 = double_integrator_QP_solver_ds_aff + 189;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc06 = double_integrator_QP_solver_dl_cc + 189;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc06 = double_integrator_QP_solver_ds_cc + 189;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub06 = double_integrator_QP_solver_ccrhs + 189;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp06 = double_integrator_QP_solver_s + 193;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp06 = double_integrator_QP_solver_l + 193;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp06 = double_integrator_QP_solver_lbys + 193;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff06 = double_integrator_QP_solver_dl_aff + 193;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff06 = double_integrator_QP_solver_ds_aff + 193;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc06 = double_integrator_QP_solver_dl_cc + 193;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc06 = double_integrator_QP_solver_ds_cc + 193;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp06 = double_integrator_QP_solver_ccrhs + 193;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip06[22];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi06[153];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W06[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd06[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd06[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z07 = double_integrator_QP_solver_z + 119;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff07 = double_integrator_QP_solver_dz_aff + 119;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc07 = double_integrator_QP_solver_dz_cc + 119;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd07 = double_integrator_QP_solver_rd + 119;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd07[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost07 = double_integrator_QP_solver_grad_cost + 119;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq07 = double_integrator_QP_solver_grad_eq + 119;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq07 = double_integrator_QP_solver_grad_ineq + 119;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv07[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v07 = double_integrator_QP_solver_v + 49;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re07[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta07[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc07[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff07 = double_integrator_QP_solver_dv_aff + 49;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc07 = double_integrator_QP_solver_dv_cc + 49;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V07[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd07[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld07[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy07[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy07[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c07[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx07[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb07 = double_integrator_QP_solver_l + 215;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb07 = double_integrator_QP_solver_s + 215;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb07 = double_integrator_QP_solver_lbys + 215;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb07[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff07 = double_integrator_QP_solver_dl_aff + 215;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff07 = double_integrator_QP_solver_ds_aff + 215;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc07 = double_integrator_QP_solver_dl_cc + 215;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc07 = double_integrator_QP_solver_ds_cc + 215;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl07 = double_integrator_QP_solver_ccrhs + 215;
int double_integrator_QP_solver_ubIdx07[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub07 = double_integrator_QP_solver_l + 219;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub07 = double_integrator_QP_solver_s + 219;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub07 = double_integrator_QP_solver_lbys + 219;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub07[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff07 = double_integrator_QP_solver_dl_aff + 219;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff07 = double_integrator_QP_solver_ds_aff + 219;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc07 = double_integrator_QP_solver_dl_cc + 219;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc07 = double_integrator_QP_solver_ds_cc + 219;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub07 = double_integrator_QP_solver_ccrhs + 219;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp07 = double_integrator_QP_solver_s + 223;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp07 = double_integrator_QP_solver_l + 223;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp07 = double_integrator_QP_solver_lbys + 223;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff07 = double_integrator_QP_solver_dl_aff + 223;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff07 = double_integrator_QP_solver_ds_aff + 223;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc07 = double_integrator_QP_solver_dl_cc + 223;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc07 = double_integrator_QP_solver_ds_cc + 223;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp07 = double_integrator_QP_solver_ccrhs + 223;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip07[22];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi07[153];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W07[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd07[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd07[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z08 = double_integrator_QP_solver_z + 136;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff08 = double_integrator_QP_solver_dz_aff + 136;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc08 = double_integrator_QP_solver_dz_cc + 136;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd08 = double_integrator_QP_solver_rd + 136;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd08[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost08 = double_integrator_QP_solver_grad_cost + 136;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq08 = double_integrator_QP_solver_grad_eq + 136;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq08 = double_integrator_QP_solver_grad_ineq + 136;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv08[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v08 = double_integrator_QP_solver_v + 56;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re08[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta08[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc08[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff08 = double_integrator_QP_solver_dv_aff + 56;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc08 = double_integrator_QP_solver_dv_cc + 56;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V08[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd08[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld08[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy08[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy08[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c08[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx08[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb08 = double_integrator_QP_solver_l + 245;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb08 = double_integrator_QP_solver_s + 245;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb08 = double_integrator_QP_solver_lbys + 245;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb08[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff08 = double_integrator_QP_solver_dl_aff + 245;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff08 = double_integrator_QP_solver_ds_aff + 245;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc08 = double_integrator_QP_solver_dl_cc + 245;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc08 = double_integrator_QP_solver_ds_cc + 245;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl08 = double_integrator_QP_solver_ccrhs + 245;
int double_integrator_QP_solver_ubIdx08[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub08 = double_integrator_QP_solver_l + 249;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub08 = double_integrator_QP_solver_s + 249;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub08 = double_integrator_QP_solver_lbys + 249;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub08[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff08 = double_integrator_QP_solver_dl_aff + 249;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff08 = double_integrator_QP_solver_ds_aff + 249;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc08 = double_integrator_QP_solver_dl_cc + 249;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc08 = double_integrator_QP_solver_ds_cc + 249;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub08 = double_integrator_QP_solver_ccrhs + 249;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp08 = double_integrator_QP_solver_s + 253;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp08 = double_integrator_QP_solver_l + 253;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp08 = double_integrator_QP_solver_lbys + 253;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff08 = double_integrator_QP_solver_dl_aff + 253;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff08 = double_integrator_QP_solver_ds_aff + 253;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc08 = double_integrator_QP_solver_dl_cc + 253;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc08 = double_integrator_QP_solver_ds_cc + 253;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp08 = double_integrator_QP_solver_ccrhs + 253;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip08[22];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi08[153];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W08[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd08[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd08[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z09 = double_integrator_QP_solver_z + 153;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff09 = double_integrator_QP_solver_dz_aff + 153;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc09 = double_integrator_QP_solver_dz_cc + 153;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd09 = double_integrator_QP_solver_rd + 153;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd09[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost09 = double_integrator_QP_solver_grad_cost + 153;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq09 = double_integrator_QP_solver_grad_eq + 153;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq09 = double_integrator_QP_solver_grad_ineq + 153;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv09[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v09 = double_integrator_QP_solver_v + 63;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re09[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta09[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc09[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff09 = double_integrator_QP_solver_dv_aff + 63;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc09 = double_integrator_QP_solver_dv_cc + 63;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V09[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd09[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld09[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy09[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy09[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c09[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx09[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb09 = double_integrator_QP_solver_l + 275;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb09 = double_integrator_QP_solver_s + 275;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb09 = double_integrator_QP_solver_lbys + 275;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb09[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff09 = double_integrator_QP_solver_dl_aff + 275;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff09 = double_integrator_QP_solver_ds_aff + 275;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc09 = double_integrator_QP_solver_dl_cc + 275;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc09 = double_integrator_QP_solver_ds_cc + 275;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl09 = double_integrator_QP_solver_ccrhs + 275;
int double_integrator_QP_solver_ubIdx09[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub09 = double_integrator_QP_solver_l + 279;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub09 = double_integrator_QP_solver_s + 279;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub09 = double_integrator_QP_solver_lbys + 279;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub09[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff09 = double_integrator_QP_solver_dl_aff + 279;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff09 = double_integrator_QP_solver_ds_aff + 279;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc09 = double_integrator_QP_solver_dl_cc + 279;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc09 = double_integrator_QP_solver_ds_cc + 279;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub09 = double_integrator_QP_solver_ccrhs + 279;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp09 = double_integrator_QP_solver_s + 283;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp09 = double_integrator_QP_solver_l + 283;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp09 = double_integrator_QP_solver_lbys + 283;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff09 = double_integrator_QP_solver_dl_aff + 283;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff09 = double_integrator_QP_solver_ds_aff + 283;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc09 = double_integrator_QP_solver_dl_cc + 283;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc09 = double_integrator_QP_solver_ds_cc + 283;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp09 = double_integrator_QP_solver_ccrhs + 283;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip09[22];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi09[153];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W09[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd09[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd09[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z10 = double_integrator_QP_solver_z + 170;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff10 = double_integrator_QP_solver_dz_aff + 170;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc10 = double_integrator_QP_solver_dz_cc + 170;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd10 = double_integrator_QP_solver_rd + 170;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd10[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost10 = double_integrator_QP_solver_grad_cost + 170;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq10 = double_integrator_QP_solver_grad_eq + 170;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq10 = double_integrator_QP_solver_grad_ineq + 170;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv10[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v10 = double_integrator_QP_solver_v + 70;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re10[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta10[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc10[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff10 = double_integrator_QP_solver_dv_aff + 70;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc10 = double_integrator_QP_solver_dv_cc + 70;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V10[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd10[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld10[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy10[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy10[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c10[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx10[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb10 = double_integrator_QP_solver_l + 305;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb10 = double_integrator_QP_solver_s + 305;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb10 = double_integrator_QP_solver_lbys + 305;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb10[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff10 = double_integrator_QP_solver_dl_aff + 305;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff10 = double_integrator_QP_solver_ds_aff + 305;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc10 = double_integrator_QP_solver_dl_cc + 305;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc10 = double_integrator_QP_solver_ds_cc + 305;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl10 = double_integrator_QP_solver_ccrhs + 305;
int double_integrator_QP_solver_ubIdx10[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub10 = double_integrator_QP_solver_l + 309;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub10 = double_integrator_QP_solver_s + 309;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub10 = double_integrator_QP_solver_lbys + 309;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub10[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff10 = double_integrator_QP_solver_dl_aff + 309;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff10 = double_integrator_QP_solver_ds_aff + 309;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc10 = double_integrator_QP_solver_dl_cc + 309;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc10 = double_integrator_QP_solver_ds_cc + 309;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub10 = double_integrator_QP_solver_ccrhs + 309;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp10 = double_integrator_QP_solver_s + 313;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp10 = double_integrator_QP_solver_l + 313;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp10 = double_integrator_QP_solver_lbys + 313;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff10 = double_integrator_QP_solver_dl_aff + 313;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff10 = double_integrator_QP_solver_ds_aff + 313;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc10 = double_integrator_QP_solver_dl_cc + 313;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc10 = double_integrator_QP_solver_ds_cc + 313;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp10 = double_integrator_QP_solver_ccrhs + 313;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip10[22];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi10[153];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W10[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd10[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd10[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z11 = double_integrator_QP_solver_z + 187;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff11 = double_integrator_QP_solver_dz_aff + 187;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc11 = double_integrator_QP_solver_dz_cc + 187;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd11 = double_integrator_QP_solver_rd + 187;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd11[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost11 = double_integrator_QP_solver_grad_cost + 187;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq11 = double_integrator_QP_solver_grad_eq + 187;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq11 = double_integrator_QP_solver_grad_ineq + 187;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv11[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v11 = double_integrator_QP_solver_v + 77;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re11[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta11[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc11[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff11 = double_integrator_QP_solver_dv_aff + 77;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc11 = double_integrator_QP_solver_dv_cc + 77;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V11[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd11[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld11[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy11[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy11[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c11[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx11[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb11 = double_integrator_QP_solver_l + 335;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb11 = double_integrator_QP_solver_s + 335;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb11 = double_integrator_QP_solver_lbys + 335;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb11[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff11 = double_integrator_QP_solver_dl_aff + 335;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff11 = double_integrator_QP_solver_ds_aff + 335;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc11 = double_integrator_QP_solver_dl_cc + 335;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc11 = double_integrator_QP_solver_ds_cc + 335;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl11 = double_integrator_QP_solver_ccrhs + 335;
int double_integrator_QP_solver_ubIdx11[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub11 = double_integrator_QP_solver_l + 339;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub11 = double_integrator_QP_solver_s + 339;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub11 = double_integrator_QP_solver_lbys + 339;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub11[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff11 = double_integrator_QP_solver_dl_aff + 339;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff11 = double_integrator_QP_solver_ds_aff + 339;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc11 = double_integrator_QP_solver_dl_cc + 339;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc11 = double_integrator_QP_solver_ds_cc + 339;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub11 = double_integrator_QP_solver_ccrhs + 339;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp11 = double_integrator_QP_solver_s + 343;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp11 = double_integrator_QP_solver_l + 343;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp11 = double_integrator_QP_solver_lbys + 343;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff11 = double_integrator_QP_solver_dl_aff + 343;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff11 = double_integrator_QP_solver_ds_aff + 343;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc11 = double_integrator_QP_solver_dl_cc + 343;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc11 = double_integrator_QP_solver_ds_cc + 343;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp11 = double_integrator_QP_solver_ccrhs + 343;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip11[22];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi11[153];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W11[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd11[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd11[49];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z12 = double_integrator_QP_solver_z + 204;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff12 = double_integrator_QP_solver_dz_aff + 204;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc12 = double_integrator_QP_solver_dz_cc + 204;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd12 = double_integrator_QP_solver_rd + 204;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd12[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost12 = double_integrator_QP_solver_grad_cost + 204;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq12 = double_integrator_QP_solver_grad_eq + 204;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq12 = double_integrator_QP_solver_grad_ineq + 204;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv12[17];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v12 = double_integrator_QP_solver_v + 84;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re12[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta12[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc12[7];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff12 = double_integrator_QP_solver_dv_aff + 84;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc12 = double_integrator_QP_solver_dv_cc + 84;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V12[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd12[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld12[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy12[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy12[7];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c12[7] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx12[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb12 = double_integrator_QP_solver_l + 365;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb12 = double_integrator_QP_solver_s + 365;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb12 = double_integrator_QP_solver_lbys + 365;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb12[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff12 = double_integrator_QP_solver_dl_aff + 365;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff12 = double_integrator_QP_solver_ds_aff + 365;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc12 = double_integrator_QP_solver_dl_cc + 365;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc12 = double_integrator_QP_solver_ds_cc + 365;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl12 = double_integrator_QP_solver_ccrhs + 365;
int double_integrator_QP_solver_ubIdx12[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub12 = double_integrator_QP_solver_l + 369;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub12 = double_integrator_QP_solver_s + 369;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub12 = double_integrator_QP_solver_lbys + 369;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub12[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff12 = double_integrator_QP_solver_dl_aff + 369;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff12 = double_integrator_QP_solver_ds_aff + 369;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc12 = double_integrator_QP_solver_dl_cc + 369;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc12 = double_integrator_QP_solver_ds_cc + 369;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub12 = double_integrator_QP_solver_ccrhs + 369;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp12 = double_integrator_QP_solver_s + 373;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp12 = double_integrator_QP_solver_l + 373;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp12 = double_integrator_QP_solver_lbys + 373;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff12 = double_integrator_QP_solver_dl_aff + 373;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff12 = double_integrator_QP_solver_ds_aff + 373;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc12 = double_integrator_QP_solver_dl_cc + 373;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc12 = double_integrator_QP_solver_ds_cc + 373;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp12 = double_integrator_QP_solver_ccrhs + 373;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip12[22];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi12[153];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W12[119];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd12[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd12[49];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_H13[15] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z13 = double_integrator_QP_solver_z + 221;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff13 = double_integrator_QP_solver_dz_aff + 221;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc13 = double_integrator_QP_solver_dz_cc + 221;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd13 = double_integrator_QP_solver_rd + 221;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd13[15];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost13 = double_integrator_QP_solver_grad_cost + 221;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq13 = double_integrator_QP_solver_grad_eq + 221;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq13 = double_integrator_QP_solver_grad_ineq + 221;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv13[15];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_C13[60] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
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
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_v13 = double_integrator_QP_solver_v + 91;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_re13[4];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_beta13[4];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_betacc13[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvaff13 = double_integrator_QP_solver_dv_aff + 91;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dvcc13 = double_integrator_QP_solver_dv_cc + 91;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_V13[60];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Yd13[10];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ld13[10];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_yy13[4];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_bmy13[4];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_c13[4] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
int double_integrator_QP_solver_lbIdx13[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb13 = double_integrator_QP_solver_l + 395;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb13 = double_integrator_QP_solver_s + 395;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb13 = double_integrator_QP_solver_lbys + 395;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb13[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff13 = double_integrator_QP_solver_dl_aff + 395;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff13 = double_integrator_QP_solver_ds_aff + 395;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc13 = double_integrator_QP_solver_dl_cc + 395;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc13 = double_integrator_QP_solver_ds_cc + 395;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl13 = double_integrator_QP_solver_ccrhs + 395;
int double_integrator_QP_solver_ubIdx13[4] = {2, 3, 4, 5};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub13 = double_integrator_QP_solver_l + 399;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub13 = double_integrator_QP_solver_s + 399;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub13 = double_integrator_QP_solver_lbys + 399;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub13[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff13 = double_integrator_QP_solver_dl_aff + 399;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff13 = double_integrator_QP_solver_ds_aff + 399;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc13 = double_integrator_QP_solver_dl_cc + 399;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc13 = double_integrator_QP_solver_ds_cc + 399;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub13 = double_integrator_QP_solver_ccrhs + 399;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sp13 = double_integrator_QP_solver_s + 403;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lp13 = double_integrator_QP_solver_l + 403;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lpbysp13 = double_integrator_QP_solver_lbys + 403;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_aff13 = double_integrator_QP_solver_dl_aff + 403;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_aff13 = double_integrator_QP_solver_ds_aff + 403;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlp_cc13 = double_integrator_QP_solver_dl_cc + 403;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsp_cc13 = double_integrator_QP_solver_ds_cc + 403;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsp13 = double_integrator_QP_solver_ccrhs + 403;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rip13[22];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi13[120];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_D13[105] = {-1.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 
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
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W13[105];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Ysd13[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lsd13[28];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_H14[5] = {0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000, 0.0000000000000000E+000};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_z14 = double_integrator_QP_solver_z + 236;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzaff14 = double_integrator_QP_solver_dz_aff + 236;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dzcc14 = double_integrator_QP_solver_dz_cc + 236;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_rd14 = double_integrator_QP_solver_rd + 236;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Lbyrd14[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_cost14 = double_integrator_QP_solver_grad_cost + 236;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_eq14 = double_integrator_QP_solver_grad_eq + 236;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_grad_ineq14 = double_integrator_QP_solver_grad_ineq + 236;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_ctv14[4];
int double_integrator_QP_solver_lbIdx14[4] = {0, 1, 2, 3};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llb14 = double_integrator_QP_solver_l + 425;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_slb14 = double_integrator_QP_solver_s + 425;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_llbbyslb14 = double_integrator_QP_solver_lbys + 425;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_rilb14[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbaff14 = double_integrator_QP_solver_dl_aff + 425;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbaff14 = double_integrator_QP_solver_ds_aff + 425;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dllbcc14 = double_integrator_QP_solver_dl_cc + 425;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dslbcc14 = double_integrator_QP_solver_ds_cc + 425;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsl14 = double_integrator_QP_solver_ccrhs + 425;
int double_integrator_QP_solver_ubIdx14[4] = {0, 1, 2, 3};
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lub14 = double_integrator_QP_solver_l + 429;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_sub14 = double_integrator_QP_solver_s + 429;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_lubbysub14 = double_integrator_QP_solver_lbys + 429;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_riub14[4];
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubaff14 = double_integrator_QP_solver_dl_aff + 429;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubaff14 = double_integrator_QP_solver_ds_aff + 429;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dlubcc14 = double_integrator_QP_solver_dl_cc + 429;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_dsubcc14 = double_integrator_QP_solver_ds_cc + 429;
double_integrator_QP_solver_FLOAT* double_integrator_QP_solver_ccrhsub14 = double_integrator_QP_solver_ccrhs + 429;
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_Phi14[4];
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_D14[4] = {-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000, 
-1.0000000000000000E+000};
double_integrator_QP_solver_FLOAT double_integrator_QP_solver_W14[4];
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
double_integrator_QP_solver_LA_INITIALIZEVECTOR_240(double_integrator_QP_solver_z, 0);
double_integrator_QP_solver_LA_INITIALIZEVECTOR_95(double_integrator_QP_solver_v, 1);
double_integrator_QP_solver_LA_INITIALIZEVECTOR_433(double_integrator_QP_solver_l, 10);
double_integrator_QP_solver_LA_INITIALIZEVECTOR_433(double_integrator_QP_solver_s, 10);
info->mu = 0;
double_integrator_QP_solver_LA_DOTACC_433(double_integrator_QP_solver_l, double_integrator_QP_solver_s, &info->mu);
info->mu /= 433;
while( 1 ){
info->pobj = 0;
double_integrator_QP_solver_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_H00, params->f1, double_integrator_QP_solver_z00, double_integrator_QP_solver_grad_cost00, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_H00, params->f2, double_integrator_QP_solver_z01, double_integrator_QP_solver_grad_cost01, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_H00, params->f3, double_integrator_QP_solver_z02, double_integrator_QP_solver_grad_cost02, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_H00, params->f4, double_integrator_QP_solver_z03, double_integrator_QP_solver_grad_cost03, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_H00, params->f5, double_integrator_QP_solver_z04, double_integrator_QP_solver_grad_cost04, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_H00, params->f6, double_integrator_QP_solver_z05, double_integrator_QP_solver_grad_cost05, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_H00, params->f7, double_integrator_QP_solver_z06, double_integrator_QP_solver_grad_cost06, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_H00, params->f8, double_integrator_QP_solver_z07, double_integrator_QP_solver_grad_cost07, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_H00, params->f9, double_integrator_QP_solver_z08, double_integrator_QP_solver_grad_cost08, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_H00, params->f10, double_integrator_QP_solver_z09, double_integrator_QP_solver_grad_cost09, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_H00, params->f11, double_integrator_QP_solver_z10, double_integrator_QP_solver_grad_cost10, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_H00, params->f12, double_integrator_QP_solver_z11, double_integrator_QP_solver_grad_cost11, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_17(double_integrator_QP_solver_H00, params->f13, double_integrator_QP_solver_z12, double_integrator_QP_solver_grad_cost12, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_15(double_integrator_QP_solver_H13, params->f14, double_integrator_QP_solver_z13, double_integrator_QP_solver_grad_cost13, &info->pobj);
double_integrator_QP_solver_LA_DIAG_QUADFCN_4(double_integrator_QP_solver_H14, params->f15, double_integrator_QP_solver_z14, double_integrator_QP_solver_grad_cost14, &info->pobj);
info->res_eq = 0;
info->dgap = 0;
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_C00, double_integrator_QP_solver_z00, double_integrator_QP_solver_D01, double_integrator_QP_solver_z01, double_integrator_QP_solver_c00, double_integrator_QP_solver_v00, double_integrator_QP_solver_re00, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_C00, double_integrator_QP_solver_z01, double_integrator_QP_solver_D01, double_integrator_QP_solver_z02, double_integrator_QP_solver_c01, double_integrator_QP_solver_v01, double_integrator_QP_solver_re01, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_C00, double_integrator_QP_solver_z02, double_integrator_QP_solver_D01, double_integrator_QP_solver_z03, double_integrator_QP_solver_c02, double_integrator_QP_solver_v02, double_integrator_QP_solver_re02, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_C00, double_integrator_QP_solver_z03, double_integrator_QP_solver_D01, double_integrator_QP_solver_z04, double_integrator_QP_solver_c03, double_integrator_QP_solver_v03, double_integrator_QP_solver_re03, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_C00, double_integrator_QP_solver_z04, double_integrator_QP_solver_D01, double_integrator_QP_solver_z05, double_integrator_QP_solver_c04, double_integrator_QP_solver_v04, double_integrator_QP_solver_re04, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_C00, double_integrator_QP_solver_z05, double_integrator_QP_solver_D01, double_integrator_QP_solver_z06, double_integrator_QP_solver_c05, double_integrator_QP_solver_v05, double_integrator_QP_solver_re05, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_C00, double_integrator_QP_solver_z06, double_integrator_QP_solver_D01, double_integrator_QP_solver_z07, double_integrator_QP_solver_c06, double_integrator_QP_solver_v06, double_integrator_QP_solver_re06, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_C00, double_integrator_QP_solver_z07, double_integrator_QP_solver_D01, double_integrator_QP_solver_z08, double_integrator_QP_solver_c07, double_integrator_QP_solver_v07, double_integrator_QP_solver_re07, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_C00, double_integrator_QP_solver_z08, double_integrator_QP_solver_D01, double_integrator_QP_solver_z09, double_integrator_QP_solver_c08, double_integrator_QP_solver_v08, double_integrator_QP_solver_re08, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_C00, double_integrator_QP_solver_z09, double_integrator_QP_solver_D01, double_integrator_QP_solver_z10, double_integrator_QP_solver_c09, double_integrator_QP_solver_v09, double_integrator_QP_solver_re09, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_C00, double_integrator_QP_solver_z10, double_integrator_QP_solver_D01, double_integrator_QP_solver_z11, double_integrator_QP_solver_c10, double_integrator_QP_solver_v10, double_integrator_QP_solver_re10, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_17_17(double_integrator_QP_solver_C00, double_integrator_QP_solver_z11, double_integrator_QP_solver_D01, double_integrator_QP_solver_z12, double_integrator_QP_solver_c11, double_integrator_QP_solver_v11, double_integrator_QP_solver_re11, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MVMSUB3_7_17_15(double_integrator_QP_solver_C00, double_integrator_QP_solver_z12, double_integrator_QP_solver_D13, double_integrator_QP_solver_z13, double_integrator_QP_solver_c12, double_integrator_QP_solver_v12, double_integrator_QP_solver_re12, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MVMSUB3_4_15_4(double_integrator_QP_solver_C13, double_integrator_QP_solver_z13, double_integrator_QP_solver_D14, double_integrator_QP_solver_z14, double_integrator_QP_solver_c13, double_integrator_QP_solver_v13, double_integrator_QP_solver_re13, &info->dgap, &info->res_eq);
double_integrator_QP_solver_LA_DENSE_MTVM_7_17(double_integrator_QP_solver_C00, double_integrator_QP_solver_v00, double_integrator_QP_solver_grad_eq00);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v01, double_integrator_QP_solver_D01, double_integrator_QP_solver_v00, double_integrator_QP_solver_grad_eq01);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v02, double_integrator_QP_solver_D01, double_integrator_QP_solver_v01, double_integrator_QP_solver_grad_eq02);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v03, double_integrator_QP_solver_D01, double_integrator_QP_solver_v02, double_integrator_QP_solver_grad_eq03);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v04, double_integrator_QP_solver_D01, double_integrator_QP_solver_v03, double_integrator_QP_solver_grad_eq04);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v05, double_integrator_QP_solver_D01, double_integrator_QP_solver_v04, double_integrator_QP_solver_grad_eq05);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v06, double_integrator_QP_solver_D01, double_integrator_QP_solver_v05, double_integrator_QP_solver_grad_eq06);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v07, double_integrator_QP_solver_D01, double_integrator_QP_solver_v06, double_integrator_QP_solver_grad_eq07);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v08, double_integrator_QP_solver_D01, double_integrator_QP_solver_v07, double_integrator_QP_solver_grad_eq08);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v09, double_integrator_QP_solver_D01, double_integrator_QP_solver_v08, double_integrator_QP_solver_grad_eq09);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v10, double_integrator_QP_solver_D01, double_integrator_QP_solver_v09, double_integrator_QP_solver_grad_eq10);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v11, double_integrator_QP_solver_D01, double_integrator_QP_solver_v10, double_integrator_QP_solver_grad_eq11);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_v12, double_integrator_QP_solver_D01, double_integrator_QP_solver_v11, double_integrator_QP_solver_grad_eq12);
double_integrator_QP_solver_LA_DENSE_MTVM2_4_15_7(double_integrator_QP_solver_C13, double_integrator_QP_solver_v13, double_integrator_QP_solver_D13, double_integrator_QP_solver_v12, double_integrator_QP_solver_grad_eq13);
double_integrator_QP_solver_LA_DIAGZERO_MTVM_4_4(double_integrator_QP_solver_D14, double_integrator_QP_solver_v13, double_integrator_QP_solver_grad_eq14);
info->res_ineq = 0;
double_integrator_QP_solver_LA_VSUBADD3_7(params->lb1, double_integrator_QP_solver_z00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_llb00, double_integrator_QP_solver_slb00, double_integrator_QP_solver_rilb00, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_6(double_integrator_QP_solver_z00, double_integrator_QP_solver_ubIdx00, params->ub1, double_integrator_QP_solver_lub00, double_integrator_QP_solver_sub00, double_integrator_QP_solver_riub00, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_22_17(params->A1, double_integrator_QP_solver_z00, params->b1, double_integrator_QP_solver_sp00, double_integrator_QP_solver_lp00, double_integrator_QP_solver_rip00, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_4(params->lb2, double_integrator_QP_solver_z01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_llb01, double_integrator_QP_solver_slb01, double_integrator_QP_solver_rilb01, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_4(double_integrator_QP_solver_z01, double_integrator_QP_solver_ubIdx01, params->ub2, double_integrator_QP_solver_lub01, double_integrator_QP_solver_sub01, double_integrator_QP_solver_riub01, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_22_17(params->A2, double_integrator_QP_solver_z01, params->b2, double_integrator_QP_solver_sp01, double_integrator_QP_solver_lp01, double_integrator_QP_solver_rip01, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_4(params->lb3, double_integrator_QP_solver_z02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_llb02, double_integrator_QP_solver_slb02, double_integrator_QP_solver_rilb02, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_4(double_integrator_QP_solver_z02, double_integrator_QP_solver_ubIdx02, params->ub3, double_integrator_QP_solver_lub02, double_integrator_QP_solver_sub02, double_integrator_QP_solver_riub02, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_22_17(params->A3, double_integrator_QP_solver_z02, params->b3, double_integrator_QP_solver_sp02, double_integrator_QP_solver_lp02, double_integrator_QP_solver_rip02, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_4(params->lb4, double_integrator_QP_solver_z03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_llb03, double_integrator_QP_solver_slb03, double_integrator_QP_solver_rilb03, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_4(double_integrator_QP_solver_z03, double_integrator_QP_solver_ubIdx03, params->ub4, double_integrator_QP_solver_lub03, double_integrator_QP_solver_sub03, double_integrator_QP_solver_riub03, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_22_17(params->A4, double_integrator_QP_solver_z03, params->b4, double_integrator_QP_solver_sp03, double_integrator_QP_solver_lp03, double_integrator_QP_solver_rip03, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_4(params->lb5, double_integrator_QP_solver_z04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_llb04, double_integrator_QP_solver_slb04, double_integrator_QP_solver_rilb04, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_4(double_integrator_QP_solver_z04, double_integrator_QP_solver_ubIdx04, params->ub5, double_integrator_QP_solver_lub04, double_integrator_QP_solver_sub04, double_integrator_QP_solver_riub04, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_22_17(params->A5, double_integrator_QP_solver_z04, params->b5, double_integrator_QP_solver_sp04, double_integrator_QP_solver_lp04, double_integrator_QP_solver_rip04, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_4(params->lb6, double_integrator_QP_solver_z05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_llb05, double_integrator_QP_solver_slb05, double_integrator_QP_solver_rilb05, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_4(double_integrator_QP_solver_z05, double_integrator_QP_solver_ubIdx05, params->ub6, double_integrator_QP_solver_lub05, double_integrator_QP_solver_sub05, double_integrator_QP_solver_riub05, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_22_17(params->A6, double_integrator_QP_solver_z05, params->b6, double_integrator_QP_solver_sp05, double_integrator_QP_solver_lp05, double_integrator_QP_solver_rip05, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_4(params->lb7, double_integrator_QP_solver_z06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_llb06, double_integrator_QP_solver_slb06, double_integrator_QP_solver_rilb06, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_4(double_integrator_QP_solver_z06, double_integrator_QP_solver_ubIdx06, params->ub7, double_integrator_QP_solver_lub06, double_integrator_QP_solver_sub06, double_integrator_QP_solver_riub06, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_22_17(params->A7, double_integrator_QP_solver_z06, params->b7, double_integrator_QP_solver_sp06, double_integrator_QP_solver_lp06, double_integrator_QP_solver_rip06, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_4(params->lb8, double_integrator_QP_solver_z07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_llb07, double_integrator_QP_solver_slb07, double_integrator_QP_solver_rilb07, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_4(double_integrator_QP_solver_z07, double_integrator_QP_solver_ubIdx07, params->ub8, double_integrator_QP_solver_lub07, double_integrator_QP_solver_sub07, double_integrator_QP_solver_riub07, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_22_17(params->A8, double_integrator_QP_solver_z07, params->b8, double_integrator_QP_solver_sp07, double_integrator_QP_solver_lp07, double_integrator_QP_solver_rip07, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_4(params->lb9, double_integrator_QP_solver_z08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_llb08, double_integrator_QP_solver_slb08, double_integrator_QP_solver_rilb08, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_4(double_integrator_QP_solver_z08, double_integrator_QP_solver_ubIdx08, params->ub9, double_integrator_QP_solver_lub08, double_integrator_QP_solver_sub08, double_integrator_QP_solver_riub08, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_22_17(params->A9, double_integrator_QP_solver_z08, params->b9, double_integrator_QP_solver_sp08, double_integrator_QP_solver_lp08, double_integrator_QP_solver_rip08, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_4(params->lb10, double_integrator_QP_solver_z09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_llb09, double_integrator_QP_solver_slb09, double_integrator_QP_solver_rilb09, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_4(double_integrator_QP_solver_z09, double_integrator_QP_solver_ubIdx09, params->ub10, double_integrator_QP_solver_lub09, double_integrator_QP_solver_sub09, double_integrator_QP_solver_riub09, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_22_17(params->A10, double_integrator_QP_solver_z09, params->b10, double_integrator_QP_solver_sp09, double_integrator_QP_solver_lp09, double_integrator_QP_solver_rip09, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_4(params->lb11, double_integrator_QP_solver_z10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_llb10, double_integrator_QP_solver_slb10, double_integrator_QP_solver_rilb10, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_4(double_integrator_QP_solver_z10, double_integrator_QP_solver_ubIdx10, params->ub11, double_integrator_QP_solver_lub10, double_integrator_QP_solver_sub10, double_integrator_QP_solver_riub10, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_22_17(params->A11, double_integrator_QP_solver_z10, params->b11, double_integrator_QP_solver_sp10, double_integrator_QP_solver_lp10, double_integrator_QP_solver_rip10, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_4(params->lb12, double_integrator_QP_solver_z11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_llb11, double_integrator_QP_solver_slb11, double_integrator_QP_solver_rilb11, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_4(double_integrator_QP_solver_z11, double_integrator_QP_solver_ubIdx11, params->ub12, double_integrator_QP_solver_lub11, double_integrator_QP_solver_sub11, double_integrator_QP_solver_riub11, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_22_17(params->A12, double_integrator_QP_solver_z11, params->b12, double_integrator_QP_solver_sp11, double_integrator_QP_solver_lp11, double_integrator_QP_solver_rip11, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_4(params->lb13, double_integrator_QP_solver_z12, double_integrator_QP_solver_lbIdx12, double_integrator_QP_solver_llb12, double_integrator_QP_solver_slb12, double_integrator_QP_solver_rilb12, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_4(double_integrator_QP_solver_z12, double_integrator_QP_solver_ubIdx12, params->ub13, double_integrator_QP_solver_lub12, double_integrator_QP_solver_sub12, double_integrator_QP_solver_riub12, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_22_17(params->A13, double_integrator_QP_solver_z12, params->b13, double_integrator_QP_solver_sp12, double_integrator_QP_solver_lp12, double_integrator_QP_solver_rip12, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_4(params->lb14, double_integrator_QP_solver_z13, double_integrator_QP_solver_lbIdx13, double_integrator_QP_solver_llb13, double_integrator_QP_solver_slb13, double_integrator_QP_solver_rilb13, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_4(double_integrator_QP_solver_z13, double_integrator_QP_solver_ubIdx13, params->ub14, double_integrator_QP_solver_lub13, double_integrator_QP_solver_sub13, double_integrator_QP_solver_riub13, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_MVSUBADD_22_15(params->A14, double_integrator_QP_solver_z13, params->b14, double_integrator_QP_solver_sp13, double_integrator_QP_solver_lp13, double_integrator_QP_solver_rip13, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD3_4(params->lb15, double_integrator_QP_solver_z14, double_integrator_QP_solver_lbIdx14, double_integrator_QP_solver_llb14, double_integrator_QP_solver_slb14, double_integrator_QP_solver_rilb14, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_VSUBADD2_4(double_integrator_QP_solver_z14, double_integrator_QP_solver_ubIdx14, params->ub15, double_integrator_QP_solver_lub14, double_integrator_QP_solver_sub14, double_integrator_QP_solver_riub14, &info->dgap, &info->res_ineq);
double_integrator_QP_solver_LA_INEQ_B_GRAD_17_7_6(double_integrator_QP_solver_lub00, double_integrator_QP_solver_sub00, double_integrator_QP_solver_riub00, double_integrator_QP_solver_llb00, double_integrator_QP_solver_slb00, double_integrator_QP_solver_rilb00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_grad_ineq00, double_integrator_QP_solver_lubbysub00, double_integrator_QP_solver_llbbyslb00);
double_integrator_QP_solver_LA_INEQ_P_22_17(params->A1, double_integrator_QP_solver_lp00, double_integrator_QP_solver_sp00, double_integrator_QP_solver_rip00, double_integrator_QP_solver_grad_ineq00, double_integrator_QP_solver_lpbysp00);
double_integrator_QP_solver_LA_INEQ_B_GRAD_17_4_4(double_integrator_QP_solver_lub01, double_integrator_QP_solver_sub01, double_integrator_QP_solver_riub01, double_integrator_QP_solver_llb01, double_integrator_QP_solver_slb01, double_integrator_QP_solver_rilb01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_grad_ineq01, double_integrator_QP_solver_lubbysub01, double_integrator_QP_solver_llbbyslb01);
double_integrator_QP_solver_LA_INEQ_P_22_17(params->A2, double_integrator_QP_solver_lp01, double_integrator_QP_solver_sp01, double_integrator_QP_solver_rip01, double_integrator_QP_solver_grad_ineq01, double_integrator_QP_solver_lpbysp01);
double_integrator_QP_solver_LA_INEQ_B_GRAD_17_4_4(double_integrator_QP_solver_lub02, double_integrator_QP_solver_sub02, double_integrator_QP_solver_riub02, double_integrator_QP_solver_llb02, double_integrator_QP_solver_slb02, double_integrator_QP_solver_rilb02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_grad_ineq02, double_integrator_QP_solver_lubbysub02, double_integrator_QP_solver_llbbyslb02);
double_integrator_QP_solver_LA_INEQ_P_22_17(params->A3, double_integrator_QP_solver_lp02, double_integrator_QP_solver_sp02, double_integrator_QP_solver_rip02, double_integrator_QP_solver_grad_ineq02, double_integrator_QP_solver_lpbysp02);
double_integrator_QP_solver_LA_INEQ_B_GRAD_17_4_4(double_integrator_QP_solver_lub03, double_integrator_QP_solver_sub03, double_integrator_QP_solver_riub03, double_integrator_QP_solver_llb03, double_integrator_QP_solver_slb03, double_integrator_QP_solver_rilb03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_grad_ineq03, double_integrator_QP_solver_lubbysub03, double_integrator_QP_solver_llbbyslb03);
double_integrator_QP_solver_LA_INEQ_P_22_17(params->A4, double_integrator_QP_solver_lp03, double_integrator_QP_solver_sp03, double_integrator_QP_solver_rip03, double_integrator_QP_solver_grad_ineq03, double_integrator_QP_solver_lpbysp03);
double_integrator_QP_solver_LA_INEQ_B_GRAD_17_4_4(double_integrator_QP_solver_lub04, double_integrator_QP_solver_sub04, double_integrator_QP_solver_riub04, double_integrator_QP_solver_llb04, double_integrator_QP_solver_slb04, double_integrator_QP_solver_rilb04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_grad_ineq04, double_integrator_QP_solver_lubbysub04, double_integrator_QP_solver_llbbyslb04);
double_integrator_QP_solver_LA_INEQ_P_22_17(params->A5, double_integrator_QP_solver_lp04, double_integrator_QP_solver_sp04, double_integrator_QP_solver_rip04, double_integrator_QP_solver_grad_ineq04, double_integrator_QP_solver_lpbysp04);
double_integrator_QP_solver_LA_INEQ_B_GRAD_17_4_4(double_integrator_QP_solver_lub05, double_integrator_QP_solver_sub05, double_integrator_QP_solver_riub05, double_integrator_QP_solver_llb05, double_integrator_QP_solver_slb05, double_integrator_QP_solver_rilb05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_grad_ineq05, double_integrator_QP_solver_lubbysub05, double_integrator_QP_solver_llbbyslb05);
double_integrator_QP_solver_LA_INEQ_P_22_17(params->A6, double_integrator_QP_solver_lp05, double_integrator_QP_solver_sp05, double_integrator_QP_solver_rip05, double_integrator_QP_solver_grad_ineq05, double_integrator_QP_solver_lpbysp05);
double_integrator_QP_solver_LA_INEQ_B_GRAD_17_4_4(double_integrator_QP_solver_lub06, double_integrator_QP_solver_sub06, double_integrator_QP_solver_riub06, double_integrator_QP_solver_llb06, double_integrator_QP_solver_slb06, double_integrator_QP_solver_rilb06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_grad_ineq06, double_integrator_QP_solver_lubbysub06, double_integrator_QP_solver_llbbyslb06);
double_integrator_QP_solver_LA_INEQ_P_22_17(params->A7, double_integrator_QP_solver_lp06, double_integrator_QP_solver_sp06, double_integrator_QP_solver_rip06, double_integrator_QP_solver_grad_ineq06, double_integrator_QP_solver_lpbysp06);
double_integrator_QP_solver_LA_INEQ_B_GRAD_17_4_4(double_integrator_QP_solver_lub07, double_integrator_QP_solver_sub07, double_integrator_QP_solver_riub07, double_integrator_QP_solver_llb07, double_integrator_QP_solver_slb07, double_integrator_QP_solver_rilb07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_grad_ineq07, double_integrator_QP_solver_lubbysub07, double_integrator_QP_solver_llbbyslb07);
double_integrator_QP_solver_LA_INEQ_P_22_17(params->A8, double_integrator_QP_solver_lp07, double_integrator_QP_solver_sp07, double_integrator_QP_solver_rip07, double_integrator_QP_solver_grad_ineq07, double_integrator_QP_solver_lpbysp07);
double_integrator_QP_solver_LA_INEQ_B_GRAD_17_4_4(double_integrator_QP_solver_lub08, double_integrator_QP_solver_sub08, double_integrator_QP_solver_riub08, double_integrator_QP_solver_llb08, double_integrator_QP_solver_slb08, double_integrator_QP_solver_rilb08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_grad_ineq08, double_integrator_QP_solver_lubbysub08, double_integrator_QP_solver_llbbyslb08);
double_integrator_QP_solver_LA_INEQ_P_22_17(params->A9, double_integrator_QP_solver_lp08, double_integrator_QP_solver_sp08, double_integrator_QP_solver_rip08, double_integrator_QP_solver_grad_ineq08, double_integrator_QP_solver_lpbysp08);
double_integrator_QP_solver_LA_INEQ_B_GRAD_17_4_4(double_integrator_QP_solver_lub09, double_integrator_QP_solver_sub09, double_integrator_QP_solver_riub09, double_integrator_QP_solver_llb09, double_integrator_QP_solver_slb09, double_integrator_QP_solver_rilb09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_grad_ineq09, double_integrator_QP_solver_lubbysub09, double_integrator_QP_solver_llbbyslb09);
double_integrator_QP_solver_LA_INEQ_P_22_17(params->A10, double_integrator_QP_solver_lp09, double_integrator_QP_solver_sp09, double_integrator_QP_solver_rip09, double_integrator_QP_solver_grad_ineq09, double_integrator_QP_solver_lpbysp09);
double_integrator_QP_solver_LA_INEQ_B_GRAD_17_4_4(double_integrator_QP_solver_lub10, double_integrator_QP_solver_sub10, double_integrator_QP_solver_riub10, double_integrator_QP_solver_llb10, double_integrator_QP_solver_slb10, double_integrator_QP_solver_rilb10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_grad_ineq10, double_integrator_QP_solver_lubbysub10, double_integrator_QP_solver_llbbyslb10);
double_integrator_QP_solver_LA_INEQ_P_22_17(params->A11, double_integrator_QP_solver_lp10, double_integrator_QP_solver_sp10, double_integrator_QP_solver_rip10, double_integrator_QP_solver_grad_ineq10, double_integrator_QP_solver_lpbysp10);
double_integrator_QP_solver_LA_INEQ_B_GRAD_17_4_4(double_integrator_QP_solver_lub11, double_integrator_QP_solver_sub11, double_integrator_QP_solver_riub11, double_integrator_QP_solver_llb11, double_integrator_QP_solver_slb11, double_integrator_QP_solver_rilb11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_grad_ineq11, double_integrator_QP_solver_lubbysub11, double_integrator_QP_solver_llbbyslb11);
double_integrator_QP_solver_LA_INEQ_P_22_17(params->A12, double_integrator_QP_solver_lp11, double_integrator_QP_solver_sp11, double_integrator_QP_solver_rip11, double_integrator_QP_solver_grad_ineq11, double_integrator_QP_solver_lpbysp11);
double_integrator_QP_solver_LA_INEQ_B_GRAD_17_4_4(double_integrator_QP_solver_lub12, double_integrator_QP_solver_sub12, double_integrator_QP_solver_riub12, double_integrator_QP_solver_llb12, double_integrator_QP_solver_slb12, double_integrator_QP_solver_rilb12, double_integrator_QP_solver_lbIdx12, double_integrator_QP_solver_ubIdx12, double_integrator_QP_solver_grad_ineq12, double_integrator_QP_solver_lubbysub12, double_integrator_QP_solver_llbbyslb12);
double_integrator_QP_solver_LA_INEQ_P_22_17(params->A13, double_integrator_QP_solver_lp12, double_integrator_QP_solver_sp12, double_integrator_QP_solver_rip12, double_integrator_QP_solver_grad_ineq12, double_integrator_QP_solver_lpbysp12);
double_integrator_QP_solver_LA_INEQ_B_GRAD_15_4_4(double_integrator_QP_solver_lub13, double_integrator_QP_solver_sub13, double_integrator_QP_solver_riub13, double_integrator_QP_solver_llb13, double_integrator_QP_solver_slb13, double_integrator_QP_solver_rilb13, double_integrator_QP_solver_lbIdx13, double_integrator_QP_solver_ubIdx13, double_integrator_QP_solver_grad_ineq13, double_integrator_QP_solver_lubbysub13, double_integrator_QP_solver_llbbyslb13);
double_integrator_QP_solver_LA_INEQ_P_22_15(params->A14, double_integrator_QP_solver_lp13, double_integrator_QP_solver_sp13, double_integrator_QP_solver_rip13, double_integrator_QP_solver_grad_ineq13, double_integrator_QP_solver_lpbysp13);
double_integrator_QP_solver_LA_INEQ_B_GRAD_4_4_4(double_integrator_QP_solver_lub14, double_integrator_QP_solver_sub14, double_integrator_QP_solver_riub14, double_integrator_QP_solver_llb14, double_integrator_QP_solver_slb14, double_integrator_QP_solver_rilb14, double_integrator_QP_solver_lbIdx14, double_integrator_QP_solver_ubIdx14, double_integrator_QP_solver_grad_ineq14, double_integrator_QP_solver_lubbysub14, double_integrator_QP_solver_llbbyslb14);
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
double_integrator_QP_solver_LA_VVADD3_240(double_integrator_QP_solver_grad_cost, double_integrator_QP_solver_grad_eq, double_integrator_QP_solver_grad_ineq, double_integrator_QP_solver_rd);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_17_7_6(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_lubbysub00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_Phi00);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_22_17(params->A1, double_integrator_QP_solver_lpbysp00, double_integrator_QP_solver_Phi00);
double_integrator_QP_solver_LA_DENSE_CHOL2_17(double_integrator_QP_solver_Phi00);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_C00, double_integrator_QP_solver_V00);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_rd00, double_integrator_QP_solver_Lbyrd00);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_17_4_4(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_lubbysub01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_Phi01);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_22_17(params->A2, double_integrator_QP_solver_lpbysp01, double_integrator_QP_solver_Phi01);
double_integrator_QP_solver_LA_DENSE_CHOL2_17(double_integrator_QP_solver_Phi01);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_C00, double_integrator_QP_solver_V01);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_D01, double_integrator_QP_solver_W01);
double_integrator_QP_solver_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_W01, double_integrator_QP_solver_V01, double_integrator_QP_solver_Ysd01);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_rd01, double_integrator_QP_solver_Lbyrd01);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_17_4_4(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_lubbysub02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_Phi02);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_22_17(params->A3, double_integrator_QP_solver_lpbysp02, double_integrator_QP_solver_Phi02);
double_integrator_QP_solver_LA_DENSE_CHOL2_17(double_integrator_QP_solver_Phi02);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_C00, double_integrator_QP_solver_V02);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_D01, double_integrator_QP_solver_W02);
double_integrator_QP_solver_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_W02, double_integrator_QP_solver_V02, double_integrator_QP_solver_Ysd02);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_rd02, double_integrator_QP_solver_Lbyrd02);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_17_4_4(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_lubbysub03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_Phi03);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_22_17(params->A4, double_integrator_QP_solver_lpbysp03, double_integrator_QP_solver_Phi03);
double_integrator_QP_solver_LA_DENSE_CHOL2_17(double_integrator_QP_solver_Phi03);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_C00, double_integrator_QP_solver_V03);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_D01, double_integrator_QP_solver_W03);
double_integrator_QP_solver_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_W03, double_integrator_QP_solver_V03, double_integrator_QP_solver_Ysd03);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_rd03, double_integrator_QP_solver_Lbyrd03);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_17_4_4(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_lubbysub04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_Phi04);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_22_17(params->A5, double_integrator_QP_solver_lpbysp04, double_integrator_QP_solver_Phi04);
double_integrator_QP_solver_LA_DENSE_CHOL2_17(double_integrator_QP_solver_Phi04);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_C00, double_integrator_QP_solver_V04);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_D01, double_integrator_QP_solver_W04);
double_integrator_QP_solver_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_W04, double_integrator_QP_solver_V04, double_integrator_QP_solver_Ysd04);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_rd04, double_integrator_QP_solver_Lbyrd04);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_17_4_4(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_lubbysub05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_Phi05);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_22_17(params->A6, double_integrator_QP_solver_lpbysp05, double_integrator_QP_solver_Phi05);
double_integrator_QP_solver_LA_DENSE_CHOL2_17(double_integrator_QP_solver_Phi05);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_C00, double_integrator_QP_solver_V05);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_D01, double_integrator_QP_solver_W05);
double_integrator_QP_solver_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_W05, double_integrator_QP_solver_V05, double_integrator_QP_solver_Ysd05);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_rd05, double_integrator_QP_solver_Lbyrd05);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_17_4_4(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_lubbysub06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_Phi06);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_22_17(params->A7, double_integrator_QP_solver_lpbysp06, double_integrator_QP_solver_Phi06);
double_integrator_QP_solver_LA_DENSE_CHOL2_17(double_integrator_QP_solver_Phi06);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_C00, double_integrator_QP_solver_V06);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_D01, double_integrator_QP_solver_W06);
double_integrator_QP_solver_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_W06, double_integrator_QP_solver_V06, double_integrator_QP_solver_Ysd06);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_rd06, double_integrator_QP_solver_Lbyrd06);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_17_4_4(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_lubbysub07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_Phi07);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_22_17(params->A8, double_integrator_QP_solver_lpbysp07, double_integrator_QP_solver_Phi07);
double_integrator_QP_solver_LA_DENSE_CHOL2_17(double_integrator_QP_solver_Phi07);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_C00, double_integrator_QP_solver_V07);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_D01, double_integrator_QP_solver_W07);
double_integrator_QP_solver_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_W07, double_integrator_QP_solver_V07, double_integrator_QP_solver_Ysd07);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_rd07, double_integrator_QP_solver_Lbyrd07);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_17_4_4(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_lubbysub08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_Phi08);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_22_17(params->A9, double_integrator_QP_solver_lpbysp08, double_integrator_QP_solver_Phi08);
double_integrator_QP_solver_LA_DENSE_CHOL2_17(double_integrator_QP_solver_Phi08);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_C00, double_integrator_QP_solver_V08);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_D01, double_integrator_QP_solver_W08);
double_integrator_QP_solver_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_W08, double_integrator_QP_solver_V08, double_integrator_QP_solver_Ysd08);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_rd08, double_integrator_QP_solver_Lbyrd08);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_17_4_4(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_lubbysub09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_Phi09);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_22_17(params->A10, double_integrator_QP_solver_lpbysp09, double_integrator_QP_solver_Phi09);
double_integrator_QP_solver_LA_DENSE_CHOL2_17(double_integrator_QP_solver_Phi09);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_C00, double_integrator_QP_solver_V09);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_D01, double_integrator_QP_solver_W09);
double_integrator_QP_solver_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_W09, double_integrator_QP_solver_V09, double_integrator_QP_solver_Ysd09);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_rd09, double_integrator_QP_solver_Lbyrd09);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_17_4_4(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_lubbysub10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_Phi10);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_22_17(params->A11, double_integrator_QP_solver_lpbysp10, double_integrator_QP_solver_Phi10);
double_integrator_QP_solver_LA_DENSE_CHOL2_17(double_integrator_QP_solver_Phi10);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_C00, double_integrator_QP_solver_V10);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_D01, double_integrator_QP_solver_W10);
double_integrator_QP_solver_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_W10, double_integrator_QP_solver_V10, double_integrator_QP_solver_Ysd10);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_rd10, double_integrator_QP_solver_Lbyrd10);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_17_4_4(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_lubbysub11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_Phi11);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_22_17(params->A12, double_integrator_QP_solver_lpbysp11, double_integrator_QP_solver_Phi11);
double_integrator_QP_solver_LA_DENSE_CHOL2_17(double_integrator_QP_solver_Phi11);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_C00, double_integrator_QP_solver_V11);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_D01, double_integrator_QP_solver_W11);
double_integrator_QP_solver_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_W11, double_integrator_QP_solver_V11, double_integrator_QP_solver_Ysd11);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_rd11, double_integrator_QP_solver_Lbyrd11);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_17_4_4(double_integrator_QP_solver_H00, double_integrator_QP_solver_llbbyslb12, double_integrator_QP_solver_lbIdx12, double_integrator_QP_solver_lubbysub12, double_integrator_QP_solver_ubIdx12, double_integrator_QP_solver_Phi12);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_22_17(params->A13, double_integrator_QP_solver_lpbysp12, double_integrator_QP_solver_Phi12);
double_integrator_QP_solver_LA_DENSE_CHOL2_17(double_integrator_QP_solver_Phi12);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi12, double_integrator_QP_solver_C00, double_integrator_QP_solver_V12);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_17(double_integrator_QP_solver_Phi12, double_integrator_QP_solver_D01, double_integrator_QP_solver_W12);
double_integrator_QP_solver_LA_DENSE_MMTM_7_17_7(double_integrator_QP_solver_W12, double_integrator_QP_solver_V12, double_integrator_QP_solver_Ysd12);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi12, double_integrator_QP_solver_rd12, double_integrator_QP_solver_Lbyrd12);
double_integrator_QP_solver_LA_INEQ_DENSE_DIAG_HESS_15_4_4(double_integrator_QP_solver_H13, double_integrator_QP_solver_llbbyslb13, double_integrator_QP_solver_lbIdx13, double_integrator_QP_solver_lubbysub13, double_integrator_QP_solver_ubIdx13, double_integrator_QP_solver_Phi13);
double_integrator_QP_solver_LA_DENSE_ADDMTDM_22_15(params->A14, double_integrator_QP_solver_lpbysp13, double_integrator_QP_solver_Phi13);
double_integrator_QP_solver_LA_DENSE_CHOL2_15(double_integrator_QP_solver_Phi13);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_4_15(double_integrator_QP_solver_Phi13, double_integrator_QP_solver_C13, double_integrator_QP_solver_V13);
double_integrator_QP_solver_LA_DENSE_MATRIXFORWARDSUB_7_15(double_integrator_QP_solver_Phi13, double_integrator_QP_solver_D13, double_integrator_QP_solver_W13);
double_integrator_QP_solver_LA_DENSE_MMTM_7_15_4(double_integrator_QP_solver_W13, double_integrator_QP_solver_V13, double_integrator_QP_solver_Ysd13);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_15(double_integrator_QP_solver_Phi13, double_integrator_QP_solver_rd13, double_integrator_QP_solver_Lbyrd13);
double_integrator_QP_solver_LA_DIAG_CHOL_LBUB_5_4_4(double_integrator_QP_solver_H14, double_integrator_QP_solver_llbbyslb14, double_integrator_QP_solver_lbIdx14, double_integrator_QP_solver_lubbysub14, double_integrator_QP_solver_ubIdx14, double_integrator_QP_solver_Phi14);
double_integrator_QP_solver_LA_DIAG_DIAGZERO_MATRIXTFORWARDSUB_4_4(double_integrator_QP_solver_Phi14, double_integrator_QP_solver_D14, double_integrator_QP_solver_W14);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_4(double_integrator_QP_solver_Phi14, double_integrator_QP_solver_rd14, double_integrator_QP_solver_Lbyrd14);
double_integrator_QP_solver_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_V00, double_integrator_QP_solver_W01, double_integrator_QP_solver_Yd00);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_V00, double_integrator_QP_solver_Lbyrd00, double_integrator_QP_solver_W01, double_integrator_QP_solver_Lbyrd01, double_integrator_QP_solver_re00, double_integrator_QP_solver_beta00);
double_integrator_QP_solver_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_V01, double_integrator_QP_solver_W02, double_integrator_QP_solver_Yd01);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_V01, double_integrator_QP_solver_Lbyrd01, double_integrator_QP_solver_W02, double_integrator_QP_solver_Lbyrd02, double_integrator_QP_solver_re01, double_integrator_QP_solver_beta01);
double_integrator_QP_solver_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_V02, double_integrator_QP_solver_W03, double_integrator_QP_solver_Yd02);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_V02, double_integrator_QP_solver_Lbyrd02, double_integrator_QP_solver_W03, double_integrator_QP_solver_Lbyrd03, double_integrator_QP_solver_re02, double_integrator_QP_solver_beta02);
double_integrator_QP_solver_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_V03, double_integrator_QP_solver_W04, double_integrator_QP_solver_Yd03);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_V03, double_integrator_QP_solver_Lbyrd03, double_integrator_QP_solver_W04, double_integrator_QP_solver_Lbyrd04, double_integrator_QP_solver_re03, double_integrator_QP_solver_beta03);
double_integrator_QP_solver_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_V04, double_integrator_QP_solver_W05, double_integrator_QP_solver_Yd04);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_V04, double_integrator_QP_solver_Lbyrd04, double_integrator_QP_solver_W05, double_integrator_QP_solver_Lbyrd05, double_integrator_QP_solver_re04, double_integrator_QP_solver_beta04);
double_integrator_QP_solver_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_V05, double_integrator_QP_solver_W06, double_integrator_QP_solver_Yd05);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_V05, double_integrator_QP_solver_Lbyrd05, double_integrator_QP_solver_W06, double_integrator_QP_solver_Lbyrd06, double_integrator_QP_solver_re05, double_integrator_QP_solver_beta05);
double_integrator_QP_solver_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_V06, double_integrator_QP_solver_W07, double_integrator_QP_solver_Yd06);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_V06, double_integrator_QP_solver_Lbyrd06, double_integrator_QP_solver_W07, double_integrator_QP_solver_Lbyrd07, double_integrator_QP_solver_re06, double_integrator_QP_solver_beta06);
double_integrator_QP_solver_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_V07, double_integrator_QP_solver_W08, double_integrator_QP_solver_Yd07);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_V07, double_integrator_QP_solver_Lbyrd07, double_integrator_QP_solver_W08, double_integrator_QP_solver_Lbyrd08, double_integrator_QP_solver_re07, double_integrator_QP_solver_beta07);
double_integrator_QP_solver_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_V08, double_integrator_QP_solver_W09, double_integrator_QP_solver_Yd08);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_V08, double_integrator_QP_solver_Lbyrd08, double_integrator_QP_solver_W09, double_integrator_QP_solver_Lbyrd09, double_integrator_QP_solver_re08, double_integrator_QP_solver_beta08);
double_integrator_QP_solver_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_V09, double_integrator_QP_solver_W10, double_integrator_QP_solver_Yd09);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_V09, double_integrator_QP_solver_Lbyrd09, double_integrator_QP_solver_W10, double_integrator_QP_solver_Lbyrd10, double_integrator_QP_solver_re09, double_integrator_QP_solver_beta09);
double_integrator_QP_solver_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_V10, double_integrator_QP_solver_W11, double_integrator_QP_solver_Yd10);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_V10, double_integrator_QP_solver_Lbyrd10, double_integrator_QP_solver_W11, double_integrator_QP_solver_Lbyrd11, double_integrator_QP_solver_re10, double_integrator_QP_solver_beta10);
double_integrator_QP_solver_LA_DENSE_MMT2_7_17_17(double_integrator_QP_solver_V11, double_integrator_QP_solver_W12, double_integrator_QP_solver_Yd11);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_17_17(double_integrator_QP_solver_V11, double_integrator_QP_solver_Lbyrd11, double_integrator_QP_solver_W12, double_integrator_QP_solver_Lbyrd12, double_integrator_QP_solver_re11, double_integrator_QP_solver_beta11);
double_integrator_QP_solver_LA_DENSE_MMT2_7_17_15(double_integrator_QP_solver_V12, double_integrator_QP_solver_W13, double_integrator_QP_solver_Yd12);
double_integrator_QP_solver_LA_DENSE_MVMSUB2_7_17_15(double_integrator_QP_solver_V12, double_integrator_QP_solver_Lbyrd12, double_integrator_QP_solver_W13, double_integrator_QP_solver_Lbyrd13, double_integrator_QP_solver_re12, double_integrator_QP_solver_beta12);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_MMT2_4_15_4(double_integrator_QP_solver_V13, double_integrator_QP_solver_W14, double_integrator_QP_solver_Yd13);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMSUB2_4_15_4(double_integrator_QP_solver_V13, double_integrator_QP_solver_Lbyrd13, double_integrator_QP_solver_W14, double_integrator_QP_solver_Lbyrd14, double_integrator_QP_solver_re13, double_integrator_QP_solver_beta13);
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
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_Ld09, double_integrator_QP_solver_Ysd10, double_integrator_QP_solver_Lsd10);
double_integrator_QP_solver_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_Yd10);
double_integrator_QP_solver_LA_DENSE_CHOL_7(double_integrator_QP_solver_Yd10, double_integrator_QP_solver_Ld10);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_yy09, double_integrator_QP_solver_beta10, double_integrator_QP_solver_bmy10);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_bmy10, double_integrator_QP_solver_yy10);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_Ysd11, double_integrator_QP_solver_Lsd11);
double_integrator_QP_solver_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_Lsd11, double_integrator_QP_solver_Yd11);
double_integrator_QP_solver_LA_DENSE_CHOL_7(double_integrator_QP_solver_Yd11, double_integrator_QP_solver_Ld11);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd11, double_integrator_QP_solver_yy10, double_integrator_QP_solver_beta11, double_integrator_QP_solver_bmy11);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld11, double_integrator_QP_solver_bmy11, double_integrator_QP_solver_yy11);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_7_7(double_integrator_QP_solver_Ld11, double_integrator_QP_solver_Ysd12, double_integrator_QP_solver_Lsd12);
double_integrator_QP_solver_LA_DENSE_MMTSUB_7_7(double_integrator_QP_solver_Lsd12, double_integrator_QP_solver_Yd12);
double_integrator_QP_solver_LA_DENSE_CHOL_7(double_integrator_QP_solver_Yd12, double_integrator_QP_solver_Ld12);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd12, double_integrator_QP_solver_yy11, double_integrator_QP_solver_beta12, double_integrator_QP_solver_bmy12);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld12, double_integrator_QP_solver_bmy12, double_integrator_QP_solver_yy12);
double_integrator_QP_solver_LA_DENSE_MATRIXTFORWARDSUB_4_7(double_integrator_QP_solver_Ld12, double_integrator_QP_solver_Ysd13, double_integrator_QP_solver_Lsd13);
double_integrator_QP_solver_LA_DENSE_MMTSUB_4_7(double_integrator_QP_solver_Lsd13, double_integrator_QP_solver_Yd13);
double_integrator_QP_solver_LA_DENSE_CHOL_4(double_integrator_QP_solver_Yd13, double_integrator_QP_solver_Ld13);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_4_7(double_integrator_QP_solver_Lsd13, double_integrator_QP_solver_yy12, double_integrator_QP_solver_beta13, double_integrator_QP_solver_bmy13);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_4(double_integrator_QP_solver_Ld13, double_integrator_QP_solver_bmy13, double_integrator_QP_solver_yy13);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_4(double_integrator_QP_solver_Ld13, double_integrator_QP_solver_yy13, double_integrator_QP_solver_dvaff13);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_4_7(double_integrator_QP_solver_Lsd13, double_integrator_QP_solver_dvaff13, double_integrator_QP_solver_yy12, double_integrator_QP_solver_bmy12);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld12, double_integrator_QP_solver_bmy12, double_integrator_QP_solver_dvaff12);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd12, double_integrator_QP_solver_dvaff12, double_integrator_QP_solver_yy11, double_integrator_QP_solver_bmy11);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld11, double_integrator_QP_solver_bmy11, double_integrator_QP_solver_dvaff11);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd11, double_integrator_QP_solver_dvaff11, double_integrator_QP_solver_yy10, double_integrator_QP_solver_bmy10);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_bmy10, double_integrator_QP_solver_dvaff10);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_dvaff10, double_integrator_QP_solver_yy09, double_integrator_QP_solver_bmy09);
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
double_integrator_QP_solver_LA_DENSE_MTVM_7_17(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff00, double_integrator_QP_solver_grad_eq00);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff01, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff00, double_integrator_QP_solver_grad_eq01);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff02, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff01, double_integrator_QP_solver_grad_eq02);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff03, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff02, double_integrator_QP_solver_grad_eq03);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff04, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff03, double_integrator_QP_solver_grad_eq04);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff05, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff04, double_integrator_QP_solver_grad_eq05);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff06, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff05, double_integrator_QP_solver_grad_eq06);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff07, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff06, double_integrator_QP_solver_grad_eq07);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff08, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff07, double_integrator_QP_solver_grad_eq08);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff09, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff08, double_integrator_QP_solver_grad_eq09);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff10, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff09, double_integrator_QP_solver_grad_eq10);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff11, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff10, double_integrator_QP_solver_grad_eq11);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvaff12, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvaff11, double_integrator_QP_solver_grad_eq12);
double_integrator_QP_solver_LA_DENSE_MTVM2_4_15_7(double_integrator_QP_solver_C13, double_integrator_QP_solver_dvaff13, double_integrator_QP_solver_D13, double_integrator_QP_solver_dvaff12, double_integrator_QP_solver_grad_eq13);
double_integrator_QP_solver_LA_DIAGZERO_MTVM_4_4(double_integrator_QP_solver_D14, double_integrator_QP_solver_dvaff13, double_integrator_QP_solver_grad_eq14);
double_integrator_QP_solver_LA_VSUB2_240(double_integrator_QP_solver_rd, double_integrator_QP_solver_grad_eq, double_integrator_QP_solver_rd);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_rd00, double_integrator_QP_solver_dzaff00);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_rd01, double_integrator_QP_solver_dzaff01);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_rd02, double_integrator_QP_solver_dzaff02);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_rd03, double_integrator_QP_solver_dzaff03);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_rd04, double_integrator_QP_solver_dzaff04);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_rd05, double_integrator_QP_solver_dzaff05);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_rd06, double_integrator_QP_solver_dzaff06);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_rd07, double_integrator_QP_solver_dzaff07);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_rd08, double_integrator_QP_solver_dzaff08);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_rd09, double_integrator_QP_solver_dzaff09);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_rd10, double_integrator_QP_solver_dzaff10);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_rd11, double_integrator_QP_solver_dzaff11);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi12, double_integrator_QP_solver_rd12, double_integrator_QP_solver_dzaff12);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi13, double_integrator_QP_solver_rd13, double_integrator_QP_solver_dzaff13);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_4(double_integrator_QP_solver_Phi14, double_integrator_QP_solver_rd14, double_integrator_QP_solver_dzaff14);
double_integrator_QP_solver_LA_VSUB_INDEXED_7(double_integrator_QP_solver_dzaff00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_rilb00, double_integrator_QP_solver_dslbaff00);
double_integrator_QP_solver_LA_VSUB3_7(double_integrator_QP_solver_llbbyslb00, double_integrator_QP_solver_dslbaff00, double_integrator_QP_solver_llb00, double_integrator_QP_solver_dllbaff00);
double_integrator_QP_solver_LA_VSUB2_INDEXED_6(double_integrator_QP_solver_riub00, double_integrator_QP_solver_dzaff00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_dsubaff00);
double_integrator_QP_solver_LA_VSUB3_6(double_integrator_QP_solver_lubbysub00, double_integrator_QP_solver_dsubaff00, double_integrator_QP_solver_lub00, double_integrator_QP_solver_dlubaff00);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_22_17(params->A1, double_integrator_QP_solver_dzaff00, double_integrator_QP_solver_rip00, double_integrator_QP_solver_dsp_aff00);
double_integrator_QP_solver_LA_VSUB3_22(double_integrator_QP_solver_lpbysp00, double_integrator_QP_solver_dsp_aff00, double_integrator_QP_solver_lp00, double_integrator_QP_solver_dlp_aff00);
double_integrator_QP_solver_LA_VSUB_INDEXED_4(double_integrator_QP_solver_dzaff01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_rilb01, double_integrator_QP_solver_dslbaff01);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_llbbyslb01, double_integrator_QP_solver_dslbaff01, double_integrator_QP_solver_llb01, double_integrator_QP_solver_dllbaff01);
double_integrator_QP_solver_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_riub01, double_integrator_QP_solver_dzaff01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_dsubaff01);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_lubbysub01, double_integrator_QP_solver_dsubaff01, double_integrator_QP_solver_lub01, double_integrator_QP_solver_dlubaff01);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_22_17(params->A2, double_integrator_QP_solver_dzaff01, double_integrator_QP_solver_rip01, double_integrator_QP_solver_dsp_aff01);
double_integrator_QP_solver_LA_VSUB3_22(double_integrator_QP_solver_lpbysp01, double_integrator_QP_solver_dsp_aff01, double_integrator_QP_solver_lp01, double_integrator_QP_solver_dlp_aff01);
double_integrator_QP_solver_LA_VSUB_INDEXED_4(double_integrator_QP_solver_dzaff02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_rilb02, double_integrator_QP_solver_dslbaff02);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_llbbyslb02, double_integrator_QP_solver_dslbaff02, double_integrator_QP_solver_llb02, double_integrator_QP_solver_dllbaff02);
double_integrator_QP_solver_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_riub02, double_integrator_QP_solver_dzaff02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_dsubaff02);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_lubbysub02, double_integrator_QP_solver_dsubaff02, double_integrator_QP_solver_lub02, double_integrator_QP_solver_dlubaff02);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_22_17(params->A3, double_integrator_QP_solver_dzaff02, double_integrator_QP_solver_rip02, double_integrator_QP_solver_dsp_aff02);
double_integrator_QP_solver_LA_VSUB3_22(double_integrator_QP_solver_lpbysp02, double_integrator_QP_solver_dsp_aff02, double_integrator_QP_solver_lp02, double_integrator_QP_solver_dlp_aff02);
double_integrator_QP_solver_LA_VSUB_INDEXED_4(double_integrator_QP_solver_dzaff03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_rilb03, double_integrator_QP_solver_dslbaff03);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_llbbyslb03, double_integrator_QP_solver_dslbaff03, double_integrator_QP_solver_llb03, double_integrator_QP_solver_dllbaff03);
double_integrator_QP_solver_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_riub03, double_integrator_QP_solver_dzaff03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_dsubaff03);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_lubbysub03, double_integrator_QP_solver_dsubaff03, double_integrator_QP_solver_lub03, double_integrator_QP_solver_dlubaff03);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_22_17(params->A4, double_integrator_QP_solver_dzaff03, double_integrator_QP_solver_rip03, double_integrator_QP_solver_dsp_aff03);
double_integrator_QP_solver_LA_VSUB3_22(double_integrator_QP_solver_lpbysp03, double_integrator_QP_solver_dsp_aff03, double_integrator_QP_solver_lp03, double_integrator_QP_solver_dlp_aff03);
double_integrator_QP_solver_LA_VSUB_INDEXED_4(double_integrator_QP_solver_dzaff04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_rilb04, double_integrator_QP_solver_dslbaff04);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_llbbyslb04, double_integrator_QP_solver_dslbaff04, double_integrator_QP_solver_llb04, double_integrator_QP_solver_dllbaff04);
double_integrator_QP_solver_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_riub04, double_integrator_QP_solver_dzaff04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_dsubaff04);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_lubbysub04, double_integrator_QP_solver_dsubaff04, double_integrator_QP_solver_lub04, double_integrator_QP_solver_dlubaff04);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_22_17(params->A5, double_integrator_QP_solver_dzaff04, double_integrator_QP_solver_rip04, double_integrator_QP_solver_dsp_aff04);
double_integrator_QP_solver_LA_VSUB3_22(double_integrator_QP_solver_lpbysp04, double_integrator_QP_solver_dsp_aff04, double_integrator_QP_solver_lp04, double_integrator_QP_solver_dlp_aff04);
double_integrator_QP_solver_LA_VSUB_INDEXED_4(double_integrator_QP_solver_dzaff05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_rilb05, double_integrator_QP_solver_dslbaff05);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_llbbyslb05, double_integrator_QP_solver_dslbaff05, double_integrator_QP_solver_llb05, double_integrator_QP_solver_dllbaff05);
double_integrator_QP_solver_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_riub05, double_integrator_QP_solver_dzaff05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_dsubaff05);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_lubbysub05, double_integrator_QP_solver_dsubaff05, double_integrator_QP_solver_lub05, double_integrator_QP_solver_dlubaff05);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_22_17(params->A6, double_integrator_QP_solver_dzaff05, double_integrator_QP_solver_rip05, double_integrator_QP_solver_dsp_aff05);
double_integrator_QP_solver_LA_VSUB3_22(double_integrator_QP_solver_lpbysp05, double_integrator_QP_solver_dsp_aff05, double_integrator_QP_solver_lp05, double_integrator_QP_solver_dlp_aff05);
double_integrator_QP_solver_LA_VSUB_INDEXED_4(double_integrator_QP_solver_dzaff06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_rilb06, double_integrator_QP_solver_dslbaff06);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_llbbyslb06, double_integrator_QP_solver_dslbaff06, double_integrator_QP_solver_llb06, double_integrator_QP_solver_dllbaff06);
double_integrator_QP_solver_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_riub06, double_integrator_QP_solver_dzaff06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_dsubaff06);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_lubbysub06, double_integrator_QP_solver_dsubaff06, double_integrator_QP_solver_lub06, double_integrator_QP_solver_dlubaff06);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_22_17(params->A7, double_integrator_QP_solver_dzaff06, double_integrator_QP_solver_rip06, double_integrator_QP_solver_dsp_aff06);
double_integrator_QP_solver_LA_VSUB3_22(double_integrator_QP_solver_lpbysp06, double_integrator_QP_solver_dsp_aff06, double_integrator_QP_solver_lp06, double_integrator_QP_solver_dlp_aff06);
double_integrator_QP_solver_LA_VSUB_INDEXED_4(double_integrator_QP_solver_dzaff07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_rilb07, double_integrator_QP_solver_dslbaff07);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_llbbyslb07, double_integrator_QP_solver_dslbaff07, double_integrator_QP_solver_llb07, double_integrator_QP_solver_dllbaff07);
double_integrator_QP_solver_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_riub07, double_integrator_QP_solver_dzaff07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_dsubaff07);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_lubbysub07, double_integrator_QP_solver_dsubaff07, double_integrator_QP_solver_lub07, double_integrator_QP_solver_dlubaff07);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_22_17(params->A8, double_integrator_QP_solver_dzaff07, double_integrator_QP_solver_rip07, double_integrator_QP_solver_dsp_aff07);
double_integrator_QP_solver_LA_VSUB3_22(double_integrator_QP_solver_lpbysp07, double_integrator_QP_solver_dsp_aff07, double_integrator_QP_solver_lp07, double_integrator_QP_solver_dlp_aff07);
double_integrator_QP_solver_LA_VSUB_INDEXED_4(double_integrator_QP_solver_dzaff08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_rilb08, double_integrator_QP_solver_dslbaff08);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_llbbyslb08, double_integrator_QP_solver_dslbaff08, double_integrator_QP_solver_llb08, double_integrator_QP_solver_dllbaff08);
double_integrator_QP_solver_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_riub08, double_integrator_QP_solver_dzaff08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_dsubaff08);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_lubbysub08, double_integrator_QP_solver_dsubaff08, double_integrator_QP_solver_lub08, double_integrator_QP_solver_dlubaff08);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_22_17(params->A9, double_integrator_QP_solver_dzaff08, double_integrator_QP_solver_rip08, double_integrator_QP_solver_dsp_aff08);
double_integrator_QP_solver_LA_VSUB3_22(double_integrator_QP_solver_lpbysp08, double_integrator_QP_solver_dsp_aff08, double_integrator_QP_solver_lp08, double_integrator_QP_solver_dlp_aff08);
double_integrator_QP_solver_LA_VSUB_INDEXED_4(double_integrator_QP_solver_dzaff09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_rilb09, double_integrator_QP_solver_dslbaff09);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_llbbyslb09, double_integrator_QP_solver_dslbaff09, double_integrator_QP_solver_llb09, double_integrator_QP_solver_dllbaff09);
double_integrator_QP_solver_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_riub09, double_integrator_QP_solver_dzaff09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_dsubaff09);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_lubbysub09, double_integrator_QP_solver_dsubaff09, double_integrator_QP_solver_lub09, double_integrator_QP_solver_dlubaff09);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_22_17(params->A10, double_integrator_QP_solver_dzaff09, double_integrator_QP_solver_rip09, double_integrator_QP_solver_dsp_aff09);
double_integrator_QP_solver_LA_VSUB3_22(double_integrator_QP_solver_lpbysp09, double_integrator_QP_solver_dsp_aff09, double_integrator_QP_solver_lp09, double_integrator_QP_solver_dlp_aff09);
double_integrator_QP_solver_LA_VSUB_INDEXED_4(double_integrator_QP_solver_dzaff10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_rilb10, double_integrator_QP_solver_dslbaff10);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_llbbyslb10, double_integrator_QP_solver_dslbaff10, double_integrator_QP_solver_llb10, double_integrator_QP_solver_dllbaff10);
double_integrator_QP_solver_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_riub10, double_integrator_QP_solver_dzaff10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_dsubaff10);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_lubbysub10, double_integrator_QP_solver_dsubaff10, double_integrator_QP_solver_lub10, double_integrator_QP_solver_dlubaff10);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_22_17(params->A11, double_integrator_QP_solver_dzaff10, double_integrator_QP_solver_rip10, double_integrator_QP_solver_dsp_aff10);
double_integrator_QP_solver_LA_VSUB3_22(double_integrator_QP_solver_lpbysp10, double_integrator_QP_solver_dsp_aff10, double_integrator_QP_solver_lp10, double_integrator_QP_solver_dlp_aff10);
double_integrator_QP_solver_LA_VSUB_INDEXED_4(double_integrator_QP_solver_dzaff11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_rilb11, double_integrator_QP_solver_dslbaff11);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_llbbyslb11, double_integrator_QP_solver_dslbaff11, double_integrator_QP_solver_llb11, double_integrator_QP_solver_dllbaff11);
double_integrator_QP_solver_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_riub11, double_integrator_QP_solver_dzaff11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_dsubaff11);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_lubbysub11, double_integrator_QP_solver_dsubaff11, double_integrator_QP_solver_lub11, double_integrator_QP_solver_dlubaff11);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_22_17(params->A12, double_integrator_QP_solver_dzaff11, double_integrator_QP_solver_rip11, double_integrator_QP_solver_dsp_aff11);
double_integrator_QP_solver_LA_VSUB3_22(double_integrator_QP_solver_lpbysp11, double_integrator_QP_solver_dsp_aff11, double_integrator_QP_solver_lp11, double_integrator_QP_solver_dlp_aff11);
double_integrator_QP_solver_LA_VSUB_INDEXED_4(double_integrator_QP_solver_dzaff12, double_integrator_QP_solver_lbIdx12, double_integrator_QP_solver_rilb12, double_integrator_QP_solver_dslbaff12);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_llbbyslb12, double_integrator_QP_solver_dslbaff12, double_integrator_QP_solver_llb12, double_integrator_QP_solver_dllbaff12);
double_integrator_QP_solver_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_riub12, double_integrator_QP_solver_dzaff12, double_integrator_QP_solver_ubIdx12, double_integrator_QP_solver_dsubaff12);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_lubbysub12, double_integrator_QP_solver_dsubaff12, double_integrator_QP_solver_lub12, double_integrator_QP_solver_dlubaff12);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_22_17(params->A13, double_integrator_QP_solver_dzaff12, double_integrator_QP_solver_rip12, double_integrator_QP_solver_dsp_aff12);
double_integrator_QP_solver_LA_VSUB3_22(double_integrator_QP_solver_lpbysp12, double_integrator_QP_solver_dsp_aff12, double_integrator_QP_solver_lp12, double_integrator_QP_solver_dlp_aff12);
double_integrator_QP_solver_LA_VSUB_INDEXED_4(double_integrator_QP_solver_dzaff13, double_integrator_QP_solver_lbIdx13, double_integrator_QP_solver_rilb13, double_integrator_QP_solver_dslbaff13);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_llbbyslb13, double_integrator_QP_solver_dslbaff13, double_integrator_QP_solver_llb13, double_integrator_QP_solver_dllbaff13);
double_integrator_QP_solver_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_riub13, double_integrator_QP_solver_dzaff13, double_integrator_QP_solver_ubIdx13, double_integrator_QP_solver_dsubaff13);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_lubbysub13, double_integrator_QP_solver_dsubaff13, double_integrator_QP_solver_lub13, double_integrator_QP_solver_dlubaff13);
double_integrator_QP_solver_LA_DENSE_MVMSUB4_22_15(params->A14, double_integrator_QP_solver_dzaff13, double_integrator_QP_solver_rip13, double_integrator_QP_solver_dsp_aff13);
double_integrator_QP_solver_LA_VSUB3_22(double_integrator_QP_solver_lpbysp13, double_integrator_QP_solver_dsp_aff13, double_integrator_QP_solver_lp13, double_integrator_QP_solver_dlp_aff13);
double_integrator_QP_solver_LA_VSUB_INDEXED_4(double_integrator_QP_solver_dzaff14, double_integrator_QP_solver_lbIdx14, double_integrator_QP_solver_rilb14, double_integrator_QP_solver_dslbaff14);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_llbbyslb14, double_integrator_QP_solver_dslbaff14, double_integrator_QP_solver_llb14, double_integrator_QP_solver_dllbaff14);
double_integrator_QP_solver_LA_VSUB2_INDEXED_4(double_integrator_QP_solver_riub14, double_integrator_QP_solver_dzaff14, double_integrator_QP_solver_ubIdx14, double_integrator_QP_solver_dsubaff14);
double_integrator_QP_solver_LA_VSUB3_4(double_integrator_QP_solver_lubbysub14, double_integrator_QP_solver_dsubaff14, double_integrator_QP_solver_lub14, double_integrator_QP_solver_dlubaff14);
info->lsit_aff = double_integrator_QP_solver_LINESEARCH_BACKTRACKING_AFFINE(double_integrator_QP_solver_l, double_integrator_QP_solver_s, double_integrator_QP_solver_dl_aff, double_integrator_QP_solver_ds_aff, &info->step_aff, &info->mu_aff);
if( info->lsit_aff == double_integrator_QP_solver_NOPROGRESS ){
exitcode = double_integrator_QP_solver_NOPROGRESS; break;
}
sigma_3rdroot = info->mu_aff / info->mu;
info->sigma = sigma_3rdroot*sigma_3rdroot*sigma_3rdroot;
musigma = info->mu * info->sigma;
double_integrator_QP_solver_LA_VSUB5_433(double_integrator_QP_solver_ds_aff, double_integrator_QP_solver_dl_aff, info->mu, info->sigma, double_integrator_QP_solver_ccrhs);
double_integrator_QP_solver_LA_VSUB6_INDEXED_17_6_7(double_integrator_QP_solver_ccrhsub00, double_integrator_QP_solver_sub00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_ccrhsl00, double_integrator_QP_solver_slb00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_rd00);
double_integrator_QP_solver_LA_VSUB6_INDEXED_17_4_4(double_integrator_QP_solver_ccrhsub01, double_integrator_QP_solver_sub01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_ccrhsl01, double_integrator_QP_solver_slb01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_rd01);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_22_17(params->A1, double_integrator_QP_solver_ccrhsp00, double_integrator_QP_solver_sp00, double_integrator_QP_solver_rd00);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_22_17(params->A2, double_integrator_QP_solver_ccrhsp01, double_integrator_QP_solver_sp01, double_integrator_QP_solver_rd01);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_rd00, double_integrator_QP_solver_Lbyrd00);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_rd01, double_integrator_QP_solver_Lbyrd01);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_V00, double_integrator_QP_solver_Lbyrd00, double_integrator_QP_solver_W01, double_integrator_QP_solver_Lbyrd01, double_integrator_QP_solver_beta00);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld00, double_integrator_QP_solver_beta00, double_integrator_QP_solver_yy00);
double_integrator_QP_solver_LA_VSUB6_INDEXED_17_4_4(double_integrator_QP_solver_ccrhsub02, double_integrator_QP_solver_sub02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_ccrhsl02, double_integrator_QP_solver_slb02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_rd02);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_22_17(params->A3, double_integrator_QP_solver_ccrhsp02, double_integrator_QP_solver_sp02, double_integrator_QP_solver_rd02);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_rd02, double_integrator_QP_solver_Lbyrd02);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_V01, double_integrator_QP_solver_Lbyrd01, double_integrator_QP_solver_W02, double_integrator_QP_solver_Lbyrd02, double_integrator_QP_solver_beta01);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd01, double_integrator_QP_solver_yy00, double_integrator_QP_solver_beta01, double_integrator_QP_solver_bmy01);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld01, double_integrator_QP_solver_bmy01, double_integrator_QP_solver_yy01);
double_integrator_QP_solver_LA_VSUB6_INDEXED_17_4_4(double_integrator_QP_solver_ccrhsub03, double_integrator_QP_solver_sub03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_ccrhsl03, double_integrator_QP_solver_slb03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_rd03);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_22_17(params->A4, double_integrator_QP_solver_ccrhsp03, double_integrator_QP_solver_sp03, double_integrator_QP_solver_rd03);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_rd03, double_integrator_QP_solver_Lbyrd03);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_V02, double_integrator_QP_solver_Lbyrd02, double_integrator_QP_solver_W03, double_integrator_QP_solver_Lbyrd03, double_integrator_QP_solver_beta02);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd02, double_integrator_QP_solver_yy01, double_integrator_QP_solver_beta02, double_integrator_QP_solver_bmy02);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld02, double_integrator_QP_solver_bmy02, double_integrator_QP_solver_yy02);
double_integrator_QP_solver_LA_VSUB6_INDEXED_17_4_4(double_integrator_QP_solver_ccrhsub04, double_integrator_QP_solver_sub04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_ccrhsl04, double_integrator_QP_solver_slb04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_rd04);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_22_17(params->A5, double_integrator_QP_solver_ccrhsp04, double_integrator_QP_solver_sp04, double_integrator_QP_solver_rd04);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_rd04, double_integrator_QP_solver_Lbyrd04);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_V03, double_integrator_QP_solver_Lbyrd03, double_integrator_QP_solver_W04, double_integrator_QP_solver_Lbyrd04, double_integrator_QP_solver_beta03);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd03, double_integrator_QP_solver_yy02, double_integrator_QP_solver_beta03, double_integrator_QP_solver_bmy03);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld03, double_integrator_QP_solver_bmy03, double_integrator_QP_solver_yy03);
double_integrator_QP_solver_LA_VSUB6_INDEXED_17_4_4(double_integrator_QP_solver_ccrhsub05, double_integrator_QP_solver_sub05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_ccrhsl05, double_integrator_QP_solver_slb05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_rd05);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_22_17(params->A6, double_integrator_QP_solver_ccrhsp05, double_integrator_QP_solver_sp05, double_integrator_QP_solver_rd05);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_rd05, double_integrator_QP_solver_Lbyrd05);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_V04, double_integrator_QP_solver_Lbyrd04, double_integrator_QP_solver_W05, double_integrator_QP_solver_Lbyrd05, double_integrator_QP_solver_beta04);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd04, double_integrator_QP_solver_yy03, double_integrator_QP_solver_beta04, double_integrator_QP_solver_bmy04);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld04, double_integrator_QP_solver_bmy04, double_integrator_QP_solver_yy04);
double_integrator_QP_solver_LA_VSUB6_INDEXED_17_4_4(double_integrator_QP_solver_ccrhsub06, double_integrator_QP_solver_sub06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_ccrhsl06, double_integrator_QP_solver_slb06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_rd06);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_22_17(params->A7, double_integrator_QP_solver_ccrhsp06, double_integrator_QP_solver_sp06, double_integrator_QP_solver_rd06);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_rd06, double_integrator_QP_solver_Lbyrd06);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_V05, double_integrator_QP_solver_Lbyrd05, double_integrator_QP_solver_W06, double_integrator_QP_solver_Lbyrd06, double_integrator_QP_solver_beta05);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd05, double_integrator_QP_solver_yy04, double_integrator_QP_solver_beta05, double_integrator_QP_solver_bmy05);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld05, double_integrator_QP_solver_bmy05, double_integrator_QP_solver_yy05);
double_integrator_QP_solver_LA_VSUB6_INDEXED_17_4_4(double_integrator_QP_solver_ccrhsub07, double_integrator_QP_solver_sub07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_ccrhsl07, double_integrator_QP_solver_slb07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_rd07);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_22_17(params->A8, double_integrator_QP_solver_ccrhsp07, double_integrator_QP_solver_sp07, double_integrator_QP_solver_rd07);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_rd07, double_integrator_QP_solver_Lbyrd07);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_V06, double_integrator_QP_solver_Lbyrd06, double_integrator_QP_solver_W07, double_integrator_QP_solver_Lbyrd07, double_integrator_QP_solver_beta06);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd06, double_integrator_QP_solver_yy05, double_integrator_QP_solver_beta06, double_integrator_QP_solver_bmy06);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld06, double_integrator_QP_solver_bmy06, double_integrator_QP_solver_yy06);
double_integrator_QP_solver_LA_VSUB6_INDEXED_17_4_4(double_integrator_QP_solver_ccrhsub08, double_integrator_QP_solver_sub08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_ccrhsl08, double_integrator_QP_solver_slb08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_rd08);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_22_17(params->A9, double_integrator_QP_solver_ccrhsp08, double_integrator_QP_solver_sp08, double_integrator_QP_solver_rd08);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_rd08, double_integrator_QP_solver_Lbyrd08);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_V07, double_integrator_QP_solver_Lbyrd07, double_integrator_QP_solver_W08, double_integrator_QP_solver_Lbyrd08, double_integrator_QP_solver_beta07);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd07, double_integrator_QP_solver_yy06, double_integrator_QP_solver_beta07, double_integrator_QP_solver_bmy07);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld07, double_integrator_QP_solver_bmy07, double_integrator_QP_solver_yy07);
double_integrator_QP_solver_LA_VSUB6_INDEXED_17_4_4(double_integrator_QP_solver_ccrhsub09, double_integrator_QP_solver_sub09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_ccrhsl09, double_integrator_QP_solver_slb09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_rd09);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_22_17(params->A10, double_integrator_QP_solver_ccrhsp09, double_integrator_QP_solver_sp09, double_integrator_QP_solver_rd09);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_rd09, double_integrator_QP_solver_Lbyrd09);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_V08, double_integrator_QP_solver_Lbyrd08, double_integrator_QP_solver_W09, double_integrator_QP_solver_Lbyrd09, double_integrator_QP_solver_beta08);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd08, double_integrator_QP_solver_yy07, double_integrator_QP_solver_beta08, double_integrator_QP_solver_bmy08);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld08, double_integrator_QP_solver_bmy08, double_integrator_QP_solver_yy08);
double_integrator_QP_solver_LA_VSUB6_INDEXED_17_4_4(double_integrator_QP_solver_ccrhsub10, double_integrator_QP_solver_sub10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_ccrhsl10, double_integrator_QP_solver_slb10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_rd10);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_22_17(params->A11, double_integrator_QP_solver_ccrhsp10, double_integrator_QP_solver_sp10, double_integrator_QP_solver_rd10);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_rd10, double_integrator_QP_solver_Lbyrd10);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_V09, double_integrator_QP_solver_Lbyrd09, double_integrator_QP_solver_W10, double_integrator_QP_solver_Lbyrd10, double_integrator_QP_solver_beta09);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd09, double_integrator_QP_solver_yy08, double_integrator_QP_solver_beta09, double_integrator_QP_solver_bmy09);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld09, double_integrator_QP_solver_bmy09, double_integrator_QP_solver_yy09);
double_integrator_QP_solver_LA_VSUB6_INDEXED_17_4_4(double_integrator_QP_solver_ccrhsub11, double_integrator_QP_solver_sub11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_ccrhsl11, double_integrator_QP_solver_slb11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_rd11);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_22_17(params->A12, double_integrator_QP_solver_ccrhsp11, double_integrator_QP_solver_sp11, double_integrator_QP_solver_rd11);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_rd11, double_integrator_QP_solver_Lbyrd11);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_V10, double_integrator_QP_solver_Lbyrd10, double_integrator_QP_solver_W11, double_integrator_QP_solver_Lbyrd11, double_integrator_QP_solver_beta10);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_yy09, double_integrator_QP_solver_beta10, double_integrator_QP_solver_bmy10);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_bmy10, double_integrator_QP_solver_yy10);
double_integrator_QP_solver_LA_VSUB6_INDEXED_17_4_4(double_integrator_QP_solver_ccrhsub12, double_integrator_QP_solver_sub12, double_integrator_QP_solver_ubIdx12, double_integrator_QP_solver_ccrhsl12, double_integrator_QP_solver_slb12, double_integrator_QP_solver_lbIdx12, double_integrator_QP_solver_rd12);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_22_17(params->A13, double_integrator_QP_solver_ccrhsp12, double_integrator_QP_solver_sp12, double_integrator_QP_solver_rd12);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_17(double_integrator_QP_solver_Phi12, double_integrator_QP_solver_rd12, double_integrator_QP_solver_Lbyrd12);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_17_17(double_integrator_QP_solver_V11, double_integrator_QP_solver_Lbyrd11, double_integrator_QP_solver_W12, double_integrator_QP_solver_Lbyrd12, double_integrator_QP_solver_beta11);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd11, double_integrator_QP_solver_yy10, double_integrator_QP_solver_beta11, double_integrator_QP_solver_bmy11);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld11, double_integrator_QP_solver_bmy11, double_integrator_QP_solver_yy11);
double_integrator_QP_solver_LA_VSUB6_INDEXED_15_4_4(double_integrator_QP_solver_ccrhsub13, double_integrator_QP_solver_sub13, double_integrator_QP_solver_ubIdx13, double_integrator_QP_solver_ccrhsl13, double_integrator_QP_solver_slb13, double_integrator_QP_solver_lbIdx13, double_integrator_QP_solver_rd13);
double_integrator_QP_solver_LA_DENSE_MTVMADD2_22_15(params->A14, double_integrator_QP_solver_ccrhsp13, double_integrator_QP_solver_sp13, double_integrator_QP_solver_rd13);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_15(double_integrator_QP_solver_Phi13, double_integrator_QP_solver_rd13, double_integrator_QP_solver_Lbyrd13);
double_integrator_QP_solver_LA_DENSE_2MVMADD_7_17_15(double_integrator_QP_solver_V12, double_integrator_QP_solver_Lbyrd12, double_integrator_QP_solver_W13, double_integrator_QP_solver_Lbyrd13, double_integrator_QP_solver_beta12);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_7_7(double_integrator_QP_solver_Lsd12, double_integrator_QP_solver_yy11, double_integrator_QP_solver_beta12, double_integrator_QP_solver_bmy12);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_7(double_integrator_QP_solver_Ld12, double_integrator_QP_solver_bmy12, double_integrator_QP_solver_yy12);
double_integrator_QP_solver_LA_VSUB6_INDEXED_4_4_4(double_integrator_QP_solver_ccrhsub14, double_integrator_QP_solver_sub14, double_integrator_QP_solver_ubIdx14, double_integrator_QP_solver_ccrhsl14, double_integrator_QP_solver_slb14, double_integrator_QP_solver_lbIdx14, double_integrator_QP_solver_rd14);
double_integrator_QP_solver_LA_DIAG_FORWARDSUB_4(double_integrator_QP_solver_Phi14, double_integrator_QP_solver_rd14, double_integrator_QP_solver_Lbyrd14);
double_integrator_QP_solver_LA_DENSE_DIAGZERO_2MVMADD_4_15_4(double_integrator_QP_solver_V13, double_integrator_QP_solver_Lbyrd13, double_integrator_QP_solver_W14, double_integrator_QP_solver_Lbyrd14, double_integrator_QP_solver_beta13);
double_integrator_QP_solver_LA_DENSE_MVMSUB1_4_7(double_integrator_QP_solver_Lsd13, double_integrator_QP_solver_yy12, double_integrator_QP_solver_beta13, double_integrator_QP_solver_bmy13);
double_integrator_QP_solver_LA_DENSE_FORWARDSUB_4(double_integrator_QP_solver_Ld13, double_integrator_QP_solver_bmy13, double_integrator_QP_solver_yy13);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_4(double_integrator_QP_solver_Ld13, double_integrator_QP_solver_yy13, double_integrator_QP_solver_dvcc13);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_4_7(double_integrator_QP_solver_Lsd13, double_integrator_QP_solver_dvcc13, double_integrator_QP_solver_yy12, double_integrator_QP_solver_bmy12);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld12, double_integrator_QP_solver_bmy12, double_integrator_QP_solver_dvcc12);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd12, double_integrator_QP_solver_dvcc12, double_integrator_QP_solver_yy11, double_integrator_QP_solver_bmy11);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld11, double_integrator_QP_solver_bmy11, double_integrator_QP_solver_dvcc11);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd11, double_integrator_QP_solver_dvcc11, double_integrator_QP_solver_yy10, double_integrator_QP_solver_bmy10);
double_integrator_QP_solver_LA_DENSE_BACKWARDSUB_7(double_integrator_QP_solver_Ld10, double_integrator_QP_solver_bmy10, double_integrator_QP_solver_dvcc10);
double_integrator_QP_solver_LA_DENSE_MTVMSUB_7_7(double_integrator_QP_solver_Lsd10, double_integrator_QP_solver_dvcc10, double_integrator_QP_solver_yy09, double_integrator_QP_solver_bmy09);
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
double_integrator_QP_solver_LA_DENSE_MTVM_7_17(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc00, double_integrator_QP_solver_grad_eq00);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc01, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc00, double_integrator_QP_solver_grad_eq01);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc02, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc01, double_integrator_QP_solver_grad_eq02);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc03, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc02, double_integrator_QP_solver_grad_eq03);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc04, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc03, double_integrator_QP_solver_grad_eq04);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc05, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc04, double_integrator_QP_solver_grad_eq05);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc06, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc05, double_integrator_QP_solver_grad_eq06);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc07, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc06, double_integrator_QP_solver_grad_eq07);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc08, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc07, double_integrator_QP_solver_grad_eq08);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc09, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc08, double_integrator_QP_solver_grad_eq09);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc10, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc09, double_integrator_QP_solver_grad_eq10);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc11, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc10, double_integrator_QP_solver_grad_eq11);
double_integrator_QP_solver_LA_DENSE_MTVM2_7_17_7(double_integrator_QP_solver_C00, double_integrator_QP_solver_dvcc12, double_integrator_QP_solver_D01, double_integrator_QP_solver_dvcc11, double_integrator_QP_solver_grad_eq12);
double_integrator_QP_solver_LA_DENSE_MTVM2_4_15_7(double_integrator_QP_solver_C13, double_integrator_QP_solver_dvcc13, double_integrator_QP_solver_D13, double_integrator_QP_solver_dvcc12, double_integrator_QP_solver_grad_eq13);
double_integrator_QP_solver_LA_DIAGZERO_MTVM_4_4(double_integrator_QP_solver_D14, double_integrator_QP_solver_dvcc13, double_integrator_QP_solver_grad_eq14);
double_integrator_QP_solver_LA_VSUB_240(double_integrator_QP_solver_rd, double_integrator_QP_solver_grad_eq, double_integrator_QP_solver_rd);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi00, double_integrator_QP_solver_rd00, double_integrator_QP_solver_dzcc00);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi01, double_integrator_QP_solver_rd01, double_integrator_QP_solver_dzcc01);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi02, double_integrator_QP_solver_rd02, double_integrator_QP_solver_dzcc02);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi03, double_integrator_QP_solver_rd03, double_integrator_QP_solver_dzcc03);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi04, double_integrator_QP_solver_rd04, double_integrator_QP_solver_dzcc04);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi05, double_integrator_QP_solver_rd05, double_integrator_QP_solver_dzcc05);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi06, double_integrator_QP_solver_rd06, double_integrator_QP_solver_dzcc06);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi07, double_integrator_QP_solver_rd07, double_integrator_QP_solver_dzcc07);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi08, double_integrator_QP_solver_rd08, double_integrator_QP_solver_dzcc08);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi09, double_integrator_QP_solver_rd09, double_integrator_QP_solver_dzcc09);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi10, double_integrator_QP_solver_rd10, double_integrator_QP_solver_dzcc10);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi11, double_integrator_QP_solver_rd11, double_integrator_QP_solver_dzcc11);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_17(double_integrator_QP_solver_Phi12, double_integrator_QP_solver_rd12, double_integrator_QP_solver_dzcc12);
double_integrator_QP_solver_LA_DENSE_FORWARDBACKWARDSUB_15(double_integrator_QP_solver_Phi13, double_integrator_QP_solver_rd13, double_integrator_QP_solver_dzcc13);
double_integrator_QP_solver_LA_DIAG_FORWARDBACKWARDSUB_4(double_integrator_QP_solver_Phi14, double_integrator_QP_solver_rd14, double_integrator_QP_solver_dzcc14);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_7(double_integrator_QP_solver_ccrhsl00, double_integrator_QP_solver_slb00, double_integrator_QP_solver_llbbyslb00, double_integrator_QP_solver_dzcc00, double_integrator_QP_solver_lbIdx00, double_integrator_QP_solver_dllbcc00);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_6(double_integrator_QP_solver_ccrhsub00, double_integrator_QP_solver_sub00, double_integrator_QP_solver_lubbysub00, double_integrator_QP_solver_dzcc00, double_integrator_QP_solver_ubIdx00, double_integrator_QP_solver_dlubcc00);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_22_17(params->A1, double_integrator_QP_solver_dzcc00, double_integrator_QP_solver_ccrhsp00, double_integrator_QP_solver_sp00, double_integrator_QP_solver_lp00, double_integrator_QP_solver_dlp_cc00);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_ccrhsl01, double_integrator_QP_solver_slb01, double_integrator_QP_solver_llbbyslb01, double_integrator_QP_solver_dzcc01, double_integrator_QP_solver_lbIdx01, double_integrator_QP_solver_dllbcc01);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_ccrhsub01, double_integrator_QP_solver_sub01, double_integrator_QP_solver_lubbysub01, double_integrator_QP_solver_dzcc01, double_integrator_QP_solver_ubIdx01, double_integrator_QP_solver_dlubcc01);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_22_17(params->A2, double_integrator_QP_solver_dzcc01, double_integrator_QP_solver_ccrhsp01, double_integrator_QP_solver_sp01, double_integrator_QP_solver_lp01, double_integrator_QP_solver_dlp_cc01);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_ccrhsl02, double_integrator_QP_solver_slb02, double_integrator_QP_solver_llbbyslb02, double_integrator_QP_solver_dzcc02, double_integrator_QP_solver_lbIdx02, double_integrator_QP_solver_dllbcc02);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_ccrhsub02, double_integrator_QP_solver_sub02, double_integrator_QP_solver_lubbysub02, double_integrator_QP_solver_dzcc02, double_integrator_QP_solver_ubIdx02, double_integrator_QP_solver_dlubcc02);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_22_17(params->A3, double_integrator_QP_solver_dzcc02, double_integrator_QP_solver_ccrhsp02, double_integrator_QP_solver_sp02, double_integrator_QP_solver_lp02, double_integrator_QP_solver_dlp_cc02);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_ccrhsl03, double_integrator_QP_solver_slb03, double_integrator_QP_solver_llbbyslb03, double_integrator_QP_solver_dzcc03, double_integrator_QP_solver_lbIdx03, double_integrator_QP_solver_dllbcc03);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_ccrhsub03, double_integrator_QP_solver_sub03, double_integrator_QP_solver_lubbysub03, double_integrator_QP_solver_dzcc03, double_integrator_QP_solver_ubIdx03, double_integrator_QP_solver_dlubcc03);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_22_17(params->A4, double_integrator_QP_solver_dzcc03, double_integrator_QP_solver_ccrhsp03, double_integrator_QP_solver_sp03, double_integrator_QP_solver_lp03, double_integrator_QP_solver_dlp_cc03);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_ccrhsl04, double_integrator_QP_solver_slb04, double_integrator_QP_solver_llbbyslb04, double_integrator_QP_solver_dzcc04, double_integrator_QP_solver_lbIdx04, double_integrator_QP_solver_dllbcc04);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_ccrhsub04, double_integrator_QP_solver_sub04, double_integrator_QP_solver_lubbysub04, double_integrator_QP_solver_dzcc04, double_integrator_QP_solver_ubIdx04, double_integrator_QP_solver_dlubcc04);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_22_17(params->A5, double_integrator_QP_solver_dzcc04, double_integrator_QP_solver_ccrhsp04, double_integrator_QP_solver_sp04, double_integrator_QP_solver_lp04, double_integrator_QP_solver_dlp_cc04);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_ccrhsl05, double_integrator_QP_solver_slb05, double_integrator_QP_solver_llbbyslb05, double_integrator_QP_solver_dzcc05, double_integrator_QP_solver_lbIdx05, double_integrator_QP_solver_dllbcc05);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_ccrhsub05, double_integrator_QP_solver_sub05, double_integrator_QP_solver_lubbysub05, double_integrator_QP_solver_dzcc05, double_integrator_QP_solver_ubIdx05, double_integrator_QP_solver_dlubcc05);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_22_17(params->A6, double_integrator_QP_solver_dzcc05, double_integrator_QP_solver_ccrhsp05, double_integrator_QP_solver_sp05, double_integrator_QP_solver_lp05, double_integrator_QP_solver_dlp_cc05);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_ccrhsl06, double_integrator_QP_solver_slb06, double_integrator_QP_solver_llbbyslb06, double_integrator_QP_solver_dzcc06, double_integrator_QP_solver_lbIdx06, double_integrator_QP_solver_dllbcc06);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_ccrhsub06, double_integrator_QP_solver_sub06, double_integrator_QP_solver_lubbysub06, double_integrator_QP_solver_dzcc06, double_integrator_QP_solver_ubIdx06, double_integrator_QP_solver_dlubcc06);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_22_17(params->A7, double_integrator_QP_solver_dzcc06, double_integrator_QP_solver_ccrhsp06, double_integrator_QP_solver_sp06, double_integrator_QP_solver_lp06, double_integrator_QP_solver_dlp_cc06);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_ccrhsl07, double_integrator_QP_solver_slb07, double_integrator_QP_solver_llbbyslb07, double_integrator_QP_solver_dzcc07, double_integrator_QP_solver_lbIdx07, double_integrator_QP_solver_dllbcc07);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_ccrhsub07, double_integrator_QP_solver_sub07, double_integrator_QP_solver_lubbysub07, double_integrator_QP_solver_dzcc07, double_integrator_QP_solver_ubIdx07, double_integrator_QP_solver_dlubcc07);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_22_17(params->A8, double_integrator_QP_solver_dzcc07, double_integrator_QP_solver_ccrhsp07, double_integrator_QP_solver_sp07, double_integrator_QP_solver_lp07, double_integrator_QP_solver_dlp_cc07);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_ccrhsl08, double_integrator_QP_solver_slb08, double_integrator_QP_solver_llbbyslb08, double_integrator_QP_solver_dzcc08, double_integrator_QP_solver_lbIdx08, double_integrator_QP_solver_dllbcc08);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_ccrhsub08, double_integrator_QP_solver_sub08, double_integrator_QP_solver_lubbysub08, double_integrator_QP_solver_dzcc08, double_integrator_QP_solver_ubIdx08, double_integrator_QP_solver_dlubcc08);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_22_17(params->A9, double_integrator_QP_solver_dzcc08, double_integrator_QP_solver_ccrhsp08, double_integrator_QP_solver_sp08, double_integrator_QP_solver_lp08, double_integrator_QP_solver_dlp_cc08);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_ccrhsl09, double_integrator_QP_solver_slb09, double_integrator_QP_solver_llbbyslb09, double_integrator_QP_solver_dzcc09, double_integrator_QP_solver_lbIdx09, double_integrator_QP_solver_dllbcc09);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_ccrhsub09, double_integrator_QP_solver_sub09, double_integrator_QP_solver_lubbysub09, double_integrator_QP_solver_dzcc09, double_integrator_QP_solver_ubIdx09, double_integrator_QP_solver_dlubcc09);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_22_17(params->A10, double_integrator_QP_solver_dzcc09, double_integrator_QP_solver_ccrhsp09, double_integrator_QP_solver_sp09, double_integrator_QP_solver_lp09, double_integrator_QP_solver_dlp_cc09);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_ccrhsl10, double_integrator_QP_solver_slb10, double_integrator_QP_solver_llbbyslb10, double_integrator_QP_solver_dzcc10, double_integrator_QP_solver_lbIdx10, double_integrator_QP_solver_dllbcc10);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_ccrhsub10, double_integrator_QP_solver_sub10, double_integrator_QP_solver_lubbysub10, double_integrator_QP_solver_dzcc10, double_integrator_QP_solver_ubIdx10, double_integrator_QP_solver_dlubcc10);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_22_17(params->A11, double_integrator_QP_solver_dzcc10, double_integrator_QP_solver_ccrhsp10, double_integrator_QP_solver_sp10, double_integrator_QP_solver_lp10, double_integrator_QP_solver_dlp_cc10);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_ccrhsl11, double_integrator_QP_solver_slb11, double_integrator_QP_solver_llbbyslb11, double_integrator_QP_solver_dzcc11, double_integrator_QP_solver_lbIdx11, double_integrator_QP_solver_dllbcc11);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_ccrhsub11, double_integrator_QP_solver_sub11, double_integrator_QP_solver_lubbysub11, double_integrator_QP_solver_dzcc11, double_integrator_QP_solver_ubIdx11, double_integrator_QP_solver_dlubcc11);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_22_17(params->A12, double_integrator_QP_solver_dzcc11, double_integrator_QP_solver_ccrhsp11, double_integrator_QP_solver_sp11, double_integrator_QP_solver_lp11, double_integrator_QP_solver_dlp_cc11);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_ccrhsl12, double_integrator_QP_solver_slb12, double_integrator_QP_solver_llbbyslb12, double_integrator_QP_solver_dzcc12, double_integrator_QP_solver_lbIdx12, double_integrator_QP_solver_dllbcc12);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_ccrhsub12, double_integrator_QP_solver_sub12, double_integrator_QP_solver_lubbysub12, double_integrator_QP_solver_dzcc12, double_integrator_QP_solver_ubIdx12, double_integrator_QP_solver_dlubcc12);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_22_17(params->A13, double_integrator_QP_solver_dzcc12, double_integrator_QP_solver_ccrhsp12, double_integrator_QP_solver_sp12, double_integrator_QP_solver_lp12, double_integrator_QP_solver_dlp_cc12);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_ccrhsl13, double_integrator_QP_solver_slb13, double_integrator_QP_solver_llbbyslb13, double_integrator_QP_solver_dzcc13, double_integrator_QP_solver_lbIdx13, double_integrator_QP_solver_dllbcc13);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_ccrhsub13, double_integrator_QP_solver_sub13, double_integrator_QP_solver_lubbysub13, double_integrator_QP_solver_dzcc13, double_integrator_QP_solver_ubIdx13, double_integrator_QP_solver_dlubcc13);
double_integrator_QP_solver_LA_DENSE_MVMSUB5_22_15(params->A14, double_integrator_QP_solver_dzcc13, double_integrator_QP_solver_ccrhsp13, double_integrator_QP_solver_sp13, double_integrator_QP_solver_lp13, double_integrator_QP_solver_dlp_cc13);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTSUB_INDEXED_4(double_integrator_QP_solver_ccrhsl14, double_integrator_QP_solver_slb14, double_integrator_QP_solver_llbbyslb14, double_integrator_QP_solver_dzcc14, double_integrator_QP_solver_lbIdx14, double_integrator_QP_solver_dllbcc14);
double_integrator_QP_solver_LA_VEC_DIVSUB_MULTADD_INDEXED_4(double_integrator_QP_solver_ccrhsub14, double_integrator_QP_solver_sub14, double_integrator_QP_solver_lubbysub14, double_integrator_QP_solver_dzcc14, double_integrator_QP_solver_ubIdx14, double_integrator_QP_solver_dlubcc14);
double_integrator_QP_solver_LA_VSUB7_433(double_integrator_QP_solver_l, double_integrator_QP_solver_ccrhs, double_integrator_QP_solver_s, double_integrator_QP_solver_dl_cc, double_integrator_QP_solver_ds_cc);
double_integrator_QP_solver_LA_VADD_240(double_integrator_QP_solver_dz_cc, double_integrator_QP_solver_dz_aff);
double_integrator_QP_solver_LA_VADD_95(double_integrator_QP_solver_dv_cc, double_integrator_QP_solver_dv_aff);
double_integrator_QP_solver_LA_VADD_433(double_integrator_QP_solver_dl_cc, double_integrator_QP_solver_dl_aff);
double_integrator_QP_solver_LA_VADD_433(double_integrator_QP_solver_ds_cc, double_integrator_QP_solver_ds_aff);
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
output->z11[6] = double_integrator_QP_solver_z10[12];
output->z12[0] = double_integrator_QP_solver_z11[0];
output->z12[1] = double_integrator_QP_solver_z11[1];
output->z12[2] = double_integrator_QP_solver_z11[2];
output->z12[3] = double_integrator_QP_solver_z11[3];
output->z12[4] = double_integrator_QP_solver_z11[4];
output->z12[5] = double_integrator_QP_solver_z11[5];
output->z12[6] = double_integrator_QP_solver_z11[12];
output->z13[0] = double_integrator_QP_solver_z12[0];
output->z13[1] = double_integrator_QP_solver_z12[1];
output->z13[2] = double_integrator_QP_solver_z12[2];
output->z13[3] = double_integrator_QP_solver_z12[3];
output->z13[4] = double_integrator_QP_solver_z12[4];
output->z13[5] = double_integrator_QP_solver_z12[5];
output->z13[6] = double_integrator_QP_solver_z12[12];
output->z14[0] = double_integrator_QP_solver_z13[0];
output->z14[1] = double_integrator_QP_solver_z13[1];
output->z14[2] = double_integrator_QP_solver_z13[2];
output->z14[3] = double_integrator_QP_solver_z13[3];
output->z14[4] = double_integrator_QP_solver_z13[4];
output->z14[5] = double_integrator_QP_solver_z13[5];
output->z14[6] = double_integrator_QP_solver_z13[10];
output->z15[0] = double_integrator_QP_solver_z14[0];
output->z15[1] = double_integrator_QP_solver_z14[1];
output->z15[2] = double_integrator_QP_solver_z14[2];
output->z15[3] = double_integrator_QP_solver_z14[3];

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
