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

#include "mex.h"
#include "../include/double_integrator_QP_solver.h"

/* copy functions */
void copyCArrayToM(double_integrator_QP_solver_FLOAT *src, double *dest, int dim) {
    while (dim--) {
        *dest++ = (double)*src++;
    }
}
void copyMArrayToC(double *src, double_integrator_QP_solver_FLOAT *dest, int dim) {
    while (dim--) {
        *dest++ = (double_integrator_QP_solver_FLOAT)*src++;
    }
}


/* THE mex-function */
void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] )  
{
	/* define variables */	
	mxArray *par;
	mxArray *outvar;
	const mxArray *PARAMS = prhs[0];
	double *pvalue;
	int i;
	int exitflag;
	const char *fname;
	const char *outputnames[15] = {"z1","z2","z3","z4","z5","z6","z7","z8","z9","z10","z11","z12","z13","z14","z15"};
	const char *infofields[15] = { "it",
		                       "res_eq",
			                   "res_ineq",
		                       "pobj",
		                       "dobj",
		                       "dgap",
							   "rdgap",
							   "mu",
							   "mu_aff",
							   "sigma",
		                       "lsit_aff",
		                       "lsit_cc",
		                       "step_aff",
							   "step_cc",
							   "solvetime"};
	double_integrator_QP_solver_params params;
	double_integrator_QP_solver_output output;
	double_integrator_QP_solver_info info;
	
	/* Check for proper number of arguments */
    if (nrhs != 1) {
        mexErrMsgTxt("This function requires exactly 1 input: PARAMS struct.\nType 'help double_integrator_QP_solver_mex' for details.");
    }    
	if (nlhs > 3) {
        mexErrMsgTxt("This function returns at most 3 outputs.\nType 'help double_integrator_QP_solver_mex' for details.");
    }

	/* Check whether params is actually a structure */
	if( !mxIsStruct(PARAMS) ) {
		mexErrMsgTxt("PARAMS must be a structure.");
	}

	/* copy parameters into the right location */
	par = mxGetField(PARAMS, 0, "f1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f1 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f1 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f1, 14);

	par = mxGetField(PARAMS, 0, "lb1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb1 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb1 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb1, 14);

	par = mxGetField(PARAMS, 0, "ub1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub1 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub1 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub1, 6);

	par = mxGetField(PARAMS, 0, "C1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C1 must be a double.");
    }
    if( mxGetM(par) != 9 || mxGetN(par) != 14 ) {
    mexErrMsgTxt("PARAMS.C1 must be of size [9 x 14]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C1, 126);

	par = mxGetField(PARAMS, 0, "e1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.e1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.e1 must be a double.");
    }
    if( mxGetM(par) != 9 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.e1 must be of size [9 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.e1, 9);

	par = mxGetField(PARAMS, 0, "f2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f2 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f2 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f2, 14);

	par = mxGetField(PARAMS, 0, "lb2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb2 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb2 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb2, 14);

	par = mxGetField(PARAMS, 0, "ub2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub2 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub2 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub2, 6);

	par = mxGetField(PARAMS, 0, "C2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C2 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 14 ) {
    mexErrMsgTxt("PARAMS.C2 must be of size [5 x 14]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C2, 70);

	par = mxGetField(PARAMS, 0, "e2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.e2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.e2 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.e2 must be of size [5 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.e2, 5);

	par = mxGetField(PARAMS, 0, "f3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f3 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f3 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f3, 14);

	par = mxGetField(PARAMS, 0, "lb3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb3 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb3 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb3, 14);

	par = mxGetField(PARAMS, 0, "ub3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub3 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub3 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub3, 6);

	par = mxGetField(PARAMS, 0, "C3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C3 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 14 ) {
    mexErrMsgTxt("PARAMS.C3 must be of size [5 x 14]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C3, 70);

	par = mxGetField(PARAMS, 0, "e3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.e3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.e3 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.e3 must be of size [5 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.e3, 5);

	par = mxGetField(PARAMS, 0, "f4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f4 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f4 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f4, 14);

	par = mxGetField(PARAMS, 0, "lb4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb4 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb4 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb4, 14);

	par = mxGetField(PARAMS, 0, "ub4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub4 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub4 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub4, 6);

	par = mxGetField(PARAMS, 0, "C4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C4 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 14 ) {
    mexErrMsgTxt("PARAMS.C4 must be of size [5 x 14]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C4, 70);

	par = mxGetField(PARAMS, 0, "e4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.e4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.e4 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.e4 must be of size [5 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.e4, 5);

	par = mxGetField(PARAMS, 0, "f5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f5 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f5 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f5, 14);

	par = mxGetField(PARAMS, 0, "lb5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb5 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb5 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb5, 14);

	par = mxGetField(PARAMS, 0, "ub5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub5 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub5 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub5, 6);

	par = mxGetField(PARAMS, 0, "C5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C5 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 14 ) {
    mexErrMsgTxt("PARAMS.C5 must be of size [5 x 14]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C5, 70);

	par = mxGetField(PARAMS, 0, "e5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.e5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.e5 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.e5 must be of size [5 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.e5, 5);

	par = mxGetField(PARAMS, 0, "f6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f6 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f6 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f6, 14);

	par = mxGetField(PARAMS, 0, "lb6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb6 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb6 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb6, 14);

	par = mxGetField(PARAMS, 0, "ub6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub6 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub6 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub6, 6);

	par = mxGetField(PARAMS, 0, "C6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C6 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 14 ) {
    mexErrMsgTxt("PARAMS.C6 must be of size [5 x 14]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C6, 70);

	par = mxGetField(PARAMS, 0, "e6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.e6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.e6 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.e6 must be of size [5 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.e6, 5);

	par = mxGetField(PARAMS, 0, "f7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f7 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f7 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f7, 14);

	par = mxGetField(PARAMS, 0, "lb7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb7 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb7 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb7, 14);

	par = mxGetField(PARAMS, 0, "ub7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub7 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub7 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub7, 6);

	par = mxGetField(PARAMS, 0, "C7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C7 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 14 ) {
    mexErrMsgTxt("PARAMS.C7 must be of size [5 x 14]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C7, 70);

	par = mxGetField(PARAMS, 0, "e7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.e7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.e7 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.e7 must be of size [5 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.e7, 5);

	par = mxGetField(PARAMS, 0, "f8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f8 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f8 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f8, 14);

	par = mxGetField(PARAMS, 0, "lb8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb8 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb8 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb8, 14);

	par = mxGetField(PARAMS, 0, "ub8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub8 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub8 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub8, 6);

	par = mxGetField(PARAMS, 0, "C8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C8 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 14 ) {
    mexErrMsgTxt("PARAMS.C8 must be of size [5 x 14]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C8, 70);

	par = mxGetField(PARAMS, 0, "e8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.e8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.e8 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.e8 must be of size [5 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.e8, 5);

	par = mxGetField(PARAMS, 0, "f9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f9 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f9 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f9, 14);

	par = mxGetField(PARAMS, 0, "lb9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb9 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb9 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb9, 14);

	par = mxGetField(PARAMS, 0, "ub9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub9 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub9 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub9, 6);

	par = mxGetField(PARAMS, 0, "C9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C9 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 14 ) {
    mexErrMsgTxt("PARAMS.C9 must be of size [5 x 14]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C9, 70);

	par = mxGetField(PARAMS, 0, "e9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.e9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.e9 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.e9 must be of size [5 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.e9, 5);

	par = mxGetField(PARAMS, 0, "f10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f10 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f10 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f10, 14);

	par = mxGetField(PARAMS, 0, "lb10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb10 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb10 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb10, 14);

	par = mxGetField(PARAMS, 0, "ub10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub10 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub10 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub10, 6);

	par = mxGetField(PARAMS, 0, "C10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C10 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 14 ) {
    mexErrMsgTxt("PARAMS.C10 must be of size [5 x 14]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C10, 70);

	par = mxGetField(PARAMS, 0, "e10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.e10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.e10 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.e10 must be of size [5 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.e10, 5);

	par = mxGetField(PARAMS, 0, "f11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f11 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f11 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f11, 14);

	par = mxGetField(PARAMS, 0, "lb11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb11 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb11 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb11, 14);

	par = mxGetField(PARAMS, 0, "ub11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub11 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub11 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub11, 6);

	par = mxGetField(PARAMS, 0, "C11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C11 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 14 ) {
    mexErrMsgTxt("PARAMS.C11 must be of size [5 x 14]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C11, 70);

	par = mxGetField(PARAMS, 0, "e11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.e11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.e11 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.e11 must be of size [5 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.e11, 5);

	par = mxGetField(PARAMS, 0, "f12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f12 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f12 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f12, 14);

	par = mxGetField(PARAMS, 0, "lb12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb12 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb12 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb12, 14);

	par = mxGetField(PARAMS, 0, "ub12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub12 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub12 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub12, 6);

	par = mxGetField(PARAMS, 0, "C12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C12 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 14 ) {
    mexErrMsgTxt("PARAMS.C12 must be of size [5 x 14]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C12, 70);

	par = mxGetField(PARAMS, 0, "e12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.e12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.e12 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.e12 must be of size [5 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.e12, 5);

	par = mxGetField(PARAMS, 0, "f13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f13 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f13 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f13, 14);

	par = mxGetField(PARAMS, 0, "lb13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb13 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb13 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb13, 14);

	par = mxGetField(PARAMS, 0, "ub13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub13 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub13 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub13, 6);

	par = mxGetField(PARAMS, 0, "C13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C13 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 14 ) {
    mexErrMsgTxt("PARAMS.C13 must be of size [5 x 14]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C13, 70);

	par = mxGetField(PARAMS, 0, "e13");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.e13 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.e13 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.e13 must be of size [5 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.e13, 5);

	par = mxGetField(PARAMS, 0, "f14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f14 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f14 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f14, 14);

	par = mxGetField(PARAMS, 0, "lb14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb14 must be a double.");
    }
    if( mxGetM(par) != 14 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb14 must be of size [14 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb14, 14);

	par = mxGetField(PARAMS, 0, "ub14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub14 must be a double.");
    }
    if( mxGetM(par) != 6 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub14 must be of size [6 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub14, 6);

	par = mxGetField(PARAMS, 0, "C14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.C14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.C14 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 14 ) {
    mexErrMsgTxt("PARAMS.C14 must be of size [5 x 14]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.C14, 70);

	par = mxGetField(PARAMS, 0, "e14");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.e14 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.e14 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.e14 must be of size [5 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.e14, 5);

	par = mxGetField(PARAMS, 0, "lb15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb15 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb15 must be of size [5 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb15, 5);

	par = mxGetField(PARAMS, 0, "ub15");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub15 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub15 must be a double.");
    }
    if( mxGetM(par) != 5 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub15 must be of size [5 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub15, 5);

	/* call solver */
	exitflag = double_integrator_QP_solver_solve(&params, &output, &info);
	
	/* copy output to matlab arrays */
	plhs[0] = mxCreateStructMatrix(1, 1, 15, outputnames);
	outvar = mxCreateDoubleMatrix(6, 1, mxREAL);
	copyCArrayToM( output.z1, mxGetPr(outvar), 6);
	mxSetField(plhs[0], 0, "z1", outvar);

	outvar = mxCreateDoubleMatrix(6, 1, mxREAL);
	copyCArrayToM( output.z2, mxGetPr(outvar), 6);
	mxSetField(plhs[0], 0, "z2", outvar);

	outvar = mxCreateDoubleMatrix(6, 1, mxREAL);
	copyCArrayToM( output.z3, mxGetPr(outvar), 6);
	mxSetField(plhs[0], 0, "z3", outvar);

	outvar = mxCreateDoubleMatrix(6, 1, mxREAL);
	copyCArrayToM( output.z4, mxGetPr(outvar), 6);
	mxSetField(plhs[0], 0, "z4", outvar);

	outvar = mxCreateDoubleMatrix(6, 1, mxREAL);
	copyCArrayToM( output.z5, mxGetPr(outvar), 6);
	mxSetField(plhs[0], 0, "z5", outvar);

	outvar = mxCreateDoubleMatrix(6, 1, mxREAL);
	copyCArrayToM( output.z6, mxGetPr(outvar), 6);
	mxSetField(plhs[0], 0, "z6", outvar);

	outvar = mxCreateDoubleMatrix(6, 1, mxREAL);
	copyCArrayToM( output.z7, mxGetPr(outvar), 6);
	mxSetField(plhs[0], 0, "z7", outvar);

	outvar = mxCreateDoubleMatrix(6, 1, mxREAL);
	copyCArrayToM( output.z8, mxGetPr(outvar), 6);
	mxSetField(plhs[0], 0, "z8", outvar);

	outvar = mxCreateDoubleMatrix(6, 1, mxREAL);
	copyCArrayToM( output.z9, mxGetPr(outvar), 6);
	mxSetField(plhs[0], 0, "z9", outvar);

	outvar = mxCreateDoubleMatrix(6, 1, mxREAL);
	copyCArrayToM( output.z10, mxGetPr(outvar), 6);
	mxSetField(plhs[0], 0, "z10", outvar);

	outvar = mxCreateDoubleMatrix(6, 1, mxREAL);
	copyCArrayToM( output.z11, mxGetPr(outvar), 6);
	mxSetField(plhs[0], 0, "z11", outvar);

	outvar = mxCreateDoubleMatrix(6, 1, mxREAL);
	copyCArrayToM( output.z12, mxGetPr(outvar), 6);
	mxSetField(plhs[0], 0, "z12", outvar);

	outvar = mxCreateDoubleMatrix(6, 1, mxREAL);
	copyCArrayToM( output.z13, mxGetPr(outvar), 6);
	mxSetField(plhs[0], 0, "z13", outvar);

	outvar = mxCreateDoubleMatrix(6, 1, mxREAL);
	copyCArrayToM( output.z14, mxGetPr(outvar), 6);
	mxSetField(plhs[0], 0, "z14", outvar);

	outvar = mxCreateDoubleMatrix(5, 1, mxREAL);
	copyCArrayToM( output.z15, mxGetPr(outvar), 5);
	mxSetField(plhs[0], 0, "z15", outvar);	

	/* copy exitflag */
	if( nlhs > 1 )
	{
		plhs[1] = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(plhs[1]) = (double)exitflag;
	}

	/* copy info struct */
	if( nlhs > 2 )
	{
		plhs[2] = mxCreateStructMatrix(1, 1, 15, infofields);
		
		/* iterations */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.it;
		mxSetField(plhs[2], 0, "it", outvar);
		
		/* res_eq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_eq;
		mxSetField(plhs[2], 0, "res_eq", outvar);

		/* res_ineq */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.res_ineq;
		mxSetField(plhs[2], 0, "res_ineq", outvar);

		/* pobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.pobj;
		mxSetField(plhs[2], 0, "pobj", outvar);

		/* dobj */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.dobj;
		mxSetField(plhs[2], 0, "dobj", outvar);

		/* dgap */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.dgap;
		mxSetField(plhs[2], 0, "dgap", outvar);

		/* rdgap */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.rdgap;
		mxSetField(plhs[2], 0, "rdgap", outvar);

		/* mu */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.mu;
		mxSetField(plhs[2], 0, "mu", outvar);

		/* mu_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.mu_aff;
		mxSetField(plhs[2], 0, "mu_aff", outvar);

		/* sigma */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.sigma;
		mxSetField(plhs[2], 0, "sigma", outvar);

		/* lsit_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.lsit_aff;
		mxSetField(plhs[2], 0, "lsit_aff", outvar);

		/* lsit_cc */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = (double)info.lsit_cc;
		mxSetField(plhs[2], 0, "lsit_cc", outvar);

		/* step_aff */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.step_aff;
		mxSetField(plhs[2], 0, "step_aff", outvar);

		/* step_cc */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.step_cc;
		mxSetField(plhs[2], 0, "step_cc", outvar);

		/* solver time */
		outvar = mxCreateDoubleMatrix(1, 1, mxREAL);
		*mxGetPr(outvar) = info.solvetime;
		mxSetField(plhs[2], 0, "solvetime", outvar);
	}
}