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
#include "../include/double_integrator_QP_solver_CD.h"

/* copy functions */
void copyCArrayToM(double_integrator_QP_solver_CD_FLOAT *src, double *dest, int dim) {
    while (dim--) {
        *dest++ = (double)*src++;
    }
}
void copyMArrayToC(double *src, double_integrator_QP_solver_CD_FLOAT *dest, int dim) {
    while (dim--) {
        *dest++ = (double_integrator_QP_solver_CD_FLOAT)*src++;
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
	const char *outputnames[12] = {"z1","z2","z3","z4","z5","z6","z7","z8","z9","z10","z11","z12"};
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
	double_integrator_QP_solver_CD_params params;
	double_integrator_QP_solver_CD_output output;
	double_integrator_QP_solver_CD_info info;
	
	/* Check for proper number of arguments */
    if (nrhs != 1) {
        mexErrMsgTxt("This function requires exactly 1 input: PARAMS struct.\nType 'help double_integrator_QP_solver_CD_mex' for details.");
    }    
	if (nlhs > 3) {
        mexErrMsgTxt("This function returns at most 3 outputs.\nType 'help double_integrator_QP_solver_CD_mex' for details.");
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
    if( mxGetM(par) != 20 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f1 must be of size [20 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f1, 20);

	par = mxGetField(PARAMS, 0, "lb1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb1 must be a double.");
    }
    if( mxGetM(par) != 7 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb1 must be of size [7 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb1, 7);

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

	par = mxGetField(PARAMS, 0, "A1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.A1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A1 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 20 ) {
    mexErrMsgTxt("PARAMS.A1 must be of size [28 x 20]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.A1, 560);

	par = mxGetField(PARAMS, 0, "b1");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.b1 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b1 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.b1 must be of size [28 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.b1, 28);

	par = mxGetField(PARAMS, 0, "f2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f2 must be a double.");
    }
    if( mxGetM(par) != 20 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f2 must be of size [20 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f2, 20);

	par = mxGetField(PARAMS, 0, "lb2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb2 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb2 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb2, 4);

	par = mxGetField(PARAMS, 0, "ub2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub2 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub2 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub2, 4);

	par = mxGetField(PARAMS, 0, "A2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.A2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A2 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 20 ) {
    mexErrMsgTxt("PARAMS.A2 must be of size [28 x 20]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.A2, 560);

	par = mxGetField(PARAMS, 0, "b2");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.b2 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b2 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.b2 must be of size [28 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.b2, 28);

	par = mxGetField(PARAMS, 0, "f3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f3 must be a double.");
    }
    if( mxGetM(par) != 20 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f3 must be of size [20 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f3, 20);

	par = mxGetField(PARAMS, 0, "lb3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb3 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb3 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb3, 4);

	par = mxGetField(PARAMS, 0, "ub3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub3 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub3 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub3, 4);

	par = mxGetField(PARAMS, 0, "A3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.A3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A3 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 20 ) {
    mexErrMsgTxt("PARAMS.A3 must be of size [28 x 20]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.A3, 560);

	par = mxGetField(PARAMS, 0, "b3");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.b3 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b3 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.b3 must be of size [28 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.b3, 28);

	par = mxGetField(PARAMS, 0, "f4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f4 must be a double.");
    }
    if( mxGetM(par) != 20 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f4 must be of size [20 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f4, 20);

	par = mxGetField(PARAMS, 0, "lb4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb4 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb4 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb4, 4);

	par = mxGetField(PARAMS, 0, "ub4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub4 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub4 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub4, 4);

	par = mxGetField(PARAMS, 0, "A4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.A4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A4 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 20 ) {
    mexErrMsgTxt("PARAMS.A4 must be of size [28 x 20]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.A4, 560);

	par = mxGetField(PARAMS, 0, "b4");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.b4 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b4 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.b4 must be of size [28 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.b4, 28);

	par = mxGetField(PARAMS, 0, "f5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f5 must be a double.");
    }
    if( mxGetM(par) != 20 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f5 must be of size [20 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f5, 20);

	par = mxGetField(PARAMS, 0, "lb5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb5 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb5 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb5, 4);

	par = mxGetField(PARAMS, 0, "ub5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub5 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub5 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub5, 4);

	par = mxGetField(PARAMS, 0, "A5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.A5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A5 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 20 ) {
    mexErrMsgTxt("PARAMS.A5 must be of size [28 x 20]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.A5, 560);

	par = mxGetField(PARAMS, 0, "b5");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.b5 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b5 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.b5 must be of size [28 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.b5, 28);

	par = mxGetField(PARAMS, 0, "f6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f6 must be a double.");
    }
    if( mxGetM(par) != 20 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f6 must be of size [20 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f6, 20);

	par = mxGetField(PARAMS, 0, "lb6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb6 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb6 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb6, 4);

	par = mxGetField(PARAMS, 0, "ub6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub6 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub6 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub6, 4);

	par = mxGetField(PARAMS, 0, "A6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.A6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A6 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 20 ) {
    mexErrMsgTxt("PARAMS.A6 must be of size [28 x 20]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.A6, 560);

	par = mxGetField(PARAMS, 0, "b6");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.b6 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b6 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.b6 must be of size [28 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.b6, 28);

	par = mxGetField(PARAMS, 0, "f7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f7 must be a double.");
    }
    if( mxGetM(par) != 20 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f7 must be of size [20 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f7, 20);

	par = mxGetField(PARAMS, 0, "lb7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb7 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb7 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb7, 4);

	par = mxGetField(PARAMS, 0, "ub7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub7 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub7 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub7, 4);

	par = mxGetField(PARAMS, 0, "A7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.A7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A7 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 20 ) {
    mexErrMsgTxt("PARAMS.A7 must be of size [28 x 20]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.A7, 560);

	par = mxGetField(PARAMS, 0, "b7");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.b7 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b7 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.b7 must be of size [28 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.b7, 28);

	par = mxGetField(PARAMS, 0, "f8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f8 must be a double.");
    }
    if( mxGetM(par) != 20 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f8 must be of size [20 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f8, 20);

	par = mxGetField(PARAMS, 0, "lb8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb8 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb8 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb8, 4);

	par = mxGetField(PARAMS, 0, "ub8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub8 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub8 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub8, 4);

	par = mxGetField(PARAMS, 0, "A8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.A8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A8 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 20 ) {
    mexErrMsgTxt("PARAMS.A8 must be of size [28 x 20]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.A8, 560);

	par = mxGetField(PARAMS, 0, "b8");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.b8 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b8 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.b8 must be of size [28 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.b8, 28);

	par = mxGetField(PARAMS, 0, "f9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f9 must be a double.");
    }
    if( mxGetM(par) != 20 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f9 must be of size [20 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f9, 20);

	par = mxGetField(PARAMS, 0, "lb9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb9 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb9 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb9, 4);

	par = mxGetField(PARAMS, 0, "ub9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub9 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub9 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub9, 4);

	par = mxGetField(PARAMS, 0, "A9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.A9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A9 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 20 ) {
    mexErrMsgTxt("PARAMS.A9 must be of size [28 x 20]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.A9, 560);

	par = mxGetField(PARAMS, 0, "b9");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.b9 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b9 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.b9 must be of size [28 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.b9, 28);

	par = mxGetField(PARAMS, 0, "f10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f10 must be a double.");
    }
    if( mxGetM(par) != 20 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f10 must be of size [20 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f10, 20);

	par = mxGetField(PARAMS, 0, "lb10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb10 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb10 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb10, 4);

	par = mxGetField(PARAMS, 0, "ub10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub10 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub10 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub10, 4);

	par = mxGetField(PARAMS, 0, "A10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.A10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A10 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 20 ) {
    mexErrMsgTxt("PARAMS.A10 must be of size [28 x 20]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.A10, 560);

	par = mxGetField(PARAMS, 0, "b10");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.b10 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b10 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.b10 must be of size [28 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.b10, 28);

	par = mxGetField(PARAMS, 0, "f11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f11 must be a double.");
    }
    if( mxGetM(par) != 18 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f11 must be of size [18 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f11, 18);

	par = mxGetField(PARAMS, 0, "lb11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb11 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb11 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb11, 4);

	par = mxGetField(PARAMS, 0, "ub11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub11 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub11 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub11, 4);

	par = mxGetField(PARAMS, 0, "A11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.A11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A11 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 18 ) {
    mexErrMsgTxt("PARAMS.A11 must be of size [28 x 18]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.A11, 504);

	par = mxGetField(PARAMS, 0, "b11");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.b11 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b11 must be a double.");
    }
    if( mxGetM(par) != 28 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.b11 must be of size [28 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.b11, 28);

	par = mxGetField(PARAMS, 0, "f12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.f12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.f12 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.f12 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.f12, 4);

	par = mxGetField(PARAMS, 0, "lb12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.lb12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.lb12 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.lb12 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.lb12, 4);

	par = mxGetField(PARAMS, 0, "ub12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.ub12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.ub12 must be a double.");
    }
    if( mxGetM(par) != 4 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.ub12 must be of size [4 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.ub12, 4);

	par = mxGetField(PARAMS, 0, "A12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.A12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.A12 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 4 ) {
    mexErrMsgTxt("PARAMS.A12 must be of size [8 x 4]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.A12, 32);

	par = mxGetField(PARAMS, 0, "b12");
#ifdef MEXARGMUENTCHECKS
    if( par == NULL )	{
        mexErrMsgTxt("PARAMS.b12 not found");
    }
    if( !mxIsDouble(par) )
    {
    mexErrMsgTxt("PARAMS.b12 must be a double.");
    }
    if( mxGetM(par) != 8 || mxGetN(par) != 1 ) {
    mexErrMsgTxt("PARAMS.b12 must be of size [8 x 1]");
    }
#endif	 
    copyMArrayToC(mxGetPr(par), params.b12, 8);

	/* call solver */
	exitflag = double_integrator_QP_solver_CD_solve(&params, &output, &info);
	
	/* copy output to matlab arrays */
	plhs[0] = mxCreateStructMatrix(1, 1, 12, outputnames);
	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM( output.z1, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "z1", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM( output.z2, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "z2", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM( output.z3, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "z3", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM( output.z4, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "z4", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM( output.z5, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "z5", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM( output.z6, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "z6", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM( output.z7, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "z7", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM( output.z8, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "z8", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM( output.z9, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "z9", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM( output.z10, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "z10", outvar);

	outvar = mxCreateDoubleMatrix(7, 1, mxREAL);
	copyCArrayToM( output.z11, mxGetPr(outvar), 7);
	mxSetField(plhs[0], 0, "z11", outvar);

	outvar = mxCreateDoubleMatrix(4, 1, mxREAL);
	copyCArrayToM( output.z12, mxGetPr(outvar), 4);
	mxSetField(plhs[0], 0, "z12", outvar);	

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