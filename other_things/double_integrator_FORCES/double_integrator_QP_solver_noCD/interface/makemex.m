% double_integrator_QP_solver_noCD : A fast customized optimization solver.
% 
% Copyright (C) 2014 EMBOTECH GMBH [info@embotech.com]
% 
% 
% This software is intended for simulation and testing purposes only. 
% Use of this software for any commercial purpose is prohibited.
% 
% This program is distributed in the hope that it will be useful.
% EMBOTECH makes NO WARRANTIES with respect to the use of the software 
% without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
% PARTICULAR PURPOSE. 
% 
% EMBOTECH shall not have any liability for any damage arising from the use
% of the software.
% 
% This Agreement shall exclusively be governed by and interpreted in 
% accordance with the laws of Switzerland, excluding its principles
% of conflict of laws. The Courts of Zurich-City shall have exclusive 
% jurisdiction in case of any dispute.
% 

mex -c -O -DUSEMEXPRINTS ../src/double_integrator_QP_solver_noCD.c 
mex -c -O -DMEXARGMUENTCHECKS double_integrator_QP_solver_noCD_mex.c
if( ispc )
    mex double_integrator_QP_solver_noCD.obj double_integrator_QP_solver_noCD_mex.obj -output "double_integrator_QP_solver_noCD" 
    delete('*.obj');
elseif( ismac )
    mex double_integrator_QP_solver_noCD.o double_integrator_QP_solver_noCD_mex.o -output "double_integrator_QP_solver_noCD"
    delete('*.o');
else % we're on a linux system
    mex double_integrator_QP_solver_noCD.o double_integrator_QP_solver_noCD_mex.o -output "double_integrator_QP_solver_noCD" -lrt
    delete('*.o');
end
copyfile(['double_integrator_QP_solver_noCD.',mexext], ['../../double_integrator_QP_solver_noCD.',mexext], 'f');
copyfile( 'double_integrator_QP_solver_noCD.m', '../../double_integrator_QP_solver_noCD.m','f');
