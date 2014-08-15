% FORCES - Fast interior point code generation for multistage problems.
% Copyright (C) 2011-14 Alexander Domahidi [domahidi@control.ee.ethz.ch],
% Automatic Control Laboratory, ETH Zurich.
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

mex -c -O -DUSEMEXPRINTS ../src/double_integrator_QP_solver_CD.c 
mex -c -O -DMEXARGMUENTCHECKS double_integrator_QP_solver_CD_mex.c
if( ispc )
    mex double_integrator_QP_solver_CD.obj double_integrator_QP_solver_CD_mex.obj -output "double_integrator_QP_solver_CD" 
    delete('*.obj');
elseif( ismac )
    mex double_integrator_QP_solver_CD.o double_integrator_QP_solver_CD_mex.o -output "double_integrator_QP_solver_CD"
    delete('*.o');
else % we're on a linux system
    mex double_integrator_QP_solver_CD.o double_integrator_QP_solver_CD_mex.o -output "double_integrator_QP_solver_CD" -lrt
    delete('*.o');
end
copyfile(['double_integrator_QP_solver_CD.',mexext], ['../../double_integrator_QP_solver_CD.',mexext], 'f');
copyfile( 'double_integrator_QP_solver_CD.m', '../../double_integrator_QP_solver_CD.m','f');
