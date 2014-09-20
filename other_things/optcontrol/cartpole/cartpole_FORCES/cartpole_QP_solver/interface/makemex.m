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

mex -c -O -DUSEMEXPRINTS ../src/cartpole_QP_solver.c 
mex -c -O -DMEXARGMUENTCHECKS cartpole_QP_solver_mex.c
if( ispc )
    mex cartpole_QP_solver.obj cartpole_QP_solver_mex.obj -output "cartpole_QP_solver" 
    delete('*.obj');
elseif( ismac )
    mex cartpole_QP_solver.o cartpole_QP_solver_mex.o -output "cartpole_QP_solver"
    delete('*.o');
else % we're on a linux system
    mex cartpole_QP_solver.o cartpole_QP_solver_mex.o -output "cartpole_QP_solver" -lrt
    delete('*.o');
end
copyfile(['cartpole_QP_solver.',mexext], ['../../cartpole_QP_solver.',mexext], 'f');
copyfile( 'cartpole_QP_solver.m', '../../cartpole_QP_solver.m','f');
