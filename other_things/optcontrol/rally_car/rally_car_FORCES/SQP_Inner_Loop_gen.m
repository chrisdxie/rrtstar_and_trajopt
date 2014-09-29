function SQP_Inner_Loop_gen()

% FORCES - Fast interior point code generation for multistage problems.
% Copyright (C) 2011-12 Alexander Domahidi [domahidi@control.ee.ethz.ch],
% Automatic Control Laboratory, ETH Zurich.

% Automated addpath stuff
currDir = pwd;
disp('currDir');
disp(currDir);
forcesDir = strcat(currDir,'/FORCES');
addpath(forcesDir);
disp(strcat(currDir,'/FORCES'));

% Problem setup
timesteps = 12;
N = timesteps-1;
nX = 8;
nU = 3;
stages = MultistageProblem(N+1);

% First stage
i = 1;
i_str = sprintf('%d', i);

% Dimensions of first stage
stages(i).dims.n = 3*nX+nU+1;                          % number of stage variables
stages(i).dims.l = 3*nX+nU+1;                                 % number of lower bounds
stages(i).dims.u = nX+nU+1;                                   % number of upper bounds
stages(i).dims.r = 2*nX+1;                                 % number of equality constraints
stages(i).dims.p = 0;                             % number of affine constraints
stages(i).dims.q = 0;                                       % number of quadratic constraints

% Cost of the first stage
stages(i).cost.H = zeros(stages(i).dims.n);                 % No quadratic cost
params(1) = newParam(['f' i_str], i, 'cost.f');             % Parameter for cost

% Lower Bounds
stages(i).ineq.b.lbidx = 1:stages(i).dims.l;             % Lower bounds on states, controls, and time
params(end+1) = newParam(['lb' i_str], i, 'ineq.b.lb');

% Upper bounds
stages(i).ineq.b.ubidx = 1:stages(i).dims.u;                % Upper bounds on states, controls, NOT time
params(end+1) = newParam(['ub' i_str], i, 'ineq.b.ub');

params(end+1) = newParam(['C',i_str], i, 'eq.C');
params(end+1) = newParam(['e',i_str], i, 'eq.c');

% Intermediate stages
for i=2:N
    
    i_str = sprintf('%d', i);

    % Dimensions
    stages(i).dims.n = 3*nX+nU+1;                          % number of stage variables
    stages(i).dims.l = 3*nX+nU+1;                                    % number of lower bounds
    stages(i).dims.u = nX+nU+1;                                    % number of upper bounds
    stages(i).dims.r = nX+1;                                 % number of equality constraints
    stages(i).dims.p = 0;                             % number of affine constraints
    stages(i).dims.q = 0;                                       % number of quadratic constraints
    
    % Cost
    stages(i).cost.H = zeros(stages(i).dims.n);                 % No quadratic cost
    params(end+1) = newParam(['f' i_str], i, 'cost.f');         % Parameter for cost
    
    % Lower Bounds
    stages(i).ineq.b.lbidx = 1:stages(i).dims.l;                % Lower bounds on velocities, controls, and time
    params(end+1) = newParam(['lb' i_str], i, 'ineq.b.lb');
    
    % Upper bounds
    stages(i).ineq.b.ubidx = 1:stages(i).dims.u;                % Upper bounds on velocities, controls, NOT time
    params(end+1) = newParam(['ub' i_str], i, 'ineq.b.ub');
     
    params(end+1) = newParam(['C',i_str], i, 'eq.C');
    params(end+1) = newParam(['e',i_str], i, 'eq.c');

    if( i==2 )
        stages(i).eq.D =  [zeros(nX,3*nX+nU+1); -eye(nX+1), zeros(nX+1,2*nX+nU)];
    else
        stages(i).eq.D = [-eye(nX+1), zeros(nX+1,2*nX+nU)];
    end   
end


% Stage N-1
i = N+1;
i_str = sprintf('%d', i);

% Dimensions
stages(i).dims.n = nX+1;                         % number of stage variables
stages(i).dims.l = nX+1;                                   % number of lower bounds
stages(i).dims.u = nX+1;                                   % number of upper bounds
stages(i).dims.r = 0;                                      % number of equality constraints
stages(i).dims.p = 0;                        % number of affine constraints
stages(i).dims.q = 0;                                       % number of quadratic constraints

% cost
stages(i).cost.H = zeros(stages(i).dims.n);
stages(i).cost.f = zeros(stages(i).dims.n,1);

% lower bounds
stages(i).ineq.b.lbidx = 1:stages(i).dims.l; % lower bound acts on these indices
params(end+1) = newParam(['lb',i_str], i, 'ineq.b.lb');

% upper bounds
stages(i).ineq.b.ubidx = 1:stages(i).dims.u; % upper bound acts on these indices
params(end+1) = newParam(['ub',i_str], i, 'ineq.b.ub');

% equality constraints
stages(i).eq.D = -eye(nX+1);

% ------------ OUTPUTS ---------------------

% define outputs of the solver
for i=1:N
    var = sprintf('z%d',i);
    outputs(i) = newOutput(var,i,1:nX+nU+1);
end
i=N+1;
var = sprintf('z%d',i);
outputs(i) = newOutput(var,i,1:nX+1);

% ------------- DONE -----------------------

solver_name = 'rally_car_QP_solver';
codeoptions = getOptions(solver_name);
codeoptions.printlevel = 0; % Debugging info for now
codeoptions.timing=1;       % Debugging, just to see how long it takes
codeoptions.maxit=50;
codeoptions.overwrite=1;
codeoptions.optlevel=1;

% generate code
generateCode(stages,params,codeoptions,outputs);

% Automated unzipping stuff
disp('Unzipping into solver directory...');
outdir = 'solver/';
system(['mkdir -p ',outdir]);
header_file = [solver_name,num2str(timesteps),'.h'];
src_file = [solver_name,num2str(timesteps),'.c'];
system(['unzip -p ',solver_name,'.zip include/',solver_name,'.h > ',outdir,header_file]);
system(['unzip -p ',solver_name,'.zip src/',solver_name,'.c > ',outdir,src_file]);
system(['rm -rf ',solver_name,'.zip @CodeGen']);

disp('Replacing incorrect #include in .c file...');
str_to_delete = ['#include "../include/',solver_name,'.h"'];
str_to_insert = ['#include "',solver_name,'.h"'];
solver_src = fileread([outdir,src_file]);
solver_src_new = strrep(solver_src,str_to_delete,str_to_insert);

fid = fopen([outdir,src_file],'w');
fwrite(fid,solver_src_new);
fclose(fid);


end
