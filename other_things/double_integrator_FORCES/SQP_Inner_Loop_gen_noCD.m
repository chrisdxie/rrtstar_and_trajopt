function SQP_Inner_Loop_gen_noCD()

% FORCES - Fast interior point code generation for multistage problems.
% Copyright (C) 2011-12 Alexander Domahidi [domahidi@control.ee.ethz.ch],
% Automatic Control Laboratory, ETH Zurich.

% State z_t = [x_t u_t x_{t+1} u_{t+1} delta_t GX_t HX_t], t = 1, ..., N-2
% GX_t and HX_t are auxiliary variables to minimize hinge loss and absolute
% value of nonconvex constraints (collisions and dynamics)
% Special values:
%   z_{N-1} = [x_{N-1} u_{N-1} x_N delta_{N-1} GX_{N-1} HX_{N-1}]
%   z_N = [x_N GX_N]

% Automated addpath stuff
currDir = pwd;
disp('currDir');
disp(currDir);
forcesDir = strcat(currDir,'/FORCES');
addpath(forcesDir);
disp(strcat(currDir,'/FORCES'));

% Problem setup
timesteps = 12;
N = timesteps;
d = 2; % 2-dimensional
nX = 2*d;
nU = d;
nO = 3; % Number of obstacles
stages = MultistageProblem(N);

% First stage
i = 1;
i_str = sprintf('%d', i);

% Dimensions of first stage
stages(i).dims.n = 2*nX+2*nU+1+nX;                          % number of stage variables
stages(i).dims.l = nX+nU+1;                                 % number of lower bounds
stages(i).dims.u = nX+nU;                                   % number of upper bounds
stages(i).dims.r = nX+nU+1;                                 % number of equality constraints
stages(i).dims.p = 4*nX+2*nU+2;                             % number of affine constraints
stages(i).dims.q = 0;                                       % number of quadratic constraints

% Cost of the first stage
stages(i).cost.H = zeros(stages(i).dims.n);                 % No quadratic cost
params(1) = newParam(['f' i_str], i, 'cost.f');             % Parameter for cost

% Lower Bounds
stages(i).ineq.b.lbidx = [1:nX+nU 2*nX+2*nU+1];             % Lower bounds on states, controls, and time
params(end+1) = newParam(['lb' i_str], i, 'ineq.b.lb');

% Upper bounds
stages(i).ineq.b.ubidx = 1:stages(i).dims.u;                % Upper bounds on states, controls, NOT time
params(end+1) = newParam(['ub' i_str], i, 'ineq.b.ub');

% Equality between z1 and z2 is time (delta), and next state. 
stages(i).eq.C = [zeros(nX+nU+1, nX+nU) eye(nX+nU+1) zeros(nX+nU+1, nX)];
stages(i).eq.c = zeros(stages(i).dims.r, 1);

% Inequality constraints: These include linearized dynamics and linearized
% collisions
params(end+1) = newParam(['A' i_str], i, 'ineq.p.A');
params(end+1) = newParam(['b' i_str], i, 'ineq.p.b');

% Intermediate stages
for i=2:N-2
    
    i_str = sprintf('%d', i);

    % Dimensions
    stages(i).dims.n = 2*nX+2*nU+1+nX;                          % number of stage variables
    stages(i).dims.l = nX+nU;                                   % number of lower bounds
    stages(i).dims.u = nX+nU;                                   % number of upper bounds
    stages(i).dims.r = nX+nU+1;                                 % number of equality constraints
    stages(i).dims.p = 4*nX+2*nU+2;                             % number of affine constraints
    stages(i).dims.q = 0;                                       % number of quadratic constraints
    
    % Cost
    stages(i).cost.H = zeros(stages(i).dims.n);                 % No quadratic cost
    params(end+1) = newParam(['f' i_str], i, 'cost.f');         % Parameter for cost
    
    % Lower Bounds
    stages(i).ineq.b.lbidx = 1:stages(i).dims.l;                % Lower bounds on states, controls, and time
    params(end+1) = newParam(['lb' i_str], i, 'ineq.b.lb');
    
    % Upper bounds
    stages(i).ineq.b.ubidx = 1:stages(i).dims.u;                % Upper bounds on states, controls, NOT time
    params(end+1) = newParam(['ub' i_str], i, 'ineq.b.ub');
    
    % Equality between z1 and z2 is time (delta), and next state. 
    stages(i).eq.C = [zeros(nX+nU+1, nX+nU) eye(nX+nU+1) zeros(nX+nU+1, nX)];
    stages(i).eq.c = zeros(stages(i).dims.r, 1);

    stages(i).eq.D = [-1*eye(nX+nU) zeros(nX+nU, nX+nU+1+nX); zeros(1, stages(i).dims.n)];
    stages(i).eq.D(nX+nU+1, 2*nX+2*nU+1) = -1;
    
    % Inequality constraints: These include linearized dynamics and linearized
    % collisions
    params(end+1) = newParam(['A' i_str], i, 'ineq.p.A');
    params(end+1) = newParam(['b' i_str], i, 'ineq.p.b');
    
end


% Stage N-1
i = N-1;
i_str = sprintf('%d', i);

% Dimensions
stages(i).dims.n = 2*nX+nU+1+nX;                         % number of stage variables
stages(i).dims.l = nX+nU;                                   % number of lower bounds
stages(i).dims.u = nX+nU;                                   % number of upper bounds
stages(i).dims.r = nX;                                      % number of equality constraints
stages(i).dims.p = 4*nX+2*nU+2;                        % number of affine constraints
stages(i).dims.q = 0;                                       % number of quadratic constraints

% Cost
stages(i).cost.H = zeros(stages(i).dims.n);                 % No quadratic cost
params(end+1) = newParam(['f' i_str], i, 'cost.f');         % Parameter for cost

% Lower Bounds
stages(i).ineq.b.lbidx = 1:stages(i).dims.l;                % Lower bounds on states, controls, and time
params(end+1) = newParam(['lb' i_str], i, 'ineq.b.lb');

% Upper bounds
stages(i).ineq.b.ubidx = 1:stages(i).dims.u;                % Upper bounds on states, controls, NOT time
params(end+1) = newParam(['ub' i_str], i, 'ineq.b.ub');

% Equality between z1 and z2 is time (delta), and next state.
stages(i).eq.C = [zeros(nX, nX+nU) eye(nX) zeros(nX, 1+nX)];
stages(i).eq.c = zeros(stages(i).dims.r, 1);

stages(i).eq.D = [-1*eye(nX+nU) zeros(nX+nU, nX+1+nX); zeros(1, stages(i).dims.n)];
stages(i).eq.D(nX+nU+1, 2*nX+nU+1) = -1;

% Inequality constraints: These include linearized dynamics and linearized
% collisions
params(end+1) = newParam(['A' i_str], i, 'ineq.p.A');
params(end+1) = newParam(['b' i_str], i, 'ineq.p.b');


% Final stage
i = N;
i_str = sprintf('%d', i);

% Dimensions of last stage
stages(i).dims.n = nX;                                   % number of stage variables
stages(i).dims.l = nX;                                      % number of lower bounds
stages(i).dims.u = nX;                                      % number of upper bounds
stages(i).dims.r = 0;                                       % number of equality constraints
stages(i).dims.p = 2*nX;                               % number of affine constraints
stages(i).dims.q = 0;                                       % number of quadratic constraints

% Cost of last stage
stages(i).cost.H = zeros(nX+1);
params(end+1) = newParam(['f' i_str], i, 'cost.f');         % This cost is for the penalty coefficient

% Lower bounds
stages(i).ineq.b.lbidx = 1:stages(i).dims.l;                % Lower bounds on states
params(end+1) = newParam(['lb' i_str], i, 'ineq.b.lb');

% Upper bounds
stages(i).ineq.b.ubidx = 1:stages(i).dims.u;                % Upper bounds on states
params(end+1) = newParam(['ub' i_str], i, 'ineq.b.ub');

stages(i).eq.D = -1*eye(nX);

% Inequality constraints: These include linearized dynamics and linearized
% collisions
params(end+1) = newParam(['A' i_str], i, 'ineq.p.A');
params(end+1) = newParam(['b' i_str], i, 'ineq.p.b');

% ------------ OUTPUTS ---------------------

% Define outputs of solver
for i=1:N-2
    var = sprintf('z%d', i);
    %outputs(i) = newOutput(var, i, [1:nX+nU 2*nX+2*nU+1:2*nX+2*nU+1+nO+nX]);
    outputs(i) = newOutput(var, i, [1:nX+nU 2*nX+2*nU+1]);
end
i = N-1;
var = sprintf('z%d', i);
%outputs(i) = newOutput(var, i, [1:nX+nU 2*nX+nU+1:2*nX+nU+1+nO+nX]);
outputs(i) = newOutput(var, i, [1:nX+nU 2*nX+nU+1]);
i = N;
var = sprintf('z%d', i);
%outputs(i) = newOutput(var, i, 1:(nX+nO));
outputs(i) = newOutput(var, i, 1:nX);

% ------------- DONE -----------------------

solver_name = 'double_integrator_QP_solver_noCD';
codeoptions = getOptions(solver_name);
codeoptions.printlevel = 0; % Debugging info for now
codeoptions.timing=1;       % Debugging, just to see how long it takes
codeoptions.maxit=100;

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