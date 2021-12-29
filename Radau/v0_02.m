%% Radau points test: Automatic Differentiation

% Initial parameters
setup.assist.nP = 1;
setup.function = @Dynamics;
% setup.phase.dynamics.position = [1,2,3];
% setup.phase.dynamics.velocity = [4,5,6];
setup.mesh.phase.fraction = (1/20)*ones(20,1);
setup.mesh.phase.colpoints = 5*ones(20,1);
setup.mesh_points = 5;
setup.mesh_number = 20;
setup.phase.assist.n = 2;
setup.phase.assist.p = 1;
setup.phase.assist.v = 0;
setup.phase.assist.cn = 1;
setup.phase.assist.cp = 1;
setup.auxdata.M = 100;
setup.auxdata.C = 15;
setup.mesh.flag = 1;
x = rand((sum(setup.mesh.phase.colpoints)+1)*2+sum(setup.mesh.phase.colpoints)+2,1);

genpath('C:\Users\Vinicius\Google Drive\NLP_Converter\tests\dstruct');
addpath(ans);
genpath('C:\Users\vinicios\Google Drive\NLP_Converter\tests\dstruct');
addpath(ans);
genpath(cd);
addpath(ans);

%% Bounds

setup.bound.lower.phase.initial.state = [0,0];
setup.bound.lower.phase.initial.time = 0;
setup.bound.lower.phase.final.state = [10,0];
setup.bound.lower.phase.final.time = 0.1;
setup.bound.lower.phase.state = [0,0];
setup.bound.lower.phase.control = [-100,-100];
setup.bound.lower.phase.integral = 0;
% setup.bound.lower.parameter = 0;
setup.bound.lower.path = -3;

setup.bound.upper.phase.initial.state = [0,0];
setup.bound.upper.phase.initial.time = 0;
setup.bound.upper.phase.final.state = [10,0];
setup.bound.upper.phase.final.time = 10;
setup.bound.upper.phase.state = [10,inf];
setup.bound.upper.phase.control = [100,100];
setup.bound.upper.phase.integral = inf;
% setup.bound.upper.parameter = 0;
setup.bound.upper.path = 3;



%% Important functions

[D,setup,I,P] = Differentiation_matrix(setup);
[ Tm,setup ] = sparse_timeM(setup);

q = 1;
setup.mesh.phase(q).istatepoints = sparse((1:sum(setup.mesh.phase.colpoints)+1)'*ones(1,setup.phase.assist.n),ones(sum(setup.mesh.phase.colpoints)+1,1)*(1:setup.phase.assist.n),1:setup.phase.assist.n*(sum(setup.mesh.phase.colpoints)+1));

setup.mesh.phase(q).statepoints = setup.mesh.phase(q).istatepoints(1:end-1,:);

setup.mesh.phase(q).controlpoints = sparse((1:sum(setup.mesh.phase.colpoints))'*ones(1,setup.phase.assist.p),ones(sum(setup.mesh.phase.colpoints),1)*(1:setup.phase.assist.p),(1:setup.phase.assist.p*(sum(setup.mesh.phase.colpoints)))+setup.mesh.phase(q).istatepoints(end));

setup.mesh.phase(1).initialtimepoint = setup.mesh.phase(1).controlpoints(end)+1;
setup.mesh.phase(1).finaltimepoint = setup.mesh.phase(1).controlpoints(end)+2;
setup.mesh.parameterpoint = (setup.mesh.phase(1).finaltimepoint+1:setup.mesh.phase(1).finaltimepoint+setup.phase(1).assist.v)';
% setup.mesh.phase(1).integralpoint = (setup.mesh.phase(1).finaltimepoint+setup.phase(1).assist.v+1:setup.mesh.phase(1).finaltimepoint+setup.phase(1).assist.v+setup.phase(1).assist.cn)';
global x_old dx_old
x_old = rand(numel(x),1);
dx_old = rand(numel(x),1);

tic
%% Constraints_RPM

constraints = Constraints_RPM(x,setup);

%% Objective_RPM

objective = Objective_RPM(x,setup);

%% Jacobian_RPM

Jfinal = Jacobian_RPM(x,setup);

%% Gradient_RPM

gradient = Gradient_RPM(x,setup);

toc
% return
%% IPOPT

  x0         = x;  % The starting point.
  options.lb = [zeros(100,1);10;0;zeros(99,1);0;-100*ones(numel(setup.mesh.phase(1).controlpoints),1);0;0.1];  % Lower bound on the variables.
  options.ub = [0;10*ones(99,1);10;0;10*ones(99,1);0;100*ones(numel(setup.mesh.phase(1).controlpoints),1);0;10];  % Upper bound on the variables.
  options.cl = [zeros(100*setup.phase.assist.n,1);0;-300*ones(100,1)];  % Lower bounds on the constraint functions.
  options.cu = [zeros(100*setup.phase.assist.n,1);inf;300*ones(100,1)];   % Upper bounds on the constraint functions.

    % Set the IPOPT options.
%     options.ipopt.jacobian_approximation = 'finite-difference-values';
global Hessstr

if setup.mesh.flag~=1
    options.ipopt.hessian_approximation  = 'limited-memory';
    Hessstr = [];
else
      Hessstr = Hessian_RPM(x,rand(1,1),rand(301,1),setup)~=0;
end

%   options.ipopt.hessian_approximation = 'limited-memory';
  options.ipopt.max_iter               = 2000;
  options.ipopt.mu_strategy           = 'adaptive';
  options.ipopt.tol                   = 1e-7;
  options.ipopt.derivative_test       = 'second-order';
  asd = 10;
  Jacstr = Jacobian_RPM(x,setup);

  
  global cont
  cont = 0;
  
  % The callback functions.
  funcs.objective         = @(x) Objective_RPM(x,setup);
  funcs.constraints       = @(x) Constraints_RPM(x,setup);
  funcs.gradient          = @(x) Gradient_RPM(x,setup);
  funcs.jacobian          = @(x) Jacobian_RPM(x,setup);
  funcs.jacobianstructure = @() Jacstr;
  funcs.hessian           = @(x,sigma,lambda) Hessian_RPM(x,sigma,lambda,setup);
  funcs.hessianstructure  = @() Hessian_test();
  
  % Run IPOPT.
  [xg,info] = ipopt(x0,funcs,options);