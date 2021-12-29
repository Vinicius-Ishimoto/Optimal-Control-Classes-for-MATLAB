function output = inv_test1(r)

param.ma = 1.5;
param.mb = 1;
param.Ja = 0.6;
param.Jb = 0.4;
param.A  = 2;
param.B  = 1.5;
param.a  = 0.8;
param.b  = 0.6;
setup.auxdata = param;
setup.mesh_points = 5;
setup.mesh_number = 20;
% setup.auxdata.M = 100;
% setup.auxdata.C = 15;
setup.function = @Dynamics_case;

genpath('C:\Users\Vinicius\Google Drive\NLP_Converter\tests\Radau');
addpath(ans);
genpath('C:\Users\vinicios\Google Drive\NLP_Converter\tests\Radau');
addpath(ans);
genpath('C:\Users\Vinicius\Google Drive\MATLAB_Classes');
addpath(ans);
genpath('C:\Users\vinicios\Google Drive\MATLAB_Classes');
addpath(ans);

%% Bounds

setup.bound.lower.phase.initial.state = [pi/2,pi/2,0,0];
setup.bound.lower.phase.initial.time = 0;
setup.bound.lower.phase.final.state = [0,0,0,0];
setup.bound.lower.phase.final.time = 10;
setup.bound.lower.phase.state = [0,0,-inf,-inf];
setup.bound.lower.phase.control = [-inf,-inf];
% setup.bound.lower.phase.path = -3;
% setup.bound.lower.pconstraints = 0;
% setup.bound.lower.parameter = 0;


setup.bound.upper.phase.initial.state = [pi/2,pi/2,0,0];
setup.bound.upper.phase.initial.time = 0;
setup.bound.upper.phase.final.state = [0,0,0,0];
setup.bound.upper.phase.final.time = 10;
setup.bound.upper.phase.state = [pi,pi,inf,inf];
setup.bound.upper.phase.control = [inf,inf];
% setup.bound.upper.phase.path = 3;
% setup.bound.upper.pconstraints = inf;
% setup.bound.upper.parameter = 0;


%% Guess
if nargin==0
setup.initial_guess.phase.state = [zeros(100,1),pi/4*ones(100,1),zeros(100,2)];
setup.initial_guess.phase.control = zeros(100,2);
setup.initial_guess.phase.time = linspace(0,10,100);
else
setup.initial_guess.phase.state = r.solution.phase.state;
setup.initial_guess.phase.control = r.solution.phase.control;
setup.initial_guess.phase.time = r.solution.phase.time;
end

output = ddiopt(setup);

function output = Dynamics_case(input)

alpha = input.phase.state(:,1);
beta = input.phase.state(:,2);
dalpha = input.phase.state(:,3);
dbeta = input.phase.state(:,4);
tau1 = input.phase.control(:,1);
tau2 = input.phase.control(:,2);
q = [alpha;beta];
dq = [dalpha;dbeta];
tau = [tau1;tau2];

param = input.auxdata;
param.flag = 1;
[Mm,k,ke] = dynamics(q,dq,tau,param);
% xdot = [dalpha,dbeta];
dxdot = Mm\(ke-k);
output.phase.derivatives = [dalpha,dbeta,dxdot'];
output.objective = integrate(input.phase.integrand,tau1.^2+tau2.^2);

