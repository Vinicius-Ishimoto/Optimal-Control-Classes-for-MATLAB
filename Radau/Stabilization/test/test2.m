function output = test2(rr)

setup.mesh_points = 5;
setup.mesh_number = 20;
setup.auxdata.M = 100;
setup.auxdata.C = 15;
param.ma = 1.5;
param.mb = 1;
param.Ja = 0.6;
param.Jb = 0.4;
param.A  = 2;
param.B  = 1.5;
param.a  = 0.8;
param.b  = 0.6;
setup.auxdata = param;
% setup.phase.stabilization_constraints
setup.function = @Dynamics_Multi;

setup.bound.lower.phase.initial.position = [pi/2,pi/2,0,param.A+param.b];
setup.bound.lower.phase.initial.velocity = [0,0,-inf(1,2)];
setup.bound.lower.phase.initial.time = 0;
setup.bound.lower.phase.final.position = [0,0,param.A+param.b,0];
setup.bound.lower.phase.final.velocity = [0,0,-inf(1,2)];
setup.bound.lower.phase.final.time = 0.1;
setup.bound.lower.phase.path = [0,0];
setup.bound.lower.phase.position = [0,0,-param.A-param.b,-param.A-param.b];
setup.bound.lower.phase.velocity = -inf(1,4);
setup.bound.lower.phase.control = -inf(1,4);

setup.bound.upper.phase.initial.position = [pi/2,pi/2,0,param.A+param.b];
setup.bound.upper.phase.initial.velocity = [0,0,inf(1,2)];
setup.bound.upper.phase.initial.time = 0;
setup.bound.upper.phase.final.position = [0,0,param.A+param.b,0];
setup.bound.upper.phase.final.velocity = [0,0,inf(1,2)];
setup.bound.upper.phase.final.time = 10;
setup.bound.upper.phase.path = [0,0];
setup.bound.upper.phase.position = [pi,pi,param.A+param.b,param.A+param.b];
setup.bound.upper.phase.velocity = inf(1,4);
setup.bound.upper.phase.control = inf(1,4);

%% Guess
if nargin==0
setup.initial_guess.phase.time = linspace(0,10,100)';
setup.initial_guess.phase.position = [zeros(100,1),pi/4*ones(100,1),zeros(100,2)];
setup.initial_guess.phase.velocity = zeros(100,4);
setup.initial_guess.phase.control = [zeros(100,1),zeros(100,1),zeros(100,2)];
else
setup.initial_guess.phase.time = rr.solution.phase.time;
setup.initial_guess.phase.position = rr.solution.phase.position;
setup.initial_guess.phase.velocity = rr.solution.phase.velocity;
setup.initial_guess.phase.control = rr.solution.phase.control;
end

output = ddiopt_MB(setup);

function output = Dynamics_Multi(input)

alpha = input.phase.position(:,1);
beta = input.phase.position(:,2);
X = input.phase.position(:,3);
Y = input.phase.position(:,4);
dalpha = input.phase.velocity(:,1);
dbeta = input.phase.velocity(:,2);
tau1 = input.phase.control(:,1);
tau2 = input.phase.control(:,2);
Fx2 = input.phase.control(:,3);
Fy2 = input.phase.control(:,4);
q = [alpha;beta];
dq = [dalpha;dbeta];
tau = [tau1;tau2];

param = input.auxdata;
param.flag = 3;
[Mm,k,ke,MC] = dynamics(q,dq,tau,param);

output.phase.RightHandSide = ke-k+MC*[Fx2;Fy2];
output.phase.MassMatrix = Mm;
output.objective = integrate(input.phase.integrand,tau1.^2+tau2.^2);
output.phase.path = [X-param.A*cos(alpha)-param.b*cos(beta),Y-param.A*sin(alpha)-param.b*sin(beta)];