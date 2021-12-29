function output = test1s(rr)

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
setup.phase.stabilization_constraints = 2;
setup.phase.stabilization_method = 'Integration';
setup.function = @Dynamics_Multi;
setup.sigma = 0.1;

setup.bound.lower.phase.initial.position = [pi/2,pi/2,0,param.A+param.b];
setup.bound.lower.phase.initial.velocity = [0,0,-inf(1,2)];
setup.bound.lower.phase.initial.time = 0;
setup.bound.lower.phase.final.position = [0,0,-param.A-param.b,-param.A-param.b];
setup.bound.lower.phase.final.velocity = [0,0,-inf(1,2)];
setup.bound.lower.phase.final.time = 0.1;
setup.bound.lower.phase.position = [0,0,-param.A-param.b,-param.A-param.b];
setup.bound.lower.phase.velocity = -inf(1,4);
setup.bound.lower.phase.control = -inf(1,4);

setup.bound.upper.phase.initial.position = [pi/2,pi/2,0,param.A+param.b];
setup.bound.upper.phase.initial.velocity = [0,0,inf(1,2)];
setup.bound.upper.phase.initial.time = 0;
setup.bound.upper.phase.final.position = [0,0,param.A+param.b,param.A+param.b];
setup.bound.upper.phase.final.velocity = [0,0,inf(1,2)];
setup.bound.upper.phase.final.time = 10;
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
dalpha = input.phase.velocity(:,1);
dbeta = input.phase.velocity(:,2);
ialpha = input.phase.iposition(:,1);
ibeta = input.phase.iposition(:,2);
iX = input.phase.iposition(:,3);
iY = input.phase.iposition(:,4);
idalpha = input.phase.ivelocity(:,1);
idbeta = input.phase.ivelocity(:,2);
idX = input.phase.ivelocity(:,3);
idY = input.phase.ivelocity(:,4);
tau1 = input.phase.control(:,1);
tau2 = input.phase.control(:,2);
Fx2 = input.phase.control(:,3);
Fy2 = input.phase.control(:,4);
q = [alpha;beta];
dq = [dalpha;dbeta];
tau = [tau1;tau2];


param = input.auxdata;
param.flag = 3;
[Mm,k,ke,MC1] = dynamics(q,dq,tau,param);
MC = [ param.A*sin(ialpha), -param.A*cos(ialpha);
        param.b*sin(ibeta),  -param.b*cos(ibeta);
                  1,             0;
                  0,             1];
output.phase.stabilization = MC'*[idalpha;idbeta;idX;idY];
output.phase.RightHandSide = ke-k+MC1*[Fx2;Fy2];
output.phase.MassMatrix = Mm;
% output.phase.stabilization = [iX-param.A*cos(ialpha)-param.b*cos(ibeta);iY-param.A*sin(ialpha)-param.b*sin(ibeta)];
output.objective = integrate(input.phase.integrand,tau1.^2+tau2.^2);