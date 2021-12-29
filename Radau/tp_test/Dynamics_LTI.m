function output = Dynamics_LTI(input)

%% Continuous
x1 = input.phase.state(:,1);
x2 = input.phase.state(:,2);
x3 = input.phase.state(:,3);
x4 = input.phase.state(:,4);
u1 = input.phase.control(:,1);
u2 = input.phase.control(:,2);
% u2 = input.phase.control(:,2);
% t = input.phase.time;

%% Points
% p = input.parameter;
xi = input.phase.initial.state;
% ti = input.phase.initial.time;
xf = input.phase.final.state;
tf = input.phase.final.time;

%% Auxdata
% M = input.auxdata.M;
% C = input.auxdata.C;

%% Out
output.phase.derivatives = [-x1/10+0.5*u1,-x2/15+0.5*u1,-x3/15+0.5*u2,-x4/10+0.5*u2];
% output.phase.constraints = integrate(input.phase.integrand,u.^2);
% output.path = u/M-dx.*C/M;
output.constraints = [[3/5,0,8/15,0]*xi',[0,2/3,0,3/5]*xi',[3/5,0,8/15,0]*xf',[0,2/3,0,3/5]*xf'];
output.phase.path = col([[3/5,0,8/15,0]*[x1;x2;x3;x4],[0,2/3,0,3/5]*[x1;x2;x3;x4]]);
output.objective = tf;

end
