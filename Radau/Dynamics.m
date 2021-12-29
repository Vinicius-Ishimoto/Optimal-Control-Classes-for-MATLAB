function output = Dynamics(input)

%% Continuous
dx = input.phase.state(:,2);
u = input.phase.control;
% t = input.phase.time;

%% Points
% p = input.parameter;
% xi = input.phase.initial.state;
% ti = input.phase.initial.time;
% xf = input.phase.final.state;
% tf = input.phase.final.time;

%% Auxdata
M = input.auxdata.M;
C = input.auxdata.C;

%% Out
output.phase.derivatives = [dx, u/M-dx*C/M];
% output.phase.constraints = integrate(input.phase.integrand,u.^2);
% output.path = u/M-dx.*C/M;
output.constraints = integrate(input.phase.integrand,u.^2);
output.phase.path = u/M-dx.*C/M;
output.objective = integrate(input.phase.integrand,u.^2);

end
