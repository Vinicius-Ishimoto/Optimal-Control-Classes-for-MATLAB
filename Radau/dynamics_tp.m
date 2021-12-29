function output = dynamics_tp(input)

%% Continuous
dx1 = input.phase(1).state(:,2);
u1 = input.phase(1).control;
dx2 = input.phase(2).state(:,2);
u2 = input.phase(2).control;

%% Points
K1 = input.phase(1).parameter;
K2 = input.phase(2).parameter;

%% Auxdata
M = input.auxdata.M;
C = input.auxdata.C;
Fr = input.auxdata.Fr;

ddx1 = u1.*(1+K1)/M-dx1.*C/M-Fr.*tanh(1000*dx1)/M;
ddx2 = u2.*(1+K2)/M-dx2.*C/M-Fr.*tanh(1000*dx2)/M;

%% Out
output.phase(1).derivatives = [dx1,u1.*(1+K1)/M-dx1.*C/M-Fr.*tanh(1000*dx1)/M];
output.phase(2).derivatives = [dx2,u2.*(1+K2)/M-dx2.*C/M-Fr.*tanh(1000*dx2)/M];
output.phase(1).path = ddx1;
output.phase(2).path = ddx2;
output.objective = integrate(input.phase(1).integrand,u1.^2)+integrate(input.phase(2).integrand,u2.^2);
output.constraints = integrate(input.phase(1).integrand,K1.*u1.*dx1)+integrate(input.phase(2).integrand,(K2./(1+K2)).*(-u2.*(1+K2)+2*C*dx2+2.*Fr.*tanh(1000*dx2)).*dx2);
output.constraints = [output.constraints,input.phase(2).initial.state-input.phase(1).final.state,input.phase(2).initial.time-input.phase(1).final.time];

end