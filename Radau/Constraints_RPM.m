function constraints = Constraints_RPM(x,setup)

dynamics = function_runner(x,setup);
global exp1
% Defects
constraints = [];
for q=1:setup.assist.nP
D = setup.phase(q).solver.D;
Tm = setup.phase(q).solver.Tm;
tf = x(setup.mesh.phase(q).finaltimepoint);
t0 = x(setup.mesh.phase(q).initialtimepoint);
constraints = [constraints;reshape(D*x(setup.mesh.phase(q).istatepoints)-(tf-t0)*Tm*double(dynamics.phase(q).derivatives),[],1)];
exp1 = double(dynamics.phase(q).derivatives);
% Path constraints
if isfield(dynamics.phase(q),'path')
    constraints = [constraints; ...
        reshape(double(dynamics.phase(q).path),[],1)];
end
end
% point constraints
if isfield(dynamics,'constraints')
    constraints = [constraints; ...
        reshape(double(dynamics.constraints),[],1)];
end



end