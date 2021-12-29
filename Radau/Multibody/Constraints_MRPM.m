function constraints = Constraints_MRPM(x,setup)

dynamics = function_Mrunner(x,setup);
global exp2
% Defects
constraints = [];
for q=1:setup.assist.nP
D = setup.phase(q).solver.D;
Tm = setup.phase(q).solver.Tm;
tf = vect(x(setup.mesh.phase(q).finaltimepoint)*diag(Tm));
t0 = vect(x(setup.mesh.phase(q).initialtimepoint)*diag(Tm));
dV = vect(D*x(setup.mesh.phase(q).ivelocitypoints))';
constraints = [constraints;reshape(D*x(setup.mesh.phase(q).ipositionpoints)-double(tf-t0,'matrix')*x(setup.mesh.phase(q).velocitypoints),[],1)];
constraints = [constraints;double(dynamics.phase(q).MassMatrix*dV-(tf-t0)*dynamics.phase(q).RightHandSide)];
if isfield(dynamics.phase(q),'derivatives')
    constraints = [constraints;reshape(D*x(setup.mesh.phase(q).istatepoints)-double((tf-t0)*dynamics.phase(q).derivatives),[],1)];
end
exp2 = x(setup.mesh.phase(q).velocitypoints);
% Path constraints
if isfield(dynamics.phase(q),'path')
    constraints = [constraints; ...
        reshape(double(dynamics.phase(q).path),[],1)];
end
% Stabilization constraints
if isfield(dynamics.phase(q),'stabilization')
    if not(isempty(dynamics.phase(q).stabilization))
    if strcmp(setup.phase(q).stabilization_method,'Integration')
    M1 = dot(setup.phase(q).solver.P,dynamics.phase(q).stabilization);
    M2 = setup.sigma*dot(setup.phase(q).solver.I,(tf-t0)*col(dynamics.phase(q).stabilization));
    constraints = [constraints; ...
    reshape(double(M1+M2),[],1); reshape(double(col(dynamics.phase(q).stabilization,1)),[],1)];
    else
    M1 = dot(setup.phase(q).solver.D,dynamics.phase(q).stabilization);
    M2 = setup.sigma*(tf-t0)*col(dynamics.phase(q).stabilization);
    constraints = [constraints; ...
    reshape(double(M1+M2),[],1)];
    end
    end
%     constraints = [constraints; ...
%     reshape(double(M1+M2),[],1); reshape(double(col(dynamics.phase(q).stabilization,1)),[],1)];
end



end

% point constraints
if isfield(dynamics,'constraints')
    constraints = [constraints; ...
        reshape(double(dynamics.constraints),[],1)];
end

end