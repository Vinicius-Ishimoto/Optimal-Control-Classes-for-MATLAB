function [ dynamics,setup ] = function_Mrunner(x,setup)

global x_old dynamics_old

if any(x_old~=x)
    for q=1:setup.assist.nP
    C = setup.mesh.phase(q).colpoints;
    V = setup.mesh.phase(q).fraction;
    ci = [];
    for i=1:numel(C)
        ci = [ci;i*ones(C(i),1)];
    end
    cV1 = cumsum(V);
    cV2 = [0;cV1(1:end-1)];
    solve.phase(q).time = vect(diag(setup.phase(q).solver.Tm).*(x(setup.mesh.phase(q).finaltimepoint)-x(setup.mesh.phase(q).initialtimepoint)).*setup.mesh.phase(q).vtau+0.5.*((cV1(ci)+cV2(ci)).*(x(setup.mesh.phase(q).finaltimepoint)-x(setup.mesh.phase(q).initialtimepoint))+2*x(setup.mesh.phase(q).initialtimepoint)));
    solve.phase(q).position = vect(x(setup.mesh.phase(q).positionpoints));
    solve.phase(q).velocity = vect(x(setup.mesh.phase(q).velocitypoints));
    solve.phase(q).iposition = vect(x(setup.mesh.phase(q).ipositionpoints));
    solve.phase(q).ivelocity = vect(x(setup.mesh.phase(q).ivelocitypoints));
    if setup.phase(q).assist.w ~= 0
    solve.phase(q).state = vect(x(setup.mesh.phase(q).statepoints));
    solve.phase(q).initial.state = x(setup.mesh.phase(q).istatepoints(1,:))';
    solve.phase(q).final.state = x(setup.mesh.phase(q).istatepoints(end,:))';
    end
    solve.phase(q).control = vect(x(setup.mesh.phase(q).controlpoints));
    solve.phase(q).initial.time = x(setup.mesh.phase(q).initialtimepoint);
    solve.phase(q).initial.position = x(setup.mesh.phase(q).ipositionpoints(1,:))';
    solve.phase(q).initial.velocity = x(setup.mesh.phase(q).ivelocitypoints(1,:))';
    solve.phase(q).final.time = x(setup.mesh.phase(q).finaltimepoint);
    solve.phase(q).final.position = x(setup.mesh.phase(q).ipositionpoints(end,:))';
    solve.phase(q).final.velocity = x(setup.mesh.phase(q).ivelocitypoints(end,:))';
    solve.phase(q).integrand.initialtime = solve.phase(q).initial.time;
    solve.phase(q).integrand.finaltime = solve.phase(q).final.time;
    solve.phase(q).integrand.locationtime = [setup.mesh.phase(q).initialtimepoint,setup.mesh.phase(q).finaltimepoint];
    solve.phase(q).integrand.weight_vector = setup.phase(q).solver.weight_vector;
    solve.phase(q).parameter = vect(ones(sum(setup.mesh.phase(q).colpoints),1)*reshape(x(setup.mesh.parameterpoint),1,[]));
    solve.phase(q).iparameter = vect(ones(sum(setup.mesh.phase(q).colpoints)+1,1)*reshape(x(setup.mesh.parameterpoint),1,[]));
    if isfield(setup,'constant_vector')
        solve.phase(q).constant_vector = vect(setup.constant_vector.phase(q).variables);
    end
    end
    
    if isfield(setup,'auxdata')
        solve.auxdata = setup.auxdata;
    end
    if isfield(setup,'vector_constants')
        
    end
    Dynamics = setup.function;
    dynamics = Dynamics(solve);
%     dynamics.phase(q).MassMatrix = col(dynamics.phase(q).MassMatrix);
%     dynamics.phase(q).RightHandSide = col(dynamics.phase(q).RightHandSide);
    x_old = x;
    dynamics_old = dynamics;
else
    dynamics = dynamics_old;
end

end