function [ dynamics,setup ] = hessian_runner(x,setup)

global dx_old derivatives_old cont

if any(dx_old~=x)
    for q=1:setup.assist.nP
    solve.phase(q).state = hderiv(x(setup.mesh.phase(q).istatepoints),numel(x),reshape(setup.mesh.phase(q).istatepoints,1,[]));
%     solve.phase(q).state = hderiv(x(setup.mesh.phase(q).statepoints),numel(x),reshape(setup.mesh.phase(q).statepoints,1,[]));
    solve.phase(q).control = hderiv([x(setup.mesh.phase(q).controlpoints);x(setup.mesh.phase(q).controlpoints(end,:))'],numel(x),reshape([setup.mesh.phase(q).controlpoints;setup.mesh.phase(q).controlpoints(end,:)],1,[]));
%     solve.phase(q).control = hderiv(x(setup.mesh.phase(q).controlpoints),numel(x),reshape(setup.mesh.phase(q).controlpoints,1,[]));
    solve.phase(q).initial.time = hderiv(x(setup.mesh.phase(q).initialtimepoint),numel(x),setup.mesh.phase(q).initialtimepoint);
    solve.phase(q).initial.state = hderiv(x(setup.mesh.phase(q).statepoints(1,:))',numel(x),setup.mesh.phase(q).statepoints(1,:));
    solve.phase(q).final.time = hderiv(x(setup.mesh.phase(q).finaltimepoint),numel(x),setup.mesh.phase(q).finaltimepoint);
    solve.phase(q).final.state = hderiv(x(setup.mesh.phase(q).istatepoints(end,:))',numel(x),setup.mesh.phase(q).istatepoints(end,:));
    solve.phase(q).integrand.initialtime = x(setup.mesh.phase(q).initialtimepoint);
    solve.phase(q).integrand.finaltime = x(setup.mesh.phase(q).finaltimepoint);
    solve.phase(q).integrand.locationtime = [setup.mesh.phase(q).initialtimepoint,setup.mesh.phase(q).finaltimepoint];
    solve.phase(q).integrand.weight_vector = setup.phase(q).solver.weight_vector;
    solve.phase(q).time = dtime(solve.phase(q).integrand.initialtime,solve.phase(q).integrand.finaltime,numel(x),solve.phase(q).integrand.locationtime, ...
        setup,q);
    solve.parameter = dparameter(x(setup.mesh.parameterpoint),numel(x),setup.mesh.parameterpoint, ...
        setup,1);
    solve.phase(q).parameter = dparameter(x(setup.mesh.parameterpoint),numel(x),setup.mesh.parameterpoint, ...
        setup,size(setup.mesh.phase(q).istatepoints,1));
%     solve.phase(q).parameter = dparameter(x(setup.mesh.parameterpoint),numel(x),setup.mesh.parameterpoint, ...
%         setup,size(setup.mesh.phase(q).statepoints,1));
    end
    if isfield(setup,'auxdata')
        solve.auxdata = setup.auxdata;
    end
    Dynamics = setup.function;
    dynamics = Dynamics(solve);
%     dynamics.phase(q).derivatives = col(dynamics.phase(q).derivatives);
    for q=1:setup.assist.nP
    dynamics.phase(q).vtime = inifit(solve.phase(q).integrand.initialtime,solve.phase(q).integrand.finaltime,numel(x),solve.phase(q).integrand.locationtime, ...
        setup,q);
    dynamics.phase(q).istate = solve.phase(q).state;
    dynamics.phase(q).derivatives = col(dynamics.phase(q).derivatives);
%     dynamics.phase(q).ivelocity = solve.phase(q).ivelocity;
    dynamics.phase(q).state = col(solve.phase(q).state);
%     dynamics.phase(q).velocity = solve.phase(q).velocity;
%     if setup.phase(q).assist.w ~= 0
%     dynamics.phase(q).state = solve.phase(q).state;
%     dynamics.phase(q).istate = solve.phase(q).istate;
%     end
    end
    cont = cont+1;
    dx_old = x;
    derivatives_old = dynamics;
else
    dynamics = derivatives_old;
end

end