function [ dynamics,setup ] = derivative_runner(x,setup)

global dx_old derivatives_old cont

if any(dx_old~=x)
    q = 1;
    solve.phase(q).istate = deriv(x(setup.mesh.phase(q).istatepoints),numel(x),reshape(setup.mesh.phase(q).istatepoints,1,[]));
    solve.phase(q).state = deriv(x(setup.mesh.phase(q).statepoints),numel(x),reshape(setup.mesh.phase(q).statepoints,1,[]));
    solve.phase(q).control = deriv(x(setup.mesh.phase(q).controlpoints),numel(x),reshape(setup.mesh.phase(q).controlpoints,1,[]));
    solve.phase(q).initial.time = deriv(x(setup.mesh.phase(q).initialtimepoint),numel(x),setup.mesh.phase(q).initialtimepoint);
    solve.phase(q).initial.state = deriv(x(setup.mesh.phase(q).statepoints(1,:)),numel(x),setup.mesh.phase(q).statepoints(1,:));
    solve.phase(q).final.time = deriv(x(setup.mesh.phase(q).finaltimepoint),numel(x),setup.mesh.phase(q).initialtimepoint);
    solve.phase(q).final.state = deriv(x(setup.mesh.phase(q).istatepoints(end,:)),numel(x),setup.mesh.phase(q).istatepoints(end,:));
    solve.phase(q).integrand.initialtime = x(setup.mesh.phase(q).initialtimepoint);
    solve.phase(q).integrand.finaltime = x(setup.mesh.phase(q).finaltimepoint);
    solve.phase(q).integrand.locationtime = [setup.mesh.phase(q).initialtimepoint,setup.mesh.phase(q).finaltimepoint];
    solve.phase(q).integrand.weight_vector = setup.phase(q).solver.weight_vector;
    solve.phase(q).time = dtime(solve.phase(q).integrand.initialtime,solve.phase(q).integrand.finaltime,numel(x),solve.phase(q).integrand.locationtime, ...
        setup,q);
    solve.parameter = x(setup.mesh.phase(1).parameterpoint);
    if isfield(setup,'auxdata')
        solve.auxdata = setup.auxdata;
    end
    dynamics = Dynamics(solve);
    cont = cont+1;
    dx_old = x;
    derivatives_old = dynamics;
else
    dynamics = derivatives_old;
end

end