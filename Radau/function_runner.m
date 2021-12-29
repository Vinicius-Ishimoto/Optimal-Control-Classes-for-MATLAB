function [ dynamics,setup ] = function_runner(x,setup)

global x_old dynamics_old

if any(x_old~=x)
    for q=1:setup.assist.nP
    solve.phase(q).state = vect(x(setup.mesh.phase(q).istatepoints));
%     solve.phase(q).state = vect(x(setup.mesh.phase(q).statepoints));
    solve.phase(q).control = vect([x(setup.mesh.phase(q).controlpoints);x(setup.mesh.phase(q).controlpoints(end,:))']);
%     solve.phase(q).control = vect(x(setup.mesh.phase(q).controlpoints));
    solve.phase(q).initial.time = x(setup.mesh.phase(q).initialtimepoint);
    solve.phase(q).initial.state = x(setup.mesh.phase(q).istatepoints(1,:))';
    solve.phase(q).final.time = x(setup.mesh.phase(q).finaltimepoint);
    solve.phase(q).final.state = x(setup.mesh.phase(q).istatepoints(end,:))';
    solve.phase(q).integrand.initialtime = solve.phase(q).initial.time;
    solve.phase(q).integrand.finaltime = solve.phase(q).final.time;
    solve.phase(q).integrand.locationtime = [setup.mesh.phase(q).initialtimepoint,setup.mesh.phase(q).finaltimepoint];
    solve.phase(q).integrand.weight_vector = setup.phase(q).solver.weight_vector;
    solve.parameter = x(setup.mesh.parameterpoint);
    solve.phase(q).parameter = vect(ones(sum(setup.mesh.phase(q).colpoints)+1,1)*reshape(x(setup.mesh.parameterpoint),1,[]));
%     solve.phase(q).parameter = vect(ones(sum(setup.mesh.phase(q).colpoints),1)*reshape(x(setup.mesh.parameterpoint),1,[]));
    end
    
    if isfield(setup,'auxdata')
        solve.auxdata = setup.auxdata;
    end
    Dynamics = setup.function;
    dynamics = Dynamics(solve);
    for q=1:setup.assist.nP
    dynamics.phase(q).derivatives = col(dynamics.phase(q).derivatives);
    end
    x_old = x;
    dynamics_old = dynamics;
else
    dynamics = dynamics_old;
end

end