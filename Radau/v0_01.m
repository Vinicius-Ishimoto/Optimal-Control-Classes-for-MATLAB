%% Radau points test

% Initial parameters
setup.assist.nP = 1;
% setup.phase.dynamics.position = [1,2,3];
% setup.phase.dynamics.velocity = [4,5,6];
setup.mesh.phase.fraction = (1/20)*ones(20,1);
setup.mesh.phase.colpoints = 5*ones(20,1);
setup.phase.assist.n = 2;
setup.phase.assist.p = 1;
setup.phase.assist.v = 0;
setup.phase.assist.cn = 1;
setup.phase.assist.cp = 1;
setup.auxdata.M = 100;
setup.auxdata.C = 15;
x = rand((sum(setup.mesh.phase.colpoints)+1)*2+sum(setup.mesh.phase.colpoints)+2+setup.phase.assist.cn,1);

%% Bounds

setup.bound.lower.phase.initial.state = [0,0];
setup.bound.lower.phase.initial.time = 0;
setup.bound.lower.phase.final.state = [10,0];
setup.bound.lower.phase.final.time = 0.1;
setup.bound.lower.phase.state = [0,0];
setup.bound.lower.phase.control = [-100,-100];
setup.bound.lower.phase.integral = 0;
% setup.bound.lower.parameter = 0;
setup.bound.lower.path = -3;

setup.bound.upper.phase.initial.state = [0,0];
setup.bound.upper.phase.initial.time = 0;
setup.bound.upper.phase.final.state = [10,0];
setup.bound.upper.phase.final.time = 10;
setup.bound.upper.phase.state = [10,inf];
setup.bound.upper.phase.control = [100,100];
setup.bound.upper.phase.integral = inf;
% setup.bound.upper.parameter = 0;
setup.bound.upper.path = 3;



%% Important functions

[D,setup,I,P] = Differentiation_matrix(setup);
[ Tm,setup ] = sparse_timeM(setup);

q = 1;
setup.mesh.phase(q).istatepoints = sparse((1:sum(setup.mesh.phase.colpoints)+1)'*ones(1,setup.phase.assist.n),ones(sum(setup.mesh.phase.colpoints)+1,1)*(1:setup.phase.assist.n),1:setup.phase.assist.n*(sum(setup.mesh.phase.colpoints)+1));

setup.mesh.phase(q).statepoints = setup.mesh.phase(q).istatepoints(1:end-1,:);

setup.mesh.phase(q).controlpoints = sparse((1:sum(setup.mesh.phase.colpoints))'*ones(1,setup.phase.assist.p),ones(sum(setup.mesh.phase.colpoints),1)*(1:setup.phase.assist.p),(1:setup.phase.assist.p*(sum(setup.mesh.phase.colpoints)))+setup.mesh.phase(q).istatepoints(end));

setup.mesh.phase(1).initialtimepoint = setup.mesh.phase(1).controlpoints(end)+1;
setup.mesh.phase(1).finaltimepoint = setup.mesh.phase(1).controlpoints(end)+2;
setup.mesh.phase(1).parameterpoint = (setup.mesh.phase(1).finaltimepoint+1:setup.mesh.phase(1).finaltimepoint+setup.phase(1).assist.v)';
setup.mesh.phase(1).integralpoint = (setup.mesh.phase(1).finaltimepoint+setup.phase(1).assist.v+1:setup.mesh.phase(1).finaltimepoint+setup.phase(1).assist.v+setup.phase(1).assist.cn)';

%% Check Sparcity

q = 1;
solve.phase(q).state = sparse(size(setup.mesh.phase(q).statepoints,1),size(setup.mesh.phase(q).statepoints,2));
solve.phase(q).control = sparse(size(setup.mesh.phase(q).controlpoints,1),size(setup.mesh.phase(q).controlpoints,2));
solve.parameter = sparse(size(setup.mesh.phase(q).parameterpoint,1),size(setup.mesh.phase(q).parameterpoint,2));
solve.phase(q).time = sparse(sum(setup.mesh.phase(q).colpoints),1);
solve.phase.initial.time = 0;
solve.phase.initial.state = sparse(1,setup.phase(q).assist.n);
solve.phase.final.time = 0;
solve.phase.final.state = sparse(1,setup.phase(q).assist.n);
solve.phase.integral = sparse(1,setup.phase(q).assist.cn);
if isfield(setup,'auxdata')
    solve.auxdata = setup.auxdata;
end

%% Introduce feasible values
M = sum(setup.mesh.phase(q).colpoints);
n = setup.phase(q).assist.n;
p = setup.phase(q).assist.p;
v = setup.phase(1).assist.v;

% Continuous
% States
for i=1:setup.phase(q).assist.n
if (isinf(setup.bound.lower.phase.state(i)))&&(isinf(setup.bound.upper.phase.state(i)))
solve.phase(q).state = solve.phase(q).state + sparse(1:M,i,100*rand*ones(1,M),M,n);
elseif (isinf(setup.bound.lower.phase.state(i)))
solve.phase(q).state = solve.phase(q).state + sparse(1:M,i,(setup.bound.upper.phase.state(i)-100*rand)*ones(1,M),M,n);
elseif (isinf(setup.bound.upper.phase.state(i)))
solve.phase(q).state = solve.phase(q).state + sparse(1:M,i,(setup.bound.lower.phase.state(i)+72*rand)*ones(1,M),M,n);
else
solve.phase(q).state = solve.phase(q).state + sparse(1:M,i,(setup.bound.upper.phase.state(i)-setup.bound.lower.phase.state(i))*3/5*ones(1,M)+setup.bound.lower.phase.state(i),M,n);
end
end

% Controls
for i=1:p
if (isinf(setup.bound.lower.phase.control(i)))&&(isinf(setup.bound.upper.phase.control(i)))
solve.phase(q).control = solve.phase(q).control + sparse(1:M,i,100*rand*ones(1,M),M,p);
elseif (isinf(setup.bound.lower.phase.control(i)))
solve.phase(q).control = solve.phase(q).control + sparse(1:M,i,setup.bound.upper.phase.control(i)*rand*ones(1,M),M,p);
elseif (isinf(setup.bound.upper.phase.control(i)))
solve.phase(q).control = solve.phase(q).control + sparse(1:M,i,setup.bound.lower.phase.control(i)*72*ones(1,M),M,p);
else
solve.phase(q).control = solve.phase(q).control + sparse(1:M,i,(setup.bound.upper.phase.control(i)-setup.bound.lower.phase.control(i))*3/5*ones(1,M)+setup.bound.lower.phase.control(i),M,p);
end
end

% Time
for i=1:p
if (isinf(setup.bound.lower.phase.final.time(i)))&&(isinf(setup.bound.upper.phase.final.time(i)))
solve.phase(q).time = solve.phase(q).time + sparse(1:M,i,100*rand*ones(1,M),M,p);
elseif (isinf(setup.bound.lower.phase.final.time(i)))
solve.phase(q).time = solve.phase(q).time + sparse(1:M,i,setup.bound.upper.phase.final.time(i)*rand*ones(1,M),M,p);
elseif (isinf(setup.bound.upper.phase.final.time(i)))
solve.phase(q).time = solve.phase(q).time + sparse(1:M,i,setup.bound.lower.phase.final.time(i)*72*ones(1,M),M,p);
else
solve.phase(q).time = solve.phase(q).time + sparse(1:M,i,(setup.bound.upper.phase.final.time(i)-setup.bound.lower.phase.final.time(i))*3/5*ones(1,M)+setup.bound.lower.phase.final.time(i),M,p);
end
end

% Points
% Initial State
for i=1:setup.phase(q).assist.n
if (isinf(setup.bound.lower.phase.initial.state(i)))&&(isinf(setup.bound.upper.phase.initial.state(i)))
solve.phase(q).initial.state = solve.phase(q).initial.state + sparse(1,i,100*rand,1,n);
elseif (isinf(setup.bound.lower.phase.initial.state(i)))
solve.phase(q).initial.state = solve.phase(q).initial.state + sparse(1,i,(setup.bound.upper.phase.initial.state(i)-100*rand),1,n);
elseif (isinf(setup.bound.upper.phase.initial.state(i)))
solve.phase(q).initial.state = solve.phase(q).initial.state + sparse(1,i,(setup.bound.lower.phase.initial.state(i)+72*rand),1,n);
else
solve.phase(q).initial.state = solve.phase(q).initial.state + sparse(1,i,(setup.bound.upper.phase.initial.state(i)-setup.bound.lower.phase.initial.state(i))*3/5+setup.bound.lower.phase.initial.state(i),1,n);
end
end

% Final State
for i=1:setup.phase(q).assist.n
if (isinf(setup.bound.lower.phase.final.state(i)))&&(isinf(setup.bound.upper.phase.final.state(i)))
solve.phase(q).final.state = solve.phase(q).final.state + sparse(1,i,100*rand,1,n);
elseif (isinf(setup.bound.lower.phase.final.state(i)))
solve.phase(q).final.state = solve.phase(q).final.state + sparse(1,i,(setup.bound.upper.phase.final.state(i)-100*rand),1,n);
elseif (isinf(setup.bound.upper.phase.final.state(i)))
solve.phase(q).final.state = solve.phase(q).final.state + sparse(1,i,(setup.bound.lower.phase.final.state(i)+72*rand),1,n);
else
solve.phase(q).final.state = solve.phase(q).final.state + sparse(1,i,(setup.bound.upper.phase.final.state(i)-setup.bound.lower.phase.final.state(i))*3/5+setup.bound.lower.phase.final.state(i),1,n);
end
end

% Initial time
i = 1;
% for i=1:setup.phase(q).assist.n
if (isinf(setup.bound.lower.phase.initial.time(i)))&&(isinf(setup.bound.upper.phase.initial.time(i)))
solve.phase(q).initial.time = solve.phase(q).initial.time + sparse(1,i,100*rand,1,1);
elseif (isinf(setup.bound.lower.phase.initial.time(i)))
solve.phase(q).initial.time = solve.phase(q).initial.time + sparse(1,i,(setup.bound.upper.phase.initial.time(i)-100*rand),1,1);
elseif (isinf(setup.bound.upper.phase.initial.time(i)))
solve.phase(q).initial.time = solve.phase(q).initial.time + sparse(1,i,(setup.bound.lower.phase.initial.time(i)+72*rand),1,1);
else
solve.phase(q).initial.time = solve.phase(q).initial.time + sparse(1,i,(setup.bound.upper.phase.initial.time(i)-setup.bound.lower.phase.initial.time(i))*3/5+setup.bound.lower.phase.initial.time(i),1,1);
end
% end

% Final time
% for i=1:setup.phase(q).assist.n
if (isinf(setup.bound.lower.phase.final.time(i)))&&(isinf(setup.bound.upper.phase.final.time(i)))
solve.phase(q).final.time = solve.phase(q).final.time + sparse(1,i,100*rand,1,1);
elseif (isinf(setup.bound.lower.phase.final.time(i)))
solve.phase(q).final.time = solve.phase(q).final.time + sparse(1,i,(setup.bound.upper.phase.final.time(i)-100*rand),1,1);
elseif (isinf(setup.bound.upper.phase.final.time(i)))
solve.phase(q).final.time = solve.phase(q).final.time + sparse(1,i,(setup.bound.lower.phase.final.time(i)+72*rand),1,1);
else
solve.phase(q).final.time = solve.phase(q).final.time + sparse(1,i,(setup.bound.upper.phase.final.time(i)-setup.bound.lower.phase.final.time(i))*3/5+setup.bound.lower.phase.final.time(i),1,1);
end
% end

% Integral
for i=1:setup.phase.assist.cn
if (isinf(setup.bound.lower.phase.integral(i)))&&(isinf(setup.bound.upper.phase.integral(i)))
solve.phase(q).integral = solve.phase(q).integral + sparse(1,i,100*rand,1,setup.phase.assist.cn);
elseif (isinf(setup.bound.lower.phase.integral(i)))
solve.phase(q).integral = solve.phase(q).integral + sparse(1,i,setup.bound.upper.phase.integral(i)*rand,1,setup.phase.assist.cn);
elseif (isinf(setup.bound.upper.phase.integral(i)))
solve.phase(q).integral = solve.phase(q).integral + sparse(1,i,setup.bound.lower.phase.integral(i)*72,1,setup.phase.assist.cn);
else
solve.phase(q).integral = solve.phase(q).integral + sparse(1,i,(setup.bound.upper.phase.integral(i)-setup.bound.lower.phase.integral(i))*3/5+setup.bound.lower.phase.integral(i),1,setup.phase.assist.cn);
end
end

% Parameter
for i=1:setup.phase(1).assist.v
if (isinf(setup.bound.lower.parameter(i)))&&(isinf(setup.bound.upper.parameter(i)))
solve.parameter = solve.parameter + sparse(1,i,100*rand,1,setup.phase(1).assist.v);
elseif (isinf(setup.bound.lower.parameter(i)))
solve.parameter = solve.parameter + sparse(1,i,setup.bound.upper.parameter(i)*rand,1,setup.phase(1).assist.v);
elseif (isinf(setup.bound.upper.parameter(i)))
solve.parameter = solve.parameter + sparse(1,i,setup.bound.lower.parameter(i)*72,1,setup.phase(1).assist.v);
else
solve.parameter = solve.parameter + sparse(1,i,(setup.bound.upper.parameter(i)-setup.bound.lower.parameter(i))*3/5+setup.bound.lower.parameter(i),1,setup.phase(1).assist.v);
end
end

%% Sparcity Checker

solve.phase(q).solve.weight_vector = setup.phase(q).solver.weight_vector;
solve.phase(q).solve.initialtime = solve.phase.initial.time;
solve.phase(q).solve.finaltime = solve.phase.final.time;
solve.phase(q).solve.locationtime = [setup.mesh.phase(q).initialtimepoint,setup.mesh.phase(q).finaltimepoint];
M = sum(setup.mesh.phase(q).colpoints);
main = solve;
% Continuous
% State
for i=1:setup.phase(q).assist.n
    solve.phase(q).state(:,i) = nan(M,1);
    answer = Dynamics(solve);
    Dstr(:,i) = (any(isnan(answer.phase(q).derivatives),1))';
    if isfield(answer.phase(q),'integrand')
        Istr(:,i) = (any(isnan(answer.phase(q).integrand),1))';
    end
    if isfield(answer,'path')
        Pstr(:,i) = (any(isnan(answer.path),1))';
    end
    if isfield(answer,'objective')
        if any(any(isnan(answer.objective),1))
            error('Don''t use continuous variables as objective.')
        end
    end
    solve = main;
end

% Control
for i=1:setup.phase(q).assist.p
    solve.phase(q).control(:,i) = nan(M,1);
    answer = Dynamics(solve);
    Dstr(:,i+setup.phase(q).assist.n) = (any(isnan(answer.phase(q).derivatives),1))';
    if isfield(answer.phase(q),'integrand')
        Istr(:,i+setup.phase(q).assist.n) = (any(isnan(answer.phase(q).integrand),1))';
    end
    if isfield(answer,'path')
        Pstr(:,i+setup.phase(q).assist.n) = (any(isnan(answer.path),1))';
    end
    if isfield(answer,'objective')
        if any(any(isnan(answer.objective),1))
            error('Don''t use continuous variables as objective.')
        end
    end
    solve = main;
end

% Time
solve.phase(q).time = nan(M,1);
answer = Dynamics(solve);
Dstr(:,1+setup.phase(q).assist.n+setup.phase(q).assist.p) = (any(isnan(answer.phase(q).derivatives),1))';
if isfield(answer.phase(q),'integrand')
    Istr(:,1+setup.phase(q).assist.n+setup.phase(q).assist.p) = (any(isnan(answer.phase(q).integrand),1))';
end
if isfield(answer,'path')
    Pstr(:,1+setup.phase(q).assist.n+setup.phase(q).assist.p) = (any(isnan(answer.path),1))';
end
if isfield(answer,'objective')
    if any(any(isnan(answer.objective),1))
        error('Don''t use continuous variables as objective.')
    end
end
solve = main;

