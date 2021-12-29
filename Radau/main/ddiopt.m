function output = ddiopt(setup)

setup.assist.nP = numel(setup.bound.upper.phase);
setup.mesh.flag = 1;
for q=1:setup.assist.nP
    try
        M = legslb(setup.phase(q).mesh_number+1);
        setup.mesh.phase(q).colpoints = setup.phase(q).mesh_points*ones(setup.phase(q).mesh_number,1);
        M = 0.5*(M+1);
        setup.mesh.phase(q).fraction = M(2:end)-M(1:end-1);
    catch
        M = legslb(setup.mesh_number+1);
        setup.mesh.phase(q).colpoints = setup.mesh_points*ones(setup.mesh_number,1);
        M = 0.5*(M+1);
        setup.mesh.phase(q).fraction = M(2:end)-M(1:end-1);
    end
end

for i=1:setup.assist.nP
    setup.phase(i).assist.n = numel(setup.bound.lower.phase(i).state);
    setup.phase(i).assist.p = numel(setup.bound.lower.phase(i).control);
try
    setup.phase(i).assist.cp = numel(setup.bound.lower.phase(i).path); 
catch
    setup.phase(i).assist.cp = 0;
end
end
    try
        setup.assist.cn = numel(setup.bound.lower.pconstraints);
    catch
        setup.assist.cn = 0;
    end
try
    setup.assist.v = numel(setup.bound.lower.parameter);
catch
    setup.assist.v = 0;
end

global D
[D,setup,I,P] = Differentiation_matrix(setup);
[ Tm,setup ] = sparse_timeM(setup);
xg = initial_guess(setup);

number = 0;
for q=1:setup.assist.nP
setup.mesh.phase(q).istatepoints = sparse((1:sum(setup.mesh.phase(q).colpoints)+1)'*ones(1,setup.phase(q).assist.n),ones(sum(setup.mesh.phase(q).colpoints)+1,1)*(1:setup.phase(q).assist.n),1+number:setup.phase(q).assist.n*(sum(setup.mesh.phase(q).colpoints)+1)+number);
setup.mesh.phase(q).statepoints = setup.mesh.phase(q).istatepoints(1:end-1,:);
setup.mesh.phase(q).controlpoints = sparse((1:sum(setup.mesh.phase(q).colpoints))'*ones(1,setup.phase(q).assist.p),ones(sum(setup.mesh.phase(q).colpoints),1)*(1:setup.phase(q).assist.p),(1:setup.phase(q).assist.p*(sum(setup.mesh.phase(q).colpoints)))+setup.mesh.phase(q).istatepoints(end));
setup.mesh.phase(q).initialtimepoint = setup.mesh.phase(q).controlpoints(end)+1;
setup.mesh.phase(q).finaltimepoint = setup.mesh.phase(q).controlpoints(end)+2;
number = setup.mesh.phase(q).finaltimepoint;
end
setup.mesh.parameterpoint = (number+1:number+setup.assist.v)';
global x_old dx_old
x_old = rand(numel(xg),1);
dx_old = rand(numel(xg),1);

try
    constraints = Constraints_RPM(xg,setup);
catch ME
    link = ['<a href="matlab:opentoline(',strrep(ME.stack(1).file,'\','\\'),',',num2str(ME.stack(1).line),',0)">line ',num2str(ME.stack(1).line),'</a>'];
    error(ME.identifier,['Constraints function could not be evaluated because:\n',ME.message,'\nIn ',ME.stack(1).name,' and ',link,'.'])
end
try
    Jfinal = Jacobian_RPM(xg,setup);
catch ME
    link = ['<a href="matlab:opentoline(',strrep(ME.stack(1).file,'\','\\'),',',num2str(ME.stack(1).line),',0)">line ',num2str(ME.stack(1).line),'</a>'];
    error(ME.identifier,['Jacobian function could not be evaluated because:\n',ME.message,'\nIn ',ME.stack(1).name,' and ',link,'.'])
end
try
    gradient = Gradient_RPM(xg,setup);
catch ME
    link = ['<a href="matlab:opentoline(',strrep(ME.stack(1).file,'\','\\'),',',num2str(ME.stack(1).line),',0)">line ',num2str(ME.stack(1).line),'</a>'];
    error(ME.identifier,['Gradient function could not be evaluated because:\n',ME.message,'\nIn ',ME.stack(1).name,' and ',link,'.'])
end
try
    objective = Objective_RPM(xg,setup);
catch ME
    link = ['<a href="matlab:opentoline(',strrep(ME.stack(1).file,'\','\\'),',',num2str(ME.stack(1).line),',0)">line ',num2str(ME.stack(1).line),'</a>'];
    error(ME.identifier,['Objective function could not be evaluated because:\n',ME.message,'\nIn ',ME.stack(1).name,' and ',link,'.'])
end

options.lb = [];
options.ub = [];
options.cl = [];
options.cu = [];
for q=1:setup.assist.nP
    for i=1:setup.phase(q).assist.n
    options.lb = [options.lb;setup.bound.lower.phase(q).initial.state(i)];
    options.ub = [options.ub;setup.bound.upper.phase(q).initial.state(i)];
    options.lb = [options.lb;setup.bound.lower.phase(q).state(i)*ones(sum(setup.mesh.phase(q).colpoints)-1,1)];
    options.ub = [options.ub;setup.bound.upper.phase(q).state(i)*ones(sum(setup.mesh.phase(q).colpoints)-1,1)];
    options.lb = [options.lb;setup.bound.lower.phase(q).final.state(i)];
    options.ub = [options.ub;setup.bound.upper.phase(q).final.state(i)];
    options.cl = [options.cl;zeros(sum(setup.mesh.phase(q).colpoints),1)];
    options.cu = [options.cu;zeros(sum(setup.mesh.phase(q).colpoints),1)];
    end
    for i=1:setup.phase(q).assist.p
        options.lb = [options.lb;setup.bound.lower.phase(q).control(i)*ones(sum(setup.mesh.phase(q).colpoints),1)];
        options.ub = [options.ub;setup.bound.upper.phase(q).control(i)*ones(sum(setup.mesh.phase(q).colpoints),1)];
    end
    options.lb = [options.lb;setup.bound.lower.phase(q).initial.time];
    options.ub = [options.ub;setup.bound.upper.phase(q).initial.time];
    options.lb = [options.lb;setup.bound.lower.phase(q).final.time];
    options.ub = [options.ub;setup.bound.upper.phase(q).final.time];
    if isfield(setup.bound.lower.phase(q),'path')
    options.cl = [options.cl;reshape(ones(sum(setup.mesh.phase(q).colpoints),1)*reshape(setup.bound.lower.phase(q).path,1,[]),[],1)];
    options.cu = [options.cu;reshape(ones(sum(setup.mesh.phase(q).colpoints),1)*reshape(setup.bound.upper.phase(q).path,1,[]),[],1)];
    end
end
try
options.cl = [options.cl;reshape(setup.bound.lower.pconstraints,[],1)];
options.cu = [options.cu;reshape(setup.bound.upper.pconstraints,[],1)];
catch
end
try
options.lb = [options.lb;reshape(setup.bound.lower.parameter,[],1)];
options.ub = [options.ub;reshape(setup.bound.upper.parameter,[],1)];
catch
end

try
    hessian = Hessian_RPM(xg,12,rand(numel(options.cl),1),setup);
catch ME
    link = ['<a href="matlab:opentoline(',strrep(ME.stack(1).file,'\','\\'),',',num2str(ME.stack(1).line),',0)">line ',num2str(ME.stack(1).line),'</a>'];
    error(ME.identifier,['Hessian function could not be evaluated because:\n',ME.message,'\nIn ',ME.stack(1).name,' and ',link,'.'])
end

%% IPOPT

%   options.ipopt.hessian_approximation = 'limited-memory';
  options.ipopt.max_iter               = 5000;
%   options.ipopt.nlp_scaling_method    = 'none';
  options.ipopt.mu_strategy           = 'adaptive';
  options.ipopt.tol                   = 1e-7;
%   options.ipopt.derivative_test       = 'second-order';
  Jacstr = Jacobian_RPM(rand(numel(xg),1),setup)~=0;
  Hesstr = Hessian_RPM(rand(numel(xg),1),12,rand(numel(options.cl),1),setup)~=0;

  
global flag
if isempty(flag)
    flag.fig = [];
end
if ishandle(flag.fig)
    delete(flag.pl1)
    flag.h1 = newplot(flag.fig);
    flag.x = 0;
    flag.y11 = objective;
    flag.pl1 = line('XData',flag.x,'YData',flag.y11,'Color',[0,0,1],'Parent',flag.h1,'linewidth',2);
%     close(flag.fig)
else
flag.fig = figure('Name','optimization');
% flag.h1 = subplot(2,1,1,'hold','on');
% hold on
flag.h1 = newplot([]);
% flag.pl1 = line('parent',flag.h1,'color',[0,0,0],'marker','o','erase','none', ...
%               'xdata',[],'ydata',[]);
% flag.p11 = plot(x(1),y11(1));
flag.pl1 = plot(0,objective);
% set(flag.h1,'XData',x(1),'YData',y11(1))
% ax = get(flag.pl1,'Parent');
%      hold(ax)
% flag.h2 = subplot(2,1,2,'hold','on');
% flag.p21 = plot(x(1),y21(1));
% flag.p22 = plot(x(1),y22(1));
% set(flag.h1,'XData',x(1),'YData',y11(1))
flag.x = 0;
flag.y11 = objective;
end
  
  % The callback functions.
  funcs.objective         = @(x) Objective_RPM(x,setup);
  funcs.constraints       = @(x) Constraints_RPM(x,setup);
  funcs.gradient          = @(x) Gradient_RPM(x,setup);
  funcs.jacobian          = @(x) Jacobian_RPM(x,setup);
  funcs.jacobianstructure = @() Jacstr;
  funcs.hessian           = @(x,sigma,lambda) Hessian_RPM(x,sigma,lambda,setup);
  funcs.hessianstructure  = @() Hesstr;
  funcs.iterfunc          = @stop_run;
  
  % Run IPOPT.
  [x,info] = ipopt(xg,funcs,options);
  
for q=1:setup.assist.nP
setup.solution.phase(q).time = [double(dtime(x(setup.mesh.phase(q).initialtimepoint),x(setup.mesh.phase(q).finaltimepoint),2,[1,2],setup,q));x(setup.mesh.phase(q).finaltimepoint)];
setup.solution.phase(q).state = x(setup.mesh.phase(q).istatepoints);
setup.solution.phase(q).control = x(setup.mesh.phase(q).controlpoints);
setup.solution.phase(q).control = [setup.solution.phase(q).control;setup.solution.phase(q).control(end,:)];
end
setup.solution.parameter = x(setup.mesh.parameterpoint);

setup.solution.info = info;
setup.solution.vector = x;
setup.solution.constraints = Constraints_RPM(x,setup);
setup.solution.objective = Objective_RPM(x,setup);

output = setup;

end