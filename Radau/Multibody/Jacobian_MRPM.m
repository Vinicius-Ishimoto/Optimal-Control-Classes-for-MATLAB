function Jfinal = Jacobian_MRPM(x,setup)

if setup.mesh.flag==1
    dynamics = hessian_Mrunner(x,setup);
else
dynamics = derivative_runner(x,setup);
end
% Defects
Jfinal = [];
for q=1:setup.assist.nP
D = setup.phase(q).solver.D;
Tm = setup.phase(q).solver.Tm;
tf = dynamics.phase(q).vtime(2);
t0 = dynamics.phase(q).vtime(1);

Jpos = jacobian(dot(D,dynamics.phase(q).iposition)-(tf-t0)*dynamics.phase(q).velocity);
Jvel = jacobian(dynamics.phase(q).MassMatrix*dot(D,dynamics.phase(q).ivelocity)'-(tf-t0)*dynamics.phase(q).RightHandSide);
% location = [setup.mesh.phase(q).initialtimepoint,setup.mesh.phase(q).finaltimepoint];
% MD = sparse((1:size(setup.mesh.phase(q).statepoints,1))'*ones(1,size(setup.mesh.phase(q).istatepoints,1)), ...
%     ones(size(setup.mesh.phase(q).statepoints,1),1)*(setup.mesh.phase(q).statepoints(1,1):size(setup.mesh.phase(q).istatepoints,1)+setup.mesh.phase(q).statepoints(1,1)-1), ...
%     D,numel(setup.mesh.phase(q).statepoints),numel(x));
% MTm = Tm;
% for i=2:setup.phase(q).assist.n
%     MD = MD + sparse(((1:size(setup.mesh.phase(q).statepoints,1))+(i-1)*size(D,1))'*ones(1,size(setup.mesh.phase(q).istatepoints,1)), ...
%         ones(size(setup.mesh.phase(q).statepoints,1),1)*((setup.mesh.phase(q).statepoints(1,1):size(setup.mesh.phase(q).istatepoints,1)+setup.mesh.phase(q).statepoints(1,1)-1)+(i-1)*size(D,2)), ...
%         D,numel(setup.mesh.phase(q).statepoints),numel(x));
%     MTm = [MTm,sparse(size(MTm,1),size(Tm,2));sparse(size(Tm,1),size(MTm,2)),Tm];
% end
% 
% J1 = sparse((1:numel(setup.mesh.phase(q).statepoints))'*ones(1,numel(x)),ones(numel(setup.mesh.phase(q).statepoints),1)*(1:numel(x)), ...
%     MD-(tf-t0)*MTm*jacobian(dynamics.phase(q).derivatives),numel(setup.mesh.phase(q).statepoints),numel(x));
% J1 = J1 + sparse((1:numel(setup.mesh.phase(q).statepoints))'*ones(1,2),ones(numel(setup.mesh.phase(q).statepoints),1)*location, ...
%     [reshape(Tm*double(dynamics.phase(q).derivatives),[],1),reshape(-Tm*double(dynamics.phase(q).derivatives),[],1)],numel(setup.mesh.phase(q).statepoints),numel(x));
Jfinal = [Jfinal;Jpos;Jvel];

% New states
if isfield(dynamics.phase(q),'derivatives')
    Jstt = jacobian(dot(D,dynamics.phase(q).istate)-(tf-t0)*dynamics.phase(q).derivatives);
    Jfinal = [Jfinal; Jstt];
end

% Path constraints
if isfield(dynamics.phase(q),'path')
    Jfinal = [Jfinal; jacobian(dynamics.phase(q).path)];
end
% Stabilization constraints
if isfield(dynamics.phase(q),'stabilization')
    if not(isempty(dynamics.phase(q).stabilization))
    if strcmp(setup.phase(q).stabilization_method,'Integration')
    M1 = dot(setup.phase(q).solver.P,dynamics.phase(q).stabilization);
    M2 = setup.sigma*dot(setup.phase(q).solver.I,(tf-t0)*col(dynamics.phase(q).stabilization));
    Jfinal = [Jfinal; jacobian(M1+M2); jacobian(col(dynamics.phase(q).stabilization,1))];
    else
    M1 = dot(setup.phase(q).solver.D,dynamics.phase(q).stabilization);
    M2 = setup.sigma*(tf-t0)*col(dynamics.phase(q).stabilization);
    Jfinal = [Jfinal; jacobian(M1+M2)];
    end
    end
%     Jfinal = [Jfinal; jacobian(M1+M2); jacobian(col(dynamics.phase(q).stabilization,1))];
end
end
% Point constraints
if isfield(dynamics,'constraints')
    Jfinal = [Jfinal; jacobian(dynamics.constraints)];
end



end