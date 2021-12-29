function Hfinal = Hessian_RPM(x,sigma,lambda,setup)

dynamics = hessian_runner(x,setup);

% Defects
cont = 0;
Hfinal = sparse(numel(x),numel(x));
for q=1:setup.assist.nP
% D = setup.phase(q).solver.D;
% Tm = setup.phase(q).solver.Tm;
% tf = x(setup.mesh.phase(q).finaltimepoint);
% t0 = x(setup.mesh.phase(q).initialtimepoint);
% location = [setup.mesh.phase(q).initialtimepoint,setup.mesh.phase(q).finaltimepoint];
% MD = sparse((1:size(setup.mesh.phase(q).statepoints,1))'*ones(1,size(setup.mesh.phase(q).istatepoints,1)), ...
%     ones(size(setup.mesh.phase(q).statepoints,1),1)*(1:size(setup.mesh.phase(q).istatepoints,1)), ...
%     D,numel(setup.mesh.phase(q).statepoints),numel(x));
% MTm = Tm;
% for i=2:setup.phase(q).assist.n
%     MD = MD + sparse(((1:size(setup.mesh.phase(q).statepoints,1))+(i-1)*size(D,1))'*ones(1,size(setup.mesh.phase(q).istatepoints,1)), ...
%         ones(size(setup.mesh.phase(q).statepoints,1),1)*((1:size(setup.mesh.phase(q).istatepoints,1))+(i-1)*size(D,2)), ...
%         D,numel(setup.mesh.phase(q).statepoints),numel(x));
%     MTm = [MTm,sparse(size(MTm,1),size(Tm,2));sparse(size(Tm,1),size(MTm,2)),Tm];
% end
% 
% [~,~,vTm] = find(Tm);
% vTm = reshape(vTm*(1:setup.phase(q).assist.n),[],1);
% multi = -(tf-t0).*vTm.*lambda(1+cont:size(vTm,1)+cont);
% H1 = shessian(dynamics.phase(q).derivatives,multi,1);
% % J1 = sparse((1:numel(setup.mesh.phase(q).statepoints))'*ones(1,numel(x)),ones(numel(setup.mesh.phase(q).statepoints),1)*(1:numel(x)), ...
% %     MD-(tf-t0)*MTm*jacobian(dynamics.phase(q).derivatives),numel(setup.mesh.phase(q).statepoints),numel(x));
% J = sum(MTm*diag(lambda(1+cont:size(vTm,1)+cont))*jacobian(dynamics.phase(q).derivatives),1);
% H2 = sparse([location(1)*ones(size(J,2),1),location(2)*ones(size(J,2),1)], ...
%             (1:size(J,2))'*ones(1,2),[J',-J'],size(J,2),size(J,2));
% H2d = diag(H2);
% H2d = sparse(1:size(H2,1),1:size(H2,1),H2d);
% H2 = tril(H2+H2'-H2d);
% % J1 = J1 + sparse((1:numel(setup.mesh.phase(q).statepoints))'*ones(1,2),ones(numel(setup.mesh.phase(q).statepoints),1)*location, ...
% %     [reshape(Tm*double(dynamics.phase(q).derivatives),[],1),reshape(-Tm*double(dynamics.phase(q).derivatives),[],1)],numel(setup.mesh.phase(q).statepoints),numel(x));
% Hfinal = Hfinal+tril(H1+H2);
% cont = cont+size(vTm,1);

D = setup.phase(q).solver.D;
Tm = setup.phase(q).solver.Tm;
tf = dynamics.phase(q).vtime(2);
t0 = dynamics.phase(q).vtime(1);
multi = lambda(1+cont:size(D,1)*setup.phase(q).assist.n+cont);
Hstt = shessian(dot(D,dynamics.phase(q).istate)-(tf-t0)*dynamics.phase(q).derivatives,multi,1);
Hfinal = Hfinal+Hstt;
cont = cont + size(D,1)*setup.phase(q).assist.n;

% Path constraints
if isfield(dynamics.phase(q),'path')
    multi = lambda(1+cont:cont+numel(double(dynamics.phase(q).path)));
    Hfinal = Hfinal+shessian(dynamics.phase(q).path,multi,1);
    cont = cont + numel(double(dynamics.phase(q).path));
end
end
% Point constraints
if isfield(dynamics,'constraints')
    multi = lambda(1+cont:cont+numel(double(dynamics.constraints)));
    Hfinal = Hfinal+shessian(dynamics.constraints,multi,1);
    cont = cont + numel(double(dynamics.constraints));
end



% Objective
Hfinal = Hfinal+shessian(dynamics.objective,sigma,1);

end