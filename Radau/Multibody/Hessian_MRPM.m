function Hfinal = Hessian_MRPM(x,sigma,lambda,setup)

dynamics = hessian_Mrunner(x,setup);

% Defects
cont = 0;
Hfinal = sparse(numel(x),numel(x));
for q=1:setup.assist.nP
D = setup.phase(q).solver.D;
Tm = setup.phase(q).solver.Tm;
tf = dynamics.phase(q).vtime(2);
t0 = dynamics.phase(q).vtime(1);
multi = lambda(1+cont:size(D,1)*0.5*setup.phase(q).assist.n+cont);
Hpos = shessian(dot(D,dynamics.phase(q).iposition)-(tf-t0)*dynamics.phase(q).velocity,multi,1);
cont = cont+size(D,1)*0.5*setup.phase(q).assist.n;
multi = lambda(1+cont:size(D,1)*0.5*setup.phase(q).assist.n+cont);
Hvel = shessian(dynamics.phase(q).MassMatrix*dot(D,dynamics.phase(q).ivelocity)'-(tf-t0)*dynamics.phase(q).RightHandSide,multi,1);

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
% J1 = J1 + sparse((1:numel(setup.mesh.phase(q).statepoints))'*ones(1,2),ones(numel(setup.mesh.phase(q).statepoints),1)*location, ...
%     [reshape(Tm*double(dynamics.phase(q).derivatives),[],1),reshape(-Tm*double(dynamics.phase(q).derivatives),[],1)],numel(setup.mesh.phase(q).statepoints),numel(x));
Hfinal = Hfinal+Hpos+Hvel;
cont = cont+size(D,1)*0.5*setup.phase(q).assist.n;

% New states
if isfield(dynamics.phase(q),'derivatives')
    multi = lambda(1+cont:size(D,1)*setup.phase(q).assist.w+cont);
    Hstt = shessian(dot(D,dynamics.phase(q).istate)-(tf-t0)*dynamics.phase(q).derivatives,multi,1);
    Hfinal = Hfinal+Hstt;
    cont = cont + size(D,1)*setup.phase(q).assist.w;
end

% Path constraints
if isfield(dynamics.phase(q),'path')
    multi = lambda(1+cont:cont+numel(double(dynamics.phase(q).path)));
    Hfinal = Hfinal+shessian(dynamics.phase(q).path,multi,1);
    cont = cont + numel(double(dynamics.phase(q).path));
end
% Stabilization constraints
if isfield(dynamics.phase(q),'stabilization')
    if not(isempty(dynamics.phase(q).stabilization))
    if strcmp(setup.phase(q).stabilization_method,'Integration')
    c2 = cont;
    multi1 = [];
    multi2 = [];
    for ij=1:setup.phase(q).stabilization_constraints
        multi1 = [multi1,reshape(lambda(1+c2:c2+sum(setup.mesh.phase(q).colpoints)),1,[])*setup.phase(q).solver.P];
        multi2 = [multi2,reshape(lambda(1+c2:c2+sum(setup.mesh.phase(q).colpoints)),1,[])*setup.phase(q).solver.I];
        c2 = c2+sum(setup.mesh.phase(q).colpoints);
    end
%     multi = reshape(multi,1,[])*setup.phase(q).solver.I;
    H1 = shessian(dynamics.phase(q).stabilization,multi1',1);
    H2 = shessian((tf-t0)*col(dynamics.phase(q).stabilization),multi2',1);
    Hfinal = Hfinal+H1+setup.sigma*H2;
    cont = cont + setup.phase(q).stabilization_constraints*sum(setup.mesh.phase(q).colpoints);
    multi = lambda(1+cont:cont+setup.phase(q).stabilization_constraints);
    Hfinal = Hfinal+shessian(col(dynamics.phase(q).stabilization,1),multi,1);
    cont = cont + setup.phase(q).stabilization_constraints;
    else
    c2 = cont;
    multi1 = [];
    multi = lambda(1+cont:cont+setup.phase(q).stabilization_constraints*sum(setup.mesh.phase(q).colpoints));
%     multi2 = [];
    for ij=1:setup.phase(q).stabilization_constraints
        multi1 = [multi1,reshape(lambda(1+c2:c2+sum(setup.mesh.phase(q).colpoints)),1,[])*setup.phase(q).solver.D];
%         multi2 = [multi2,reshape(lambda(1+c2:c2+sum(setup.mesh.phase(q).colpoints)),1,[])*setup.phase(q).solver.I];
        c2 = c2+sum(setup.mesh.phase(q).colpoints);
    end
%     multi = reshape(multi,1,[])*setup.phase(q).solver.I;
    H1 = shessian(dynamics.phase(q).stabilization,multi1',1);
    H2 = shessian((tf-t0)*col(dynamics.phase(q).stabilization),multi,1);
    Hfinal = Hfinal+H1+setup.sigma*H2;
    cont = cont + setup.phase(q).stabilization_constraints*sum(setup.mesh.phase(q).colpoints);
    end
    end
%     multi = lambda(1+cont:cont+setup.phase(q).stabilization_constraints);
%     Hfinal = Hfinal+shessian(col(dynamics.phase(q).stabilization,1),multi,1);
%     cont = cont + setup.phase(q).stabilization_constraints;
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