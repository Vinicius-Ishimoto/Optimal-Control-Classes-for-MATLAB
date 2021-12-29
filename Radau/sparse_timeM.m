function [ Tm,setup ] = sparse_timeM(setup)

for q=1:setup.assist.nP
C = setup.mesh.phase(q).colpoints;
% t = setup.phase(q).assist.t;
V = setup.mesh.phase(q).fraction;
N = sum(C);
cC = cumsum(V);
weight = setup.mesh.phase(q).weight;

% tV = [0;cumsum(V)]*(t(2)-t(1))+t(1)*ones(size(V,1)+1,size(V,2));
Tm = sparse(N,N);
W = [];
tV = [];
sT = [];
cCC = [0;cC];
for i=1:numel(V)
    
    Tm = Tm+sparse((1+(i-1)*C(i):i*C(i))'*ones(1,C(i)),ones(C(i),1)*(1+(i-1)*C:i*C),0.5*V(i)*eye(C(i),C(i)),N,N);
    W = [W,weight{i}'*0.5*V(i)];
    tV = [tV;ones(C(i),1)*2*cC(i)];
    sT = [sT;V(i)*0.5*(setup.mesh.phase(q).tau{i}+1)+cCC(i)];
end

setup.mesh.phase(q).time = tV;
setup.mesh.phase(q).ctime = sT;
setup.phase(q).solver.Tm = Tm;
setup.phase(q).assist.M = sum(C);
setup.phase(q).solver.weight_vector = W;
end
end