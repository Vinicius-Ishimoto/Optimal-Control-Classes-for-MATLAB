function [ Df,setup,I,P ] = Differentiation_matrix(setup)

% V = mesh.fraction;
% tau = -ones(numel(V)+1,1);
% 
% for i=1:numel(V)
%     tau(i+1)=tau(i)+V(i)*2;
% end


for q=1:setup.assist.nP
    wv = [];
    mesh = setup.mesh.phase(q);
D = sparse(sum(mesh.colpoints),sum(mesh.colpoints)+1);
N = sum(mesh.colpoints);
for i=1:numel(mesh.fraction)
    
[tau{i},w{i}] = legsrd(mesh.colpoints(i));
wv = [wv;tau{i}];
fD = sparse_D(mesh.colpoints(i),[tau{i};1]);
D = D + sparse((1+(i-1)*mesh.colpoints(i):i*mesh.colpoints(i))'*ones(1,mesh.colpoints(i)+1),ones(mesh.colpoints(i),1)*(1+(i-1)*mesh.colpoints(i):i*mesh.colpoints(i)+1),fD,N,N+1);

end

mesh.tau = tau;
mesh.weight = w;
setup.mesh.phase(q).weight = w;
setup.mesh.phase(q).vtau = wv;
I = sparse(sum(mesh.colpoints),sum(mesh.colpoints));
P = sparse(sum(mesh.colpoints),sum(mesh.colpoints)+1);
for w=1:numel(mesh.fraction)
    fI = inv(D(1+(w-1)*mesh.colpoints(w):w*mesh.colpoints(w),2+(w-1)*mesh.colpoints(w):w*mesh.colpoints(w)+1));
    I = I + sparse((1+(w-1)*mesh.colpoints(w):w*mesh.colpoints(w))'*ones(1,mesh.colpoints(w)),ones(mesh.colpoints(w),1)*(1+(w-1)*mesh.colpoints(w):w*mesh.colpoints(w)),fI,N,N);
    [X,Y] = meshgrid(0+(w-1)*mesh.colpoints(w):w*mesh.colpoints(w)-1,1:2);
    P = P + sparse(ones(2,1)*(1+(w-1)*mesh.colpoints(w):w*mesh.colpoints),[ones(1,mesh.colpoints(w))+(w-1)*mesh.colpoints(w);(2+(w-1)*mesh.colpoints(w):w*mesh.colpoints(w)+1)],[-ones(1,mesh.colpoints(w));ones(1,mesh.colpoints(w))],N,N+1);

end

setup.phase(q).solver.D = D;
Df{q} = D;
setup.phase(q).solver.I = I;
setup.phase(q).solver.P = P;
setup.mesh.phase(q).tau = tau;
clear w
end
% for i=1:N+1
%     syms t
%     tVj = tau(ones(N,1)*(1:N)+[zeros(N,i-1),ones(N,N+1-i)]);
%     tVi = tau(i*ones(N,N));
%     Qw = prod((t-tVj(1,:))./(tVi(1,:)-tVj(1,:)),2);
%     dQ = diff(Qw,t);
%     
%     D(:,i) = double(subs(dQ,'t',tau(1:end-1)));
% %     P = (tV-tVj)==0;
% %     D(:,i) = prod((tV-tVj)./(tVi-tVj),2).*sum(1./(tV+1e10*P-tVj),2);
% end



% for j=1:N+1
%     for i=1:N
%         D(i,j) = alternative_dl(j,tau,tau(i));
%     end
% end

end