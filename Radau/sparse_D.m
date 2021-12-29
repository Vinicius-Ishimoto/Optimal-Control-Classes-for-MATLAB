function fD = sparse_D(N,tau)

Neye1 = triu(ones(N,N-1),0);
fD = sparse(N,N+1);
for j=1:N+1
    if j==1
        Var = N-1;
        Neye2 = triu(ones(N,N-1),0);
    else
        Var = N-j+1;
    Neye2 = triu(ones(N,N-1),0)+tril([zeros(N,j-2),-ones(N,(N+1)~=j),zeros(N,N-j)],-1);
    end
    for i=1:N
        tVm = tau(ones(N,1)*(1:N-1)+[zeros(N,j-2),ones(N,Var)]+[Neye1(1:j-1,:);Neye2(j:N,:)]);
        tVl = tau((1:N)'+[zeros(j-1,1);ones(N-j+1,1)]);
        
        fD(i,j) = sum((1./(tau(j)*ones(N,1)-tVl)).*prod((tau(i)*ones(N,N-1)-tVm)./(tau(j)*ones(N,N-1)-tVm),2),1);
    end
end

end