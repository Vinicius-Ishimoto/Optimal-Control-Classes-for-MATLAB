function out = inv(in)

U = [];
ndiff = size(in(1).dV,2);
nvector = size(in(1).value,1);
for j=1:size(in,2)
    dU = [];
    for i=1:size(in,1)
        dU = sparse([dU;diag(in(i,j).value)]);
    end
    U = sparse([U,dU]);
end

iU = inv(U);
ddU = in;
for j=1:size(in,2)
    for i=1:size(in,1)
            UdU{i,j} = iU((i-1)*nvector+1:i*nvector,:)*jacobian(in(:,j));
            UddU(i,j).ddV = sparse(nvector,0);
            UddU(i,j).nzx = sparse(nvector,0);
            UddU(i,j).nzy = sparse(nvector,0);
            UddU(i,j).nzr = sparse(nvector,0);
            for k=1:size(in,2)
                ddU(k,j).ddV = iU((i-1)*nvector+1:i*nvector,(k-1)*nvector+1:k*nvector)*in(k,j).ddV;
                UddU(i,j) = hessian_str(UddU(i,j),ddU(k,j),ndiff);
            end
                
    end
end

for j=1:size(in,2)
    for i=1:size(in,1)
        dUdU(i,j).ddV = sparse(nvector,0);
        dUdU(i,j).nzx = sparse(nvector,0);
        dUdU(i,j).nzy = sparse(nvector,0);
        dUdU(i,j).nzr = sparse(nvector,0);
        for k=1:size(in,2)
            UdU2{i,j} = Hjacobian(UdU{i,k},UdU{k,j});
            dUdU(i,j) = hessian_str(dUdU(i,j),UdU2{i,j},ndiff);
        end
    end
end
dUdUt = dUdU;
UddUt = UddU;
for j=1:size(in,2)
    for i=1:size(in,1)
        out(i,j).value = diag(iU((i-1)*nvector+1:i*nvector,(j-1)*nvector+1:j*nvector));
        out(i,j).dV = sparse(nvector,ndiff);
        out(i,j).ddV = sparse(nvector,0);
        out(i,j).nzx = sparse(nvector,0);
        out(i,j).nzy = sparse(nvector,0);
        out(i,j).nzr = sparse(nvector,0);
        for k=1:size(in,2)
            out(i,j).dV = out(i,j).dV-iU((k-1)*nvector+1:k*nvector,(j-1)*nvector+1:j*nvector)*UdU{i,k};
            dUdUt(i,k).ddV = iU((k-1)*nvector+1:k*nvector,(j-1)*nvector+1:j*nvector)*dUdU(i,k).ddV;
            UddUt(i,k).ddV = -iU((k-1)*nvector+1:k*nvector,(j-1)*nvector+1:j*nvector)*UddU(i,k).ddV;
            Hf = hessian_str(out(i,j),dUdUt(i,k),dUdUt(i,k),UddUt(i,k),ndiff);
            out(i,j).ddV = Hf.ddV;
            out(i,j).nzx = Hf.nzx;
            out(i,j).nzy = Hf.nzy;
            out(i,j).nzr = Hf.nzr;
        end
%         Hf = hessian_str(out(i,j),dUdV(i,j),UddV(i,j),ndiff);
%         out(i,j).ddV = Hf.ddV;
%         out(i,j).nzx = Hf.nzx;
%         out(i,j).nzy = Hf.nzy;
%         out(i,j).nzr = Hf.nzr;
    end
end

out = class(out,'hderiv');

end