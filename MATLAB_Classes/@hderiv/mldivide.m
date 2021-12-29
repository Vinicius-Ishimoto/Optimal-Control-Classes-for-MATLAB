function out = mldivide(ina,inb)

im = size(ina,1);
jm = size(inb,2);
km = size(ina,2);
try
    nvector = size(ina(1).value,1);
    ndiff = size(ina(1).dV,2);
catch
    nvector = size(inb(1).value,1);
    ndiff = size(inb(1).dV,2);
end
if isa(ina,'hderiv')&&isa(inb,'hderiv')
    if size(ina,1)==size(ina,2)
        out = mder_mder(ina,inb);
    end

else
    try 
        if numel(inb)==1
            for j=1:size(ina,2)
                for i=1:size(ina,1)
                 out(i,j).value = ina(i,j).value/inb;
%                  out(i,j).value = out(i,j).value(i,j);
                 out(i,j).dV = ina(i,j).dV/inb;
%                  H = hessian_str(out(i,j),inb(i,k),size(inb(i,k).dV,2));
                 out(i,j).ddV = ina(i,j).ddV/inb;
                 out(i,j).nzx = ina(i,j).nzx;
                 out(i,j).nzy = ina(i,j).nzy;
                 out(i,j).nzr = ina(i,j).nzr;
                end
            end
        else
        for j=1:size(inb,2)
            for i=1:size(ina,1)
                out(i,j).value = sparse(nvector,1);
                out(i,j).dV = sparse(nvector,ndiff);
                out(i,j).ddV = sparse(nvector,1);
                out(i,j).nzx = sparse(nvector,0);
                out(i,j).nzy = sparse(nvector,0);
                out(i,j).nzr = sparse(nvector,0);
                for k=1:size(ina,2)
                 out(i,j).value = out(i,j).value+ina(i,k).value./inb(k,j);
%                  out(i,j).value = out(i,j).value(i,j);
                 out(i,j).dV = out(i,j).dV+ina(i,k).dV./inb(k,j);
                 ina(i,k).ddV = ina(i,k).ddV/inb(k,j);
                 H = hessian_str(out(i,j),ina(i,k),size(ina(i,k).dV,2));
                 out(i,j).ddV = H.ddV;
                 out(i,j).nzx = H.nzx;
                 out(i,j).nzy = H.nzy;
                 out(i,j).nzr = H.nzr;
                end
                end
            end
        end
    catch
      if numel(ina)==1
          for j=1:size(inb,2)
              for i=1:size(inb,1)
                 out(i,j).value = ina/inb(i,j).value;
%                  out(i,j).value = out(i,j).value(i,j);
                 out(i,j).dV = -ina*inb(i,j).dV/sparse(1:nvector,1:nvector,inb(i,j).value.^2);
%                  H = hessian_str(out(i,j),inb(i,k),size(inb(i,k).dV,2));
                 jH = Hjacobian(inb(i,j).dV,inb(i,j).dV);
                 jH.ddV = 2*ina*jH.ddV/sparse(1:nvector,1:nvector,inb(i,j).value.^3);
                 inb(i,j).ddV = -ina*inb(i,j).ddV/sparse(1:nvector,1:nvector,inb(i,j).value.^2);
                 H = hessian_str(jH,inb(i,j),ndiff);
                 out(i,j).ddV = H.ddV;
                 out(i,j).nzx = H(i,j).nzx;
                 out(i,j).nzy = H(i,j).nzy;
                 out(i,j).nzr = H(i,j).nzr;
              end
          end
      else
        for j=1:size(ina,2)
            for i=1:size(inb,1)
                for k=1:size(inb,2)
                 out(i,j).value = out(i,j).value+inb(i,k).value.*ina(k,j);
%                  out(i,j).value = out(i,j).value(i,j);
                 out(i,j).dV = out(i,j).dV+inb(i,k).dV.*ina(k,j);
                 H = hessian_str(out(i,j),inb(i,k),size(inb(i,k).dV,2));
                 out(i,j).ddV = H.ddV;
                 out(i,j).nzx = H.nzx;
                 out(i,j).nzy = H.nzy;
                 out(i,j).nzr = H.nzr;
                end
                end
            end
        end
    end
end

function out = mder_mder(ina,inb)
        
U = double(ina,'matrix');
if all(isnan(diag(U)))
    iU = U;
else
    iU = inv(U);
end

ddU = ina;
V = double(inb,'matrix');
ddV = inb;
for j=1:size(ina,2)
    for i=1:im
            UdU{i,j} = iU((i-1)*nvector+1:i*nvector,:)*jacobian(ina(:,j));
            UddU(i,j).value = sparse(nvector,0);
            UddU(i,j).dV = sparse(nvector,ndiff);
            UddU(i,j).ddV = sparse(nvector,0);
            UddU(i,j).nzx = sparse(nvector,0);
            UddU(i,j).nzy = sparse(nvector,0);
            UddU(i,j).nzr = sparse(nvector,0);
            for k=1:km
                ddU(k,j).ddV = iU((i-1)*nvector+1:i*nvector,(k-1)*nvector+1:k*nvector)*ina(k,j).ddV;
                UddU(i,j) = hessian_str(UddU(i,j),ddU(k,j),ndiff);
            end
                
    end
end
for j=1:size(inb,2)
    for i=1:im
            UdV{i,j} = iU((i-1)*nvector+1:i*nvector,:)*jacobian(inb(:,j));
            UddV(i,j).value = sparse(nvector,0);
            UddV(i,j).dV = sparse(nvector,ndiff);
            UddV(i,j).ddV = sparse(nvector,0);
            UddV(i,j).nzx = sparse(nvector,0);
            UddV(i,j).nzy = sparse(nvector,0);
            UddV(i,j).nzr = sparse(nvector,0);
            for k=1:km
                ddV(k,j).ddV = iU((i-1)*nvector+1:i*nvector,(k-1)*nvector+1:k*nvector)*inb(k,j).ddV;
                UddV(i,j) = hessian_str(UddV(i,j),ddV(k,j),ndiff);
            end
                
    end
end

for j=1:size(inb,1)
    for i=1:im
        dUdU(i,j).value = sparse(nvector,0);
        dUdU(i,j).dV = sparse(nvector,ndiff);
        dUdU(i,j).ddV = sparse(nvector,0);
        dUdU(i,j).nzx = sparse(nvector,0);
        dUdU(i,j).nzy = sparse(nvector,0);
        dUdU(i,j).nzr = sparse(nvector,0);
        for k=1:km
            UdU2{i,j} = Hjacobian(UdU{i,k},UdU{k,j});
            dUdU(i,j) = hessian_str(dUdU(i,j),UdU2{i,j},ndiff);
        end
    end
end
for j=1:size(inb,2)
    for i=1:im
        dUdV(i,j).value = sparse(nvector,0);
        dUdV(i,j).dV = sparse(nvector,ndiff);
        dUdV(i,j).ddV = sparse(nvector,0);
        dUdV(i,j).nzx = sparse(nvector,0);
        dUdV(i,j).nzy = sparse(nvector,0);
        dUdV(i,j).nzr = sparse(nvector,0);
        for k=1:km
            UdUdV{i,j}= Hjacobian(UdU{i,k},UdV{k,j});
            dUdV(i,j) = hessian_str(dUdV(i,j),UdUdV{i,j},ndiff);
        end
    end
end

out = struct([]);
UV = U\V;
for j=1:jm
    for i=1:im
        out(i,j).value = diag(UV((i-1)*nvector+1:i*nvector,(j-1)*nvector+1:j*nvector));
        out(i,j).dV = UdV{i,j};
        out(i,j).ddV = sparse(nvector,0);
        out(i,j).nzx = sparse(nvector,0);
        out(i,j).nzy = sparse(nvector,0);
        out(i,j).nzr = sparse(nvector,0);
        dUdV(i,j).ddV = -2*dUdV(i,j).ddV;
        for k=1:km
            out(i,j).dV = out(i,j).dV-UV((k-1)*nvector+1:k*nvector,(j-1)*nvector+1:j*nvector)*UdU{i,k};
            dUdU(i,k).ddV = 2*UV((k-1)*nvector+1:k*nvector,(j-1)*nvector+1:j*nvector)*dUdU(i,k).ddV;
            UddU(i,k).ddV = -UV((k-1)*nvector+1:k*nvector,(j-1)*nvector+1:j*nvector)*UddU(i,k).ddV;
            Hf = hessian_str(out(i,j),dUdU(i,k),UddU(i,k),ndiff);
            out(i,j).ddV = Hf.ddV;
            out(i,j).nzx = Hf.nzx;
            out(i,j).nzy = Hf.nzy;
            out(i,j).nzr = Hf.nzr;
        end
        Hf = hessian_str(out(i,j),dUdV(i,j),UddV(i,j),ndiff);
        out(i,j).ddV = Hf.ddV;
        out(i,j).nzx = Hf.nzx;
        out(i,j).nzy = Hf.nzy;
        out(i,j).nzr = Hf.nzr;
    end
end

end
out = class(out,'hderiv');

end