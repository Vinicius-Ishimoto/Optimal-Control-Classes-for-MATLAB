function out = power(ina,inb)

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
out = struct([]);
for j=1:jm
    for i=1:im
        out(i,j).value = sparse(nvector,1);
        out(i,j).dV = sparse(nvector,size(ina(1).dV,2));
        out(i,j).ddV = sparse(nvector,1);
        out(i,j).nzx = sparse(nvector,0);
        out(i,j).nzy = sparse(nvector,0);
        out(i,j).nzr = sparse(nvector,0);
        for k=1:km
            out(i,j).value = out(i,j).value+ina(i,k).value.*inb(k,j).value;
            out(i,j).dV = out(i,j).dV+sparse(1:nvector,1:nvector,ina(i,k).value)*inb(k,j).dV+ ...
                sparse(1:nvector,1:nvector,inb(k,j).value)*ina(i,k).dV;
            jH1 = Hjacobian(ina(i,k).dV,inb(k,j).dV);
            jH2 = Hjacobian(inb(k,j).dV,ina(i,k).dV);
            ina(i,k).ddV = sparse(1:nvector,1:nvector,inb(k,j).value)*ina(i,k).ddV;
            inb(k,j).ddV = sparse(1:nvector,1:nvector,ina(i,k).value)*inb(k,j).ddV;
            H = hessian_str(ina(i,k),inb(k,j),jH1,jH2,size(ina(i,k).dV,2));
%             out(i,j).ddV = tril(out(i,j).ddV+ina(i,k).value*inb(k,j).ddV+ina(i,k).ddV*inb(k,j).value+ina(i,k).dV'*inb(k,j).dV+inb(k,j).dV'*ina(i,k).dV);
            Hf = hessian_str(out(i,j),H,size(ina(i,k).dV,2));
            out(i,j).ddV = Hf.ddV;
            out(i,j).nzx = Hf.nzx;
            out(i,j).nzy = Hf.nzy;
            out(i,j).nzr = Hf.nzr;
        end
    end
end
else
    out = mder_var(ina,inb,nvector);
%     try 
%         if numel(inb)==1
%             
%         else
%         for j=1:size(inb,2)
%             for i=1:size(ina,1)
%                 out(i,j).value = sparse(nvector,1);
%                 out(i,j).dV = sparse(nvector,ndiff);
%                 out(i,j).ddV = sparse(nvector,1);
%                 out(i,j).nzx = sparse(nvector,0);
%                 out(i,j).nzy = sparse(nvector,0);
%                 out(i,j).nzr = sparse(nvector,0);
%                 for k=1:size(ina,2)
%                  out(i,j).value = out(i,j).value+ina(i,k).value.*inb(k,j);
% %                  out(i,j).value = out(i,j).value(i,j);
%                  out(i,j).dV = out(i,j).dV+ina(i,k).dV.*inb(k,j);
%                  H = hessian_str(out(i,j),ina(i,k),size(ina(i,k).dV,2));
%                  out(i,j).ddV = H.ddV;
%                  out(i,j).nzx = H.nzx;
%                  out(i,j).nzy = H.nzy;
%                  out(i,j).nzr = H.nzr;
%                 end
%                 end
%             end
%         end
%     catch
%       if numel(ina)==1
%           for j=1:size(inb,2)
%               for i=1:size(inb,1)
%                  out(i,j).value = inb(i,j).value.*ina;
% %                  out(i,j).value = out(i,j).value(i,j);
%                  out(i,j).dV = inb(i,j).dV.*ina;
% %                  H = hessian_str(out(i,j),inb(i,k),size(inb(i,k).dV,2));
%                  out(i,j).ddV = inb(i,j).ddV;
%                  out(i,j).nzx = inb(i,j).nzx;
%                  out(i,j).nzy = inb(i,j).nzy;
%                  out(i,j).nzr = inb(i,j).nzr;
%               end
%           end
%       else
%         for j=1:size(ina,2)
%             for i=1:size(inb,1)
%                 for k=1:size(inb,2)
%                  out(i,j).value = out(i,j).value+inb(i,k).value.*ina(k,j);
% %                  out(i,j).value = out(i,j).value(i,j);
%                  out(i,j).dV = out(i,j).dV+inb(i,k).dV.*ina(k,j);
%                  H = hessian_str(out(i,j),inb(i,k),size(inb(i,k).dV,2));
%                  out(i,j).ddV = H.ddV;
%                  out(i,j).nzx = H.nzx;
%                  out(i,j).nzy = H.nzy;
%                  out(i,j).nzr = H.nzr;
%                 end
%                 end
%             end
%         end
%     end
end

out = class(out,'hderiv');

function out = mder_var(ina,inb,nvector)

out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(ina,1),size(ina,2));
for j=1:size(ina,2)
                for i=1:size(ina,1)
                 out(i,j).value = ina(i,j).value.^inb;
                 out(i,j).dV = inb*sparse(1:nvector,1:nvector,ina(i,j).value.^(inb-1))*ina(i,j).dV;
                 jH = Hjacobian(ina(i,j).dV,ina(i,j).dV);
                 jH.ddV = inb*(inb-1)*sparse(1:nvector,1:nvector,ina(i,j).value.^(inb-2))*jH.ddV;
                 ina(i,j).ddV = inb*sparse(1:nvector,1:nvector,ina(i,j).value.^(inb-1))*ina(i,j).ddV;
                 H = hessian_str(ina(i,j),jH,size(ina(1).dV,2));
                 out(i,j).ddV = H.ddV;
                 out(i,j).nzx = H.nzx;
                 out(i,j).nzy = H.nzy;
                 out(i,j).nzr = H.nzr;
                end
end