function out = rdivide(ina,inb)

warning('off','MATLAB:nearlySingularMatrix')
try
    nvector = size(ina(1).value,1);
%     ndiff = size(ina(1).dV,2);
catch
    nvector = size(inb(1).value,1);
%     ndiff = size(inb(1).dV,2);
end

if isa(ina,'hderiv') && isa(inb,'hderiv')
    out = mder_mder(ina,inb,nvector);
elseif isnumeric(ina)
    out = mvar_mder(ina,inb,nvector);
elseif isnumeric(inb)
    out = mder_mvar(ina,inb,nvector);
end

out = class(out,'hderiv');
warning('on','MATLAB:nearlySingularMatrix')

function out = mvar_mder(ina,inb,nvector)
out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(ina,1),size(ina,2));
for j=1:size(ina,2)
    for i=1:size(ina,1)
        out(i,j).value = ina(i,j)./inb(i,j).value;
        out(i,j).dV = -sparse(1:nvector,1:nvector,inb(i,j).value.^2)\ ...
                sparse(1:nvector,1:nvector,ina(i,j))*inb(i,j).dV;
        jH3 = Hjacobian(inb(i,j).dV,inb(i,j).dV);
        jH3.ddV = sparse(1:nvector,1:nvector,inb(i,j).value.^3)\sparse(1:nvector,1:nvector,2*ina(i,j))*jH3.ddV;
        inb(i,j).ddV = -sparse(1:nvector,1:nvector,inb(i,j).value.^2)\sparse(1:nvector,1:nvector,ina(i,j))*inb(i,j).ddV;
        Hf = hessian_str(inb(i,j),jH3,size(inb(i,j).dV,2));
        out(i,j).ddV = Hf.ddV;
        out(i,j).nzx = Hf.nzx;
        out(i,j).nzy = Hf.nzy;
        out(i,j).nzr = Hf.nzr;
    end
end

function out = mder_mvar(ina,inb,nvector)
out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(ina,1),size(ina,2));
for j=1:size(ina,2)
    for i=1:size(ina,1)
        out(i,j).value = ina(i,j).value./inb(i,j);
        out(i,j).dV = sparse(1:nvector,1:nvector,inb(i,j))\ina(i,j).dV;
        out(i,j).ddV = sparse(1:nvector,1:nvector,inb(i,j))\ina(i,j).ddV;
        out(i,j).nzx = ina(i,j).nzx;
        out(i,j).nzy = ina(i,j).nzy;
        out(i,j).nzr = ina(i,j).nzr;
    end
end

function out = mder_mder(ina,inb,nvector)
out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(ina,1),size(ina,2));
for j=1:size(ina,2)
    for i=1:size(ina,1)
        out(i,j).value = ina(i,j).value./inb(i,j).value;
        out(i,j).dV = sparse(1:nvector,1:nvector,inb(i,j).value)\ina(i,j).dV-sparse(1:nvector,1:nvector,inb(i,j).value.^2)\ ...
                sparse(1:nvector,1:nvector,ina(i,j).value)*inb(i,j).dV;
            jH1 = Hjacobian(ina(i,j).dV,inb(i,j).dV);
            jH1.ddV = -sparse(1:nvector,1:nvector,inb(i,j).value.^2)\jH1.ddV;
            jH2 = Hjacobian(inb(i,j).dV,ina(i,j).dV);
            jH2.ddV = -sparse(1:nvector,1:nvector,inb(i,j).value.^2)\jH2.ddV;
            jH3 = Hjacobian(inb(i,j).dV,inb(i,j).dV);
            jH3.ddV = sparse(1:nvector,1:nvector,inb(i,j).value.^3)\sparse(1:nvector,1:nvector,2*ina(i,j).value)*jH3.ddV;
            ina(i,j).ddV = sparse(1:nvector,1:nvector,inb(i,j).value)\ina(i,j).ddV;
            inb(i,j).ddV = -sparse(1:nvector,1:nvector,inb(i,j).value.^2)\sparse(1:nvector,1:nvector,ina(i,j).value)*inb(i,j).ddV;
            Hf = hessian_str(ina(i,j),inb(i,j),jH1,jH2,jH3,size(ina(i,j).dV,2));
%             out(i,j).ddV = tril(out(i,j).ddV+ina(i,k).value*inb(k,j).ddV+ina(i,k).ddV*inb(k,j).value+ina(i,k).dV'*inb(k,j).dV+inb(k,j).dV'*ina(i,k).dV);
%             Hf = hessian_str(out(i,j),H,size(ina(i,j).dV,2));
            out(i,j).ddV = Hf.ddV;
            out(i,j).nzx = Hf.nzx;
            out(i,j).nzy = Hf.nzy;
            out(i,j).nzr = Hf.nzr;
    end
end


