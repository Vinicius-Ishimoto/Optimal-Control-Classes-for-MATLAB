function out = mrdivide(ina,inb)

% im = size(ina,1);
% jm = size(inb,2);
% km = size(ina,2);
try
    nvector = size(ina(1).value,1);
%     ndiff = size(ina(1).dV,2);
catch
    nvector = size(inb(1).value,1);
%     ndiff = size(inb(1).dV,2);
end
if isa(ina,'hderiv')&&isa(inb,'hderiv')
    if numel(inb)==1
        out = mder_der(ina,inb,nvector);
    else
        out = mder_mder(ina,inb,nvector);
    end
elseif isa(ina,'hderiv')
    if numel(inb)==1
        out = mder_num(ina,inb,nvector);
    else
        out = mder_mnum(ina,inb,nvector);
    end
else
    if numel(inb)==1
        out = mvar_der(ina,inb,nvector);
    end
end

out = class(out,'hderiv');

function out = mder_mder(ina,inb,nvector)

U = double(ina,'matrix');
V = double(inb,'matrix');
iV = inv(V);
ndiff = size(ina(1).dV,2);

ddU = ina;
ddV = inb;

dVV = cell(size(inb,1),size(inb,2));
ddVV = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
ddVV = repmat(ddVV,size(inb,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(inb,1)
%             dVV{i,j} = iV(:,(j-1)*nvector+1:j*nvector)*jacobian(inb(i,:));
            ddVV(i,j).ddV = sparse(nvector,0);
            ddVV(i,j).nzx = sparse(nvector,0);
            ddVV(i,j).nzy = sparse(nvector,0);
            ddVV(i,j).nzr = sparse(nvector,0);
            dVV{i,j} = sparse(nvector,ndiff);
            for k=1:size(inb,2)
                dVV{i,j} = dVV{i,j}+iV((k-1)*nvector+1:k*nvector,(j-1)*nvector+1:j*nvector)*inb(i,k).dV;
                ddV(i,k).ddV = iV((k-1)*nvector+1:k*nvector,(j-1)*nvector+1:j*nvector)*inb(i,k).ddV;
                ddVV(i,j) = hessian_str(ddVV(i,j),ddV(i,k),ndiff);
            end
    end
end
dUV = cell(size(ina,1),size(inb,2));
ddUV = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
ddUV = repmat(ddUV,size(ina,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(ina,1)
%             dUV{i,j} = iV(:,(j-1)*nvector+1:j*nvector)*jacobian(inb(i,:));
            ddUV(i,j).ddV = sparse(nvector,0);
            ddUV(i,j).nzx = sparse(nvector,0);
            ddUV(i,j).nzy = sparse(nvector,0);
            ddUV(i,j).nzr = sparse(nvector,0);
            dUV{i,j} = sparse(nvector,ndiff);
            for k=1:size(ina,2)
                dUV{i,j} = dUV{i,j}+iV((k-1)*nvector+1:k*nvector,(j-1)*nvector+1:j*nvector)*ina(i,k).dV;
                ddU(i,k).ddV = iV((k-1)*nvector+1:k*nvector,(j-1)*nvector+1:j*nvector)*ina(i,k).ddV;
                ddUV(i,j) = hessian_str(ddUV(i,j),ddU(i,k),ndiff);
            end
    end
end
dVV2 = cell(size(inb,1),size(inb,2));
dVdV = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
dVdV = repmat(dVdV,size(inb,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(inb,1)
        dVdV(i,j).ddV = sparse(nvector,0);
        dVdV(i,j).nzx = sparse(nvector,0);
        dVdV(i,j).nzy = sparse(nvector,0);
        dVdV(i,j).nzr = sparse(nvector,0);
        for k=1:size(ina,2)
            dVV2{i,j} = Hjacobian(dVV{i,k},dVV{k,j});
            dVdV(i,j) = hessian_str(dVdV(i,j),dVV2{i,j},ndiff);
        end
    end
end
dUdVV = cell(size(ina,1),size(inb,2));
dUdV = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
dUdV = repmat(dUdV,size(ina,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(ina,1)
        dUdV(i,j).ddV = sparse(nvector,0);
        dUdV(i,j).nzx = sparse(nvector,0);
        dUdV(i,j).nzy = sparse(nvector,0);
        dUdV(i,j).nzr = sparse(nvector,0);
        for k=1:size(ina,2)
            dUdVV{i,j}= Hjacobian(dUV{i,k},dVV{k,j});
            dUdV(i,j) = hessian_str(dUdV(i,j),dUdVV{i,j},ndiff);
        end
    end
end

UV = U/V;
out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(ina,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(ina,1)
        out(i,j).value = diag(UV((i-1)*nvector+1:i*nvector,(j-1)*nvector+1:j*nvector));
        out(i,j).dV = dUV{i,j};
        out(i,j).ddV = sparse(nvector,0);
        out(i,j).nzx = sparse(nvector,0);
        out(i,j).nzy = sparse(nvector,0);
        out(i,j).nzr = sparse(nvector,0);
        dUdV(i,j).ddV = -2*dUdV(i,j).ddV;
        for k=1:size(ina,2)
            out(i,j).dV = out(i,j).dV-UV((i-1)*nvector+1:i*nvector,(k-1)*nvector+1:k*nvector)*dVV{k,j};
            dVdV(k,j).ddV = 2*UV((i-1)*nvector+1:i*nvector,(k-1)*nvector+1:k*nvector)*dVdV(k,j).ddV;
            ddVV(k,j).ddV = -UV((i-1)*nvector+1:i*nvector,(k-1)*nvector+1:k*nvector)*ddVV(k,j).ddV;
            Hf = hessian_str(out(i,j),dVdV(k,j),ddVV(k,j),ndiff);
            out(i,j).ddV = Hf.ddV;
            out(i,j).nzx = Hf.nzx;
            out(i,j).nzy = Hf.nzy;
            out(i,j).nzr = Hf.nzr;
        end
        Hf = hessian_str(out(i,j),dUdV(i,j),ddUV(i,j),ndiff);
        out(i,j).ddV = Hf.ddV;
        out(i,j).nzx = Hf.nzx;
        out(i,j).nzy = Hf.nzy;
        out(i,j).nzr = Hf.nzr;
    end
end

function out = mder_der(ina,inb,nvector)

out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(ina,1),size(ina,2));
for j=1:size(ina,2)
    for i=1:size(ina,1)
            out(i,j).value = ina(i,j).value./inb.value;
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
            out(i,j).ddV = Hf.ddV;
            out(i,j).nzx = Hf.nzx;
            out(i,j).nzy = Hf.nzy;
            out(i,j).nzr = Hf.nzr;
    end
end

function out = mder_mnum(ina,inb,nvector)

% U = double(ina,'matrix');
% V = double(inb,'matrix');
iV = inv(V);
% UV = U/V;
Mina = ina;
out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(ina,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(ina,1)
        out(i,j).value = sparse(nvector,0);
        out(i,j).dV = sparse(nvector,size(ina(1).dV,2));
        out(i,j).ddV = sparse(nvector,0);
        out(i,j).nzx = sparse(nvector,0);
        out(i,j).nzy = sparse(nvector,0);
        out(i,j).nzr = sparse(nvector,0);
        for k=1:size(ina,2)
            out(i,j).value = out(i,j).value+ina(i,k).value*iV(k,j);
            out(i,j).dV = out(i,j).dV+iV(k,j)*ina(i,k).dV;
            Mina(i,k).ddV = iV(k,j)*ina(i,k).ddV;
            Hf = hessian_str(out(i,j),Mina(i,k),size(ina(1).dV,2));
        end
        out(i,j).ddV = Hf.ddV;
        out(i,j).nzx = Hf.nzx;
        out(i,j).nzy = Hf.nzy;
        out(i,j).nzr = Hf.nzr;
    end
end

function out = mder_num(ina,inb,nvector)

out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(ina,1),size(ina,2));
for j=1:size(ina,2)
    for i=1:size(ina,1)
            out(i,j).value = ina(i,j).value./inb;
            out(i,j).dV = sparse(1:nvector,1:nvector,inb)\ina(i,j).dV;
            out(i,j).ddV = sparse(1:nvector,1:nvector,inb)\ina(i,j).ddV;
%             out(i,j).ddV = Hf.ddV;
            out(i,j).nzx = ina.nzx;
            out(i,j).nzy = ina.nzy;
            out(i,j).nzr = ina.nzr;
    end
end

function out = mvar_der(ina,inb,nvector)
out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(ina,1),size(ina,2));
Minb = inb;
for j=1:size(ina,2)
    for i=1:size(ina,1)
        out(i,j).value = ina(i,j)./inb.value;
        out(i,j).dV = -sparse(1:nvector,1:nvector,inb.value.^2)\ ...
                sparse(1:nvector,1:nvector,ina(i,j))*inb.dV;
        jH3 = Hjacobian(inb.dV,inb.dV);
        jH3.ddV = sparse(1:nvector,1:nvector,inb.value.^3)\sparse(1:nvector,1:nvector,2*ina(i,j))*jH3.ddV;
        Minb.ddV = -sparse(1:nvector,1:nvector,inb.value.^2)\sparse(1:nvector,1:nvector,ina(i,j))*inb.ddV;
        Hf = hessian_str(Minb,jH3,size(inb.dV,2));
        out(i,j).ddV = Hf.ddV;
        out(i,j).nzx = Hf.nzx;
        out(i,j).nzy = Hf.nzy;
        out(i,j).nzr = Hf.nzr;
    end
end