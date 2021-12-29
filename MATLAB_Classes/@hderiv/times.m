function out = times(ina,inb)

% im = size(ina,1);
% jm = size(inb,2);
% km = size(ina,2);
% try
%     nvector = size(ina(1).value,1);
%     ndiff = size(ina(1).dV,2);
% catch
%     nvector = size(inb(1).value,1);
%     ndiff = size(inb(1).dV,2);
% end
if isa(ina,'hderiv')&&isa(inb,'hderiv')
    nvector = size(ina(1).value,1);
    if numel(ina)==1
        out = der_mder(ina,inb,nvector);
    elseif numel(inb)==1
        out = der_mder(inb,ina,nvector);
    else
        out = mder_mder(ina,inb,nvector);
    end
elseif isa(inb,'hderiv')
    nvector = size(inb(1).value,1);
    if numel(ina)==1
        out = num_mder(ina,inb);
    elseif numel(inb)==1
        out = der_mnum(inb,ina);
    else
        out = mder_mnum(inb,ina,nvector);
    end
else
    nvector = size(ina(1).value,1);
    if numel(ina)==1
        out = der_mnum(ina,inb);
    elseif numel(inb)==1
        out = num_mder(inb,ina);
    else
        out = mder_mnum(ina,inb);
    end
end

out = class(out,'hderiv');

function out = mder_mder(ina,inb,nvector)

out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(inb,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(ina,1)
            out(i,j).value = ina(i,j).value.*inb(i,j).value;
            out(i,j).dV = sparse(1:nvector,1:nvector,ina(i,j).value)*inb(i,j).dV+ ...
                sparse(1:nvector,1:nvector,inb(i,j).value)*ina(i,j).dV;
            jH1 = Hjacobian(ina(i,j).dV,inb(i,j).dV);
            jH2 = Hjacobian(inb(i,j).dV,ina(i,j).dV);
            ina(i,j).ddV = sparse(1:nvector,1:nvector,inb(i,j).value)*ina(i,j).ddV;
            inb(i,j).ddV = sparse(1:nvector,1:nvector,ina(i,j).value)*inb(i,j).ddV;
            H = hessian_str(ina(i,j),inb(i,j),jH1,jH2,size(ina(i,j).dV,2));
            out(i,j).ddV = H.ddV;
            out(i,j).nzx = H.nzx;
            out(i,j).nzy = H.nzy;
            out(i,j).nzr = H.nzr;
    end
end

function out = der_mder(ina,inb,nvector)

out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(inb,1),size(inb,2));
Mina = ina;
for j=1:size(inb,2)
    for i=1:size(inb,1)
            out(i,j).value = ina.value.*inb(i,j).value;
            out(i,j).dV = sparse(1:nvector,1:nvector,ina.value)*inb(i,j).dV+ ...
                sparse(1:nvector,1:nvector,inb(i,j).value)*ina.dV;
            jH1 = Hjacobian(ina.dV,inb(i,j).dV);
            jH2 = Hjacobian(inb(i,j).dV,ina.dV);
            Mina.ddV = sparse(1:nvector,1:nvector,inb(i,j).value)*ina.ddV;
            inb(i,j).ddV = sparse(1:nvector,1:nvector,ina.value)*inb(i,j).ddV;
            H = hessian_str(Mina,inb(i,j),jH1,jH2,size(ina.dV,2));
            out(i,j).ddV = H.ddV;
            out(i,j).nzx = H.nzx;
            out(i,j).nzy = H.nzy;
            out(i,j).nzr = H.nzr;
%         end
    end
end

function out = mder_mnum(ina,inb,nvector)

out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(inb,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(ina,1)
        out(i,j).value = sparse(nvector,1);
        out(i,j).dV = sparse(nvector,size(ina(1).dV,2));
        out(i,j).ddV = sparse(nvector,1);
        out(i,j).nzx = sparse(nvector,0);
        out(i,j).nzy = sparse(nvector,0);
        out(i,j).nzr = sparse(nvector,0);
        out(i,j).value = out(i,j).value+ina(i,j).value.*inb(i,j);
        out(i,j).dV = out(i,j).dV+ina(i,j).dV.*inb(i,j);
        out(i,j).ddV = ina(i,j).ddV;
        out(i,j).nzx = ina(i,j).nzx;
        out(i,j).nzy = ina(i,j).nzy;
        out(i,j).nzr = ina(i,j).nzr;
    end
end
            
function out = num_mder(ina,inb)

out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(inb,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(inb,1)
        out(i,j).value = inb(i,j).value.*ina;
        out(i,j).dV = inb(i,j).dV.*ina;
        out(i,j).ddV = inb(i,j).ddV;
        out(i,j).nzx = inb(i,j).nzx;
        out(i,j).nzy = inb(i,j).nzy;
        out(i,j).nzr = inb(i,j).nzr;
    end
end
   
function out = der_mnum(ina,inb)

out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(inb,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(inb,1)
        out(i,j).value = ina.value.*inb(i,j);
        out(i,j).dV = ina.dV.*inb(i,j);
        out(i,j).ddV = ina.ddV.*inb(i,j);
        out(i,j).nzx = ina.nzx;
        out(i,j).nzy = ina.nzy;
        out(i,j).nzr = ina.nzr;
    end
end