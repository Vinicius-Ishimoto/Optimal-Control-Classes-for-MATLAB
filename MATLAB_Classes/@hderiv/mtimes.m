function out = mtimes(ina,inb)

% im = size(ina,1);
% jm = size(inb,2);
% km = size(ina,2);

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
        out = mnum_mder(ina,inb,nvector);
    end
else
    nvector = size(ina(1).value,1);
    if numel(ina)==1
        out = der_mnum(ina,inb);
    elseif numel(inb)==1
        out = num_mder(inb,ina);
    else
        out = mder_mnum(ina,inb,nvector);
    end
end

out = class(out,'hderiv');


function out = mder_mder(ina,inb,nvector)
    
out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(ina,1),size(inb,2));
Mina = ina;
Minb = inb;
for j=1:size(inb,2)
    for i=1:size(ina,1)
        out(i,j).value = sparse(nvector,1);
        out(i,j).dV = sparse(nvector,size(ina(1).dV,2));
        out(i,j).ddV = sparse(nvector,0);
        out(i,j).nzx = sparse(nvector,0);
        out(i,j).nzy = sparse(nvector,0);
        out(i,j).nzr = sparse(nvector,0);
        for k=1:size(ina,2)
            out(i,j).value = out(i,j).value+ina(i,k).value.*inb(k,j).value;
            out(i,j).dV = out(i,j).dV+sparse(1:nvector,1:nvector,ina(i,k).value)*inb(k,j).dV+ ...
                sparse(1:nvector,1:nvector,inb(k,j).value)*ina(i,k).dV;
            jH1 = Hjacobian(ina(i,k).dV,inb(k,j).dV);
            jH1.ddV = 2*jH1.ddV;
            Mina(i,k).ddV = sparse(1:nvector,1:nvector,inb(k,j).value)*ina(i,k).ddV;
            Minb(k,j).ddV = sparse(1:nvector,1:nvector,ina(i,k).value)*inb(k,j).ddV;
            H = hessian_str(Mina(i,k),Minb(k,j),jH1,size(ina(i,k).dV,2));
%             out(i,j).ddV = tril(out(i,j).ddV+ina(i,k).value*inb(k,j).ddV+ina(i,k).ddV*inb(k,j).value+ina(i,k).dV'*inb(k,j).dV+inb(k,j).dV'*ina(i,k).dV);
            Hf = hessian_str(out(i,j),H,size(ina(i,k).dV,2));
            out(i,j).ddV = Hf.ddV;
            out(i,j).nzx = Hf.nzx;
            out(i,j).nzy = Hf.nzy;
            out(i,j).nzr = Hf.nzr;
        end
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
    end
end


function out = num_mder(ina,inb)

out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(inb,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(inb,1)
        out(i,j).value = inb(i,j).value.*ina;
        out(i,j).dV = inb(i,j).dV.*ina;
        out(i,j).ddV = inb(i,j).ddV.*ina;
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


function out = mnum_mder(ina,inb,nvector)

out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(ina,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(ina,1)
        out(i,j).value = sparse(nvector,1);
        out(i,j).dV = sparse(nvector,size(inb(1).dV,2));
        out(i,j).ddV = sparse(nvector,0);
        out(i,j).nzx = sparse(nvector,0);
        out(i,j).nzy = sparse(nvector,0);
        out(i,j).nzr = sparse(nvector,0);
        for k=1:size(inb,1)
            out(i,j).value = out(i,j).value+inb(k,j).value.*ina(i,k);
            out(i,j).dV = out(i,j).dV+inb(k,j).dV.*ina(i,k);
            H = hessian_str(out(i,j),inb(k,j),size(inb(k,j).dV,2));
            out(i,j).ddV = H.ddV;
            out(i,j).nzx = H.nzx;
            out(i,j).nzy = H.nzy;
            out(i,j).nzr = H.nzr;
        end
    end
end     


function out = mder_mnum(ina,inb,nvector)

out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(ina,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(ina,1)
        out(i,j).value = sparse(nvector,1);
        out(i,j).dV = sparse(nvector,size(ina(1).dV,2));
        out(i,j).ddV = sparse(nvector,0);
        out(i,j).nzx = sparse(nvector,0);
        out(i,j).nzy = sparse(nvector,0);
        out(i,j).nzr = sparse(nvector,0);
        for k=1:size(ina,2)
            out(i,j).value = out(i,j).value+ina(i,k).value.*inb(k,j);
            out(i,j).dV = out(i,j).dV+ina(i,k).dV.*inb(k,j);
            H = hessian_str(out(i,j),ina(i,k),size(ina(i,k).dV,2));
            out(i,j).ddV = H.ddV;
            out(i,j).nzx = H.nzx;
            out(i,j).nzy = H.nzy;
            out(i,j).nzr = H.nzr;
        end
    end
end

