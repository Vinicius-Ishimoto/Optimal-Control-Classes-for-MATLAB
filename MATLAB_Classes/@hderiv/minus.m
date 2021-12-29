function out = minus(ina,inb)

if isa(ina,'hderiv') && isa(inb,'hderiv')
    out = mder_mder(ina,inb);
elseif isa(ina,'hderiv')
    out = mder_mvar(ina,inb);
else
    out = mvar_mder(ina,inb);
end
out = class(out,'hderiv');

function out = mder_mder(ina,inb)

out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(ina,1),size(ina,2));
for j=1:size(ina,2)
    for i=1:size(ina,1)
        out(i,j).value = ina(i,j).value-inb(i,j).value;
        out(i,j).dV = ina(i,j).dV-inb(i,j).dV;
        inb(i,j).ddV = -inb(i,j).ddV;
        H = hessian_str(ina(i,j),inb(i,j),size(ina(i,j).dV,2));
% H = hessian_str(ina(i,j),inb(i,j));
        out(i,j).ddV = H.ddV;
        out(i,j).nzx = H.nzx;
        out(i,j).nzy = H.nzy;
        out(i,j).nzr = H.nzr;
    end
end

function out = mder_mvar(ina,inb)

out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(ina,1),size(ina,2));
for j=1:size(ina,2)
        for i=1:size(ina,1)
            out(i,j).value = ina(i,j).value-inb(i,j);
            out(i,j).dV = ina(i,j).dV;
            out(i,j).ddV = ina(i,j).ddV;
            out(i,j).nzx = ina(i,j).nzx;
            out(i,j).nzy = ina(i,j).nzy;
            out(i,j).nzr = ina(i,j).nzr;
        end
end

function out = mvar_mder(ina,inb)

out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(inb,1),size(inb,2));
    for j=1:size(inb,2)
        for i=1:size(inb,1)
            out(i,j).value = -inb(i,j).value+ina(i,j);
            out(i,j).dV = -inb(i,j).dV;
            out(i,j).ddV = -inb(i,j).ddV;
            out(i,j).nzx = inb(i,j).nzx;
            out(i,j).nzy = inb(i,j).nzy;
            out(i,j).nzr = inb(i,j).nzr;
        end
    end