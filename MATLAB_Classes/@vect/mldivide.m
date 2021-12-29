function out = mldivide(ina,inb)

if isa(ina,'vect')&&isa(inb,'vect')
    if or(numel(ina)==1,numel(inb)==1)
        out = vec_mvec(ina,inb);
    elseif size(ina,1)==size(inb,1)
        out = mvec_mvec(ina,inb);
    else
        error('Matrix dimensions must agree.')
    end
elseif isnumeric(ina)||isnumeric(inb)
    if min(size(ina)==size(inb))
        out = mnum_mvec(ina,inb);
    elseif or(numel(ina)==1,numel(inb)==1)
        out = num_mvec(ina,inb);
    else
        error('Matrix dimensions must agree.')
    end
end

function out = mvec_mvec(ina,inb)
row = size(ina,2);
col = size(inb,2);
nvl = size(ina(1).value,1);

M = double(ina,'matrix')\double(inb,'matrix');
jcon = 0;
out = struct('value',[]);
out = repmat(out,size(ina,1),size(inb,2));
for j=1:col
    icon = 0;
    for i=1:row
        out(i,j).value = diag(full(M(icon+1:icon+nvl,jcon+1:jcon+nvl)));
        icon = icon+nvl;
    end
    jcon = jcon+nvl;
end

end

function out = vec_mvec(ina,inb)

nvl = size(ina(1).value,1);
if numel(ina)==1
out = struct('value',[]);
out = repmat(out,size(inb,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(inb,1)
        out(i,j).value = ina.value.\inb(i,j).value;
    end
end
else
row = size(ina,2);
M = double(ina,'matrix')\double(inb,'matrix');
icon = 0;
for j=1:row
        out(j,1).value = diag(full(M(icon+1:icon+nvl,1:nvl)));
    icon = icon+nvl;
end
end

end

function out = mnum_mvec(ina,inb)

if isnumeric(ina)
row = size(ina,2);
col = size(inb,2);
nvl = size(inb(1).value,1);
Ma = [];
for j=1:size(ina,2)
    Mai = [];
    for i=1:size(ina,1)
        Mai = sparse([Mai;diag(ina(i,j)*ones(size(inb(1).value,1),1))]);
    end
    Ma = [Ma,Mai];
end
M = Ma\double(inb,'matrix');
jcon = 0;
out = struct('value',[]);
out = repmat(out,size(ina,1),size(inb,2));
for j=1:col
    icon = 0;
    for i=1:row
        out(i,j).value = diag(full(M(icon+1:icon+nvl,jcon+1:jcon+nvl)));
        icon = icon+nvl;
    end
    jcon = jcon+nvl;
end
else
row = size(ina,2);
col = size(inb,2);
nvl = size(ina(1).value,1);
Ma = [];
for j=1:size(inb,2)
    Mai = [];
    for i=1:size(inb,1)
        Mai = sparse([Mai;diag(inb(i,j)*ones(nvl,1))]);
    end
    Ma = [Ma,Mai];
end
M = double(ina,'matrix')\Ma;
jcon = 0;
out = struct('value',[]);
out = repmat(out,size(ina,1),size(inb,2));
for j=1:col
    icon = 0;
    for i=1:row
        out(i,j).value = diag(full(M(icon+1:icon+nvl,jcon+1:jcon+nvl)));
        icon = icon+nvl;
    end
    jcon = jcon+nvl;
end
end

end

function out = num_mvec(ina,inb)

if isnumeric(ina)&&numel(ina)==1
out = struct('value',[]);
out = repmat(out,size(inb,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(inb,1)
        out(i,j).value = ina\inb(i,j).value;
    end
end
elseif isnumeric(ina)&&numel(inb)==1
nvl = size(inb(1).value,1);
Ma = [];
for j=1:size(ina,2)
    Mai = [];
    for i=1:size(ina,1)
        Mai = sparse([Mai;diag(ina(i,j)*ones(nvl,1))]);
    end
    Ma = [Ma,Mai];
end
M = Ma\double(inb,'matrix');
icon = 0;
for j=1:size(ina,2)
        out(j,1).value = diag(full(M(icon+1:icon+nvl,1:nvl)));
    icon = icon+nvl;
end
elseif isnumeric(inb)&&numel(ina)==1
out = struct('value',[]);
out = repmat(out,size(inb,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(inb,1)
        out(i,j).value = ina.value.\inb(i,j);
    end
end
elseif isnumeric(inb)&&numel(inb)==1
nvl = size(ina(1).value,1);
Ma = [];
for j=1:size(inb,2)
    Mai = [];
    for i=1:size(inb,1)
        Mai = sparse([Mai;diag(inb(i,j)*ones(nvl,1))]);
    end
    Ma = [Ma,Mai];
end
M = double(ina,'matrix')\Ma;
icon = 0;
for j=1:size(ina,2)
        out(j,1).value = diag(full(M(icon+1:icon+nvl,1:nvl)));
    icon = icon+nvl;
end
else
    error('Undefined operation.')
end

end

out = class(out,'vect');
end