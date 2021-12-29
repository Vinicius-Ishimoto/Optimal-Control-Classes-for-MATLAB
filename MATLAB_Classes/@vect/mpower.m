function out = mpower(ina,inb)

if isa(ina,'vect')&&isa(inb,'vect')
    if min(size(ina)==size(inb))
        out = mvec_mvec(ina,inb);
    elseif or(numel(ina)==1,numel(inb)==1)
        out = vec_mvec(ina,inb);
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

for j=1:size(ina,2)
    for i=1:size(ina,1)
        out(i,j).value = ina(i,j).value.^inb(i,j).value;
    end
end

end

function out = vec_mvec(ina,inb)
    
if numel(ina)==1
for j=1:size(inb,2)
    for i=1:size(inb,1)
        out(i,j).value = ina.value.^inb(i,j).value;
    end
end
else
for j=1:size(ina,2)
    for i=1:size(ina,1)
        out(i,j).value = ina(i,j).value.^inb.value;
    end
end
end

end

function out = mnum_mvec(ina,inb)

if isnumeric(ina)
for j=1:size(ina,2)
    for i=1:size(ina,1)
        out(i,j).value = ina(i,j).^inb(i,j).value;
    end
end
else
for j=1:size(ina,2)
    for i=1:size(ina,1)
        out(i,j).value = ina(i,j).value.^inb(i,j);
    end
end
end

end

function out = num_mvec(ina,inb)

if isnumeric(ina)&&numel(ina)==1
for j=1:size(inb,2)
    for i=1:size(inb,1)
        out(i,j).value = ina.^inb(i,j).value;
    end
end
elseif isnumeric(ina)&&numel(inb)==1
for j=1:size(ina,2)
    for i=1:size(ina,1)
        out(i,j).value = ina(i,j).^inb.value;
    end
end
elseif isnumeric(inb)&&numel(ina)==1
for j=1:size(inb,2)
    for i=1:size(inb,1)
        out(i,j).value = ina.value.^inb(i,j);
    end
end
elseif isnumeric(inb)&&numel(inb)==1
M = double(ina,'matrix')^inb;
row = size(ina,1);
col = size(ina,2);
nvl = size(ina(1).value,1);
jcon = 0;
for j=1:col
    icon = 0;
    for i=1:row
        out(i,j).value = full(diag(M(icon+1:icon+nvl,jcon+1:jcon+nvl)));
        icon = icon+nvl;
    end
    jcon = jcon+nvl;
end
else
    error('Undefined operation.')
end

end

out = class(out,'vect');
end