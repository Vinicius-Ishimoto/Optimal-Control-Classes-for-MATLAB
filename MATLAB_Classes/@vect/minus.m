function out = minus(ina,inb)

if isa(ina,'vect')&&isa(inb,'vect')
    if min(size(ina)==size(inb))
        out = mvec_mvec(ina,inb);
    elseif or(numel(ina)==1,numel(inb)==1)
        out = vec_mvec(ina,inb);
    else
        error('Matrix dimensions must agree.')
    end
else
    if isnumeric(ina)||isnumeric(inb)
        if numel(inb)==1&&numel(ina)~=1
            aux = inb;
            inb = ina;
            ina = aux;
            flag = 1;
        else
            flag = 0;
        end
    if min(size(ina)==size(inb))
        out = mnum_mvec(ina,inb,flag);
    elseif or(numel(ina)==1,numel(inb)==1)
        out = num_mvec(ina,inb,flag);
    else
        error('Matrix dimensions must agree.')
    end
    end
end

function out = mvec_mvec(ina,inb)

out = struct('value',[]);
out = repmat(out,size(ina,1),size(ina,2));
for j=1:size(ina,2)
    for i=1:size(ina,1)
        out(i,j).value = ina(i,j).value-inb(i,j).value;
    end
end

end

function out = vec_mvec(ina,inb)

if numel(ina)==1
    out = struct('value',[]);
out = repmat(out,size(inb,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(inb,1)
        out(i,j).value = ina.value-inb(i,j).value;
    end
end
elseif numel(inb)==1
    out = struct('value',[]);
out = repmat(out,size(ina,1),size(ina,2));
for j=1:size(ina,2)
    for i=1:size(ina,1)
        out(i,j).value = ina(i,j).value-inb.value;
    end
end
end

end

function out = mnum_mvec(ina,inb,flag)
out = struct('value',[]);
out = repmat(out,size(inb,1),size(inb,2));
for j=1:size(ina,2)
    for i=1:size(ina,1)
        out(i,j).value = double(ina(i,j))-double(inb(i,j));
    end
end

end

function out = num_mvec(ina,inb,flag)
out = struct('value',[]);
out = repmat(out,size(inb,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(inb,1)
        out(i,j).value = double(ina)-double(inb(i,j));
        if flag
            out(i,j).value=-out(i,j).value;
        end
    end
end

end

out = class(out,'vect');
end