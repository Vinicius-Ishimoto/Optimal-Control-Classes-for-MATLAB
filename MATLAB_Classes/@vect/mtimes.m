function out = mtimes(ina,inb)

if isa(ina,'vect')&&isa(inb,'vect')
    if size(ina,2)==size(inb,1)
        out = mvec_mvec(ina,inb);
    elseif or(numel(ina)==1,numel(inb)==1)
        out = vec_mvec(ina,inb);
    else
        error('Matrix dimensions must agree.')
    end
else
    if isnumeric(ina)||isnumeric(inb)
    if size(ina,2)==size(inb,1)
        out = mnum_mvec(ina,inb);
    elseif numel(ina)==1
        out = num_mvec(ina,inb);
    elseif numel(inb)==1
        out = num_mvec(inb,ina);
    else
        error('Matrix dimensions must agree.')
    end
    end
end

function out = mvec_mvec(ina,inb)
out = struct('value',[]);
out = repmat(out,size(ina,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(ina,1)
        out(i,j).value = sparse(size(ina(1).value,1),1);
        for k =1:size(inb,1)
            out(i,j).value = +out(i,j).value+ina(i,k).value.*inb(k,j).value;
        end
    end
end

end

function out = vec_mvec(ina,inb)
    
if numel(inb)==1&&numel(ina)~=1
    aux = inb;
    inb = ina;
    ina = aux;
end
out = struct('value',[]);
out = repmat(out,size(inb,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(inb,1)
        out(i,j).value = ina.value.*inb(i,j).value;
    end
end

end

function out = mnum_mvec(ina,inb)
out = struct('value',[]);
out = repmat(out,size(ina,1),size(inb,2));
if isnumeric(ina)
for j=1:size(inb,2)
    for i=1:size(ina,1)
        out(i,j).value = sparse(size(inb(1).value,1),1);
        for k=1:size(inb,1)
            out(i,j).value = out(i,j).value+ina(i,k)*inb(k,j).value;
        end
    end
end
else
for j=1:size(inb,2)
    for i=1:size(ina,1)
        out(i,j).value = sparse(size(ina(1).value,1),1);
        for k=1:size(inb,1)
            out(i,j).value = out(i,j).value+ina(i,k).value*inb(k,j);
        end
    end
end
end

end

function out = num_mvec(ina,inb)
out = struct('value',[]);
out = repmat(out,size(inb,1),size(inb,2));
for j=1:size(inb,2)
    for i=1:size(inb,1)
        out(i,j).value = double(ina)*double(inb(i,j));
    end
end

end

out = class(out,'vect');
end