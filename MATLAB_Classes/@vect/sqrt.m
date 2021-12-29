function out = sqrt(ina)


out = struct('value',[]);
out = repmat(out,size(ina,1),size(ina,2));
for j=1:size(ina,2)
    for i=1:size(ina,1)
        out(i,j).value = sqrt(ina(i,j).value);
    end
end

out = class(out,'vect');
end