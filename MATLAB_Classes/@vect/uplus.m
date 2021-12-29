function out = uplus(in)

out = struct('value',[]);
out = repmat(out,size(in,1),size(in,2));
for j=1:size(in,2)
    for i=1:size(in,1)
        out(i,j).value = uplus(in(i,j).value);
    end
end

out = class(out,'vect');

end