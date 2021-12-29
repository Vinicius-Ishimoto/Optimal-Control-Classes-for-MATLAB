function out = tanh(in)

for j=1:size(in,2)
    for i=1:size(in,1)
        out(i,j).value = tanh(in(i,j).value);
    end
end

out = class(out,'vect');

end