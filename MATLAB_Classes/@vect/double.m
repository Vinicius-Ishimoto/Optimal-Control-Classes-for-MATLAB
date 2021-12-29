function out = double(in,type)

if nargin==1
if size(in,1)==1
    out = column(in);
elseif size(in,2)==1
    out = row(in);
else
    out = matrix(in);
end
else
if strcmp(type,'matrix')
    out = matrix(in);
end
end


function out = column(in)

for i=1:size(in,2)
    out(:,i) = in(1,i).value;
end

end

function out = row(in)

out = [];
for i=1:size(in,1)
    out = [out;in(i,1).value];
end

end

function out = matrix(in)
out = [];
for j=1:size(in,2)
    outv = [];
    for i=1:size(in,1)
        outv = sparse([outv;diag(in(i,j).value)]);
    end
    out = sparse([out,outv]);
end

end


end