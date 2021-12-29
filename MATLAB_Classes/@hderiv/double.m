function out = double(in,type)

if nargin==1
nvector = size(in(1).value,1);
% j = size(in,2);
% out = sparse(size(in,1),size(in,2));
for i=1:size(in,1)
    for j=1:size(in,2)
         out(1+(i-1)*nvector:i*nvector,j) = in(i,j).value;
    end
end
elseif strcmp(type,'matrix')
    out = matrix(in);
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