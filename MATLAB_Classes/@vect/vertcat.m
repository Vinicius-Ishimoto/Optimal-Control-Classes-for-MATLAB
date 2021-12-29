function out = vertcat(varargin)

for i=1:nargin
    if isa(varargin{i},'vect')
        nvl = size(varargin{i}(1).value,1);
        break
    end
end

for w=1:size(varargin{1},2)
    cont = 1;
for i=1:nargin
    if isnumeric(varargin{i})
        for j=1:size(varargin{i},1)
        out(cont,w).value = varargin{i}(j,w)*ones(nvl,1);
        cont = cont+1;
        end
    else
        for j=1:size(varargin{i},1)
        out(cont,w).value = varargin{i}(j,w).value;
        cont = cont+1;
        end
    end
end
end
out = class(out,'vect');

end