function out = vect(vector,ndiff)

if nargin==1
    ndiff = size(vector,1);
    if numel(vector)==0
        out = [];
        return
    end
end

for i=1:size(vector,2)
    cont = 0;
    for j=1:size(vector,1)/ndiff
        out(j,i).value = vector(cont+1:cont+ndiff,i);
        cont = cont+ndiff;
    end
end

out = class(out,'vect');
end