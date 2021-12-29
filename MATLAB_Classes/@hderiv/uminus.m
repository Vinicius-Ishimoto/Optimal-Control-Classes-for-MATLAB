function out = uminus(in)

for j=1:size(in,2)
    for i=1:size(in,1)
out(i,j).value = -in(i,j).value;
out(i,j).dV = -in(i,j).dV;
out(i,j).ddV = -in(i,j).ddV;
out(i,j).nzx = in(i,j).nzx;
out(i,j).nzy = in(i,j).nzy;
out(i,j).nzr = in(i,j).nzr;
    end
end

out = class(out,'hderiv');

end