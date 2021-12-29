function out = col(in,number)

if nargin==1
for j=1:size(in,2)
    for i=1:size(in,1)
        out(i,j).value = in(i,j).value(1:end-1,:);
        out(i,j).dV = in(i,j).dV(1:end-1,:);
        out(i,j).ddV = in(i,j).ddV(1:end-1,:);
        out(i,j).nzx = in(i,j).nzx(1:end-1,:);
        out(i,j).nzy = in(i,j).nzy(1:end-1,:);
        out(i,j).nzr = in(i,j).nzr(1:end-1,:);
    end
end
else
for j=1:size(in,2)
    for i=1:size(in,1)
        out(i,j).value = in(i,j).value(number,:);
        out(i,j).dV = in(i,j).dV(number,:);
        out(i,j).ddV = in(i,j).ddV(number,:);
        out(i,j).nzx = in(i,j).nzx(number,:);
        out(i,j).nzy = in(i,j).nzy(number,:);
        out(i,j).nzr = in(i,j).nzr(number,:);
    end
end
end
out = class(out,'hderiv');
end