function out = Mminus(ina,inb)

for j=1:size(ina,2)
    for i=1:size(ina,1)
        out(i,j).value = ina(i,j).value-inb(:,j);
        out(i,j).dV = ina(i,j).dV;
        out(i,j).ddV = ina(i,j).ddV;
        out(i,j).nzx = ina(i,j).nzx;
        out(i,j).nzy = ina(i,j).nzy;
        out(i,j).nzr = ina(i,j).nzr;
    end
end

out = class(out,'hderiv');

end