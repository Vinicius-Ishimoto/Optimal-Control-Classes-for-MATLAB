function out = sqrt(ina)

nvector = size(ina(1).value,1);
out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(ina,1),size(ina,2));
inb = 0.5;

for j=1:size(ina,2)
                for i=1:size(ina,1)
                 out(i,j).value = ina(i,j).value.^inb;
                 out(i,j).dV = inb*sparse(1:nvector,1:nvector,ina(i,j).value.^(inb-1))*ina(i,j).dV;
                 jH = Hjacobian(ina(i,j).dV,ina(i,j).dV);
                 jH.ddV = inb*(inb-1)*sparse(1:nvector,1:nvector,ina(i,j).value.^(inb-2))*jH.ddV;
                 ina(i,j).ddV = inb*sparse(1:nvector,1:nvector,ina(i,j).value.^(inb-1))*ina(i,j).ddV;
                 H = hessian_str(ina(i,j),jH,size(ina(1).dV,2));
                 out(i,j).ddV = H.ddV;
                 out(i,j).nzx = H.nzx;
                 out(i,j).nzy = H.nzy;
                 out(i,j).nzr = H.nzr;
                end
end

out = class(out,'hderiv');

