function out = exp(in)

out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
out = repmat(out,size(in,1),size(in,2));
for j=1:size(in,2)
    for i=1:size(in,1)
        out(i,j).value = exp(in(i,j).value);
        out(i,j).dV = sparse(1:numel(in(i,j).value),1:numel(in(i,j).value),exp(in(i,j).value))*in(i,j).dV;
        jH = Hjacobian(in(i,j).dV,in(i,j).dV);
        jH.ddV = sparse(1:numel(in(i,j).value),1:numel(in(i,j).value),exp(in(i,j).value))*jH.ddV;
        in(i,j).ddV = sparse(1:numel(in(i,j).value),1:numel(in(i,j).value),exp(in(i,j).value))*in(i,j).ddV;
        H = hessian_str(in(i,j),jH,size(in(i,j).dV,2));
% H = hessian_str(in(i,j),jH);
        out(i,j).ddV = H.ddV;
        out(i,j).nzx = H.nzx;
        out(i,j).nzy = H.nzy;
        out(i,j).nzr = H.nzr;
    end
end

out = class(out,'hderiv');

end