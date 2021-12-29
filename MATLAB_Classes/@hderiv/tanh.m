function out = tanh(in)

for j=1:size(in,2)
    for i=1:size(in,1)
        out(i,j).value = tanh(in(i,j).value);
        out(i,j).dV = sparse(1:numel(in(i,j).value),1:numel(in(i,j).value),1-tanh(in(i,j).value).^2)*in(i,j).dV;
        jH = Hjacobian(in(i,j).dV,in(i,j).dV);
        jH.ddV = sparse(1:numel(in(i,j).value),1:numel(in(i,j).value),2*tanh(in(i,j).value).*(tanh(in(i,j).value).^2-1))*jH.ddV;
        in(i,j).ddV = sparse(1:numel(in(i,j).value),1:numel(in(i,j).value),1-tanh(in(i,j).value).^2)*in(i,j).ddV;
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