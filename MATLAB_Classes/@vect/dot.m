function out = dot(D,in)

for j=1:size(in,2)
    for i=1:size(in,1)
        out(i,j).value = D*in(i,j).value;
%         out(i,j).dV = D*in(i,j).dV;
%         if numel(nonzeros(in(i,j).ddV))==0
%             out(i,j).ddV = in(i,j).ddV(1:end-1,:);
%             out(i,j).nzx = in(i,j).nzx(1:end-1,:);
%             out(i,j).nzy = in(i,j).nzy(1:end-1,:);
%             out(i,j).nzr = in(i,j).nzr(1:end-1,:);
%         else
%             out(i,j).ddV = in(i,j).ddV;
%             out(i,j).nzx = in(i,j).nzx;
%             out(i,j).nzy = in(i,j).nzy;
%             out(i,j).nzr = in(i,j).nzr;
%             for k=1:size(in(i,j).value,1)-1
%                 ddV = sparse(diag(D(k,:)))*in(i,j).ddV;
%             end
%         end
    end
end

out = class(out,'vect');

end