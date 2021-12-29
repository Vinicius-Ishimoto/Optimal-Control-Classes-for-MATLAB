function out = Hjacobian(ina,inb)

for i=1:size(ina,1)
    dH = sparse(ina(i,:)'*inb(i,:));
%     if tril(dH)==tril(dH')
        H = 0.5*tril(dH+dH');
%     else
%     H = tril(ina(i,:)'*inb(i,:))+tril(dH');
%     H = 0.5*tril(H,-1)+0.5*sparse(1:size(H,1),1:size(H,1),diag(H),size(H,1),size(H,1));
%     H = 0.5*H;
%     end
    [I,J,K] = find(H);
    jH(i,1:numel(I)) = K';
    Iv(i,1:numel(I)) = I';
    Jv(i,1:numel(I)) = J';
    N = find(H);
    Nv(i,1:numel(I)) = N';
end

out.ddV = jH;
out.nzx = Iv;
out.nzy = Jv;
out.nzr = Nv;

end