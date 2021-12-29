function out = shessian(in,lambda,flag)

if nargin>1
if flag==1
out = sparse(size(in(1).dV,2),size(in(1).dV,2));
nvector = size(in(1).dV,1);
cont = 0;
for j=1:size(in,2)
    for i=1:size(in,1)
        if numel(in(i,j).ddV)~=numel(in(i,j).nzx)
            warning(['wrong combination of rows in (',num2str(i),',',num2str(j),').'])
        end
%         [~,~,kn] = find(diag(lambda(1+cont:nvector+cont))*in(i,j).ddV);
%         if isempty(kn)
%         else
        ddV = diag(lambda(1+cont:nvector+cont))*in(i,j).ddV;
        [~,~,ki] = find(in(i,j).nzx);
        [~,~,kj] = find(in(i,j).nzy);
        kn = find(in(i,j).nzr);
        out = out+sparse(ki,kj,ddV(kn),size(in(i,j).dV,2),size(in(i,j).dV,2));
        cont = cont+size(in(i,j).ddV,1);
%         end
    end
end
else
for j=1:size(in,2)
    for i=1:size(in,1)
        if numel(in(i,j).ddV)~=numel(in(i,j).nzx)
            warning(['wrong combination of rows in (',num2str(i),',',num2str(j),').'])
        end
        [~,~,ki] = find(in(i,j).nzx);
        [~,~,kj] = find(in(i,j).nzy);
        [~,~,kn] = find(diag(lambda)*in(i,j).ddV);
        out{i,j} = sparse(ki,kj,kn,size(in(i,j).dV,2),size(in(i,j).dV,2));
    end
end
end
else
for j=1:size(in,2)
    for i=1:size(in,1)
        if numel(in(i,j).ddV)~=numel(in(i,j).nzx)
            warning(['wrong combination of rows in (',num2str(i),',',num2str(j),').'])
        end
        nX = in(i,j).nzx;
        nX(nX==0) = 1;
        nY = in(i,j).nzy;
        nY(nY==0) = 2;
%         [~,~,ki] = find(in(i,j).nzx);
%         [~,~,kj] = find(in(i,j).nzy);
%         kn = find(in(i,j).nzr);
%         [~,~,kn] = find(in(i,j).ddV);
        out{i,j} = sparse(nX,nY,in(i,j).ddV,size(in(i,j).dV,2),size(in(i,j).dV,2));
    end
end
end
end