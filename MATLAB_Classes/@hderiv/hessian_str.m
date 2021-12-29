function out = hessian_str(ina,varargin)

t = 1;
ref = ina.nzr;


% for i=1:size(ref,1)
%     for j=1:size(varargin{w}.nzr,2)
%         if any(varargin{w}.nzr(i,j)==ref(i,:))
%             out.ddV(i,:) = varargin{w}.ddV(i,j)*(varargin{w}.nzr(i,j)==ref(i,:))+ina.ddV(i,:);
%             [~,k,n] = find(ina.ddV(i,:));
%             if isempty(k)
%                 k=0;
% %                 n=0;
%             end
%             out.nzx(i,1:k) = ina.nzx(i,:);
%             out.nzy(i,1:k) = ina.nzy(i,:);
%             out.nzr(i,1:k) = ina.nzr(i,:);
%         else
% %             k = find(ina.ddV(i,:));
%             [~,k,n] = find(ina.ddV(i,:));
%             if isempty(k)
%                 k=0;
% %                 n=0;
%             end
%             out.ddV(i,1:max(k)+1) = [n',varargin{w}.ddV(i,j)];
%             out.nzx(i,1:max(k)+1) = [ina.nzx(i,:),varargin{w}.nzx(i,j)];
%             out.nzy(i,1:max(k)+1) = [ina.nzy(i,:),varargin{w}.nzy(i,j)];
%             out.nzr(i,1:max(k)+1) = [ina.nzr(i,:),varargin{w}.nzr(i,j)];
%         end
%     end
% end

ndiff = varargin{end};

% for i=1:size(ref,1)
%     
%     for w=1:nargin-2
%     [~,~,ki] = find(varargin{w}.nzx(i,:));
%     [~,~,kj] = find(varargin{w}.nzy(i,:));
%     [~,~,kn] = find(varargin{w}.ddV(i,:));
%     Hb = tril(sparse(ki,kj,kn,ndiff,ndiff));
%     if w>1
%         H = H+Hb;
%     else
%     [~,~,ki] = find(ina.nzx(i,:));
%     [~,~,kj] = find(ina.nzy(i,:));
%     [~,~,kn] = find(ina.ddV(i,:));
%     Ha = tril(sparse(ki,kj,kn,ndiff,ndiff));
%     H = Ha+Hb;
%     end
%     end
%     
%     [I,J,K] = find(H);
%     jH(i,1:numel(I)) = K';
%     Iv(i,1:numel(I)) = I';
%     Jv(i,1:numel(I)) = J';
%     N = find(H);
%     Nv(i,1:numel(I)) = N';
%     
% end

    for w=1:nargin-2
        if w>1
            jH = [jH,varargin{w}.ddV];
            Iv = [Iv,varargin{w}.nzx];
            Jv = [Jv,varargin{w}.nzy];
            Nv = [Nv,varargin{w}.nzr];
        else
        jH = [ina.ddV,varargin{w}.ddV];
        Iv = [ina.nzx,varargin{w}.nzx];
        Jv = [ina.nzy,varargin{w}.nzy];
        Nv = [ina.nzr,varargin{w}.nzr];
        end
    end

            


% out = struct([]);
out.value = sparse(size(jH,1),0);
out.dV = sparse(size(jH,1),ndiff);
out.ddV = jH;
out.nzx = Iv;
out.nzy = Jv;
out.nzr = Nv;



end
            