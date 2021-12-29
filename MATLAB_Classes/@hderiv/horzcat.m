function out = horzcat(varargin)

for w=1:nargin
    if isa(varargin{w},'hderiv')
        ndiff = size(varargin{w}(1).dV,2);
        nvector = size(varargin{w}(1).dV,1);
        ncol = size(varargin{w},1);
        break
    end
end

cont = 0;
for w=1:nargin
    if isa(varargin{w},'hderiv')
            for i=1:size(varargin{w},1)
                for j=1:size(varargin{w},2)
                    
                    out(i,j+cont).value = varargin{w}(i,j).value;
                    out(i,j+cont).dV = varargin{w}(i,j).dV;
                    out(i,j+cont).ddV = varargin{w}(i,j).ddV;
                    out(i,j+cont).nzx = varargin{w}(i,j).nzx;
                    out(i,j+cont).nzy = varargin{w}(i,j).nzy;
                    out(i,j+cont).nzr = varargin{w}(i,j).nzr;
                    
                end
            end
            cont = cont+j;
    else
            for j=1:size(varargin{w},2)
                for i=1:ncol
        out(i,1+cont).value = varargin{w}(i,j)*ones(nvector,1);
        out(i,1+cont).dV = sparse(nvector,ndiff);
        out(i,1+cont).ddV = sparse(nvector,0);
        out(i,1+cont).nzx = sparse(nvector,0);
        out(i,1+cont).nzy = sparse(nvector,0);
        out(i,1+cont).nzr = sparse(nvector,0);
        
                end
            cont = cont + 1;
            end
%             out(:,1+cont:cont+size(varargin{w},2)).dV = sparse(nvector,ndiff);
%         out(:,1+cont:cont+size(varargin{w},2)).ddV = sparse(nvector,1);
%         out(:,1+cont:cont+size(varargin{w},2)).nzx = sparse(nvector,0);
%         out(:,1+cont:cont+size(varargin{w},2)).nzy = sparse(nvector,0);
%         out(:,1+cont:cont+size(varargin{w},2)).nzr = sparse(nvector,0);
%         cont = cont + size(varargin{w},2);
    end
end


out = class(out,'hderiv');
end