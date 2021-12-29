function out = vertcat(varargin)

for w=1:nargin
    if isa(varargin{w},'hderiv')
        ndiff = size(varargin{w}(1).dV,2);
        nvector = size(varargin{w},2);
        nvalue = size(varargin{w}(1).dV,1);
        break
    end
end

cont = 0;
for w=1:nargin
    if isa(varargin{w},'hderiv')
            for i=1:size(varargin{w},1)
                for j=1:size(varargin{w},2)
                    
                    out(i+cont,j).value = varargin{w}(i,j).value;
                    out(i+cont,j).dV = varargin{w}(i,j).dV;
                    out(i+cont,j).ddV = varargin{w}(i,j).ddV;
                    out(i+cont,j).nzx = varargin{w}(i,j).nzx;
                    out(i+cont,j).nzy = varargin{w}(i,j).nzy;
                    out(i+cont,j).nzr = varargin{w}(i,j).nzr;
                    
                end
            end
            cont = cont+i;
    else
        
            for j=1:size(varargin{w},1)
                for i=1:nvector
        out(1+cont,i).value = varargin{w}(j,i)*ones(nvalue,1);
        out(1+cont,i).dV = sparse(nvalue,ndiff);
        out(1+cont,i).ddV = sparse(nvalue,0);
        out(1+cont,i).nzx = sparse(nvalue,0);
        out(1+cont,i).nzy = sparse(nvalue,0);
        out(1+cont,i).nzr = sparse(nvalue,0);
        
                end
            cont = cont + 1;
        end
    end
end


out = class(out,'hderiv');
end