function out = hderiv(vector,ndiff,location,varargin)
% - Sparse Class with an automatic differentiation for NLP purposes.
% - It calculates first (jacobian) and second (hessian) derivatitives
%
% Inputs:
% hderiv(vector,ndiff,location,varargin)
% -vector: the vector of values, it creates a sparse value by column;
% -ndiff:  maximum number of variables in the problem;
% -location: the location of the values in 'vector'.
%            it needs to have the same size as number of rows in 'vector';
%
% EG: hderiv(rand(100,1),100,1:100)

if nargin==6
    if strcmp(varargin{3},'time')
    out = dtime(vector(1),vector(2),ndiff,location,varargin{1},varargin{2});
    elseif strcmp(varargin{3},'parameter')
    out = dparameter(vector,ndiff,location,varargin{1},varargin{2});
    elseif strcmp(varargin{3},'inifit')
    out = inifit(vector(1),vector(2),ndiff,location,varargin{1},varargin{2});
    end
else

if numel(location) > 0
for i=1:size(vector,2)
    out(1,i).value = vector(:,i);
    out(1,i).dV = sparse(1:size(vector,1),location(1+(i-1)*size(vector,1):i*size(vector,1)),ones(size(vector,1),1),size(vector,1),ndiff);
    out(1,i).ddV = sparse(size(vector,1),0);
    out(1,i).nzx = sparse(size(vector,1),0);
    out(1,i).nzy = sparse(size(vector,1),0);
    out(1,i).nzr = sparse(size(vector,1),0);
end
else
for i=1:size(vector,2)
    out(1,i).value = vector(:,i);
    out(1,i).dV = sparse(size(vector,1),ndiff);
    out(1,i).ddV = sparse(size(vector,1),0);
    out(1,i).nzx = sparse(size(vector,1),0);
    out(1,i).nzy = sparse(size(vector,1),0);
    out(1,i).nzr = sparse(size(vector,1),0);
end
end

out = class(out,'hderiv');
end

end

function out = dtime(t0,tf,ndiff,location,setup,phase)

% Vectorize the time used to calculate the optimal control problem.
%
%       out = dtime(t0,tf,ndiff,setup,location,phase)
%
% - t0 = initial time [1,1];
% - tf = final time   [1,1];
% - ndiff = number of derivatives [1,1];
% - setup = setup of the optimal control problem [struct];
% - location = location of t0 and tf in the jacobian numel(2);
% - phase = actual phase to compute [1,1];
%
%   Out is a variable of format deriv.
%

Tm = setup.phase(phase).solver.Tm;
W = setup.mesh.phase(phase).vtau;
C = setup.mesh.phase(phase).colpoints;
V = setup.mesh.phase(phase).fraction;
ci = [];
for i=1:numel(C)
    ci = [ci;i*ones(C(i),1)];
end
cV1 = cumsum(V);
cV2 = [0;cV1(1:end-1)];
if numel(location)~=2
    error('location must be a column or row vector with the indices of t0 and tf respectively.') 
end

out.value = diag(Tm).*(tf-t0).*W+0.5.*((cV1(ci)+cV2(ci)).*(tf-t0)+2*t0);

out.dV = sparse((1:numel(out.value))'*ones(1,2),ones(numel(out.value),1)*reshape(location,1,[]), ...
    [1-diag(Tm).*W-0.5.*(cV1(ci)+cV2(ci)),diag(Tm).*W+0.5.*(cV1(ci)+cV2(ci))],numel(out.value),ndiff);

    out.ddV = sparse(size(out.value,1),0);
    out.nzx = sparse(size(out.value,1),0);
    out.nzy = sparse(size(out.value,1),0);
    out.nzr = sparse(size(out.value,1),0);

out = class(out,'hderiv');

end

function out = dparameter(vector,ndiff,location,setup,ncol)
cont = 1;
for j=1:size(vector,2)
    for i=1:size(vector,1)
        out(i,j).value = vector(i,j)*ones(ncol,1);
        out(i,j).dV = sparse(1:ncol,location(cont),1,ncol,ndiff);
        out(i,j).ddV = sparse(ncol,0);
        out(i,j).nzx = sparse(ncol,0);
        out(i,j).nzy = sparse(ncol,0);
        out(i,j).nzr = sparse(ncol,0);
        cont = cont+1;
    end
end
out = class(out,'hderiv');
end

function out = inifit(t0,tf,ndiff,location,setup,phase)

% Vectorize the time used to calculate the optimal control problem.
%
%       out = dtime(t0,tf,ndiff,setup,location,phase)
%
% - t0 = initial time [1,1];
% - tf = final time   [1,1];
% - ndiff = number of derivatives [1,1];
% - setup = setup of the optimal control problem [struct];
% - location = location of t0 and tf in the jacobian numel(2);
% - phase = actual phase to compute [1,1];
%
%   Out is a variable of format deriv.
%

Tm = setup.phase(phase).solver.Tm;
W = setup.mesh.phase(phase).vtau;
C = setup.mesh.phase(phase).colpoints;
V = setup.mesh.phase(phase).fraction;
ci = [];
for i=1:numel(C)
    ci = [ci;i*ones(C(i),1)];
end
cV1 = cumsum(V);
cV2 = [0;cV1(1:end-1)];
if numel(location)~=2
    error('location must be a column or row vector with the indices of t0 and tf respectively.') 
end

out(1,1).value = diag(t0*Tm);
out(1,2).value = diag(tf*Tm);

out(1,1).dV = sparse((1:numel(out(1).value))',ones(numel(out(1).value),1)*location(1), ...
    diag(Tm),numel(out(1).value),ndiff);
out(1,2).dV = sparse((1:numel(out(1).value))',ones(numel(out(1).value),1)*location(2), ...
    diag(Tm),numel(out(1).value),ndiff);

    out(1,1).ddV = sparse(size(out(1,1).value,1),0);
    out(1,1).nzx = sparse(size(out(1,1).value,1),0);
    out(1,1).nzy = sparse(size(out(1,1).value,1),0);
    out(1,1).nzr = sparse(size(out(1,1).value,1),0);

out = class(out,'hderiv');

end