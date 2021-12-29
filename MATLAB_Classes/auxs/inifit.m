function out = inifit(t0,tf,ndiff,location,setup,phase)

% Vetorize the initial and final time between the meshs.
%
%       out = inifit(t0,tf,ndiff,setup,location,phase)
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
if setup.mesh.flag==1
    out = hderiv([t0,tf],ndiff,location,setup,phase,'inifit');
else
out = deriv([t0,tf],ndiff,location,setup,phase,'time');
end
end