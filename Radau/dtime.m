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
if setup.mesh.flag==1
    out = hderiv([t0,tf],ndiff,location,setup,phase,'time');
else
out = deriv([t0,tf],ndiff,location,setup,phase,'time');
end
end