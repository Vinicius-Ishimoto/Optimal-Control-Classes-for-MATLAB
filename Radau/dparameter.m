function out = dparameter(value,ndiff,location,setup,ncol)

if isempty(value)
    out = [];
else
out = hderiv(value,ndiff,location,setup,ncol,'parameter');
end


end