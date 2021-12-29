function out = ifThenElse(condition,itrue,ifalse,factor)

if nargin == 3
    factor = 0;
end

if factor == 0
    nvector = size(condition.value,1);
	flt = zeros(nvector,1);
	flt = flt + (condition.value>0);
	flt = flt + (condition.value==0)*0.5;
	flf = zeros(nvector,1);
	flf = flf + (condition.value<0);
	flf = flf + (condition.value==0)*0.5;
	
	out = struct('value',[]);
	out = repmat(out,1,1);
    itrue.value  = sparse(1:nvector,1:nvector,flt)*itrue.value;
    ifalse.value = sparse(1:nvector,1:nvector,flf)*ifalse.value;
	out.value = itrue.value + ifalse.value;
    out = class(out,'vect');
    
else
	flt = 1./(1+exp(-factor*condition));
	flf = 1-1./(1+exp(-factor*condition));
	
	out = flt.*itrue + flf.*ifalse;
end
	
	
