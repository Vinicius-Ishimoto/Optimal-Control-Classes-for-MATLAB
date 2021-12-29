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
	
	itrue.value  = sparse(1:nvector,1:nvector,flt)*itrue.value;
	itrue.dV     = sparse(1:nvector,1:nvector,flt)*itrue.dV;
	itrue.ddV    = sparse(1:nvector,1:nvector,flt)*itrue.ddV;
	ifalse.value = sparse(1:nvector,1:nvector,flf)*ifalse.value;
	ifalse.dV    = sparse(1:nvector,1:nvector,flf)*ifalse.dV;
	ifalse.ddV   = sparse(1:nvector,1:nvector,flf)*ifalse.ddV;
	
	out = struct('value',[],'dV',[],'ddV',[],'nzx',[],'nzy',[],'nzr',[]);
	out = repmat(out,1,1);
	out.value = itrue.value + ifalse.value;
	out.dV = itrue.dV + ifalse.dV;
	H = hessian_str(itrue,ifalse,size(condition.dV,2));
    out.ddV = H.ddV;
    out.nzx = H.nzx;
    out.nzy = H.nzy;
    out.nzr = H.nzr;
    out = class(out,'hderiv');
else
	flt = 1./(1+exp(-factor*condition));
	flf = 1-1./(1+exp(-factor*condition));
	
	out = flt.*itrue + flf.*ifalse;
end