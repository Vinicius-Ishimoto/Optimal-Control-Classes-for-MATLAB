function yi = interp1_lg(x,y,points,flag)
 
% M = legslb(points+1);
% M = 0.5*(M+1);
% 
% yi = interp1(x,y,M*(x(end)-x(1))+x(1),'cubic','extrap');
% yi = reshape(yi,[],1);

if nargin > 3
    M = legslb(points+1);
    M = 0.5*(M+1);
    vt = M*(x(end)-x(1))+x(1);
    yi = interp1(x(2:end),y,vt(2:end),'pchip','extrap');
    yi = reshape(yi,[],1);
else
    M = legslb(points+1);
    M = 0.5*(M+1);

    yi = interp1(x,y,M*(x(end)-x(1))+x(1),'pchip','extrap');
    yi = reshape(yi,[],1);
end

end