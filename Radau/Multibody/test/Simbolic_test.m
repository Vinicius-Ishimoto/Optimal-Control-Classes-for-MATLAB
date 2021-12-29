%% Hessian test

syms y1 y2 real

M2 = [y1,cos(y2);-sin(y1),y2^5];
M3 = [y2;-y1^7];

M4 = M2*M3;
% M2*dM3 + dM2*M3
% dM2*dM3 + M2*ddM3 + ddM2*M3 + dM2*dM3
Hs = hessian(M4(1,1),[y1,y2]);

J2 = jacobian(M2,[y1,y2]);
J3 = jacobian(M3,[y1,y2]);
H2{1,1} = hessian(M2(1,1),[y1,y2]);
H2{1,2} = hessian(M2(1,2),[y1,y2]);
H2{2,1} = hessian(M2(2,1),[y1,y2]);
H2{2,2} = hessian(M2(2,2),[y1,y2]);
H3{1,1} = hessian(M3(1,1),[y1,y2]);
H3{2,1} = hessian(M3(2,1),[y1,y2]);

simplify((J2(1,:)'*J3(1,:))'+J2(3,:)'*J3(2,:)+...
    (M2(1)*H3{1,1}+M2(3)*H3{2,1})+...
    (H2{1,1}*M3(1)+H2{1,2}*M3(2)))

M4 = inv(M2);
% inv(M2)
% -inv(M2)*dM2*inv(M2)
% 2*inv(M2)*dM2*inv(M2)*dM2*inv(M2)-inv(M2)*ddM2*inv(M2)
Hs = simplify(hessian(M4(1,1),[y1,y2]));
UdU(1,:) = simplify(M4(1)*J2(1,:)+M4(3)*J2(2,:));
UdU(3,:) = simplify(M4(1)*J2(3,:)+M4(3)*J2(4,:));
UdU(2,:) = simplify(M4(2)*J2(1,:)+M4(4)*J2(2,:));
UdU(4,:) = simplify(M4(2)*J2(3,:)+M4(4)*J2(4,:));
dUdU{1,1} = simplify((UdU(1,:)'*UdU(1,:))'+UdU(3,:)'*UdU(2,:));
dUdU{1,2} = simplify((UdU(1,:)'*UdU(3,:))'+UdU(3,:)'*UdU(4,:));
dUdU{2,1} = simplify((UdU(2,:)'*UdU(1,:))'+UdU(4,:)'*UdU(2,:));
dUdU{2,2} = simplify((UdU(2,:)'*UdU(3,:))'+UdU(4,:)'*UdU(4,:));
UddU{1,1} = simplify(M4(1)*H2{1,1}+M4(3)*H2{2,1});
UddU{1,2} = simplify(M4(1)*H2{1,2}+M4(3)*H2{2,2});
UddU{2,1} = simplify(M4(2)*H2{1,1}+M4(4)*H2{2,1});
UddU{2,2} = simplify(M4(2)*H2{1,2}+M4(4)*H2{2,2});

simplify((dUdU{1,1}*M4(1)+dUdU{1,2}*M4(2))-...
    UddU{1,1}*M4(1)-UddU{1,2}*M4(2))