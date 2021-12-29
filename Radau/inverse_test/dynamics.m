function [Mm,k,ke,MC] = dynamics(q,dq,tau,param)

switch param.flag
    case 1
    [Mm,k,ke] = minimum_d(q,dq,tau,param);
    case 2
    [Mm,k,ke,MC] = maximum_d1(q,dq,tau,param);
    case 3
    [Mm,k,ke,MC] = maximum_d2(q,dq,tau,param);
end


function [Mm,k,ke] = minimum_d(q,dq,tau,param)
    
ma = param.ma;
mb = param.mb;
Ja = param.Ja;
Jb = param.Jb;
A = param.A;
a = param.a;
B = param.B;
b = param.b;

alpha = q(1);
beta = q(2);
da = dq(1);
db = dq(2);
t1 = tau(1);
t2 = tau(2);

Mm = [     mb*A^2 + ma*a^2 + Ja, A*b*mb*cos(alpha - beta);
       A*b*mb*cos(alpha - beta),              mb*b^2 + Jb];
k = [                            0, A*b*db*mb*sin(alpha - beta);
      -A*b*da*mb*sin(alpha - beta),                           0];
ke =  [t1 - t2 + (981*A*mb*cos(alpha))/100 + (981*a*ma*cos(alpha))/100;
                                         t2 + (981*b*mb*cos(beta))/100];
k = k*[da;db];

end

function [Mm,k,ke,MC] = maximum_d1(q,dq,tau,param)
    
ma = param.ma;
mb = param.mb;
Ja = param.Ja;
Jb = param.Jb;
A = param.A;
a = param.a;
B = param.B;
b = param.b;

alpha = q(1);
beta = q(2);
da = dq(1);
db = dq(2);
t1 = tau(1);
t2 = tau(2);
    
Mm = [ ma*a^2 + Ja,               0,               0,              0
                 0,     mb*b^2 + Jb, -b*mb*sin(beta), b*mb*cos(beta)
                 0, -b*mb*sin(beta),              mb,              0
                 0,  b*mb*cos(beta),               0,             mb];
k = [ 0,                  0, 0, 0;
      0,                  0, 0, 0;
      0, -b*db*mb*cos(beta), 0, 0;
      0, -b*db*mb*sin(beta), 0, 0];
ke = [ t1 - t2 + (981*a*ma*cos(alpha))/100;
             t2 + (981*b*mb*cos(beta))/100;
                                         0;
                             (981*mb)/100];
MC =  [A*sin(alpha),-A*cos(alpha);
                  0,            0;
                  1,            0;
                  0,            1];
              
k = k*dq;

end

function [Mm,k,ke,MC] = maximum_d2(q,dq,tau,param)
    
ma = param.ma;
mb = param.mb;
Ja = param.Ja;
Jb = param.Jb;
A = param.A;
a = param.a;
B = param.B;
b = param.b;

alpha = q(1);
beta = q(2);
da = dq(1);
db = dq(2);
t1 = tau(1);
t2 = tau(2);

Mm = [ ma*a^2 + Ja,  0,  0,  0;
                 0, Jb,  0,  0;
                 0,  0, mb,  0;
                 0,  0,  0, mb];
    
k = zeros(4,1);

ke = [t1 - t2 + (981*a*ma*cos(alpha))/100;
                                       t2;
                                        0;
                             (981*mb)/100];

                         
MC = [ A*sin(alpha), -A*cos(alpha);
        b*sin(beta),  -b*cos(beta);
                  1,             0;
                  0,             1];
end

end