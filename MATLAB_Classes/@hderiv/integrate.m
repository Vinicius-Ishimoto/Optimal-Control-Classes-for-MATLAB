function out = integrate(solver,fun)

W = solver.weight_vector;
tf = solver.finaltime;
t0 = solver.initialtime;
% fun = col(fun);
location = solver.locationtime;
nvector = size(fun(1).value,1);
for j=1:size(fun,2)
    for i=1:size(fun,1)
v = (tf-t0)*W*fun(i,j).value;
dV = (tf-t0)*W*fun(i,j).dV;
J = W*fun(i,j).dV;
dV = dV + sparse(1,reshape(location,1,[]),[-W*fun(i,j).value,W*fun(i,j).value], ...
    1,size(fun(i,j).dV,2));
% out.location = fun.location;
% ddV = sparse(size(fun(1).dV,2),size(fun(1).dV,2));

        fun(i,j).ddV = (tf-t0)*sparse(1:nvector,1:nvector,W)*fun(i,j).ddV;
        H1 = shessian(fun(i,j));
        H2 = sparse([location(1)*ones(size(fun(1).dV,2),1),location(2)*ones(size(fun(1).dV,2),1)], ...
            (1:size(fun(1).dV,2))'*ones(1,2),[-J',J'],size(fun(1).dV,2),size(fun(1).dV,2));
        H2d = diag(H2);
        H2d = sparse(1:size(H2,1),1:size(H2,1),H2d);
        H2 = tril(H2+H2'-H2d);
        H = H1{:}+H2;
       [I,J,K] = find(H);
       jH(1,1:numel(I)) = K';
       Iv(1,1:numel(I)) = I';
       Jv(1,1:numel(I)) = J';
       N = find(H);
       Nv(1,1:numel(I)) = N';
       out(i,j).value = v;
       out(i,j).dV = dV;
       out(i,j).ddV = jH;
       out(i,j).nzx = Iv;
       out(i,j).nzy = Jv;
       out(i,j).nzr = Nv;
    end
end

% out.v = v;
% out.dV = dV;
% out.ddV = ddV;

out = class(out,'hderiv');
end