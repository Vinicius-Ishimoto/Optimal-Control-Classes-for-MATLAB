function out = integrate(solver,fun)

W = solver.weight_vector;
tf = solver.finaltime;
t0 = solver.initialtime;
F = double(fun);

out = (tf-t0)*W*F(1:end,:);

end