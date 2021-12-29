function out = integrate(solver,fun)

W = solver.weight_vector;
tf = solver.finaltime;
t0 = solver.initialtime;

out = (tf-t0)*W*fun;

end