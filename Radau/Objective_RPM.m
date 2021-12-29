function objective = Objective_RPM(x,setup)

dynamics = function_runner(x,setup);

% Objective
objective = double(dynamics.objective);

end