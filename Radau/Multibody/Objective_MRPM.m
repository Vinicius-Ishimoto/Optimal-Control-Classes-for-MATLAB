function objective = Objective_MRPM(x,setup)

dynamics = function_Mrunner(x,setup);

% Objective
objective = double(dynamics.objective);

end