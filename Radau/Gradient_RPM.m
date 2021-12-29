function gradient = Gradient_RPM(x,setup)

if setup.mesh.flag==1
    dynamics = hessian_runner(x,setup);
else
dynamics = derivative_runner(x,setup);
end

gradient = reshape(jacobian(dynamics.objective),[],1);

end