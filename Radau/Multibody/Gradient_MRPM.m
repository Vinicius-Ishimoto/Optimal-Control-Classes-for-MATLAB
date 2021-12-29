function gradient = Gradient_MRPM(x,setup)

if setup.mesh.flag==1
    dynamics = hessian_Mrunner(x,setup);
else
dynamics = derivative_runner(x,setup);
end

gradient = reshape(jacobian(dynamics.objective),[],1);

end