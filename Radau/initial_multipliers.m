function options = initial_multipliers(setup,lambda,options)

% xg = [];
% M = legslb(setup.mesh_number+1);
% M = 0.5*(M+1);
zl = [];
zu = [];
lb = [];
for w=1:setup.assist.nP
    lstate = [lambda.lower.phase(w).position,lambda.lower.phase(w).velocity];
    ustate = [lambda.upper.phase(w).position,lambda.upper.phase(w).velocity];
    constr = [lambda.constraints.phase(w).position,lambda.constraints.phase(w).position];
    for i=1:setup.phase(w).assist.n
        zl = [zl;interp1_lg(setup.initial_guess.phase(w).time,lstate(:,i),sum(setup.mesh.phase(w).colpoints))];
        zu = [zu;interp1_lg(setup.initial_guess.phase(w).time,ustate(:,i),sum(setup.mesh.phase(w).colpoints))];
        lb = [lb;interp1_lg(setup.initial_guess.phase(w).time,constr(:,i),sum(setup.mesh.phase(w).colpoints),1)];
    end
%     if setup.phase(w).assist.w ~=0
    for i=1:setup.phase(w).assist.w
        zl = [zl;interp1_lg(setup.initial_guess.phase(w).time,lambda.lower.phase(w).state(:,i),sum(setup.mesh.phase(w).colpoints))];
        zu = [zu;interp1_lg(setup.initial_guess.phase(w).time,lambda.upper.phase(w).state(:,i),sum(setup.mesh.phase(w).colpoints))];
        lb = [lb;interp1_lg(setup.initial_guess.phase(w).time,lambda.constraints.phase(w).state(:,i),sum(setup.mesh.phase(w).colpoints),1)];
    end
    for i=1:setup.phase(w).assist.p
        zl = [zl;interp1_lg(setup.initial_guess.phase(w).time,lambda.lower.phase(w).control(:,i),sum(setup.mesh.phase(w).colpoints),1)];
        zu = [zu;interp1_lg(setup.initial_guess.phase(w).time,lambda.upper.phase(w).control(:,i),sum(setup.mesh.phase(w).colpoints),1)];
    end
    zl = [zl;lambda.lower.phase(w).time(1)];
    zl = [zl;lambda.lower.phase(w).time(end)];
    zu = [zu;lambda.upper.phase(w).time(1)];
    zu = [zu;lambda.upper.phase(w).time(end)];
    for i=1:setup.phase(w).assist.cp
        lb = [lb;interp1_lg(setup.initial_guess.phase(w).time,lambda.constraints.phase(w).path(:,i),sum(setup.mesh.phase(w).colpoints),1)];
    end
    if isfield(setup.phase(w),'stabilization_constraints')
        if not(isempty(setup.phase(w).stabilization_constraints))
            for i=1:setup.phase(w).stabilization_constraints
                if strcmp(setup.phase(w).stabilization_method,'Integration')
                    lb = [lb;interp1_lg(setup.initial_guess.phase(w).time,lambda.constraints.phase(w).stabilization(:,i),sum(setup.mesh.phase(w).colpoints))];
                else
                    lb = [lb;interp1_lg(setup.initial_guess.phase(w).time,lambda.constraints.phase(w).stabilization(:,i),sum(setup.mesh.phase(w).colpoints),1)];
                end
            end
        end
    end
end
if setup.assist.cn~=0
    lb = [lb;reshape(lambda.constraints.pconstraints,[],1)];
end
if setup.assist.v~=0
    zl = [zl;lambda.lower.parameters];
    zu = [zu;lambda.upper.parameters];
end


options.zl = zl;
options.zu = zu;
options.lambda = lb;

end