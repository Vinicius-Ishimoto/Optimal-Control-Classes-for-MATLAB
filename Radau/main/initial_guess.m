function xg = initial_guess(setup)

xg = [];
% M = legslb(setup.mesh_number+1);
% M = 0.5*(M+1);
for w=1:setup.assist.nP
    for i=1:setup.phase(w).assist.n
        xg = [xg;interp1_lg(setup.initial_guess.phase(w).time,setup.initial_guess.phase(w).state(:,i),sum(setup.mesh.phase(w).colpoints))];
    end
    for i=1:setup.phase(w).assist.p
        x = setup.initial_guess.phase(w).time;
        cg = interp1(setup.initial_guess.phase(w).time,setup.initial_guess.phase(w).control(:,i),double(dtime(x(1),x(end),2,[1,2],setup,w)),'linear','extrap');
        xg = [xg;cg];
    end
    xg = [xg;setup.initial_guess.phase(w).time(1)];
    xg = [xg;setup.initial_guess.phase(w).time(end)];
end
try
    xg = [xg;setup.initial_guess.parameter];
catch
    try
       xg = [xg;setup.initial_guess.parameter'];
    catch
    end
end

end