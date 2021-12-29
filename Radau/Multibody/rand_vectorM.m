function xg = rand_vectorM(setup)

xg = [];
% M = legslb(setup.mesh_number+1);
% M = 0.5*(M+1);
for w=1:setup.assist.nP
    lbstt = ones(sum(setup.mesh.phase(w).colpoints)+1,1)*[setup.bound.lower.phase(w).position,setup.bound.lower.phase(w).velocity];
    lbstt(1,:) = [setup.bound.lower.phase(w).initial.position,setup.bound.lower.phase(w).initial.velocity];
    lbstt(end,:) = [setup.bound.lower.phase(w).final.position,setup.bound.lower.phase(w).final.velocity];
    lbstt(isinf(lbstt)) = -10^5;
    ubstt = ones(sum(setup.mesh.phase(w).colpoints)+1,1)*[setup.bound.upper.phase(w).position,setup.bound.upper.phase(w).velocity];
    ubstt(1,:) = [setup.bound.upper.phase(w).initial.position,setup.bound.upper.phase(w).initial.velocity];
    ubstt(end,:) = [setup.bound.upper.phase(w).final.position,setup.bound.upper.phase(w).final.velocity];
    ubstt(isinf(ubstt)) = 10^5;

    for i=1:setup.phase(w).assist.n
        rdstt = lbstt(:,i)+rand(sum(setup.mesh.phase(w).colpoints)+1,1).*(ubstt(:,i)-lbstt(:,i));
        xg = [xg;rdstt];
    end
%     if setup.phase(w).assist.w ~=0
    for i=1:setup.phase(w).assist.w
        lbstt = ones(sum(setup.mesh.phase(w).colpoints)+1,1)*setup.bound.lower.phase(w).state(i);
        lbstt(1) = setup.bound.lower.phase(w).initial.state(i);
        lbstt(end) = setup.bound.lower.phase(w).final.state(i);
        lbstt(isinf(lbstt)) = -10^5;
        ubstt = ones(sum(setup.mesh.phase(w).colpoints)+1,1)*setup.bound.upper.phase(w).state(i);
        ubstt(1) = setup.bound.upper.phase(w).initial.state(i);
        ubstt(end) = setup.bound.upper.phase(w).final.state(i);
        ubstt(isinf(ubstt)) = 10^5;
        
        rdstt = lbstt+rand(sum(setup.mesh.phase(w).colpoints)+1,1).*(ubstt-lbstt);
        xg = [xg;rdstt];
    end
    for i=1:setup.phase(w).assist.p
        lbstt = ones(sum(setup.mesh.phase(w).colpoints),1)*setup.bound.lower.phase(w).control(i);
        lbstt(isinf(lbstt)) = -10^5;
        ubstt = ones(sum(setup.mesh.phase(w).colpoints),1)*setup.bound.upper.phase(w).control(i);
        ubstt(isinf(ubstt)) = 10^5;
        
        rdstt = lbstt+rand(sum(setup.mesh.phase(w).colpoints),1).*(ubstt-lbstt);
        xg = [xg;rdstt];
    end
    xg = [xg;setup.initial_guess.phase(w).time(1)];
    xg = [xg;setup.initial_guess.phase(w).time(end)];
end
try
    lbstt = reshape(setup.bound.lower.parameter,[],1);
    lbstt(isinf(lbstt)) = -10^5;
    ubstt = reshape(setup.bound.upper.parameter,[],1);
    ubstt(isinf(ubstt)) = 10^5;
    
    rdstt = lbstt+rand(numel(lbstt),1)*(ubstt-lbstt);
    xg = [xg;rdstt];
catch
end

end