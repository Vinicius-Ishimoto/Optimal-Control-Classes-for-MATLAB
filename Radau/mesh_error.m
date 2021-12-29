function error = mesh_error(result)

funcs = result.function;

nPh = numel(result.solution.phase);
for q = 1:nPh
    position = result.solution.phase(q).position;
    velocity = result.solution.phase(q).velocity;
    control = full(result.solution.phase(q).control);
    time = result.solution.phase(q).time;
    V = diff(time);
    clear Vp
    for i=1:numel(V)
        Vp(i,1) = (time(i+1)+time(i))*0.5;
    end

    [tau,w] = legsrd(5);
    tt = 0.5*V*tau'+Vp*ones(1,5);
    tt = reshape(tt',[5*numel(V),1]);
    Tiw = reshape((0.5*V*w')',[numel(V)*5,1])';
    input.phase(q).integrand.weight_vector = Tiw;
    input.phase(q).time = vect(tt);
    
    clear bposition cposition bvelocity cvelocity bstate cstate

    for i=1:size(position,2)    
        PP = spline(time,position(:,i));
        bposition(:,i) = ppval(PP,tt);
        p_der = fnder(PP,1);
        cposition(:,i) = ppval(p_der,tt);
    end
    input.phase(q).position = vect(bposition);
    compare.phase(q).position = cposition;
    
    for i=1:size(velocity,2)    
        PP = spline(time,velocity(:,i));
        bvelocity(:,i) = ppval(PP,tt);
        p_der = fnder(PP,1);
        cvelocity(:,i) = ppval(p_der,tt);
    end
    input.phase(q).velocity = vect(bvelocity);
    compare.phase(q).velocity = cvelocity;
    
    if isfield(result.solution.phase(q),'state')
        state = result.solution.phase(q).state;
    for i=1:size(state,2)    
        PP = spline(time,state(:,i));
        bstate(:,i) = ppval(PP,tt);
        p_der = fnder(PP,1);
        cstate(:,i) = ppval(p_der,tt);
    end
    input.phase(q).state = vect(bstate);
    compare.phase(q).state = cstate;
    end
    
    bcontrol = interp1(time,control,tt,'linear','extrap');
    input.phase(q).control = vect(bcontrol);
end

solve = pre_function(input,result);

F = funcs(solve);
fprintf([' -----------******** Maximum error found *****************-------\n']);
for q=1:nPh
    velocity = input.phase(q).velocity;
    rhs = F.phase(q).RightHandSide;
    MM = F.phase(q).MassMatrix;
    aceleration = double((MM\rhs)');
    if isfield(F.phase(q),'derivatives')
        dstate = F.phase(q).derivatives;
        fprintf([' -----------******** State from phase\t',num2str(q),'\t *****************-------\n']);
        [error.phase(q).state,error.phase(q).statemax] = validation(input.phase(q).integrand.weight_vector,double(dstate),compare.phase(q).state,double(input.phase(q).time),double(input.phase(q).state));
    end
    fprintf([' -----------******** Position from phase\t',num2str(q),'\t *****************-------\n']);
    [error.phase(q).position,error.phase(q).positionmax] = validation(input.phase(q).integrand.weight_vector,double(velocity),compare.phase(q).position,double(input.phase(q).time),double(input.phase(q).position));
    fprintf([' -----------******** Velocity from phase\t',num2str(q),'\t *****************-------\n']);
    [error.phase(q).velocity,error.phase(q).velocitymax] = validation(input.phase(q).integrand.weight_vector,aceleration,compare.phase(q).velocity,double(input.phase(q).time),double(input.phase(q).velocity));
    
end



function setup = pre_function(input,result)

setup = input;
for q=1:numel(result.solution.phase)
    
    setup.phase(q).iposition = vect([input.phase(q).position;result.solution.phase(q).position(end,:)]);
    setup.phase(q).ivelocity = vect([input.phase(q).velocity;result.solution.phase(q).velocity(end,:)]);
    if isfield(result.solution.phase(q),'state')
        setup.phase(q).initial.state = result.solution.phase(q).state(1,:);
        setup.phase(q).final.state = result.solution.phase(q).state(end,:);
    end
    setup.phase(q).initial.time = result.solution.phase(q).time(1);
    setup.phase(q).initial.position = result.solution.phase(q).position(1,:);
    setup.phase(q).initial.velocity = result.solution.phase(q).velocity(1,:);
    setup.phase(q).final.time = result.solution.phase(q).time(end);
    setup.phase(q).final.position = result.solution.phase(q).position(end,:);
    setup.phase(q).final.velocity = result.solution.phase(q).velocity(end,:);
    setup.phase(q).integrand.initialtime = setup.phase(q).initial.time;
    setup.phase(q).integrand.finaltime = setup.phase(q).final.time;
    setup.phase(q).integrand.locationtime = [result.mesh.phase(q).initialtimepoint,result.mesh.phase(q).finaltimepoint];
    if isfield(result.solution,'parameter')
        setup.phase(q).parameter = vect(ones(numel(setup.phase(q).time),1)*result.solution.parameter');
        setup.phase(q).iparameter = vect(ones(numel(setup.phase(q).time)+1,1)*result.solution.parameter');
    end
end
if isfield(result,'auxdata')
    setup.auxdata = result.auxdata;
end

function [output,maxi] = validation(Tiw,x,xref,time,Ix)

Int = abs(xref-x);

for w=1:size(x,1)/5
    Err(w,:) = Tiw(1+5*(w-1):w*5)*Int(1+5*(w-1):w*5,:);
    Err2(w,:) = trapz(time(1+5*(w-1):w*5),Int(1+5*(w-1):w*5,:));
    Sums(w,:) = max(abs([Ix(1+(w-1)*5:w*5,:);xref(1+(w-1)*5:w*5,:)]),[],1)+ones(1,size(Ix,2));

end
% Err = Tiw*Int;

Err = Err./Sums;
Err2 = Err2./Sums;

for i=1:size(x,2)

    Max = max(Err(:,i));
    I = find(Err(:,i)>=Max);
    Me = mean(Err(:,i));
    maxi(1,i) = Max;
fprintf(['- State ',num2str(i),', defined in the ',num2str(I),' collocation point:\n'])
fprintf(['--------- with value: ',num2str(Max),'.\n'])
fprintf(['--------- mean of errors: ',num2str(Me),'.\n'])
fprintf(['--------- with: ',num2str(abs((Max-Me)/Me)*100),' percent higher than the mean.\n'])

end

output = Err;