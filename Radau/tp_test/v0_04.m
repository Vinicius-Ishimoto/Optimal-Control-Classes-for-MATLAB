setup.phase(1).mesh_points = 5;
setup.phase(1).mesh_number = 4;
setup.phase(2).mesh_points = 5;
setup.phase(2).mesh_number = 4;
setup.auxdata.M = 110;
setup.auxdata.C = 4.6;
setup.auxdata.Fr = 8.9;
setup.function = @dynamics_tp;

genpath('C:\Users\Vinicius\Google Drive\NLP_Converter\tests\dstruct');
addpath(ans);
genpath('C:\Users\vinicios\Google Drive\NLP_Converter\tests\dstruct');
addpath(ans);
genpath(cd);
addpath(ans);

%% Bounds

setup.bound.lower.phase(1).initial.state = [0,0];
setup.bound.lower.phase(1).initial.time = 0;
setup.bound.lower.phase(1).final.state = [0,0];
setup.bound.lower.phase(1).final.time = 0.1;
setup.bound.lower.phase(1).state = [0,0];
setup.bound.lower.phase(1).control = -100;
setup.bound.lower.phase(1).path = 0;
setup.bound.lower.phase(2).initial.state = [0,0];
setup.bound.lower.phase(2).initial.time = 0.1;
setup.bound.lower.phase(2).final.state = [10,0];
setup.bound.lower.phase(2).final.time = 10/0.9;
setup.bound.lower.phase(2).state = [0,0];
setup.bound.lower.phase(2).control = -100;
setup.bound.lower.phase(2).path = -3;
setup.bound.lower.pconstraints = [0,0,0,0];
% setup.bound.lower.parameter = 0;


setup.bound.upper.phase(1).initial.state = [0,0];
setup.bound.upper.phase(1).initial.time = 0;
setup.bound.upper.phase(1).final.state = [10,inf];
setup.bound.upper.phase(1).final.time = 10/0.9;
setup.bound.upper.phase(1).state = [10,inf];
setup.bound.upper.phase(1).control = 100;
setup.bound.upper.phase(1).path = 3;
setup.bound.upper.phase(2).initial.state = [10,inf];
setup.bound.upper.phase(2).initial.time = 10/0.9;
setup.bound.upper.phase(2).final.state = [10,0];
setup.bound.upper.phase(2).final.time = 10/0.9;
setup.bound.upper.phase(2).state = [10,inf];
setup.bound.upper.phase(2).control = 100;
setup.bound.upper.phase(2).path = 0;
setup.bound.upper.pconstraints = [(0.5*110*0.9^2+4.6*0.9*5+8.9*5)*6/4,0,0,0];
% setup.bound.upper.parameter = 0;
setup.bound.lower.parameter  = 0;
setup.bound.upper.parameter = inf;


%% Guess

load guess
setup.initial_guess.phase(1).time = results_1i.solution.time;
setup.initial_guess.phase(2).time = results_2i.solution.time;
setup.initial_guess.phase(1).state = results_1i.solution.state;
setup.initial_guess.phase(2).state = results_2i.solution.state;
setup.initial_guess.phase(1).control = results_1i.solution.control;
setup.initial_guess.phase(2).control = results_2i.solution.control;
setup.initial_guess.parameter = 1;

ddiopt(setup);