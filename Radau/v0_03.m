setup.mesh_points = 5;
setup.mesh_number = 20;
setup.auxdata.M = 100;
setup.auxdata.C = 15;
setup.function = @Dynamics;

genpath('C:\Users\Vinicius\Google Drive\NLP_Converter\tests\dstruct');
addpath(ans);
genpath('C:\Users\vinicios\Google Drive\NLP_Converter\tests\dstruct');
addpath(ans);
genpath(cd);
addpath(ans);

%% Bounds

setup.bound.lower.phase.initial.state = [0,0];
setup.bound.lower.phase.initial.time = 0;
setup.bound.lower.phase.final.state = [10,0];
setup.bound.lower.phase.final.time = 0.1;
setup.bound.lower.phase.state = [0,0];
setup.bound.lower.phase.control = -100;
setup.bound.lower.phase.path = -3;
setup.bound.lower.pconstraints = 0;
% setup.bound.lower.parameter = 0;


setup.bound.upper.phase.initial.state = [0,0];
setup.bound.upper.phase.initial.time = 0;
setup.bound.upper.phase.final.state = [10,0];
setup.bound.upper.phase.final.time = 10;
setup.bound.upper.phase.state = [10,inf];
setup.bound.upper.phase.control = 100;
setup.bound.upper.phase.path = 3;
setup.bound.upper.pconstraints = inf;
% setup.bound.upper.parameter = 0;


%% Guess

setup.initial_guess.phase.state = rand(100,2);
setup.initial_guess.phase.control = rand(100,1);
setup.initial_guess.phase.time = linspace(0,10,100);

ddiopt(setup);