setup.mesh_points = 5;
setup.mesh_number = 20;
setup.auxdata.M = 100;
setup.auxdata.C = 15;
setup.function = @Dynamics_LTI;
% genpath('C:\Users\Vinicius\Google Drive\NLP_Converter\tests\dstruct');
% addpath(ans);
% genpath('C:\Users\vinicios\Google Drive\NLP_Converter\tests\dstruct');
% addpath(ans);
% genpath(cd);
% addpath(ans);

%% Bounds

setup.bound.lower.phase.initial.state = zeros(1,4);
setup.bound.lower.phase.initial.time = 0;
setup.bound.lower.phase.final.state = -inf(1,4);
setup.bound.lower.phase.final.time = 1e-4;
setup.bound.lower.phase.state = -inf(1,4);
setup.bound.lower.phase.control = [-1,-1];
setup.bound.lower.phase.path = [-0.01,-0.01];
setup.bound.lower.pconstraints = [0,0,1,1];
% setup.bound.lower.parameter = 0;


setup.bound.upper.phase.initial.state = zeros(1,4);
setup.bound.upper.phase.initial.time = 0;
setup.bound.upper.phase.final.state = inf(1,4);
setup.bound.upper.phase.final.time = 10;
setup.bound.upper.phase.state = inf(1,4);
setup.bound.upper.phase.control = [1,1];
setup.bound.upper.phase.path = [1.01,1.01];
setup.bound.upper.pconstraints = [0,0,1,1];
% setup.bound.upper.parameter = 0;


%% Guess

setup.initial_guess.phase.state = rand(100,4);
setup.initial_guess.phase.control = rand(100,2);
setup.initial_guess.phase.time = linspace(0,10,100);

ddiopt(setup);