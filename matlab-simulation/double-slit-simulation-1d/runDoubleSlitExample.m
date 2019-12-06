% run the simulation for both constrained and unconstrained cases
%
% Author: Yize Wang
% Email: yizwang@student.ethz.ch

clc
clear all
close all

%% settings
load('paramDoubleSlit') % load parameters
initState = 0; % value of initial state
param.numSample = 1000; % number of samples
param.barrierPos = [-3,2,20;-20,-2,3];  % barrier position. the i-th column
                                        % stands for the i-th barrier pos
                                        % first row -> top position
                                        % second row -> bottom position
constraintType = 1; % the type of constraint
                    % 0: no constraints
                    % 1: input constraints
                    % 2: state-input constraints
param.uMin = -0.03; % min input for constraint type 1
param.uMax = 0.2;   % max input for constraint type 1
param.e = @(x)[-0.5;-0.05]; % e+Fu<0        for constraint type 2
param.F = @(x)[1;-1];       % -0.05<u<0.5   for constraint type 2

%% start simulation
[unconstrained_actualPath,unconstrained_U,unconstrained_J,unconstrained_Cost,unconstrained_Psi,unconstrained_isCollision] = runSimulation(initState,false,param);
[constrained_actualPath,constrained_U,constrained_J,constrained_Cost,constrained_Psi,constrained_isCollision] = runSimulation(initState,constraintType,param);
unconstrained_cost = 0.5*unconstrained_U*unconstrained_U'*param.simInterval + 0.5*unconstrained_actualPath(:,end).^2;
constrained_cost = 0.5*constrained_U*constrained_U'*param.simInterval + 0.5*constrained_actualPath(:,end).^2;

%% print results
fprintf("**************************************************\n");
fprintf("      Constrained Case: Cost = "+num2str(constrained_cost,3)+"\n");
fprintf("    Unconstrained Case: Cost = "+num2str(unconstrained_cost,3)+"\n");
fprintf("**************************************************\n");
fprintf("      Constrained Case: Terminal State = "+num2str(constrained_actualPath(end),3)+"\n");
fprintf("    Unconstrained Case: Terminal State = "+num2str(unconstrained_actualPath(end),3)+"\n");
fprintf("**************************************************\n");