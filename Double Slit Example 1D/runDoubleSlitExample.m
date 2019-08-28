clc
clear all
close all

load('paramDoubleSlit')
initState = 0;
param.numSample = 10000;
% param.barrierPos = [-19,-18,20;-20,-19,18];
constraintType = 1;
param.e = @(x)[-0.5;-0.05];
param.F = @(x)[1;-1];
% [unconstrained_actualPath,unconstrained_U,unconstrained_J,unconstrained_Cost,unconstrained_Psi,unconstrained_isCollision] = runSimulation(initState,false,param);
[constrained_actualPath,constrained_U,constrained_J,constrained_Cost,constrained_Psi,constrained_isCollision] = runSimulation(initState,constraintType,param);

% unconstrained_cost = 0.5*unconstrained_U*unconstrained_U'*param.simInterval + 0.5*unconstrained_actualPath(:,end).^2;
constrained_cost = 0.5*constrained_U*constrained_U'*param.simInterval + 0.5*constrained_actualPath(:,end).^2;

% fprintf("**************************************************\n");
fprintf("*   Constrained Case: Cost = "+num2str(constrained_cost,3)+"\n");
% fprintf("* Unconstrained Case: Cost = "+num2str(unconstrained_cost,3)+"\n");
% fprintf("**************************************************\n");
fprintf("*   Constrained Case: Terminal State = "+num2str(constrained_actualPath(end),3)+"\n");
% fprintf("* Unconstrained Case: Terminal State = "+num2str(unconstrained_actualPath(end),3)+"\n");
% fprintf("**************************************************\n");