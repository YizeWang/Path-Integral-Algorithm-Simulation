function [actualPath,U,J,Cost,Psi,isCollision] = runSimulation(initState,constraintType,param)
% runSimulation function. The function will run the simulation and plot
% sample visualization, actual path, control input sequence and cost-to-go.
%
% Input:
%   initState           a value of initial position
%   constraintType      the type of constraint
%                       0: no constraints
%                       1: input constraints
%                       2: state-input constraints
%   param               system parameters
% Output:
%   actualPath          a matrix of actual trajectories
%   U                   vector of input sequence
%   J                   vector of optimal cost
%   Cost                matrix of cost
%   Psi                 vector of Psi
%   isCollision         boolean, whether crashed
%
% Author: Yize Wang
% Email: yizwang@student.ethz.ch

%% parameter initialization
u = 0; % if no control found at first step
U = [];
J = [];
Psi = [];
Cost = [];
state = initState;
actualPath = [state];
simHorizon = param.simStart:param.simInterval:param.simEnd;
barrierStep = find(simHorizon==param.barrierTime);

for currentTime = param.simStart:param.simInterval:param.simEnd-param.simInterval
    simHorizon = currentTime:param.simInterval:param.simEnd;
    [trajectory,noiseInput] = computeTrajectory(state,simHorizon,param,constraintType);
    if (currentTime > param.barrierTime)        % if behind barrier
        isBarrierDetected = false(param.numSample,1);
    else                                        % if before or at barrier
        isBarrierDetected = detectBarrier(trajectory,param,barrierStep);
    end
    [cost,psi] = computeCost(trajectory,isBarrierDetected,param);

    if psi == 0 % if psi 0, use previous control
        fprintf('Warning: All Samples Crashed \n');
    else % if psi !0, compute new control with path integral
        u = computeU(cost,psi,noiseInput,param);
    end
    state = state + param.G*u;
    barrierStep = barrierStep - 1;
    
    % record data
    actualPath = [actualPath state];
    U = [U u];
    J = [J -param.gamma*log(psi)];
    Cost = [Cost cost];
    Psi = [Psi psi];
end

%% collision test
barrierStep = find(param.simStart:param.simInterval:param.simEnd==param.barrierTime);
isCollision = detectBarrier(actualPath,param,barrierStep);
if isCollision
    fprintf("Crashed! \n");
else
    fprintf("Successfully Passed Barrier! \n");
end

%% plot figures
% initialize figure
if constraintType
    figure('Name',"Double Slit Example (Input Constrained)");
else
    figure('Name',"Double Slit Example (Unconstrained)");
end

% plot rollouts
subplot(2,2,1)
plotSample(initState,0,param,constraintType);
hold on

% plot actual path
subplot(2,2,2)
title("Actual Path",'FontSize',param.fontSize)
xlabel("Time",'FontSize',param.fontSize)
ylabel("Position",'FontSize',param.fontSize)

axis([param.simStart param.simEnd -10 10])
ax = gca;
ax.FontSize = param.fontSize;
hold on
plot(param.simStart:param.simInterval:param.simEnd,actualPath)
hold on

% plot barriers
line([param.barrierTime; param.barrierTime],param.barrierPos,'color','k','LineWidth',8*param.lineWidth)
hold on

% plot control input sequence
subplot(2,2,3)
title("Control Input Sequence",'FontSize',param.fontSize)
xlabel("Time",'FontSize',param.fontSize)
ylabel("Control Input",'FontSize',param.fontSize)
axis([param.simStart param.simEnd -0.2 0.2])
ax = gca;
ax.FontSize = param.fontSize;
hold on
plot(param.simStart:param.simInterval:param.simEnd-param.simInterval,U)
hold on
if constraintType == 1
    line([param.simStart param.simEnd],[param.uMax param.uMin;param.uMax param.uMin],'color','r','LineWidth',param.lineWidth)
    hold on
end

% plot optimal cost
subplot(2,2,4)
title("Optimal Cost",'FontSize',param.fontSize)
xlabel("Time",'FontSize',param.fontSize)
ylabel("Optimal Cost",'FontSize',param.fontSize)
axis([param.simStart param.simEnd 0 1.2*max(J)])
ax = gca;
ax.FontSize = param.fontSize;
hold on
plot(param.simStart:param.simInterval:param.simEnd-param.simInterval,J)
hold on

end