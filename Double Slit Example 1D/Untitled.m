load('test.mat')
initState = 0;
currentTime = 0;
constraintType = 1;
param.Q = 4;
param.gamma = sqrt(1.6);
param.uMin = -3.8;
param.numSample = 10000;
close all
% plotSample function. The function will plot all rollouts at any step.
%
% Input:
%   initState           a value of current position
%   currentTime         a value of current step
%   param               system parameters
%   constraintType      the type of constraint
% Output:
%   visualization of samples

    %% initialize parameters
    simHorizon = currentTime:param.simInterval:param.simEnd;
    barrierStep = find(simHorizon==param.barrierTime);
    
    %% compute trajectory
    [trajectory,~] = computeTrajectory(initState,simHorizon,param,constraintType);
    
    %% detect barrier & truncate trajectory
    isBarrierDetected = detectBarrier(trajectory,param,barrierStep);
    feasibleTrajectory = trajectory(~isBarrierDetected,:);
    inFeasibleTrajectory = trajectory(isBarrierDetected,1:barrierStep);

 up = sum(trajectory(:,barrierStep)>2 & trajectory(:,barrierStep)<4);
 down = sum(trajectory(:,barrierStep)<-2 & trajectory(:,barrierStep)>-4);
    %% plot samples
    % figure initialization
    passRate = size(feasibleTrajectory,1)/param.numSample*100;
    title("Sample Visualization"+" "+"(Up:"+" "+int2str(up)+" Down: "+int2str(down)+")")
    xlabel("Time")
    ylabel("Sample Position")
    axis([param.simStart param.simEnd -10 10])
    hold on

    % plot samples
    if ~isempty(feasibleTrajectory)
        plot(simHorizon,feasibleTrajectory)
        hold on
    end

    if ~isempty(inFeasibleTrajectory)
        plot(simHorizon(1:barrierStep),inFeasibleTrajectory)
        hold on
    end

    % plot barriers
    line([param.barrierTime; param.barrierTime],param.barrierPos,'color','k','LineWidth',2)
    hold on

    % plot initial state
    plot(currentTime,initState,'.r','MarkerSize',20)
    hold on
    