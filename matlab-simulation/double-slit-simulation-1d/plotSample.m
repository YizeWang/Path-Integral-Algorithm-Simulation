function [] = plotSample(initState,currentTime,param,constraintType)
% plotSample function. The function will plot all rollouts at any step.
%
% Input:
%   initState           a value of current position
%   currentTime         a value of current step
%   param               system parameters
%   constraintType      the type of constraint
%                       0: no constraints
%                       1: input constraints
%                       2: state-input constraints
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

    %% plot samples
    % figure initialization
    passRate = size(feasibleTrajectory,1)/param.numSample*100;
    title("Sample Visualization"+" "+"(Pass Rate:"+" "+num2str(passRate,3)+"%)",'FontSize',param.fontSize)
    xlabel("Time",'FontSize',param.fontSize)
    ylabel("Sample Position",'FontSize',param.fontSize)
    axis([param.simStart param.simEnd -6 6])
    ax = gca;
    ax.FontSize = param.fontSize;
    hold on
    box on
    hold on

    % plot samples
    if ~isempty(feasibleTrajectory)
        plot(simHorizon,feasibleTrajectory,'LineWidth',param.lineWidth)
        hold on
    end

    if ~isempty(inFeasibleTrajectory)
        plot(simHorizon(1:barrierStep),inFeasibleTrajectory,'LineWidth',param.lineWidth)
        hold on
    end

    % plot barriers
    line([param.barrierTime; param.barrierTime],param.barrierPos,'color','k','LineWidth',8*param.lineWidth)
    hold on

    % plot initial state
    plot(currentTime,initState,'.r','MarkerSize',32*param.lineWidth)
    hold on

    % plot target state
    plot(param.simEnd,0,'.r','MarkerSize',32*param.lineWidth)
    hold on
    
end