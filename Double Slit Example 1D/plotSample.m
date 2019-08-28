function [] = plotSample(initState,currentTime,param,constraintType)
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

    %% plot samples
    % figure initialization
    passRate = size(feasibleTrajectory,1)/param.numSample*100;
    title("Sample Visualization"+" "+"(Pass Rate:"+" "+num2str(passRate,3)+"%)",'FontSize',param.fontSize)
    xlabel("Time",'FontSize',param.fontSize)
    ylabel("Sample Position",'FontSize',param.fontSize)
    axis([param.simStart param.simEnd -10 10])
    ax = gca;
    ax.FontSize = param.fontSize;
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

end