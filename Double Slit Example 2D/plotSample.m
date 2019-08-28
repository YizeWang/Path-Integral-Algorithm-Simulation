function [] = plotSample(initState,currentTime,param,constraintType)
% plotSample function.

%% load parameters
numSample = param.numSample;
simHorizon = currentTime:param.simInterval:param.simEnd;

%% compute trajectories and collision
[trajectory,~] = computeTrajectory(initState,simHorizon,param,constraintType);
barrierStep = find(simHorizon==param.barrierTime*max(param.barrierX));
isBarrierDetected = detectBarrier(trajectory,param,barrierStep);

%% plot truncated samples
for n = 1:numSample
    if isBarrierDetected(n) % plot truncated trajectory for crashing ones
        plot3(simHorizon(1:barrierStep),trajectory(1,1:barrierStep,n),trajectory(2,1:barrierStep,n));
        hold on
    else % plot complete trajectory
        plot3(simHorizon,trajectory(1,:,n),trajectory(2,:,n));
        hold on
    end
end

%% plot barrier
fill3(param.barrierTime*param.barrierX,param.barrierSide*param.barrierY,param.barrierSide*param.barrierZ,'black');
hold on

%% figure settings
passRate = (1-sum(isBarrierDetected)/param.numSample)*100;
title("Sample Visualization"+" "+"(Pass Rate:"+" "+num2str(passRate,3)+"%)")
xlabel("Time")
ylabel("x_1")
zlabel("x_2")
hold on

%% plot initial state
plot3(currentTime,initState(1),initState(2),'.k','MarkerSize',20)
hold on

end