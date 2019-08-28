
close all

for i = 1:0.5:3.5

initState = 0;
isConstrained = false;
load('paramDoubleSlit')
param.numSample = 50000;
param.barrierPos = [-4,i,20;
                    -20,-i,4];
param.uMin = -0.03;
param.uMax = 0.2;
fontSize = 16;

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
    [trajectory,noiseInput] = computeTrajectory(state,simHorizon,param,isConstrained);
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

% plot actual path
title("Actual Path",'FontSize',param.fontSize)
xlabel("Time",'FontSize',param.fontSize)
ylabel("Position",'FontSize',param.fontSize)
axis([param.simStart param.simEnd -4 4])
ax = gca;
ax.FontSize = param.fontSize;
hold on
plot(param.simStart:param.simInterval:param.simEnd,actualPath)
hold on
end
legend('barrier length 2','barrier length 3','barrier length 4','barrier length 5','barrier length 6','barrier length 7','FontSize',param.fontSize)