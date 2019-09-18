clc
clear all
close all

ActualPath = [];
halfLength = [0.3 0.5 0.7 1.0 1.5 2.0 2.5 3.0 3.5];

for i = halfLength

initState = 0;
isConstrained = false;
load('paramDoubleSlit')
param.numSample = 50000;
param.barrierPos = [-4,i,20;
                    -20,-i,4];
param.uMin = -0.1;
param.uMax = 0.2;

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

ActualPath = [ActualPath; actualPath];

end

%% 
figure('Name','Symmetry Breaking Illustration')
title("Actual Path",'FontSize',param.fontSize)
xlabel("Time",'FontSize',param.fontSize)
ylabel("Position",'FontSize',param.fontSize)
axis([param.simStart param.simEnd -4 4])
ax = gca;
ax.FontSize = param.fontSize;
hold on
plot(param.simStart:param.simInterval:param.simEnd,ActualPath(1,:),'lineWidth',1,'color','r');
param.barrierPos = [-4,halfLength(1),20;
                    -20,-halfLength(1),4];
line([param.barrierTime; param.barrierTime],param.barrierPos,'color','k','LineWidth',2);
pause(10);

for n = 2:size(ActualPath,1)
    plot(param.simStart:param.simInterval:param.simEnd,ActualPath(1:n-1,:),'lineWidth',1,'color',[0.7 0.7 0.7]);
    param.barrierPos = [-4,halfLength(n),20;
                        -20,-halfLength(n),4];
    line([param.barrierTime; param.barrierTime],param.barrierPos,'color','k','LineWidth',2)
    plot(param.simStart:param.simInterval:param.simEnd,ActualPath(n,:),'lineWidth',1,'color','r');
    pause(10);
end

%%
modAcutualPath = ActualPath;
for n = 1:size(ActualPath,1)
    if ActualPath(n,barrierStep)<0
        modAcutualPath(n,:) = -modAcutualPath(n,:);
    end
end
figure('Name','Modified Symmetry Breaking Illustration')
title("Actual Path",'FontSize',param.fontSize)
xlabel("Time",'FontSize',param.fontSize)
ylabel("Position",'FontSize',param.fontSize)
axis([param.simStart param.simEnd -4 4])
ax = gca;
ax.FontSize = param.fontSize;
hold on
plot(param.simStart:param.simInterval:param.simEnd,modAcutualPath,'lineWidth',1);
legend('barrier length 0.6','barrier length 1.0','barrier length 1.4','barrier length 2.0','barrier length 3.0','barrier length 4.0','barrier length 5.0','barrier length 6.0','barrier length 7.0','FontSize',param.fontSize)