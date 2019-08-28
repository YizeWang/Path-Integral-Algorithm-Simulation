tic
clc
clear all
close all

%% settings
load('paramDoubleSlit');
initState = [0;0];
constraintType = 2;
param.barrierSide = 0.2;
param.stateInputConstraint_e = @(x)[-abs(x(2));-abs(x(2))];
param.stateInputConstraint_F = @(x)[-100 0;100 0];
param.inputConstraint = [1 1;-1 -1];
param.numSample = 20;
param.barrierZ = [10;-10;-10;10];

%% parameter initialization
u = [0;0]; % if no control found at first step, implement 0 control
U = [];
J = [];
Psi = [];
Cost = [];
state = initState;
actualPath = [state];
simHorizon = param.simStart:param.simInterval:param.simEnd;
barrierStep = find(simHorizon==param.barrierTime);

%% display constraint type
switch constraintType
    case 0
        fprintf("Constraint Type: No Constraint \n");
    case 1
        fprintf("Constraint Type: Input Constraints \n");
    case 2
        fprintf("Constraint Type: State-Input Constraints \n");
end

%% compute actual path
for currentTime = param.simStart:param.simInterval:param.simEnd-param.simInterval
    if mod((currentTime-param.simStart)/(param.simEnd-param.simStart),0.1) == 0
        disp("Completed: "+int2str((currentTime-param.simStart)/(param.simEnd-param.simStart)*100)+"%")
    end
    simHorizon = currentTime:param.simInterval:param.simEnd;
    [trajectory,noiseInput] = computeTrajectory(state,simHorizon,param,constraintType);
    if (currentTime > param.barrierTime)        % if behind barrier
        isBarrierDetected = false(param.numSample,1);
    else                                        % if before or at barrier
        isBarrierDetected = detectBarrier(trajectory,param,barrierStep);
    end
    [cost,psi] = computeCost(trajectory,isBarrierDetected,param);
    if psi == 0 % if psi 0, use previous control
        fprintf("Warning: All Samples Crashed at"+num2str(currentTime)+"\n");
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
%% actual path collision test
barrierStep = find(param.simStart:param.simInterval:param.simEnd==param.barrierTime*max(param.barrierX));
isCollision = detectBarrier(actualPath,param,barrierStep);
if isCollision
    fprintf('Actual Path Crashed! \n')
else
    fprintf('Successfully Passed Barrier \n')
end
%% plot figures
% figure initialization
close all
figure('Name',"Double Slit Example 2D")

% plot rollouts
subplot(2,2,1)
plotSample(initState,0,param,constraintType);
hold on
% plot actual path
subplot(2,2,2)
plot3(param.simStart:param.simInterval:param.simEnd,actualPath(1,:),actualPath(2,:));
hold on
% plot barrier
fill3(param.barrierTime*param.barrierX,param.barrierSide*param.barrierY,param.barrierSide*param.barrierZ,'black');
hold on
title("Actual Path")
xlabel("Time")
ylabel("x_1")
zlabel("x_2")
hold on

% plot control input sequence
subplot(2,2,3)
title("|x_2| and 10u_1")
xlabel("Time")
ylabel("|x_2| or 10u_1")
hold on
plot(param.simStart:param.simInterval:param.simEnd-param.simInterval,10*U(1,:))
hold on
plot(param.simStart:param.simInterval:param.simEnd-param.simInterval,abs(actualPath(2,1:end-1)),'color','r','LineWidth',0.5)
hold on
plot(param.simStart:param.simInterval:param.simEnd-param.simInterval,-abs(actualPath(2,1:end-1)),'color','r','LineWidth',0.5)
hold on
legend('10u_1','|x_2|','-|x_2|')

% plot optimal cost
subplot(2,2,4)
title("Optimal Cost")
xlabel("Time")
ylabel("Optimal Cost")
axis([param.simStart param.simEnd 0 1.2*max(J)])
hold on
plot(param.simStart:param.simInterval:param.simEnd-param.simInterval,J)
hold on

toc