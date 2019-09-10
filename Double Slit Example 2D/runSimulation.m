clc
close all
clear all

%% settings
load('paramDoubleSlit');
timeStart = tic;
initState = [0;0];
constraintType = 0;
reSamPolicy = 'trandn'; % proj / trandn / rej
param.numSample = 100;
param.maxAttempt = 10;
param.barrierSide = 0.2;
param.barrierZ = [10;-10;-10;10];
param.P = [1 0;0 1];
param.inputConstraint = [1 1;-1 -1];
param.stateInputConstraint_F = @(x)[-20 0;20 0];
param.stateInputConstraint_e = @(x)[-abs(x(2));-abs(x(2))];

%% parameter initialization
simHorizon = param.simStart:param.simInterval:param.simEnd;
u = zeros(2,numel(simHorizon)-1); % in case no control found at first time step
U = [];
J = [];
Psi = [];
Cost = [];
state = initState;
actualPath = [state];
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

%% determine independence of inputs in constraints
if ~isdiag(param.Q*param.Q')
    error('Constraints must be decoupled on inputs.')
end

%% compute actual path
for currentTime = param.simStart:param.simInterval:param.simEnd-param.simInterval
    % display process bar
    completionRate(numel(param.simStart:param.simInterval:currentTime)/numel(param.simStart:param.simInterval:param.simEnd-param.simInterval)*100);
    simHorizon = currentTime:param.simInterval:param.simEnd;
    [trajectory,noiseInput] = computeTrajectory(state,simHorizon,param,constraintType,reSamPolicy);
    if (currentTime > param.barrierTime)        % if behind barrier
        isBarrierDetected = false(param.numSample,1);
    else                                        % if before or at barrier
        isBarrierDetected = detectBarrier(trajectory,param,barrierStep);
    end
    [cost,psi] = computeCost(trajectory,isBarrierDetected,param);
    if psi == 0 % if psi 0, use previous control
        u = u(:,2:end);
        fprintf(['Warning: All Samples Crashed at ' num2str(currentTime) '\n']);
    else % if psi !0, compute new control with path integral
        u = computeU(cost,psi,noiseInput,param);
    end
    state = state + param.G*u(:,1);
    barrierStep = barrierStep - 1;
    
    % record data
    actualPath = [actualPath state];
    U = [U u(:,1)];
    J = [J -param.gamma*log(psi)];
    Cost = [Cost cost];
    Psi = [Psi psi];
end

%% actual path collision test
barrierStep = find(param.simStart:param.simInterval:param.simEnd==param.barrierTime*max(param.barrierX));
isCollision = detectBarrier(actualPath,param,barrierStep);
if isCollision
    fprintf('         Result: Actual Path Crashed! \n')
else
    fprintf('         Result: Successfully Passed Barrier \n')
end

%% plot figures
% figure initialization
close all
figure('Name',"Double Slit Example 2D")

% plot rollouts
subplot(2,2,1)
plotSample(initState,0,param,constraintType,reSamPolicy);
hold on
% plot actual path
subplot(2,2,2)
plot3(param.simStart:param.simInterval:param.simEnd,actualPath(1,:),actualPath(2,:),'LineWidth',1);
hold on
% plot barrier
fill3(param.barrierTime*param.barrierX,param.barrierSide*param.barrierY,param.barrierSide*param.barrierZ,'black');
hold on
title("Actual Path",'fontsize',param.fontSize)
axisLimit = 1.5*param.barrierSide*max(abs([param.barrierY(:);param.barrierZ(:)]));
axis ([0 param.simEnd -axisLimit axisLimit -axisLimit axisLimit])
xlabel("t",'fontsize',param.fontSize)
ylabel("x",'fontsize',param.fontSize)
zlabel("z",'fontsize',param.fontSize)
hold on

% plot control input sequence
subplot(2,2,3)
title("Constraints",'fontsize',param.fontSize)
xlabel("t",'fontsize',param.fontSize)
ylabel("20u_x",'fontsize',param.fontSize)
hold on
plot(param.simStart:param.simInterval:param.simEnd-param.simInterval,20*U(1,:),'LineWidth',1)
hold on
plot(param.simStart:param.simInterval:param.simEnd-param.simInterval,abs(actualPath(2,1:end-1)),'color','r','LineWidth',1)
hold on
plot(param.simStart:param.simInterval:param.simEnd-param.simInterval,-abs(actualPath(2,1:end-1)),'color','r','LineWidth',1)
hold on
legend('20u_x','±|z|','fontsize',param.fontSize)

% plot optimal cost
subplot(2,2,4)
title("Optimal Cost",'fontsize',param.fontSize)
xlabel("Time",'fontsize',param.fontSize)
ylabel("Optimal Cost",'fontsize',param.fontSize)
axis([param.simStart param.simEnd 0 1.2*max(J)])
hold on
plot(param.simStart:param.simInterval:param.simEnd-param.simInterval,J)
hold on

%% plot three view drawing (no title)
figure('Name','Side View');
plot(param.simStart:param.simInterval:param.simEnd,actualPath(2,:),'LineWidth',1);
xlabel("t",'fontsize',param.fontSize)
ylabel("z",'fontsize',param.fontSize)
hold on
line([param.barrierX(1) param.barrierX(1)],[param.barrierSide*min(param.barrierZ) param.barrierSide*max(param.barrierZ)],'color','k','LineWidth',2);
axis([0 2 -2 2])
hold on
figure('Name','Illustrative Diagram');
plot3(param.simStart:param.simInterval:param.simEnd,actualPath(1,:),actualPath(2,:),'LineWidth',1);
hold on
fill3(param.barrierTime*param.barrierX,param.barrierSide*param.barrierY,param.barrierSide*param.barrierZ,'black'); % plot barrier
hold on
axisLimit = 1.5*param.barrierSide*max(abs([param.barrierY(:);param.barrierZ(:)]));
axis ([0 param.simEnd -axisLimit axisLimit -axisLimit axisLimit])
xlabel("t",'fontsize',param.fontSize)
ylabel("x",'fontsize',param.fontSize)
zlabel("z",'fontsize',param.fontSize)
hold on
figure('Name','Top View');
plot(param.simStart:param.simInterval:param.simEnd,actualPath(1,:),'LineWidth',1);
hold on
line([param.barrierX(1) param.barrierX(1)],[param.barrierSide*min(param.barrierY) param.barrierSide*max(param.barrierY)],'color','k','LineWidth',2);
xlabel("t",'fontsize',param.fontSize)
ylabel("x",'fontsize',param.fontSize)
hold on
axis([0 2 -0.3 0.3])
figure('Name','Constraints Satisfaction');
xlabel("t",'fontsize',param.fontSize)
ylabel("20u_x",'fontsize',param.fontSize)
hold on
plot(param.simStart:param.simInterval:param.simEnd-param.simInterval,20*U(1,:),'LineWidth',1)
hold on
plot(param.simStart:param.simInterval:param.simEnd-param.simInterval,abs(actualPath(2,1:end-1)),'color','r','LineWidth',1)
hold on
plot(param.simStart:param.simInterval:param.simEnd-param.simInterval,-abs(actualPath(2,1:end-1)),'color','r','LineWidth',1)
hold on
legend('20u_x','±|z|','fontsize',param.fontSize)

% print elapsed time
fprintf(['   Time Elapsed: ' num2str(toc(timeStart)) ' seconds\n']);
fprintf([' Terminal State: [' num2str(actualPath(1,end)) ' ' num2str(actualPath(2,end)) ']\n'])