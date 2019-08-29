clc 
clear all
close all

% plotSample function.
initState = [0;0];
currentTime = 0;
load('paramDoubleSlit')
constraintType = 0;

%% load parameters
numSample = param.numSample;
simHorizon = currentTime:param.simInterval:param.simEnd;

%% compute trajectories and collision
[trajectory,~] = computeTrajectory(initState,simHorizon,param,constraintType);
barrierStep = find(simHorizon==param.barrierTime*max(param.barrierX));
isBarrierDetected = detectBarrier(trajectory,param,barrierStep);
feasiblePath = trajectory(:,:,~isBarrierDetected);
inFeasiblePath = trajectory(:,:,isBarrierDetected); 
cost = reshape(0.5*(feasiblePath(1,end,:).^2+feasiblePath(2,end,:).^2),[],1);
sampleWeight = (max(cost)-cost)/(max(cost)-min(cost));
numcolor = size(feasiblePath,3);
colorMat = jet(numcolor);
[~,~,bin] = histcounts(sampleWeight,linspace(0,1,numcolor));

%% plot truncated samples
for n = 1:size(feasiblePath,3)
    plot3(simHorizon,feasiblePath(1,:,n),feasiblePath(2,:,n),'color',colorMat(bin(n),:));
    hold on
end
for n = 1:size(inFeasiblePath,3)
    plot3(simHorizon(1:barrierStep),inFeasiblePath(1,1:barrierStep,n),inFeasiblePath(2,1:barrierStep,n),'color','k');
    hold on
end
colormap(jet);
colorbar('Ticks',[0 1],'TickLabels',{'Low Weight','High Weight'});

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
% 
% x = 1:100;
% y = rand(50,100);
% weight = rand(100,1);
% 
% h = plot(x,y); % There should be 1 handle per line object
% % choose the colormap (eg, jet) and number of levels (eg, 100)
% nLevels = 100; 
% cmat = jet(nLevels); 
% % Assign each weight to a row of color. 
% weightNorm = (weight-min(weight))./(max(weight)-min(weight)); %normalized 0:1
% [~,~,crow] = histcounts(weightNorm,linspace(0,1,nLevels));
% % assign color
% set(h,{'Color'},mat2cell(cmat(crow,:),ones(size(h),3)))