clc
close all
clear all
load('paramDoubleSlit');

% settings
initState = [0.2;1];
currentTime = 1;
constraintType = 2;
param.numSample = 1000;

% overwrite parameters
param.maxAttempt = 10;
param.barrierSide = 0.2;
param.barrierZ = [10;-10;-10;10];
param.inputConstraint = [1 1;-1 -1];
param.stateInputConstraint_F = @(x)[-20 0;20 0];
param.stateInputConstraint_e = @(x)[-abs(x(2));-abs(x(2))];

%% plot histogram only
numSample = 1e5;
edge = linspace(-1,1,20+1);
lowerBound = -1;
upperBound = 1;
mu = 0;
sigma = 1;
noiseInput1 = randn(1,numSample);
noiseInput2 = zeros(1,numSample);
noiseInput3 = zeros(1,numSample);
% re-projection sampling
noiseInput1(noiseInput1>upperBound) = upperBound;
noiseInput1(noiseInput1<lowerBound) = lowerBound;
% truncated Gaussian sampling
for n = 1:numSample
    noiseInput2(n) = truncatedGaussian(mu,sigma,lowerBound,upperBound);
end
% rejection sampling
for n = 1:numSample
    noiseInput3(n) = randn;
    while noiseInput3(n)>upperBound||noiseInput3(n)<lowerBound
        noiseInput3(n) = randn;
    end
end
% plot histograms
figure('Name','Sampling Policy Comparison')
subplot(2,3,1)
histogram(noiseInput1,edge);
% title("Re-Projection",'fontsize',param.fontSize)
xlabel("Value of Samples",'fontsize',param.fontSize)
ylabel("Number of Samples",'fontsize',param.fontSize)
ylim([0 6000])
subplot(2,3,2)
histogram(noiseInput2,edge);
% title("Truncated Gaussian",'fontsize',param.fontSize)
xlabel("Value of Samples",'fontsize',param.fontSize)
ylabel("Number of Samples",'fontsize',param.fontSize)
subplot(2,3,3)
histogram(noiseInput3,edge);
% title("Rejection Sampling",'fontsize',param.fontSize)
xlabel("Value of Samples",'fontsize',param.fontSize)
ylabel("Number of Samples",'fontsize',param.fontSize)

%% plot samples and histogram
figure('Name','Sampling Policy Comparison with Paths')
edge = linspace(-0.1,0.1,40);
% re-projection
tStart = tic;
subplot(2,3,1)
[Tra1,Temp1] = plotSample(initState,currentTime,param,constraintType,'proj');
Temp12 = Temp1(1,1,:);
noiseInput1 = Temp12(:);
title("Re-Projection",'fontsize',param.fontSize)
hold on
subplot(2,3,4)
histogram(noiseInput1,edge)
title("u_1 Histogram",'fontsize',param.fontSize)
fprintf(['Reprojection Sampling: ' num2str(toc(tStart)) ' seconds\n']);
% truncated Gaussian
tStart = tic;
subplot(2,3,2)
[Tra2,Temp2] = plotSample(initState,currentTime,param,constraintType,'trandn');
Temp22 = Temp2(1,1,:);
noiseInput2 = Temp22(:);
title("Truncated Gaussian",'fontsize',param.fontSize)
hold on
subplot(2,3,5)
histogram(noiseInput2,edge)
title("u_1 Histogram",'fontsize',param.fontSize)
hold on
fprintf(['Truncated Normal Distribution: ' num2str(toc(tStart)) ' seconds\n']);
% rejection
tStart = tic;
subplot(2,3,3)
[Tra3,Temp3] = plotSample(initState,currentTime,param,constraintType,'rej');
Temp32 = Temp3(1,1,:);
noiseInput3 = Temp32(:);
title("Rejection",'fontsize',param.fontSize)
hold on
subplot(2,3,6)
histogram(noiseInput3,edge)
title("u_1 Histogram",'fontsize',param.fontSize)
hold on
fprintf(['Rejection Sampling: ' num2str(toc(tStart)) ' seconds\n']);