function [trajectory,noiseInput] = computeTrajectoryTRandn(initState,simHorizon,param,constraintType)
% computeTrajectory function. The function will compute the noise-driven
% trajectories starting from the initial state
%
% Input:
%   initState           a value of current position
%   simHorizon          the simulation steps
%   param               system parameters
%   constraintType      the type of constraints
% Output:
%   trajectory          a matrix of rollout trajectories
%   noiseInput          a vector of noise input at first step

%% load parameters
G = param.G;
Q = param.Q;
numSample = param.numSample;
simInterval = param.simInterval;
timeStep = numel(simHorizon);

%% compute the trajectories
trajectory = zeros(2,timeStep,numSample);
noiseInput = zeros(2,timeStep-1,numSample);
trajectory(:,1,:) = repmat(initState,[1,1,numSample]);

if constraintType == 2 % state-input constraints
    for k = 2:timeStep
        for n = 1:numSample
            noiseInput(:,k-1,n) = Q*sqrt(simInterval)*trandn([-0.05*abs(trajectory(2,k-1,n)) -inf],[0.05*abs(trajectory(2,k-1,n)) inf]);
            trajectory(:,k,n) = trajectory(:,k-1,n) + G*noiseInput(:,k-1,n);
        end
    end
else
    error('Only state-input constraint sovler is implemented.');
end

end