function [trajectory,noiseInput] = computeTrajectory(initState,simHorizon,param,constraintType)
% computeTrajectory function. The function will compute the noise-driven
% trajectories starting from the initial state.
%
% Input:
%   initState           a value of current position
%   simHorizon          the simulation steps
%   param               system parameters
%   constraintType      the type of constraint
%                       0: no constraints
%                       1: input constraints
%                       2: state-input constraints
% Output:
%   trajectory          a matrix of rollout trajectories
%   noiseInput          a vector of noise input at first step

%% load parameters
G = param.G;
Q = param.Q;
numSample = param.numSample;

if ~constraintType              % if no constraints applied
    noise = Q*randn(numSample,size(simHorizon,2)-1)*sqrt(param.simInterval);
    noiseInput = noise(:,1);
    trajectory = cumsum([initState*ones(numSample,1) G*noise],2);
elseif constraintType == 1      % if constraints applied on input
    noise = Q*randn(numSample,size(simHorizon,2)-1)*sqrt(param.simInterval);
    maxViolation = G*noise>param.uMax;
    minViolation = G*noise<param.uMin;
    noise(maxViolation) = param.uMax;
    noise(minViolation) = param.uMin;
    trajectory = cumsum([initState*ones(numSample,1) G*noise],2);
    noiseInput = noise(:,1);
elseif constraintType == 2      % if state-input constraints
    numConstraint = size(param.e(0),1);
    trajectory = [initState*ones(numSample,1) zeros(numSample,size(simHorizon,2)-1)];
    noise = Q*randn(numSample,size(simHorizon,2)-1)*sqrt(param.simInterval);
    for t = 1:size(simHorizon,2)-1
        for n = 1:numSample
            e = param.e(trajectory(n,t));
            F = param.F(trajectory(n,t));
            for c = 1:numConstraint
                if e(c)+F(c)*noise(n,t)>0 % projection sampling
                    noise(n,t) = -e(c)/F(c);
                end
            end
            trajectory(n,t+1) = trajectory(n,t) + noise(n,t);
        end
    end
    noiseInput = noise(:,1);
end

end