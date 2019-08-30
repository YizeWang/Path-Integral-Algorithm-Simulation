function [trajectory,noiseInput] = computeTrajectory(initState,simHorizon,param,constraintType)
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
options = optimset('Display','off');
stateInputConstraint_e = param.stateInputConstraint_e;
stateInputConstraint_F = param.stateInputConstraint_F;
inputConstraint = param.inputConstraint;

%% compute the trajectories
trajectory = zeros(2,timeStep,numSample);
noiseInput = zeros(2,timeStep-1,numSample);
trajectory(:,1,:) = repmat(initState,[1,1,numSample]);

if constraintType == 0 % no constraints
    for k = 2:timeStep
        for n = 1:numSample
            noiseInput(:,k-1,n) = Q*sqrt(simInterval)*randn(2,1);
            trajectory(:,k,n) = trajectory(:,k-1,n) + G*noiseInput(:,k-1,n);
        end
    end
elseif constraintType == 1 % input constraints
    for k = 2:timeStep
        for n = 1:numSample
            noiseInput(:,k-1,n) = Q*sqrt(simInterval)*randn(2,1);
            if inputConstraint*noiseInput(:,k-1,n)<=[0;0]
                % constraint satisfied
            else
                noiseInput(:,k-1,n) = quadprog(param.R,-param.R'*noiseInput(:,k-1,n),inputConstraint,[0;0],[],[],[],[],[],options);
            end
            trajectory(:,k,n) = trajectory(:,k-1,n) + G*noiseInput(:,k-1,n);
        end
    end
elseif constraintType == 2 % state-input constraints
    for k = 2:timeStep
        for n = 1:numSample
            noiseInput(:,k-1,n) = Q*sqrt(simInterval)*randn(2,1);
            if stateInputConstraint_e(trajectory(:,k-1,n))+stateInputConstraint_F(trajectory(:,k-1,n))*noiseInput(:,k-1,n)<=[0;0]
                % constraint satisfied
            else
                noiseInput(:,k-1,n) = quadprog(param.R,-param.R'*noiseInput(:,k-1,n),stateInputConstraint_F(trajectory(:,k-1,n)),-stateInputConstraint_e(trajectory(:,k-1,n)),[],[],[],[],[],options);
            end
            trajectory(:,k,n) = trajectory(:,k-1,n) + G*noiseInput(:,k-1,n);
        end
    end
end

end