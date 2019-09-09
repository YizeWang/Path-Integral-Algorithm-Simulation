function [trajectory,noiseInput] = computeTrajectory(initState,simHorizon,param,constraintType,reSamPolicy)
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

%% initialize the trajectories
trajectory = zeros(2,timeStep,numSample);
noiseInput = zeros(2,timeStep-1,numSample);
trajectory(:,1,:) = repmat(initState,[1,1,numSample]);

%% compute trajectory based on given resample policy
if constraintType == 0 % no constraints
    for k = 2:timeStep
        for n = 1:numSample
            noiseInput(:,k-1,n) = Q*sqrt(simInterval)*randn(2,1);
            trajectory(:,k,n) = trajectory(:,k-1,n) + G*noiseInput(:,k-1,n);
        end
    end
elseif strcmpi(reSamPolicy,'proj')
    options = optimset('Display','off');
    if constraintType == 1 % input constraints
        for k = 2:timeStep
            for n = 1:numSample
                noiseInput(:,k-1,n) = Q*sqrt(simInterval)*randn(2,1);
                if param.inputConstraint*noiseInput(:,k-1,n)<=[0;0]
                    % constraint satisfied
                else
                    noiseInput(:,k-1,n) = quadprog(param.R,-param.R'*noiseInput(:,k-1,n),param.inputConstraint,[0;0],[],[],[],[],[],options);
                end
                trajectory(:,k,n) = trajectory(:,k-1,n) + G*noiseInput(:,k-1,n);
            end
        end
    elseif constraintType == 2 % state-input constraints
        for k = 2:timeStep
            for n = 1:numSample
                noiseInput(:,k-1,n) = Q*sqrt(simInterval)*randn(2,1);
                if param.stateInputConstraint_e(trajectory(:,k-1,n))+param.stateInputConstraint_F(trajectory(:,k-1,n))*noiseInput(:,k-1,n)<=[0;0]
                    % constraint satisfied
                else
                    noiseInput(:,k-1,n) = quadprog(param.R,-param.R'*noiseInput(:,k-1,n),param.stateInputConstraint_F(trajectory(:,k-1,n)),-param.stateInputConstraint_e(trajectory(:,k-1,n)),[],[],[],[],[],options);
                end
                trajectory(:,k,n) = trajectory(:,k-1,n) + G*noiseInput(:,k-1,n);
            end
        end
    end
elseif strcmpi(reSamPolicy,'trandn')
    q1 = Q(1,1);
    q2 = Q(2,2);
    if constraintType == 2 % state-input constraints
        for k = 2:timeStep
            for n = 1:numSample
                if trajectory(2,k-1,n)==0
                    noiseInput(1,k-1,n) = 0;
                else
                    sigma =  q1*sqrt(simInterval);
                    PhiA = 0.5*(1+erf(((-0.05*abs(trajectory(2,k-1,n)))/sigma)/sqrt(2)));
                    PhiB = 0.5*(1+erf(((0.05*abs(trajectory(2,k-1,n)))/sigma)/sqrt(2)));
                    noiseInput(1,k-1,n) = erfinv(2*(PhiA+rand*(PhiB-PhiA))-1)*sqrt(2)*sigma;
                end
                noiseInput(2,k-1,n) = q2*sqrt(simInterval)*randn(1);
                trajectory(:,k,n) = trajectory(:,k-1,n) + G*noiseInput(:,k-1,n);
            end
        end
    else
        error('Only state-input constraint sovler is implemented.');
    end
elseif strcmpi(reSamPolicy,'rej')
    q1 = Q(1,1);
    q2 = Q(2,2);
    if constraintType == 2 % state-input constraints
        for k = 2:timeStep
            for n = 1:numSample
                noiseInput(1,k-1,n) = q1*sqrt(simInterval)*randn(1);
                noiseInput(2,k-1,n) = q2*sqrt(simInterval)*randn(1);
                attempt = 1;
                while(abs(noiseInput(1,k-1,n))>0.05*abs(trajectory(2,k-1,n)))
                    noiseInput(1,k-1,n) = q1*sqrt(simInterval)*randn(1);
                    attempt = attempt + 1;
                    if attempt>param.maxAttempt
                        if trajectory(2,k-1,n)==0
                            noiseInput(1,k-1,n) = 0;
                            break;
                        else
                            sigma =  q1*sqrt(simInterval);
                            PhiA = 0.5*(1+erf(((-0.05*abs(trajectory(2,k-1,n)))/sigma)/sqrt(2)));
                            PhiB = 0.5*(1+erf(((0.05*abs(trajectory(2,k-1,n)))/sigma)/sqrt(2)));
                            noiseInput(1,k-1,n) = erfinv(2*(PhiA+rand*(PhiB-PhiA))-1)*sqrt(2)*sigma;
                            break;
                        end
                    end
                end
                trajectory(:,k,n) = trajectory(:,k-1,n) + G*noiseInput(:,k-1,n);
            end
        end
    else
        error('Only state-input constraint sovler is implemented.');
    end
else
    error('Unknown resampling policy.');
end

end