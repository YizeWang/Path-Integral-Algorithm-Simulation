function [cost,psi] = computeCost(trajectory,isBarrierDetected,param)
% computeCost function. The function will compute the cost accumulated
% along each path. If the path goes through a barrier, the cost would be
% infinite.
% 
% Input:
%   trajectory          a matrix of rollout trajectories
%   isBarrierDetected   a vector denotes whether path goes through barrier
%   param               system parameters
% Output:
%   cost                a vector of cost accumulated along each path

%% compute cost
cost = inf(param.numSample,1);
cost(~isBarrierDetected) = 0.5*trajectory(~isBarrierDetected,end).^2;

%% compute psi
if min(cost) == inf % if all path crashed
    psi = 0;
else                % if there exists at least one feasible path
    psi = mean(exp(-1/param.gamma*(cost-min(cost)))); %softmax
end

end