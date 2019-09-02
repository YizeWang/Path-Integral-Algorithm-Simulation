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
%   psi                 value of psi

%% compute cost
cost = inf(param.numSample,1);
cost(~isBarrierDetected) = reshape(0.5*(trajectory(1,end,~isBarrierDetected).^2+trajectory(2,end,~isBarrierDetected).^2),[],1);

%% compute psi
if min(cost) == inf % if all path crashed
    psi = 0;
else % if there exists at least one feasible path
    psi = mean(exp(-1/param.gamma*(cost-min(cost))));
end

end