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
cost = zeros(param.numSample,1);
for n = 1:param.numSample
    cost(n) = 0.5*trajectory(:,end,n)'*param.P*trajectory(:,end,n);
end
cost(isBarrierDetected) = inf;

%% compute psi
if min(cost) == inf % if all path crashed
    psi = 0;
else % if there exists at least one feasible path
    psi = mean(exp(-1/param.gamma*(cost-min(cost))));
end

end