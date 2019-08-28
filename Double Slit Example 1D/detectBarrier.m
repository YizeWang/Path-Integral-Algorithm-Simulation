function [isBarrierDetected] = detectBarrier(trajectory,param,barrierStep)
% detectBarrier function. The function will check along each path whether
% the trajectory goes through a barrier.
%
% Input:
%   trajectory          a matrix of rollout trajectories
%   param               system parameters
%   barrierStep         time step of barrier
% Output:
%   isBarrierDetected   a vector of booleans indicates collision

% load parameters
barrierPos = param.barrierPos;
numSample = size(trajectory,1);
numBarrier = size(barrierPos,2);
isBarrierDetected = false(numSample,1);

% loop over all barriers
for i = 1:numBarrier
    % read the trajectory position at a barrier
    trajectoryPosition = trajectory(:,barrierStep);
    % if the trajectory is inside a barrier, return true
    isBarrierDetected = isBarrierDetected | (trajectoryPosition-barrierPos(1,i)).*(trajectoryPosition-barrierPos(2,i))<=0;
end

end
