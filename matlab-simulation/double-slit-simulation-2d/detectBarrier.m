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

%% load parameters
barrierY = param.barrierSide*param.barrierY;
barrierZ = param.barrierSide*param.barrierZ;
numSample = size(trajectory,3);
isBarrierDetected = false(numSample,1);

%% detect barrier for all samples
for n = 1:numSample
    % read the trajectory position at the barrier step
    trajPos = trajectory(:,barrierStep,n);
    % if the trajectory is inside a barrier, return true
    isBarrierDetected(n) = trajPos(1)>min(barrierY) && trajPos(1)<max(barrierY) && trajPos(2)>min(barrierZ) && trajPos(2)<max(barrierZ);
end

end