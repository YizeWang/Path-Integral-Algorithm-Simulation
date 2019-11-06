function [u] = computeU(cost,psi,noiseInput,param)
% computeU function. The function will compute the optimal control input.
%
% Input:
%   cost                a vector of cost associated with each trajectory
%   psi                 the value of psi
%   noiseInput          sampled noise inputs
%   param               system parameters
% Output:
%   u                   optimal control input

%% compute u
unWeightedU = zeros(2,size(noiseInput,2));
for n = 1:param.numSample
    unWeightedU = unWeightedU + noiseInput(:,:,n).*exp(-1/param.gamma*(cost(n)-min(cost)));
end
u = 1/psi*unWeightedU/param.numSample;

end