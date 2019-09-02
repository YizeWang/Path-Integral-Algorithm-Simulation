function [u] = computeU(cost,psi,noiseInput,param)
% computeU function. The function will compute the optimal control input.
%
% Input:
%   cost                a vector of cost associated with each trajectory
%   psi                 the value of psi
%   param               system parameters
%   prevU               previous optimal input
% Output:
%   u                   optimal control input
%   isControlFound      whether optimal control input found

%% compute u
unWeightedU = zeros(2,size(noiseInput,2));
for n = 1:param.numSample
    unWeightedU = unWeightedU + noiseInput(:,:,n).*exp(-1/param.gamma*(cost(n)-min(cost)));
end
u = 1/psi*unWeightedU/param.numSample;

end