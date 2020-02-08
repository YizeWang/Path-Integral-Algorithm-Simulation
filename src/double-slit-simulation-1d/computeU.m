function [u] = computeU(cost,psi,noiseInput,param)
% computeU function. The function will compute the optimal control input.
%
% Input:
%   cost                a vector of cost associated with each trajectory
%   psi                 the value of psi
%   noiseInput          a vector of noise input at first step
%   param               system parameters
% Output:
%   u                   optimal control input

u = 1/psi*mean(noiseInput.*exp(-1/param.gamma*(cost-min(cost)))); % softmax

end