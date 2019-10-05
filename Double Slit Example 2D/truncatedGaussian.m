function [noiseInput] = truncatedGaussian(mu,sigma,lowerBound,upperBound)
% truncatedGaussian function. The function will return a sample drawn from
% the truncated Gaussian distribution.
%
% Input:
%   mu          mean of the Gaussian distribution
%   sigma       standard deviation of the Gaussian distribution
%   lowerBound  lower bound of the truncation
%   upperBound  upper bound of the truncation
% Output:
%   noiseInput  noise input sampled from truncated Gaussian distribution

PhiA = 0.5*(1+erf((lowerBound-mu)/sigma/sqrt(2)));
PhiB = 0.5*(1+erf((upperBound-mu)/sigma/sqrt(2)));
noiseInput = erfinv(2*(PhiA+rand*(PhiB-PhiA))-1)*sqrt(2)*sigma+mu;

end