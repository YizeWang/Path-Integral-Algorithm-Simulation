function [noiseInput] = truncatedGaussian(mu,sigma,lowerBound,upperBound)

PhiA = (lowerBound-mu)/sigma;
PhiB = (upperBound-mu)/sigma;
PhiX = PhiA+rand*(PhiB-PhiA);
noiseInput = erfinv(2*PhiX-1)*sqrt(2)*sigma+mu;

end