function [noiseInput] = truncatedGaussian(mu,sigma,lowerBound,upperBound)

    PhiA = 0.5*(1+erf((lowerBound-mu)/sigma/sqrt(2)));
    PhiB = 0.5*(1+erf((upperBound-mu)/sigma/sqrt(2)));
    noiseInput = erfinv(2*(PhiA+rand*(PhiB-PhiA))-1)*sqrt(2)*sigma+mu;

end