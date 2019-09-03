function [noiseInput] = truncatedGaussian(mu,sigma,lowerBound,upperbound)

oriPDF = makedist('Normal','mu',mu,'sigma',sigma);
newPDF = truncate(oriPDF,lowerBound,upperbound);
noiseInput = random(newPDF);

end