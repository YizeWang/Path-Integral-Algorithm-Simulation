function [] = completionRate(percentage)
% completionRate function. The function prints the completion rate.
% 
% Input:
%   percentage      completion percentage
% Output:
%   print completion percentage to command window

percentage = floor(percentage);
strOut = [num2str(percentage) '%%\n'];
fprintf(['      Completed: ' strOut]);

end