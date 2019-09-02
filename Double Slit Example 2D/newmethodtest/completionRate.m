function [] = completionRate(percentage)

percentage = floor(percentage);
strOut = [num2str(percentage) '%%\n'];
fprintf(['      Completed: ' strOut]);

end