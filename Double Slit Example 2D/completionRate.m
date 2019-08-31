function [] = completionRate(percentage)

persistent strCR

if isempty(strCR)
   fprintf('      Completed: ');
end
percentage = floor(percentage);
strOut = [num2str(percentage) '%%\n'];
fprintf([strCR strOut]);
strCR = repmat('\b',1,length(strOut)-2);

end