function T = ReadAllCSVRegularExp(directory, expression)
files = dir(fullfile(directory, strcat(expression)))
T=[];

for i=1:size(files,1)
    name=files.name
    T=[T csvread(strcat(directory,name))];
end
end

