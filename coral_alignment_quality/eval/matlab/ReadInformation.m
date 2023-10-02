function ReadInformation = importfile(filename)

fileID = fopen(filename,'r');
formatSpec = '%f';
ReadInformation = fscanf(fileID,formatSpec);

end

