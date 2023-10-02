function [diff,s,t,m] = DifferentialInformation(scan_src_cell, scan_tar_cell, scan_merged_cell)
%% Calculation
diff=zeros(1,size(scan_merged_cell,2));
t=zeros(1,size(scan_merged_cell,2));
s=zeros(1,size(scan_merged_cell,2));
m=zeros(1,size(scan_merged_cell,2));
for i=1:size(scan_merged_cell,2)
    scan_mer=nonzeros(cell2mat(scan_merged_cell(i)));
    scan_src=nonzeros(cell2mat(scan_tar_cell(i)));
    scan_tar=nonzeros(cell2mat(scan_src_cell(i)));
    s(i)=mean(scan_src);
    t(i)=mean(scan_tar);
    m(i)=mean(scan_mer);
    separate=(s(i)+t(i))/2.0;
    diff(i)=(m(i)-separate)/separate;
end

end

