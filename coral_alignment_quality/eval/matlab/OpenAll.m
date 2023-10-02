function [scan_tar_gt, scan_tar, scan_src_gt, scan_src, scan_merged_gt, scan_merged] = OpenAll(dir, max_files)




type = {'/smalloffset', '/gt'}
comp = {'/comp_src_', '/comp_tar_','/comp_merged_'}
i=0;
scan_src={}
scan_tar={}
scan_merged={}
scan_src_gt={}
scan_tar_gt={}
scan_merged_gt={}
while(i<max_files)
    index = num2str(i);
    vals=[];
    
    path=char(strcat(strcat(strcat(dir, type(1)), comp(1)), index));
    S = ReadInformation(path);
    tmp1=[scan_src {S}];
    scan_src=tmp1;
    
    path=char(strcat(strcat(strcat(dir, type(1)), comp(2)), index));
    S = ReadInformation(path);
    tmp1=[scan_tar {S}];
    scan_tar=tmp1;
    
    path=char(strcat(strcat(strcat(dir, type(1)), comp(3)), index));
    S = ReadInformation(path);
    tmp1=[scan_merged {S}];
    scan_merged=tmp1;
    
        path=char(strcat(strcat(strcat(dir, type(2)), comp(1)), index));
    S = ReadInformation(path);
    tmp1=[scan_src_gt {S}];
    scan_src_gt=tmp1;
    
    path=char(strcat(strcat(strcat(dir, type(2)), comp(2)), index));
    S = ReadInformation(path);
    tmp1=[scan_tar_gt {S}];
    scan_tar_gt=tmp1;
    
    path=char(strcat(strcat(strcat(dir, type(2)), comp(3)), index));
    S = ReadInformation(path);
    tmp1=[scan_merged_gt {S}];
    scan_merged_gt=tmp1;
    
    i=i+1;
end

end