function [T]=LoadEvaluation(base,dataset,sufix,size_error)
%base='/mnt/disk0/challanging_datasets/';
%dataset={'Apartment','stairs','ETH_Hauptgebaude'}
%dataset={'Apartment','stairs','ETH_Hauptgebaude','Gazebo_in_summer','Gazebo_in_winter','Mountain_plain','Wood_in_autumn','Wood_in_summer'};
%sufix='/global_frame/pcd/eval_ndt/'
T=[];
for set=1:size(dataset,2)
    d =strcat(base,dataset{set},sufix);

   
   files = dir(fullfile(d, strcat('*',size_error,'*txt')));

    
    for i=1:size(files,1)
        name=files(i).name;
        T=[T; readtable(strcat(d,name))];
    end
end