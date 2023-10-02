clc
clear all
close all

r=0.3;
T=[];
base='/mnt/disk0/challanging_datasets/';
%dataset={'Apartment','stairs','ETH_Hauptgebaude'}
dataset={'Apartment','stairs','ETH_Hauptgebaude','Gazebo_in_summer','Gazebo_in_winter','Mountain_plain','Wood_in_autumn','Wood_in_summer'};
sufix='/global_frame/pcd/eval_ndt/'
for set=1:size(dataset,2)
    d =strcat(base,dataset{set},sufix);
    files = dir(fullfile(d, strcat('*.txt')));
    for i=1:size(files,1)
        name=files(i).name
        T=[T; readtable(strcat(d,name))];
    end
end



%% ENTROPY
method='entropy'
Tent=T(strcmp(T.Method,method),:)
Tndt=T(strcmp(T.Method,'ndtp2d'),:)
r=0.1

mer_al=table2array(T(T.Label==1&strcmp(T.Method,method)&T.Radius==r,'Merged'))
sep_al=table2array(T(T.Label==1&strcmp(T.Method,method)&T.Radius==r,'Separate'))
al_diff=table2array(T(T.Label==1&strcmp(T.Method,method)&T.Radius==r,'Differential'))

mer_error=table2array(T(T.Label==0&strcmp(T.Method,method)&T.Radius==r,'Merged'))
sep_error=table2array(T(T.Label==0&strcmp(T.Method,method)&T.Radius==r,'Separate'))
err_diff=table2array(T(T.Label==0&strcmp(T.Method,method)&T.Radius==r,'Differential'))

figure(1)
plot(sep_al,mer_al,'r*')
hold on
plot(sep_error,mer_error,'bx')
%axis equal
title('Entropy')
figure(2)
plot(al_diff,'r*')

hold on
plot(err_diff,'bx')
title('Entropy')

return
%% NDT
figure(3)
method='ndtp2d'
mer_al_ndt=table2array(T(T.Label==1&strcmp(T.Method,method),'Merged'))
sep_al_ndt=table2array(T(T.Label==1&strcmp(T.Method,method),'Separate'))
al_diff_ndt=table2array(T(T.Label==1&strcmp(T.Method,method),'Differential'))

mer_error_ndt=table2array(T(T.Label==0&strcmp(T.Method,method),'Merged'))
sep_erro_ndtr=table2array(T(T.Label==0&strcmp(T.Method,method),'Separate'))
err_diff_ndt=table2array(T(T.Label==0&strcmp(T.Method,method),'Differential'))


plot(sep_al,mer_al,'r*')
hold on
plot(sep_error,mer_error,'bx')
axis equal
figure(4)
plot(al_diff_ndt,'r*')

hold on
plot(err_diff_ndt,'bx')


