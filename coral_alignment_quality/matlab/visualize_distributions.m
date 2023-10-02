clear all
close all
clc
%base='/mnt/disk0/'
%öleardataset={'orkla_pcds'}
%sufix={''}
%base='/mnt/disk0/challanging_datasets/';
%dataset={'ETH_Hauptgebaude'}%,'Apartment','Gazebo_in_summer','Gazebo_in_winter','Mountain_plain','Wood_in_autumn','Wood_in_summer'};
%sufix={'/global_frame/pcd/no_inconsistencies/','/global_frame/pcd/with_inconsistencies/'}
T=[]
m_err=[]
src_err=[]
tar_err=[]
m_al=[]
src_al=[]
tar_al=[]
%for set=1:size(dataset,2)
%d =strcat(base,dataset{set},sufix{1});
%d='/mnt/disk0/orkla_pcds/';
%d='/mnt/disk0/challanging_datasets/Apartment/global_frame/pcd/no_inconsistencies/'
d='/mnt/disk0/orkla_pcds/csv_output/'
m_err=ReadAllCSVRegularExp(d,'*merged*error*.csv');
sep_err=[ReadAllCSVRegularExp(d,'*src*error*.csv') ReadAllCSVRegularExp(d,'*tar*error*.csv')];

m_al=ReadAllCSVRegularExp(d,'*merged*aligned*.csv');
sep_al=[ReadAllCSVRegularExp(d,'*tar*aligned*.csv') ReadAllCSVRegularExp(d,'*src*aligned*.csv')];

%end
%%% mean

%%
%min_limit = -8
%m_err=m_err(m_err>min_limit)
%sep_err=sep_err(sep_err>min_limit)
%m_al=m_al(m_al>min_limit);
%sep_al=sep_al(sep_al>min_limit);

merged_error=mean(m_err);
separate_error=mean(sep_err);
errordiff = merged_error - separate_error

merged_aligned=mean(m_al);
separate_aligned=mean(sep_al);
aligned_diff=merged_aligned-separate_aligned

merged_error_median=median(m_err);
separate_error_median=median(sep_err);
errordiff_median = merged_error_median - separate_error_median

merged_aligned_median=median(m_al);
separate_aligned_median=median(sep_al);
aligned_diff_median=merged_aligned_median-separate_aligned_median

% m_err=rmoutliers(m_err,'percentiles',[20 98]);
% sep_err=rmoutliers(sep_err,'percentiles',[20 98]);
% m_al=rmoutliers(m_al,'percentiles',[20 98]);
% sep_al=rmoutliers(sep_al,'percentiles',[20 98]);
%
% merged_error=median(m_err);
% separate_error=median(sep_err);
% errordiff = merged_error - separate_error
%
% merged_aligned=median(m_al);
% separate_aligned=median(sep_al);
% aligned_diff=merged_aligned-separate_aligned
%
%
% errordiff = merged_error - separate_error
% aligned_diff=merged_aligned-separate_aligned

%% Plot


