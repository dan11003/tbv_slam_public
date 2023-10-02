%% Clear all
clc
clear all 
close all

%% Load %%
dir ='/home/daniel/Documents/arla_pcd_output'
[scan_tar_gt_cell, scan_tar_cell, scan_src_gt_cell, scan_src_cell, scan_merged_gt_cell, scan_merged_cell]=OpenAll(dir,107)


%% Calculation
[diff,s,t,m]=DifferentialInformation(scan_src_cell,scan_tar_cell,scan_merged_cell);
[diff_gt,s_gt,t_gt,m_gt]=DifferentialInformation(scan_src_gt_cell,scan_tar_gt_cell,scan_merged_gt_cell);


%% Plot %%
figure(3)
title('RelativeInformation')
plot(diff,'r*')
hold on
plot(diff_gt,'b*')
legend('misaligned','gt')

%% Plot All
figure(1)
plot(s,'r*')
title('misaligned')
hold on
plot(t,'b*')
plot(m,'k*')
figure(2)
plot(s_gt,'r*')
title('Aligned')
hold on
plot(t_gt,'b*')
plot(m_gt,'k*')
%plot(diff_gt,'b.')