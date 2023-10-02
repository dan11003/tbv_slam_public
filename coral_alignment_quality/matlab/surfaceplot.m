clc
clear all
close all

%T = table2array(readtable('/mnt/disk0/challanging_datasets/stairs/global_frame/pcd/surface-stairs-r=0.3.txt'));


%files = dir(fullfile('/mnt/disk0/challanging_datasets/stairs/global_frame/pcd/eval', '*r=0.3*.txt'));
%T = table2array(readtable('/mnt/disk0/challanging_datasets/stairs/global_frame/pcd/small_angle_surface_stairs.txt'));
T = table2array(readtable('/mnt/disk0/challanging_datasets/stairs/global_frame/pcd/large_surface_stairs.txt'));
T2 = table2array(readtable('/mnt/disk0/challanging_datasets/stairs/global_frame/pcd/small_surface_stairs.txt'));
T=[T;T2]

% xData=T(:,1)
% yData=T(:,2)
% zData=-T(:,7)

 xData=T(:,1)
 yData=T(:,2)
 zData=T(:,7)
%scatter3(xData,yData,zData)
%return

% Set up fittype and options.
ft = 'thinplateinterp';

% Fit model to data.
[fitresult, gof] = fit( [xData, yData], zData, ft, 'Normalize', 'on' );

% Plot fit with data.
figure( 'Name', 'fig1' );
%h = plot( fitresult, [xData, yData], zData );
h = plot( fitresult);

hold on
plot(xData,yData,'w.')

%legend( h, 'untitled fit 1', 'Z vs. X, Y', 'Location', 'NorthEast', 'Interpreter', 'none' );
title('Quality Measure')
% Label axes
xlabel( 'x-displacement [m]', 'Interpreter', 'none','fontweight','bold','fontsize',15 );
ylabel( 'theta-displacement [m]', 'Interpreter', 'none','fontweight','bold','fontsize',15 );
%ylabel( 'y-displacement [m]', 'Interpreter', 'none','fontweight','bold','fontsize',15 );
zlabel( 'AlignmentQuality', 'Interpreter', 'none','fontweight','bold','fontsize',15 );
grid on
%view( -0.8, 90.0 );
view( -5, 5);
xticks([min(xData):(max(xData)-min(xData))/4: max(xData) ]);
yticks([min(yData):(max(yData)-min(yData))/4: max(yData) ]);
ax = gca;
ax.FontSize = 15; 
ax.FontWeight = 'bold'; 




colormap(flipud(jet))
hold on
caxis('manual')
caxis([0.2 1.0])
c=colorbar
c.FontSize=15;
c.FontWeight='bold';
%c.LineWidth = 1.5;
%caxis([-0.05 -0.4])