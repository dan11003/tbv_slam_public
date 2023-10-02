%%Load data
clc
clear all
close all


T=[];
base='/mnt/disk0/challanging_datasets/';
%dataset={'Apartment','stairs','ETH_Hauptgebaude'};
FullDataset       = {'Apartment', 'stairs','ETH_Hauptgebaude','Gazebo_in_summer','Gazebo_in_winter','Mountain_plain','Wood_in_autumn','Wood_in_summer'};
%TrainingDataset   = {'Apartment'};
%EvaluationDataset = {'Apartment'}; %{'Apartment','stairs','ETH_Hauptgebaude','Gazebo_in_summer','Gazebo_in_winter','Mountain_plain','Wood_in_autumn','Wood_in_summer'};

sufix='/global_frame/pcd/3_eval/';
T=LoadEvaluation(base, FullDataset, sufix);
Tfuzzy=LoadEvaluation(base, FullDataset, sufix);
Tfuzzy.Properties.VariableNames = {'Method' 'Merged' 'Separate' 'Differential' 'Dataset' 'Radius' 'Offset_x' 'Offset_y' 'Offset_theta' 'Label'}
T=[T;Tfuzzy]

methods={'entropy','entropy-median','ndtp2d'};
for i=1:8
    
    TrainingDataset   = FullDataset(i);
    EvaluationDataset = FullDataset(i); 
    %subplot(1,9,i)
    
    EvaluateAndPlot(TrainingDataset,EvaluationDataset,methods,T)
    EvaluationDataset={strrep(EvaluationDataset,'mean-map-entropy','MME')};
    pause
 
    %xlabel(strrep(EvaluationDataset{1,1},'_',' '));
    %ylabel('Accuracy')
    %set(gca,'FontSize',18)
end

