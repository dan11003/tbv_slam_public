%%Load data
clc
clear all
close all


T=[];
base='/mnt/disk0/challanging_datasets/';
%dataset={'Apartment','stairs','ETH_Hauptgebaude'};
FullDataset       = {'Apartment', 'stairs','ETH_Hauptgebaude','Gazebo_in_summer','Gazebo_in_winter','Mountain_plain','Wood_in_autumn','Wood_in_summer'};


sufix='/global_frame/pcd/full_eval/';
T=LoadEvaluation(base, FullDataset, sufix,'_med');
sufix='/global_frame/pcd/fuzzybnb/'

Tfuzzy=LoadEvaluation(base, FullDataset, sufix,'_med');
Tfuzzy.Properties.VariableNames = {'Method' 'Merged' 'Separate' 'Differential' 'Dataset' 'Radius' 'Offset_x' 'Offset_y' 'Offset_theta' 'Label'}
for i=1:size(Tfuzzy,1)
    Tfuzzy{i,2}=Tfuzzy{i,4};
    Tfuzzy{i,3}=0;
end
T=[T;Tfuzzy]

methods={'entropy-median','entropy','mean-map-entropy','ndtp2d','FuzzyQA', 'rel-ndtp2d'};
methods_renamed={'CorAl','CorAl-median','FuzzyQA','MME','NDT', 'Rel-NDT'};

EvaluationDataset = FullDataset(1:3);
TrainingDataset   = FullDataset(4:8);
Tfull=[]
for i=1:size(EvaluationDataset,2)
    EvalSet = EvaluationDataset(i); 
    ax=subplot(2,4,i)
    
    Tthis=EvaluateAndPlot(TrainingDataset,EvalSet,methods,T)
    Tfull=[Tfull;Tthis]
    EvalSet={strrep(EvalSet,'mean-map-entropy','MME')};
    box(ax,'off')
    
    xlabel(strrep(EvalSet{1,1},'_',' '),'fontweight','bold','fontsize',8);
    ylabel('Accuracy','fontweight','bold','fontsize',8)
    set(gca,'fontweight','bold','FontSize',8)
end 
EvaluationDataset = FullDataset(4:8);
TrainingDataset   = FullDataset(1:3);
for i=1:size(EvaluationDataset,2)
    EvalSet = EvaluationDataset(i); 
    ax=subplot(2,4,3+i)
    
    
    Tthis=EvaluateAndPlot(TrainingDataset,EvalSet,methods,T)
    Tfull=[Tfull;Tthis]
    EvalSet={strrep(EvalSet,'mean-map-entropy','MME')};
    box(ax,'off')
    xlabel(strrep(EvalSet{1,1},'_',' '),'fontweight','bold','fontsize',8);
    ylabel('Accuracy','fontweight','bold','fontsize',8)
    set(gca,'fontweight','bold','FontSize',8)
end

box off
x0=10;
y0=10;
width=1000;
height=500
set(gcf,'position',[x0,y0,width,height])
tot=[]
Tfull=Tfull(~strcmp(Tfull.EvaluationData,'Wood_in_summer')&~strcmp(Tfull.EvaluationData,'Wood_in_autumn'),:)
for i=1:size(methods_renamed,2)
    Tmethod=Tfull(strcmp(Tfull.Method,methods_renamed(i)),:)
    acc_i=Tmethod{:,1}
    size_i=Tmethod{:,5}
    tot=[tot dot(acc_i,size_i)/sum(size_i)]
end