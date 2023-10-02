%%Load data
clc
clear all
close all


T=[];
base='/mnt/disk0/';
dataset={'orkla_pcds/'  };
FullDataset= {'dairy-production' 'warehouse'};


sufix='logistics_top_competitors/';
T=LoadEvaluation(base, dataset, sufix,'_med');
 

%Twarehouse=Tall{55393:59256,:}
%T=Tall(31777:47664,:)
%T=[Tall(31777:47664,:);Tall(55393:59256,:)];

methods={'entropy-median','entropy','ndtp2d'}%,'rel-ndtp2d'};

EvaluationDataset = FullDataset(1);
TrainingDataset   = FullDataset(2);

for i=1:size(EvaluationDataset,2)
    EvalSet = EvaluationDataset(i); 
    ax=subplot(1,2,i)
    
    EvaluateAndPlot(TrainingDataset,EvalSet,methods,T)
    EvalSet={strrep(EvalSet,'mean-map-entropy','MME')};
    box(ax,'off')
    
    xlabel(strrep(EvalSet{1,1},'_',' '),'fontweight','bold','fontsize',8);
    ylabel('Accuracy','fontweight','bold','fontsize',8)
    set(gca,'fontweight','bold','FontSize',8)
end 
EvaluationDataset = FullDataset(2);
TrainingDataset   = FullDataset(1);
for i=1:size(EvaluationDataset,2)
    EvalSet = EvaluationDataset(i); 
    ax=subplot(1,2,1+i)
    
    EvaluateAndPlot(TrainingDataset,EvalSet,methods,T)
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
