%%Load data
clc
clear all
close all


T=[];
base='/mnt/disk0/challanging_datasets/';
%dataset={'Apartment','stairs','ETH_Hauptgebaude'};
FullDataset       = {'Apartment', 'stairs','ETH_Hauptgebaude','Gazebo_in_summer','Gazebo_in_winter','Mountain_plain','Wood_in_autumn','Wood_in_summer'};
%FullDataset       = {'ETH_Hauptgebaude'}



sufix='/global_frame/pcd/full_eval/';
T=LoadEvaluation(base, FullDataset, sufix,'_med');
sufix='/global_frame/pcd/fuzzybnb/'
 
Tfuzzy=LoadEvaluation(base, FullDataset, sufix,'med');
Tfuzzy.Properties.VariableNames = {'Method' 'Merged' 'Separate' 'Differential' 'Dataset' 'Radius' 'Offset_x' 'Offset_y' 'Offset_theta' 'Label'}
T=[T;Tfuzzy]
Tfull=[]
methods={'entropy-median','entropy','mean-map-entropy','ndtp2d','FuzzyQA', 'rel-ndtp2d'};
methods_renamed={'CorAl','CorAl-median','FuzzyQA','MME','NDT', 'Rel-NDT'};
Tfull=[]
for i=1:size(FullDataset,2)
    TrainingDataset   = FullDataset(i);
    EvaluationDataset = FullDataset(i); 
    ax=subplot(2,4,i)
    
    Tthis=EvaluateAndPlot(TrainingDataset,EvaluationDataset,methods,T)
    Tfull=[Tfull;Tthis]
    %EvaluationDataset={strrep(EvaluationDataset,'mean-map-entropy','MME')};
    %EvaluationDataset={strrep(EvaluationDataset,'entropy','CorAl')};
    %EvaluationDataset={strrep(EvaluationDataset,'entropy-median','CorAl-median')};
    box(ax,'off')
 
    xlabel(strrep(EvaluationDataset{1,1},'_',' '),'fontweight','bold','fontsize',8);
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
for i=1:size(methods_renamed,2)
    Tmethod=Tfull(strcmp(Tfull.Method,methods_renamed(i)),:)
    acc_i=Tmethod{:,1}
    size_i=Tmethod{:,5}
    tot=[tot dot(acc_i,size_i)/sum(size_i)]
    
end
%title('ETH dataset accuracy')
% %% Split datasets
% Tab_i=methods;
%
% Td=[]
% Ed=[]
% disp('training dataset')
% for set=1:size(TrainingDataset,2) %Extract all training samples
%     TrainingDataset(set);
%     Td=[Td;T(strcmp(T.Dataset,TrainingDataset{set}),:)];
% end
% disp('evaluation dataset')
% for set=1:size(EvaluationDataset,2) %Extract all evaluation samples
%     EvaluationDataset(set);
%     Ed=[Ed;T(strcmp(T.Dataset,EvaluationDataset{set}),:)];
% end
%
%
%
% models={}
%
% %%  Perform training
% for i=1:size(methods,2)
%     TrainingData=Td(strcmp(Td.Method,methods{i}),:);
%
%
%     [trainedClassifier, validationAccuracy] = trainClassifier(TrainingData);
%     methods{i}
%     validationAccuracy
%     %disp([methods{i},' =',validationAccuracy]);
%
%     models{i}=trainedClassifier;
%     diff = trainedClassifier.predictFcn(TrainingData) == TrainingData{:,10};
%     acc=sum(diff)/size(diff,1);
%
%     subplot(1,size(methods,2),i);
%     plot(TrainingData{TrainingData.Label==0,3},TrainingData{TrainingData.Label==0,2},'bx');
%     title(methods{i})  ;
%     hold on;
%     plot(TrainingData{TrainingData.Label==1,3},TrainingData{TrainingData.Label==1,2},'ro');
% end
%
%
% %%  Perform evaluation
% %Tresults={'Accuracy' 'Method' 'Evaluation data' 'Training data'};
% Tresults = array2table(zeros(0,4));
% Tresults.Properties.VariableNames = {'Accuracy' 'Method' 'EvaluationData' 'TrainingData'}
%
% for i=1:size(methods,2)
%     EvaluationData=Ed(strcmp(Ed.Method,methods{i}),:);
%
%     diff = models{i}.predictFcn(EvaluationData) == EvaluationData{:,10};
%     acc=sum(diff)/size(diff,1);
%     Tresults=[Tresults;{acc,methods{i}, EvaluationDataset, TrainingDataset}]
%
%     X = [methods{i},' accuracy: ',acc,'% (',sum(diff),'/',size(diff,1)];
%
%
%     subplot(1,size(methods,2),i);
%     plot(EvaluationData{EvaluationData.Label==0,3},EvaluationData{EvaluationData.Label==0,2},'bx');
%     title(methods{i})  ;
%     hold on;
%     plot(EvaluationData{EvaluationData.Label==1,3},EvaluationData{EvaluationData.Label==1,2},'ro');
% end
% figure(2)
% x=categorical(Tresults{:,2}),
% y=Tresults{:,1}
% bar(x,y);
% for i1=1:numel(y)
%     text(x(i1),y(i1),num2str(y(i1),'%0.2f'),...
%                'HorizontalAlignment','center',...
%                'VerticalAlignment','bottom')
% end
% xlabel('Classifier')
% ylabel('Accuracy')
% %X = categorical({'Small','Medium','Large','Extra Large'});
% %X = reordercats(X,{'Small','Medium','Large','Extra Large'});
% %Y = [10 21 33 52];
% %bar(X,Y)
