%%Load data
clc
clear all
close all


T=[];
base='/mnt/disk0/';
dataset={'orkla_pcds/'};
FullDataset= { 'warehouse'};


sufix='logistics_top_competitors/';
T=LoadEvaluation(base, dataset, sufix,'_med');
%T=T(T.Radius==0.2,:);
 

%Twarehouse=Tall{55393:59256,:}
%T=Tall(31777:47664,:)
%T=[Tall(31777:47664,:);Tall(55393:59256,:)];

%methods={'entropy-median','entropy','ndtp2d','rel-ndtp2d'}%,'rel-ndtp2d'};
methods={'entropy' 'mean-map-entropy' 'ndtp2d'}
%methods={'rel-ndtp2d','ndtp2d'};
for i=1:size(FullDataset,2)
    TrainingDataset   = FullDataset(i)
    EvaluationDataset = FullDataset(i)
    subplot(1,2,i)
    
    EvaluateAndPlot(TrainingDataset,EvaluationDataset,methods,T)
    EvaluationDataset={strrep(EvaluationDataset,'mean-map-entropy','MME')};
    
 
    xlabel(strrep(EvaluationDataset{1,1},'_',' '),'fontweight','bold','fontsize',8);
    ylabel('Accuracy','fontweight','bold','fontsize',8)
    set(gca,'fontweight','bold','FontSize',8)
    %set(gca,'FontSize',18)
end
x0=10;
y0=10;
width=1700;
height=250
set(gcf,'position',[x0,y0,width,height])
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
