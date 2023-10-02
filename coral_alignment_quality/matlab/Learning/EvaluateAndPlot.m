function [Tresults]=EvaluateAndPlot(TrainingDataset,EvaluationDataset,methods,T)
%% Split datasets
Tab_i=methods;

Td=[]
Ed=[]
disp('training dataset')
for set=1:size(TrainingDataset,2) %Extract all training samples from TrainingDataset
    TrainingDataset(set);
    Td=[Td;T(strcmp(T.Dataset,TrainingDataset{set}),:)];
end
disp('evaluation dataset')
for set=1:size(EvaluationDataset,2) %Extract all evaluation samples from Evaluation Dataset
    EvaluationDataset(set);
    Ed=[Ed;T(strcmp(T.Dataset,EvaluationDataset{set}),:)];
end



models={}

%%  Perform training
for i=1:size(methods,2)
    TrainingData=Td(strcmp(Td.Method,methods{i}),:);
    
    
    [trainedClassifier, validationAccuracy] = trainClassifier(TrainingData);
    methods{i}
    validationAccuracy
    %disp([methods{i},' =',validationAccuracy]);
    
    models{i}=trainedClassifier;
    diff = trainedClassifier.predictFcn(TrainingData) == TrainingData{:,10};
    acc=sum(diff)/size(diff,1);
    
%     subplot(1,size(methods,2),i);
%     plot(TrainingData{TrainingData.Label==0,3},TrainingData{TrainingData.Label==0,2},'bx');
%     title(methods{i})  ;
%     hold on;
%     plot(TrainingData{TrainingData.Label==1,3},TrainingData{TrainingData.Label==1,2},'ro');   
end


%%  Perform evaluation
%Tresults={'Accuracy' 'Method' 'Evaluation data' 'Training data'};
Tresults = array2table(zeros(0,5));
Tresults.Properties.VariableNames = {'Accuracy' 'Method' 'EvaluationData' 'TrainingData' 'Datapoints'}

for i=1:size(methods,2)
    EvaluationData=Ed(strcmp(Ed.Method,methods{i}),:);
   
    diff = models{i}.predictFcn(EvaluationData) == EvaluationData{:,10};
    acc=sum(diff)/size(diff,1);
    Tresults=[Tresults;{acc,methods{i}, EvaluationDataset, ' ',size(diff,1);}];
    
    X = [methods{i},' accuracy: ',acc,'% (',sum(diff),'/',size(diff,1)];
    

%      subplot(1,size(methods,2),i);
%      plot(EvaluationData{EvaluationData.Label==0,3},EvaluationData{EvaluationData.Label==0,2},'bx');
%      title(methods{i})  ;
%      hold on;
%      plot(EvaluationData{EvaluationData.Label==1,3},EvaluationData{EvaluationData.Label==1,2},'ro');   
     
end
%figure(2)
T_changename=Tresults{:,2};
for i=1:size(T_changename,1)
    disp(Tresults{i,2});
    T_changename{i,1}=strrep(T_changename{i,1},'mean-map-entropy','MME');
    T_changename{i,1}=strrep(T_changename{i,1},'entropy-median','CorAl-median');
    T_changename{i,1}=strrep(T_changename{i,1},'entropy','CorAl');
    T_changename{i,1}=strrep(T_changename{i,1},'rel-ndtp2d','Rel-NDT');
    T_changename{i,1}=strrep(T_changename{i,1},'ndtp2d','NDT');
    
    disp(Tresults{i,2});
end
Tresults{:,2}=T_changename
x=categorical(T_changename)
y=Tresults{:,1}
bar(x,y);
for i1=1:numel(y);
    text(x(i1),y(i1),num2str(y(i1),'%0.2f'),...
               'HorizontalAlignment','center',...
               'VerticalAlignment','bottom','fontweight','bold','fontsize',8);
end

% X = categorical({'Small','Medium','Large','Extra Large'});
% X = reordercats(X,{'Small','Medium','Large','Extra Large'});
% Y = [10 21 33 52];
% bar(X,Y)
