function [trainedNet, info] = TrainLSTM4(XTrain,YTrain,numHiddenUnits,ExecutionEnvironment,maxEpochs,miniBatchSize,learnRate, XVal,YVal)
%% Define Network
numFeatures = 4; % Number of input
numResponses = 2; % Number of output
   
layers = [ ...
    sequenceInputLayer(numFeatures)
    lstmLayer(numHiddenUnits,'OutputMode','sequence')
    fullyConnectedLayer(numResponses)
    regressionLayer];

options = trainingOptions('adam', ...
    'ExecutionEnvironment',ExecutionEnvironment, ...
    'MaxEpochs',maxEpochs, ...
    'MiniBatchSize',miniBatchSize, ...
    'InitialLearnRate',learnRate, ...
    'GradientThreshold',1, ...
    'Shuffle','once', ...
    'Plots','training-progress',...
    'Verbose',1, ...
    'LearnRateSchedule','piecewise',...
    'LearnRateDropPeriod',500, ...
    'ValidationFrequency', 10, ...
    'ValidationData',[{XVal} {YVal}], ...
    'OutputNetwork','best-validation-loss', ...
    'LearnRateDropFactor',0.75);
% 'Shuffle'options: 'once', 'never','every-epoch'
%% Train the Network
[trainedNet, info] = trainNetwork(XTrain,YTrain,layers,options);