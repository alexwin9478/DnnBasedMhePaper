%% BOF
clear;clc;
userpath = pwd; 
load('Train_14T_Customized_WLTP.mat') % [Training data]
load('Test_01T_NordSchleife.mat') % [Test data]

% plot settings
fntsze = 13;
plot_data_distr = false;

% Set Training Parameter
ExecutionEnvironment = 'cpu'; % 'CPU' or 'GPU' with driver
numHiddenUnits = 8; % [Manipulated parameter]
maxEpochs = 10000; % [Manipulated parameter]
miniBatchSize = 512; % Manipulated parameter
learnRate = 0.02; % Manipulated parameter   

%% Train and Test the Network
Length = size(XTest,2);
Data = struct('net',{},'YPred',{},'Error',{},'RMSE',{}); % Define data struction to storge trained ANN
MP = 2024;
XTotal = XTrain;
YTotal = YTrain;

% first 80% are training data
ratio_train = 0.8;
dataset_length = size(XTrain, 2);
indx_tr = floor(ratio_train*dataset_length);
XTrain = [XTotal(1,1:indx_tr); XTotal(2,1:indx_tr); XTotal(3,1:indx_tr); XTotal(4,1:indx_tr)];
YTrain = [YTotal(1,1:indx_tr); YTotal(2,1:indx_tr)];
XVal = [XTotal(1,indx_tr+1:end); XTotal(2,indx_tr+1:end); XTotal(3,indx_tr+1:end); XTotal(4,indx_tr+1:end)];
YVal = [YTotal(1,indx_tr+1:end); YTotal(2,indx_tr+1:end)];

% % last 80% are training data
% ratio_train = 0.2;
% dataset_length = size(XTrain, 2);
% indx_tr = floor(ratio_train*dataset_length);
% XVal = [XTotal(1,1:indx_tr); XTotal(2,1:indx_tr); XTotal(3,1:indx_tr); XTotal(4,1:indx_tr)];
% YVal = [YTotal(1,1:indx_tr); YTotal(2,1:indx_tr)];
% XTrain = [XTotal(1,indx_tr+1:end); XTotal(2,indx_tr+1:end); XTotal(3,indx_tr+1:end); XTotal(4,indx_tr+1:end)];
% YTrain = [YTotal(1,indx_tr+1:end); YTotal(2,indx_tr+1:end)];

%% Plotting datasets, histograms
if plot_data_distr
    y_1_tr = YTrain(1,:);
    % y_2_tr = YTrain(2,:);
    y_1_ts = YVal(1,:);
    % y_2_tr = YVal(2,:);
    max_scale = max( [max(y_1_tr), max(y_1_ts)]);
    min_scale = min( [min(y_1_tr), min(y_1_ts)]);
    
    % T_w plots
    fig = figure;
    subplot(2,1,1)
    set(gcf,'color','w');
    histogram(y_1_tr)
    grid on
    ylabel("Count / -",'Interpreter', 'latex')
    % xlabel({'Winding Temp. Gradient',' / (degC/s)'},'Interpreter','latex')
    xlim([min_scale,max_scale])
    % title('Training Dataset','Interpreter', 'latex')
    set(gcf,'units','points','position',[200,200,900,400])
    set(gca,'FontSize',fntsze)
    set(gca,'TickLabelInterpreter','latex')
    ax = gca;
    ax.YAxis.Exponent = 0;
    
    subplot(2,1,2)
    set(gcf,'color','w');
    histogram(y_1_ts)
    grid on
    ylabel("Count / -",'Interpreter', 'latex')
    xlabel({'Winding Temp. Gradient / (degC/s)'},'Interpreter','latex')
    xlim([min_scale,max_scale])
    % title('Test Dataset','Interpreter', 'latex')
    set(gcf,'units','points','position',[200,200,900,400])
    set(gca,'FontSize',fntsze)
    set(gca,'TickLabelInterpreter','latex')
    ax2 = gca;
    % ax2.XTick = 0:1e5:11e5;
    ax2.YAxis.Exponent = 0;
    % sgt = sgtitle('Winding Temp. Gradient (Output) Data Distribution','Interpreter', 'latex','Color','black', 'FontSize', fntsze);
    linkaxes([ax ax2],'x')
end


% test dataset
% normalization settings
Max_n_em = 15000;
Max_torque_em = 160.36;
T_cool = 60;
T_max = 160;
T_abs_range_norm = T_max-T_cool;
T_wr_grad_max_norm = YMax; % YMax = max(T_w_Grad);
T_wr_grad_min_norm = YMin; % YMin = min(T_w_Grad);
T_wr_grad_min_range_norm = T_wr_grad_max_norm - T_wr_grad_min_norm;

% outputs TRAIN
y_1_ts = YTest(1,:); % T_w_grad_Test
y_2_ts = YTest(2,:); % T_r_grad_Test
y_1_ts_norm = (YTest(1,:) - T_wr_grad_min_norm) ./ T_wr_grad_min_range_norm; % T_w_grad_Test
y_2_ts_norm = (YTest(2,:) - T_wr_grad_min_norm) ./ T_wr_grad_min_range_norm; % T_r_grad_Test 

% preallocate 
trainingrun = 0;
load('../results/MHE_pmsm_net_2024_0008_0001.mat');
load('../results/MHE_pmsm_net_info_2024_0008_0001.mat');

%% Training 
maxEpochs = 10000; % [Manipulated parameter]

tic
parfor trainingrun = 11:16
    tic
      
    % Train the Network
    [MHE_pmsm_net, MHE_pmsm_net_info] = TrainLSTM(XTrain,YTrain,numHiddenUnits,ExecutionEnvironment,...
    maxEpochs,miniBatchSize,learnRate, XVal,YVal); % in '\04_Function'
    % MHE_pmsm_net_analysis = analyzeNetwork(MHE_pmsm_net); % analysis including total number of learnable parameters
    
    trainedNetworks{trainingrun} = MHE_pmsm_net;
    trainedNetworksInfo{trainingrun} = MHE_pmsm_net_info;
    trainedNetworksValLoss{trainingrun} = MHE_pmsm_net_info.FinalValidationLoss;
    
    % prediciton new net TEST
    y_pred_ts_norm_new = predict(MHE_pmsm_net, XTest);
    y_1_pred_ts_norm_new = y_pred_ts_norm_new(1,:); y_2_pred_ts_norm_new = y_pred_ts_norm_new(2,:);
    y_1_pred_ts_new = dataTraindeNormalize(y_1_pred_ts_norm_new, T_wr_grad_min_norm, T_wr_grad_min_range_norm);
    y_2_pred_ts_new = dataTraindeNormalize(y_2_pred_ts_norm_new, T_wr_grad_min_norm, T_wr_grad_min_range_norm);
    XTrain_Denorm = dataTraindeNormalize(XTrain, T_wr_grad_min_norm, T_wr_grad_min_range_norm);
    
    trainedNetworksRmseY1{trainingrun} = rmse((y_1_ts),(y_1_pred_ts_new),"all"); % dec/s
    trainedNetworksRmseY2{trainingrun} = rmse((y_2_ts),(y_2_pred_ts_new),"all"); % dec/s
    trainedNetworksRmse{trainingrun} = (trainedNetworksRmseY1{trainingrun} + trainedNetworksRmseY2{trainingrun} ) / 2; % dec/s; % dec/s

    toc
end

toc

% extract the necessary networks
figure;plot(cell2mat(trainedNetworksValLoss(1,11:end))); title("ValLoss");
figure;plot(cell2mat(trainedNetworksRmseY1(1,11:end))); title("Rmse T_w");
figure;plot(cell2mat(trainedNetworksRmseY2(1,11:end))); title("Rmse T_r");
figure;plot(cell2mat(trainedNetworksRmse(1,11:end))); title("Rmse Mean");

savename = [sprintf('%04d',MP),'_',sprintf('%04d',numHiddenUnits),'_',sprintf('%04d',trainingrun),'.mat'];
save(['../results/MHE_pmsm_net_',savename],"trainedNetworks");
save(['../results/MHE_pmsm_net_info_',savename],"trainedNetworksInfo");


