%% Your simulink model file name
clear all;
model= "rlVarTh_func";
agent1= "rlVarTh_func/RL Attacker Agent";
agent2= "rlVarTh_func/RL Controller Agent";
agent3= "rlVarTh_func/RL Detector Agent";
% esp_env = RLenv;

%% Sampling Period, Episode duration
Tf = 200;
Ts = 0.1;

% %%% Global params needed inmodel
% %% esp
% % Ts=0.1;
% % A = [0.4450 -0.0458;1.2939 0.4402];
% % B = [0.0550;4.5607];
% % C = [0 1];
% % D = 0;
% % K=[-0.0987 0.1420];
% % L= [-0.0390;0.4339];
% % safex = [1,2];
% % tolerance = [0.1,0.1];
% % th = 0.8;
% % sensorRange = [2.5];
% % actuatorRange = [0];
% 
% %% trajectory tracking
s.Ts = 0.1;
s.A = [1.0000    0.1000; 0    1.0000];
s.B = [0.0050; 0.1000];
s.C = [1 0];
s.D = [0];
s.K = [16.0302    5.6622];  % settling time around 10
s.L = [0.9902; 0.9892];
s.safex = [25,30];
s.tolerance = [1,1];
s.th = 2.15;              % new: changed it a bit so that without attack, the residue remains below threshold.
s.sensorRange = [30000];
s.actuatorRange = [36];

%% init
s.z=[0,0];
s.t(1)=0*Ts;            
s.x_act(:,1) =[2*s.safex(1,1)*randn(1)-s.safex(1,1);...
                    2*s.safex(1,2)*randn(1)-s.safex(1,2)];
% s. xatk(1)= s.x_act(1);
s.xhat(:,1)=[0;0];
s.est_err(:,1)= s.x_act(:,1)-s.xhat(:,1);
s.u_act(:,1)= s.K*s.xhat(:,1);
s.a_u(:,1)= zeros(size(s.u_act(:,1)));
s.uatk(:,1)= s.u_act(:,1)+s.a_u(:,1);
s.y_act(:,1)= s.C*s.x_act(:,1);
s.a_y(:,1)= zeros(size(s.y_act(:,1)));
s.yatk(:,1)= s.y_act(:,1)+s.a_y(:,1);
s.z(:,1)= s.yatk(:,1)-s.C*s.xhat(:,1);
s.z_mean(:,1)= zeros(size(s.z));
s.P= cov(s.z');
s.g(1)= 0;
s.chi_tst(1)= 0;
s.threshold(1)= s.th;
s.tau(1)= 1;

%% Multi Agent Observation Properties
obsInfo = {[rlNumericSpec([5 1])], [rlNumericSpec([4 1])], [rlNumericSpec([4 1])]};%,...3 observeations in continuous domain: residue,output,actuation
%     'LowerLimit',[-inf -inf 0  ]',...
%     'UpperLimit',[ inf  inf inf]');
obsInfo{1,1}.Name , obsInfo{1,2}.Name, obsInfo{132}.Name = 'observations';
obsInfo{1,1}.Description , obsInfo{1,2}.Description, obsInfo{1,3}.Description = 'residue,output and control input';
numObservationsAtk = obsInfo{1,1}.Dimension(1);
numObservationsCon = obsInfo{1,2}.Dimension(1);
numObservationsDtc = obsInfo{1,3}.Dimension(1);

%% Multi Agent Action Properties
actInfo = {rlNumericSpec([2 1]),rlNumericSpec([1 1]),rlNumericSpec([2 1])};
actInfo{1,1}.Name = 'actuator attack';
actInfo{1,2}.Name = 'control input';
actInfo{1,2}.Name = 'Detection threshold';
numActionsAtk = actInfo{1,1}.Dimension(1);
numActionsCon = actInfo{1,2}.Dimension(1);
numActionsDtc = actInfo{1,3}.Dimension(1);
% agentblocks = {[model '/RL Attacker Agent'],[ model '/RL Controller Agent' ]};
% 
% rng(0)
% 
%% Critic Network for attacker agent
statePath = [
    featureInputLayer(numObservationsAtk,'Normalization','none','Name','State')
    fullyConnectedLayer(50,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(25,'Name','CriticStateFC2')];
actionPath = [
    featureInputLayer(numActionsAtk,'Normalization','none','Name','Action')
    fullyConnectedLayer(25,'Name','CriticActionFC1')];
commonPath = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu')
    fullyConnectedLayer(1,'Name','CriticOutput')];

criticNetwork = layerGraph();
criticNetwork = addLayers(criticNetwork,statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);
criticNetwork = connectLayers(criticNetwork,'CriticStateFC2','add/in1');
criticNetwork = connectLayers(criticNetwork,'CriticActionFC1','add/in2');
criticOpts = rlRepresentationOptions('LearnRate',1e-03,'GradientThreshold',1);
criticAtk = rlQValueRepresentation(criticNetwork,obsInfo{1,1},actInfo{1,1},'Observation',{'State'},'Action',{'Action'},criticOpts);

%% Critic Network for Controller Agent
statePath = [
    featureInputLayer(numObservationsCon,'Normalization','none','Name','State')
    fullyConnectedLayer(50,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(25,'Name','CriticStateFC2')];
actionPath = [
    featureInputLayer(numActionsCon,'Normalization','none','Name','Action')
    fullyConnectedLayer(25,'Name','CriticActionFC1')];
commonPath = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu')
    fullyConnectedLayer(1,'Name','CriticOutput')];

criticNetwork = layerGraph();
criticNetwork = addLayers(criticNetwork,statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);
criticNetwork = connectLayers(criticNetwork,'CriticStateFC2','add/in1');
criticNetwork = connectLayers(criticNetwork,'CriticActionFC1','add/in2');
criticOpts = rlRepresentationOptions('LearnRate',1e-03,'GradientThreshold',1);
criticCon = rlQValueRepresentation(criticNetwork,obsInfo{1,2},actInfo{1,2},'Observation',{'State'},'Action',{'Action'},criticOpts);

%% Critic Network for Detector Agent
statePath = [
    featureInputLayer(numObservationsDtc,'Normalization','none','Name','State')
    fullyConnectedLayer(50,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(25,'Name','CriticStateFC2')];
actionPath = [
    featureInputLayer(numActionsDtc,'Normalization','none','Name','Action')
    fullyConnectedLayer(25,'Name','CriticActionFC1')];
commonPath = [
    additionLayer(2,'Name','add')
    reluLayer('Name','CriticCommonRelu')
    fullyConnectedLayer(1,'Name','CriticOutput')];

criticNetwork = layerGraph();
criticNetwork = addLayers(criticNetwork,statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);
criticNetwork = connectLayers(criticNetwork,'CriticStateFC2','add/in1');
criticNetwork = connectLayers(criticNetwork,'CriticActionFC1','add/in2');
criticOpts = rlRepresentationOptions('LearnRate',1e-03,'GradientThreshold',1);
criticDtc = rlQValueRepresentation(criticNetwork,obsInfo{1,3},actInfo{1,3},'Observation',{'State'},'Action',{'Action'},criticOpts);


%% Actor for Attacker Agent
actorNetwork = [
    featureInputLayer(numObservationsAtk,'Normalization','none','Name','State')
    fullyConnectedLayer(3, 'Name','actorFC')
    tanhLayer('Name','actorTanh')
    fullyConnectedLayer(numActionsAtk,'Name','Action')];
actorOptions = rlRepresentationOptions('LearnRate',1e-04,'GradientThreshold',1);
actorAtk = rlDeterministicActorRepresentation(actorNetwork,obsInfo{1,1},actInfo{1,1},'Observation',{'State'},'Action',{'Action'},actorOptions);

%% Actor for Controller Agent
actorNetwork = [
    featureInputLayer(numObservationsCon,'Normalization','none','Name','State')
    fullyConnectedLayer(3, 'Name','actorFC')
    tanhLayer('Name','actorTanh')
    fullyConnectedLayer(numActionsCon,'Name','Action')];
actorOptions = rlRepresentationOptions('LearnRate',1e-04,'GradientThreshold',1);
actorCon = rlDeterministicActorRepresentation(actorNetwork,obsInfo{1,2},actInfo{1,2},'Observation',{'State'},'Action',{'Action'},actorOptions);

%% Actor for Detector Agent
actorNetwork = [
    featureInputLayer(numObservationsDtc,'Normalization','none','Name','State')
    fullyConnectedLayer(3, 'Name','actorFC')
    tanhLayer('Name','actorTanh')
    fullyConnectedLayer(numActionsDtc,'Name','Action')];
actorOptions = rlRepresentationOptions('LearnRate',1e-04,'GradientThreshold',1);
actorDtc = rlDeterministicActorRepresentation(actorNetwork,obsInfo{1,3},actInfo{1,3},'Observation',{'State'},'Action',{'Action'},actorOptions);

%% DDPG Agent properties
agentOpts = rlDDPGAgentOptions(...
    'SampleTime',Ts,...
    'TargetSmoothFactor',1e-3,...
    'DiscountFactor',1.0, ...
    'MiniBatchSize',64, ...
    'ExperienceBufferLength',1e6); 
agentOpts.NoiseOptions.Variance = 0.3;
agentOpts.NoiseOptions.VarianceDecayRate = 1e-5;
% agentObj = rlDDPGAgent(actor,critic,agentOpts);

%% environment creation
env = rlSimulinkEnv(model,[agent1, agent2, agent3]);
% obsInfo=getObservationInfo(env); actInfo=getActorInfo(env); %% How to
% reset the environment in every iteration/episode
% @(in)localResetFcn(in); env.ResetFcn = @(in)
% env.ResetFcn = setVariable(in,'theta0',randn,'Workspace',mdl);

%% Training
% maxepisodes = 5000;
% maxsteps = ceil(Tf/Ts);
% trainOpts = rlTrainingOptions(...
%     'MaxEpisodes',maxepisodes, ...
%     'MaxStepsPerEpisode',maxsteps, ...
%     'ScoreAveragingWindowLength',20, ...
%     'Verbose',false, ...
%     'Plots','training-progress',...
%     'StopTrainingCriteria','AverageReward',...
%     'StopTrainingValue',800);
% 
% doTraining = true;
% 
% if doTraining
%     % Train the agent.
%     trainingStats = train(agent,env,trainOpts);
% else
%     % Load the pretrained agent for the example.
%     load('WaterTankDDPG.mat','agent')
% end
% 
% simOpts = rlSimulationOptions('MaxSteps',maxsteps,'StopOnError','on');
% experiences = sim(env,agent,simOpts);