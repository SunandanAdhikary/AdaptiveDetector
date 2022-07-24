%% Your simulink model file name
clear all;
model= "rlVarTh_siml";
agent1= "rlVarTh_siml/RL Attacker Agent";
agent2= "rlVarTh_siml/RL Controller Agent";
agent3= "rlVarTh_siml/RL Detector Agent";
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
Ts = 0.1;
A = [1.0000    0.1000; 0    1.0000];
B = [0.0050; 0.1000];
C = [1 0];
D = [0];
K = [16.0302    5.6622];  % settling time around 10
L = [0.9902; 0.9892];
safex = [25,30];
tolerance = [1,1];
th = 2.15;              % new: changed it a bit so that without attack, the residue remains below threshold.
sensorRange = [30000];
actuatorRange = [36];

%% init
% z=[0,0];
ulim= actuatorRange;
ylim= sensorRange;
t(1)=0*Ts;            
x_act(:,1) =[2*safex(1,1)*randn(1)-safex(1,1);...
                    2*safex(1,2)*randn(1)-safex(1,2)];
%  xatk(1)= x_act(1);
xhat(:,1)=[0;0];
est_err(:,1)= x_act(:,1)-xhat(:,1);
u_act(:,1)= max(-ulim,min(ulim,K*xhat(:,1)));
a_u(:,1)= zeros(size(u_act(:,1)));
uatk(:,1)= max(-ulim,min(ulim,u_act(:,1)+a_u(:,1)));
y_act(:,1)= max(-ylim,min(ylim,C*x_act(:,1)));
a_y(:,1)= zeros(size(y_act(:,1)));
yatk(:,1)= max(-ylim,min(ylim,y_act(:,1)+a_y(:,1)));
z(:,1)= yatk(:,1)-C*xhat(:,1);
z_mean(:,1)= z(:,1);
P= cov(z');
g(1)= 0;
chi_tst(1)= 0;
threshold(1)= th;
tau(1)= 1;

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
agentOptNoiseOptionVariance = 0.3;
agentOptNoiseOptionVarianceDecayRate = 1e-5;
% agentObj = rlDDPGAgent(actor,critic,agentOpts);

%% environment creation
env = rlSimulinkEnv(model,[agent1, agent2, agent3]);
% obsInfo=getObservationInfo(env); actInfo=getActorInfo(env); %% How to
%% reset the environment in every iteration/episode
% @(in)localResetFcn(in); env.ResetFcn = @(in);
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