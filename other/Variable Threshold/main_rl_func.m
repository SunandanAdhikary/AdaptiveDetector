%% Your simulink model file name
% clear all;
whichAgents="acd";
whichAg_sim=30;
model= "rlVarTh_func_withoutController";
agents= model+["/RL Attacker Agent",...%"/RL Controller Agent",...
                                       "/RL Detector Agent"];

% agent1= "rlVarTh_func2/RL Attacker Agent";
% agent2= "rlVarTh_func2/RL Controller Agent";
% agent3= "rlVarTh_func2/RL Detector Agent";
% agents = agent1;
% open_system(model);
format long g;
%% Sampling Period, Episode duration
Tf = 15;
Ts = 0.1;
simlen=ceil(Tf/Ts);
maxepisodes = 15000;
%%% Global params needed inmodel
%% esp
% Ts=0.1;
% A = [0.4450 -0.0458;1.2939 0.4402];
% B = [0.0550;4.5607];
% C = [0 1];
% D = 0;
% K=[-0.0987 0.1420];
% L= [-0.0390;0.4339];
% safex = [1,2];
% tolerance = [0.1,0.1];
% th = 0.8;
% sensorRange = [2.5];
% actuatorRange = [0];

%% trajectory tracking
% s.whichAgents= "ac";
s.Ts = 0.1;
s.A = [1.0000    0.1000; 0    1.0000];
s.B = [0.0050; 0.1000];
s.C = [1 0];
s.D = [0];
s.K = [16.0302    5.6622];  % settling time around 10
s.L = [0.9902; 0.9892];
s.safex = [25,30];
s.tolerance = [1,1];
s.th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
s.sensorRange = [30000];
s.actuatorRange = [36];

%% init
% s.z=[0,0];
xdim= size(s.A,2);
ydim= size(s.C,1);
udim= size(s.B,2);

ulim= s.actuatorRange;
ylim= s.sensorRange;
rng shuffle;
s.proc_noise= 1.2*rand(xdim,simlen);
s.meas_noise= 0.02*rand(ydim,simlen);

s.t = Ts*zeros(simlen);  
s.x_act =zeros(xdim,simlen);

s.x_act(:,1) =[2*s.safex(1,1)*rand(1)-s.safex(1,1);...
                    2*s.safex(1,2)*rand(1)-s.safex(1,2)]
% s. xatk(1)= s.x_act(1);
s.xhat = zeros(xdim,simlen);
% s.est_err(:,simlen)= zeros(size(xdim,simlen));
s.est_err= s.x_act -s.xhat;
s.u_act = max(-ulim,min(ulim,s.K*s.xhat));
s.a_u= zeros(size(s.u_act));
s.uatk= max(-ulim,min(ulim,s.u_act +s.a_u));
s.y_act = s.C*s.x_act+s.meas_noise;
s.y_act = max(-ylim,min(ylim,s.C*s.x_act+s.meas_noise));
s.a_y = zeros(size(s.y_act));
s.yatk = max(-ylim,min(ylim,s.y_act+s.a_y));
s.z = s.yatk-s.C*s.xhat;
s.z_mean= s.z ;
s.z_var =ones(size(s.z,1),simlen);
s.g = zeros(1,simlen);
s.chi_tst= zeros(1,simlen);
s.threshold= s.th*ones(1,simlen);
s.tau= ones(1,simlen);
s.non_cent= zeros(size(s.z,1),simlen);

%% Multi Agent Observation Properties
obsInfo = {[rlNumericSpec([9 1])], [rlNumericSpec([5 1])],...
            [rlNumericSpec([5 1])]};%,...3 observeations in continuous domain: residue,output,actuation
%     'LowerLimit',[-inf -inf 0  ]',...
%     'UpperLimit',[ inf  inf inf]');
obsInfo{1,1}.Name , obsInfo{1,2}.Name, obsInfo{1,3}.Name = 'observations';
% obsInfo{1,1}.Name , obsInfo{1,2}.Name ='observations';
obsInfo{1,1}.Description , obsInfo{1,2}.Description,...
    obsInfo{1,3}.Description = 'residue,output and control input';
numObservationsAtk = obsInfo{1,1}.Dimension(1);
numObservationsCon = obsInfo{1,2}.Dimension(1);
numObservationsDtc = obsInfo{1,3}.Dimension(1);
% obsInfo{1,1}.Datatype , obsInfo{1,2}.Datatype, obsInfo{1,3}.Datatype= long g;
%% Multi Agent Action Properties
actInfo = {rlNumericSpec([2 1]),rlNumericSpec([1 1]),...
                rlNumericSpec([2 1])};
actInfo{1,1}.Name = 'actuator attack';
actInfo{1,2}.Name = 'control input';
actInfo{1,3}.Name = 'Detection threshold';
numActionsAtk = actInfo{1,1}.Dimension(1);
numActionsCon = actInfo{1,2}.Dimension(1);
numActionsDtc = actInfo{1,3}.Dimension(1);
% agentblocks = {[model '/RL Attacker Agent'],[ model '/RL Controller Agent' ]};
% 
% rng(0)
% 
%% Critic Network for attacker agent
statePath = [
    featureInputLayer(numObservationsAtk,'Normalization','none','Name','StateAtk')
    fullyConnectedLayer(500,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(350,'Name','CriticStateFC2')];
actionPath = [
    featureInputLayer(numActionsAtk,'Normalization','none','Name','ActionAtk')
    fullyConnectedLayer(350,'Name','CriticActionFC1')];
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
criticAtk = rlQValueRepresentation(criticNetwork,obsInfo{1,1},actInfo{1,1},...
    'Observation',{'StateAtk'},'Action',{'ActionAtk'},criticOpts);

%% Critic Network for Controller Agent
statePath = [
    featureInputLayer(numObservationsCon,'Normalization','none','Name','StateCon')
    fullyConnectedLayer(500,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(350,'Name','CriticStateFC2')];
actionPath = [
    featureInputLayer(numActionsCon,'Normalization','none','Name','ActionCon')
    fullyConnectedLayer(350,'Name','CriticActionFC1')];
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
criticCon = rlQValueRepresentation(criticNetwork,obsInfo{1,2},actInfo{1,2},...
    'Observation',{'StateCon'},'Action',{'ActionCon'},criticOpts);

%% Critic Network for Detector Agent
statePath = [
    featureInputLayer(numObservationsDtc,'Normalization','none','Name','StateDtc')
    fullyConnectedLayer(500,'Name','CriticStateFC1')
    reluLayer('Name','CriticRelu1')
    fullyConnectedLayer(350,'Name','CriticStateFC2')];
actionPath = [
    featureInputLayer(numActionsDtc,'Normalization','none','Name','ActionDtc')
    fullyConnectedLayer(350,'Name','CriticActionFC1')];
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
criticDtc = rlQValueRepresentation(criticNetwork,obsInfo{1,3},actInfo{1,3},...
    'Observation',{'StateDtc'},'Action',{'ActionDtc'},criticOpts);

%% Actor for Attacker Agent
actorNetwork = [
    featureInputLayer(numObservationsAtk,'Normalization','none','Name','StateAtk')
    fullyConnectedLayer(3, 'Name','actorFC1')
    reluLayer('Name','actorRelu1')
    fullyConnectedLayer(400, 'Name','actorFC2')
    reluLayer('Name','actorRelu2')
    fullyConnectedLayer(numActionsAtk,'Name','actorFC3')
    tanhLayer('Name','actorTanhAtk')];
actorOptions = rlRepresentationOptions('LearnRate',1e-04,'GradientThreshold',1);
actorAtk = rlDeterministicActorRepresentation(actorNetwork,obsInfo{1,1},actInfo{1,1},...
    'Observation',{'StateAtk'},'Action',{'actorTanhAtk'},actorOptions);

%% Actor for Controller Agent
actorNetwork = [
    featureInputLayer(numObservationsCon,'Normalization','none','Name','StateCon')
    fullyConnectedLayer(500, 'Name','actorFC1')
    reluLayer('Name','actorRelu1')
    fullyConnectedLayer(400, 'Name','actorFC2')
    reluLayer('Name','actorRelu2')
    fullyConnectedLayer(numActionsCon,'Name','actorFC3')
    tanhLayer('Name','actorTanhCon')];
actorOptions = rlRepresentationOptions('LearnRate',1e-04,'GradientThreshold',1);
actorCon = rlDeterministicActorRepresentation(actorNetwork,obsInfo{1,2},actInfo{1,2},...
    'Observation',{'StateCon'},'Action',{'actorTanhCon'},actorOptions);

%% Actor for Detector Agent
actorNetwork = [
    featureInputLayer(numObservationsDtc,'Normalization','none','Name','StateDtc')
    fullyConnectedLayer(500, 'Name','actorFC1')
    reluLayer('Name','actorRelu1')
    fullyConnectedLayer(400, 'Name','actorFC2')
    reluLayer('Name','actorRelu2')
    fullyConnectedLayer(numActionsDtc,'Name','actorFC3')
    tanhLayer('Name','actorTanhDtc')];
actorOptions = rlRepresentationOptions('LearnRate',1e-04,'GradientThreshold',1);
actorDtc = rlDeterministicActorRepresentation(actorNetwork,obsInfo{1,3},actInfo{1,3},...
                            'Observation',{'StateDtc'},'Action',{'actorTanhDtc'},actorOptions);

%% DDPG Agents and their properties
agentOpts = rlDDPGAgentOptions(...
    'SampleTime',Ts,...
    'TargetSmoothFactor',1e-3,...
    'DiscountFactor',0.98, ...
    'MiniBatchSize',64, ...
    'ExperienceBufferLength',1e6); 
agentOpts.NoiseOptions.Variance = 0.3;
agentOpts.NoiseOptions.VarianceDecayRate = 1e-5;

agentAtk = rlDDPGAgent(actorAtk,criticAtk,agentOpts);
% agentCon = rlDDPGAgent(actorCon,criticCon,agentOpts);
agentDtc = rlDDPGAgent(actorDtc,criticDtc,agentOpts);
% agentObjs = [agentAtk,agentCon];
agentObjs = [agentAtk,agentDtc];
% agentObjs = [agentAtk, agentCon, agentDtc];
%% environment creation
env = rlSimulinkEnv(model,agents)%,obsInfo,actInfo);
% obsInfo=getObservationInfo(env); actInfo=getActorInfo(env); %% How to
%% reset the environment in every iteration/episode
% @(in)localResetFcn(in); env.ResetFcn = @(in);
env.ResetFcn = @(in) randomReset(in, s.safex, simlen, xdim, ydim, ...
                                            ylim, ulim, s.C, s.K, s.th);

%% Training
maxsteps = simlen;
trainOpts = rlTrainingOptions(...
    'MaxEpisodes',maxepisodes, ...
    'MaxStepsPerEpisode',maxsteps, ...
    'ScoreAveragingWindowLength',20, ...
    'Verbose',false, ...
    'Plots','training-progress',...
    'StopTrainingCriteria','AverageReward',...
    'StopTrainingValue',1500,...
    'SaveAgentCriteria','EpisodeReward',...
    'SaveAgentValue',2400);

doTraining = true;

if doTraining
    % Train the agent.
    trainingStats = train(agentObjs,env,trainOpts);
    save(trainOpts.SaveAgentDirectory + "/AttckerAgentFin.mat",...
        'agentAtk');
    save(trainOpts.SaveAgentDirectory + "/ControllerAgentFin.mat",...
        'agentCon');
    save(trainOpts.SaveAgentDirectory + "/DetectorAgentFin.mat",...
        'agentDtc')
else
    % Load the pretrained agent for the example.
    disp("train the model");
end

% simOpts = rlSimulationOptions('MaxSteps',maxsteps,'StopOnError','on');
% experiences = sim(env,agentObjs,simOpts);