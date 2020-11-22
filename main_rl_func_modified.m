% a for attacker, d for detector, c for controller
system = "esp"
whichAgents = "d";
whichAg_sim=1; % same: updated accordingly for simulink
model = "rlVarTh_func_withController"; % can run with "ad"
%reset: in case models are not trained right, make fresh models chucking the current object
doReset = false; 
doTraining = true;
if ~doTraining
    PRE_TRAINED_MODEL_DIR = "";%take trained matfile from folder from saved_agent dir if not training
end
ifSim = true; % if you want to simulate after training
format long g;
%% Sampling Period, Episode duration
Tf = 15;
Ts = 0.1;
simlen=ceil(Tf/Ts);
maxepisodes  = 10000;
% agents= model+["/RL Attacker Agent","/RL Controller Agent"]%...
%                                             "/RL Detector Agent"];

if system== "esp"
    %% esp
    s.Ts=0.04;
    s.A = [0.4450 -0.0458;1.2939 0.4402];
    s.B = [0.0550;4.5607];
    s.C = [0 1];
    s.D = 0;
    % K=[-0.0987 0.1420];
    s.K = [0.2826    0.0960];
    s.L= [-0.0390;0.4339];
    s.safex = [1,2];
    s.init = 0.1;
    s.perf = 0.1;
    s.th = 4.35; 
    s.settlingTime = 5 
    s.sensorRange = [2.5] 
    s.actuatorRange = [0.8125] 
end
if system=="trajectory"
%% trajectory tracking
    s.Ts = 0.1;
    s.A = [1.0000    0.1000; 0    1.0000];
    s.B = [0.0050; 0.1000];
    s.C = [1 0];
    s.D = [0];
    s.K = [16.0302    5.6622];  % settling time around 10
    s.L = [0.9902; 0.9892];
    s.safex = [25,30];
    s.init = 0.1;
    s.perf = 0.3
    s.th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    s.sensorRange = [30000];
    s.actuatorRange = [36];
    s.settlingTime = 13; 
end
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

s.x_act(:,1) =[2*s.init*s.safex(1,1)*rand(1)-s.safex(1,1)*s.init;...
                    2*s.init*s.safex(1,2)*rand(1)-s.safex(1,2)*s.init]
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
            [rlNumericSpec([5 1])]};% 3 observeations in continuous domain: residue,output,actuation
%     'LowerLimit',[-inf -inf 0  ]',...
%     'UpperLimit',[ inf  inf inf]');
obsInfo{1,1}.Name , obsInfo{1,2}.Name, obsInfo{1,3}.Name = 'observations';
% obsInfo{1,1}.Name , obsInfo{1,2}.Name ='observations';
obsInfo{1,1}.Description , obsInfo{1,2}.Description,...
    obsInfo{1,2}.Description = 'residue,output and control input';
numObservationsAtk = obsInfo{1,1}.Dimension(1);
numObservationsCon = obsInfo{1,2}.Dimension(1);
numObservationsDtc = obsInfo{1,3}.Dimension(1);
% obsInfo{1,1}.Datatype , obsInfo{1,2}.Datatype, obsInfo{1,3}.Datatype= long g;
%% Multi Agent Action Properties
actInfo = {rlNumericSpec([2 1],'LowerLimit', -s.sensorRange,'UpperLimit',s.sensorRange),...
    rlNumericSpec([1 1],'LowerLimit', -s.actuatorRange,'UpperLimit',s.actuatorRange),...
                rlNumericSpec([2 1],'LowerLimit', 0,'UpperLimit',inf)};
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
actorAtk = rlDeterministicActorRepresentation(actorNetwork,obsInfo{1,1},...
    actInfo{1,1},'Observation',{'StateAtk'},'Action',{'actorTanhAtk'},...
    actorOptions);

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
actorCon = rlDeterministicActorRepresentation(actorNetwork,obsInfo{1,2},...
    actInfo{1,2},'Observation',{'StateCon'},'Action',{'actorTanhCon'},...
    actorOptions);

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
actorDtc = rlDeterministicActorRepresentation(actorNetwork,obsInfo{1,3},...
    actInfo{1,3}, 'Observation',{'StateDtc'},'Action',{'actorTanhDtc'},...
    actorOptions);

%% DDPG Agents and their properties
agentOpts = rlDDPGAgentOptions(...
    'SampleTime',Ts,...
    'TargetSmoothFactor',1e-3,...
    'DiscountFactor',0.98, ...
    'MiniBatchSize',64, ...
    'ExperienceBufferLength',1e6); 
agentOpts.NoiseOptions.Variance = 0.3;
agentOpts.NoiseOptions.VarianceDecayRate = 1e-5;
agentOpts.SaveExperienceBufferWithAgent = 1;
%% environment creation
                                          
agents= [];
observations={};
actions={};
agentObjs=[];
agentAttacker= "/RL Attacker Agent";
agentController= "/RL Controller Agent";
agentDetector= "/RL Detector Agent";

if doReset
    agentAtk = rlDDPGAgent(actorAtk,criticAtk,agentOpts);
     agentCon = rlDDPGAgent(actorCon,criticCon,agentOpts);
      agentDtc = rlDDPGAgent(actorDtc,criticDtc,agentOpts);
end

if ~exist('agentAtk','var')
    agentAtk = rlDDPGAgent(actorAtk,criticAtk,agentOpts);
end
if ~exist('agentCon','var')
    agentCon = rlDDPGAgent(actorCon,criticCon,agentOpts);
end
if ~exist('agentDtc','var')
    agentDtc = rlDDPGAgent(actorDtc,criticDtc,agentOpts);
end
    
% a for attcker agent = 2
if whichAgents.contains("a")
    agents= [agents model+agentAttacker];
    observations{end+1}= obsInfo{1,1};
    actions{end+1}= actInfo{1,1};
    agentObjs= [agentObjs agentAtk];
    whichAg_sim= whichAg_sim*2;
end
% c for controller agent = 3
if whichAgents.contains("c")
    agents= [agents model+agentController];
    observations{end+1}= obsInfo{1,2};
    actions{end+1}= actInfo{1,2};
    agentObjs= [agentObjs agentCon];
    whichAg_sim= whichAg_sim*3;
end
% d for detector agent = 5
if whichAgents.contains("d")
    agents=[agents model+agentDetector];
    observations{end+1}= obsInfo{1,3};
    actions{end+1}= actInfo{1,3};
    agentObjs= [agentObjs agentDtc];
    whichAg_sim= whichAg_sim*5
end
% agents = agent1;
% open_system(model);
env = rlSimulinkEnv(model,agents)%,observations,actions);
% obsInfo=getObservationInfo(env); actInfo=getActorInfo(env); %% How to
%% reset the environment in every iteration/episode
% @(in)localResetFcn(in); env.ResetFcn = @(in);
env.ResetFcn = @(in) randomReset(in, s.init, s.safex, simlen, xdim, ydim, ...
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
    'StopTrainingValue',3500,...
    'SaveAgentCriteria','EpisodeReward',...
    'SaveAgentValue',3000)%,...
%     'SaveExperienceBufferWithAgent',false);


USE_PRE_TRAINED_MODEL = ~doTraining; % Set to true, to use pre-trained
if doTraining
    agentOpts.ResetExperienceBufferBeforeTraining = false;
    % run with new agents
    trainingStats = train(agentObjs,env,trainOpts);  
    dt=strrep(datestr(datetime),':','_');
    mkdir(trainOpts.SaveAgentDirectory,dt);
    savedir= trainOpts.SaveAgentDirectory+'/'+dt;
    save(savedir + "/AttckerAgentFin.mat",'agentAtk');
    save(savedir + "/ControllerAgentFin.mat",'agentCon');
    save(savedir + "/DetectorAgentFin.mat",'agentDtc');
else
    % Load the pretrained agent for the example.
    disp("Use trained model");
    % Set agent option parameter:
    agentOpts.ResetExperienceBufferBeforeTraining = not(USE_PRE_TRAINED_MODEL);
    % Load experiences from pre-trained agent    
    sprintf('- Continue training pre-trained model: %s', PRE_TRAINED_MODEL_FILE);   
    load(PRE_TRAINED_MODEL_DIR+"/AttckerAgentFin.mat",...
        'agentAtk');
    load(PRE_TRAINED_MODEL_DIR+"/DetectorAgentFin.mat",...
        'agentDtc');
    load(PRE_TRAINED_MODEL_DIR+"/ControllerAgentFin.mat",...
        'agentCon');
    agentObjs = [atk_agent dtc_agent ctrl_agent];
end
% Simulate the agent.
if ifSim
    simOpts = rlSimulationOptions('MaxSteps',maxsteps,'StopOnError','on');
    experiences = sim(env,agentObjs,simOpts);
    save(savedir + "/trainedSimulation.mat",'experiences');
end
