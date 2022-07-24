clc;
clear slBus1;

% log? then on
% diary on;
%% configuration params
% choose among {esp_bicycle,dcmotor,quadrotor,fuel_injection,driveline_mngmnt,esp,ttc}
system = "trajectory"

% agent activation: a= attacker, d= detector, c= controller
whichAgents = "ad";
whichAg_sim=1; % same as whichAgents: numeric for simulink

model = "envModel_rl";
open(model);
% reset: in case models are not trained right, 
% ..make fresh models removing the current object
doReset = true;    % if you want to dreshly create model,agents
doTraining = false;  % if you want to newly train the agents
doSimulation = true; % if you want to simulate after training
loadPreTrained = true;  % if you want to reuse well trained agents
if loadPreTrained
    % choose a backed up folder from savedAgents folder
    PRE_TRAINED_MODEL_DIR = "savedAgents/15-Jul-2021_23-17-25_"+system+"_"+whichAgents;
        %01-Apr-2021_20-28-26_esp_ad";
%     31-Mar-2021_08-19-09_esp_ad";lastth0
end

%% log? then on, else comment out
dt=strrep(datestr(datetime),':','-');dt=strrep(dt,' ','_');
% mkdir('..','logs');
% logfile = '..\logs\'+system+'_'+whichAgents+'_'+dt+'.log';
% diary(logfile);
% diary on;

%% systems
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
    % % safer region of this system to start from
    s.init = 0.9;
    % from perfReg.py with this system
    s.perf = 0.1;
    % for central chi2 FAR < 0.05
    s.th = 4.5; 
    s.settlingTime = 5 ;
    s.sensorRange = [2.5] ;     % columnwise range of each y
    s.actuatorRange = [0.8125]; % columnwise range of each u
    s.proc_noise_var=0.001;
    s.meas_noise_var= 0.0001;
    % from system_with_noise.m with this system
    s.noisy_zvar=0.27;
    % from system_with_noise.m with this system
    s.noisy_zmean= 1.68;
    s.noisy_delta= 11.1;
    s.nonatk_zvar= 0.1169;
    s.nonatk_zmean= 0.0858;
    s.nonatk_delta= 0.063;
    s.uatkon=[1];   % attack on which u
    s.yatkon=[1];   % attack on which y
end

if system=="esp_journal"
%%     esp journal
    s.Ts=0.04;
    s.A = [0.6278   -0.0259;
        0.4644    0.7071];

    s.B = [0.1246   -0.00000028;
        3.2763    0.000016];

    s.C = [0    1.0000
     -338.7813    1.1293];

    s.D = [0         0;
      169.3907         0];

    s.K = [5.261 -0.023;
        -414911.26, 57009.48];

    s.L = [-0.00000000002708 -0.00000000063612;
        0.00000000033671  0.00000000556308];
    
    s.safex = [1,2];
    % from perfReg.py with this system
    s.init = 1;
    % from perfReg.py with this system
    s.perf = 0.2;
    % for central chi2 FAR < 0.05
    s.th = 4.35; 
    s.settlingTime = 12 ;
    s.sensorRange = [2.5;15];  % columnwise range of each y
    s.actuatorRange = [0.8125;10000]; % columnwise range of each u
    s.proc_noise_var=0.01;
    s.meas_noise_var= 0.001;
    % from system_with_noise.m with this system
    s.noisy_zvar=[10000 -21;-21 15727];   
    % from system_with_noise.m with this system
    s.noisy_zmean= [3.6 -372.7];
    s.noisy_delta= 41;
    s.uatkon=[1;0];   % attack on which u
    s.yatkon=[1;1];   % attack on which y
end

if system=="trajectory"
%% trajectory tracking
    s.Ts = 0.1;
    s.A = [1.0000    0.1000; 0    1.0000];
    s.B = [0.0050; 0.1000];
    s.C = [1 0];
    s.D = [0];
%     s.K = [16.0302    5.6622];  % settling time around 15
%     s.L = [0.9902; 0.9892];
%   ---new values for better intermediate safety---
%     Q= eye(size(s.A,2));
%     R= eye(size(s.B,2));
%     [K,S,E] = dlqr(s.A,s.B,Q,R);
%     QN = 1500;
%     RN = eye(1);
%     sys_ss = ss(s.A,s.B,s.C,s.D,s.Ts);
%     [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
    s.K = [0.9171    1.6356];  
    s.L = [0.8327;   2.5029];
    s.safex = [25,30];
    % safer region of this system to start from
    s.init = 0.6;
    % from perfReg.py with this system
    s.perf = s.init;% 0.3;
    % for central chi2 FAR < 0.05
    s.th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    s.sensorRange = [30];  % columnwise range of each y
    s.actuatorRange = [72]; %[36];   % columnwise range of each u
    s.settlingTime = 13; 
    s.proc_noise_var=0.01;
    s.meas_noise_var= 0.001;
    % from system_with_noise.m with this system
    s.noisy_zvar= 0.14;
    % from system_with_noise.m with this system
    s.noisy_zmean= 0.52;
    s.noisy_delta= 1.86;
    s.nonatk_zvar= 12.6041;%15.8507
    s.nonatk_zmean= 0.6064;
    s.nonatk_delta= 0.0292;
    s.uatkon=[1];   % attack on which u
    s.yatkon=[1];   % attack on which y
end


%% training/simulation length
simlen=80;
maxepisodes  = 3000;
% simlen=ceil(Tf/Ts);
% maxepisodes  = 5;
% agents= model+["/RL Attacker Agent","/RL Controller Agent"]%...
%                                             "/RL Detector Agent"];
%% Sampling Period, Episode duration
% simlen=5;
Ts = s.Ts;
Tf = simlen*Ts;
%% dimensions
% s.z=[0,0];
xdim= size(s.A,2);
ydim= size(s.C,1);
udim= size(s.B,2);
%% ranges
ulim= s.actuatorRange;
ylim= s.sensorRange;
thlim= 10;
taulim= 1;
%% random noise vars
rng shuffle;
seed= rng;
s.proc_noise= s.proc_noise_var*rand(xdim,simlen);
s.meas_noise= s.meas_noise_var*rand(ydim,simlen);
%% init system vars
 atkOn=randi([0,1]);
isatk=0;
s.time = 0.00;  
s.x_act =zeros(xdim,simlen);
s.x_act(:,1) =(2*s.init*s.safex*rand(xdim)-s.safex*s.init)';
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
s.z_mean= ones(size(s.z,1),simlen) ;
s.z_var =ones(size(s.z,1),simlen);
s.g = zeros(1,simlen);
s.chi_tst= zeros(1,simlen);
s.threshold= s.th*ones(1,simlen);
s.tau= ones(1,simlen);
s.non_cent= zeros(1,simlen);
s.avgfar= chi2cdf(s.th,1*size(s.C,1),'upper')*ones(1,simlen);
s.avgtpr = ncx2cdf(s.th,1*size(s.C,1),s.non_cent(1),'upper')*ones(1,simlen);
% save("system.mat",'-struct','s');
%% Agent Observation / Action Dimensions
atkActDim= udim+ydim;%[au;ay]
dtcActDim= 2; %[th;l]
% dtcActDim= 1; %[th;l]
conActDim= udim; % [u_new]
atkObsDim= ydim+udim+1+atkActDim;%[y;u;g;prevAct]
dtcObsDim= dtcActDim+1+1;%[prevAct;g;delta]
conObsDim= udim+conActDim+ydim+1+size(s.C*s.safex',1);%[ulim;prevAct;y;ref;safey]
%% Multi Agent Observation Properties
% 1-- attacker, 2-- Controller, 3-- Detector
obsInfo = {[rlNumericSpec([atkObsDim 1])], [rlNumericSpec([conObsDim 1])],...
            [rlNumericSpec([dtcObsDim 1])]};% 3 observeations in continuous domain: residue,output,actuation
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
% 1-- attacker, 2-- Controller, 3-- Detector
actInfo = {rlNumericSpec([atkActDim 1],...
            'LowerLimit', -[ulim;ylim],'UpperLimit',[ulim;ylim]),...
           rlNumericSpec([conActDim 1],...
            'LowerLimit', -ulim,'UpperLimit',ulim),...
           rlNumericSpec([dtcActDim 1],...
            'LowerLimit',[0;1],'UpperLimit',[thlim;taulim])};
%         'LowerLimit',[0;1],'UpperLimit',[50;10])};
actInfo{1,1}.Name = 'actuator n sensor attack';
actInfo{1,2}.Name = 'control input';
actInfo{1,3}.Name = 'threshold n window';
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
    tanhLayer('Name','actorTanhAtk')
    scalingLayer('Name','actorScalingAtk','Scale',[ulim;ylim].*[s.uatkon;s.yatkon])];
actorOptions = rlRepresentationOptions('LearnRate',1e-04,'GradientThreshold',1);
actorAtk = rlDeterministicActorRepresentation(actorNetwork,obsInfo{1,1},...
    actInfo{1,1},'Observation',{'StateAtk'},'Action',{'actorScalingAtk'},...
    actorOptions);

%% Actor for Controller Agent
actorNetwork = [
    featureInputLayer(numObservationsCon,'Normalization','none','Name','StateCon')
    fullyConnectedLayer(500, 'Name','actorFC1')
    reluLayer('Name','actorRelu1')
    fullyConnectedLayer(400, 'Name','actorFC2')
    reluLayer('Name','actorRelu2')
    fullyConnectedLayer(numActionsCon,'Name','actorFC3')
    tanhLayer('Name','actorTanhCon')
    scalingLayer('Name','actorScalingCon','Scale',ulim)];
actorOptions = rlRepresentationOptions('LearnRate',1e-04,'GradientThreshold',1);
actorCon = rlDeterministicActorRepresentation(actorNetwork,obsInfo{1,2},...
    actInfo{1,2},'Observation',{'StateCon'},'Action',{'actorScalingCon'},...
    actorOptions);

%% Actor for Detector Agent
actorNetwork = [
    featureInputLayer(numObservationsDtc,'Normalization','none','Name','StateDtc')
    fullyConnectedLayer(500, 'Name','actorFC1')
    reluLayer('Name','actorRelu1')
    fullyConnectedLayer(400, 'Name','actorFC2')
    reluLayer('Name','actorRelu2')
    fullyConnectedLayer(numActionsDtc,'Name','actorFC3')
    tanhLayer('Name','actorTanhDtc')
    scalingLayer('Name','actorScalingDtc','Scale',[thlim/2;ceil(taulim/2)],'Bias',[thlim/2;ceil(taulim/2)+1])];
actorOptions = rlRepresentationOptions('LearnRate',1e-04,'GradientThreshold',1);
actorDtc = rlDeterministicActorRepresentation(actorNetwork,obsInfo{1,3},...
    actInfo{1,3}, 'Observation',{'StateDtc'},'Action',{'actorScalingDtc'},...
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
agentOpts.SaveExperienceBufferWithAgent = 0;
%% environment creation
                                          
agents= [];
observations={};
actions={};
agentObjs=[];
target=[];
targetAvgRwd = 10000;
agentAttacker= "/attacker/RL Attacker Agent";
agentController= "/controller/RL Controller Agent";
agentDetector= "/detector/RL Detector Agent";

% create models freshly if doReset is true..
% and not loading a pretrained model
if doReset & ~loadPreTrained
    agentAtk = rlDDPGAgent(actorAtk,criticAtk,agentOpts);
     agentCon = rlDDPGAgent(actorCon,criticCon,agentOpts);
      agentDtc = rlDDPGAgent(actorDtc,criticDtc,agentOpts);
end

if loadPreTrained
        % Load the pretrained agent for the example.
    disp("Use trained model");
    % Set this agent option to false if you want to
    % load experience buffers from pre-trained agent?
    agentOpts.ResetExperienceBufferBeforeTraining = not(loadPreTrained);
     
    sprintf('load pre-trained model from: %s', PRE_TRAINED_MODEL_DIR);   
    load(PRE_TRAINED_MODEL_DIR+"/AttckerAgentFin.mat",...
        'agentAtk');
    load(PRE_TRAINED_MODEL_DIR+"/DetectorAgentFin.mat",...
        'agentDtc');
    load(PRE_TRAINED_MODEL_DIR+"/ControllerAgentFin.mat",...
        'agentCon');
end

% do not create models if already in workspace
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
    target= [target targetAvgRwd];
end
% c for controller agent = 3
if whichAgents.contains("c")
    agents= [agents model+agentController];
    observations{end+1}= obsInfo{1,2};
    actions{end+1}= actInfo{1,2};
    agentObjs= [agentObjs agentCon];
    whichAg_sim= whichAg_sim*3;
    target= [target targetAvgRwd];
end
% d for detector agent = 5
if whichAgents.contains("d")
    agents=[agents model+agentDetector];
    observations{end+1}= obsInfo{1,3};
    actions{end+1}= actInfo{1,3};
    agentObjs= [agentObjs agentDtc];
    whichAg_sim= whichAg_sim*5;
    target= [target targetAvgRwd];
end
% save("system.mat",'s');
% save("test.mat",'Simulink.Bus.createObject(s)');
Simulink.Bus.createObject(s);
% s= evalin('base','s');
% agents = agent1;
% open_system(model);
env = rlSimulinkEnv(model,agents,observations,actions);
% obsInfo=getObservationInfo(env); actInfo=getActorInfo(env); %% How to
%% reset the environment in every episode
% @(in)localResetFcn(in); env.ResetFcn = @(in);
env.ResetFcn = @(in) randomReset(in, s.init, s.safex, simlen, xdim, ydim, ...
                   ylim, ulim, s.C, s.K, s.th,s.non_cent(1),s.proc_noise_var,s.meas_noise_var);

%% Training
maxsteps = simlen;
trainOpts = rlTrainingOptions(...
    'MaxEpisodes',maxepisodes, ...
    'MaxStepsPerEpisode',maxsteps, ...
    'ScoreAveragingWindowLength',10, ...
    'Verbose',false, ...
    'Plots','training-progress',...
    'StopTrainingCriteria','AverageReward',...
    'StopTrainingValue',target,...
    'SaveAgentCriteria','EpisodeReward',...
    'SaveAgentValue',target)%,...
%     'SaveExperienceBufferWithAgent',false);


if doTraining
    agentOpts.ResetExperienceBufferBeforeTraining = false;
    
    % make directory with model, specs in name
%     dt=strrep(datestr(datetime),':','-');dt=strrep(dt,' ','_');
    dtf=dt+"_"+system+"_"+whichAgents;
    mkdir(trainOpts.SaveAgentDirectory,dtf);
    savedir= trainOpts.SaveAgentDirectory+'/'+dtf;

    % train agents
    trainingStats = train(agentObjs,env,trainOpts);  
    % save trained agents in created directory
    save(savedir + "/AttckerAgentFin.mat",'agentAtk');
    save(savedir + "/ControllerAgentFin.mat",'agentCon');
    save(savedir + "/DetectorAgentFin.mat",'agentDtc');
else
    % Load the pretrained agent for the example.
    disp("Use well trained model from workspace");
end
% Simulate the agent.
if doSimulation
    simOpts = rlSimulationOptions('MaxSteps',maxsteps,'StopOnError','on');
    experiences = sim(env,agentObjs,simOpts);
    save(savedir + "/trainedSimulation.mat",'experiences');
end
