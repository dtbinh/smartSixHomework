clc;
warning off
%StopVrepSimulation(); %pre precauzione


%%%%%%%%%%%%%%%%%%%%%%%%%%   Set Path  %%%%%%%%%%%%%%%%%%%%%%%%%%
%restoredefaultpath
addpath(genpath('/usr/local/src/vrep/'))
% OGNUNO DI VOI DOVRA' METTERE IL PATH ALLA CARTELLA PROGRAMMING NELLA
% DIRECTORY DI INSTALLAZIONE DI VREP
%addpath(genpath('C:\Program Files (x86)\V-REP3\V-REP_PRO_EDU\programming'));
addpath(genpath('./')); %aggiungo il path corrente al path di matlab in modo che le funzioni nelle sottodirectory possano viste dal Matlab
savepath;


%%%%%%%%%              Inizializzazione  %%%%%%%%%%%%%%%%%%%%%%%%%
InitConnectionWithSimulator();%initialize the communication with the Simulator
%MotorParametersBushelessLafert(); %load in workspace the parameters of the motors



T = 0.001; %sample time. (USARE QUESTO VALORE.)



%ESEMPIO DI MOVIMENTAZIONE DEL ROBOT
%JOINT = [1 1 0 -pi/2 pi/4 0 0 0]; %(m (x), m(y), rad(joint1), rad(joint2), rad(joint3), rad(joint4), rad(joint5), rad(joint6))
%COMANDO: SendPoseToVRep(JOINT);
