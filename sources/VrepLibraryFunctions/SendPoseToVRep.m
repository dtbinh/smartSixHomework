function [err]=SendPoseToVRep(in)
%Invia a vrep la posizione ai giunti del manipolatore.
%Da usare in simulink. Il vettore "in" è costituito da [time JointDH] (dimensione [8,1])

global vrep client Robot Mode



JointDH = in(1:end);
err(1:8) = 0;

err = setPosDH(vrep, client, Robot, JointDH, Mode);
