function [err]=setPosDH(vrep, clientID, Robot, JointDH, mode)
%Invia a vrep la posizione ai giunti del manipolatore

%1) vrep e' l'oggetto vrep precedentemente creato
%2) clientID e' il descrittore del socket di comunicazione restituito da vrep.simxStart
%3) Robot   e' la struttura contente gli handles ai giunti puntatore agli oggetti giunti restituito da vrep.simxGetObjectHandle
%4) JointDH [8,1] e' la posizione ai giunti in DH da inviare a vrep. I
%   1) JointDH[1]: valore giunto prismatico che muove la base lungo x (m)
%   2) JointDH[2]: valore giunto prismatico che muove la base lungo y (m)
%   3) JointDH[3-8]: valori giunti rotoidali del manipolatore (rad)
%5) mode è la modalita'  di comunicazione. Vedere documentazione API



err(1) = vrep.simxSetJointPosition(clientID, Robot.MobileBase.PrismJointXHandle, JointDH(1), mode);
err(2) = vrep.simxSetJointPosition(clientID, Robot.MobileBase.PrismJointYHandle, JointDH(2), mode);


for i=1:6
    err(i+2) = vrep.simxSetJointPosition(clientID, Robot.Manipulator.RevJointHandle(i), JointDH(i+2), mode);
end