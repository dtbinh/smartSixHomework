%Launch this file to start the communication with v-rep

global vrep client Robot Mode
IP = '127.0.0.1'; %IP of the machine on which vrep runs
Port = 19998;
Timeout = 5000; %timeout for response (ms)
SampleTimeComm =  10; %ms sample time for communication with vrep 



vrep=remApi('remoteApi','extApi.h');
client = vrep.simxStart(IP, Port, true, true, Timeout, SampleTimeComm);
Mode = vrep.simx_opmode_oneshot; %communication mode (see manual remote API)



pause(0.2);


%%%%%%%% THE HANDLE TO JOINTS NEEDS TO BE READ INORDER TO SET THEIR VALUE
%%%%%%%% LATER

%HANDLES TO REVOLUTE JOINTS OF THE MANIPULATOR
for i=1:6
    [err, Robot.Manipulator.RevJointHandle(i)] = vrep.simxGetObjectHandle(client, ['Joint', num2str(i)],  vrep.simx_opmode_oneshot_wait);
   if(err)
       error(['Err reading pointer to revolute joint of the manipulator. Joint: ', num2str(i), '. Check that Vrep is active and the connection established']);
   end
end


%HANDLES TO THE PRISMATIC JOINT OF THE BASE
[err, Robot.MobileBase.PrismJointXHandle] = vrep.simxGetObjectHandle(client, ['JointXBase'],  vrep.simx_opmode_oneshot_wait);
   if(err)
       error(['Err reading pointer to primsatic joint X of the base']);
   end

[err, Robot.MobileBase.PrismJointYHandle] = vrep.simxGetObjectHandle(client, ['JointYBase'],  vrep.simx_opmode_oneshot_wait);
    if(err)
       error(['Err reading pointer to Primsatic joint Y of the base']);
   end

%Start the simulator
 err=vrep.simxStartSimulation(client, vrep.simx_opmode_oneshot_wait);
 if(err>1)
   error(['Err starting simulation']);
 end
 
