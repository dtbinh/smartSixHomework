function [ p, Phi, R, A] = cindir( q, EulerAngles )


%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


    %Memory allocation for the output variables of the function
    A = zeros([4 4 8]); %Creation of the multidimensional array for the A matrices (8 matrices of 4x4 dimension)
    
    R = zeros([3 3]); %Creation of the rotation matrix
    
    Phi = zeros([3 1]); %Creation of the Euler Angles vector
    
    p = zeros([3 1]); %Creation of the position vector
    %----------------------------------------------------------------------------------------------------------
    
    %Creation of the constants 
    Ane = [1 0 0 0; 0 1 0 0; 0 0 1 0.0848; 0 0 0 1]; %Creation of the rototranslation matrix of the frame 8 to the end-effector
    
    Ab0 = [0 0 1 0; 0 1 0 0; -1 0 0 0.2985; 0 0 0 1]; %Creation of the rototranslation matrix of the base to the frame 0
    
    alpha_i = [ pi/2 pi/2 -pi/2 0 -pi/2 pi/2 -pi/2 0 ]; %Creation of the vector of angles between the axes z and z-1 about the x axis        
        
    a = [ 0 0 0.150 0.590 0.130 0 0 0]; %Creation of the vector of distances between the Oi and the Oi' origins
    %----------------------------------------------------------------------------------------------------------
    
    limiti_giunto_inf = [-200 -200 2.9671   -3.0543    1.3963    3.6652   -2.2689    9.4248]; %m m radx6
    limiti_giunto_sup = [ 200  200 -2.9671    1.1345   -1.5708   -3.6652    2.2689   -3.1416]; %m m radx6
    
    if length(q) == 8 %Test of the dimension of the q vector
    for i = [1,2,4,7] % test of the limit inf for negative to positive range
       if q(i) < limiti_giunto_inf(i) 
           X = ['The value of the junction ',num2str(i),' is too little'];
           disp(X);
           return;
       end
       if q(i) > limiti_giunto_sup(i)
           X = ['The value of the junction ',num2str(i),' is too high'];
           disp(X);
           return;
       end
    end
    for i = [3,5,6,8] % test of the limit inf for positive to negative range
       if q(i) > limiti_giunto_inf(i) 
           X = ['The value of the junction ',num2str(i),' is too little'];
           disp(X);
           return;
       end
       if q(i) < limiti_giunto_sup(i)
           X = ['The value of the junction ',num2str(i),' is too high'];
           disp(X);
           return;
       end
    end
        
        %Creation of the joint variables from the vector q
        di = [ q(1) q(2) 0.450 0 0 0.64707 0 0.095]; %Creation of the distance vector for coordinate of the Oi origins along zi?1 

        theta_i = [ 0 0 q(3) q(4) q(5) q(6) q(7) q(8) ]; %Creation of the angle vector for the angles between axes xi?1 and x i about axis zi?1
        %----------------------------------------------------------------------------------------------------------
        
        
        

        %Computing of the differents A matrices
        for i= 1:8
        A(:,:,i) = [ cos(theta_i(i)) -sin(theta_i(i))*cos(alpha_i(i)) sin(theta_i(i))*sin(alpha_i(i)) a(i)*cos(theta_i(i));...
        sin(theta_i(i)) cos(theta_i(i))*cos(alpha_i(i)) -cos(theta_i(i))*sin(alpha_i(i)) a(i)*sin(theta_i(i));...
        0 sin(alpha_i(i)) cos(alpha_i(i)) di(i);...
        0 0 0 1];
        end
        %----------------------------------------------------------------------------------------------------------
        

        
        
        T08 = A(:,:,1)*A(:,:,2)*A(:,:,3)*A(:,:,4)*A(:,:,5)*A(:,:,6)*A(:,:,7)*A(:,:,8); %Computinf of the rototranslation matrix between the frame 0 and the frame 6
        T = Ab0*T08*Ane; %Computing of the direct kinematics equations
        

        p = T([1 2 3], 4); %Extraction of the position vector from the direct kinematics equations
        R = T([1 2 3], [1 2 3]); %Extraction of the rotation matrix from the direct kinematics equations
        
        
        %Exctraction of the Euler angles from the direct kinematics equations
        if strcmp('ZYZ', EulerAngles) %Test to know if the Euler angles are the 'ZYZ' angles
            
            phi = atan2(R(2,3),R(1,3)); %angle around the z axis (of the first frame)
            
            theta = atan2(sqrt(R(1,3)^2 + R(2,3)^2), R(3,3)); %angle around the y' axis (of the second frame)
            
            psi = atan2(R(3,2), -R(3,1)); %angle around the z'' axis (of the third frame)
            
            Phi = [phi; theta; psi]; %Euler vector angle for the 'ZYZ' convention
            
        elseif strcmp('RPY', EulerAngles)|| strcmp('ZYX', EulerAngles) %Test to know if the Euler angles are the 'RPY' angles (or 'ZYX', because they are the same)
            
            phi = atan2(R(2,1),R(1,1)); %angle around the x axis
            
            theta = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2)); %angle around the y axis
            
            psi = atan2(R(3,2), -R(3,3)); %angle around the z axis
            
            Phi = [phi; theta; psi]; %Euler vector angle for the 'RPY' convention
            
        else %Display an error message if the character chains are not recognized
            
            disp('The characters chain are not recognized, please use "ZYZ" for the first convention of the Euler angles or use "RPY" or "ZYX" for the second convention of Euler angles.');
            
        end

    else %Display an error message if the size of the vector q is not the good one
        disp('the vector q should contain 8 parameters');
    end

end

