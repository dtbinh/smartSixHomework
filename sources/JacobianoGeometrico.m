function [ J ] = JacobianoGeometrico( q )
%JacobianoGeometrico: This function allow the creation of the geometric Jacobian matrix with the joints variables.
%Input variable q: the values of the variables of each joints.
%Output J: the geometric Jacobian of the manipulator in the configuration gave by q.





% Utilisation of the function of the first Homework to get the A matrix and the position vector of the end-effector
[ p, ~, ~, A ] = cindir( q, 'ZYZ' );
%----------------------------------------------------------------------------------------------------------


% Memory allocation for the used variables and for the Jacobian
J = zeros(6,8);

z = zeros(3,1,7); % Axis vector z1 to axis vector z7

p_tilde = zeros(4,1,7); % Position vector p1 to position vector p7
%----------------------------------------------------------------------------------------------------------


% Creation of the constants data
Ab0 = [0  0 1 0; ...
       0  1 0 0; ...
       -1 0 0 0.2985; ...
       0  0 0 1          ]; %Creation of the rototranslation matrix of the base frame to the frame 0
    
zb = [0 0 1]';% Creation of the initial value of the z axis vector

pb_tilde = [0 0 0 1]'; % Creation of the postion vector of the base frame, put in vector of dimension (4 x 1)

pe_tilde = [p; 1]; % Creation of the postion vector of the frame of the end-effector, put in vector of dimension (4 x 1)
%----------------------------------------------------------------------------------------------------------


%Computation of the z axis vectors and of the positions vectors of each joint frame
Ai = Ab0; %Rototranslation matrix from the base frame the the frame i (here i=0, initial value)
%p0_tilde = Ai*pb_tilde; %Creation of the position vector p0 of the origine of the frame 0
z0 = Ai(1:3, 1:3)*zb; % Creation of the axis vector z0 of the frame 0
for i = 1:7
    Ai = Ai*A(:,:,i);
    p_tilde(:,:,i) = Ai*pb_tilde; % Computation of the positions vectors of each joint
    z(:,:,i) = Ai(1:3, 1:3)*zb; % Computation of the z axis of the frame of each joint
    
end
%----------------------------------------------------------------------------------------------------------


% Computation of the Jacobian matrix
J(:,1) = [z0 ; [0 0 0]']; % Prismatic joint 1
J(:,2) = [z(:,:,1) ; [0 0 0]']; % Prismatic joint 2
for i=3:8 % Revolute joint 3 to revolute joint 8
       J(:,i) = [cross( z(:,:,i-1), (pe_tilde(1:3)-p_tilde(1:3,:,i-1)) ); z(:,:,i-1)];
end
%----------------------------------------------------------------------------------------------------------
