function [ Ja ] = jacobianoAnalitico( q, EulerAngles )

    [ ~, Phi, ~, ~] = cindir(q, EulerAngles);
    
    %phi = 
    Phi = Phi';
    phi = Phi(1);
    theta = Phi(2); % check if it's ok but no...
    
    
    T = [ 0 -sin(phi) cos(theta)*sin(theta); ...
        0 cos(phi) sin(phi)*sin(theta); ...
        1 0 cos(theta)]; % getting the T matrix from Euler angles to get Ja
    
    Ta = [ eye(3,3) zeros(3,3); ...
        zeros(3,3) T]; % changing to Ta so that we can multiply it by the
                        % Geometric jacobian
    
    J = JacobianoGeometrico(q); % getting the geometric jacobian
    
    Ja = inv(Ta)*J % Ja form from J = Ta(Phi)*Ja => gives Ja
    
end