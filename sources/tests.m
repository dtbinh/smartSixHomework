p = 0;
Phi = 0;
R = zeros(3,3);
A = zeros(4,4);
T = zeros(4,4);
q = [10 0 0 0 0 0 0 0];
EulerAngles = 'RPY';
%[p, Phi, R, A] = cindir(q, 'RPY');


Ja = pseudoInverseJacobian(q, EulerAngles);

