p = 0;
Phi = 0;
R = zeros(3,3);
A = zeros(4,4);
T = zeros(4,4);
[p, Phi, R, A] = cindir([0 0 0 0 0 0 pi/2 0], 'RPY');
display(A);
