function [K,P,Kth,Pth] = nonlinGetLQR
m = 6; d = 0.1; % Controlling x' and y'
A = [0   1  0  0 ;
     0 -d/m 0  0 ;
     0   0  0  1 ;
     0   0  0 -d/m];
B = [0    0;
     1    0;
     0    0;
     0    1];
 
Q = eye(4);
R = eye(2);

[K,~,~] = lqr(A,B,Q,R);

P = care(A,B,Q,R);

% Stabilizing theta

Ath = [0 1;
       0 0];
   
Bth = [0 0
      -1 0];
Qth = eye(2);
Rth = eye(2);

[Kth,~,~] = lqr(Ath,Bth,Qth,Rth);

Pth = care(Ath,Bth,Qth,Rth);

%% GOOD LQR
%    m = 6; d = 0.1; gx = 0; gy = 9.8; r = 0.25; J = 0.1425;  
    
        % Model
%     Nil = [0, 0; 0, 0];
%     Ax = [0, 1; 0, -d/m];      Bx = Nil;   
%     Ay = Ax;                   By = [0, 0; 1/m, 1/m];
%     At = [0,1;0,0]; C=At;      Bt = [0, 0; r/J, -r/J];         
%     A = [Ax, Nil, [0,0; -gy,0]; Nil, Ay, Nil; Nil, Nil, At]; 
%     B = [Bx; By; Bt];
%     p = [-3, -4, -5, -6, -7, -8];
%     K = place(A,B,p); M = A-B*K;
%     Q = eye(6);
%     P;  P = care(A,B,Q);

%Good
%     P = [1.7235    0.9801    0.0000    0.0000   -2.9568   -0.4031;
%          0.9801    1.3794    0.0000    0.0000   -4.6560   -0.6881;
%          0.0000    0.0000    3.0806    4.2426   -0.0000    0.0000;
%          0.0000    0.0000    4.2426   12.7700   -0.0000    0.0000;
%         -2.9568   -4.6560   -0.0000   -0.0000   21.6575    3.8713;
%         -0.4031   -0.6881    0.0000    0.0000    3.8713    1.1917];


end