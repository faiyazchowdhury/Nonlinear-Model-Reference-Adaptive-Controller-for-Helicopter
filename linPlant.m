function dx = linPlant(x, u)
    m = 6; d = 0.1; gy = 9.8; J = 0.1425; r = 0.25;
    
    A = [0   1  0  0   0  0;
         0 -d/m 0  0  -gy 0;
         0   0  0  1   0  0;
         0   0  0 -d/m 0  0;
         0   0  0  0   0  1;
         0   0  0  0   0  0];
     B = [0    0;
          0    0;
          0    0;
          1    1;
          0    0;
         r/J -r/J];
    
    dx = A*x+B*u;
end