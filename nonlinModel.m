function dx = nonlinModel(x, Ref)
m = 6; d = 0.1;
K= [1.0000    1.7155         0         0         0         0 ;
         0         0    1.0000    1.7155         0         0];
u = linController(x, Ref, K, K);

A = [0   1  0  0   0  0;
     0 -d/m 0  0   0  0;
     0   0  0  1   0  0;
     0   0  0 -d/m 0  0;
     0   0  0  0   0  1;
     0   0  0  0   0  0];
 B = [0    0;
      1    0;
      0    0;
      0    1;
      0    0;
      0    0];
dx = A*x+B*u;
end