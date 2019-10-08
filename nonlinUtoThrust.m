function f = nonlinUtoThrust(u, th)
    m = 6; gy = 9.8;
    f = u;
    f(2) = u(2) + gy; %Now in terms of f_lr and f_ud
    M = [-1/m*sin(th)-cos(th)     -1/m*sin(th)+cos(th);
          1/m*cos(th)-sin(th)      1/m*cos(th)+sin(th)];
    f = M\f; %Now in terms of f1 and f2
end