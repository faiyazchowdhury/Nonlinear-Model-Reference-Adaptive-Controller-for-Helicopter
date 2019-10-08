function dk = adaptGains(xp,xm,Refer)
    gamma_x = 1; gamma_r = 1;
    
    % Reference Command Shaping of r_x
    if Refer(1) > xp(1)+30
        Refer(1) = xp(1)+30;
    elseif Refer(1) < xp(1)-30
        Refer(1) = xp(1)-30;
    end
    % Reference Command Shaping of r_y
    if Refer(3) > xp(3)+30
        Refer(3) = xp(3)+30;
    elseif Refer(3) < xp(3)-30
        Refer(3) = xp(3)-30;
    end

    r = 0.25; J = 0.1425;
    B = [0    0;
         0    0;
         0    0;
         1    1;
         0    0;
        r/J -r/J];
    
    P = [2.4454    2.4684   -0.0000   -0.0000  -12.7148   -1.2746;
         2.4684    4.6874    0.0000    0.0000  -29.3466   -3.0644;
        -0.0000    0.0000    5.2804   13.4164   -0.0000   -0.0000;
        -0.0000    0.0000   13.4164   67.8442   -0.0000   -0.0000;
       -12.7148  -29.3466   -0.0000   -0.0000  277.5484   30.8325;
        -1.2746   -3.0644   -0.0000   -0.0000   30.8325   16.2057];
    
    e = xp-xm;
    Mat = e'*P*B;
    dkx  = gamma_x*xp*Mat;
    dkr  = -gamma_r*Refer*Mat;
    
    dk = [dkx(:,1);dkx(:,2);dkr(:,1);dkr(:,2)];
end