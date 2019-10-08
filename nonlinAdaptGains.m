function dk = nonlinAdaptGains(xp,xm,Refer)
    gamma_x = 1; gamma_r = 1; gamma_a = 1;
    phi = [cos(xp(5)); sin(xp(5))*xp(6);   sin(xp(5)); cos(xp(5))*xp(6)]*xp(6);
    
    % Reference Command Shaping of r_x
    if Refer(1) > xp(1)+10
        Refer(1) = xp(1)+10;
    elseif Refer(1) < xp(1)-10
        Refer(1) = xp(1)-10;
    end
    % Reference Command Shaping of r_y
    if Refer(3) > xp(3)+10
        Refer(3) = xp(3)+10;
    elseif Refer(3) < xp(3)-10
        Refer(3) = xp(3)-10;
    end
    
    B = [0    0;
         1    0;
         0    0;
         0    1;
         0    0;
        -1    0];
    
    P = [1.7321    1.0000         0         0         0         0;
         1.0000    1.7155         0         0         0         0;
              0         0    1.7321    1.0000         0         0;
              0         0    1.0000    1.7155         0         0;
              0         0         0         0    1.7321    1.0000;
              0         0         0         0    1.0000    1.7321];
    
    e = xp-xm;
    Mat = e'*P*B;
    dkx  = gamma_x*xp*Mat;
    dkr  = -gamma_r*Refer*Mat;
    dka =  gamma_a*phi*Mat;
    
    dk = [dkx(:,1);dkx(:,2);dkr(:,1);dkr(:,2);dka(:,1);dka(:,2)];
end