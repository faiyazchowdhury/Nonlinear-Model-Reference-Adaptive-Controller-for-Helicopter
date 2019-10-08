function dx = controllerSim(t, x)
global type; global rad; global Ref;
    % Trajectory Generation
    Refer = [Ref(1), 0, Ref(2), 0, 0, 0]';
    if (rad > 0) %Trajectory Generation
        traj_theta = mod(t/2/rad,2*pi);
        x_coord = rad*cos(traj_theta);
        y_coord = rad*sin(traj_theta);
        Refer(1) = Refer(1)+x_coord;
        Refer(3) = Refer(3)+y_coord;
    end

    % Controller (Control Law)
    if type == 1
        K = [-0.7071   -1.6450    0.7071    1.0904   15.7745    7.6806;
              0.7071    1.6450    0.7071    1.0904  -15.7745   -7.6806];
        u = linController(x, Refer([1,3]), K, K);
    elseif type == 2
        m = 6; d = 0.1; J = 0.1425; r = 0.25; lambda = J/r;
        x_prime = x + lambda*[-sin(x(5)); -cos(x(5))*x(6); cos(x(5)); -sin(x(5))*x(6); 0; 0];
        K= [1.0000    1.7155         0         0    -1.0000e0   -1.7321e0 ;
                 0         0    1.0000    1.7155         0         0];
        alpha = lambda*[-d/m   1     0    0;
                          0    0   -d/m  -1];
        u = linController(x_prime, [Refer(1);Refer(3)+lambda], K, K, alpha);
    end
    
    % Converting u notation to thrusters
    if type == 1
        f = linUtoThrust(u);
    elseif type == 2
        f = nonlinUtoThrust(u, x(5));
    end
    
    % Plant
    dx = plant(x, f);
end