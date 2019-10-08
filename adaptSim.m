function [dx] = adaptSim(t, x)
global type; global rad; global Ref; global adapt;
    xp = x(1:6); xm = x(7:12);
    k1x = x(13:18)'; k2x = x(19:24)';
    k1r = x(25:30)'; k2r = x(31:36)';
    Kx = [k1x; k2x];
    Kr = [k1r; k2r];
    if type == 2
        k1a = x(37:40)'; k2a = x(41:44)';
        Ka = [k1a; k2a];
    end
    
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
        u = linController(xp, Refer([1,3]), Kx, Kr);
    elseif type == 2
        J = 0.1425; r = 0.25; lambda = J/r;
        x_prime = xp + lambda*[-sin(xp(5)); -cos(xp(5))*xp(6); cos(xp(5)); -sin(xp(5))*xp(6); 0; 0];
        Refer_prime = Refer;
        Refer_prime(3) = Refer(3)+lambda;
        u = linController(x_prime, [Refer_prime(1);Refer_prime(3)], Kx, Kr, Ka);
    end
    
    % Converting u notation to thrusters
    if type == 1
        f = linUtoThrust(u);
    elseif type == 2
        f = nonlinUtoThrust(u, xp(5));
    end
    
    % Plant
    dxp = plant(xp, f);

    % Nonlinear Model
    if type == 1
        dxm = linModel(xm, Refer([1,3]));
    elseif type == 2
        dxm = nonlinModel(xm, Refer_prime([1,3]));
    end
    
    % Error Feedback (Adaptation Law)
    if type == 1
        dk = zeros(24,1); % Nulling Adaptation
        if adapt
            dk = adaptGains(xp,xm,Refer);
        end
    elseif type == 2
        dk = zeros(32,1); % Nulling Adaptation
        if adapt
            dk = nonlinAdaptGains(x_prime,xm,Refer_prime); %Do we shift
        end
    end

    dx = [dxp;dxm;dk];
end