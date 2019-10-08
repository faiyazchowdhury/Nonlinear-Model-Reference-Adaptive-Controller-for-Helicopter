function dx = modelSim(t, x)
global type; global rad; global Ref;

    Refer = [Ref(1), 0, Ref(2), 0, 0, 0]';
    if (rad > 0) %Trajectory Generation
        traj_theta = mod(t/2/rad,2*pi);
        x_coord = rad*cos(traj_theta);
        y_coord = rad*sin(traj_theta);
        Refer(1) = Refer(1)+x_coord;
        Refer(3) = Refer(3)+y_coord;
    end
    
    if type == 1
        dx = linModel(x, Refer);
    elseif type == 2
        dx = nonlinModel(x, Refer);
    end
end