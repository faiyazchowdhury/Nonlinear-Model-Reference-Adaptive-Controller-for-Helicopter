function results = plotter(run, tspan, ref_com, radius, adapt_com, var, learnt)
global m; global d; global gy; global r; global J;
m = 6; d = 0.1; gy = 9.8; J = 0.1425; r = 0.25;

global type;
global Ref;
global rad;
global adapt;
Ref = ref_com;
rad = radius;
adapt = false;

%% -- Linear Plant Controller -----
if (run == 1)
    type = 1;
    x0 = [0, 0, 0, 0, 0, 0];    
    [t,results] = ode23(@modelSim,tspan,x0);
    target = (Ref*ones(1,length(t)))';
    if (rad == 0)
        titletxt = 'Linear Controller on Linear Plant, Point-to-Point Movement';
    else
        target = target + rad*[cos(t/2/rad),sin(t/2/rad)];
        titletxt = 'Linear Controller on Linear Plant, Circular Trajectory';
    end
    
    figure;
    subplot(4,1,1);  plot(t,results(:,1:2),t,target(:,1));  hold off;
    legend('x', 'xdot','x target');  title(titletxt);
    subplot(4,1,2);  plot(t,results(:,3:4),t,target(:,2));  hold off;
    legend('y', 'ydot','y-target');  title(titletxt);
    subplot(4,1,3);  plot(t,results(:,5:6));  
    legend('\theta', '\thetadot'); title(titletxt);
    subplot(4,1,4);  plot(results(:,1),results(:,3),target(:,1),target(:,2));  
    legend('Plant', 'Target'); xlabel('x'); ylabel('y'); title(['Trajectory, ' titletxt]);
end

%% -- Linear Controller Model -----
if (run == 2)
    type = 1;
    x0 = zeros(6,1);
    [t,results] = ode23(@controllerSim,tspan,x0);
    target = (Ref*ones(1,length(t)))';
    if (rad == 0)
        titletxt = 'Linear Controller on Nonlinear Plant, Point-to-Point Movement';
    else
        target = target + rad*[cos(t/2/rad),sin(t/2/rad)];
        titletxt = 'Linear Controller on Nonlinear Plant, Circular Trajectory';
    end
    figure; 
    subplot(4,1,1); plot(t,results(:,1:2),t,target(:,1));  legend('x_p', 'xdot_p','x target'); title(titletxt); hold on;
    subplot(4,1,2);  plot(t,results(:,3:4),t,target(:,2));  legend('y_p', 'ydot_p','y target'); title(titletxt)
    subplot(4,1,3);  plot(t,results(:,5:6));  legend('\theta_p', '\thetadot_p'); title(titletxt)
    subplot(4,1,4);  plot(results(:,1),results(:,3),target(:,1),target(:,2));  
    legend('Plant', 'Target'); xlabel('x'); ylabel('y'); title(['Trajectory, ' titletxt]);
end

%% -- Linear Adaptive Model -----
if (run == 3)
    if adapt_com
        adapt = true;
    end
    type = 1;
    
    if var
        v = 0.4; m=m*(v*rand(1)+0.8); d=d*(v*rand(1)+0.8);  r=r*(v*rand(1)+0.8);  J=J*(v*rand(1)+0.8);
    end
    
    K = [-0.7071   -1.6450    0.7071    2.1283   15.7745    7.6806;
          0.7071    1.6450    0.7071    2.1283  -15.7745   -7.6806];
    
    x0 = zeros(36,1);
    x0(13:18) = K(1,:); x0(19:24) = K(2,:);
    x0(25:30) = K(1,:); x0(31:36) = K(2,:);
    
    [t,results] = ode23(@adaptSim,tspan,x0);
    if learnt
        x0 = [zeros(1,12) results(end,13:36)];
        [t,results] = ode23(@adaptSim,tspan,x0);
    end
    
    target = (Ref*ones(1,length(t)))';
    if (rad == 0)
        titletxt = 'Adaptive Controlled Non-linear Plant, Point-to-Point Movement';
    else
        target = target + rad*[cos(t/2/rad),sin(t/2/rad)];
        titletxt = 'Adaptive Controlled Non-linear Plant, Circular Trajectory';
    end
    figure; 
    subplot(4,1,1); plot(t,results(:,1:2),t,results(:,7:8),t,target(:,1));  legend('x_p','xdot_p','x_m','xdot_m','x target'); title(titletxt); hold on;
    subplot(4,1,2);  plot(t,results(:,3:4),t,results(:,9:10),t,target(:,2));  legend('y_p','ydot_p','y_m','ydot_m','y target'); title(titletxt)
    subplot(4,1,3);  plot(t,results(:,5:6),t,results(:,11:12));  legend('\theta_p','\thetadot_p','\theta_m','\thetadot_m'); title(titletxt)
    subplot(4,1,4);  plot(results(:,1),results(:,3),results(:,7),results(:,9),target(:,1),target(:,2)); legend('Plant','Model','Target'); xlabel('x'); ylabel('y'); title(['Trajectory, ' titletxt]);
    figure,
    plot(t,results(:,13:36)); title('Gains');
end


%% -- Non-Linear Plant Controller -----
if (run == 11)
    type = 2;
    x0 = [0, 0, 0, 0, 0, 0];    
    [t,results] = ode23(@modelSim,tspan,x0);
    target = (Ref*ones(1,length(t)))';
    if (rad == 0)
        titletxt = 'Differential Flatness Linear Controller on Linear Plant, Point-to-Point Movement';
    else
        target = target + rad*[cos(t/2/rad),sin(t/2/rad)];
        titletxt = 'Differential Flatness Linear Controller on Linear Plant, Circular Trajectory';
    end
    
    figure;
    subplot(4,1,1);  plot(t,results(:,1:2),t,target(:,1));  hold off;
    legend('x', 'xdot','x target');  title(titletxt);
    subplot(4,1,2);  plot(t,results(:,3:4),t,target(:,2));  hold off;
    legend('y', 'ydot','y-target');  title(titletxt);
    subplot(4,1,3);  plot(t,results(:,5:6));  
    legend('\theta', '\thetadot'); title(titletxt);
    subplot(4,1,4);  plot(results(:,1),results(:,3),target(:,1),target(:,2));  
    legend('Plant', 'Target'); xlabel('x'); ylabel('y'); title(['Trajectory, ' titletxt]);
end

%% -- Non-Linear Controller Model -----
if (run == 12)
    type = 2;
    x0 = zeros(6,1);
    [t,results] = ode23(@controllerSim,tspan,x0);
    target = (Ref*ones(1,length(t)))';
    if (rad == 0)
        titletxt = 'Differential Flatness, Linear Controller on Nonlinear Plant, Point-to-Point Movement';
    else
        target = target + rad*[cos(t/2/rad),sin(t/2/rad)];
        titletxt = 'Differential Flatness, Linear Controller on Nonlinear Plant, Circular Trajectory';
    end
    figure; 
    subplot(4,1,1); plot(t,results(:,1:2),t,target(:,1));  legend('x_p', 'xdot_p','x target'); title(titletxt); hold on;
    subplot(4,1,2);  plot(t,results(:,3:4),t,target(:,2));  legend('y_p', 'ydot_p','y target'); title(titletxt)
    subplot(4,1,3);  plot(t,results(:,5:6));  legend('\theta_p', '\thetadot_p'); title(titletxt)
    subplot(4,1,4);  plot(results(:,1),results(:,3),target(:,1),target(:,2));  
    legend('Plant', 'Target'); xlabel('x'); ylabel('y'); title(['Trajectory, ' titletxt]);
end

%% -- Non-Linear Adaptive Model -----
if (run == 13)
    if adapt_com
        adapt = true;
    end
    type = 2;
    
    if var
        v = 0.4; m=m*(v*rand(1)+0.8); d=d*(v*rand(1)+0.8);  r=r*(v*rand(1)+0.8);  J=J*(v*rand(1)+0.8);
    end
    
    K = [1.0000    1.7155         0         0    -1.0000e0   -1.7321e0 ;
              0         0    1.0000    1.7155         0         0];
    alpha = 0.57*[-.1/6   1     0     0;
                     0    0  -.1/6  -1];
    x0 = zeros(44,1);
    x0(13:18) = K(1,:); x0(19:24) = K(2,:);
    x0(25:30) = K(1,:); x0(31:36) = K(2,:);
    x0(37:40) = alpha(1,:); x0(41:44) = alpha(2,:); %% Need to change

    [t,results] = ode23(@adaptSim,tspan,x0);
    if learnt
        x0 = [zeros(1,12) results(end,13:44)];
        [t,results] = ode23(@adaptSim,tspan,x0);
    end
    
    target = (Ref*ones(1,length(t)))';
    if (rad == 0)
        titletxt = 'Differential Flatness Adaptive Controlled Non-linear Plant, Point-to-Point Movement';
    else
        target = target + rad*[cos(t/2/rad),sin(t/2/rad)];
        titletxt = 'Differential Flatness, Adaptive Controlled Non-linear Plant, Circular Trajectory';
    end
    figure; 
    subplot(4,1,1); plot(t,results(:,1:2),t,results(:,7:8),t,target(:,1));  legend('x_p','xdot_p','x_m','xdot_m','x target'); title(titletxt); hold on;
    subplot(4,1,2);  plot(t,results(:,3:4),t,results(:,9:10),t,target(:,2));  legend('y_p','ydot_p','y_m','ydot_m','y target'); title(titletxt)
    subplot(4,1,3);  plot(t,results(:,5:6),t,results(:,11:12));  legend('\theta_p','\thetadot_p','\theta_m','\thetadot_m'); title(titletxt)
    subplot(4,1,4);  plot(results(:,1),results(:,3),results(:,7),results(:,9),target(:,1),target(:,2)); legend('Plant','Model','Target'); xlabel('x'); ylabel('y'); title(['Trajectory, ' titletxt]);
    figure,
    plot(t,results(:,13:36)); title('Gains');
end


end