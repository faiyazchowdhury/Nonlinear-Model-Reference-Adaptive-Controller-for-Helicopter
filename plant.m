function dx = plant(x, f)
    global m; global d; global gy; global r; global J;
    f(f<0) = 0;
    f(f>3*m*gy) = 3*m*gy;
    x2 = x(2); y2 = x(4); T1 = x(5); T2 = x(6);
    x1dot = x2;  y1dot = y2;  T1dot = T2;
    x2dot = (-d/m)*x2 - sin(T1)/m*[1 1]*f;
    y2dot = (-d/m)*y2 + cos(T1)/m*[1 1]*f - gy;
    T2dot = (r/J)*[1 -1]*f;
    dx= [x1dot; x2dot; y1dot; y2dot; T1dot; T2dot];
end