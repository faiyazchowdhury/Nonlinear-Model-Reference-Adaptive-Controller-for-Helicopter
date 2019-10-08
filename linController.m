function u = linController(x, Ref, Kx, Kr, Ka)
m = 6; gy = 9.8;
phi = [cos(x(5)); sin(x(5))*x(6); sin(x(5)); cos(x(5))*x(6)]*x(6);
if size(Ref,1) > 2
    Ref = [Ref(1);Ref(3)];
end
% Reference Command Shaping of r_x
if Ref(1) > x(1)+10
    Ref(1) = x(1)+10;
elseif Ref(1) < x(1)-10
    Ref(1) = x(1)-10;
end
% Reference Command Shaping of r_y
if Ref(2) > x(3)+10
    Ref(2) = x(3)+10;
elseif Ref(2) < x(3)-10
    Ref(2) = x(3)-10;
end

u = -Kx*x + Kr(:,[1,3])*Ref;
if nargin >4 
    u = u - Ka*phi;
end

u(u<=-m*gy/2) = -m*gy/2;
u(u>=5*m*gy/2) = 5*m*gy/2;
end