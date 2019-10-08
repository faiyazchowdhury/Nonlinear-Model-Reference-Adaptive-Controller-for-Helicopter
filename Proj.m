function result = Proj(theta,v, theta_max, epsilon_theta)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
Psi = (theta'*theta - theta_max^2)/(epsilon_theta * theta_max^2);
Delta_phi = 2*theta/(epsilon_theta * theta_max^2);
result = v;
if (Psi >= 0) && (Delta_phi'*v > 0)
    Psi_N = Delta_phi / norm(Delta_phi);
    result = v - Psi*(Psi_N'*v)*Psi_N;
end

end