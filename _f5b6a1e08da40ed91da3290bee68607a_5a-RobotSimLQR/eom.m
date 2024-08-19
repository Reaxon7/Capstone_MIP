% function qdd = eom(params, th, phi, dth, dphi, u)
%   % This is the starter file for the week5 assignment
% 
%   % Provided params are
%   % params.g: gravitational constant
%   % params.mr: mass of the "rod"
%   % params.ir: rotational inertia of the rod
%   % params.d: distance of rod CoM from the wheel axis
%   % params.r: wheel radius
% 
%   % Provided states are:
%   % th: wheel angle (relative to body)
%   % phi: body pitch
%   % dth, dphi: time-derivatives of above
%   % u: torque applied at the wheel
% 
%   syms ddth ddphi
% 
%   x1 = (-u - 2*params.r*params.d*cos(phi)*ddphi + 2*params.r*params.d*sin(phi)*dphi^2)/(params.mr*params.r^2)-ddphi == ddth;
% 
%   x2 = params.mr/2*(-2*params.r*(dth+dphi)*params.d*sin(phi)*dphi) + params.mr*params.g*params.d*sin(phi) ==...
%       params.mr/2*(params.r^2*(2*ddphi+2*ddth) + 2*params.r*ddth*params.d*cos(phi) - 2*params.r*dth*params.d*sin(phi)*dphi +...
%       4*params.r*ddphi*params.d*cos(phi) - 4*params.r*params.d*dphi^2*sin(phi) + 2*params.d^2*ddphi) + params.ir*ddphi;
%   
% 
%   %[sol_ddth, sol_ddphi] = solve([x1, x2], [ddth, ddphi]);
%   
% 
%   qdd = solve([x1, x2], [ddth, ddphi]);
%   qdd = qdd';
%   % THE STUDENT WILL FILL THIS OUT
% end

% function qdd = eom(params, th, phi, dth, dphi, u)
%   % This is the starter file for the week5 assignment
% 
%   % Provided params are
%   % params.g: gravitational constant
%   % params.mr: mass of the "rod"
%   % params.ir: rotational inertia of the rod
%   % params.d: distance of rod CoM from the wheel axis
%   % params.r: wheel radius
% 
%   % Provided states are:
%   % th: wheel angle (relative to body)
%   % phi: body pitch
%   % dth, dphi: time-derivatives of above
%   % u: torque applied at the wheel
% 
%   syms ddth ddphi
% 
%   % Equation 1
%   %eq1 = (-u - 2*params.r*params.d*cos(phi)*ddphi + 2*params.r*params.d*sin(phi)*dphi^2)/(params.mr*params.r^2) - ddphi == ddth;
% 
%   eq1 = -u == params.mr/2*(params.r^2*(2*ddphi+2*ddth) + 2*params.r*params.d*cos(phi)*ddphi - 2*params.r*params.d*sin(phi)*dphi^2);
% 
%   % Equation 2
%   eq2 = params.mr/2 * (-2*params.r*(dth + dphi)*params.d*sin(phi)*dphi) + params.mr*params.g*params.d*sin(phi) == ...
%         params.mr/2 * (params.r^2*(2*ddphi + 2*ddth) + 2*params.r*ddth*params.d*cos(phi) - 2*params.r*dth*params.d*sin(phi)*dphi + ...
%         4*params.r*ddphi*params.d*cos(phi) - 4*params.r*params.d*dphi^2*sin(phi) + 2*params.d^2*ddphi) + params.ir*ddphi;
% 
%   % Solve the system of equations for ddth and ddphi
%   sol = solve([eq1, eq2], [ddth, ddphi]);
% 
%   % Convert the solution to a vector form [ddth; ddphi]
%   qdd = [sol.ddth; sol.ddphi];
%   %qdd = [0;0];
% 
% end

function qdd = eom(params, th, phi, dth, dphi, u)
    % Provided params are:
    % params.g: gravitational constant
    % params.mr: mass of the "rod"
    % params.ir: rotational inertia of the rod
    % params.d: distance of rod CoM from the wheel axis
    % params.r: wheel radius

    % Provided states are:
    % th: wheel angle (relative to body)
    % phi: body pitch
    % dth, dphi: time-derivatives of the above
    % u: torque applied at the wheel

    % Coefficients in the equations of motion:
    A11 = params.mr * params.r^2;
    A12 = params.mr * params.r * params.d * cos(phi);
    A21 = A12;
    A22 = params.mr * params.d^2 + params.ir + params.mr * params.r^2;
    
    % Right-hand side:
    b1 = u + params.mr * params.r * params.d * sin(phi) * dphi^2;
    b2 = params.mr * params.d * params.g * sin(phi) - params.mr * params.r * params.d * sin(phi) * dth * dphi;
    
    % Coefficient matrix:
    A = [2*A11, 2*A12; 2*A21, 2*A22];

    % Solving for qdd:
    qdd = A \ [b1; b2];

end