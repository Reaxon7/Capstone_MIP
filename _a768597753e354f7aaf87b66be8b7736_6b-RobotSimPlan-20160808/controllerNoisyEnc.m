
function u = controllerNoisyEnc(params, t, obs, th, dth)
  % This is the starter file for the week5 assignment
  % Now you only receive noisy measurements for theta, and must use your EKF from week 3 to filter the data and get an estimate of the state
  % obs = [ay; az; gx] (same as last week)
  % New for 6b: you also have access to params.traj(t)

  % Template code (same as last week)
  xhat = EKFupdate(params, t, obs);
  phi = xhat(1);
  phidot = xhat(2);

  % fill this out
  persistent newstate t_last
  if isempty(newstate)
    % initialize
    newstate = 0;
    t_last = 0;
  end
  % 
  %keyboard
  phides = params.traj(t) / params.r - th;
  Kv = 2.5;
  Kp = 52;
  ki = 1700;

  deltat = t - t_last;
  newstate = newstate + phi*deltat;
  t_last = t;

  u = Kp*sin(phi-phides) + Kv*phidot +ki*newstate;
  %u=0;
end

function xhatOut = EKFupdate(params, t, z)
% z = [ay; az; gx] with a* in units of g's, and gx in units of rad/s
% You can borrow most of your week 3 solution, but this must only implement a single predict-update step of the EKF
% Recall (from assignment 5b) that you can use persistent variables to create/update any additional state that you need.

%xhat = zeros(2,length(t));
%xk = zeros(2,length(t));

persistent xhat t_last P h xkp_o
if isempty(t_last)
    % initialize
    xhat = [atan2(z(1),z(2));z(3)];
    t_last = 0;
    P = [0.005 0;0 0.005];
    h = [sin(xhat(1)),cos(xhat(1)),xhat(2)]';
    xkp_o = xhat(:, 1);
end


Q = [0.01 0;0 0.01];
R = [1e-3 0 0;0 1e-3 0; 0 0 1e-3];
%   Q = [0.1 0; 0 0.1];
%   R = [0.009 0 0; 0 0.009 0; 0 0 0.0076];
g = params.g;


%xhat(:,1) = [2; 2];
%xk(:,1) = [atan2(z(1,1),z(2,1));z(3,1)];


%for i = 2:length(t)
dt = t - t_last;
A = [1 dt; 0 1];

% Prediction step
xkp = A * xhat;
pkp = A * P * A' + Q;

% Jacobian of state Z d/dphi, d/dphi_dot at t = tk
H = [cos(xhat(1)) 0; -sin(xhat(1)) 0; 0 1];

% Optimal Kalman Gain
K = pkp * H' / (H * pkp * H' + R);

% Update step
%h = [g * sind(xkp(1)); g * cosd(xkp(1)); xkp(2)];
xhat = xkp + K * (z - h);
P = (eye(2) - K * H) * pkp;

% Linearization of h for the next step
h = h + H * (xkp - xkp_o);
xkp_o = xkp;

t_last = t;
%end

% Student completes this
xhatOut = xhat;
end

