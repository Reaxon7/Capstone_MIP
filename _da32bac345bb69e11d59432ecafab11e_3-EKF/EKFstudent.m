
function xhat = EKFstudent(t, z)
  % In this exercise, you will batch-process this data: you are provided a vector of timestamps (of length T), and a 3xT matrix of observations, z.
  xhat = zeros(2,length(t));
  xk = zeros(2,length(t));

  P = [0.005 0;0 0.005];
  Q = [0.01 0;0 0.01];
  R = [1e-3 0 0;0 1e-3 0; 0 0 1e-3];
%   Q = [0.1 0; 0 0.1];
%   R = [0.009 0 0; 0 0.009 0; 0 0 0.0076];
  g = 9.81;
  

  xhat(:,1) = [2; 2];
  %xk(:,1) = [atan2(z(1,1),z(2,1));z(3,1)];
  h = [sind(xhat(1,1)),cosd(xhat(1,1)),xhat(1,2)]';
  xkp_o = xhat(:, 1);
%   for i = 2:length(t)
%       dt = t(i) - t(i-1); % not complete
%       A = [1 dt; 0 1];
%       xk(:,i) = [atan2(z(1,i),z(2,i));z(3,i)];
%       H = [cosd(xhat(1,i))*pi/180,-sind(xhat(1,i))*pi/180,0;0,0,1]';
%       
%       %keyboard
% 
% 
%       xhat(:,i) = A*xhat(:,i-1);
%       P = A*P*A'+ Q;
% 
%       h = h + H*(xk(:,i)-xk(:,i-1));
% 
%       K = P*H'/(H*P*H' + R);
% 
%       xhat(:,i) = xhat(:,i) + K*(z(:,i) - h);
%       P = (eye(2)-K*H)*P;
%   end

    for i = 2:length(t)
        dt = t(i) - t(i - 1);
        A = [1 dt; 0 1];
    
        % Prediction step
        xkp = A * xhat(:, i - 1);
        pkp = A * P * A' + Q;
    
        % Jacobian of state Z d/dphi, d/dphi_dot at t = tk
        H = [cosd(xhat(1, i))*pi/180 0; -sind(xhat(1, i))*pi/180 0; 0 1];
    
        % Optimal Kalman Gain
        K = pkp * H' / (H * pkp * H' + R);
    
        % Update step
        %h = [g * sind(xkp(1)); g * cosd(xkp(1)); xkp(2)];
        xhat(:, i) = xkp + K * (z(:, i) - h);
        P = (eye(2) - K * H) * pkp;
    
        % Linearization of h for the next step
        h = h + H * (xkp - xkp_o);
        xkp_o = xkp;
    end
  % Student completes this
end


% function xhat = EKFstudent(t, z)
%     % In this exercise, you will batch-process this data: you are provided a vector of timestamps (of length T), and a 3xT matrix of observations, z.
%     xhat = zeros(2, length(t));
% 
%     % Parameters for the Kalman filter
%     P = zeros(2, 2, length(t));
%     Q = [100000 0; 0 0.0001];
%     R = [0.0009 0 0; 0 0.0009 0; 0 0 0.0076];
%     g = 9.81;
% 
%     % Initial States
%     xhat(:, 1) = [2; 2];  % Initial state estimate
%     P(:, :, 1) = [0.5 0.5; 0.5 0.5] * 10;
%     
%     h = [g * sind(xhat(1, 1)); g * cosd(xhat(1, 1)); xhat(2, 1)];
%     xkp_o = xhat(:, 1);
%     
%     % Extended Kalman Filter loop
%     for i = 2:length(t)
%         dt = t(i) - t(i - 1);
%         A = [1 dt; 0 1];
%         
%         % Prediction step
%         xkp = A * xhat(:, i - 1);
%         pkp = A * P(:, :, i - 1) * A' + Q;
%         
%         % Jacobian of state Z d/dphi, d/dphi_dot at t = tk
%         H = [g * cosd(xhat(1, i)) 0; -g * sind(xhat(1, i)) 0; 0 1];
%         
%         % Optimal Kalman Gain
%         K = pkp * H' / (H * pkp * H' + R);
%         
%         % Update step
%         h = [g * sind(xkp(1)); g * cosd(xkp(1)); xkp(2)];
%         xhat(:, i) = xkp + K * (z(:, i) - h);
%         P(:, :, i) = (eye(2) - K * H) * pkp;
%         
%         % Linearization of h for the next step
%         h = h + H * (xkp - xkp_o);
%         xkp_o = xkp;
%     end
% end