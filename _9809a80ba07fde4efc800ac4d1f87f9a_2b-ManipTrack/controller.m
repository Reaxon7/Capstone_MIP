
function u = controller(params, t, X)
  %u=[0; 0];
  % 1. write out the forward kinematics, such that p = FK(theta1, theta2)
  % 2. Let e = p - params.traj(t) be the task-space error
  % 3. Calculate the manipulator Jacobian J = d p / d theta
  % 4. Use a "natural motion" PD controller, u = - kp * J^T * e - kd * [dth1; dth2]
  
  % Define the length of the links
  l = params.l; % Assuming both links have the same length stored in params.l

  % Joint angles
  theta1 = X(1);
  theta2 = X(2);

  p = params.l*[cos(X(1)),sin(X(1))] + params.l*[cos(X(1)+X(2)),sin(X(1)+X(2))];

  % Compute the elements of the Jacobian matrix J
  J11 = -l * sin(theta1) - l * sin(theta1 + theta2);
  J12 = -l * sin(theta1 + theta2);
  J21 = l * cos(theta1) + l * cos(theta1 + theta2);
  J22 = l * cos(theta1 + theta2);

  % Form the Jacobian matrix J
  J = [J11, J12;
      J21, J22];

  kp = 1000;
  kd = 2;

  e = p' - params.traj(t);
%   if t > 1
%     keyboard
%   end
  u = - kp * J' * e - kd * [X(3); X(4)] + [0;params.m*params.g];
  
end

