
params = struct();

params.g = 9.81;
params.mr = 0.25;
params.ir = 0.0001;
params.d = 0.1;
params.r = 0.02;

% 1. Learn how to use the symbolic toolbox in MATLAB
% you will need the state variables, and control input to be declared as symbolic
syms th phi dth dphi u;

% 2. call your "eom" function to get \ddot{q} symbolically
qdd = eom(params, th, phi, dth, dphi, u);

x = [th; phi; dth; dphi];
f = [dth; dphi; qdd(1); qdd(2)];

% 3. Linearize the system at 0 (as shown in lecture)
% You should end up with A (4x4), and b (4x1)
% A = subs(A, {th,phi,dth,dphi}, {0,0,0,0});
% B = subs(B,{phi,u},{0,0});
A_sym = jacobian(f, x);
B_sym = jacobian(f, u);

% Substitute the equilibrium point (all variables = 0) into A and B
A_sym = subs(A_sym, [th, phi, dth, dphi, u], [0, 0, 0, 0, 0]);
B_sym = subs(B_sym, [th, phi, dth, dphi, u], [0, 0, 0, 0, 0]);

% Convert symbolic matrices to numeric form
A = double(A_sym);
B = double(B_sym);

disp('Jacobian A (with respect to [th, phi, dth, dphi]):');
disp(A);
disp('Jacobian B (with respect to u):');
disp(B);

% 4. Check that (A,b) is  controllable
% Number of uncontrollable states should return 0
Co = ctrb(A,B);
unco = length(A)-rank(Co);
disp(['Number of uncontrollable states: ', num2str(unco)]);

% 5. Use LQR to get K as shown in the lecture
K = lqr(A,B,0.008*eye(4),100,0);
disp(['K: ', num2str(K)]);