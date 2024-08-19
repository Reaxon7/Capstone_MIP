
function u = controller(params, t, phi, phidot)
  % STUDENT FILLS THIS OUT
  % 
  % Initialize any added state like this:
  % 
  persistent newstate t_last
  if isempty(newstate)
    % initialize
    newstate = 0;
    t_last = 0;
  end
  % 
  Kv = 2.5;
  Kp = 52;
  ki = 1700;

  deltat = t - t_last;
  newstate = newstate + phi*deltat;
  t_last = t;

  u = Kp*phi + Kv*phidot +ki*newstate;
  %u = -u;
end

