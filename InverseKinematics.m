function err_norm = InverseKinematics(to, Target)
global uLINK

lambda = 0.9;
idx = FindRoute(to);                  % Find out a route for the body to the target link
ForwardKinematics(1);
err = CalcVWerr(Target, uLINK(to));   % Calculate the error in the position and attitude of the link
for n = 1:10
  if norm(err) < 1E-6 break, end;
  J  = CalcJacobian(idx);             % Funtion to Calculate the Jacobian
  dq = lambda * (J \ err);            % Solves the Linear Equation (Eq. 2.71) without doing the explicit matrix inversion
  MoveJoints(idx, dq);
  ForwardKinematics(1);
  err = CalcVWerr(Target, uLINK(to));
end

if nargout == 1 
    err_norm = norm(err);
end
