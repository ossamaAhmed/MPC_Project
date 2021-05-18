% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_lqr(T)
% controller variables
persistent param;

% initialize controller, if not done already
if isempty(param)
    param = init();
end

% compute control action
T_input = T - param.T_sp;
p = param.F * T_input + param.p_sp;

% compute LQR cost
if param.iter == 0
    J = T_input' * param.P * T_input;
    fprintf('[LQR] Infinite Horizon Cost at t=0: %f\n', J);
end

% check response rate
if param.iter == 30
    if norm(T_input) > 0.2 * norm(param.x0_1)
        warning('[LQR] Closed loop response of the system is slow.')
    end
end

param.iter = param.iter + 1;

end

function param = init()
param = compute_controller_base_parameters;

% add additional parameters if necessary
[P, ~, K] = dare(param.A, param.B, param.Q, param.R);
param.F = - K;
param.P = P;
param.iter = 0;
param.x0_1 = [3; 1; 0];

end