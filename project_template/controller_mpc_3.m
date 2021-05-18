% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_3(T)
    % controller variables
    persistent param yalmip_optimizer

    % initialize controller, if not done already
    if isempty(param)
        [param, yalmip_optimizer] = init();
    end

    T_err = T - param.T_sp; %calculate error

    %% evaluate control action by solving MPC problem, e.g.
    [u_mpc, errorcode] = yalmip_optimizer{T_err};
    if (errorcode ~= 0)
          warning('MPC infeasible');
    end
    p = u_mpc{1} + param.p_sp;

    if param.iteration == 0
        fprintf('Finite Horizon Cost as calculated at t %d : %f\n', param.iteration, u_mpc{2});
        fprintf('Terminal Cost as calculated at t %d : %f\n', param.iteration, u_mpc{3});
    end
    
    param.iteration = param.iteration + 1;
end

function [param, yalmip_optimizer] = init()
    % initializes the controller on first call and returns parameters and
    % Yalmip optimizer object
    param = compute_controller_base_parameters; % get basic controller parameters

    %% implement your MPC using Yalmip here, e.g.
    N = 30;
    nx = size(param.A,1);
    nu = size(param.B,2);
    param.iteration = 0;
    
    % oprimization variables
    U = sdpvar(repmat(nu, 1, N-1), ones(1, N-1), 'full');
    X = sdpvar(repmat(nx, 1, N), ones(1, N), 'full');
    
    % constraints 
    G_x = [eye(3); -eye(3)];
    h_x = [param.Xcons(:, 2); - param.Xcons(:, 1)]; 
    G_u = [eye(2); -eye(2)];
    h_u = [param.Ucons(:, 2); -param.Ucons(:, 1)];
    
    objective = 0;
    constraints = [];
    
    % for timestep 1 to N - 1
    for k = 1:N-1
       % objective term: add stage cost
       objective = objective + X{:, k}' * param.Q * X{:, k} + U{:, k}' * param.R * U{:, k};
       % system input constraints
       constraints = [constraints, G_u * U{:, k} <= h_u];
       if k > 1
           % system state constraints
           constraints = [constraints, G_x * X{:, k} <= h_x];
           % system dynamics constraints
           constraints = [constraints, X{:, k} == param.A * X{:, k-1} + param.B * U{:, k-1}];
       end
    end
    % for timestep N
    % objective term: add terminal cost
    [~, ~, K] = dare(param.A, param.B, param.Q, param.R);
    param.F = -K;
    terminal_cost =  X{:, N}' * (param.Q * param.F'* param.R * param.F) * X{:, N};
    objective = objective + terminal_cost;
    % terminal state constraint
    constraints = [constraints, X{:, N} == param.A * X{:, N-1} + param.B * U{:, N-1}];
    [A_x, b_x] = compute_X_LQR;
    constraints = [constraints, A_x * X{:, N} <= b_x];
    
    % define input and output to optimzer
    init_state = X{1, 1};   % Input
    input_action = U{1, 1}; % Output

    % define the optimizer function
    ops = sdpsettings('verbose', 0, 'solver', 'quadprog');
    yalmip_optimizer = optimizer(constraints, objective,ops, init_state, {input_action, objective, terminal_cost});
end