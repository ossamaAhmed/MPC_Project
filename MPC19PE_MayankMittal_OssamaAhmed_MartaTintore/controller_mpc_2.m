% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_2(T)

    % controller variables
    persistent param yalmip_optimizer

    % initialize controller, if not done already
    if isempty(param)
        [param, yalmip_optimizer] = init();
    end
    
    % Convert sensor measurement to system input
    T_input = T - param.T_sp;
    
    %% evaluate control action by solving MPC problem
    [output_mpc, errorcode] = yalmip_optimizer{T_input};
    if (errorcode ~= 0)
          warning('MPC infeasible');
    end
    
    % Convert system output to unit inputs
    p = output_mpc{1} + param.p_sp;
    
    % Obtain the MPC cost for the initial state
    if param.iter == 0
        fprintf('[MPC2] MPC Cost at t=0: %f\n', output_mpc{2});
    end
    param.iter = param.iter + 1;
end

%% Initialize parameters for the MPC controller
function [param, yalmip_optimizer] = init()
    % initializes the controller on first call and returns parameters and
    % Yalmip optimizer object

    % get basic controller parameters
    param = compute_controller_base_parameters; 
    param.iter = 0;

    %% implement your MPC using Yalmip here
    N = 30;
    nx = size(param.A, 1);
    nu = size(param.B, 2);

    U = sdpvar(repmat(nu, 1, N-1), ones(1, N-1), 'full');
    X = sdpvar(repmat(nx, 1, N), ones(1, N), 'full');

    objective = 0;
    constraints = [];
    
    % for timestep 1 to N - 1
    for k = 1:N-1
       % objective term: add stage cost
       objective = objective + X{:, k}' * param.Q * X{:, k} + U{:, k}' * param.R * U{:, k};
       % system input constraints
       constraints = [constraints, param.Ucons(:, 1) <= U{:, k} <= param.Ucons(:, 2)];
       if k > 1
            % system state constraints
            constraints = [constraints, param.Xcons(:, 1) <= X{:, k} <= param.Xcons(:, 2)];
            % system dynamics constraints
            constraints = [constraints, X{:, k} == param.A * X{:, k-1} + param.B * U{:, k-1}];
       end
    end
    % for timestep N
    % objective term: add terminal cost
    [P, ~, ~] = dare(param.A, param.B, param.Q, param.R);
    % terminal state constraint
    constraints = [constraints, X{:, N} == param.A * X{:, N-1} + param.B * U{:, N-1}];
    constraints = [constraints, X{:, N} == 0];

    % define input and output to optimzer
    init_state = X{1, 1};   % Input
    input_action = U{1, 1}; % Output
    
    % define the optimizer function
    ops = sdpsettings('verbose', 0, 'solver', 'quadprog');
    yalmip_optimizer = optimizer(constraints, objective,ops, init_state, {input_action, objective});
end