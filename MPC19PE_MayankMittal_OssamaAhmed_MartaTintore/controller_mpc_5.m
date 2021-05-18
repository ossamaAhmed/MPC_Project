% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_5(T)
    % controller variables
    persistent param yalmip_optimizer current_disturbance x_hat_k

    % initialize controller, if not done already
    if isempty(param)
        [param, current_disturbance, yalmip_optimizer] = init();
        x_hat_k = T;
    end

    [T_sp, p_sp] = calculate_steady_state(param, param.B_d_disturbance, current_disturbance);
    
    %% evaluate control action by solving MPC problem, e.g.
    augmented_current_state = [T; current_disturbance];
    [u_mpc, errorcode] = yalmip_optimizer([augmented_current_state; T_sp; p_sp]);
    if (errorcode ~= 0)
          warning('MPC infeasible');
    end
    p = u_mpc{1};

    if param.iteration == 0
            fprintf('Finite Horizon Cost as calculated at t %d : %f\n', param.iteration, u_mpc{2});
            fprintf('Terminal Cost as calculated at t %d : %f\n', param.iteration, u_mpc{3});
    end

    %% Get next disturbance estimate
    next_estimated_aug_state = param.A_aug * [x_hat_k; current_disturbance] + param.B_aug * p + param.L * (T - param.C_aug * [x_hat_k; current_disturbance]);
    x_hat_k = next_estimated_aug_state(1:3);
    current_disturbance = next_estimated_aug_state(4:6);
    param.iteration = param.iteration + 1;
    
end

function [T_sp, p_sp] = calculate_steady_state(param, B_d_disturbance, d_d)
    r = [-20, 0.25]';
    H = [[1, 0, 0];
         [0, 1, 0]];
    target_condition_matrix = [eye(3) - param.A, -param.B;
                               H, zeros(2, 2)];

    steady_state = pinv(target_condition_matrix) * [B_d_disturbance * d_d; r];
    T_sp = steady_state(1:3);
    p_sp = steady_state(4:5);
end

%% Initialize parameters for the MPC controller
function [param, current_disturbance, yalmip_optimizer] = init()
    % initializes the controller on first call and returns parameters and
    % Yalmip optimizer object

    param = compute_controller_base_parameters; % get basic controller parameters
    param.iteration = 0;
    A_aug = [param.A, param.B_d_disturbance; zeros(3), eye(3)]; 
    B_aug = [param.B; zeros(3, 2)];
    C_aug = [param.C, zeros(3)];
    D_aug = [zeros(3, 2), zeros(3, 2)];
    
    % test
    if rank([A_aug - eye(6); C_aug]) ~= 6
        warning("Rank must be nx+nd == 6")
    end
    param.L = (place(A_aug',C_aug',[0,0,0,0.05,0.05,0.05]))';
    
    % state the error dynamics
    param.error_dynamics = A_aug - param.L*C_aug;
    eigenvalues = eig(param.error_dynamics);
    
    % (4) constraints for delta formulation
    param.A_aug = A_aug;
    param.B_aug = B_aug;
    param.C_aug = C_aug;
    
    %% implement your MPC using Yalmip here, e.g.
    N = 30;
    nx = size(param.A_aug,1);
    nu = size(param.B_aug,2);
    param.iteration = 0;
    
    % oprimization variables
    U = sdpvar(repmat(nu,1,N-1),repmat(1,1,N-1),'full');
    X = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');
    T_sp = sdpvar(nx/2,1);
    p_sp = sdpvar(nu,1);

    objective = 0;
    constraints = [];
    
    for k = 1:N-1
       % objective term: add stage cost
       objective = objective + (X{:, k}(1:3, :) - T_sp)' * param.Q * (X{:, k}(1:3, :) - T_sp) + (U{:, k} - p_sp)' * param.R * (U{:, k} - p_sp);
       % put constraints on future steps but not the initial step though
       constraints = [constraints, param.Pcons(:, 1) <= U{:, k} <= param.Pcons(:, 2)];
       if k > 1
           constraints = [constraints, param.Tcons(:, 1) <= X{:, k}(1:3, :) <= param.Tcons(:, 2)];
           constraints = [constraints, X{:, k} == param.A_aug * X{:, k-1} + param.B_aug * U{:, k-1}];
       end 
    end
    % calculating terminal cost 
    constraints = [constraints, X{:, N} == param.A_aug * X{:, N-1} + param.B_aug * U{:, N-1}];
    terminal_cost =  (X{:, N}(1:3, :) - T_sp)' * (param.Q * param.F'* param.R * param.F) * (X{:, N}(1:3, :) - T_sp);
    % terminal_cost = 0;
    objective = objective + terminal_cost;
    [A_x, b_x] = compute_X_LQR;
    constraints = [constraints, A_x * (X{:, N}(1:3, :) - T_sp)  <= b_x];

    % define input and output to optimzer
    init_state = X{1, 1};   % Input
    input_action = U{1, 1}; % Output
    
    % define the optimizer function
    ops = sdpsettings('verbose', 0, 'solver', 'quadprog');
    yalmip_optimizer = optimizer(constraints, objective, ops, [init_state; T_sp; p_sp], {input_action, objective, terminal_cost});
    current_disturbance = param.d_d;
end