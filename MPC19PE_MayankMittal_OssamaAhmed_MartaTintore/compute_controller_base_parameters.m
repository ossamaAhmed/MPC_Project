function param = compute_controller_base_parameters
    % load truck parameters
    load('system/parameters_truck');
    
    %% Continuous time model
    % Dynamics: dot_x = A_c * x + B_c * u + B_c_d * d_c
    % Observation: y = C_c * x + D_c * u
    
    A_c = [-(truck.a1o + truck.a12)/truck.m1, truck.a12/truck.m1, 0;
             truck.a12/truck.m2, -(truck.a2o + truck.a12 + truck.a23)/truck.m2, truck.a23/truck.m2;
             0, truck.a23/truck.m3, -(truck.a3o + truck.a23)/truck.m3];
    
    B_c = [1/truck.m1, 0;
           0, 1/truck.m2;
           0, 0];
    B_c_d = diag([1/truck.m1, 1/truck.m2, 1/truck.m3]);
    d_c = [truck.a1o * truck.To + truck.w(1);
           truck.a2o * truck.To + truck.w(2);
           truck.a3o * truck.To + truck.w(3)];
    
    C_c = eye(3);
    D_c = zeros(3, 2);
    
    %% (2) discretization
    Ts = 60;
    sysd = c2d(ss(A_c, [B_c,B_c_d], C_c, []), Ts);
    A = sysd.A;
    B = sysd.B(:,1:2);
    B_d = sysd.B(:,3:5);
    d = d_c;
    
    %% (3) set point computation
    H = [1, 0, 0;
         0, 1, 0];
    r = [-20; 0.25];
    % Steady-state computation: M * [T_sp, p_sp] = [B_d * d; r]
    M = [eye(3) - A, -B;
         H, zeros(2, 2)];
    b = [B_d * d; r];
    x_sp = pinv(M) * b;
    T_sp = x_sp(1:3);
    p_sp = x_sp(4:5);
    
    %% (4) system constraints
    Pcons = truck.InputConstraints;
    Tcons = truck.StateConstraints;
    
    % (4) constraints for delta formulation
    Ucons = Pcons - p_sp;
    Xcons = Tcons - T_sp;
    
    %% (5) LQR cost function
    Q = diag([3000, 1500, 0]);
    R = diag([0.05, 0.05]);
    
    % put everything together
    param.A = A;
    param.B = B;
    param.C=sysd.C;
    param.Q = Q;
    param.R = R;
    [P, ~, K] = dare(param.A, param.B, param.Q, param.R);
    param.F = -K;
    param.P = P;
    param.T_sp = T_sp;
    param.p_sp = p_sp;
    param.Ucons = Ucons;
    param.Xcons = Xcons;
    param.Tcons = Tcons;
    param.Pcons = Pcons;
    param.B_d_disturbance = B_d;
    param.d_d = d;
end

