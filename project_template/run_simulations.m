% Init
clc
clear all
close all
addpath(genpath(cd));
load('system/parameters_scenarios.mat');

%% 1-4: System Modelling
param = compute_controller_base_parameters;

%% 5-6: Execute simulation with LQR

% clear persisten variables of function controller_lqr
clear controller_lqr; 
% initial condition
x0_1 = [3; 1; 0];
T0_1 = param.T_sp + x0_1;
% execute simulation starting from T0_1 using lqr controller with scenario 1
figure;
[T, p] = simulate_truck(T0_1, @controller_lqr, scen1);

%% 7: Execute simulation with LQR

% clear persisten variables of function controller_lqr
clear controller_lqr; 
% initial condition
x0_2 = [-1; -0.1; -4.5];
T0_2 = param.T_sp + x0_2;
% execute simulation starting from T0_2 using lqr controller with scenario 1
figure;
[T, p] = simulate_truck(T0_2, @controller_lqr, scen1);

%% 8: Compute Control Invariance Set
figure;
% Create polyhedron set
[A_x, b_x] = compute_X_LQR;

%% 9: MPC Controller 1

% clear persisten variables of function controller_mpc_1
clear controller_mpc_1; 
% initial condition
x0_1 = [3; 1; 0];
T0_1 = param.T_sp + x0_1;
% execute simulation starting from T0_1 using MPC_1 controller with scenario 1
figure;
[T, p] = simulate_truck(T0_1, @controller_mpc_1, scen1);


% clear persisten variables of function controller_mpc_1
clear controller_mpc_1; 
% initial condition
x0_2 = [-1; -0.1; -4.5];
T0_2 = param.T_sp + x0_2;
% execute simulation starting from T0_2 using MPC_1 controller with scenario 1
figure;
[T, p] = simulate_truck(T0_2, @controller_mpc_1, scen1);

%% 12: MPC Controller 2

% clear persisten variables of function controller_mpc_2
clear controller_mpc_2; 
% initial condition
x0_1 = [3; 1; 0];
T0_1 = param.T_sp + x0_1;
% execute simulation starting from T0_1 using MPC_2 controller with scenario 1
figure;
[T, p] = simulate_truck(T0_1, @controller_mpc_2, scen1);

%% 13: Cost analysis for MPC Controllers 1 and 2

% clear persisten variables of function controller_mpc_1, controller_mpc_2
clear controller_mpc_1; 
clear controller_mpc_2; 
% initial condition
x0_1 = [3; 1; 0];
T0_1 = param.T_sp + x0_1;
fprintf('[MAIN] Temperature at t=0: %f %f %f\n', T0_1');
% execute simulation starting from T0_1 using MPC 1 & 2 controllers with scenario 1
figure;
[T, p] = simulate_truck(T0_1, @controller_mpc_1, scen1);
figure;
[T, p] = simulate_truck(T0_1, @controller_mpc_2, scen1);

fprintf('-------------------------------------\n');

% clear persisten variables of function controller_mpc_1, controller_mpc_2
clear controller_mpc_1; 
clear controller_mpc_2; 
% initial condition
x0_2 = [-1; -0.1; -4.5];
T0_2 = param.T_sp + x0_2;
fprintf('[MAIN] Temperature at t=0: %f %f %f\n', T0_2');
% execute simulation starting from T0_2 using MPC 1 & 2 controllers with scenario 1
figure;
[T, p] = simulate_truck(T0_2, @controller_mpc_1, scen1);
figure;
[T, p] = simulate_truck(T0_2, @controller_mpc_2, scen1);

%% 14: MPC Controller 3

% clear persisten variables of function controller_mpc_3
clear controller_mpc_3; 
% initial condition
x0_1 = [3; 1; 0];
T0_1 = param.T_sp + x0_1;
% execute simulation starting from T0_1 using MPC_3 controller with scenario 1
figure;
[T, p] = simulate_truck(T0_1, @controller_mpc_3, scen1);

% clear persisten variables of function controller_mpc_3
clear controller_mpc_3; 
% initial condition
x0_2 = [-1; -0.1; -4.5];
T0_2 = param.T_sp + x0_2;
% execute simulation starting from T0_2 using MPC_3 controller with scenario 1
figure;
[T, p] = simulate_truck(T0_2, @controller_mpc_3, scen1);

%% 17: MPC Controller 3 on scene 2

% clear persisten variables of function controller_mpc_3
clear controller_mpc_3; 
% initial condition
x0_2 = [-1; -0.1; -4.5];
T0_2 = param.T_sp + x0_2;
% execute simulation starting from T0_2 using MPC_2 controller with scenario 2
figure;
[T, p] = simulate_truck(T0_2, @controller_mpc_3, scen2);

%% 18: MPC Controller 4 on scene 2

% clear persisten variables of function controller_mpc_4
clear controller_mpc_4; 
% initial condition
x0_2 = [-1; -0.1; -4.5];
T0_2 = param.T_sp + x0_2;
% execute simulation starting from T0_2 using MPC_4 controller with scenario 2
figure;
[T, p] = simulate_truck(T0_2, @controller_mpc_4, scen2);

%% 19: MPC Controller 3 and 4 on scene 1

% clear persisten variables of function controller_mpc_4
clear controller_mpc_3; 
clear controller_mpc_4; 
% initial condition
x0_2 = [-1; -0.1; -4.5];
T0_2 = param.T_sp + x0_2;
% execute simulation starting from T0_2 using MPC 3 & 4 controller with scenario 1
figure;
[T, p] = simulate_truck(T0_2, @controller_mpc_3, scen1);
figure;
[T, p] = simulate_truck(T0_2, @controller_mpc_4, scen1);

%% 22: MPC Controller 3 and 5 on scene 3

% clear persisten variables of function controller_mpc_4
clear controller_mpc_3; 
clear controller_mpc_5; 
% initial condition
x0_1 = [3; 1; 0];
T0_1 = param.T_sp + x0_1;
% execute simulation starting from T0_1 using MPC 3 & 5 controller with
% scenario 3
figure;
[T, p] = simulate_truck(T0_1, @controller_mpc_3, scen3);
x0_1 = [3; 1; 0];
T0_1 = param.T_sp + x0_1;
figure;
[T, p] = simulate_truck(T0_1, @controller_mpc_5, scen3);

%% 23: FORCES MPC Controller on scene 1

% initial condition
x0_2 = [-1; -0.1; -4.5];
T0_2 = param.T_sp + x0_2;

% compare running time
N = 10;
t_forces = zeros(N, 1);
t_ya = zeros(N, 1);

% load the parameters and optimizers
% doing it once to ensure that parameters are initialized
clear controller_mpc_1_forces;
clear controller_mpc_1; 
[~, ~, ~] = simulate_truck(T0_2, @controller_mpc_1_forces, scen1);
[~, ~, ~] = simulate_truck(T0_2, @controller_mpc_1, scen1);

for i = 1:N
    close all;
    % run the controller implementation in FORCES
    figure;
    [~, ~, t_sim_forces] = simulate_truck(T0_2, @controller_mpc_1_forces, scen1);
    t_forces(i) = t_sim_forces;
    % run the controller implementation in YALMIP
    figure;
    [~, ~, t_sim] = simulate_truck(T0_2, @controller_mpc_1, scen1);
    t_ya(i) = t_sim;
end

disp('------------------------------------------------------')
disp('FORCES Timing: ')
disp(mean(t_forces))
disp(std(t_forces))

disp('YALMIP Timing:')
disp(mean(t_ya))
disp(std(t_ya))