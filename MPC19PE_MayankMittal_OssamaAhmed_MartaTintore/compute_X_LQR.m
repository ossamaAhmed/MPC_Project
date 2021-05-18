% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}
function [A_x, b_x] = compute_X_LQR
    % get basic controller parameters
    param = compute_controller_base_parameters;
    
    % compute gains for LQR
    [~, ~, K] = dare(param.A, param.B, param.Q, param.R);
    param.F = - K;
    
    %% Here you need to implement the X_LQR computation and assign the result.
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Manual Solution for computing polytopic constraint
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % For LQR controller: x = F*x
    % Set of constraints are: x_min <= x <= x_max, u_min <= F*x <= u_max
    % Writing this in the form given A_x * x <= b_x
    A_x = [eye(3); -eye(3); param.F; -param.F];
    b_x = [param.Xcons(:, 2); -param.Xcons(:, 1); param.Ucons(:, 2); -param.Ucons(:, 1)];
    Xp = Polyhedron('A', A_x, 'b', b_x);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % MPT  Solution for computing approxmiate control invariant set
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % For LQR controller: x(k+1) = (A + B*F)*x
    systemLQR = LTISystem('A', param.A + param.B * param.F, 'B', zeros(3, 2));
    systemLQR.x.with('setConstraint');
    systemLQR.x.setConstraint = Xp;
    invarianceSet = systemLQR.invariantSet();
    A_x = invarianceSet.A;
    b_x = invarianceSet.b;
    % To plot:
    invarianceSet.plot();
end