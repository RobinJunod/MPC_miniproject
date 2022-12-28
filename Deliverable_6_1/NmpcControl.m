classdef NmpcControl < handle
    
    properties
        solver
        nx, nu, N
        nlp_x0
        nlp_lbx, nlp_ubx
        nlp_lbg, nlp_ubg
        nlp_p
        
        T_opt
        sol
        idx
        
        % Warmstart
        nlp_lam_x0
        nlp_lam_g0
    end
    
    methods
        function obj = NmpcControl(rocket, tf)
           
            import casadi.*
            
            N_segs = ceil(tf/rocket.Ts); % MPC horizon
            nx = 12; % Number of states
            nu = 4;  % Number of inputs
            
            % Decision variables (symbolic)
            N = N_segs + 1; % Index of last point
            X_sym = SX.sym('X_sym', nx, N); % state trajectory
            U_sym = SX.sym('U_sym', nu, N-1); % control trajectory)
            
            % Parameters (symbolic)
            x0_sym  = SX.sym('x0_sym', nx, 1);  % initial state
            ref_sym = SX.sym('ref_sym', 4, 1);  % target position
            
            % Default state and input constraints
            ubx = inf(nx, 1);
            lbx = -inf(nx, 1);
            ubu = inf(nu, 1);
            lbu = -inf(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%solve
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % Cost
            cost = 0;
            % Equality constraints (Casadi SX), each entry == 0
            eq_constr = [ ; ];
            % eq_constr = [eq_constr; <new constriants>];
            % eq_constr = [eq_constr; X_sym(2:4,1234) - [1; 2; 3]];

            % discretize the system
            h = 0.1;
            f = @(x,u) rocket.f(x,u);
            f_discrete = @(x,u) RK4(x,u,h,f);
            for k=1:N-1
                if k == 1
                    eq_constr = [eq_constr,X_sym(:,k) - x0_sym(:)];
                end
                eq_constr = [eq_constr; X_sym(:,k+1) - f_discrete(X_sym(:,k), U_sym(:,k))];
            end

            % Inequality constraints (Casadi SX), each entry <= 0
            ineq_constr = [ ; ];
            % Beta < 80 degree
            % others constraints stay the same
            

            % For box constraints on state and input, overwrite entries of
            % lbx, ubx, lbu, ubu defined above
            % constraints on state:
            % alpha
            ubx(4) = 0.1222;
            lbx(4) = -0.1222;
            % beta
            ubx(5) = 0.1222;
            lbx(5) = -0.1222;
            % constraints on input:
            % d1
            ubu(1) = 0.26;
            lbu(1) = -0.26;
            % d2
            ubu(2) = 0.26;
            lbu(2) = -0.26;
            % Pavg
            ubu(3) = 80;
            lbu(3) = 50;            
            % Pdiff
            ubu(4) = 20;
            lbu(4) = -20;
            
           
            % Define system dynamics
            %xdot = rocket.f(X_sym,U_sym);

            %x_next = f_discrete(x0_sym,0);
            [xs, us] = rocket.trim();


            % inequality constraints
            
            
            % cost
            R = speye(4);
            cost = 50*(X_sym(10,:)-ref_sym(1))*(X_sym(10,:)-ref_sym(1))' + ...         %min. x error
                           50*(X_sym(11,:)-ref_sym(2))*(X_sym(11,:)-ref_sym(2))' + ... %min. y error
                           200*(X_sym(12,:)-ref_sym(3))*(X_sym(12,:)-ref_sym(3))' + ...%min. z error
                           25*(X_sym(6,:)-ref_sym(4))*(X_sym(6,:)-ref_sym(4))' + ...   %min. roll error
                           50*(X_sym(9,:)-xs(9))*(X_sym(9,:)-xs(9))'+ ...              %min. v_z error
                           5*(X_sym(1,:)-xs(1))*(X_sym(1,:)-xs(1))'+ ...               %min. w_x error
                           5*(X_sym(2,:)-xs(2))*(X_sym(2,:)-xs(2))'+ ...               %min. w_y error
                           5*(X_sym(3,:)-xs(3))*(X_sym(3,:)-xs(3))'+ ...               %min. w_z error
                           5*(X_sym(4,:)-xs(4))*(X_sym(4,:)-xs(4))'+...                %min. alpha error
                           5*(X_sym(5,:)-xs(5))*(X_sym(5,:)-xs(5))'+...                %min. beta error
                           5*(X_sym(7,:)-xs(7))*(X_sym(7,:)-xs(7))' + ...              %min. v_x error
                           5*(X_sym(8,:)-xs(8))*(X_sym(8,:)-xs(8))' + ...              %min. v_y error
                           trace((U_sym-us)'*R*(U_sym-us));                            %min inputs error


            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % ---- Assemble NLP ------
            nlp_x = [X_sym(:); U_sym(:)];
            nlp_p = [x0_sym; ref_sym];
            nlp_f = cost;
            nlp_g = [eq_constr; ineq_constr];
            
            nlp = struct('x', nlp_x, 'p', nlp_p, 'f', nlp_f, 'g', nlp_g);
            
            % ---- Setup solver ------
            opts = struct('ipopt', struct('print_level', 0), 'print_time', false);
            obj.solver = nlpsol('solver', 'ipopt', nlp, opts);
            
            % ---- Assemble NLP bounds ----
            obj.nlp_x0  = zeros(size(nlp_x));
            
            obj.nlp_ubx = [repmat(ubx, N, 1); repmat(ubu, (N-1), 1)];
            obj.nlp_lbx = [repmat(lbx, N, 1); repmat(lbu, (N-1), 1)];
            
            obj.nlp_ubg = [zeros(size(eq_constr)); zeros(size(ineq_constr))];
            obj.nlp_lbg = [zeros(size(eq_constr)); -inf(size(ineq_constr))];
            
            obj.nlp_p = [zeros(size(x0_sym)); zeros(size(ref_sym))];
            
            obj.nlp_lam_x0 = [];
            obj.nlp_lam_g0 = [];
            
            obj.nx = nx;
            obj.nu = nu;
            obj.N = N;
            obj.T_opt = linspace(0, N * rocket.Ts, N);
            
            obj.idx.X = [1, obj.N * obj.nx];
            obj.idx.U = obj.idx.X(2) + [1, (obj.N-1) * obj.nu];
            obj.idx.u0 = obj.idx.U(1) + [0, obj.nu-1];
        end
        
        function [u, T_opt, X_opt, U_opt] = get_u(obj, x0, ref)
            
            obj.solve(x0, ref);
            
            % Evaluate u0
            nlp_x = obj.sol.x;
            id = obj.idx.u0;
            u = full( nlp_x(id(1):id(2)) );      
            
            if nargout > 1, T_opt = obj.get_T_opt(); end
            if nargout > 2, X_opt = obj.get_X_opt(); end
            if nargout > 3, U_opt = obj.get_U_opt(); end
            return
            
            % Additional evaluation
            % Complete trajectory
            % % X_opt = full(reshape(nlp_x(idx_X(1):idx_X(2)), obj.nx, obj.N));
            % % U_opt = full(reshape(nlp_x(idx_U(1):idx_U(2)), obj.nu, obj.N - 1));
            % %
            % % cost_opt = full(sol.f);
            % % constr_opt = full(sol.g);
            % %
            % % stats = obj.solver.stats;
        end
        
        function solve(obj, x0, ref)
            
            % ---- Set the initial state and reference ----
            obj.nlp_p = [x0; ref];     % Initial condition
            obj.nlp_x0(1:obj.nx) = x0; % Initial guess consistent
            
            % ---- Solve the optimization problem ----
            args = {'x0', obj.nlp_x0, ...
                'lbg', obj.nlp_lbg, ...
                'ubg', obj.nlp_ubg, ...
                'lbx', obj.nlp_lbx, ...
                'ubx', obj.nlp_ubx, ...
                'p', obj.nlp_p, ...
                %                 'lam_x0', obj.nlp_lam_x0, ...
                %                 'lam_g0', obj.nlp_lam_g0
                };
            
            obj.sol = obj.solver(args{:});
            if obj.solver.stats.success ~= true
                solve_status_str = obj.solver.stats.return_status;
                fprintf([' [' class(obj) ': ' solve_status_str '] ']);
                obj.sol.x(obj.idx.u0) = nan;
            end
            
            % Use the current solution to speed up the next optimization
            obj.nlp_x0 = obj.sol.x;
            obj.nlp_lam_x0 = obj.sol.lam_x;
            obj.nlp_lam_g0 = obj.sol.lam_g;
        end
        function T_opt = get_T_opt(obj)
            T_opt = obj.T_opt;
        end
        function X_opt = get_X_opt(obj)
            nlp_x = obj.sol.x;
            id = obj.idx.X;
            X_opt = full(reshape(nlp_x(id(1):id(2)), obj.nx, obj.N));
        end
        function U_opt = get_U_opt(obj)
            nlp_x = obj.sol.x;
            id = obj.idx.U;
            U_opt = full(reshape(nlp_x(id(1):id(2)), obj.nu, obj.N - 1));
        end
    end
end
