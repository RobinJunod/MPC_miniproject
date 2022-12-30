classdef MpcControl_x < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(H/Ts); % Horizon steps
            N = N_segs + 1;      % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.2)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N);
            U = sdpvar(nu, N-1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are
            %       the DISCRETE-TIME MODEL of your system
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            obj = 0;
            con = [];
            
            %% Constraints sub_sys x
            % x in X = { x | Fx <= f } with x of dim 4
            F = [0 1 0 0;
                 0 -1 0 0]; 
            f = [0.1222; 0.1222];
            % u in U = { u | Mu <= m } only for the d2
            M = [1;-1]; m = [0.26; 0.26];
            % model matricies
            A = mpc.A;
            B = mpc.B;
            % cost matrices depending on the inupt and state
            Q = 0.1 * eye(size(mpc.A,2));
            R = 10;

            %% compute final cost final constraints
            % Compute LQR controller for unconstrained system
            [K,Qf,~] = dlqr(A,B,Q,R);
            % MATLAB defines K as -K, so invert its signal
            K = -K; 
            % COMPUTE INVARIANT SET
            Xf = polytope([F;M*K],[f;m]);
            Acl = A + B*K;
            
            % MPT version
            sysX = LTISystem('A',A,'B',B);
            sysX.x.min = [-Inf;-0.1222;-Inf;-Inf]; sysX.x.max = [Inf;0.1222;Inf;Inf];
            sysX.u.min = [-0.26]; sysX.u.max = [0.26];
            sysX.x.penalty = QuadFunction(Q); sysX.u.penalty = QuadFunction(R);
            Xf = sysX.LQRSet;
            %Qf = sysX.LQRPenalty;
            Ff = Xf.A;
            ff = Xf.b;
            %[Ff,ff] = double(Xf);
            % Prepare figure
%             lab = ["{\omega}_y","{\beta}","{v_x}","x"];
%             tit = ["{X_f} projection along dimensions {\beta} and {\omega}_y","{X_f} projection along dimensions {v_x} and {\beta}","X_f projection along dimensions x and v_x","X_f projection along dimensions {\omega}_y and x"];
%             figure()
%             pause
%             for i = 1:4
%                 subplot(2,2,i)
%                 %pause(1)
%                 if i == 4
%                     Xf.projection([4 1]).plot()
%                     ylabel(lab(1))
%                 else
%                     Xf.projection(i:i+1).plot()
%                     ylabel(lab(i+1))
%                 end
%                 xlabel(lab(i))
%                 title(tit(i))
%                 %pause(1)
%             end
%             while 1
%                 prevXf = Xf;
%                 [T,t] = double(Xf);
%                 preXf = polytope(T*Acl,t);
%                 Xf = intersect(Xf, preXf);
%                 if isequal(prevXf, Xf)
%                     break
%                 end
%                 % Visualizing the sets
%                 for i = 1:3
%                     subplot(3,1,i)
%                     Xf.projection(i:i+1).plot()
%                     xlabel(lab(i))
%                     ylabel(lab(i+1))
%                     title(tit(i))
%                 end
%                 pause(0.2)
%             end
%             [Ff,ff] = double(Xf);
            
            
            % WITH YALIMP mpc problem
            con = (X(:,2) == A*X(:,1) + B*U(:,1)) + (M*U(:,1) <= m);
            obj = U(:,1)'*R*U(:,1);
            for i = 2:N-1
                con = con + (X(:,i+1) == A*X(:,i) + B*U(:,i));
                con = con + (F*X(:,i) <= f) + (M*U(:,i) <= m);
                obj = obj + X(:,i)'*Q*X(:,i) + U(:,i)'*R*U(:,i);
            end
            % Final cost and constraints
            con = con + (Ff*X(:,N) <= ff);
            obj = obj + X(:,N)'*Qf*X(:,N);


            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref}, {U(:,1), X, U});
        end
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);

            % Steady-state targets
            xs = sdpvar(nx, 1);
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            obj = 0;
            con = [xs == 0, us == 0];
            



            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
