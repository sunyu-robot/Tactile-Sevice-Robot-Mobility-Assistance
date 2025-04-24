function u = centroidalNMPC(Q, R, q0, qd, p_h, h)

   nmpc = casadi.Opti();
   opts = struct('ipopt', struct('print_level', 0, 'warm_start_init_point', 'yes', 'max_iter', 100));
   nmpc.solver('ipopt', opts);
   X_ref = generateReference(q0,qd',p_h);
   X = nmpc.variable(18,p_h); 
   F = nmpc.variable(12,p_h); 
   nmpc.subject_to ( X(:,1) == q0 );
   for k = 1 : p_h-1
        % dynamics constraint
        dX = centroidalDynamics(F(:,k), X(:,k));
        nmpc.subject_to( X(:, k+1) == X(:, k) + h*dX );
        % 基础假设 力是符合摩擦锥约束的，位置沿着直线移动
%         % friction cone
        nmpc.subject_to( F(1,k) == 0);
        nmpc.subject_to( F(2,k) <= 0);
        nmpc.subject_to( F(3,k) == 0);
        nmpc.subject_to( F(4,k) == 0);
        nmpc.subject_to( F(5,k) >= 0);
        nmpc.subject_to( F(6,k) == 0);
        % geometry constriant i.e. only z axis motion is allowed
        nmpc.subject_to( F(7,k) == 0);
        nmpc.subject_to( F(8,k) == 0);
        nmpc.subject_to( F(10,k) == 0);
        nmpc.subject_to( F(11,k) == 0);
        % lower bound upper bound
        nmpc.subject_to( -40 <= F(:,k) <= 40);
   end
   
   J = 0;
   % cost function
   for k = 1 : p_h
        J = J + (X_ref(:,k) - X(:,k))' * Q ... 
            * (X_ref(:,k) - X(:,k)) + F(:,k)' * R * F(:,k);
   end
   nmpc.minimize(J);
   solution = nmpc.solve();
   f_all = solution.value(F);
   u = f_all(:,1);
end