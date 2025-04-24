%% 全身柔顺框架
clear;clc;
format long
%------------------------------初始参数-----------------------------------%
h = 0.005;
n = 3;
N = n/h;
skinstates = repmat(struct('x_deflection',zeros(2,N),'dx_deflection',zeros(2,N),'ddx_deflection',zeros(2,N),...
'jacobian_matrix',zeros(2,3),'weight',zeros(1,N),'x_desire',zeros(2,N),'x_real',zeros(2,N),'x_track_error',...
zeros(2,N),'x_new_desire',zeros(2,N),'x_de_r',zeros(1,N),'dx_de_r',zeros(1,N),'ddx_de_r',zeros(1,N),...
'jacobian_matrix_r',zeros(1,3)),1,2);

critical_point = repmat(struct('radius',0.1,'center',zeros(2,N)),1,17);
q_real = zeros(3,N);
q_real(:,1) = [0;pi/2 - 0.01;-pi/4];
x_init = forward_kinematics(q_real(2:3,1), 1, 1);
dq_cmd = zeros(3,N);
dq_desire = zeros(3,N);
force = zeros(2,2,N);
Minv = 1;
damp = 4;
stiff = 0;
damp_cost = diag([1,0,0]);
u1_star = zeros(3,N);u2_star = zeros(3,N);

t = zeros(1,N);
for i = 1:N
    if i < N/2
        force(:,:,i) = [0,0;
                        0,0];
    else
        force(:,:,i) = [0,0;
                        0,0];
    end
end

for i = 1:N
    if i < N/2
        torque(:,:,i) = [0,3];
    else
        torque(:,:,i) = [0,3];
    end
end
%% loop begin
for i = 1:N-1
    % update the jacobian matrix
    skinstates(1).jacobian_matrix = [1, -sin(q_real(2,i)),0;0,cos(q_real(2,i)),0];
    skinstates(2).jacobian_matrix = [1, - sin(q_real(2,i) + q_real(3,i)) - 1*sin(q_real(2,i)), -sin(q_real(2,i) + q_real(3,i)); 0,   1*cos(q_real(2,i) + q_real(3,i)) + 1*cos(q_real(2,i)),  1*cos(q_real(2,i) + q_real(3,i))];
    % forward kinematics
    pos = forward_kinematics(q_real(2:3,i), 1, 1)++[q_real(1,i),0,q_real(1,i),0];
    skinstates(1).x_real(:,i) = pos(1:2).';
    skinstates(2).x_real(:,i) = pos(3:4).';
    % obstacle position
    critical_point(14).center(:,i) = [1.3;2];critical_point(15).center(:,i) = [1.7;2];
    critical_point(16).center(:,i) = [1.3;1.6];critical_point(17).center(:,i) = [1.7;1.6];

    for k = 1:13
        % critical point
        critical_point(k).center(:,i) = forward_critical_point(k,q_real(:,i),pos);
    end
    % collision constraint & guidance 
    dot_hq = [];count = 0;hq_cbf = [];
    H_first = 0.001*eye(3,3); f_first = zeros(1,3);
    H_second = 0.001*eye(3,3); f_second = zeros(1,3);
    
    for k = 1:13
        for m = 1:17
            if m ~= k 
                count = count + 1;
                distance_vec = critical_point(k).center(:,i)-critical_point(m).center(:,i);
                h_cbf(m,k) = norm(distance_vec)^2 - (critical_point(k).radius + critical_point(m).radius)^2;
                jacobian = get_cri_jacobian(k,q_real(:,i));
                h_dot_cbf = 2*(critical_point(k).center(:,i)-critical_point(m).center(:,i)).'*jacobian;
                dot_hq = [dot_hq;h_dot_cbf];
                hq_cbf = [hq_cbf;h_cbf(m,k)];
                % guidance
            end
        end
    end
    A = -1*dot_hq;
    b = 1*hq_cbf;
    % admittance update
    lb = [-1;-1;-1];ub = [1;1;1];
    for j = 1:2
        skinstates(j).x_real(:,i) = pos(2*(j-1)+1:2*(j-1)+2)';
        
        skinstates(j).ddx_deflection(:,i) = Minv * (force(:,j,i) - damp*skinstates(j).dx_deflection(:,i) - stiff*skinstates(j).x_deflection(:,i));
        skinstates(j).dx_deflection(:,i+1) = skinstates(j).ddx_deflection(:,i)*h + skinstates(j).dx_deflection(:,i);
        skinstates(j).x_deflection(:,i+1) = skinstates(j).dx_deflection(:,i)*h + skinstates(j).x_deflection(:,i);
        
        skinstates(j).x_new_desire(:,i) = x_init(2*(j-1)+1:2*(j-1)+2)' + skinstates(j).x_deflection(:,i);
        skinstates(j).x_track_error(:,i) = skinstates(j).x_new_desire(:,i) - skinstates(j).x_real(:,i);

        skinstates(j).ddx_de_r(:,i) = Minv * (torque(:,j,i) - damp*skinstates(j).dx_de_r(:,i) - stiff*skinstates(j).x_de_r(:,i));
        skinstates(j).dx_de_r(:,i+1) = skinstates(j).ddx_de_r(:,i)*h + skinstates(j).dx_de_r(:,i);
%         skinstates(j).x_de_r(:,i+1) = skinstates(j).dx_de_r(:,i)*h + skinstates(j).x_de_r(:,i);
        
        skinstates(j).weight(i) =  prindex(skinstates(j).dx_de_r(:,i+1),skinstates(j).dx_deflection(:,i));
        skinstates(j).weight(2) = 1;
        
        H_first = skinstates(j).weight(i)*skinstates(j).jacobian_matrix.'*skinstates(j).jacobian_matrix + H_first;
        f_first = -skinstates(j).weight(i)*skinstates(j).dx_deflection(:,i+1).'* skinstates(j).jacobian_matrix + f_first;
    end
    u1_star(:,i) = quadprog(H_first,f_first,A,b,[],[],lb,ub);
    %second priority
    dq_desire = [-1;0;0];
    Aeq = []; beq = [];
    skinstates(1).jacobian_matrix_r = [0,1,0];
    skinstates(2).jacobian_matrix_r = [0,1,1];

    %rotate
    for j = 1:2    
        H_second = skinstates(j).weight(i)*skinstates(j).jacobian_matrix_r.'*skinstates(j).jacobian_matrix_r+ H_second;
        f_second = -skinstates(j).weight(i)*skinstates(j).dx_de_r(:,i+1).'* skinstates(j).jacobian_matrix_r + f_second;
    end

    for j = 1:2
        Aeq_skin = skinstates(j).weight(i)*skinstates(j).jacobian_matrix;
        beq_skin = skinstates(j).weight(i)*skinstates(j).jacobian_matrix *u1_star(:,i);
        Aeq = [Aeq;Aeq_skin]; beq = [beq;beq_skin];
    end
    u2_star(:,i) = quadprog(H_second,f_second,A,b,Aeq,beq,lb,ub);
    
    dq_cmd(:,i) = u2_star(:,i);
    q_real(:,i+1) = dq_cmd(:,i)*h + q_real(:,i);
    t(i+1) = t(i)+h;
end
%% draw figure
figure(1)
plot(t,dq_cmd(1,:),'-','linewidth',2),title('ddqe');hold on;
plot(t,dq_cmd(2,:),'-','linewidth',2),title('dqe'   );hold on;
plot(t,dq_cmd(3,:),'-','linewidth',2),title('qe');hold on;
legend('dq_0','dq_1','dq_2');
figure(2)
plot(t,skinstates(1).x_real(1,:),'-','linewidth',2);hold on;
plot(t,skinstates(1).x_real(2,:),'-','linewidth',2),title('x_{real}1');hold on;
legend('x1_r','y1_r');
figure(3)
plot(t,skinstates(2).x_real(1,:),'-','linewidth',2);hold on;
plot(t,skinstates(2).x_real(2,:),'-','linewidth',2),title('x_{real}2');hold on;
legend('x1_r','y1_r');
figure(4)
plot(t,skinstates(2).x_new_desire(1,:),'-','linewidth',2);hold on;
plot(t,skinstates(2).x_new_desire(2,:),'-','linewidth',2),title('x_{x_new_desire}2');hold on;
legend('x_new_desire','y_new_desire');
% 可视化
figure(5)
for i = 1:N
% 将绘图操作添加到动画中
    pos = forward_kinematics(q_real(2:3,i), 1, 1)+[q_real(1,i),0,q_real(1,i),0];
    x2(i) = pos(3);y2(i) = pos(4);
    plot([q_real(1,i) pos(1)],[0 pos(2)],'LineWidth',2);  % 画直线
    hold on;
%     obstale3 = [0.8,1.5,0,0.3];
%     corner_x = [obstale3(1), obstale3(2), obstale3(2), obstale3(1), obstale3(1)];
%     corner_y = [obstale3(3), obstale3(3), obstale3(4), obstale3(4), obstale3(3)];
%     plot(corner_x, corner_y, 'LineWidth', 2, 'Color', 'r');hold on;
    plot([pos(1) pos(3)],[pos(2) pos(4)],'LineWidth',2);  % 画直线
    u1 = force(1,1,i);
    v1 = force(2,1,i);
    u2 = force(1,2,i);
    v2 = force(2,2,i);
    scale = 0.2;
    quiver(pos(1), pos(2), u1, v1,scale, 'MaxHeadSize', 0.7,'linewidth', 2);
    quiver(pos(3), pos(4), u2, v2,scale, 'MaxHeadSize', 0.7,'linewidth', 2);
    width = 1;height = 0.4;
    rectangle('Position', [q_real(1,i)-width/2,-height/2, width, height], 'FaceColor', 'none', 'EdgeColor', 'k');
	for k = 1:17
        position_circle = [critical_point(k).center(1,i)-critical_point(k).radius,critical_point(k).center(2,i)-...
            critical_point(k).radius,critical_point(k).radius*2,critical_point(k).radius*2];
        rectangle('Position', position_circle, 'Curvature',[1,1],'LineWidth',1.5);
    end
    hold off;
    xlim([-3 3]); ylim([-2 3]);  % 设置坐标轴范围
    xlabel('X'); ylabel('Y');  % 设置坐标轴标签
    grid on;  % 显示网格
    drawnow;  % 刷新图形
end
