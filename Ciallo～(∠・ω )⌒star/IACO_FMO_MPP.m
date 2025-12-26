function [R_best,F_best,L_best,T_best,S_best,S_ave,Shortest_Route,Shortest_Length] = IACO_FMO_MPP(D,initial,destination,dis,h,NC_max,sizepop,E_max,stage_threshold)
%% 鱼群迁移优化算法的路径规划问题求解函数
%% 主要符号说明
%% D        8方向距离转移矩阵
%% initial  初始坐标
%% destination  终点坐标
%% dis      距离启发矩阵
%% h        高度矩阵
%% NC_max   最大迭代次数
%% sizepop  鱼群数量
%% E_max    最大能量值
%% stage_threshold 阶段阈值
%% R_best   各代最佳路线
%% L_best   各代最佳路线的长度
%% ================================================================
%% 第一步：变量初始化
global MM;
global Lgrid;
global Dir;

NC = 1;                         % 迭代计数器
R_best = zeros(NC_max, MM^2);   % 各代最佳路线
R_best_to_direct = zeros(NC_max, MM^2); % 各代最佳路线（转移方向）
L_best = inf.*ones(NC_max, 1);  % 各代最佳路线的长度
F_best = zeros(NC_max, 1);      % 各代最佳路线高度均方差
T_best = zeros(NC_max, 1);      % 各代最佳路线转弯次数
S_best = zeros(NC_max, 1);      % 各代最佳路线综合得分
S_ave = zeros(NC_max, 1);       % 各代路线的平均长度
inum = MM + (initial(1)/Lgrid-0.5)*MM - (initial(2)/Lgrid-0.5);
dnum = MM + (destination(1)/Lgrid-0.5)*MM - (destination(2)/Lgrid-0.5);
stage = ones(sizepop, 1);       % 鱼群阶段，初始化为阶段1
enger = 2*E_max*ones(sizepop, 1); % 鱼群能量，初始化为较大能量值以支持全局探索
best_route_prev = zeros(1, MM^2);
Routes = zeros(sizepop, MM^2);  % 存储路径
to_direct = zeros(sizepop, MM^2); % 存储转移方向
pheromone_matrix = zeros(MM^2, 1); % 用于存储类似信息素的数据
pheromone_decay = 0.95; % 信息素衰减系数
best_pheromone_increment = 2.0; % 最优鱼留下的信息素增量
direction_cache = cell(MM^2, 1); % 用于缓存每个栅格的可行方向
direction_valid = false(MM^2, 1); % 标记哪些栅格的缓存是有效的

%% 第二步：迭代寻找最优路径
while NC <= NC_max
    if NC == 1
        direction_cache = cell(MM^2, 1); % 重置缓存
        direction_valid = false(MM^2, 1); % 重置缓存有效标记
    end
    Routes(:, 1) = inum;
    for i = 1:sizepop % 路径探索
        j = 2; % 从第二个栅格开始
        if NC > 1
            Routes(i, 2:end) = 0;
            to_direct(i, :) = 0;
            % 如果是第一条鱼且上一次迭代有最优解，则使用上一次迭代的最优路线
            if i == 1 && best_route_prev(1) ~= 0
                valid_idx = find(best_route_prev ~= 0);
                Routes(i, 1:length(valid_idx)) = best_route_prev(valid_idx);
                if length(valid_idx) > 1
                    for dir_idx = 1:(length(valid_idx)-1)
                        current_node = Routes(i, dir_idx);
                        next_node = Routes(i, dir_idx+1);
                        if direction_valid(current_node)
                            cached_dirs = direction_cache{current_node};
                            for k_dir = 1:size(cached_dirs, 1)
                                if cached_dirs(k_dir, 1) == next_node
                                    to_direct(i, dir_idx) = cached_dirs(k_dir, 2);
                                    break;
                                end
                            end
                        end
                    end
                    j = length(valid_idx) + 1; % 从最优路线的末尾继续搜索
                    continue; % 跳过当前循环，继续路径搜索
                end
            end
        end
        while Routes(i, j-1) ~= dnum && j < MM^2 % 路径搜索过程
            visited = Routes(i, 1:(j-1)); % 已访问的栅格
            current_grid = visited(end); % 当前栅格
            if direction_valid(current_grid) % 检查是否有缓存的可行方向
                cached_directions = direction_cache{current_grid};
                valid_indices = [];
                for idx = 1:size(cached_directions, 1)
                    next_grid = cached_directions(idx, 1);
                    if isempty(find(visited == next_grid, 1))
                        valid_indices = [valid_indices, idx];
                    end
                end
                if ~isempty(valid_indices)
                    J = cached_directions(valid_indices, 1)'; % 待访问栅格，转置为行向量
                    N = cached_directions(valid_indices, 2)'; % 待访问栅格转移方向，转置为行向量
                else
                    J = zeros(1, 1);
                    N = J;
                end
            else
                % 没有缓存，需要计算可行方向
                J = zeros(1, 1);  % 待访问的栅格
                N = J;            % 待访问的栅格转移方向
                Jc = 1;           % 循环下标
                % 寻找可行的下一步栅格
                all_directions = []; % 存储所有可行方向（不考虑已访问）
                for k = 1:8
                    k1 = Dir(k) + current_grid;
                    if D(current_grid, k) == inf
                        continue
                    end
                    % 存储所有可行方向（不考虑已访问）
                    all_directions = [all_directions; k1, k];
                    % 检查是否已访问
                    if isempty(find(visited == k1, 1))
                        J(Jc) = k1; % 待访问栅格标号矩阵
                        N(Jc) = k;  % 待访问栅格转移标号矩阵
                        Jc = Jc + 1;
                    end
                end
                % 更新缓存
                if ~isempty(all_directions)
                    direction_cache{current_grid} = all_directions;
                    direction_valid(current_grid) = true;
                end
            end
            % 如果没有可行的下一步，表示遇到死路
            if J == 0
                Routes(i, :) = 0;
                to_direct(i, :) = 0;
                break
            end
            % 判断是全局探索还是局部探索
            energy_ratio = enger(i) / E_max;
            is_global_exploration = energy_ratio > 1.5; % 能量较高时进行全局探索
            % 计算每个可能方向的启发值
            scores = zeros(length(J), 1);
            for k = 1:length(J)
                % 距离启发
                dist_factor = dis(J(k));
                % 高度启发
                height_diff = abs(h(visited(end)) - h(J(k)));
                % 转向启发（如果不是第一步）
                turn_factor = 0;
                if j > 2
                    turn_factor = ~(~(N(k) - to_direct(i, j-2)));
                end 
                if is_global_exploration
                    % 综合评分（越小越好）
                    % scores(k) = dist_factor*2;
                    scores(k) = dist_factor*3 + height_diff*3 + turn_factor*3;
                else
                    % 信息素启发（检查该栅格的信息素浓度）
                    pheromone_factor = 0;
                    if pheromone_matrix(J(k)) > 0
                        % 信息素值越大，评分越低（越好）
                        pheromone_factor = -2 * log(1 + pheromone_matrix(J(k))); % 使用对数函数避免信息素值过大导致的过度偏好
                    end
                    % 综合评分（越小越好）
                    % scores(k) = dist_factor*3 + height_diff*3 + turn_factor*3 + pheromone_factor*2;
                    scores(k) = height_diff*2 + turn_factor*3 + pheromone_factor*5;
                end
            end
            % 使用轮盘赌选择下一步
            % 将分数转换为选择概率（分数越小，概率越大）
            neg_scores = -scores;  
            exp_scores = exp(neg_scores - max(neg_scores));  % 数值稳定处理
            probs = exp_scores / sum(exp_scores);
            % 轮盘赌选择
            cum_probs = cumsum(probs);
            r = rand();
            selected = find(cum_probs >= r, 1, 'first');
            if isempty(selected)
                selected = length(probs); % 防止浮点误差
            end
            % 更新路径
            Routes(i, j) = J(selected);
            to_direct(i, j-1) = N(selected);
            j = j + 1;
        end
    end
    %% 第三步：记录本次迭代最佳与更新信息素
    % 计算每条路径的评价指标
    L = zeros(sizepop, 1); % 路径长度
    F = zeros(sizepop, 1); % 高度均方差
    T = zeros(sizepop, 1); % 转弯次数
    S = zeros(sizepop, 1); % 综合得分
    x_weight = 1;  % 路径长度权重
    y_weight = 100; % 高度均方差权重
    z_weight = 1;   % 转弯次数权重
    for i = 1:sizepop
        if Routes(i, :) == 0 % 跳过死路
            L(i) = inf;
            F(i) = inf;
            T(i) = inf;
            S(i) = inf;
            continue
        end
        % 计算路径长度
        j = 2;
        L(i) = Lgrid * D(Routes(i, 1), to_direct(i, 1));
        while Routes(i, j+1) ~= 0 && j < MM^2
            L(i) = L(i) + Lgrid * D(Routes(i, j), to_direct(i, j));
            T(i) = T(i) + ~(~(to_direct(i, j) - to_direct(i, j-1)));
            j = j + 1;
        end
        % 计算高度均方差
        F(i)=std(h(Routes(i,:)~=0));
        % 计算综合得分
        S(i) = x_weight * L(i) + y_weight * F(i) + z_weight * T(i);
    end
    % 更新全局最优解
    [min_S, min_idx] = min(S);
    if min_S < inf
        S_best(NC) = min_S;
        L_best(NC) = L(min_idx);
        F_best(NC) = F(min_idx);
        T_best(NC) = T(min_idx);
        R_best(NC, :) = Routes(min_idx, :);
        R_best_to_direct(NC, :) = to_direct(min_idx, :);
        S_ave(NC)=mean(S(S~=inf));
        % 更新信息素启发矩阵
        % 首先对所有信息素进行衰减
        pheromone_matrix = pheromone_matrix * pheromone_decay;
        % 只有当前迭代中最优的鱼才留下信息素
        valid_route = Routes(min_idx, Routes(min_idx, :) ~= 0); % 获取最优路径
        % 在最优路径上增加信息素
        for k = 1:length(valid_route)
            pheromone_matrix(valid_route(k)) = pheromone_matrix(valid_route(k)) + best_pheromone_increment;
        end
        % 保存当前迭代的最优路线，用于下一次迭代
        best_route_prev = Routes(min_idx, :);
    end
    %% 第四步： 更新鱼群能量和阶段
    for i = 1:sizepop
        % 基础能量衰减 - 随着迭代次数增加，衰减率增大，使后期更多鱼进入局部探索
        decay_rate = 0.9 + 0.07*rand - 0.4 * (NC / NC_max); % 随迭代进行，衰减率增大
        decay_rate = max(0.6, decay_rate); % 确保衰减率不低于0.6
        enger(i) = enger(i) * decay_rate;
        if S(i) < inf
            if S(i) < S_best(NC)
                % 找到更优解，增加能量以支持全局探索
                energy_reward = 0.8 * E_max;
                enger(i) = enger(i) + energy_reward;
            end
            % 所有找到有效解的鱼都进化阶段
            stage(i) = stage(i) + 1;
        end
        % 能量归零处理
        if enger(i) <= 0
            % 使用固定的重置能量值
            reset_energy = 2 * E_max;
            enger(i) = reset_energy;
            stage(i) = 1;         % 重置为阶段1
            Routes(i, 2:end) = 0;
            to_direct(i, :) = 0;
        end
    end
    % 处理不同阶段的鱼
    stage_2 = find(stage == 2);
    stage_3 = find(stage == 3);
    stage_4 = find(stage >= stage_threshold);
    % 阶段2的鱼有小概率重新初始化
    for s2 = 1:length(stage_2)
        if rand < 0.15
            stage(stage_2(s2)) = 1;  % 重置为阶段1
            enger(stage_2(s2)) = 2 * E_max;  % 重置为较大能量值
        end
    end
    % 阶段3的鱼有较大概率重新初始化
    for s3 = 1:length(stage_3)
        if rand < 0.35
            stage(stage_3(s3)) = 1;  % 重置为阶段1
            enger(stage_3(s3)) = 2 * E_max;  % 重置为较大能量值
        end
    end
    % 阶段4的鱼必须重新初始化
    for s4 = 1:length(stage_4)
        stage(stage_4(s4)) = 1;  % 重置为阶段1
        enger(stage_4(s4)) = 2 * E_max;  % 重置为较大能量值
    end
    % 进入下一次迭代
    NC = NC + 1;
end
%% 第五步：输出结果
[~, best_iter] = min(S_best);
Shortest_Route = R_best(best_iter, :);
Shortest_Route = Shortest_Route(Shortest_Route ~= 0); % 去掉末尾的0
Shortest_Length = L_best(best_iter);
end
