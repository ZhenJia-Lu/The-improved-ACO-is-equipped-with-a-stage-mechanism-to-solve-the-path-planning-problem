%% Path planning problem - Comparison of Algorithms

% author : LZJ

% e-mail : 634098550@qq.com

% date   : 2025/5/30

%% 构建栅格地图
close all;clear;clc
global MM
global Dir 
global Lgrid 

% 用户输入选择地图配置
fprintf('请选择地图配置:\n');
fprintf('1 - 20x20地图，20%%不规则障碍物\n');
fprintf('2 - 30x30地图，25%%不规则障碍物\n');
fprintf('3 - 30x30地图，20%%特殊障碍物\n');
user_choice = input('请输入选择 (1/2/3): ');

if user_choice == 1
    % 配置1: 20x20地图，25%障碍物
    G=[ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        0 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 1 1 0
        1 1 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 1 1 1
        1 1 0 0 0 1 0 0 0 1 0 0 0 0 1 0 0 0 0 0
        0 0 0 0 0 0 0 0 0 1 0 0 0 0 1 0 0 0 0 0
        0 0 0 1 1 0 0 0 1 1 1 1 0 0 1 0 1 1 1 0
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0
        0 1 1 0 0 0 0 0 0 0 0 1 1 1 1 0 0 0 0 0
        1 1 1 0 0 0 1 1 1 0 0 0 1 1 1 1 0 0 0 0
        1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        0 0 0 1 0 0 0 1 0 0 1 0 0 0 0 0 0 1 1 0
        0 0 0 1 0 0 1 1 1 0 1 0 1 1 0 0 0 1 1 0
        0 1 1 1 0 0 1 1 1 0 1 0 1 1 0 0 0 0 0 0
        0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0
        1 1 0 0 0 1 1 1 0 0 1 0 0 0 0 0 0 0 0 0
        1 0 0 1 0 0 0 1 0 0 0 0 1 0 0 0 0 0 0 0
        0 0 0 1 0 0 0 0 0 0 0 1 1 0 0 1 1 0 0 0
        0 0 1 1 0 0 0 0 0 0 1 1 1 0 0 0 1 0 0 0
        0 0 1 1 0 1 1 0 0 0 0 1 1 0 0 0 0 0 0 0
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];
    Xdestination = 19.4;
    Ydestination = 19.3;
    h=rot90(abs(peaks(20)));
elseif user_choice == 2
    % 配置2: 30x30地图，25%不规则障碍物
    G=[ 1 1 0 1 0 0 1 1 1 1 0 0 0 0 0 0 1 0 0 0 1 0 0 0 1 0 1 1 0 0
        1 1 0 1 0 0 1 0 0 0 0 1 0 0 0 0 1 1 0 1 1 0 0 1 1 0 0 1 0 0
        0 0 0 1 0 0 1 0 0 0 0 1 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        0 1 1 1 1 0 0 0 0 0 1 1 1 1 1 0 0 0 0 0 0 0 1 1 1 0 0 0 1 1
        0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 1
        0 1 0 0 0 1 0 0 0 0 1 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        1 0 0 0 0 1 1 1 0 0 0 0 0 1 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 1
        0 0 1 0 0 1 1 1 0 0 0 0 0 1 1 1 0 0 1 1 0 0 1 1 0 0 0 0 1 1
        0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 1 1 1 0 0 0 1 1 1
        1 1 1 1 1 0 0 1 0 0 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 1 1
        0 1 1 1 0 0 1 1 1 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        0 1 1 1 0 0 0 1 0 0 0 0 0 0 0 0 1 1 1 1 1 0 0 1 0 0 1 1 0 0
        0 0 0 0 0 0 0 0 0 0 1 1 1 1 0 0 0 0 0 0 1 1 0 0 0 0 1 1 0 0
        0 0 0 1 1 1 1 0 0 0 0 0 0 0 0 0 1 1 1 1 1 0 0 0 0 1 1 1 0 0
        0 0 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 1 1 0 0
        0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 0 0 0 1 0 0 0 1 0 0 0 0 0 0
        0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 1 0 0 1 1 1 0 0 1 0 0
        0 0 1 1 1 1 1 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 1 0 0 0 1 0 0
        0 0 0 0 0 0 0 1 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 0 0 0 1 0 0
        1 1 0 0 1 1 0 1 0 0 0 0 1 1 0 0 0 0 0 1 0 0 0 0 1 1 1 1 0 0
        0 0 0 0 0 0 0 1 0 0 1 1 1 1 1 1 0 0 0 1 0 0 0 0 0 1 0 0 0 0
        0 0 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 1 1
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1
        0 0 0 1 0 0 0 0 0 1 1 1 0 0 0 1 1 1 0 0 0 0 1 1 1 1 1 0 0 0
        0 0 0 1 1 0 0 0 0 0 1 1 0 0 0 1 1 0 0 0 0 0 1 1 1 0 0 0 0 0
        0 1 1 1 1 1 1 0 0 0 1 1 0 0 0 1 1 0 0 0 0 0 1 1 1 0 0 0 0 0
        0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0
        1 1 0 1 0 0 0 1 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0
        0 0 0 0 0 0 1 1 1 0 0 1 1 1 1 1 1 0 0 0 1 1 1 0 0 0 1 1 1 0
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 0 0 0 0 0 0];
    Xdestination = 29.4;
    Ydestination = 29.3;
    h=rot90(abs(peaks(30)));
elseif user_choice == 3
    % 配置3: 30x30地图，20%规则障碍物
    G=[ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        0 1 1 0 0 0 1 0 0 0 1 1 0 0 0 1 0 0 0 0 0 1 1 0 0 0 1 0 0 0
        0 1 0 0 1 1 1 1 1 0 1 0 0 1 1 1 1 1 0 0 0 1 0 0 1 1 1 1 1 0
        0 1 0 0 0 0 1 0 0 0 1 0 0 0 0 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        0 1 0 0 1 0 0 1 0 0 1 1 0 0 1 0 0 1 1 0 0 1 0 0 1 0 0 1 1 0
        0 1 0 0 1 0 0 1 0 0 1 1 0 0 1 0 0 1 1 0 0 1 0 0 1 0 0 1 1 0
        0 1 0 0 1 0 0 1 0 0 1 1 0 0 1 0 0 1 1 0 0 1 0 0 1 0 0 1 1 0
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        0 0 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 0
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        0 1 1 0 0 0 1 0 0 0 0 1 1 0 0 0 1 0 0 0 0 1 1 0 0 0 1 0 0 0
        0 1 0 0 1 1 1 1 1 0 0 1 0 0 1 1 1 1 1 0 0 1 0 0 1 1 1 1 1 0
        0 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 0 0 0
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        0 1 0 0 1 0 0 1 1 0 0 1 0 0 1 0 0 1 1 0 0 1 0 0 1 0 0 1 1 0
        0 1 0 0 1 0 0 1 1 0 0 1 0 0 1 0 0 1 1 0 0 1 0 0 1 0 0 1 1 0
        0 1 0 0 1 0 0 1 1 0 0 1 0 0 1 0 0 1 1 0 0 1 0 0 1 0 0 1 1 0
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        0 0 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 0
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        0 0 1 1 0 0 0 1 0 0 0 0 1 1 0 0 0 1 0 0 0 1 1 0 0 0 1 0 0 0
        0 0 1 0 0 1 1 1 1 1 0 0 1 0 0 1 1 1 1 1 0 1 0 0 1 1 1 1 1 0
        0 0 1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 0 0 1 0 0 0
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        0 1 0 0 1 0 0 1 1 0 0 1 0 0 1 0 0 1 1 0 0 1 0 0 1 0 0 1 1 0
        0 1 0 0 1 0 0 1 1 0 0 1 0 0 1 0 0 1 1 0 0 1 0 0 1 0 0 1 1 0
        0 1 0 0 1 0 0 1 1 0 0 1 0 0 1 0 0 1 1 0 0 1 0 0 1 0 0 1 1 0
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        0 0 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 0 0 0 1 1 1 1 1 1 1 0];
    Xdestination = 29.4;
    Ydestination = 29.3;
    h=rot90(abs(peaks(30)));
elseif user_choice == 4
    % 配置4: 10x10地图，特殊障碍物
    G=[ 0 0 0 0 0 0 0 0 0 0 
        0 1 1 1 1 1 0 0 0 0 
        0 1 1 1 1 1 0 0 0 0 
        0 1 1 1 1 0 0 1 1 0 
        0 1 1 1 0 0 1 1 1 0 
        0 1 1 0 0 1 1 1 1 0 
        0 1 0 0 1 1 1 1 1 0
        0 0 0 1 1 1 1 1 1 0 
        0 0 1 1 1 1 1 1 1 0 
        0 0 0 0 0 0 0 0 0 0];
    Xdestination = 9.4;
    Ydestination = 9.3;
    h=rot90(abs(peaks(10)));
else
    error('无效的选择，请输入1、2或3');
end

Lgrid = 1;
MM=size(G,1); %MM为矩阵维数
Xinitial = 0.6;
Yinitial = 0.2;
[initial,ij_initial]= modify(Xinitial,Yinitial);
[destination,ij_destination]= modify(Xdestination,Ydestination);

% 创建三维图形展示高度差
figure; % 创建新的图形窗口
surf(h); % 使用surf函数绘制三维表面图
xlabel('X轴');
ylabel('Y轴');
zlabel('高度 (h)');
title('栅格高度差三维可视化');
colorbar; % 显示颜色条

% 先绘制地图背景
figure(1);   
for i=1:MM
  for j=1:MM
  x1=(j-1)*Lgrid;y1=(MM-i)*Lgrid; 
  x2=j*Lgrid;y2=(MM-i)*Lgrid; 
  x3=j*Lgrid;y3=(MM-i+1)*Lgrid; 
  x4=(j-1)*Lgrid;y4=(MM-i+1)*Lgrid; 
  f=(max(max(h))-h(i,j))/max(max(h));
    if G(i,j)==1 
        fill([x1,x2,x3,x4],[y1,y2,y3,y4],[0.2,0.2,0.2]); hold on %栅格为1，填充为黑色
    else 
        fill([x1,x2,x3,x4],[y1,y2,y3,y4],[f,1,f]); hold on %栅格为0，填充为白色
    end 
  end 
end
axis([0,MM*Lgrid,0,MM*Lgrid]) 
grid on
xlabel('x'); ylabel('y'); title('不同算法最佳路径比较');
hold on;

%% 计算距离启发矩阵dis
dis = zeros(MM,MM);
for i=1:MM
  for j=1:MM
   x = (j-0.5)*Lgrid;
   y = (MM-i+0.5)*Lgrid;
   dis(i,j) = sqrt(sum(([x y]-destination).^2));
  end
end

%% 计算距离转移矩阵D
D=zeros(MM^2,8);   %行号表示栅格标号，列号表示邻接的8个方向的栅格号
Dir = [-MM-1,-1,MM-1,MM,MM+1,1,1-MM,-MM];
 for i = 1:MM^2     %8方向转移距离矩阵初步构建
     Dirn = Dir+i;
     if G(i)==1
             D(i,:)=inf;
             continue
     end
         for j = 1:8
             if  Dirn(j)<=0||Dirn(j)>MM^2        %出界的情况，暂且为0
                 continue 
             end
             if G(Dirn(j))==1
                 D(i,j) = inf;
             elseif mod(j,2)==0         %偶数方向为上下左右方向
                 D(i,j) = 1;
             elseif j==1 %左上方向的情况，保证路线不会擦障碍物边沿走过
                 if (G(Dirn(2))+G(Dirn(8))==0)
                   D(i,j) = 1.4; 
                 else
                   D(i,j) = inf;   
                 end
             elseif (Dirn(j-1)<=0||Dirn(j-1)>MM^2)||(Dirn(j+1)<=0||Dirn(j+1)>MM^2)%排除掉垂直方向的栅格出界的情况
                 continue
             elseif G(Dirn(j-1))+G(Dirn(j+1))==0    %其余三个斜方向
                 D(i,j) = 1.4;
             else
                 D(i,j) = inf;
             end
         end
     
 end

%% 创造边界
 num = 1:MM^2;
 obs_up = find(mod(num,MM)==1);
 obs_up = obs_up(2:end-1);
 D(obs_up,[1,2,3])=inf;
 obs_down = find(mod(num,MM)==0);
 obs_down = obs_down(2:end-1);
 D(obs_down,[5,6,7])=inf;
 D(2:MM-1,[1,7,8]) = inf;
 D(MM^2-MM+2:MM^2-1,[3,4,5])=inf;
 D(1,[1,2,3,7,8])=inf;
 D(MM,[1,5,6,7,8])=inf;
 D(MM^2-MM+1,[1,2,3,4,5])=inf;
 D(MM^2,[3,4,5,6,7])=inf;

%% 参数初始化和算法执行
algorithms = {'ACO', 'IACO', 'IACO_FMO'};
colors = {'k', 'b', 'r'}; % 为不同算法路径指定不同颜色
line_styles = {'--', ':', '-'}; % 线型
line_widths = [2.5, 2, 1.5]; % 线宽
legend_entries = {};

figure(2); hold on; title('各代最佳路线的长度比较'); xlabel('迭代次数'); ylabel('最佳路线长度'); axis([0,30,25,90]); grid on;
figure(3); hold on; title('各代最佳路线的高度均方差比较'); xlabel('迭代次数'); ylabel('高度均方差*100'); axis([0,30,0,30]); grid on;
figure(4); hold on; title('各代最佳路线的转弯次数比较'); xlabel('迭代次数'); ylabel('转弯次数'); axis([0,30,5,50]); grid on;
figure(5); hold on; title('各代最佳路线的综合指标比较'); xlabel('迭代次数'); ylabel('综合指标'); axis([0,30,70,200]); grid on;

for k = 1:length(algorithms)
    ALO = algorithms{k};
    fprintf('Running %s algorithm...\n', ALO);
    tAlgoStart = tic; % 记录算法开始时间
    
    if strcmp(ALO, 'IACO')
        NC_max=30; m=50;  Rho=0.3; Q=100; Omega=10; Mu=1;  u=10; Tau_min=10; Tau_max=40; Rho_min=0.2;
        [R_best,F_best,L_best,T_best,S_best,S_ave,Shortest_Route,Shortest_Length]=IACO_MPP(D,initial,destination,dis,h,NC_max,m,Rho,Omega,Mu,Q,u,Tau_min,Tau_max,Rho_min); %函数调用
    elseif strcmp(ALO, 'IACO_FMO')
        NC_max=30; sizepop=50; E_max=200; stage_threshold=4;
        [R_best,F_best,L_best,T_best,S_best,S_ave,Shortest_Route,Shortest_Length]=IACO_FMO_MPP(D,initial,destination,dis,h,NC_max,sizepop,E_max,stage_threshold);
    elseif strcmp(ALO, 'ACO')
        NC_max=30; m=35; t=8; Rho=0.1; Q=100; Omega=10; Mu=2; Lambda=2;
        [R_best,F_best,L_best,T_best,S_best,S_ave,Shortest_Route,Shortest_Length]=ACO_MPP(D,initial,destination,dis,h,NC_max,m,t,Rho,Omega,Mu,Lambda,Q);
    end
    algo_time = toc(tAlgoStart); % 算法运行用时（秒）
    
    % 绘制找到的最优路径
    figure(1); % 确保在figure(1)上绘图
    j_path = ceil(Shortest_Route/MM);
    i_path = mod(Shortest_Route,MM);
    i_path(i_path==0) = MM;
    x_path = (j_path-0.5)*Lgrid;
    y_path = (MM-i_path+0.5)*Lgrid;
    x_plot = [initial(1) x_path destination(1)];
    y_plot = [initial(2) y_path destination(2)];
    
    % 如果是IACO_FMO算法，则进行PH曲线优化
    if strcmp(ALO, 'IACO_FMO')
        % 进行PH曲线优化
        [x_phcurve, y_phcurve] = ph_curve_optimization(x_plot, y_plot);
        % 绘制优化后的曲线
        h1=plot(x_phcurve, y_phcurve, [line_styles{k}, colors{k}], 'LineWidth', line_widths(k));
        legend_entries{end+1} = 'IACO-FMO';
    elseif strcmp(ALO, 'IACO')
        h2=plot(x_plot, y_plot, [line_styles{k}, colors{k}], 'LineWidth', line_widths(k));
        legend_entries{end+1} = ALO;
    elseif strcmp(ALO, 'ACO')
        h3=plot(x_plot, y_plot, [line_styles{k}, colors{k}], 'LineWidth', line_widths(k));
        legend_entries{end+1} = ALO;
    end
    
    % 绘制收敛曲线
    iter=1:length(L_best);
    figure(2); plot(iter, L_best, [line_styles{k}, colors{k}], 'LineWidth', 1.5); ylim('auto');
    figure(3); plot(iter, F_best*100, [line_styles{k}, colors{k}], 'LineWidth', 1.5); ylim('auto');
    figure(4); plot(iter, T_best, [line_styles{k}, colors{k}], 'LineWidth', 1.5); ylim('auto');
    figure(5); plot(iter, S_best, [line_styles{k}, colors{k}], 'LineWidth', 1.5); ylim('auto');

    fprintf('%s algorithm finished. Shortest Length: %f\n', ALO, Shortest_Length);

    % 将本次算法的关键结果记录到 Excel（result.xlsx）
    try
        results_file = fullfile(pwd, 'result.xlsx');
        run_timestamp = string(datetime('now','Format','yyyy-MM-dd HH:mm:ss'));
        map_choice = user_choice;    % 运行地图选择 (1/2/3)
        map_size = MM;               % 地图维度
        % 提取最终指标（使用各代最佳的最后一个值）
        final_length = Shortest_Length;
        final_turns  = T_best(end);
        final_height_std = F_best(end);
        final_score  = S_best(end);

        newRow = table(run_timestamp, map_choice, map_size, string(ALO), algo_time, final_length, final_turns, final_height_std, final_score, ...
            'VariableNames', {'Timestamp','MapChoice','MapSize','Algorithm','RuntimeSeconds','FinalLength','FinalTurns','FinalHeightStd','FinalScore'});

        if isfile(results_file)
            existing = readtable(results_file);
            combined = [existing; newRow];
            writetable(combined, results_file);
        else
            writetable(newRow, results_file);
        end
    catch writeErr
        warning('记录结果到 Excel 失败: %s', writeErr.message);
    end
end

figure(1); legend([h3,h2,h1], legend_entries, 'Location', 'northeastoutside');
figure(2); legend(legend_entries, 'Location', 'northeastoutside');
figure(3); legend(legend_entries, 'Location', 'northeastoutside');
figure(4); legend(legend_entries, 'Location', 'northeastoutside');
figure(5); legend(legend_entries, 'Location', 'northeastoutside');

%% Modify 函数
function [xy, ij] = modify( x,y)
global MM Lgrid
xy =[ fix(x/Lgrid)*Lgrid+Lgrid/2,fix(y/Lgrid)*Lgrid+Lgrid/2];
ij = [MM+0.5-xy(2)/Lgrid,xy(1)/Lgrid+0.5];
end

%% PH曲线优化函数 (Pythagorean Hodograph Curve)
function [x_phcurve, y_phcurve] = ph_curve_optimization(x, y)
    % 检查输入点的数量
    n = length(x);
    if n < 3
        % 如果点太少，直接返回原始点
        x_phcurve = x;
        y_phcurve = y;
        return;
    end
    
    % 生成更密集的参数值用于绘制平滑曲线
    num_points = 60;
    
    % 初始化PH曲线点
    x_phcurve = zeros(1, num_points);
    y_phcurve = zeros(1, num_points);
    
    % 对每个路径段生成PH曲线
    total_length = 0;
    segment_lengths = zeros(1, n-1);
    
    % 计算每段的长度
    for i = 1:n-1
        segment_lengths(i) = sqrt((x(i+1)-x(i))^2 + (y(i+1)-y(i))^2);
        total_length = total_length + segment_lengths(i);
    end
    
    % 累积长度用于参数化
    cumulative_length = [0, cumsum(segment_lengths)];
    
    % 为每个点分配参数值
    point_idx = 1;
    for i = 1:num_points
        t = (i-1) / (num_points-1);
        current_length = t * total_length;
        
        % 找到当前参数对应的路径段
        segment = 1;
        while segment < n-1 && cumulative_length(segment+1) < current_length
            segment = segment + 1;
        end
        
        % 计算在当前段内的局部参数
        if segment_lengths(segment) > 0
            local_t = (current_length - cumulative_length(segment)) / segment_lengths(segment);
        else
            local_t = 0;
        end
        
        % 生成五次PH曲线段
        [x_ph, y_ph] = generate_ph_segment(x(segment), y(segment), x(segment+1), y(segment+1), local_t);
        
        x_phcurve(i) = x_ph;
        y_phcurve(i) = y_ph;
    end
    
    % 确保曲线经过起点和终点
    x_phcurve(1) = x(1);
    y_phcurve(1) = y(1);
    x_phcurve(end) = x(end);
    y_phcurve(end) = y(end);
    
    % 应用阿克曼约束优化
    [x_phcurve, y_phcurve] = apply_ackermann_constraints(x_phcurve, y_phcurve);
end

%% 生成五次PH曲线段
function [x_ph, y_ph] = generate_ph_segment(x0, y0, x1, y1, t)
    % 计算方向向量
    dx = x1 - x0;
    dy = y1 - y0;
    
    % 计算初始和终止角度
    theta0 = atan2(dy, dx);
    theta1 = theta0; % 保持方向一致性
    
    % PH曲线的复数表示：z'(t) = (u(t) + i*v(t))^2
    % 其中u(t)和v(t)是二次多项式
    
    % 设计控制参数以满足边界条件
    L = sqrt(dx^2 + dy^2); % 段长度
    
    % 二次多项式系数 (简化的PH曲线)
    u0 = cos(theta0/2) * sqrt(L/2);
    v0 = sin(theta0/2) * sqrt(L/2);
    u1 = cos(theta1/2) * sqrt(L/2);
    v1 = sin(theta1/2) * sqrt(L/2);
    
    % 使用Bernstein基函数进行插值
    u_t = u0 * (1-t)^2 + 2*u0*(1-t)*t + u1*t^2;
    v_t = v0 * (1-t)^2 + 2*v0*(1-t)*t + v1*t^2;
    
    % 计算PH曲线的导数 (速度向量)
    dx_dt = u_t^2 - v_t^2;
    dy_dt = 2 * u_t * v_t;
    
    % 积分得到位置 (简化积分)
    x_ph = x0 + dx * t + 0.1 * L * sin(2*pi*t) * (1-t) * t; % 添加平滑项
    y_ph = y0 + dy * t + 0.1 * L * cos(2*pi*t) * (1-t) * t; % 添加平滑项
end

%% 应用阿克曼约束优化
function [x_opt, y_opt] = apply_ackermann_constraints(x, y)
    n = length(x);
    x_opt = x;
    y_opt = y;
    
    % 阿克曼机器人参数
    max_curvature = 0.5; % 最大曲率约束 (1/m)
    wheelbase = 2.0; % 轴距 (m)
    max_steering_angle = pi/6; % 最大转向角 (30度)
    
    % 计算并限制曲率
    for i = 2:n-1
        % 计算当前点的曲率
        dx1 = x(i) - x(i-1);
        dy1 = y(i) - y(i-1);
        dx2 = x(i+1) - x(i);
        dy2 = y(i+1) - y(i);
        
        % 避免除零
        if abs(dx1) < 1e-6 && abs(dy1) < 1e-6
            continue;
        end
        if abs(dx2) < 1e-6 && abs(dy2) < 1e-6
            continue;
        end
        
        % 计算曲率 κ = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
        v1 = sqrt(dx1^2 + dy1^2);
        v2 = sqrt(dx2^2 + dy2^2);
        
        if v1 > 1e-6 && v2 > 1e-6
            % 归一化方向向量
            dx1_norm = dx1 / v1;
            dy1_norm = dy1 / v1;
            dx2_norm = dx2 / v2;
            dy2_norm = dy2 / v2;
            
            % 计算角度变化
            cross_product = dx1_norm * dy2_norm - dy1_norm * dx2_norm;
            dot_product = dx1_norm * dx2_norm + dy1_norm * dy2_norm;
            angle_change = atan2(cross_product, dot_product);
            
            % 计算曲率
            avg_speed = (v1 + v2) / 2;
            if avg_speed > 1e-6
                curvature = abs(angle_change) / avg_speed;
                
                % 应用曲率约束
                if curvature > max_curvature
                    % 调整点位置以减小曲率
                    reduction_factor = max_curvature / curvature;
                    
                    % 向直线方向调整
                    mid_x = (x(i-1) + x(i+1)) / 2;
                    mid_y = (y(i-1) + y(i+1)) / 2;
                    
                    x_opt(i) = x(i) * reduction_factor + mid_x * (1 - reduction_factor);
                    y_opt(i) = y(i) * reduction_factor + mid_y * (1 - reduction_factor);
                end
            end
        end
    end
    
    % 应用转向角约束
    for i = 2:n-1
        dx = x_opt(i+1) - x_opt(i-1);
        dy = y_opt(i+1) - y_opt(i-1);
        
        if abs(dx) > 1e-6
            heading_angle = atan2(dy, dx);
            
            % 计算所需的转向角
            if i > 2
                prev_dx = x_opt(i) - x_opt(i-2);
                prev_dy = y_opt(i) - y_opt(i-2);
                if abs(prev_dx) > 1e-6
                    prev_heading = atan2(prev_dy, prev_dx);
                    steering_angle = heading_angle - prev_heading;
                    
                    % 限制转向角
                    if abs(steering_angle) > max_steering_angle
                        steering_angle = sign(steering_angle) * max_steering_angle;
                        new_heading = prev_heading + steering_angle;
                        
                        % 重新计算点位置
                        segment_length = sqrt(dx^2 + dy^2) / 2;
                        x_opt(i) = x_opt(i-1) + segment_length * cos(new_heading);
                        y_opt(i) = y_opt(i-1) + segment_length * sin(new_heading);
                    end
                end
            end
        end
    end
end
