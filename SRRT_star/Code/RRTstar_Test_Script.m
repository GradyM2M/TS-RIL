%=================================== RRTstar Test Script ===================================
% 定义参数：迭代次数，扩展步长，目标趋向性，初始节点
% 节点参数：当前节点位置，母节点小标，开销，非碰撞子节点下标，碰撞子节点下标，是否到达目标
%
% 描述：加载地图，定义初始和目标节点，进行 RRTstar，完成后绘制最终 RRT，并输出结果
%
% 说明：
%      1. 地图采用 2D 栅格地图，用二维矩阵表示，行表示 x 轴，列表示 y 轴；
%      2. 将计算结果取整以在地图中表示；
%
%==========================================================================================
%%

clc
clear all

disp('RRT* Algorithm is executing...')

% 加载 3D 栅格地图
mapData = load('threeDimMap01');
mapSize = size(mapData.threeDimMap);    % 获取地图大小

% plot3DMap(mapData.threeDimMap, mapData.obstacleStartVertex, mapData.obstacleLenghtXYZ);
% 
% return;

% ----------------------------------------------------------------
% -------------------------- 参数设置 -----------------------------
% ----------------------------------------------------------------
% 定义起始点和目标点
% newMap01
start_vertex = [25, 50, 25];
goal_vertex = [375, 250, 75];


K = 4000;
u = 20;
p = 0;
minDis = 2 * u * sin(pi / 8);      % 只有 type = 2/3 时有效
R = 2 * u;

rN = [4, 3, 2];        % 划分区间行列个数
sN = 2000;      % 每个区间采样个数
lim = [0.6, 1.4];       % 参数缩放比例区间
mid = 1;       % 参数变化原始值所处比例
type = 3;       % 是否动态更改参数[0-3: 原始方法；改进1：增加区域障碍物比例；改进2：增加区域采样概率；综合改进1&2]

k = 1;      % 目标区域最小采样比例系数（区域平均采样概率的倍数）
pdf = [0, 1];       % 正态分布参数

global obstacleNum
maxTimes = 20;
% 循环指定次数
for T = 1 : maxTimes
    
    obstacleNum = 0;

    % ----------------------------------------------------------------
    % ------------------------- RRT 扩展过程 --------------------------
    % ----------------------------------------------------------------

    tic;        % Start the time

    % 定义 RRT 参数
    RRT_param = struct('iterativeNum', K, 'expandDis', u, 'goalTrend', p, 'minDistance', minDis, 'regionRadius', R, 'goalScaleK', k);

    % 定义动态设置参数
    dynamic_param = struct('regionNum', rN, 'sampleNum', sN, 'scaleLim', lim, 'midScale', mid, 'pdfParam', pdf, 'isAdjustScale', type);

    [flag, vertex_param, optimalPath, minCost] = RRTstar_Algorithm(mapData.threeDimMap, RRT_param, start_vertex, goal_vertex, dynamic_param);

    elapsedTime = toc;        % End the time

    % 绘制图像
    if(T == maxTimes)
        RRT_Plot(mapData, start_vertex, goal_vertex, vertex_param, optimalPath);
    end

    vertexNum = length(vertex_param);        % 总节点个数

    % 结果输出
    disp('------------- Result -------------');
    fprintf('elapsedTime: %0.3f sec\n', elapsedTime);       % 程序执行时间
    fprintf('vertexNum:   %d \n', vertexNum);          % 总节点个数节点
    fprintf('minPathCost: %0.3f \n', minCost);         % 最短路径开销
    fprintf('obstacleNum: %d \n', obstacleNum);        % 碰撞检测次数
    disp('----------------------------------');

    if(flag == 0)       % 未找到可行路径
        return;     % 返回，不存储数据
    end

    % ----------------------------------------------------------------
    % ---------------------- 多组数据存储及处理 ------------------------
    % ----------------------------------------------------------------
    % 文件属性
    filename1 = sprintf('.\\dataFiles\\experimentalData0%d.txt', type);
    filename2 = '.\\dataFiles\\experimentalDataAverage.txt';
    % filename3 = '.\\dataFiles\\experimentalPath.txt';
    dataGroupNumMax = 10;
    
    if(T == 1)
        isDeleteOldData = 1;        % 需手动更新，否则一直记录数据（清除数据后，需手动调回）
    else
        isDeleteOldData = 0;
    end
    
    % 存入数据
    saveData([elapsedTime, vertexNum, minCost, obstacleNum], filename1, isDeleteOldData);

    % pathNum = length(optimalPath);
    % pathXYZ = zeros(3, pathNum);
    % for i = 1 : pathNum
    %     pathXYZ(:, i) = optimalPath(i).vertex';
    % end
    % saveData(pathXYZ', filename3, 1);

    % 读出数据
    data = extractData(filename1, 4);

    % 数据处理
    dataSize = size(data);  
    dataAverage = calAverage(data, 1);      % 对每组数据求取平均值（每列的平均值：除去最后一列标志）
    tmp = extractData(filename2, 4);
    if(isempty(tmp))
        tmp = zeros(4, dataSize(2));        % 初始化数据
    end
    tmp(type + 1, :) = dataAverage;     % 更新重新计算的平均值
    saveData(tmp, filename2, 1);        % 更新文件数据

    % 提示实验类型和次数
    fprintf('Type-%d: This is the %d-th (Max: %d) experiment.\n',type, dataSize(1), dataGroupNumMax);

    % 达到预设次数后，每次都进行提醒，数据照常更新
    if(dataSize(1) >= dataGroupNumMax)      % 达到最大数据组数，进行数据处理（多余实验数据照常存储
        % 结果输出
        disp('----------- All Result -----------');
        fprintf('Average elapsedTime: %0.3f sec\n', dataAverage(1));       % 程序执行时间
        fprintf('Average vertexNum:   %d \n', round(dataAverage(2)));          % 总节点个数节点
        fprintf('Average minPathCost: %0.3f \n', dataAverage(3));         % 最短路径开销
        fprintf('Average obstacleNum: %d \n', round(dataAverage(4)));        % 碰撞检测次数
        disp('----------------------------------');
    end
    
end



