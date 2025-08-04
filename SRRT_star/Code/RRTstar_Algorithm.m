%======================================= RRT* Main =======================================
% 输入参数：3D 栅格地图，RRT参数，初始点，目标点，动态参数
% 输出参数：标志位，整个RRT，最小开销路径
%
% 描述：进行迭代生成 RRT，当搜索到可行路径或达到最大迭代次数时结束，输出搜索结果
%
% 说明：
%      1. 地图采用 3D 栅格地图，用二维矩阵表示，行表示 x 轴，列表示 y 轴；
%      2. 将计算结果取整以在地图中表示；
%      3. 标志位：0/1 表示 无/有可行路径；
%
%============================================================================================
%%
function [flag, vertex_param, optimalPath, minCost] = RRTstar_Algorithm(map, RRT_param, startVertex, goalVertex, dynamic_param)

    % 动态调节参数
    if(dynamic_param.isAdjustScale == 1 || dynamic_param.isAdjustScale == 3)
        u = RRT_param.expandDis;
        R = RRT_param.regionRadius;
        p = RRT_param.goalTrend;
    end

    % 计算地图自由区域体积
    [~, obstacleScale] = getObstacleScale(map, [1, 1, 1], 5000);
    mapSize = tsize(map);
    mapFreeVolume = mapSize(1) * mapSize(2) * mapSize(3) * (1 - obstacleScale);
    rd = 10 * mapFreeVolume / pi;      % 计算局部优化区域半径

    % 定义扩展节点参数
    expandVertex = struct('rand', [], 'near', startVertex, 'new', []);
    
    % 初始化节点参数
    vertex_param = struct('vertex', startVertex, 'parentIndex', 0, 'cost', 0,...
                   'obstacleFreeIndex', [], 'isGetGoal', 0);
    
    pathEndVertex = vertex_param;       % 记录最短路径末端节点
    
    % 计算各区域障碍物比例及参数缩放比例
    if(dynamic_param.isAdjustScale > 0)
        [regionLengthXYZ, obstacleScale] = getObstacleScale(map, dynamic_param.regionNum, dynamic_param.sampleNum);
        if(dynamic_param.isAdjustScale == 2)      % 各区域标准比例
            standardScale = ones(size(obstacleScale));
        else      % 根据障碍物比例和预设比例范围计算各区域标准比例
            standardScale = reviseObstacleScale(obstacleScale, dynamic_param.scaleLim, dynamic_param.midScale);         % 将障碍物比例转换成参数变化的比例    
        end
    end
    
    % 计算各区域采样概率
    if(dynamic_param.isAdjustScale == 2 || dynamic_param.isAdjustScale == 3)
        regionSize = tsize(obstacleScale);
        regionVertexNum = zeros(regionSize);       % 初始化每个区域节点个数矩阵
        
        % 计算目标节点所在区域行列号
        indexI = min([regionSize(1), floor(goalVertex(1) / regionLengthXYZ(1)) + 1]);
        indexJ = min([regionSize(2), floor(goalVertex(2) / regionLengthXYZ(2)) + 1]);
        indexK = min([regionSize(3), floor(goalVertex(2) / regionLengthXYZ(3)) + 1]);
        goalXYZ = [indexI, indexJ, indexK];
        
        % 计算每个区域标准节点数(基于自由区域体积和节点最小距离)
        standardVertexNum = calRegionStandardVertexNum(regionLengthXYZ, obstacleScale, standardScale, RRT_param.minDistance);
        
        % 初始化区域采样概率
        [sampleScale, regionVertexNum] = updateRegionSampleScale([], regionLengthXYZ, regionVertexNum, standardVertexNum,...
                                              startVertex, dynamic_param.pdfParam, goalXYZ, RRT_param.goalTrend, RRT_param.goalScaleK);
    end

    k = 1;      % 记录节点个数（第一点为起始节点）
    flag = 0;       % 标志位：默认无可行路径
    
    percent = 0.05;
    % 进行迭代（起始点为第一点）
%-------------------------------------------------
%---------------------- 迭代 ---------------------
%-------------------------------------------------
    for i = 1 : RRT_param.iterativeNum
        
        % 显示进程
        tmp = i / RRT_param.iterativeNum;
        if(tmp > percent)
            fprintf('Process: %0.2f \nVertex Number: %d \r\n', percent, length(vertex_param));
            if(percent < 0.29)
                percent = percent + 0.05;
            else
                if(percent < 0.49)
                    percent = percent + 0.02;
                else
                    percent = percent + 0.01;
                end
            end
        end

        % 随机节点采样
        if(dynamic_param.isAdjustScale == 2 || dynamic_param.isAdjustScale == 3)        % 分区域随机采样
            expandVertex.rand = findRandVertex(mapSize, goalVertex, RRT_param.goalTrend, regionLengthXYZ, sampleScale);
        else        % 整体随机采样
            expandVertex.rand = findRandVertex(mapSize, goalVertex, RRT_param.goalTrend, [], []);
        end
        
        % 查找最近点
        nearVertexIndex = findNearVertex(vertex_param, expandVertex.rand);
        
        % 确定最近节点
        expandVertex.near = vertex_param(nearVertexIndex).vertex;
        
        % 动态调节参数
        if(dynamic_param.isAdjustScale == 1 || dynamic_param.isAdjustScale == 3)
            % 获取最近点所在区域
            x = min([dynamic_param.regionNum(1), floor(expandVertex.near(1) / regionLengthXYZ(1)) + 1]);
            y = min([dynamic_param.regionNum(2), floor(expandVertex.near(2) / regionLengthXYZ(2)) + 1]);
            z = min([dynamic_param.regionNum(3), floor(expandVertex.near(3) / regionLengthXYZ(3)) + 1]);

            scale = standardScale(x, y, z);

            % 根据障碍物比例修正参数
            RRT_param.expandDis = u * scale;
            RRT_param.regionRadius = R * scale;
            RRT_param.goalTrend = p * scale;
        end
        
        % 计算邻域半径
        n = length(vertex_param);
        rn = min([(rd * log(n) / n)^(1 / 3), RRT_param.regionRadius]);

        % 确定最新节点
        expandVertex.new = findNewVertex(expandVertex.near, expandVertex.rand, RRT_param.expandDis);
        
        % 搜索指定区域内节点（无效点也进行检测，很浪费时间，应修正）
        neighboursIndex = getNeighbours(map, vertex_param, expandVertex.new, rn);
        
        % 确定最新节点的母节点（更新最近节点下标）
        nearVertexIndex = chooseParent(vertex_param, neighboursIndex, nearVertexIndex, expandVertex.new);
            
        %----------------------------------------------------
        % 碰撞检测
        tmp = isObstacleFree(map, vertex_param(nearVertexIndex).vertex, expandVertex.new);
        if(tmp < 1)       % 超出地图或碰撞
            expandVertex.new = [];      % 新节点设为空，用于 区域采样比例更新
        else
            if(tmp > 0)       % 非碰撞：保存非碰撞子节点下标，添加节点，计算开销

                isGoal = isGetGoal(map, expandVertex.new, goalVertex, RRT_param.expandDis);      % 是否到达目标点
                if(isGoal == 0 || isGoal == 1)     % 非目标点

                    % 计算开销（链接邻近节点的最新节点开销）
                    newCost = getCostFroomRoot(vertex_param, nearVertexIndex, expandVertex.new);    

                    k = k + 1;

                    vertex_param(nearVertexIndex).obstacleFreeIndex = [vertex_param(nearVertexIndex).obstacleFreeIndex; k];       % 保存非碰撞子节点下标
                    vertex_param(k) = struct('vertex', expandVertex.new, 'parentIndex', nearVertexIndex, 'cost', newCost,...
                                      'obstacleFreeIndex', [], 'isGetGoal', 0);

                    % 若区域内节点大于1（除去最近点），则重新连接区域内节点
                    if(length(neighboursIndex) > 1)
                        vertex_param = rewire(vertex_param, neighboursIndex, k); 
                    end
                end

                if(isGoal == 1)     % 可到达目标点

                    % 计算开销（在前面链接邻近点和最新点的基础上，链接目标点和最新点，并计算目标点的开销）
                    newCost = getCostFroomRoot(vertex_param, k, goalVertex);       

                    k = k + 1;

                    vertex_param(k - 1).obstacleFreeIndex = [vertex_param(k - 1).obstacleFreeIndex; k];       % 保存非碰撞子节点下标
                    vertex_param(k) = struct('vertex', goalVertex, 'parentIndex', k - 1, 'cost', newCost,...
                                      'obstacleFreeIndex', [], 'isGetGoal', 1);
                else
                    if(isGoal == 2)     % 为目标点

                        % 计算开销（最新点改为目标点，则链接邻近节点和目标节点，并计算目标节点的开销）
                        newCost = getCostFroomRoot(vertex_param, nearVertexIndex, goalVertex); 

                        k = k + 1;

                        vertex_param(nearVertexIndex).obstacleFreeIndex = [vertex_param(nearVertexIndex).obstacleFreeIndex; k];       % 保存非碰撞子节点下标
                        vertex_param(k) = struct('vertex', goalVertex, 'parentIndex', nearVertexIndex, 'cost', newCost,...
                                          'obstacleFreeIndex', [], 'isGetGoal', 1);
                    end
                end

                if(isGoal > 0)
                    if(pathEndVertex.cost == 0 || pathEndVertex.cost > newCost)
                        pathEndVertex = vertex_param(k);        % 保存最短路径
                        flag = 1;
                        break;
                    end
                end
            end
        end
        %----------------------------------------------------
        
        if(dynamic_param.isAdjustScale == 2 || dynamic_param.isAdjustScale == 3)
            % 更新采样区域采样比例（除去目标趋向概率）
            [sampleScale, regionVertexNum] = updateRegionSampleScale(sampleScale, regionLengthXYZ, regionVertexNum, standardVertexNum,...
                                                  expandVertex.new, dynamic_param.pdfParam, goalXYZ, RRT_param.goalTrend, RRT_param.goalScaleK);
        end
    end
    
    i = 1;
    k = pathEndVertex.parentIndex;      % 记录倒数第二点下标
    optimalPath =  pathEndVertex;        % 记录最后一点
    minCost = pathEndVertex.cost;       % 输出最小开销
    
    % 提取最短路径
    if(flag == 1)
        while(k ~= 1)       
            i = i + 1;
            optimalPath(i) =  vertex_param(k);      % 逐个记录路径点
            k = vertex_param(k).parentIndex;      % 逐个记录节点的母节点
        end
        optimalPath(i + 1) = vertex_param(1);      % 添加首点
    end
    
    % 将路径点顺序还原
    pathlength = length(optimalPath);
    for i = 1 : floor(pathlength / 2)
        tmp = optimalPath(i);
        optimalPath(i) = optimalPath(pathlength - i + 1);
        optimalPath(pathlength - i + 1) = tmp;
    end
    
    if(dynamic_param.isAdjustScale == 2 || dynamic_param.isAdjustScale == 3)
        standardVertexNum

        regionVertexNum

        sampleScale
    end

    
