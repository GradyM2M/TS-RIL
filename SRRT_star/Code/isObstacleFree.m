%=================================== is Obstacle Free ====================================
% 输入参数：2D 栅格地图，临近节点，新节点
% 节点参数：标志位
%
% 描述：判断两点是否可连通，即两点间不存在障碍物；
%
% 说明：
%     1. 标志位：-1/0/1 表示 超出地图/有碰撞/无碰撞；
%     2. 一栅格为间隔进行检测；
%     3. 超出地图当障碍物处理；
%
%==========================================================================================
%%
function flag = isObstacleFree(map, nearVertex, newVertex)
    
    global obstacleNum

    obstacleNum = obstacleNum + 1;      % 统计障碍检测次数

    mapSize = tsize(map);        % 获取地图大小

    % 检测新节点是否在地图内
    if(newVertex(1) > mapSize(1) || newVertex(1) < 1 || newVertex(2) > mapSize(2)|| newVertex(2) < 1 || newVertex(3) > mapSize(3) || newVertex(3) < 1)      % 超出地图
        flag = -1;       % 超出地图
        return;     % 返回
    end

    % 检测新节点是否在障碍物空间
    if(map(newVertex(1), newVertex(2), newVertex(3)) == 1)
        flag = 0;       % 检测到碰撞
        return;     % 返回
    end

    v = newVertex - nearVertex;        % 计算矢量
    
    totalDistance = norm(v);
    
    if(totalDistance == 0)      % 总距离为零
        flag = 1;       % 无碰撞
        return;
    end
    
    u = v ./ totalDistance;        % 计算单位矢量，即方向矢量
    
    for i = 1 : totalDistance
        currentCoordinate = nearVertex + (i .* u);     % 计算中间的坐标
        
        if(map(floor(currentCoordinate(1)), floor(currentCoordinate(2)), floor(currentCoordinate(3))) == 1)      % 中间的是障碍物
            flag = 0;       % 检测到碰撞
            return;     % 返回
        end
    end
         
    flag = 1;       % 无碰撞
    
    
    