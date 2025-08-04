%======================================= RRT* Main =======================================
% 输入参数：2D 栅格地图，节点参数，临近节点下标，新节点，扩展步长，节点最小间距
% 输出参数：RRT参数，修改后的新节点
%
% 描述：若新节点与临近点不连通，则将新节点绕临近旋转，直至处于无碰撞区域；
%       旋转过程中同时记录碰撞的节点
%
% 说明：
%      1. 地图采用 2D 栅格地图，用二维矩阵表示，行表示 x 轴，列表示 y 轴；
%      2. 将计算结果取整以在地图中表示；
%      3. 标志位：0/1 表示 无/有可行路径；
%
%============================================================================================
%%
function [vertexParam, newVertex] = adjustNewVertex(map, vertexParam, nearVertexIndex, newVertex, expandDis, minDis)   

    % 若新节点无碰撞，则返回
    if(isObstacleFree(map, vertexParam(nearVertexIndex).vertex, newVertex))
        return;
    end
    
    % 若新节点发生碰撞，则进行调节
    vertexParam(nearVertexIndex).obstacleVertex = [vertexParam(nearVertexIndex).obstacleVertex; newVertex];       % 记录碰撞子节点
    rotateAngle = asin(minDis / (2 * expandDis));       % 计算旋转角度
    u = newVertex - vertexParam(nearVertexIndex).vertex;
    initAngle = atan2(u(2), u(1));      % 计算新节点与邻近点的连线角度
    
    % 旋转新节点,直到不发生碰撞
    for det = rotateAngle : rotateAngle : 2 * pi - rotateAngle
        theta = det + initAngle;
        newVertex(1) = vertexParam(nearVertexIndex).vertex(1) + floor(expandDis * cos(theta));
        newVertex(2) = vertexParam(nearVertexIndex).vertex(2) + floor(expandDis * sin(theta));
        
        % 发生碰撞，记录碰撞子节点
        if(isClose(vertexParam, nearVertexIndex, newVertex, minDis) == 0)   % 若与现有节点距离较远，则记录碰撞子节点
            if(isObstacleFree(map, vertexParam(nearVertexIndex).vertex, newVertex) == 0)   
                vertexParam(nearVertexIndex).obstacleVertex = [vertexParam(nearVertexIndex).obstacleVertex; newVertex];       % 记录碰撞子节点
            else        % 没发生碰撞，返回
                return;
            end
        end
    end
    
    
        
        
   