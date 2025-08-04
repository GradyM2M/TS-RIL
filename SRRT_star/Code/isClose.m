%=================================== is Close ===================================
% 输入参数：RRT，临近节点下标，最新节点，最小距离
% 节点参数：标志位
%
% 描述：判断新节点是否与最近节点（新节点的母节点）的子节点距离过近；
%
% 说明：
%     1. 为防止子节点过多限制扩展效率，将距离过近的子节点删除；
%     2. 标志位： 0/1 表示 不接近/接近；
%     3. 最小距离为零时，不进行检测；
%
%========================================================================================
%%
function flag = isClose(vertex_param, nearVertexIndex, newVertex, minDis)

    % 获取非碰撞和碰撞子节点个数
    obstacleFreeVertex = vertex_param(nearVertexIndex).obstacleFreeIndex;
    obstacleFreeNum = length(obstacleFreeVertex);
    
    obstacleVertex = vertex_param(nearVertexIndex).obstacleVertex;
    obstacleNum = length(obstacleVertex(:, 1));

    % 非碰撞子节点检测（最小距离大于零时，进行检测）
    if(obstacleFreeNum > 0 && minDis > 0) 
        for i = 1 : obstacleFreeNum
            if(pdist2(vertex_param(obstacleFreeVertex(i)).vertex, newVertex) < minDis)
                flag = 1;       % 接近非碰撞子节点
                return;     % 返回
            end
        end
    end
    
    % 碰撞子节点检测
    if(obstacleNum > 0 && minDis > 0)
        for i = 1 : obstacleNum
            if(pdist2(obstacleVertex(i, :), newVertex) < minDis)
                flag = 1;       % 接近非碰撞子节点
                return;     % 返回
            end
        end
    end
    
    flag = 0;       % 不接近所有子节点
    
    
        
        
    