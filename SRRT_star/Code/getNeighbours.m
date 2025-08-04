%==================================== Get Neighbours ====================================
% 输入参数：地图，RRT，最新节点，区域半径
% 输出参数：区域内节点下标
%
% 描述：在整个 RRT 中搜索最新节点指定半径区域内无碰撞的节点下标；
%
% 说明：
%      1. 区域内无节点时，返回空矩阵；
%      2. 区域内节点与最新节点无碰撞
%
%========================================================================================
%%
function neighboursIndex = getNeighbours(map, vertex_param, newVertex, regionRadius)

    vertexNum = length(vertex_param);       % 获取节点个数
    neighboursIndex = [];
    
    % 必须保证不碰撞
    for i = 1 : vertexNum
        if(pdist2(vertex_param(i).vertex, newVertex) < regionRadius)            % 若在区域内
            if(isObstacleFree(map, vertex_param(i).vertex, newVertex))      % 若无碰撞
                neighboursIndex = [neighboursIndex, i];     % 添加区域内节点下标
            end
        end
    end


    