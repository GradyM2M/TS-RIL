%==================================== Rewire ====================================
% 输入参数：RRT，区域内节点下标，最新节点下标
% 输出参数：区域内节点下标
%
% 描述：将临近区域内的节点逐个尝试将新节点作为母节点，进行重新连接，以减少各节点的开销；
%
% 说明：
%      1. 最新节点的母节点不进行重链接；
%
%========================================================================================
%%
function vertex_param = rewire(vertex_param, neighboursIndex, newVertexIndex)

    neighboursNum = length(neighboursIndex);       % 获取区域内节点个数
    
    for i = 1 : neighboursNum
        newCost = getCostFroomRoot(vertex_param, newVertexIndex, vertex_param(neighboursIndex(i)).vertex);        % 逐个计算最新点与区域内节点的开销
        detCost = newCost - vertex_param(neighboursIndex(i)).cost;
        
        if(detCost < 0)     % 开销降低，重新连接
            
            % 将区域内点母节点更新为最新节点
            parentIndex = vertex_param(neighboursIndex(i)).parentIndex;     % 保存母节点下标（母节点至少有一个子节点：即此节点）
            vertex_param(neighboursIndex(i)).parentIndex = newVertexIndex;       % 更新母节点下标    
           
            % 最新节点子节点扩充
            vertex_param(newVertexIndex).obstacleFreeIndex = [vertex_param(newVertexIndex).obstacleFreeIndex; neighboursIndex(i)];
            
            childVertexNum = length(vertex_param(parentIndex).obstacleFreeIndex);       % 获取区域内节点的母节点的子节点个数
            childVertexIndex = zeros(childVertexNum - 1, 1);        % 列向量：初始化区域内节点的母节点的子节点下标矩阵（若只有一个子节点，则为空矩阵）
            
            k = 1;
            for j = 1 : childVertexNum
                if(vertex_param(parentIndex).obstacleFreeIndex(j) ~= neighboursIndex(i))        % 删去子节点下标
                    childVertexIndex(k) = vertex_param(parentIndex).obstacleFreeIndex(j);
                    k = k + 1;
                end
            end
            vertex_param(parentIndex).obstacleFreeIndex = childVertexIndex;     % 更新区域内节点的母节点的子节点下标矩阵
            
            if(childVertexNum < 1 || k ~= childVertexNum)
                error('Error: childVertexNum error!')
            end
            
            % 更新开销（修改当前节点以及与此节点相通的所有子节点的开销）
            vertex_param = resetAllChildVertexCost(vertex_param, neighboursIndex(i), detCost);
        end
        
    end


    