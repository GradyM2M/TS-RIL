%==================================== Get Neighbours ====================================
% 输入参数：RRT，区域内节点下标，邻近节点下标，最新节点
% 输出参数：区域内节点下标
%
% 描述：在临近区域内节点中查找到最新节点开销最小的节点，以作为新节点的母节点；
%
% 说明：
%      1. 至少返回邻近节点下标，即区域内无可行点时也返回临近节点；
%
%========================================================================================
%%
function parentIndex = chooseParent(vertex_param, neighboursIndex, nearVertexIndex, newVertex)

    neighboursNum = length(neighboursIndex);       % 获取区域内节点个数
    parentIndex = nearVertexIndex;      % 初始化输出
    
    minCost = getCostFroomRoot(vertex_param, nearVertexIndex, newVertex);
    
    for i = 1 : neighboursNum
        newCost = getCostFroomRoot(vertex_param, neighboursIndex(i), newVertex);        % 逐个计算最新点与区域内节点的开销
        
        if(minCost > newCost)     % 选取最小开销母节点（与其它子节点不接近）
            parentIndex = neighboursIndex(i);
            minCost = newCost;
        end 
    end


    