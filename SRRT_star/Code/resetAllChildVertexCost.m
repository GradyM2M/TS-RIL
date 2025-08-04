%============================== reset All Child Vertex Cost ==============================
% 输入参数：RRT，母节点下标，开销变化量
% 输出参数：RRT
%
% 描述：修改与指定节点以及其所有相关子节点（子节点的子节点...）的开销；
%
% 说明：
%      1. 修改当前母节点以及与此母节点相通的所有子节点的开销；
%      2. 迭代法不适用，因为内存限制，子节点过多会导致内存溢出，需结合队列进行遍历；
%
%=========================================================================================
%%
function vertex_param = resetAllChildVertexCost(vertex_param, parentVertexIndex, detCost)

    % 初始化记录节点下标的队列
    vertexIndexList = parentVertexIndex;
    listStart = 1;      % 队列首指针
    listEnd = 1;        % 队列后指针
    
    listNumMax = length(vertex_param);
    
    while(listStart <= listEnd)
        
        last = listStart;
        
        newVertexIndex = vertexIndexList(listStart);        % 获取最新出队的节点下标
        
        % 获取子节点个数（队尾插入操作）
        childVertexNum = length(vertex_param(newVertexIndex).obstacleFreeIndex);
        if(childVertexNum > 0)
            vertexIndexList = [vertexIndexList, vertex_param(newVertexIndex).obstacleFreeIndex'];       % 保存子节点下标（需转置）
            listEnd = listEnd + childVertexNum;     % 更新队列后指针
        end
    
        % 更新当前节点开销（队前出队操作）
        vertex_param(newVertexIndex).cost = vertex_param(newVertexIndex).cost + detCost;
        listStart = listStart + 1;      % 更新队列前指针
        
        if(listStart > listNumMax)
            error('Error: resetAllChildVertexCost!')
        end
    end

    
    
    
    

    