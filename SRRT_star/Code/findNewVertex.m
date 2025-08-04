%=================================== find New Vertex ====================================
% 输入参数：临近节点，随机节点，步长
% 节点参数：最新点
%
% 描述：计算新节点的坐标；
%
% 说明：
%     1. 输出新节点坐标为整数；
%
%========================================================================================
%%
function newVertex = findNewVertex(nearVertex, randVertex, expandDis)

    v = randVertex - nearVertex;        % 计算矢量
    
    u = v ./ norm(v);        % 计算单位矢量，即方向矢量
    
    % 随机节点和最近节点重合
    if(norm(v) == 0)
        error('RRT：randVertex == nearVertex');
    end
    
    newVertex = floor(nearVertex + expandDis .* u);      % 计算新节点（取整）

    
    