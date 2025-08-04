%=================================== find Near Vertex ===================================
% 输入参数：整个RRT，随机节点
% 节点参数：最近节点下标
%
% 描述：查找跟随机节点的最近节点；
%
% 说明：
%     1. 两点重合时，不满足要求，故省去；
%
%========================================================================================
%%
function minDistanceIndex = findNearVertex(vertex_param, randVertex)

    vertexNum = length(vertex_param);       % 获取节点个数
    
    if(vertexNum < 1)
        error('RRT: The near vertex not find.');
    end
    
    euclideanDistances = zeros(1, vertexNum);
    
    minDistanceIndex = 0;       % 初始化输出
    
    % 逐个查找
    for i = 1 : vertexNum
        euclideanDistances(i) = pdist2(randVertex, vertex_param(i).vertex, 'euclidean');       % 计算随机采样点跟每个节点的距离
        
        if(minDistanceIndex == 0 && euclideanDistances(i) > 0)      % 初始化最小距离（非零）
            minDistanceIndex = i;
        end
            
        if(euclideanDistances(i) > 0 && euclideanDistances(i) < euclideanDistances(minDistanceIndex))       % 记录最小点下标，且除去距离为零的点
            minDistanceIndex = i;
        end
    end
    
    % 距离全为零，报错
    if(minDistanceIndex == 0)    
        error('RRT: The near vertex not find.');
    end


