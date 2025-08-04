%=================================== Optimize Path ====================================
% 输入参数：2D 栅格地图，路径节点参数
% 节点参数：优化后的路径，开销
%
% 描述：利用贪心算法，优化路径：减少路径拐点，缩短路径；
%
% 说明：
%     1. 将路径上的节点重新链接，以缩短路径；
%
%========================================================================================
%%
function [pathVertex, cost] = optimizePath(map, path)

    vertexNum = length(path);       % 获取节点个数
    
    pathVertex = path(1).vertex;        % 初始化输出
    
    k = 1;      % 记录优化路径点个数
    s = 2;      % 记录每次查找起点
    cost = 0;       % 初始化开销
    while(s < vertexNum)        % 直到 s 指向倒数第二点或最后一点
        for i = s + 1 : vertexNum
            if(isObstacleFree(map, pathVertex(k, :), path(i).vertex) == 0 || i == vertexNum)         % 检测到碰撞或到最后一点
                if(isObstacleFree(map, pathVertex(k, :), path(i).vertex) == 0)
                    s = i - 1;      % 记录碰撞的前一点
                else
                    s = i;      % 检测到最后一点且无碰撞，则保存最后一点
                end
                
                cost = cost + pdist2(pathVertex(k, :), path(s).vertex);        % 计算开销
                
                k = k + 1;      % 优化路径点个数递增
                pathVertex = [pathVertex; path(s).vertex];      % 保存碰撞的前一点
            end
        end
    end
            
    
    