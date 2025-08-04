%======================================= RRT Plot =======================================
% 输入参数：2D 栅格地图，初始节点，目标节点，整个RRT，最优路径
%
% 描述：绘制地图，RRT以及可行路径；
%
% 说明：
%      1. 地图采用 2D 栅格地图，用二维矩阵表示，行表示 x 轴，列表示 y 轴；
%
%========================================================================================
%%
function RRT_Plot(mapData, startVertex, goalVertex, vertex_param, optimalPath)

    RRTVertexNum = length(vertex_param);       % RRT节点个数
    pathVertexNum = length(optimalPath);       % 路径节点个数
    
    % 绘制 2D 栅格地图
    plot3DMap(mapData.threeDimMap, mapData.obstacleStartVertex, mapData.obstacleLenghtXYZ);
    hold on
    
    % 绘制初始节点和末端节点
    plot3([startVertex(1), goalVertex(1)], [startVertex(2), goalVertex(2)], [startVertex(3), goalVertex(3)], 'r*', 'LineWidth', 2, 'Markersize', 10);
    hold on

    % 绘制 RRT
    for i = 1 : RRTVertexNum
        plot3(vertex_param(i).vertex(1), vertex_param(i).vertex(2), vertex_param(i).vertex(3), 'g.', 'Markersize',10);        % 画点
        hold on
        
        index = vertex_param(i).parentIndex;
        if(index > 0)
            x = [vertex_param(i).vertex(1), vertex_param(index).vertex(1)];
            y = [vertex_param(i).vertex(2), vertex_param(index).vertex(2)];
            z = [vertex_param(i).vertex(3), vertex_param(index).vertex(3)];
            
            plot3(x, y, z, 'b', 'LineWidth', 1);        % 画线
            hold on
        end
    end
    
    % 绘制 Path
    for i = 1 : pathVertexNum
        plot3(optimalPath(i).vertex(1), optimalPath(i).vertex(2), optimalPath(i).vertex(3), 'r.', 'Markersize', 15);        % 画点
        % plot3(optimalPath(i).vertex(1), optimalPath(i).vertex(2), optimalPath(i).vertex(3), 'ro', 'Markersize', 30);        % 画点

        hold on
        
        if(i < pathVertexNum)
            x = [optimalPath(i).vertex(1), optimalPath(i + 1).vertex(1)];
            y = [optimalPath(i).vertex(2), optimalPath(i + 1).vertex(2)];
            z = [optimalPath(i).vertex(3), optimalPath(i + 1).vertex(3)];
            
            plot3(x, y, z, 'r', 'LineWidth', 2);        % 画线
            hold on
        end
    end
    
    xlabel('X-axis')
    ylabel('Y-axis')
    zlabel('Z-axis')
    
    % 优化路径
%     [pathVertex, ~] = optimizePath(map, optimalPath);
%     plot(pathVertex(:, 1), pathVertex(:, 2), 'c-.', 'LineWidth', 2);        % 画线
%     hold on
    
