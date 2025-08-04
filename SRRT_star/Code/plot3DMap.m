%======================================== Plot 3D Map =====================================
% 输入参数：3D 栅格地图矩阵，立方体障碍物左下角坐标，立方体障碍物尺寸
% 输出参数：无
%
% 描述：根据立方体障碍物信息，绘制3D栅格地图
%
% 说明：
%     1. 立方体障碍物为多个，每个立方体障碍物的参数占一行；
%
%========================================================================================
%%
function plot3DMap(threeDimMap, obstacleStartVertex, obstacleLenghtXYZ)

    mapSize = size(threeDimMap);
    lengthX = mapSize(1);
    lengthY = mapSize(2);
    lengthZ = mapSize(3);
    
    obstacleNum = length(obstacleStartVertex(:, 1));        % 获取障碍物个数
    
    faces_matrix = [1 2 6 5; 2 3 7 6; 3 4 8 7;4 1 5 8; 1 2 3 4;5 6 7 8];        % 立方体各顶点链接顺序
    colorGray = [0.7, 0.7, 0.7];
    
    for i = 1 : obstacleNum
        x = obstacleStartVertex(i, 1);     % 立方体坐标x
        y = obstacleStartVertex(i, 2);     % 立方体坐标y
        z = obstacleStartVertex(i, 3);     % 立方体坐标z

        detx = obstacleLenghtXYZ(i, 1);
        dety = obstacleLenghtXYZ(i, 2);
        detz = obstacleLenghtXYZ(i, 3);
        
        vertex_matrix = [x, y, z; x + detx, y, z; x + detx, y + dety, z; x, y + dety, z; x, y, z + detz;...
                             x + detx, y, z + detz; x + detx, y + dety, z + detz; x, y + dety, z + detz];        % 立方体各顶点坐标

        patch('Vertices', vertex_matrix,'Faces', faces_matrix, 'FaceColor', colorGray);      % 绘制灰色立方体
    end

    view(3)
    axis equal

    % 设置地图范围
    xlim([0, lengthX])
    ylim([0, lengthY])
    zlim([0, lengthZ])
    
    grid on
    
