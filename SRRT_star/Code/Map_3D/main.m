%======================================== 主函数 =====================================
% 输入参数：地图类型
% 输出参数：各立方障碍物前左下角顶点坐标，各立方障碍物尺寸，栅格地图矩阵
%
% 描述：根据三维矩阵，绘制3D栅格地图
%
% 说明：
%     1. type 0/1/2 表示 创建不同类型的三维栅格地图；
%
%============================================================================================

clc
clear all

% 定义type类型，type = 2 or 3时生成motion_planning_sim.world中的障碍栅格地图
type = 3;

[obstacleStartVertex, obstacleLenghtXYZ, threeDimMap] = creat3DMap(type);

plot3DMap(threeDimMap, obstacleStartVertex, obstacleLenghtXYZ);

% 保存为.mat文件
save('3DMap_03.mat', 'threeDimMap', 'obstacleStartVertex', 'obstacleLenghtXYZ', '-v7.3')

hold on

% 绘制空间子区域划分线
% x = [20, 20, 40, 40, 60, 60, 0,  80, 0,  80, 0,  80];
% y = [0,  80, 0,  80, 0,  80, 20, 20, 40, 40, 60, 60];
% z = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
% for i = 2 : 2 : length(x)
%     plot3(x(i - 1 : i), y(i - 1 : i), z(i - 1 : i), 'r-.', 'LineWidth', 2);
%     hold on
% end

