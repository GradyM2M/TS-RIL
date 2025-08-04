%======================================== Creat 3D Map =====================================
% 输入参数：地图类型
% 输出参数：各立方障碍物前左下角顶点坐标，各立方障碍物尺寸，栅格地图矩阵
%
% 描述：根据三维矩阵，绘制3D栅格地图
%
% 说明：
%     1. type 0/1/2 表示 创建不同类型的三维栅格地图；
%     2. 各立方障碍物参数矩阵列数表示障碍物个数，即每一行表示一个障碍物；
%     3. 在原有基础上新增3Dmap_03类型，障碍物参考panda_moveit_config/worlds/motion_planning_sim.world;
%     4. 2023/11/16新增3Dmap_04类型，为补充3Dmap_03算例;
%     
%============================================================================================
%%
function [obstacleStartVertex, obstacleLenghtXYZ, threeDimMap] = creat3DMap(type)
    
    switch type
        case 0
            [obstacleStartVertex, obstacleLenghtXYZ, threeDimMap] = create3DMap01();
        case 1
            [obstacleStartVertex, obstacleLenghtXYZ, threeDimMap] = create3DMap02();
        case 2
            [obstacleStartVertex, obstacleLenghtXYZ, threeDimMap] = create3DMap03();  % 在原有基础上新增3Dmap_03类型
        case 3
            [obstacleStartVertex, obstacleLenghtXYZ, threeDimMap] = create3DMap04();  % 在原有基础上新增3Dmap_04类型
        otherwise
            obstacleStartVertex = [];
            obstacleLenghtXYZ = [];
            threeDimMap = [];
    end
    
    
% -----------------------------------------
% 创建指定类型的地图01
% -----------------------------------------
function [obstacleStartVertex, obstacleLenghtXYZ, threeDimMap] = create3DMap01()

    % 初始化三维栅格地图矩阵
    threeDimMap = zeros(400, 300, 100);

    % 设置障碍物区域参数
    obstacleStartVertex = [50, 0, 0; 236, 100, 0; 0, 236, 0; 200, 0, 0];
    obstacleLenghtXYZ = [114, 200, 100; 114, 200, 100; 200, 64, 100; 200, 64, 100];

    % 在地图中添加障碍物
    threeDimMap = creatRectangleRegion(threeDimMap, obstacleStartVertex, obstacleLenghtXYZ);
    
% -----------------------------------------
% 创建指定类型的地图02
% -----------------------------------------
function [obstacleStartVertex, obstacleLenghtXYZ, threeDimMap] = create3DMap02()

    % 初始化三维栅格地图矩阵
    threeDimMap = zeros(400, 300, 100);

    % 设置障碍物区域参数
    obstacleStartVertex = [102, 0, 0; 288, 152, 0; 0, 288, 0; 252, 0, 0];
    obstacleLenghtXYZ = [10, 148, 100; 10, 148, 100; 148, 12, 100; 148, 12, 100];

    % 在地图中添加障碍物
    threeDimMap = creatRectangleRegion(threeDimMap, obstacleStartVertex, obstacleLenghtXYZ);

% -----------------------------------------
% 创建指定类型的地图03（新增3Dmap_03类型）
% -----------------------------------------
function [obstacleStartVertex, obstacleLenghtXYZ, threeDimMap] = create3DMap03()

    % 初始化三维栅格地图矩阵
    threeDimMap = zeros(80, 80, 15);

    % 设置障碍物区域参数
    obstacleStartVertex = [0, 39, 0;
                           19,39, 0;
                           0, 59, 0;
                           18,19, 0;
                           39,19, 0;    % 5号
                           39,39, 0;
                           39,59, 0;
                           60,59, 0;
                           59,40, 0;
                           59, 0, 0];   % 1-10号矩形障碍的起始点坐标
    obstacleLenghtXYZ = [21, 2,15;
                         2, 22,15;
                         21, 2,15;
                         23, 2,15;
                         2, 22,15;      % 5号
                         2, 22,15;
                         21, 2,15;
                         20, 2,15;
                         2, 21,15;
                         2, 20,15;];    % 1-10号矩形障碍的长宽高尺寸
    % 在地图中添加障碍物
    threeDimMap = creatRectangleRegion(threeDimMap, obstacleStartVertex, obstacleLenghtXYZ);

% -----------------------------------------
% 创建指定类型的地图04（新增3Dmap_04类型）
% -----------------------------------------
function [obstacleStartVertex, obstacleLenghtXYZ, threeDimMap] = create3DMap04()

    % 初始化三维栅格地图矩阵
    threeDimMap = zeros(80, 80, 80);

    % 设置障碍物区域参数
    obstacleStartVertex = [0, 19, 0;
                           0, 39, 0;
                           0, 59, 0;
                           19,39, 0;
                           39,18, 0;
                           39,39, 0;    % 5号
                           39,59, 0;
                           59,19, 0;
                           59,19, 0;
                           59,39, 0];   % 1-10号矩形障碍的起始点坐标
    obstacleLenghtXYZ = [20, 2,15;
                         21, 2,15;
                         20, 2,15;
                         22, 2,15;
                         2, 23,15;
                         2, 22,15;      % 5号
                         21, 2,15;
                         21, 2,15;
                         2, 22,15;
                         21, 2,15];    % 1-10号矩形障碍的长宽高尺寸
    % 在地图中添加障碍物
    threeDimMap = creatRectangleRegion(threeDimMap, obstacleStartVertex, obstacleLenghtXYZ);


