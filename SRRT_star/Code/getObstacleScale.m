%================================== Get Obstacle Scale ==================================
% 输入参数：3D 栅格地图，划分区域数，随机采样点数
% 输出参数：划分后的区域尺寸，区域障碍物比例
%
% 描述：将地图划分为矩阵区域，并通过蒙特卡洛法计算出每个区域障碍物所占的比例
%
% 说明：
%      1. 划分区域数为 [numX, numY, numZ]；
%      2. 随机采样点用于估计障碍物比例；
%      3. 划分区域尺寸 [lengthX, lengthY, lengthZ]；
%      4. 区域障碍物比例矩阵大小 numY * numX * numZ;
%
%========================================================================================
%%
function [regionLengthXYZ, obstacleScale] = getObstacleScale(map, regionNum, sampleNum)

    mapSize = tsize(map);        % 获取地图大小
    
    % 计算区域大小
    lengthX = floor(mapSize(1) / regionNum(1));
    lengthY = floor(mapSize(2) / regionNum(2));
    lengthZ = floor(mapSize(3) / regionNum(3));

    obstacleScale = zeros(regionNum);        % 初始化输出障碍物比例矩阵
    
    for i = 1 : regionNum(1)
        xLim = [(i - 1) * lengthX + 1, i * lengthX];        % 计算当前区域x范围
        for j = 1 : regionNum(2)
            yLim = [(j - 1) * lengthY + 1, j * lengthY];        % 计算当前区域y范围
            for k = 1 : regionNum(3)
                zLim = [(k - 1) * lengthZ + 1, k * lengthZ];        % 计算当前区域z范围
                
                obstacleNum = 0;
                for t = 1 : sampleNum
                    if(map(randi(xLim), randi(yLim), randi(zLim)) == 1)      % 检测采样点是否为障碍物
                        obstacleNum = obstacleNum + 1;
                    end
                end
                obstacleScale(i, j, k) = obstacleNum / sampleNum;      % 计算障碍物比例
            end
        end
    end
    
    regionLengthXYZ = [lengthX, lengthY, lengthZ];
                
                
                
                

