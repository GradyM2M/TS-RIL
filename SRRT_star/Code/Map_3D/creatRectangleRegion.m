%=================================== Creat 3D Rectangle ================================
% 输入参数：3D 栅格地图，前左下角坐标，立方体尺寸
% 输出参数：3D 栅格地图
%
% 描述：根据坐标和尺寸生成立方体障碍物。
%
% 说明：
%
%========================================================================================
%%
function threeDimMap = creatRectangleRegion(threeDimMap, startVertex, lengthXYZ)

    % 添加立方体障碍物
    for sec = 1 : length(startVertex(:, 1))
        for i = startVertex(sec, 1) + 1 : startVertex(sec, 1) + lengthXYZ(sec, 1) - 1
            for j = startVertex(sec, 2) + 1 : startVertex(sec, 2) + lengthXYZ(sec, 2) - 1
                for k = startVertex(sec, 3) + 1 : startVertex(sec, 3) + lengthXYZ(sec, 3) - 1
                    threeDimMap(i, j, k) = 1;
                end
            end
        end
    end
