%====================================== Find RandVertex ======================================
% 输入参数：地图尺寸，目标节点，目标趋向概率，区域尺寸，各区域采样比例
% 输出参数：随机采样节点
%
% 描述：生成随机采样节点；
%
% 说明：
%      1. 若各区域采样比例为空，则进行整体随机采样；否则，分区域进行采样
%
%============================================================================================
%%
function randVertex = findRandVertex(mapSize, goalVertex, goalTrend, regionLengthXYZ, sampleScale)

    scale = rand();     % 生成 0~1 的随机数

    % 若各区域采样比例为空，则进行整体随机采样
    if(isempty(sampleScale))
        if(scale > goalTrend)       % 随机采样
            randVertex = [randi(mapSize(1)), randi(mapSize(2)), randi(mapSize(3))];
        else        % 选取目标节点
            randVertex = goalVertex;
        end
        
        return;     % 返回
    end

    % 若各区域采样比例不为空，则分区域进行采样
    
    regionSize = tsize(sampleScale);     % 获取区域行列数

    % 根据比例选择随机采样区域
    tmp = 0;
    indexI = 0;
    indexJ = 0;
    indexK = 0;
    for i = 1 : regionSize(1)
        for j = 1 :  regionSize(2)
            for k = 1 : regionSize(3)
                tmp = tmp + sampleScale(i, j, k);
                if(tmp > scale)
                    indexI = i;        % 记录采样到的区域一维号
                    indexJ = j;       % 记录采样到的区域二维号
                    indexK = k;       % 记录采样到的区域三号
                    break;      % 退出循环
                end
            end
            
            if(indexI > 0)      % 选取到区域
                break;      % 退出循环
            end
        end
        
        if(indexI > 0)      % 选取到区域
            break;      % 退出循环
        end
        
    end
    
    % 采样点为目标点
    if(indexI == 0)
        randVertex = goalVertex;
        return;
    end
    
    % 计算区域前左下角顶点的坐标
    leftBottom = [max([regionLengthXYZ(1) * (indexI - 1), 1]), max([regionLengthXYZ(2) * (indexJ - 1), 1]), max([regionLengthXYZ(3) * (indexK - 1), 1])];
    
    % 在随机选定的区域内随机采样点
    randx = min([mapSize(1), floor(leftBottom(1) + rand() * regionLengthXYZ(1)) + 1]);      % 随机采样 X 坐标，范围 [1, mapSize(1)]
    randy = min([mapSize(2), floor(leftBottom(2) + rand() * regionLengthXYZ(2)) + 1]);      % 随机采样 Y 坐标，范围 [1, mapSize(2)]
    randz = min([mapSize(3), floor(leftBottom(3) + rand() * regionLengthXYZ(3)) + 1]);      % 随机采样 Y 坐标，范围 [1, mapSize(3)]
    
    % 保存输出
    randVertex = [randx, randy, randz];
    
    
            
    
