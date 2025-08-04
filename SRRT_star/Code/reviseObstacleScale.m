%================================== revise Obstacle Scale ==================================
% 输入参数：原始比例，修正比例范围， 修正参考比例中值
% 输出参数：修正比例
%
% 描述：将障碍物比例转换成自由区间比例，且以平均值作为分界线，扩张至指定区间；
%
% 说明：
%      1. 将原始比例扩展到修正比例范围；
%      2. “参考比例中值”必须在“修正比例范围”之内；
%
%==========================================================================================
%%
function reviseScale = reviseObstacleScale(initScale, scaleLim, midScale)

    initScale = 1 - initScale;      % 转换成自由区间比例

    initScaleMin = min(min(min(initScale)));        % 计算自由区间比例最小值
    initScaleMax = max(max(max(initScale)));        % 取出自由区间比例最大值

    dataSize = tsize(initScale);     % 若最后一维个数为1，则只是二维矩阵
    toatalNum = dataSize(1) * dataSize(2) * dataSize(3);
    aveScale = sum(sum(sum(initScale))) / toatalNum;       % 计算平均值
    
    leftDet = aveScale - initScaleMin;      % 取左端差值
    rightDet = initScaleMax - aveScale;      % 取右端差值

    reviseLeftDet = midScale - scaleLim(1);      % 取修正范围的左端差值
    reviseRightDet = scaleLim(2) - midScale;      % 取修正范围的右端差值
    
    reviseScale = zeros(dataSize);      % 初始化输出
    
    % 计算修正后的值
    for i = 1 : toatalNum 
        if(initScale(i) < aveScale)
            reviseScale(i) = scaleLim(1) + (initScale(i) - initScaleMin) / leftDet * reviseLeftDet;
        else
            reviseScale(i) = midScale + (initScale(i) - aveScale) / rightDet * reviseRightDet;
        end
    end
                
                
                
                

