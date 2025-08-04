%================================= Update Region SampleScale =================================
% 输入参数：各区域尺寸，各区域节点个数，各区域标准节点个数，正态分布参数，
%          目标节点所在区域行列号，目标趋向比例，目标区域最小采样比例系数
%
% 输出参数：更新后的各区域采样比例及标准点数
%
% 描述：更新各区域采样比例，且目标区域采样概率始终不低于平均值；
%
% 说明：
%      1. 输入新节点为空，则不更新；
%      2. 正态分布参数为 [期望值u，标准方差sigma]；
%      3. 总采样比例范围为 [0, 1]；
%      4. 目标区域最小采样比例系数 表示 区域平均采样概率的倍数；
%
%============================================================================================
%%
function [sampleScale, regionVertexNum] = updateRegionSampleScale(sampleScale, regionLengthXYZ, regionVertexNum, standardVertexNum, newVertex,...
                                                                  pdfParam, goalXYZ, goalTrend, goalScaleK)

    % 若新节点为空，则不进行更新
    if(isempty(newVertex))
        return;
    end
    
    regionSize = tsize(standardVertexNum);       % 获取区域行列数
    regionNum = regionSize(1) * regionSize(2) * regionSize(3);

    % 计算新节点所处区域行列号（限制在总区域范围内）
    indexI = min([regionSize(1), floor(newVertex(1) / regionLengthXYZ(1)) + 1]);
    indexJ = min([regionSize(2), floor(newVertex(2) / regionLengthXYZ(2)) + 1]);
    indexK = min([regionSize(3), floor(newVertex(3) / regionLengthXYZ(3)) + 1]);
    
    regionVertexNum(indexI, indexJ, indexK) = regionVertexNum(indexI, indexJ, indexK) + 1;      % 计数加 1
    
    pdfScale = zeros(regionSize);
    totalScaleValue = 0;
    
    % 计算正态分布比例
    for i = 1 : regionNum
        if(standardVertexNum(i) > 0)        % 区域标准点数不为零（为零时，采样概率也为0）
            standardScale = regionVertexNum(i) / standardVertexNum(i);
            pdfScale(i) = 2 * normcdf(-standardScale, pdfParam(1), pdfParam(2));
            totalScaleValue = totalScaleValue + pdfScale(i);
        end
    end
    
    totalSampleScale = 1 - goalTrend;
    
    % 修改目标区域采样概率，使之不小于平均值
    goalPdfPercent = pdfScale(goalXYZ(1), goalXYZ(2), goalXYZ(3)) / totalScaleValue;
    averagePdfPercent = totalSampleScale * goalScaleK / regionNum;
    if(goalPdfPercent < averagePdfPercent)        % 修改目标区域采样概率，使之不小于平均值
        detScale = (averagePdfPercent - goalPdfPercent) / (1 - averagePdfPercent) * totalScaleValue;
        pdfScale(goalXYZ(1), goalXYZ(2), goalXYZ(3)) = pdfScale(goalXYZ(1), goalXYZ(2), goalXYZ(3)) + detScale;     % 更新目标区域比例
        totalScaleValue = totalScaleValue + detScale;       % 更新总比例
    end
    
    % 计算采样比例
    sampleScale = pdfScale ./ totalScaleValue * totalSampleScale;

    
    
