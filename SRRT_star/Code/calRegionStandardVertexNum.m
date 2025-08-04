%============================== Cal Region Standard VertexNum ===============================
% 输入参数：各区域尺寸，各区域障碍物比例，各区域扩展步长比例，每个节点最小间距
% 输出参数：各区域标准点数
%
% 描述：根据自由空间大小，计算各区域容纳的标准节点数
%
% 说明：
%      1. 各区节点密度比例 根区域自由区域比例和扩展步长比例分别成正比和反比；
%
%============================================================================================
%%
function standardVertexNum = calRegionStandardVertexNum(regionLengthXYZ, obstacleScale,  expandDisScale, minDis)

    regionVolume = regionLengthXYZ(1) * regionLengthXYZ(2) * regionLengthXYZ(3);       % 计算区域体积大小
    
    if(minDis == 0)
        minDis = 1;     % 最小间距为1
    end
    
    vertexVolume = 4 / 3 * pi * minDis^3;       % 计算单个节点所需体积大小
    
    regionVertexDensityScale = (1 - obstacleScale) ./ expandDisScale;
    
    % 计算每个区域可容纳的标准节点个数（至少一个）
    standardVertexNum = floor(regionVolume / vertexVolume .* regionVertexDensityScale);

    
    