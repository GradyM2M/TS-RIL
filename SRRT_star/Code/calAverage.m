%===================================== Cal Average ======================================
% 输入参数：数据
% 输出参数：平均值
%
% 描述：求数据的平均值；
%
% 说明：
%     1. type = 0/1/2 表示 行/列/整体 平均值；
%     2. 返回 列/行（type = 0/1） 向量；
%
%========================================================================================
%%
function dataAverage = calAverage(data, type)

    dataSize = size(data);      % 获取数据大小
    
    switch type
        case 0
            dataAverage = sum(data, 2) / dataSize(2);        % 求每行的平均值
        case 1
            dataAverage = sum(data, 1) / dataSize(1);        % 求每列的平均值
        otherwise
            dataAverage = sum(sum(data)) / (dataSize(1) * dataSize(2));        % 求整体的平均值
    end

