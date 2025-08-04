%===================================== Extract Data =====================================
% 输入参数：文件名，数据列数
% 输出参数：数据
%
% 描述：将数据存入文件，统一使用浮点型格式；
%
% 说明：
%     1. 若文件不存在，则返回空数据；
%
%========================================================================================
%%
function data = extractData(filename, dataColNum)

    if(exist(filename, 'file'))     % 如果文件存在
        fp = fopen(filename, 'r');      % 以读格式打开文件，保留之前数据

        dataFormat = '%f';
        for i = 2 : dataColNum
            dataFormat = strcat(dataFormat, 32, '%f');       % 合成格式字符串
        end

        data = cell2mat(textscan(fp, dataFormat));

        fclose(fp);     % 关闭文件
    else     % 如果文件不存在
        data = [];      % 返回空数据
    end
    
    
    