%=================================== is Close ===================================
% 输入参数：数据，文件名，是否删除之前数据
% 输出参数：标志位
%
% 描述：将数据存入文件，统一使用浮点型格式；
%
% 说明：
%     1. 为不影响数据读取，每次输入的数据data列数要一致；
%     2. isUpdateFile： 0/1 表示 不删除/删除原数据
%     3. 
%
%========================================================================================
%%
function saveData(data, filename, isDeleteOldData)

    if(isDeleteOldData)
        fp = fopen(filename, 'w');      % 以写格式打开文件，自动清除之前数据
    else
        fp = fopen(filename, 'a');      % 以添加格式打开文件，保留之前数据
    end

    dataSize = size(data);      % 获取数据大小
    
    lineBreak = '\r\n';     % 定义换行符
    
    for i = 1 : dataSize(1)
        for j = 1 : dataSize(2)
            fprintf(fp, '%0.3f ', data(i, j));      % 将数据存入文件
        end
        fprintf(fp, lineBreak);     % 换行
    end
        
    fclose(fp);     % 关闭文件
    
    
    