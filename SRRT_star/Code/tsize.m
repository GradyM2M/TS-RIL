%============================== Three Dim Matix Size ===============================
% 输入参数：三维矩阵
% 输出参数：矩阵大小
%
% 描述：为弥补size函数在三维矩阵最后一维为1时识别为二维矩阵的问题。
%
%
%====================================================================================
%%
function threeDimMatrixSize = tsize(target)

    threeDimMatrixSize = size(target);
    if(length(threeDimMatrixSize) < 3)
        threeDimMatrixSize(3) = 1;
    end
