%=================================== is Get Goal ====================================
% 输入参数：2D 栅格地图，新节点，目标点，步长
% 节点参数：标志位
%
% 说明：
%     1. 标志位：0/1/2 表示 不可到达/可到达/是目标点；
%     2. 距目标点距离小于步长时，进行碰撞检测；
%     3. 如此检测，目标点一定不在新节点和临近点的连线上；
%
%=====================================================================================
%%
function flag = isGetGoal(map, newVertex, goalVertex, expandDis)

    v = goalVertex - newVertex;        % 计算矢量
    
    distance = norm(v);     % 计算距离
    if(distance <= expandDis)         % 若距离小于步长
        if(distance < 1)        % 若距离小于1，则与目标点重合
            flag = 2;
        else
            if(isObstacleFree(map, newVertex, goalVertex))      % 检测碰撞
                flag = 1;       % 可到达目标点
            else
                flag = 0;       % 不可到达目标点       
            end
        end
    else
        flag = 0;       % 不可到达目标点
    end
    
    
    