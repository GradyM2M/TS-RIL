%================================= Get Cost Froom Root ==================================
% 输入参数：RRT，邻近节点下标，新节点
% 输出参数：新节点的开销
%
% 描述：计算最新节点的开销；
%
% 说明：
%      1. 
%
%========================================================================================
%%
function cost = getCostFroomRoot(vertex_param, nearVertexIndex, newVertex)

    cost = vertex_param(nearVertexIndex).cost + pdist2(vertex_param(nearVertexIndex).vertex, newVertex);

    