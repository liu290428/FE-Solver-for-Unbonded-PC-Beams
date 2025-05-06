function [model,materials,load,solver] = pre_process()
% 钢筋混凝土梁（仅考虑材料非线性）前处理函数

%% 梁的几何信息
length_beam = 5000;
heigth_beam = 250;
width_beam = 150;

% 钢筋层
steel_layer = struct('area',[236,0],'depth',[220,30]);

%% 梁的材料信息
materials = struct(...
    'concrete', struct(...  % 混凝土参数
        'fc', 30, ...       % 抗压强度 (MPa)
        'ela_mod', 33000, ...    % 弹性模量 (MPa)
        'eps_cu', 0.0035, ... % 极限压应变
        'ft', 2.95 ...      % 抗拉强度 (MPa)
    ), ...
    'steel', struct(...       % 钢筋参数
        'fy', 400, ...      % 屈服强度 (MPa)
        'ela_mod', 200000 ...    % 弹性模量 (MPa)
    ) ...
);

%% 梁的离散化
length_element = 100;
assert(mod(length_beam, length_element) == 0, '梁长需为单元长度的整数倍');
number_element = length_beam/length_element;
number_node = number_element+1;
number_dof = number_node*3;                     %自由度个数

% 节点坐标
coordinates_node = zeros(number_node, 2);
coordinates_node(:,1) = length_element*(0:number_element);

% 单元节点连接矩阵
connection = [1:number_node-1;2:number_node]';

% 约束向量
constrain = zeros(number_dof,1);
constrain([1,2,end-1]) = 1;

% 荷载向量
load = zeros(number_dof,1);
load([17*3-1,35*3-1]) = -40000;

% 自由度索引
% 节点自由度索引
id_node = reshape(1:3*number_node, 3, number_node)';
% 单元自由度索引
id_element = zeros(number_element,6);
for i = 1:number_element
   id_element(i,1:3)=id_node(i,:);
   id_element(i,4:6)=id_node(i+1,:);
end

%% 截面分层
number_layer = 10;
heigth_layer = heigth_beam/number_layer;
area_layer = heigth_layer*width_beam;

% 1. 混凝土层
section_concrete=struct(...
    'area',repmat(area_layer, number_layer,1),...
    'y',(heigth_layer/2:heigth_layer:heigth_beam)-heigth_beam/2,...
    'sigma',zeros(number_layer,1),...
    'epsilon',zeros(number_layer,1),...
    'ela_mod',repmat(materials.concrete.ela_mod, number_layer, 1) ...
);
%  2. 钢筋层
section_steel = struct(...
    'area',steel_layer.area,...
    'y',steel_layer.depth-heigth_beam/2,...
    'sigma',zeros(2,1),...
    'epsilon',zeros(2,1),...
    'ela_mod',repmat(materials.steel.ela_mod,2,1)...
    );
% 截面结构体(三个积分截面)
section = struct('section_1',[],'section_2',[],'section_3',[]);
section.section_1 = struct('concrete',section_concrete,'steel',section_steel);
section.section_2 = struct('concrete',section_concrete,'steel',section_steel);
section.section_3 = struct('concrete',section_concrete,'steel',section_steel);

%% 建立结构体
% 节点结构体
node = struct('coordinates',[],'id',[]);
for i = number_node
    node(i).coordinates = coordinates_node(i,:);
    node(i).id = id_node(i,:);
end

% 单元结构体
element = struct('node_left',[],'node_right',[],'length',length_element,...
    'angle',0,'f_node',zeros(6,1),'id',[],'section',section,...
    'k_mat_local',zeros(6,6),'k_mat_global',zeros(6,6));
for i = 1:number_element
   element(i).node_left = node(connection(i,1));
   element(i).node_right = node(connection(i,2));
   element(i).length = length_element;
   element(i).angle = 0;
   element(i).f_node = zeros(6,1);
   element(i).id = id_element(i,:)';
   element(i).section = section;
   element(i).k_mat_local = zeros(6,6);
   element(i).k_mat_global = zeros(6,6);
end

% 模型结构体
model = struct('number_node',number_node,'number_element',number_element,'number_dof',number_dof,...
    'p',zeros(number_dof,1),'u',zeros(number_dof,1),...
    'constrain',constrain,'element',element,'k',zeros(number_dof,number_dof));

% 求解器结构体
solver = struct('d_lambda_1_1',0.01,'max_incremental_steps',200,'max_iterative_steps',50,'tolerance',1e-2);


end








