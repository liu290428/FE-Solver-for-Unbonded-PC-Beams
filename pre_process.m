function [model,materials,load,solver] = pre_process()
% �ֽ���������������ǲ��Ϸ����ԣ�ǰ������

%% ���ļ�����Ϣ
length_beam = 5000;
heigth_beam = 250;
width_beam = 150;

% �ֽ��
steel_layer = struct('area',[236,0],'depth',[220,30]);

%% ���Ĳ�����Ϣ
materials = struct(...
    'concrete', struct(...  % ����������
        'fc', 30, ...       % ��ѹǿ�� (MPa)
        'ela_mod', 33000, ...    % ����ģ�� (MPa)
        'eps_cu', 0.0035, ... % ����ѹӦ��
        'ft', 2.95 ...      % ����ǿ�� (MPa)
    ), ...
    'steel', struct(...       % �ֽ����
        'fy', 400, ...      % ����ǿ�� (MPa)
        'ela_mod', 200000 ...    % ����ģ�� (MPa)
    ) ...
);

%% ������ɢ��
length_element = 100;
assert(mod(length_beam, length_element) == 0, '������Ϊ��Ԫ���ȵ�������');
number_element = length_beam/length_element;
number_node = number_element+1;
number_dof = number_node*3;                     %���ɶȸ���

% �ڵ�����
coordinates_node = zeros(number_node, 2);
coordinates_node(:,1) = length_element*(0:number_element);

% ��Ԫ�ڵ����Ӿ���
connection = [1:number_node-1;2:number_node]';

% Լ������
constrain = zeros(number_dof,1);
constrain([1,2,end-1]) = 1;

% ��������
load = zeros(number_dof,1);
load([17*3-1,35*3-1]) = -40000;

% ���ɶ�����
% �ڵ����ɶ�����
id_node = reshape(1:3*number_node, 3, number_node)';
% ��Ԫ���ɶ�����
id_element = zeros(number_element,6);
for i = 1:number_element
   id_element(i,1:3)=id_node(i,:);
   id_element(i,4:6)=id_node(i+1,:);
end

%% ����ֲ�
number_layer = 10;
heigth_layer = heigth_beam/number_layer;
area_layer = heigth_layer*width_beam;

% 1. ��������
section_concrete=struct(...
    'area',repmat(area_layer, number_layer,1),...
    'y',(heigth_layer/2:heigth_layer:heigth_beam)-heigth_beam/2,...
    'sigma',zeros(number_layer,1),...
    'epsilon',zeros(number_layer,1),...
    'ela_mod',repmat(materials.concrete.ela_mod, number_layer, 1) ...
);
%  2. �ֽ��
section_steel = struct(...
    'area',steel_layer.area,...
    'y',steel_layer.depth-heigth_beam/2,...
    'sigma',zeros(2,1),...
    'epsilon',zeros(2,1),...
    'ela_mod',repmat(materials.steel.ela_mod,2,1)...
    );
% ����ṹ��(�������ֽ���)
section = struct('section_1',[],'section_2',[],'section_3',[]);
section.section_1 = struct('concrete',section_concrete,'steel',section_steel);
section.section_2 = struct('concrete',section_concrete,'steel',section_steel);
section.section_3 = struct('concrete',section_concrete,'steel',section_steel);

%% �����ṹ��
% �ڵ�ṹ��
node = struct('coordinates',[],'id',[]);
for i = number_node
    node(i).coordinates = coordinates_node(i,:);
    node(i).id = id_node(i,:);
end

% ��Ԫ�ṹ��
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

% ģ�ͽṹ��
model = struct('number_node',number_node,'number_element',number_element,'number_dof',number_dof,...
    'p',zeros(number_dof,1),'u',zeros(number_dof,1),...
    'constrain',constrain,'element',element,'k',zeros(number_dof,number_dof));

% ������ṹ��
solver = struct('d_lambda_1_1',0.01,'max_incremental_steps',200,'max_iterative_steps',50,'tolerance',1e-2);


end








