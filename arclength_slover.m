function result = arclength_slover(model,materials,solver,load)
% �����������

% ������ʼ��
step = 0;   % ������
u = zeros(model.number_dof,1); % ��λ������
lambda = 0;  % ��������
result = struct('step',[],'lamda',[],'model',[]);


%************************************����������������***********************
while ((step < solver.max_incremental_steps) && (lambda < 0.999))
    
    step = step + 1;
    fprint('��%d��������ʼ����\n',step)
    
    % ���¸նȾ���
    for i = 1:model.number_element
        model.element(i).k_mat_local = update_k_mat(model.element(i));  %���²��ϸնȾ���
        model.element(i).k_mat_global = model.element(i).k_mat_local;   %�����Ǽ��η����ԣ�1D���ľֲ���Ԫ�նȾ���=ȫ�ֵ�Ԫ�նȾ���
    end
    model.k = assembly(model);  % ��װ�ṹ�����ȫ�ָնȾ���
    
    % ��⵱ǰ�������µ�λ����������
    d_ut = line_solver(model,load);
    
    %************************************Ԥ��׶����***********************
    % ����Ԥ�ⲽ������������d_lambda�ͷ���ϵ��sign
    if step == 1
       sign = 1;
       d_lambda = solver.d_lambda_1_1;
       d_ut_1_1 = d_ut; %�洢��ʼ��������λ���������������ں�������GPS
    else
        % ����gsp��Ϊȷ��sign
        gsp = ((d_ut_1_1)'*(d_ut_1_1))/((d_ut)'*(d_ut_1_old));
        if gsp < 0
            sign = -1;
        end
        d_lambda = sign*abs(d_lambda_1_old)*sqrt((d_ut_1_old'*d_ut_1_old+load'*load)/(d_ut'*d_ut+load'*load));
    end
    
    % ����ܺ�������,��ֹ����1
    if (d_lambda + lambda) > 0.999
       d_lambda = 1- lambda; 
    end
    
    % ���Ԥ�ⲽλ������
    d_u = d_ut*d_lambda;
    
    
    
    
end
% ���������������








end