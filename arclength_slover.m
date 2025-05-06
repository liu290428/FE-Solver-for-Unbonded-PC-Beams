function result = arclength_slover(model,materials,solver,load)
% 弧长法求解器

% 变量初始化
step = 0;   % 增量步
u = zeros(model.number_dof,1); % 总位移向量
lambda = 0;  % 荷载因子
result = struct('step',[],'lamda',[],'model',[]);


%************************************进入增量迭代程序***********************
while ((step < solver.max_incremental_steps) && (lambda < 0.999))
    
    step = step + 1;
    fprint('第%d增量步开始运行\n',step)
    
    % 更新刚度矩阵
    for i = 1:model.number_element
        model.element(i).k_mat_local = update_k_mat(model.element(i));  %更新材料刚度矩阵
        model.element(i).k_mat_global = model.element(i).k_mat_local;   %不考虑几何非线性，1D梁的局部单元刚度矩阵=全局单元刚度矩阵
    end
    model.k = assembly(model);  % 组装结构整体的全局刚度矩阵
    
    % 求解当前增量步下的位移切线增量
    d_ut = line_solver(model,load);
    
    %************************************预测阶段求解***********************
    % 计算预测步荷载因子增量d_lambda和方向系数sign
    if step == 1
       sign = 1;
       d_lambda = solver.d_lambda_1_1;
       d_ut_1_1 = d_ut; %存储初始增量步中位移切线增量，会在后续计算GPS
    else
        % 计算gsp，为确定sign
        gsp = ((d_ut_1_1)'*(d_ut_1_1))/((d_ut)'*(d_ut_1_old));
        if gsp < 0
            sign = -1;
        end
        d_lambda = sign*abs(d_lambda_1_old)*sqrt((d_ut_1_old'*d_ut_1_old+load'*load)/(d_ut'*d_ut+load'*load));
    end
    
    % 检查总荷载因子,防止超过1
    if (d_lambda + lambda) > 0.999
       d_lambda = 1- lambda; 
    end
    
    % 求解预测步位移增量
    d_u = d_ut*d_lambda;
    
    
    
    
end
% 增量迭代程序结束








end