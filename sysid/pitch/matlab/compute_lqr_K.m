function out = compute_lqr_K(Q, R, dataset_dir, static_friction_nm)
% COMPUTE_LQR_K 用本次俯仰模型与用户Q/R计算1 ms离散增益，不绘图。
% Q为对称半正定2x2矩阵或两个对角权重；R为正标量（单输入的1x1矩阵）。
% 返回离散极点、10度阶跃指标和增量力矩预测；不等同于实机验收。
% 可选第四参数对应已确认的连续到位前馈幅值，默认0；仅用于静止目标预测。
assert(nargin >= 3 && nargin <= 4 && ~isempty(dataset_dir), '必须明确指定本次俯仰模型目录');
if nargin < 4, static_friction_nm=0; end
assert(isscalar(static_friction_nm) && isfinite(static_friction_nm) && static_friction_nm >= 0, ...
    '连续前馈幅值须为有限非负标量');
if isstruct(dataset_dir), S=dataset_dir; model_dir='in-memory external model';
else, S=load(fullfile(dataset_dir,'pitch_model.mat')); model_dir=char(dataset_dir); end
assert(S.validated, '本次模型独立验证未通过，不生成待下发增益');
assert(abs(S.Ts-0.001) < eps, '模型采样周期必须与固件1 ms一致');
if isvector(Q) && numel(Q) == 2, Q = diag(Q); end
assert(isequal(size(Q),[2 2]) && all(isfinite(Q(:))) && norm(Q-Q','fro') <= 1e-12, 'Q须为有限对称2x2矩阵');
assert(min(eig(Q)) >= -1e-12, 'Q须半正定');
assert(isscalar(R) && isfinite(R) && R > 0, 'R须为有限正标量');
if isfield(S,'A') && isfield(S,'B'), Kc=lqr(S.A,S.B,Q,R); else, Kc=[]; end
Kd = dlqr(S.Ad,S.Bd,Q,R);
poles = eig(S.Ad-S.Bd*Kd);
assert(all(abs(poles)<1), '离散闭环不稳定');
% 延迟只在离线闭环验证中增广；固件仍是两状态反馈，无观察器或滤波器。
input_count=0; angle_count=0; rate_count=0;
if isfield(S,'delay_s'), input_count=ceil(S.delay_s/S.Ts); end
if isfield(S,'angle_delay_s'), angle_count=ceil(S.angle_delay_s/S.Ts); end
if isfield(S,'rate_delay_s'), rate_count=ceil(S.rate_delay_s/S.Ts); end
history=max(angle_count,rate_count); count=2+2*history+input_count;
Ftheta=zeros(1,count); Ftheta(1+2*angle_count)=1;
Frate=zeros(1,count); Frate(2+2*rate_count)=1;
ratio=1; torque_sign=1;
if isfield(S,'motor_per_axis'), ratio=S.motor_per_axis; end
if isfield(S,'torque_sign'), torque_sign=S.torque_sign; end
g=S.gravity(2)*ratio*torque_sign;
static_range=deg2rad(2);
% 饱和区与目标附近的两个斜率都计入延迟检查；固件无新增控制状态。
for extra_gain=unique([0,static_friction_nm/static_range])
    N=Kd(1)+g+extra_gain;
    Ceffort=-N*Ftheta-Kd(2)*Frate; Ceffort(1)=Ceffort(1)+g;
    Acl=zeros(count); Acl(1:2,1:2)=S.Ad; Bcl=zeros(count,1);
    if input_count==0
        Acl(1:2,:)=Acl(1:2,:)+S.Bd*Ceffort; Bcl(1:2)=S.Bd*N;
    else
        first=3+2*history; Acl(1:2,end)=S.Bd;
        Acl(first,:)=Ceffort; Bcl(first)=N;
        for index=first+1:count, Acl(index,index-1)=1; end
    end
    for index=1:history
        dest=2*index+1:2*index+2; src=2*index-1:2*index;
        Acl(dest,src)=eye(2);
    end
    poles=eig(Acl);
    assert(all(abs(poles)<1),'计入延迟及连续前馈局部斜率后闭环不稳定');
    if extra_gain==0
        base_A=Acl; base_B=Bcl; base_C=Ceffort; base_N=N; base_poles=poles;
    end
end
Ctheta=Ftheta; Cvelocity=zeros(1,count); Cvelocity(2)=ratio;
closed = ss(Acl,Bcl,Ctheta,0,S.Ts);
t = (0:S.Ts:2)';
if static_friction_nm==0
    y = squeeze(step(closed,t))*10;
    u = squeeze(step(ss(Acl,Bcl,Ceffort,N,S.Ts),t))*deg2rad(10);
    velocity = squeeze(step(ss(Acl,Bcl,Cvelocity,0,S.Ts),t))*deg2rad(10);
else
    % 大阶跃直接重放连续饱和公式，不能把目标附近的斜率当成全程线性增益。
    injected=zeros(count,1);
    if input_count==0, injected(1:2)=S.Bd; else, injected(3+2*history)=1; end
    state=zeros(count,1); reference=deg2rad(10);
    y=zeros(size(t)); u=y; velocity=y;
    for index=1:numel(t)
        assisted=static_friction_nm*min(1,max(-1,(reference-Ftheta*state)/static_range));
        y(index)=rad2deg(Ctheta*state); velocity(index)=Cvelocity*state;
        u(index)=base_C*state+base_N*reference+assisted;
        state=base_A*state+base_B*reference+injected*assisted;
    end
end
info = stepinfo(y,t,10,'SettlingTimeThreshold',0.05);
out = struct('Q',Q,'R',R,'Kc',Kc,'Kd',Kd,'Ts',S.Ts,'poles',poles, ...
    'bandwidth_hz',bandwidth(closed)/(2*pi), 'settling_s',info.SettlingTime, ...
    'overshoot_percent',info.Overshoot,'predicted_peak_incremental_torque_nm',max(abs(u)), ...
    'predicted_peak_velocity_rad_s',max(abs(velocity)), ...
    'gravity',S.gravity,'model_dir',model_dir, ...
    'static_friction_nm',static_friction_nm,'static_friction_linear_range_deg',2, ...
    'unassisted_poles',base_poles, ...
    'predicted_target_pass',info.SettlingTime<=0.15 && info.Overshoot<=15 && max(abs(velocity))<20);
if isfield(S,'feedback_source'), out.feedback_source=S.feedback_source; end
if isfield(S,'validation_scope'), out.validation_scope=S.validation_scope; end
if isfield(S,'nominal_approximation'), out.nominal_approximation=S.nominal_approximation; end
disp(out);
if isfield(S,'nominal_approximation')
    fprintf('辨识模型为ARX；以下LQR使用其低频两状态近似，极点与阶跃为名义预测。\n');
end
if isfield(S,'validation_scope'), fprintf('模型验证范围：%s\n',S.validation_scope); end
fprintf('固件下发离散增益：K %.9g %.9g\n',Kd);
fprintf('这是模型预测；还需核对有效频带、重力负载、20 rad/s速度边界及实机阶跃。\n');
end
