function result = gimbal_tune(action, port)
% GIMBAL_TUNE 一次运行计算、写入并核对双轴参数；不控制使能或运动目标。
% gimbal_tune                  计算并写入；修改下方参数区后再次运行即可。
% gimbal_tune("query")         查询当前运行参数并导出，适用于写入应答丢失后。
% gimbal_tune("calculate")     仅计算，不打开串口、不导出已确认参数。
% gimbal_tune("write","COM5") 指定串口；Linux可用/dev/serial/by-id/...。
% Q/R方式需要Control System Toolbox（控制系统工具箱）的dlqr。
% 调参只写主控运行内存；复位恢复Gimbal_Config.h中的默认值。

%% 参数区：每次改这里，然后运行gimbal_tune
TUNE_AXIS = "pitch"; % "pitch"仅更新Pitch；"both"/"yaw"供原双轴工具使用。
MODE = "gain"; % 默认使用固化增益；"qr"仍受下方模型来源限制。
PORT = "/dev/serial/by-id/usb-Echo_Echo_324438453233-if00"; % 主控稳定路径，不随ttyACM编号变化。
YAW_Q = diag([220, 0.8]);
YAW_R = 1;
PITCH_Q = diag([60, 0.01]);
PITCH_R = 1;
YAW_K = [14.5215994, 0.934219941];
PITCH_K = [9.29481325, 1.99631282];
% 前馈依次为重力常数[N*m]、重力斜率[N*m/rad]、运动摩擦[N*m]。
YAW_FEEDFORWARD = [0, 0, 0.14];
PITCH_FEEDFORWARD = [0.210217638139, -0.714052927197, 0];
% Pitch到位辅助B=0.15 N*m固定在Gimbal_Config.h，2°连续区间；不包含在现有十参数协议中。
RESPONSE_TIMEOUT_S = 2;

if nargin < 1, action = "write"; end
if nargin < 2, port = PORT; end
action = string(action);
assert(any(action == ["write", "query", "calculate"]), 'action须为write/query/calculate');

% 冻结的1ms电机反馈名义模型，便于分支长期独立使用，不依赖忽略的.mat文件。
% Yaw来源：sysid/yaw/results/20261118_final/yaw_model.json。
% Pitch来源：sysid/pitch/results/dynamic_amp06/pitch_model.json。
% 它们不含当前外置惯性反馈的噪声/额外延迟；求得K不等于惯性闭环已验收。
yaw_Ad = [1, 0.000993508358590463; 0, 0.9870448724304949];
yaw_Bd = [2.2126053457186182e-05; 0.044156142809492684];
pitch_Ad = [1, 0.0009181125479529363; 0, 0.8408223602466922];
pitch_Bd = [0.0002463962462122134; 0.47895949789222214];
PITCH_MODEL_SOURCE = "motor_legacy"; % 新外置模型独立验证后由部署步骤更新。
pitch_delay_s = 0;
pitch_angle_delay_s = 0;
pitch_rate_delay_s = 0;
pitch_motor_per_axis = 1;
pitch_torque_sign = 1;
pitch_reference_K = [27.6671015, 0.536113934]; % Q1090/.8的独立回归参考。

if action ~= "query"
    assert(any(TUNE_AXIS == ["pitch","yaw","both"]), 'TUNE_AXIS须为pitch/yaw/both');
    assert(any(MODE == ["qr", "gain"]), 'MODE须为qr或gain');
    if MODE == "qr"
        if TUNE_AXIS ~= "pitch"
            ValidateWeights(YAW_Q,YAW_R);
            YAW_K=dlqr(yaw_Ad,yaw_Bd,YAW_Q,YAW_R);
        end
        if TUNE_AXIS ~= "yaw"
            assert(PITCH_MODEL_SOURCE == "external_hi91", ...
                '当前Pitch仍为旧电机模型；请先完成外置IMU独立辨识并部署，或显式选择gain手动模式。');
            ValidateWeights(PITCH_Q,PITCH_R);
            model=struct('Ad',pitch_Ad,'Bd',pitch_Bd,'Ts',.001,'validated',true, ...
                'feedback_source','external_hi91','delay_s',pitch_delay_s, ...
                'angle_delay_s',pitch_angle_delay_s,'rate_delay_s',pitch_rate_delay_s, ...
                'motor_per_axis',pitch_motor_per_axis,'torque_sign',pitch_torque_sign, ...
                'gravity',[PITCH_FEEDFORWARD(1:2),0,0]);
            addpath(fullfile(fileparts(mfilename('fullpath')),'pitch','matlab'));
            prediction=compute_lqr_K(PITCH_Q,PITCH_R,model);
            PITCH_K=prediction.Kd;
        end
    end
    requested = single([YAW_K, YAW_FEEDFORWARD, PITCH_K, PITCH_FEEDFORWARD]);
    assert(numel(requested) == 10 && isreal(requested) && all(isfinite(requested)), '两轴参数须共10个有限实数');
    assert(all(requested([1, 2, 5, 6, 7, 10]) >= 0), '角度/速度增益及摩擦补偿不得为负');
    fprintf('候选Yaw K=[%.9g, %.9g]，Pitch K=[%.9g, %.9g]\n', requested([1, 2, 6, 7]));
else
    requested = zeros(1, 10, 'single');
end
if action == "calculate"
    result = struct('confirmed', false, 'values', requested);
    fprintf('仅计算，未连接主控、未写参数。Pitch模型来源与延迟按本次部署记录；仅calculate不代表主控确认。\n');
    return;
end

port = string(port);
if strlength(port) == 0
    ports = serialportlist("available");
    assert(numel(ports) == 1, '请在PORT中指定主控串口，可运行serialportlist("available")查看');
    port = ports(1);
end
if isunix
    % 保留稳定设备选择，但用真实节点打开，避免别名路径在本机MATLAB中留下串口锁。
    device_path = java.io.File(char(port));
    port = string(device_path.getCanonicalPath());
end
link = serialport(port, 115200, 'Timeout', RESPONSE_TIMEOUT_S);
cleanup = onCleanup(@() delete(link)); %#ok<NASGU>
flush(link, "input");
if action == "write" && TUNE_AXIS ~= "both"
    previous=Exchange(link,uint8(2),zeros(1,10,'single'),RESPONSE_TIMEOUT_S);
    assert(previous(6)==0, '读取当前参数失败，未发送写入');
    current=UnpackSingle(previous(13:52));
    if TUNE_AXIS == "pitch", requested(1:5)=current(1:5);
    else, requested(6:10)=current(6:10); end
end
operation=uint8(1+(action=="query"));
reply=Exchange(link,operation,requested,RESPONSE_TIMEOUT_S);
actual = UnpackSingle(reply(13:52));
if reply(6) ~= 0
    error('主控拒绝请求，状态%d（1参数非法/2未初始化）；未导出参数。', reply(6));
end
if action == "write"
    assert(isequal(typecast(actual, 'uint32'), typecast(requested, 'uint32')), ...
        '主控回读与请求不一致，未导出参数');
end
root = fileparts(fileparts(mfilename('fullpath')));
output = fullfile(root, 'build', 'gimbal_tuned_parameters.txt');
if ~isfolder(fileparts(output)), mkdir(fileparts(output)); end
ExportParameters(output, actual);
result = struct('confirmed', true, 'port', port, 'values', actual, 'file', output);
fprintf('主控已确认：Yaw K=[%.9g, %.9g]，Pitch K=[%.9g, %.9g]\n', actual([1, 2, 6, 7]));
fprintf('仅参数文件：%s\n', output);
end

function reply = Exchange(link,operation,values,timeout_s)
% 一次请求只发送一次；单轴写入前独立查询另一轴的当前实际值。
sequence=randi([1,double(intmax('uint32'))],1,'uint32');
packet=[uint8('GT01'),operation,uint8([0,0,0]),Pack32(sequence),PackSingle(values)];
write(link,[packet,Pack32(Checksum(packet))],"uint8");
reply=ReceiveReply(link,sequence,bitor(operation,uint8(128)),timeout_s);
end

function ValidateWeights(Q, R)
% Q/R为求解器的数学输入约束，不改变固件输出边界。
assert(isequal(size(Q), [2, 2]) && isreal(Q) && all(isfinite(Q(:))) && ...
    isequal(Q, Q.') && all(eig(Q) >= 0), 'Q须为2x2实对称半正定矩阵');
assert(isscalar(R) && isreal(R) && isfinite(R) && R > 0, 'R须为正实标量');
end

function packet = ReceiveReply(link, sequence, operation, timeout_s)
bytes = uint8([]);
started = tic;
while toc(started) < timeout_s
    available = link.NumBytesAvailable;
    if available > 0
        bytes = [bytes, reshape(read(link, available, 'uint8'), 1, [])]; %#ok<AGROW>
    end
    while numel(bytes) >= 4
        if ~isequal(bytes(1:4), uint8('GT01'))
            bytes(1) = [];
            continue;
        end
        if numel(bytes) < 56, break; end
        candidate = bytes(1:56);
        if Checksum(candidate(1:52)) ~= Unpack32(candidate(53:56))
            bytes(1) = [];
            continue;
        end
        bytes(1:56) = [];
        if candidate(5) == operation && Unpack32(candidate(9:12)) == sequence
            packet = candidate;
            return;
        end
    end
    pause(0.005);
end
error(['应答等待超时，写入结果未确认，参数可能已经生效。' ...
    '未自动重发；请运行gimbal_tune("query",同一串口)读取当前值。']);
end

function bytes = Pack32(values)
[~, ~, endian] = computer;
if endian == 'B', values = swapbytes(values); end
bytes = reshape(typecast(uint32(values), 'uint8'), 1, []);
end

function values = Unpack32(bytes)
values = typecast(uint8(bytes), 'uint32');
[~, ~, endian] = computer;
if endian == 'B', values = swapbytes(values); end
end

function bytes = PackSingle(values)
bytes = Pack32(typecast(single(values), 'uint32'));
end

function values = UnpackSingle(bytes)
values = typecast(Unpack32(bytes), 'single');
end

function crc = Checksum(bytes)
% CRC（Cyclic Redundancy Check，循环冗余校验）32/ISO-HDLC。
crc = uint32(hex2dec('FFFFFFFF'));
polynomial = uint32(hex2dec('EDB88320'));
for byte = reshape(bytes, 1, [])
    crc = bitxor(crc, uint32(byte));
    for bit = 1:8
        low = bitand(crc, uint32(1));
        crc = bitshift(crc, -1);
        if low ~= 0, crc = bitxor(crc, polynomial); end
    end
end
crc = bitxor(crc, uint32(hex2dec('FFFFFFFF')));
end

function ExportParameters(path, values)
fid = fopen(path, 'w');
assert(fid >= 0, '主控参数已确认，但导出文件无法打开：%s', path);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
axes = {'yaw', 'pitch'};
fields = {'k_angle', 'k_speed', 'gravity_constant', 'gravity_linear', 'moving_friction'};
for axis = 1:2
    for field = 1:5
        text = sprintf('%.9g', values((axis - 1) * 5 + field));
        if ~contains(text, '.') && ~contains(lower(text), 'e'), text = [text, '.0']; end
        fprintf(fid, 'config.%s.%s = %sf;\n', axes{axis}, fields{field}, text);
    end
end
end
