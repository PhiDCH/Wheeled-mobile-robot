%% Mô phỏng quan sát trạng thái xe tự hành

%% Khởi tạo
% Tần số trích mẫu IMU là 100, của GPS là 10Hz.

imuFs = 100;
gpsFs = 10;

% Đặt tọa độ ban đầu cho vị trí xe [kinh độ, vĩ độ, cao độ]
localOrigin = [42.2825 -71.343 53.0352];

% Để mô phỏng dễ dàng, cần đặt tốc độ trích mẫu IMU bằng một số nguyên
% lần tốc độ trích mẫu của GPS

% số trích mẫu IMU trong 1 lần trích mẫu GPS
imuSamplesPerGPS = (imuFs/gpsFs);
assert(imuSamplesPerGPS == fix(imuSamplesPerGPS), ...
    'Tốc độ trích mẫu không phù hợp');
% hàm "fix(a)"         làm tròn số a
% hàm assert(dk,"a")   báo lỗi a khi đk=False

%% Bộ lọc
% Đối tượng "insfilterNonholonomic" có hai hàm chính "predict" và "fusegps"

gndFusion = insfilterNonholonomic('ReferenceFrame', 'ENU', ...
    'IMUSampleRate', imuFs, ...
    'ReferenceLocation', localOrigin, ...
    'DecimationFactor', 2);

% 'ReferenceFrame', 'ENU'           đặt 3 chục hệ quy chiếu tham chiếu là 
%                                   Est-North-Up  
% 'IMUSampleRate', imuFs            đặt tốc độ trích mẫu
% 'ReferenceLocation', localOrigin  đặt gốc tọa độ tham chiếu
% 'DecimationFactor', 2             đặt cấp chính xác

%% Tạo quỹ đạo cho xe
% Xác định các tham số cho quỹ đạo là hình tròn 

% Tham số
r = 8.42; % (m)
speed = 2.50; % (m/s)
center = [0, 0]; % (m)
initialYaw = 90; % (degrees)
numRevs = 2; % số vòng 

% Tạo vector chứa các lần quay được 90 độ và thời gian tương ứng
revTime = 2*pi*r / speed;  % thời gian đi một vòng
theta = (0:pi/2:2*pi*numRevs).';  % đánh dấu  mỗi lần quay 90 độ
t = linspace(0, revTime*numRevs, numel(theta)).';  % thời gian quay 90 độ

% y = linspace(x1,x2,n) generates n points.
% The spacing between the points is (x2-x1)/(n-1).

% Define position.
x = r .* cos(theta) + center(1);
y = r .* sin(theta) + center(2);
z = zeros(size(x));
position = [x, y, z];

% Define orientation.
yaw = theta + deg2rad(initialYaw);
yaw = mod(yaw, 2*pi);  % mod - phép chia lấy dư
pitch = zeros(size(yaw));
roll = zeros(size(yaw));
orientation = quaternion([yaw, pitch, roll], 'euler', ...
    'ZYX', 'frame');   % chuyển sang quaternion

% Tạo quỹ đạo
groundTruth = waypointTrajectory('SampleRate', imuFs, ...
    'Waypoints', position, ...
    'TimeOfArrival', t, ...
    'Orientation', orientation);

% 'Sample rate' tần số trích mẫu quỹ đạo
% 'Waypoints', 'Orientation', 'TimeOfArrival' là trường chứa các điểm,
% hướng và thời gian tương ứng.

% Tạo số ngẫu nhiên để thêm nhiễu vào sensor, để mặc định seed=0
rng('default');

%% GPS

gps = gpsSensor('UpdateRate', gpsFs, 'ReferenceFrame', 'ENU');
gps.ReferenceLocation = localOrigin;

% Tùy chỉnh các tham số của cảm biến 
gps.DecayFactor = 0.5;                
gps.HorizontalPositionAccuracy = 1.0;   
gps.VerticalPositionAccuracy =  1.0;
gps.VelocityAccuracy = 0.1;

%% IMU Sensors

imu = imuSensor('accel-gyro', ...
    'ReferenceFrame', 'ENU', 'SampleRate', imuFs);

% Tùy chỉnh các tham số của cảm biến
% Accelerometer
imu.Accelerometer.MeasurementRange =  19.6133;
imu.Accelerometer.Resolution = 0.0023928;
imu.Accelerometer.NoiseDensity = 0.0012356;

% Gyroscope
imu.Gyroscope.MeasurementRange = deg2rad(250);
imu.Gyroscope.Resolution = deg2rad(0.0625);
imu.Gyroscope.NoiseDensity = deg2rad(0.025);


%% Khởi tạo tham số cho insfilterNonholonomic
% Sử dụng class 'groundTruth()' để khởi tạo giá trị ban đầu cho các tham số
% của bộ lọc

% Lấy giá trị ban đầu trước rồi mới reset()
[initialPos, initialAtt, initialVel] = groundTruth();
reset(groundTruth);

% Thiết lập các giá trị đầu cho tham số bộ lọc
gndFusion.State(1:4) = compact(initialAtt).';
gndFusion.State(5:7) = imu.Gyroscope.ConstantBias;
gndFusion.State(8:10) = initialPos.';
gndFusion.State(11:13) = initialVel.';
gndFusion.State(14:16) = imu.Accelerometer.ConstantBias;

%% Thiết lập phương sai cho bộ lọc 
% Nhiễu đo, đặc trưng cho mức độ nhiễu của cảm biến GPS và mức độ bất định
% trong mô hình động lực học xe tự hành
Rvel = gps.VelocityAccuracy.^2;
Rpos = gps.HorizontalPositionAccuracy.^2;
% phương sai bằng bình phương độ lệch chuẩn

% Phương sai của vận tốc hai bên sườn xe. Nếu không có hiện tượng trượt thì
% tham số này bằng 0
gndFusion.ZeroVelocityConstraintNoise = 1e-2; % (m/s)^2 

% Nhiễu quá trình, đặc trưng cho mức độ chính xác của bộ lọc
% Giá trị này càng cao thì độ tin cậy của phép đo càng thấp
gndFusion.GyroscopeNoise = 4e-6;
gndFusion.GyroscopeBiasNoise = 4e-14;
gndFusion.AccelerometerNoise = 4.8e-2;
gndFusion.AccelerometerBiasNoise = 4e-14;

% Hiệp phương sai
gndFusion.StateCovariance = 1e-9*ones(16);

%% Khởi tạo Scopes
% Class HelperScrollingPlotter dùng để vẽ 3-D dữ liệu từ bộ lọc
% (insfillterNonholonomic) và dữ liệu tham chiếu (groundTruth).

useErrScope = true; % xem sai lệch vị trí
usePoseView = true;  % xem hướng

if useErrScope
    errscope = HelperScrollingPlotter( ...
            'NumInputs', 4, ...
            'TimeSpan', 10, ...
            'SampleRate', imuFs, ...
            'YLabel', {'degrees', ...
            'meters', ...
            'meters', ...
            'meters'}, ...
            'Title', {'Quaternion Distance', ...
            'Position X Error', ...
            'Position Y Error', ...
            'Position Z Error'}, ...
            'YLimits', ...
            [-1, 1
             -1, 1
             -1, 1
             -1, 1]);
end

if usePoseView
    viewer = HelperPoseViewer( ...
        'XPositionLimits', [-15, 15], ...
        'YPositionLimits', [-15, 15], ...
        'ZPositionLimits', [-5, 5], ...
        'ReferenceFrame', 'ENU');
end

%% Mô phỏng
totalSimTime = 30; % seconds

% Khai báo biến lưu trữ dữ liệu (Log Data)
numsamples = floor(min(t(end), totalSimTime) * gpsFs);
truePosition = zeros(numsamples,3);
trueOrientation = quaternion.zeros(numsamples,1);
estPosition = zeros(numsamples,3);
estOrientation = quaternion.zeros(numsamples,1);

idx = 0;

% Vòng này chạy trên tần số GPS
for sampleIdx = 1:numsamples
    % Vòng này chạy trên tần số IMU
    for i = 1:imuSamplesPerGPS  
        if ~isDone(groundTruth)
            idx = idx + 1;
            
            % Lấy dữ liệu IMU 
            [truePosition(idx,:), trueOrientation(idx,:), ...
                trueVel, trueAcc, trueAngVel] = groundTruth();
            [accelData, gyroData] = imu(trueAcc, trueAngVel, ...
                trueOrientation(idx,:));
            
            % Dựa trên dữ liệu IMU thu được, tính toán đầu ra bộ lọc
            predict(gndFusion, accelData, gyroData);
            % 'predict' là một method, áp dụng thuật toán Particle Filter
            % để quan sát trạng thái
            
            % Lưu lại giá trị đầu ra bộ lọc (Position and Orientation)
            [estPosition(idx,:), estOrientation(idx,:)] = pose(gndFusion);
            % [position, orientation, velocity] = pose(FUSE) 
            % returns the current estimate of the pose.
            
            % Tính sai lệch và plot
            if useErrScope
                orientErr = rad2deg( ...
                    dist(estOrientation(idx,:), trueOrientation(idx,:)));
                posErr = estPosition(idx,:) - truePosition(idx,:);
                errscope(orientErr, posErr(1), posErr(2), posErr(3));
            end

            % Update hướng (the pose viewer)
            if usePoseView
                viewer(estPosition(idx,:), estOrientation(idx,:), ...
                    truePosition(idx,:), estOrientation(idx,:));
            end
        end
    end
    
    % Tiếp tục đo vị trí bằng GPS
    if ~isDone(groundTruth)
        % Lấy dữ liệu GPS
        [lla, gpsVel] = gps(truePosition(idx,:), trueVel);
        % lla - longtitude, latitude, altitude 

        % Update vị trí (position)
        fusegps(gndFusion, lla, Rpos, gpsVel, Rvel);
        % Rpos, Rvel là phương sai bộ lọc, đã khai báo ở trên
    end
end

%% Tính sai lệch
posd = estPosition - truePosition;

% sai lệch hướng được tính qua hàm 'dist', chuyên để tính sai lệch trên
% quaternion
quatd = rad2deg(dist(estOrientation, trueOrientation));

% In ra command window
fprintf('\n\nEnd-to-End Simulation Position RMS Error\n');
msep = sqrt(mean(posd.^2));
fprintf('\tX: %.2f , Y: %.2f, Z: %.2f   (meters)\n\n', msep(1), ...
    msep(2), msep(3));

fprintf('End-to-End Quaternion Distance RMS Error (degrees) \n');
fprintf('\t%.2f (degrees)\n\n', sqrt(mean(quatd.^2)));
