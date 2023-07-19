%% KUKA Admmitance control test
% 尝试基于速度的导纳控制，导纳模型包括质量项、阻尼项和弹簧项
% 7.14
% Create by lxd

close all;
clear;
clc;
warning('off')
iiwapath = 'D:\KST-Kuka-Sunrise-Toolbox-master\KST-Kuka-Sunrise-Toolbox-master\Matlab_client';
addpath(genpath(iiwapath));

%% Create the robot object
ip='172.31.1.147'; % The IP of the controller
arg1=KST.LBR14R820; % choose the robot iiwa7R800 or iiwa14R820
arg2=KST.Medien_Flansch_elektrisch; % choose the type of flange
Tef_flange=eye(4); % transofrm matrix of EEF with respect to flange.
% Tef_flange(3,4)=30/1000;
iiwa=KST(ip,arg1,arg2,Tef_flange); % create the object

%% Start a connection with the server

flag=iiwa.net_establishConnection();
if flag==0
  return;
end
disp('Kuka connected!');
pause(1);


%% Go to initial configuration 

relVel=0.25; % over ride relative joint velocities
pos={0., pi / 180 * 36, 0, -pi / 180 * 85, 0,pi / 180 * 58, 0};
iiwa.movePTPJointSpace( pos, relVel); % go to initial position
pause(2)
disp('初始');
[eef_T, eef_jacobian ] = iiwa.gen_DirectKinematics(cell2mat(pos));  %正运动学求解
init_eef_cart = eef_T(1:3,4)   %初始末端执行器笛卡尔坐标

%% Start direct servo in Joints space
iiwa.realTime_startVelControlJoints();

%% Basic Parameter setting
runTime = 20;
fs = 100; % 单位Hz 这是指采样信号的频率，
timeInt  = 1 / fs;
timeVec  = [0:timeInt:runTime];   
totalLoop = length(timeVec);

% % 一段速度轨迹
eefTarget = init_eef_cart; % 初始位置
minCartVel_val = 0;
maxCartVel_val = 0.005;% 单位 m/(0.01s)
T = 2; % 单位 s，这是运动的周期
w = 2 * pi / T;
FE_vel_1 = minCartVel_val + maxCartVel_val * sin(w * timeVec); % sin
FE_vel_2 = minCartVel_val + maxCartVel_val * sawtooth(w * timeVec + pi/2, 1/2); % 三角波
eefdTarget = zeros(3,totalLoop);
eefdTarget(1, : ) = minCartVel_val + 0.002 * sin(w * timeVec); % sin
eefdTarget(2, : ) = FE_vel_2;
jPosdmax = [85; 85; 100; 75; 130; 135; 135] * pi / 180 * 0.9; % 安全限速


% %保持初始位置不动
% eefTarget = init_eef_cart;
% eefdTarget = zeros(3,totalLoop);

%% Admittance
%高导纳参数
k_cartesian_high = diag([100,100,100]*1*1)*1.3*5*2/2;
b_cartesian_high = diag([100,100,100]*2.5);
H_inv_high          = diag([1 1 1]/10/5/1.4*3) ; 

%低导纳参数
k_cartesian_low = diag([100,100,100])*3;  
b_cartesian_low = diag([100,100,100]*1.0);
H_inv_low          = diag([1 1 1]/10/5*3)   ;

% k_cartesian = k_cartesian_high
% b_cartesian = b_cartesian_high
% H_inv          = H_inv_high 

k_cartesian = k_cartesian_low
b_cartesian = b_cartesian_low
H_inv          = H_inv_low 
b_cartesian_inv = inv(b_cartesian);

%% Control Loop
ALL_EEFCartBias = [];
ALL_EFFCart = []; % 测量的末端位置轨迹
ALL_EFFCartd = []; % 测量得到的末端速度轨迹
ALL_EFFCartRef = []; % 由参考速度积分得到的参考末端轨迹
ALL_ExTor = [];
ALL_ExEEFForce = [];
ALL_ControlSignal = [];


eefErrorLast = [0;0;0];  %上一周期的偏差
eefdErrorLast = [0;0;0];
eefddErrorLast = [0;0;0];
ALL_EFFCart = [ALL_EFFCart init_eef_cart];

deltaT = timeInt;
KP = 2;
for i = 1:totalLoop
    eefTarget = eefTarget + eefdTarget(:,i); % 由末端的速度积分得到的末端位置
    ALL_EFFCartRef = [ALL_EFFCartRef eefTarget];
    jPos = iiwa.getJointsPos();                              %读取角度
    [eefT, eefJacobian ] = iiwa.gen_DirectKinematics(cell2mat(jPos));
    eefCartNow = eefT(1:3,4);  %当前末端位置
    ALL_EFFCartd = [ALL_EFFCartd (eefCartNow - ALL_EFFCart(:, end))]; % 计算实际末端速度
    ALL_EFFCart = [ALL_EFFCart eefCartNow];  % 计算实际末端位置
    ALL_EEFCartBias = [ ALL_EEFCartBias (eefCartNow-eefTarget)];
    ExTor = cell2mat( iiwa.sendJointsPositionsExTorque(jPos)  );  %关节力矩
    ALL_ExTor = [ ALL_ExTor ExTor' ];
    JVel = eefJacobian(1:3,:);        %速度雅各比
%    jPosd = pinv(JVel) * eefTargetd(:,i);  %末端笛卡尔速度*雅各比矩阵 --> 关节速度
    
    ExEEFForce = JVel * ExTor';    %末端力
    ExEEFForce(1) = ExEEFForce(1) + 0.56; % 外力补偿
    ExEEFForce(3) = ExEEFForce(3) + 1.74;
    ALL_ExEEFForce = [ALL_ExEEFForce ExEEFForce]; 
    
    eefddError = H_inv*(ExEEFForce - b_cartesian * eefdErrorLast - k_cartesian * eefErrorLast); %本周期 加速度偏差
    eefdError = eefdErrorLast +  eefddErrorLast * deltaT;
    eefError = eefErrorLast + eefdErrorLast * deltaT;
    
    
    eefTargetdNew = eefdTarget(:,i)  + eefdError;
    eefTargetNew = eefTarget + eefError;   %导纳控制器更新后的目标位置
    
    
    ep = eefTargetNew - eefCartNow;   %当前位置与更新后的目标位置的偏差
    controlSignal = KP*ep + eefTargetdNew ; 
    controlSignal = [controlSignal; 0 ; 0 ; 0];    %锁定末端旋转
    ALL_ControlSignal = [ALL_ControlSignal controlSignal];
    
    jPosd = pinv(eefJacobian) * controlSignal;
    safeFlag = abs(jPosd) < abs(jPosdmax);
    if(sum(safeFlag) ~= 7)
       badJoint = find(safeFlag == 0);
       disp('以下关节超出限速！！！');
       disp(badJoint);
       break;
    end

    iiwa.sendJointsVelocities(num2cell(jPosd));  %输出关节速度
    
    eefErrorLast = eefError;   %记录本次偏差
    eefdErrorLast = eefdError;
    eefddErrorLast = eefddError;
end

%% turn off the server
iiwa.realTime_stopVelControlJoints();
iiwa.net_turnOffServer( );
warning('on')
rmpath(genpath(iiwapath));

%%
Len = size(ALL_EEFCartBias,2);
% timeVec  = 1:Len;  
figure(1)
subplot(2,1,1)
plot(timeVec , ALL_ExEEFForce(1,:),'b','Linewidth',2);
title('末端x方向受力','Fontsize',10);
grid on
subplot(2,1,2)
plot(timeVec, ALL_EEFCartBias(1,:),'r','Linewidth',2);
title('末端x方向受力位移偏差','Fontsize',10);
grid on

figure(2)
subplot(2,1,1)
plot(timeVec , ALL_ExEEFForce(2,:),'b','Linewidth',2);
title('末端y方向受力','Fontsize',10);
grid on
subplot(2,1,2)
plot(timeVec, ALL_EEFCartBias(2,:),'r','Linewidth',2);
title('末端y方向受力位移偏差','Fontsize',10);
grid on

figure(3)
subplot(2,1,1)
plot(timeVec , ALL_ExEEFForce(3,:),'b','Linewidth',2);
title('末端z方向受力','Fontsize',10);
grid on
subplot(2,1,2)
plot(timeVec, ALL_EEFCartBias(3,:),'r','Linewidth',2);
title('末端z方向受力位移偏差','Fontsize',10);
grid on

figure(4)
plot(timeVec , ALL_ExEEFForce(1,:),'b','Linewidth',2);
hold on
plot(timeVec , ALL_ExEEFForce(2,:),'g','Linewidth',2);
plot(timeVec , ALL_ExEEFForce(3,:),'r','Linewidth',2);
legend('X','Y','Z');
title('末端受力','Fontsize',25);
grid on

figure(5)
subplot(3, 1, 1);
plot(timeVec , ALL_EFFCartRef(1,:),'r','Linewidth',2);hold on
plot(timeVec , ALL_EFFCart(1,1:end-1),'b','Linewidth',2);hold on
legend('参考轨迹','实际轨迹');
title('X方向位置轨迹');
subplot(3, 1, 2);
plot(timeVec , ALL_EFFCartRef(2,:),'r','Linewidth',2);hold on
plot(timeVec , ALL_EFFCart(2,1:end-1),'b','Linewidth',2);hold on
legend('参考轨迹','实际轨迹');
title('Y方向位置轨迹');
subplot(3, 1, 3);
plot(timeVec , ALL_EFFCartRef(3,:),'r','Linewidth',2);hold on
plot(timeVec , ALL_EFFCart(3,1:end-1),'b','Linewidth',2);hold on
legend('参考轨迹','实际轨迹');
title('Z方向位置轨迹');

figure(6)
subplot(3, 1, 1);
plot(timeVec , eefdTarget(1,:),'r','Linewidth',2);hold on
plot(timeVec , ALL_EFFCartd(1,:),'b','Linewidth',2);hold on
legend('参考轨迹','实际轨迹');
title('X方向速度轨迹');
subplot(3, 1, 2);
plot(timeVec , eefdTarget(2,:),'r','Linewidth',2);hold on
plot(timeVec , ALL_EFFCartd(2,:),'b','Linewidth',2);hold on
legend('参考轨迹','实际轨迹');
title('Y方向速度轨迹');
subplot(3, 1, 3);
plot(timeVec , eefdTarget(3,:),'r','Linewidth',2);hold on
plot(timeVec , ALL_EFFCartd(3,:),'b','Linewidth',2);hold on
legend('参考轨迹','实际轨迹');
title('Z方向速度轨迹');
