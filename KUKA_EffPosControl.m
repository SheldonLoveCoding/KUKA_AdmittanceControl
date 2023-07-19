%% KUKA Admmitance control test
% 尝试基于位置的导纳控制，简化导纳模型，只保留了 阻尼项和弹簧项
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
pos={0., pi / 180 * 36, 0, -pi / 180 * 85, 0,pi / 180 * 58, 0};% initial cofiguration
iiwa.movePTPJointSpace( pos, relVel); % go to initial position
pause(2)
disp('初始');
[eef_T, eef_jacobian ] = iiwa.gen_DirectKinematics(cell2mat(pos));  %正运动学求解
init_eef_cart = eef_T(1:3,4)   %初始末端执行器笛卡尔坐标

%% Start direct servo in Cartesian space       
iiwa.realTime_startDirectServoCartesian();

%% Basic Parameter setting
runTime = 20;
fs = 100; % 单位Hz 这是指采样信号的频率，
timeInt  = 1 / fs;
timeVec  = [0:timeInt:runTime];   
totalLoop = length(timeVec);

% 一段位置轨迹
FE_range = 0.2;% 单位 m
RU_range = 0.1;
T = 4; % 单位 s
w = 2 * pi / T;
FE = FE_range * sin(w * timeVec);
RU = RU_range * sin(w * timeVec);
eefTarget = zeros(3,totalLoop);
eefTarget(1, : ) = init_eef_cart(1) + RU;     
eefTarget(2, : ) = init_eef_cart(2) + FE;    
eefTarget(3, : ) = init_eef_cart(3);

% % %保持初始位置不动
% eefTarget = zeros(3,totalLoop);
% eefTarget(1, : ) = init_eef_cart(1);     
% eefTarget(2, : ) = init_eef_cart(2);    
% eefTarget(3, : ) = init_eef_cart(3);


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
ALL_EFFCart = [];
ALL_ExTor = [];
ALL_ExEEFForce = [];
ALL_ControlSignal = [];


eefErrorLast = [0;0;0];  %上一周期的偏差
eefdErrorLast = [0;0;0];

deltaT = timeInt;
ALL_TimeInt = [];
for i = 1:totalLoop
    tic
    jPos = iiwa.getJointsPos();                              %读取角度
    [eefT, eefJacobian ] = iiwa.gen_DirectKinematics(cell2mat(jPos));
    eefCartNow = eefT(1:3,4);  %当前末端位置
    ALL_EFFCart = [ALL_EFFCart eefCartNow];
    ALL_EEFCartBias = [ ALL_EEFCartBias (eefCartNow-eefTarget(:,i))];
    ExTor = cell2mat( iiwa.sendJointsPositionsExTorque(jPos)  );  %关节力矩
    ALL_ExTor = [ ALL_ExTor ExTor' ];
    JVel = eefJacobian(1:3,:);        %速度雅各比
%    jPosd = pinv(JVel) * eefTargetd(:,i);  %末端笛卡尔速度*雅各比矩阵 --> 关节速度
    
    ExEEFForce = JVel * ExTor';    %末端力
    ALL_ExEEFForce = [ALL_ExEEFForce ExEEFForce]; 
    
    eefdError = b_cartesian_inv*(ExEEFForce - k_cartesian * eefErrorLast); %本周期 速度偏差
    eefError = eefErrorLast + eefdErrorLast * deltaT;
    
    eefTargetNew = eefTarget(:,i) + eefError;   %导纳控制器更新后的目标位置
    
    controlSignal = [eefTargetNew.*1000; pi ; 0 ; pi];   %锁定末端旋转
    ALL_ControlSignal = [ALL_ControlSignal controlSignal];

    iiwa.sendEEfPositionf(num2cell(controlSignal'));
    
    eefErrorLast = eefError;   %记录本次偏差
    eefdErrorLast = eefdError;
    t0 = toc;
    ALL_TimeInt = [ALL_TimeInt t0];
end

%% turn off the server
iiwa.realTime_stopDirectServoCartesian();
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
plot(timeVec , eefTarget(1,:),'r','Linewidth',2);hold on
plot(timeVec , ALL_EFFCart(1,:),'b','Linewidth',2);hold on
legend('参考轨迹','实际轨迹');
title('X方向位置轨迹');
subplot(3, 1, 2);
plot(timeVec , eefTarget(2,:),'r','Linewidth',2);hold on
plot(timeVec , ALL_EFFCart(2,:),'b','Linewidth',2);hold on
legend('参考轨迹','实际轨迹');
title('Y方向位置轨迹');
subplot(3, 1, 3);
plot(timeVec , eefTarget(3,:),'r','Linewidth',2);hold on
plot(timeVec , ALL_EFFCart(3,:),'b','Linewidth',2);hold on
legend('参考轨迹','实际轨迹');
title('Z方向位置轨迹');
