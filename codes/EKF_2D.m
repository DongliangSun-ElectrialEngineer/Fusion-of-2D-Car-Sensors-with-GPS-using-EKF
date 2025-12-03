set(0,'DefaultFigureWindowStyle','docked');
close all;
clear;
clc;

load ('data');

f_a_noise_std = 0.0059; 
f_theta_noise_std = 0.1*pi/180;
pos_meas_noise_std = 10; % common horizontal accuracy of standard GPS
heading_meas_noise_std = 5*pi/180; % common accuracy of magnetic sensor heading 
y_observed = [East_GPS; North_GPS; Azimuth_Magnetometer];

%-> State Vector Initialization
x(1,1) = East_ref(1);
x(2,1) = North_ref(1);
x(3,1) = initial_forward_speed;
x(4,1) = initial_accel;
x(5,1) = Azimuth_ref(1);
x(6,1) = 0.0; % f_a bias
x(7,1) = 0.0; % f_theta bias

%--> The system (transition) matrix F
F = [0 0 sin(x(5,1)) 0  x(3,1)*cos(x(5,1))  0 0;
     0 0 cos(x(5,1)) 0  -x(3,1)*sin(x(5,1)) 0 0;
     0 0 0           1  0                   0 0;
     0 0 0           0  0                  -1 0;
     0 0 0           0  0                  0 -1;
     0 0 0           0  0                  0 0;
     0 0 0           0  0                  0 0;
    ];

%--> Design matrux H
H = [1 0 0 0 0 0 0;
     0 1 0 0 0 0 0;
     0 0 0 0 1 0 0];

G = diag([0.1 0.1 0.1 0.1 0.1 f_a_noise_std^2 f_theta_noise_std^2]);
Q = diag([1 1 1 1 1 1 1]);

R = diag([pos_meas_noise_std^2 pos_meas_noise_std^2 heading_meas_noise_std^2]);

P(:,:,1) = diag([25; 25; 10; 10; 1e-03; 5; 5]);

dT = mean(diff(t));

system_order = 7;

for k = 1:length(t)-1
    %--> KF Prediction
    %State Prediction
    error_states = zeros(system_order,1);
    x(6,k+1) = x(6,k);
    x(7,k+1) = x(7,k);
    x(3,k+1) = x(3,k) + x(4,k)*dT; % forward velocity prediction
    x(4,k+1) = x(4,k) + (f_a_noisy(k) - x(6,k))*dT;% forward acceleration prediction
    x(5,k+1) = x(5,k) + (f_theta_noisy(k) - x(7,k))*dT;% azimuth prediction
   
    x(1,k+1) = x(1,k) + x(3,k)*sin(x(5,k))*dT; % east position prediction
    x(2,k+1) = x(2,k) + x(3,k)*cos(x(5,k))*dT; % north position prediction
    
    F = [0 0 sin(x(5,k)) 0  x(3,k)*cos(x(5,k))  0 0;
         0 0 cos(x(5,k)) 0 -x(3,k)*sin(x(5,k))  0 0;
         0 0 0           1  0                   0 0;
         0 0 0           0  0                  -1 0;
         0 0 0           0  0                  0 -1;
         0 0 0           0  0                  0 0;
         0 0 0           0  0                  0 0;
        ];

    Phi = eye(system_order,system_order) + F*dT;

    % Calculate System Noise Matrix
    Qd = dT^2*G*Q*G';
    
    % Predict State Error Covariance
    P(:,:,k+1) = Phi*P(:,:,k)*Phi' + Qd;
    
    %--> KF Update every one second, apply outage between second 30 and 50
    %(&& ~(t(k)>= 30 && t(k) <= 50))
    if (mod(t(k),0.01) == 0 && t(k) > 0 )
        % Calculate Kalman Gain
        K = P(:,:,k+1)*H'/(H*P(:,:,k+1)*H'+R);
        % Update error covariance matrix P
        P(:,:,k+1) = P(:,:,k+1) - K*H*P(:,:,k+1);
        % Calculate error state
        error_states = K*(y_observed(:,k+1) - H*x(:,k+1));
        % Correct states
        x(:,k+1) = x(:,k+1) + error_states;
    end
    % Normalize P
    P(:,:,k+1) = (P(:,:,k+1)+P(:,:,k+1))/2;
end


figure;
plot(t,Azimuth_ref*180/pi,'b');grid on;hold on;
plot(t,Azimuth_Magnetometer*180/pi,'k*');
plot(t,x(5,:)*180/pi,'-.r','LineWidth',2);
title('Azimth(degrees)');xlabel ('time(s)');ylabel('Azimuth(degrees)');
legend('Ground Truth','Noisy Measurements','EKF Estimation');

figure;plot(t,East_ref,'b');grid on;hold on;
plot(t,East_GPS,'k*');
plot(t,x(1,:),'r','linewidth',4);
title('East(m)');xlabel ('time(s)');ylabel('East(m)');
legend('Ground Truth','Noisy Measurements','EKF Estimation');

figure;plot(t,North_ref,'b');grid on;hold on;
plot(t,North_GPS,'k*');
plot(t,x(2,:),'r','linewidth',4);
title('North(m)');xlabel ('time(s)');ylabel('North(m)');
legend('Ground Truth','Noisy Measurements','EKF Estimation');

figure;
plot(t,x(6,:),'r')  
title('f_a bias');xlabel ('time(s)');ylabel('bias(m/s^3)');
legend('f_a bias');

figure;plot(t,x(7,:),'r')
title('f_{theta} bias');xlabel ('time(s)');ylabel('bias(rad/s)');
legend('f_{theta} bias');
 
figure;
plot(y_observed(1,:),y_observed(2,:),'*k');hold on;grid on;
plot(x(1,:),x(2,:),'r','linewidth',3);
plot(East_ref,North_ref,'b','linewidth',3);
legend('Noisy Measurements','EKF','Ground Truth');
title('2D position(m)');xlabel ('East(m)');ylabel('North(m)');

figure;
plot(x(1,:),x(2,:),'r','linewidth',3);hold on;grid on;
plot(East_ref,North_ref,'b','linewidth',3);
legend('EKF','Ground Truth');
title('2D position(m)');xlabel ('East(m)');ylabel('North(m)');
