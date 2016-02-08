%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hybrid and Embedded control systems
% Homework 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all, clear all, clc  
init_tanks;
g = 9.82;
Tau = 1/alpha1*sqrt(2*tank_h10/g);
K = 60*beta*Tau;
Gamma = alpha1^2/alpha2^2;
Ts = 1;
Q_int = 100/(2^6);
Sat_upper = 100;
Sat_lower = -100;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continuous Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
s = tf('s');

uppertank=tf([K],[Tau 1]); % Transfer function for upper tank
lowertank=tf([Gamma],[Gamma*Tau 1]); % Transfer function for upper tank
G=uppertank*lowertank; % Transfer function from input to lower tank level

% CalculatePID paramaeters
chi = 0.5;
zeta = 0.8;
omega0 = 0.2;
[K, Ti, Td, N] = polePlacePID(chi, omega0, zeta,Tau,Gamma,K);
F = K + K/(Ti*s) + (K*Td*N*s)/(s+N); % Transfer function for the controller

OpenLoop = F*G;
Gc = (F*G)/(1+F*G);
[Gm,Pm,Wgm,Wpm] = margin(OpenLoop) ;
disp(['Crossover Frequency : ' , num2str(Wpm)])
samplingtime =  1/ ((2*pi)/(0.35/Wpm));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Digital Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ts = 1; % Sampling time
samplingTimes = [1,2,4,8,16];

% for i = 1:length(samplingTimes)
for i = 0.25:0.01:0.3
    Ts = i;
    Ff = c2d(F,Ts,'zoh');    
%     sim('tanks')
%     figure
%     plot(LR_zero.Time,LR_zero.Data,'b')
%     hold on
%     plot(LR_con.Time,LR_con.Data,'r')
%     hold on
%     plot([130 130], [35 55],'--k') %settling time limit
%     hold on
%     plot([0,200],[53.5,53.5],'--k')
%     hold on
%     plot([0,200],[49.8,49.8],'--c')
%     hold on
%     plot([0,200],[50.2,50.2],'--c')
%     xlabel('Time [s]'); ylabel('Water level [cm]')
%     legend('Discrete Controller','Continous')
%     title(['Discrete Controller and continuos Controller Sampling time: ',num2str(Ts)])
%     print(['zero_dis_comp_sample_',num2str(Ts)],'-dpng')
end
% Ts = samplingtime
%  F_d = c2d(F,Ts,'zoh'); % Transfer function for discrete designed controller
 
%  sim('tanks')
%   plot(LR_con.Time,LR_con.Data,'b')
%   hold on
%   plot(LR_zero.Time,LR_zero.Data,'r')
%   xlabel('Time [s]'); ylabel('Water level [cm]')
%   legend('Continous','Zero-order hold')
%   title('Continous Controller and hold zero sampling 1')

% Discretize the continous controller, save it in state space form
% [Aa,Ba,Ca,Da] = ; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discrete Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%To discretize the continous controller use c2d(Controller,Ts,'zoh')
% TASK 11
Ts = 4;
F_d = c2d(F,Ts,'zoh');
%sim('tanks');
%plot(LR_disc.Time,LR_disc.Data)
title('Performance Closed loop from Q8 with Ts=4')

% TASK 12

Ts = 4;
Fd = F_d;
Gd = c2d(G,Ts,'zoh');
Gd_closed = (Fd*Gd)/(1+Fd*Gd);
fprintf('\nTask 11: Coefficients for Gd\n')
for i = 1:length(Gd.num{1}) 
        disp(['a',num2str(i),' = ', num2str(Gd.num{1}(i))])
end

for i = 1:length(Gd.den{1})
        disp(['b',num2str(i),' = ', num2str(Gd.den{1}(i))])
 
end

% TASK 13
% Inside the unit circle

% TASK 14

Gc_minreal = minreal(Gc);
polesG = pole(Gc_minreal);  % Poles for Continous
polesGd = exp(Ts*polesG); % Calculated poles for Discrete'

Gc_polynomial = poly(polesGd);

fprintf('\nTask 14: Desired polynomial cooef for the closed loop system\n')
for i = 1:5 
        disp(['d',num2str(i),' = ', num2str(Gc_polynomial(i))])
end

% TASK 16

a1 = Gd.num{1}(2); a2 = Gd.num{1}(3);
b1 = Gd.den{1}(2); b2 = Gd.den{1}(3);
d0 = Gc_polynomial(2); d1 = Gc_polynomial(3); d2 = Gc_polynomial(4);
d3 = Gc_polynomial(5);

syms r c0 c1 c2

A = [1 a1 0 0; b1-1 a2 a1 0;b2-b1 0 a2 a1;-b2 0 0 a2];
B = [d0 - b1 + 1 ; d1 - b2 + b1 ; d2 + b2 ; d3];

C = A\B;

r = C(1);
c0 = C(2);
c1 = C(3);
c2 = C(4);

Fd = filt([c0 c1 c2], [1 r-1 -r], Ts);

Gdc = Gd*Fd/(1+Gd*Fd);
Gdc_minreal = minreal(Gdc);
Gdc_poles = pole(Gdc_minreal)

%TASK 17

sim('tanks')

plot(LR17)
hold on
plot(LR_disc, 'k')
title('Comparison between c2d discretized and pole placed controller')
legend('Discrete pole placed controller', 'Discrete controller with c2d')
xlabel('Time')
ylabel('Tank level')

%TASK 19
j=1;
figure
color = ['b' 'r' 'k' 'c' 'g'];
for i = 6:10
    disp('Q: ')
    Q_int = 100/(2^i)
    sim('tanks')
    plot(LRQ, color(j))
    hold on
    j = j + 1;
end
title('Level of second tank with different quantization levels')
legend('6 bits', '7 bits', '8 bits', '9 bits', '10 bit')
xlabel('Time')
ylabel('Tank level')

figure
plot(LR_disc)
title('Closed-loop system with a sampling time of 4s')
xlabel('Time')
ylabel('Tank level')

% z^4+d0*z^3+d1*z^2+d3
% A = [1 d0 d1 d3]

% Gd = 1; % Sampled system model
% Fd = 1; % Transfer function for discrete designed controller

syms z r c0 a1 a2 b1 b2 c1 c2
expand((z-1)*(z+r)*(z^2+b1*z+b2)+(c0*z^2+c1*z+c2)*(a1*z+a2))

