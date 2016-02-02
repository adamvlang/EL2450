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

%TOOOYOOTOTOTO
Ts = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Continuous Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
s = tf('s');

upperNUM = {[K]};
upperDEN = {[Tau 1]};
lowerNUM = {[Gamma]};
lowerDEN = {[Gamma*Tau 1]};
uppertank=tf([upperNUM],[upperDEN]); % Transfer function for upper tank
lowertank=tf([lowerNUM],[lowerDEN]); % Transfer function for upper tank

uppertank=tf(upperNUM,upperDEN); % Transfer function for upper tank
lowertank=tf(lowerNUM,lowerDEN); % Transfer function for upper tank
G=uppertank*lowertank; % Transfer function from input to lower tank level

% CalculatePID paramaeters
chi = 0.5;
zeta = 0.8;
omega0 = 0.2;
[K, Ti, Td, N] = polePlacePID(chi, omega0, zeta,Tau,Gamma,K);
C = K + K/(Ti*s) + (K*Td*N*s)/(s+N);
F = C; % Transfer function for the controller

OpenLoop = F*G;
[Gm,Pm,Wgm,Wpm] = margin(OpenLoop) ;
disp(['Crossover Frequency : ' , num2str(Wpm)])
samplingtime =  1/((2*pi)/(0.35/Wpm))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Digital Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ts = 1; % Sampling time
samplingTimes = [1,2,4,8,16];

% for i = 1:length(samplingTimes)
for i = 0.25:0.01:0.3
    Ts = i;
    Ff = c2d(F,Ts,'zoh');    
    sim('tanks')
    figure
    plot(LR_zero.Time,LR_zero.Data,'b')
    hold on
    plot(LR_con.Time,LR_con.Data,'r')
    hold on
    plot([130 130], [35 55],'--k') %settling time limit
    hold on
    plot([0,200],[53.5,53.5],'--k')
    hold on
    plot([0,200],[49.8,49.8],'--c')
    hold on
    plot([0,200],[50.2,50.2],'--c')
    xlabel('Time [s]'); ylabel('Water level [cm]')
    legend('Discrete Controller','Continous')
    title(['Discrete Controller and continuos Controller Sampling time: ',num2str(Ts)])
%     print(['zero_dis_comp_sample_',num2str(Ts)],'-dpng')
end
Ff = c2d(F,Ts,'zoh'); % Transfer function for discrete designed controller
 sim('tanks')
 plot(LR_con.Time,LR_con.Data,'b')
 hold on
 plot(LR_zero.Time,LR_zero.Data,'r')
 xlabel('Time [s]'); ylabel('Water level [cm]')
 legend('Continous','Zero-order hold')
 title('Continous Controller and hold zero sampling 1')

% Discretize the continous controller, save it in state space form
% [Aa,Ba,Ca,Da] = ; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Discrete Control design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%To discretize the continous controller use c2d(Controller,Ts,'zoh')


% Gd = 1; % Sampled system model
% Fd = 1; % Transfer function for discrete designed controller
