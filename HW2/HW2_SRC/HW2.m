close all, clear all, clc
ex1 = 1;
time = 5;
sim('inv_pend_three')
figure
subplot(3,1,1)
plot(pend1)
title('Response for Pendulum 1, execution time 10 ms, RM')
ylabel('Angle')
subplot(3,1,2)
plot(pend2)
title('Response for Pendulum 2, execution time 10 ms, RM')
ylabel('Angle')
subplot(3,1,3)
plot(pend3)
ylabel('Angle')
title('Response for Pendulum 3, execution time 10 ms, RM')
figure
subplot(3,1,1)
plot(ctrl1)

title('Control performance for Pendulum 1, execution time 10 ms, RM')
subplot(3,1,2)
plot(ctrl2)
title('Control performance for Pendulum 2, execution time 10 ms, RM')
subplot(3,1,3)
plot(ctrl3)
title('Control performance for Pendulum 3, execution time 10 ms, RM')



