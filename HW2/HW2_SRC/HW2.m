close all, clear all, clc
sim('inv_pend_three')
figure

title('hej')
subplot(3,1,1)
plot(pend1)
subplot(3,1,2)
plot(pend2)
subplot(3,1,3)
plot(pend3)


figure
subplot(3,1,1)
plot(ctrl1)
subplot(3,1,2)
plot(ctrl2)
subplot(3,1,3)
plot(ctrl3)
