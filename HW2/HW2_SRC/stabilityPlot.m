clear all; close all;
syms K t h

% ezplot(1/(K)==t)
% hold on
% ezplot(1/2-1/K==t)

for K = 0:0.1:2
    x1 = 1/(K*h);
    x2 = 1/2-1/(K*h);
end

ezplot(x1)
hold on
ezplot(x2)
title('Stability region')
ylabel('\tau/h')
axis([0 2 0 1])