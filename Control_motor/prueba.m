% G = tf([0.02839],[1 0.2588 0.02939])
% pzmap(G)
% grid on

s = tf('s');
plant = 0.02839 / (s^2 + 0.2588*s + 0.02933);
controlSystemDesigner(plant);
