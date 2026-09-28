clc
clear all
close all

pkg load control

G1 = tf(1, [1, 2]);
rlocus(G1)

G2 = tf(1, conv([1, 2], [1, 4]));
figure
rlocus(G2)

G3 = tf(1, conv(conv([1, 2], [1, 4]), [1, 6]));
figure
rlocus(G3)
