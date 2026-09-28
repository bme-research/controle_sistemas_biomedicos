clc
clear all
close all

pkg load control

den = conv(conv([1, 2], [1, 4]), [1, 6]);

G1 = tf(1, den);
rlocus(G1)

# adicionando zero depois do -6
G2 = tf([1, 8], den)
figure
rlocus(G2)

# adicionando zero antes do -6
G3 = tf([1, 5], den)
figure
rlocus(G3)

# adicionando zero antes do -2
G4 = tf([1, 1], den)
figure
rlocus(G4)
