pkg load control
close all
clear all

G = tf(10, conv([1,0], [1,1]))

# método 1
figure(1)
rlocus(G, 0.5, 0, 100)
xlim([-5, 0])
ylim([-5, 5])
sgrid(0.5, 3)
hold on
plot(-1.5, 2.598, 'rx')
plot(-1.5, -2.598, 'rx')
plot(-2.5, 0, 'ko') # zero
plot(-3.5, 0, 'kx') # polo
legend("off")
hold off

figure(2)
Gc = tf([1, 1.937], [1, 4.648]);
rlocus(G*Gc, 0.05, 0, 100)
xlim([-5, 0])
ylim([-5, 5])
sgrid(0.5, 3)
hold on
plot(-1.5, 2.598, 'rx')
plot(-1.5, -2.598, 'rx')
legend("off")
hold off

Kc = 1.23
Gmf1 = feedback(Kc*Gc*G,1);
figure(3)
rlocus(Gc*G)
figure(4)
step(Gmf1)


# método 2
figure(5)
rlocus(G, 0.5, 0, 100)
xlim([-5, 0])
ylim([-5, 5])
sgrid(0.5, 3)
hold on
plot(-1.5, 2.598, 'rx')
plot(-1.5, -2.598, 'rx')
plot(-1, 0, 'ko') # zero
plot(-2.5, 0, 'kx') # polo
legend("off")
hold off

Gc = 0.9*tf([1, 1],[1, 3]);
Gmf2 = feedback(Gc*G,1)
figure(6)
rlocus(Gc*G)
figure(7)
step(Gmf2)

figure(8)
step(Gmf1, Gmf2)
