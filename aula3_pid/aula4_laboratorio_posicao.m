% =====================================================================
% Controle de Sistemas Biomedicos - Laboratorio de Controle de Posicao
% Script de apoio (Octave) - QUBE-Servo 3
% Modelo tensao -> posicao (inclui back-EMF).
% =====================================================================
pkg load control

% ---------------------------------------------------------------------
% PARAMETROS DO GRUPO  (edite aqui)
% ---------------------------------------------------------------------
N = 1;                       % <-- numero do seu grupo
tr_alvo = 0.15;              % tempo de subida alvo [s]
Mp_alvo = max(8, 2*N)/100;   % sobressinal alvo (fracao)

% =====================================================================
% 2. FUNDAMENTO: A PLANTA
% =====================================================================
% --- Disco de inercia (dados da bancada) ---
m = 54e-3;          % kg
D = 49.5e-3;        % m
r = D/2;
Jdisk = 0.5 * m * r^2;       % disco cheio (acoplamento magnetico, sem furo)

% --- Parametros do motor QUBE-Servo 3 (datasheet) ---
Rm = 8.4;           % ohm  (resistencia de armadura)
kt = 0.042;         % N m/A (constante de torque)
km = 0.042;         % V s/rad (constante de fem)
Jrotor = 4.0e-6;    % kg m^2 (inercia do rotor)
b  = 4.0e-6;        % N m s/rad (atrito viscoso mecanico)

% --- Planta efetiva tensao->posicao ---
% J theta_dd + (b + kt*km/Rm) theta_d = (kt/Rm) V
J    = Jdisk + Jrotor;
Beff = b + kt*km/Rm;         % amortecimento efetivo (domina o back-EMF)
Kdc  = kt/Rm;                % ganho tensao->torque

printf('--- Secao 2: Planta ---\n');
printf('Jdisk = %.4e | Jrotor = %.1e | J = %.4e kg m^2\n', Jdisk, Jrotor, J);
printf('Beff  = %.4e N m s/rad (b + kt*km/Rm)\n', Beff);
printf('Kdc   = kt/Rm = %.4f\n', Kdc);

s = tf('s');
Gp = Kdc / ( s*(J*s + Beff) );   % planta tensao -> posicao
printf('\nPlanta G_theta(s):\n'); Gp

% =====================================================================
% 4.1 REQUISITOS  ->  parametros de 2a ordem alvo
% =====================================================================
zeta_alvo = -log(Mp_alvo) / sqrt(pi^2 + log(Mp_alvo)^2);
beta      = atan2(sqrt(1-zeta_alvo^2), zeta_alvo);
wd_alvo   = (pi - beta) / tr_alvo;
wn_alvo   = wd_alvo / sqrt(1-zeta_alvo^2);

printf('\n--- Secao 4.1: Requisitos (N=%d) ---\n', N);
printf('Mp_alvo = %.1f %% | tr_alvo = %.3f s\n', Mp_alvo*100, tr_alvo);
printf('zeta_alvo = %.3f | wn_alvo = %.2f rad/s\n', zeta_alvo, wn_alvo);

% =====================================================================
% 4.2 CONTROLADOR P
% =====================================================================
% CL: Kdc*Kp / (J s^2 + Beff s + Kdc*Kp)
%   wn = sqrt(Kdc*Kp/J),  zeta = Beff/(2 sqrt(Kdc*Kp*J))
Kp_P   = wn_alvo^2 * J / Kdc;
wn_P   = sqrt(Kdc*Kp_P/J);
zeta_P = Beff / (2*sqrt(Kdc*Kp_P*J));
Mp_P   = exp(-pi*zeta_P/sqrt(1-zeta_P^2))*100;

printf('\n--- Secao 4.2: Controlador P ---\n');
printf('Kp = %.4f\n', Kp_P);
printf('wn = %.2f rad/s | zeta = %.3f | Mp previsto = %.0f %%\n', wn_P, zeta_P, Mp_P);

T_P = feedback(Kp_P*Gp, 1);

% =====================================================================
% 4.3 CONTROLADOR PD
% =====================================================================
% CL: Kdc(Kp+Kd s) / (J s^2 + (Beff + Kdc*Kd) s + Kdc*Kp)
%   wn = sqrt(Kdc*Kp/J),  2 zeta wn = (Beff + Kdc*Kd)/J
% Chute inicial pela formula de 2a ordem:
Kp_PD    = wn_alvo^2 * J / Kdc;
Kd_chute = (2*zeta_alvo*wn_alvo*J - Beff) / Kdc;

T_PD_chute = feedback((Kp_PD + Kd_chute*s)*Gp, 1);

printf('\n--- Secao 4.3: Controlador PD (chute inicial) ---\n');
printf('Kp = %.4f | Kd = %.5f\n', Kp_PD, Kd_chute);

% Refino de Kd por causa do zero em s = -Kp/Kd (que aumenta o Mp real)
Kd_PD = Kd_chute;
for mult = 1:0.05:8
  Kd_try = Kd_chute*mult;
  info   = stepinfo(feedback((Kp_PD + Kd_try*s)*Gp, 1));
  if info.Overshoot <= Mp_alvo*100
    Kd_PD = Kd_try;
    break;
  end
end
T_PD    = feedback((Kp_PD + Kd_PD*s)*Gp, 1);
info_PD = stepinfo(T_PD);
printf('Kd refinado = %.5f (zero em s = -%.1f)\n', Kd_PD, Kp_PD/Kd_PD);
printf('Mp simulado (PD refinado) = %.0f %%\n', info_PD.Overshoot);

% ---- Item opcional 4.3: preco do zero do PD ----
Mp_formula  = exp(-pi*zeta_alvo/sqrt(1-zeta_alvo^2))*100;
Mp_simulado = stepinfo(T_PD_chute).Overshoot;
printf('\n--- Secao 4.3 (opcional): preco do zero ---\n');
printf('Mp (formula)  = %.0f %%\n', Mp_formula);
printf('Mp (simulado) = %.0f %%\n', Mp_simulado);
printf('Delta Mp      = %.0f %%\n', Mp_simulado - Mp_formula);

% =====================================================================
% 4.4 SIMULACAO
% =====================================================================
t = 0:0.0005:1;
figure;
yP  = step(T_P, t);
yPD = step(T_PD, t);
plot(t, yP, '--', 'linewidth', 1.5); hold on;
plot(t, yPD, '-', 'linewidth', 1.5);
plot(t, ones(size(t)), ':k');
grid on;
xlabel('t [s]'); ylabel('\theta (normalizado)');
legend('P', 'PD', 'referencia', 'location', 'southeast');
title(sprintf('Resposta ao degrau - Grupo %d', N));

info_P = stepinfo(T_P);
printf('\n--- Secao 4.4: Simulacao (tr 10-90%% do stepinfo) ---\n');
printf('        %8s %8s\n', 'tr[s]', 'Mp[%]');
printf('  P   : %8.3f %8.0f\n', info_P.RiseTime, info_P.Overshoot);
printf('  PD  : %8.3f %8.0f\n', info_PD.RiseTime, info_PD.Overshoot);

% Checagem de esforco (tensao de pico para um degrau de referencia)
R0 = deg2rad(90);   % degrau de 90 graus (ajuste ao seu ensaio)
V_pico = Kp_P * R0;
printf('\nEsforco: degrau de %.0f deg -> V_pico(P) ~ %.2f V (limite QUBE ~10 V)\n', rad2deg(R0), V_pico);

% =====================================================================
% 4.5 GANHOS PARA A ESTACAO (digitar no Python do QUBE)
% =====================================================================
printf('\n--- Secao 4.5: Ganhos para a estacao (Grupo %d) ---\n', N);
printf('  P puro : Kp = %.4f\n', Kp_P);
printf('  PD     : Kp = %.4f | Kd = %.5f\n', Kp_PD, Kd_PD);
