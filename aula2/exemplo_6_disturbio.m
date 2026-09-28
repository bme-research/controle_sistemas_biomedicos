clc
clear all
close all
pkg load control

%% Controle de Sistemas Biomedicos - Sensibilidade a Disturbio e Ruido
% Planta massa-mola-amortecedor (1 GDL): m*x'' + B*x' + Ks*x = fi
% G(s) = 1/(m s^2 + B s + Ks) ; C(s) = Kp ; H(s) = 1
% Y(s) = (CG/(1+CGH)) R + (G/(1+CGH)) D + (-CGH/(1+CGH)) N
clear; clc; close all;

%% ----------------- PARAMETROS DA PLANTA -----------------
m  = 2;      % kg
B  = 4;      % N.s/m
Ks = 20;     % N/m

G = tf(1, [m B Ks]);   % planta
H = tf(1, 1);          % sensor unitario

%% ----------------- GANHOS A COMPARAR --------------------
Kp_list = [5 100];     % caso menos sensivel ao ruido / menos sensivel ao disturbio

%% ----------------- CHAVES LIGA/DESLIGA ------------------
% Comece com os tres ligados (=1). Na aula, zere um de cada vez (=0)
% para mostrar a contribuicao isolada de cada entrada.
usar_R = 1;   % referencia
usar_D = 1;   % disturbio na entrada da planta
usar_N = 1;   % ruido de medicao

%% ----------------- SINAIS DE ENTRADA --------------------
t = 0:0.001:6;                          % vetor de tempo

R = 1.0 * ones(size(t));               % referencia: degrau unitario
D = .5 * (t >= 2);                     % disturbio: degrau 0.5 em t=2s
N = 5 * sin(2*pi*30*t);             % ruido de medicao: senoide 100 Hz

% Aplica as chaves
R = usar_R * R;
D = usar_D * D;
N = usar_N * N;

%% ----------------- FUNCOES DE TRANSFERENCIA -------------
figure('Name','Resposta Y(t)','Color','w');

for k = 1:numel(Kp_list)
    Kp = Kp_list(k);
    C  = tf(Kp,1);
    L  = C*G*H;                 % malha aberta

    T_yr = feedback(C*G, H);    % Y/R = CG/(1+CGH)
    T_yd = feedback(G,   C*H);  % Y/D =  G/(1+CGH)
    T_yn = feedback(-C*G*H, 1); % Y/N = -CGH/(1+CGH)

    % Superposicao das tres contribuicoes
    y_r = lsim(T_yr, R, t); % degrau unitário
    y_d = lsim(T_yd, D, t); % degrau módulo 0.5 em t>2
    y_n = lsim(T_yn, N, t);
    y   = y_r + y_d + y_n;

    subplot(numel(Kp_list),1,k);
    plot(t, y, 'LineWidth', 1.5); hold on;
    plot(t, R, 'k--', 'LineWidth', 1.0);
    plot(t, y_d, 'r')
    plot(t, y_r, 'k')
    grid on;
    title(sprintf('Kp = %g   |   R=%d  D=%d  N=%d', Kp, usar_R, usar_D, usar_N));
    xlabel('t (s)'); ylabel('x_0(t)');
    legend('y(t)','referencia', 'resp.', 'contr. dist.');

    % Ganhos DC (regime permanente) para discussao
    fprintf('--- Kp = %g ---\n', Kp);
    fprintf('  Resp. do sistema |Y/R|_{s->0} = %.4f\n', y(end));
    fprintf('  Sensib. disturbio |Y/D|_{s->0} = %.4f\n', abs(dcgain(T_yd)));
    % fprintf('  Sensib. ruido     |Y/N|_{s->0} = %.4f\n', abs(dcgain(T_yn)));
end
