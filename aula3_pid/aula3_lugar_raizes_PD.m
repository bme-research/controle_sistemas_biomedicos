% =========================================================================
%  LUGAR DAS RAIZES - PENDULO COM CONTROLADOR PD
%  Kd FIXO, Kp VARIANDO  (um lugar das raizes para cada Kd)
%
%  FT de malha fechada dada:
%
%        Theta            Kd*s + Kp
%       -------(s) = ------------------------------
%       Theta_d       s^2 + a*Kd*s + (b + a*Kp)
%
%  com a = 1/I  e  b = m*g*l/I.
%
%  Equacao caracteristica reescrita como lugar das raizes EM Kp:
%
%       s^2 + a*Kd*s + b + a*Kp = 0
%                    a
%   1 + Kp * ------------------- = 0     ->   L(s) = a/(s^2 + a*Kd*s + b)
%             s^2 + a*Kd*s + b                ganho variavel = Kp
%
%  OBSERVACAO IMPORTANTE (aparece nos graficos):
%  a soma das raizes vale -a*Kd, que NAO depende de Kp. Logo, com Kd fixo,
%  variar Kp move os polos ao longo de uma RETA VERTICAL em
%      sigma = -a*Kd/2   (enquanto o par for complexo).
%  => Kp aumenta wn e diminui zeta, mas NAO muda a taxa de decaimento
%     (ts fica praticamente constante). Quem escolhe ts e' o Kd.
%
%  Nao requer o pacote 'control' (raizes por roots(), degrau por expm()).
%
%  Octave:  >> lugar_raizes_PD
% =========================================================================
1;

% =========================================================================
%  FUNCOES AUXILIARES
% =========================================================================
function grade_zeta_wn(ax)
% linhas de zeta constante (raios) e wn constante (arcos), estilo sgrid
  xl = xlim(ax); yl = ylim(ax);  Rmax = max(abs([xl yl]));
  for z = [0.3 0.5 0.7 0.9]
    plot(ax, [0 -Rmax*z], [0  Rmax*sqrt(1-z^2)], ':', 'color', [.6 .6 .6]);
    plot(ax, [0 -Rmax*z], [0 -Rmax*sqrt(1-z^2)], ':', 'color', [.6 .6 .6]);
  end
  th = linspace(pi/2, 3*pi/2, 200);
  for w = 5:5:Rmax
    plot(ax, w*cos(th), w*sin(th), ':', 'color', [.85 .85 .85]);
  end
  plot(ax, xl, [0 0], 'k-');  plot(ax, [0 0], yl, 'k-');
end

function R = lr_raizes(a, b, Kd, Kp_var)
% raizes de s^2 + a*Kd*s + (b + a*Kp) para cada Kp  (uma linha por Kp)
  R = zeros(numel(Kp_var), 2);
  for k = 1:numel(Kp_var)
    R(k,:) = roots([1, a*Kd, b + a*Kp_var(k)]).';
  end
end

function [t, y] = degrau(a, b, Kd, Kp, amp, T, kn, N)
% resposta ao degrau de amplitude 'amp' de (Kd*s+Kp)/(s^2+a*Kd*s+b+a*Kp)
% via discretizacao exata (expm) - dispensa o pacote control
  if nargin < 7 || isempty(kn), kn = 1; end
  if nargin < 8, N = 4000; end
  A  = [0 1; -(b + a*Kp), -a*Kd];   B = [0; 1];   C = kn*[Kp, Kd];
  dt = T/N;  t = (0:N)'*dt;
  M  = expm([A, B; zeros(1,3)]*dt);
  Ad = M(1:2,1:2);  Bd = M(1:2,3);
  x  = zeros(2,1);  y = zeros(N+1,1);
  for k = 1:N+1
    y(k) = C*x;
    x    = Ad*x + Bd*amp;
  end
end

function m = metricas(t, y, yss)
% tr 10-90% do valor FINAL, tp e Mp medidos na curva
  m.tr = NaN; m.tp = NaN; m.Mp = 0;
  i10 = find(y >= 0.1*yss, 1);  i90 = find(y >= 0.9*yss, 1);
  if ~isempty(i10) && ~isempty(i90), m.tr = t(i90) - t(i10); end
  [ymax, imax] = max(y);
  if ymax > yss
    m.tp = t(imax);
    m.Mp = 100*(ymax - yss)/yss;
  end
end


% ------------------------------------------------------------------ dados
a = 0.8879;      % 1/I      [1/(kg m^2)]
b = 4.9;         % m*g*l/I  [1/s^2]   <-- CONFERIR: com I = 1.1262 e o CM a
                 %                        0.75 m do pivo, b = 9.8, nao 4.9.
                 %                        Troque aqui e reveja tudo.

Kd_lista = [0, 2, 5, 10, 17.3, 25];     % valores de Kd a analisar
Kp_max   = 400;                          % faixa de varredura de Kp
Kp_var   = linspace(0, Kp_max, 3000);    % varredura fina (desenha o lugar)
Kp_marca = [10 25 50 100 200 400];       % Kp destacados com marcador
qd       = 0.1745;                       % referencia (10 graus) p/ o degrau
% Ganho do numerador. A FT do enunciado traz (Kd*s + Kp) "puro"; fisicamente
% o numerador tambem sai dividido por I, ou seja a*(Kd*s + Kp). Com kn = 1 o
% ganho DC tende a 1/a = %.3f (>1) quando Kp cresce, o que nao existe no
% sistema real. Use kn = a (correto) ou kn = 1 para reproduzir o enunciado.
kn       = a;
Tsim     = 3;                            % horizonte de simulacao [s]

cores = [0.00 0.45 0.70; 0.90 0.42 0.10; 0.00 0.62 0.45; ...
         0.80 0.15 0.35; 0.45 0.35 0.75; 0.35 0.35 0.35];

% =========================================================================
%  FIGURA 1 - todos os lugares das raizes sobrepostos
% =========================================================================
figure(1); clf; hold on; grid on;
for i = 1:numel(Kd_lista)
  Kd = Kd_lista(i);  c = cores(mod(i-1,rows(cores))+1,:);
  R  = lr_raizes(a, b, Kd, Kp_var);
  plot(real(R(:)), imag(R(:)), '.', 'color', c, 'markersize', 4);
  % polos de malha aberta equivalentes (Kp = 0)
  p0 = roots([1, a*Kd, b]);
  plot(real(p0), imag(p0), 'x', 'color', c, 'markersize', 12, 'linewidth', 2);
end
h = zeros(numel(Kd_lista),1);
for i = 1:numel(Kd_lista)
  h(i) = plot(NaN, NaN, '-', 'color', cores(mod(i-1,rows(cores))+1,:), 'linewidth', 3);
end
legend(h, arrayfun(@(k) sprintf('Kd = %.4g', k), Kd_lista, 'uniformoutput', false), ...
       'location', 'eastoutside');
axis([-1.2*a*max(Kd_lista)/2-2, 2, -25, 25]);
grade_zeta_wn(gca);
xlabel('Re(s)  [1/s]'); ylabel('Im(s)  [1/s]');
title(sprintf('Lugar das raizes em Kp (0 a %g), um ramo por Kd   |   a=%.4f  b=%.4f', Kp_max, a, b));

% =========================================================================
%  FIGURA 2 - um subplot por Kd, com Kp destacados e o zero -Kp/Kd
% =========================================================================
figure(2); clf;
nl = ceil(numel(Kd_lista)/3);
for i = 1:numel(Kd_lista)
  Kd = Kd_lista(i);  c = cores(mod(i-1,rows(cores))+1,:);
  subplot(nl, 3, i); hold on; grid on;
  R = lr_raizes(a, b, Kd, Kp_var);
  plot(real(R(:)), imag(R(:)), '.', 'color', c, 'markersize', 4);
  p0 = roots([1, a*Kd, b]);
  plot(real(p0), imag(p0), 'kx', 'markersize', 11, 'linewidth', 2);
  for Kp = Kp_marca
    r = roots([1, a*Kd, b + a*Kp]);
    plot(real(r), imag(r), 'o', 'color', c, 'markerfacecolor', 'w', ...
         'markersize', 7, 'linewidth', 1.5);
    text(real(r(1))+0.3, imag(r(1)), sprintf('%g', Kp), 'fontsize', 8);
    if Kd > 0                       % zero de malha fechada em -Kp/Kd
      plot(-Kp/Kd, 0, 'o', 'color', [.5 .5 .5], 'markersize', 6);
    end
  end
  axis([-max(20, a*Kd), 2, -22, 22]);
  grade_zeta_wn(gca);
  Kp_c = ((a*Kd)^2/4 - b)/a;      % Kp de separacao real <-> complexo
  title({sprintf('Kd = %.4g', Kd), ...
         sprintf('sigma = %.2f   |   Kp_{sep} = %.1f', -a*Kd/2, Kp_c)}, 'fontsize', 9);
  xlabel('Re(s)'); ylabel('Im(s)');
end

% =========================================================================
%  FIGURA 3 - resposta ao degrau (qd) para cada par (Kd, Kp)
% =========================================================================
figure(3); clf;
for i = 1:numel(Kd_lista)
  Kd = Kd_lista(i);
  subplot(nl, 3, i); hold on; grid on;
  for j = 1:numel(Kp_marca)
    Kp = Kp_marca(j);
    [t, y] = degrau(a, b, Kd, Kp, qd, Tsim, kn);
    plot(t, y, 'linewidth', 1.3, 'color', cores(mod(j-1,rows(cores))+1,:));
  end
  plot([0 Tsim], qd*[1 1], 'k--');
  title(sprintf('Kd = %.4g', Kd));
  xlabel('t [s]'); ylabel('theta [rad]');
  if i == 1
    legend(arrayfun(@(k) sprintf('Kp=%g', k), Kp_marca, 'uniformoutput', false), ...
           'location', 'southeast');
  end
end

% =========================================================================
%  TABELA - metricas medidas na propria resposta (nao por formula de 2a
%  ordem: o zero em -Kp/Kd altera Mp e tr)
% =========================================================================
printf('\n a = %.4f   b = %.4f   qd = %.4f rad\n', a, b, qd);
printf('%6s %7s | %8s %7s %9s | %7s %7s %7s %8s\n', ...
       'Kd','Kp','wn[rad/s]','zeta','sigma','tr[ms]','tp[ms]','Mp[%]','e_ss[%]');
printf('%s\n', repmat('-', 1, 80));
for i = 1:numel(Kd_lista)
  Kd = Kd_lista(i);
  for Kp = Kp_marca
    wn    = sqrt(b + a*Kp);
    zeta  = a*Kd/(2*wn);
    [t,y] = degrau(a, b, Kd, Kp, qd, Tsim, kn);
    yss   = qd*kn*Kp/(b + a*Kp);
    m     = metricas(t, y, yss);
    printf('%6.4g %7.4g | %8.3f %7.3f %9.3f | %7.1f %7.1f %7.1f %8.1f\n', ...
           Kd, Kp, wn, zeta, -a*Kd/2, 1e3*m.tr, 1e3*m.tp, m.Mp, 100*(1 - yss/qd));
  end
  printf('%s\n', repmat('-', 1, 80));
end

% =========================================================================
%  PROJETO INVERSO: escolha zeta e wn -> ganhos
%       wn^2 = b + a*Kp      ->  Kp = (wn^2 - b)/a
%       2*zeta*wn = a*Kd     ->  Kd = 2*zeta*wn/a
% =========================================================================
zeta_alvo = 0.7;
tr_alvo   = 0.300;                                    % [s]
beta      = atan2(sqrt(1 - zeta_alvo^2), zeta_alvo);
wn_alvo   = (pi - beta)/(tr_alvo*sqrt(1 - zeta_alvo^2));
Kp_p      = (wn_alvo^2 - b)/a;
Kd_p      = 2*zeta_alvo*wn_alvo/a;
printf('\nProjeto para zeta = %.2f e tr = %.0f ms:\n', zeta_alvo, 1e3*tr_alvo);
printf('  wn = %.3f rad/s  ->  Kp = %.2f   Kd = %.3f\n', wn_alvo, Kp_p, Kd_p);
printf('  erro de regime = %.1f %%  (PD nao zera; use +m*g*l*sin(theta_d) ou termo I)\n', ...
       100*(1 - kn*Kp_p/(b + a*Kp_p)));
if Kp_p <= 0
  printf('  ATENCAO: Kp <= 0, a propria gravidade ja da wn maior que o alvo.\n');
end

