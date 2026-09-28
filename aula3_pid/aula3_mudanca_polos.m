a  = 0.8879;
b  = 4.9;
Kd = [0 2 5 10 17.3 25];
Kp = linspace(0, 400, 2000);

figure(1); clf; hold on; grid on;
for i = 1:numel(Kd)
  r = zeros(numel(Kp), 2);
  for k = 1:numel(Kp)
    r(k,:) = roots([1, a*Kd(i), b + a*Kp(k)]).';
  end
  h(i) = plot(real(r(:)), imag(r(:)), '.', 'markersize', 5);
  leg{i} = sprintf('Kd = %g', Kd(i));
end
legend(h, leg, 'location', 'eastoutside');
xlabel('Re(s)'); ylabel('Im(s)');
title('Lugar das raizes variando Kp');
axis([-15 2 -25 25]);
