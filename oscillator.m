%osc

prec = @(x) single(x)

dt = prec(.01);

t = (0:dt:100000)';

q = prec(zeros(size(t)));
p = prec(zeros(size(t)));
q_dot = prec(zeros(size(t)));
p_dot = prec(zeros(size(t)));
q(1) = 1;
f = prec(.1);
w = 2*pi*f;
for i = 1:length(t)-1
    p_dot(i+1) = -w^2*q(i);
    p(i+1) = p_dot(i+1)*dt + p(i);
    q_dot(i+1) = p(i+1);
    q(i+1) = q_dot(i+1)*dt + q(i);
end

figure(1);
plot(t,q,'.');
ylim([0.9995, 1.0005])