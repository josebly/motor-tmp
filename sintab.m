prec = @(x) single(x)

n = 512;
nt = 10000;
x = linspace(0, (n-1)/n*2*pi, n);
xt = linspace(0, (nt-1)/nt*2*pi, nt);
stab = prec(sin(x));
ctab = prec(cos(x));

figure(1)
plot(x, stab)


i = floor(xt*n/2/pi)+1;
f = prec(mod(xt*n/2/pi,1));
dx = prec(f*2*pi/n);
sin_est1 = stab(i);
sin_est2 = stab(i) + dx.*ctab(i);
sin_est3 = stab(i) + dx.*ctab(i) - .5*dx.^2.*stab(i);
sin_est4 = stab(i) + dx.*ctab(i) - .5*dx.^2.*stab(i) - 1.0/6.0*dx.^3.*ctab(i);
sin_est5 = stab(i) + dx.*(ctab(i) - .5*dx.*(stab(i) - 1.0/6.0*dx.*ctab(i)));

figure(2)
plot(xt, sin(xt)-sin_est1, xt, sin(xt)-sin_est2, xt, sin(xt)-sin_est3, xt, sin(xt)-sin_est4)

figure(3)
plot(xt, sin(xt)-sin_est4)

figure(4);
plot(xt, prec(sin(prec(xt))) - sin_est4)

eps(single(2*pi))

