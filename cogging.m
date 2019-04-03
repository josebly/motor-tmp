
data = readtable('cog.csv');

n = 1024;
l = 1024
x = mod(data.Var1,l);
figure(1);
plot(x, data.Var2,'.')

xr = linspace(0,l, n+1);
[~,i] = histc(x, xr);
i = accumarray(i,data.Var2, [], @mean);
figure(2); clf; hold on;
plot(xr(1:end-1), i)

f = fft(i);
[~,i2] = maxk(abs(f), 20); % or should it be limited in frequency?

freq = fftfreq(length(f),1/n);
figure(3);
plot(freq,abs(f));
i2 = abs(freq) < 200;

mf = zeros(size(f));
mf(i2) = f(i2);
mf(1) = 0; % remove average
iq = ifft(mf);
figure(2);
plot(xr(1:end-1), iq)


