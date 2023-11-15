x = data.VelX.Data(data.VelX.Time>135&data.VelX.Time<143);

Fs = 1/0.008;

N = length(x);
xdft = fft(detrend(x));
xdft = xdft(1:N/2+1);
psdx = (1/(Fs*N)) * abs(xdft).^2;
psdx(2:end-1) = 2*psdx(2:end-1);
freq = 0:Fs/length(x):Fs/2;

plot(freq,abs(xdft),'.')
grid on
title('Periodogram Using FFT')
xlabel('Frequency (Hz)')
ylabel('Power/Frequency (dB/Hz)')