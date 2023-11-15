function spektrum(x,npoint,fs)
%Funktion:  Zeichnen der  FFT des Signals x
%npoint:    anzahl der punkte; muss eine 2er potenz sein und darf nicht
%           größer als die länge von x sein.
%x:         Signalvektor
%fs:        Abtastfrequenz in Hz
%                
                   
n = length(x);
c = fft(x)/n;
amp = 2*abs(c);
amp(1) = amp(1)/2;
m = floor(npoint/2);
area(linspace(0,(m-1)*fs/n,m), amp(1:m))
title('Amplitudenspektrum')
xlabel('Frequenz [Hz]')
ylabel('Amplitude')
end