% LOGFFT.M
% 
% MATLAB function to read (sample) values saved from a microcontroller
% memory using Keil uvision 5 and plot FFT magnitude on log scale
% data should be 32-bit floating point saved in
% Intel hex format file
%

function logfft()

fname = input('enter filename ','s');
fs = input('enter sampling frequency in Hz ');
xscale = input('linear (0) or log (1) frequency scale? ');
fid = fopen(fname,'rt');
floatcount = 0;
dummy = fscanf(fid,'%c',1);
if (dummy ~= ':')
    disp('error: initial colon not found');
else
%process data from this file
finished = 0;
while (finished == 0)
    % move to next line
    while (fscanf(fid,'%c',1) ~= ':'); end
    % get number of 32-bit hex values on line
    N = hex2dec(fscanf(fid,'%c',2))/4;
    % read and discard next 6 characters
    fscanf(fid,'%c',6);
    if (N > 0)
        for i=1:N
          % read 8 character hex string and convert to IEEE float 754 single
          hexstring = fscanf(fid,'%c',8);
          reordered(1) = hexstring(7);
          reordered(2) = hexstring(8);
          reordered(3) = hexstring(5);
          reordered(4) = hexstring(6);
          reordered(5) = hexstring(3);
          reordered(6) = hexstring(4);
          reordered(7) = hexstring(1);
          reordered(8) = hexstring(2);
          mydata(floatcount+1)= hexsingle2num(reordered);
          floatcount = floatcount+1;
        end
    else
    finished = 1;
    end
end
end
fclose(fid);          

N = floatcount;
g = abs(fft(mydata));
ff = 0:fs/N:(fs/2-fs/N);
figure(1)
plot(ff,20*log10(g(1:(N/2))),'LineWidth',2.0);
grid on
xlabel('frequency (Hz)','FontSize',12,'FontName','times');
ylabel('magnitude (dB)','FontSize',12,'FontName','times');
if xscale == 1
  set(gca,'FontSize',12,'XScale','log','FontName','times');
else
  set(gca,'FontSize',12,'XScale','lin','FontName','times');
end
tt = 0:1/fs:(N-1)/fs;
figure(2)
str = [num2str(N),' sample values read from file'];
disp(str);
% PTS = input('enter number of sample values to plot ');
PTS = N;
plot(tt(1:PTS),mydata(1:PTS),'LineWidth',2.0);
grid on
xlabel('time (s)','FontSize',12,'FontName','times');
ylabel('sample value','FontSize',12,'FontName','times');
set(gca,'FontSize',12,'FontName','times');
