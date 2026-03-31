% PLOT_COMPLEX.M
% 
%
% MATLAB function to read (sample) values saved from STM32F4
% memory using Keil uvision 5 and plot FFT magnitude on log scale
% data should be 32-bit floating point saved in
% Intel hex format file
%

function plot_complex()

fname = input('enter filename ','s');
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
          memdata(floatcount+1)= hexsingle2num(reordered);
          floatcount = floatcount+1;
        end
    else
    finished = 1;
    end
end
end
fclose(fid);          
N = floatcount;
figure;
set(gcf,'numberTitle','off')
set(gcf,'name','complex data read from the microcontroller memory in Keil uVision 5')
subplot(2,1,1);
plot(0:(N/2-1),memdata(1:2:N),'LineWidth',2.0);
grid on
xlabel('n','FontSize',12,'FontName','times');
ylabel('real','FontSize',12,'FontName','times');
subplot(2,1,2);
plot(0:(N/2-1),memdata(2:2:N),'LineWidth',2.0);
grid on
xlabel('n','FontSize',12,'FontName','times');
ylabel('imaginary','FontSize',12,'FontName','times');