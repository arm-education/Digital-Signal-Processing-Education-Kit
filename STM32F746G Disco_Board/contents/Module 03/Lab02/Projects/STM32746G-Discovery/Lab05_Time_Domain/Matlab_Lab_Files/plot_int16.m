% PLOT_INT16.M
% 
% MATLAB function to read (sample) values saved from a microcontroller
% memory using Keil uvision 5 and plot them
% data should be 16-bit integer saved in Intel hex format file
%

function plot_int16()

fname = input('enter filename ','s');
plot_option = input('line (0) or bar (1) graph? ');
fid = fopen(fname,'rt');
int16count = 0;
dummy = fscanf(fid,'%c',1);
if (dummy ~= ':')
    disp('error: initial colon not found');
else
% process data from this file
finished = 0;
while (finished == 0)
    % move to next line
    while (fscanf(fid,'%c',1) ~= ':'); end
    % get number of 16-bit integer values on current line
    % equal to number of bytes of data on current line divided by two
    % as indicated by first two hex digits
    num16bitvalues = hex2dec(fscanf(fid,'%c',2))/2;
    % read and discard next 6 characters (hex digits)
    % indicating address and record type
    fscanf(fid,'%c',6);
    if (num16bitvalues > 0)
        for i=1:num16bitvalues
          % read 4 character hex string and convert to decimal
          hexstring = fscanf(fid,'%c',4);
          reordered(1) = hexstring(3);
          reordered(2) = hexstring(4);
          reordered(3) = hexstring(1);
          reordered(4) = hexstring(2);
          memdata(int16count+1)= hex2dec(reordered);
          % hex2dec assumes unsigned 16-bit integer
          % alter result assuming data is signed 16-bit integer
          if (memdata(int16count+1) > 32767)
             memdata(int16count+1) = memdata(int16count+1)-65536;
          end
          % update total number of integers read from file
          int16count = int16count+1;
        end
    else
      finished = 1;
    end
end
end
fclose(fid);          
set(gcf,'numberTitle','off')
set(gcf,'name','int16 data read from the microcontroller memory in Keil uVision 5')
if (plot_option == 0)
   plot(0:(int16count-1),memdata(1:1:int16count)/2,'LineWidth',2.0);
else
    bar(0:(int16count-1),memdata(1:1:int16count)/2,0.2);
end
grid on
xlabel('n','FontSize',12,'FontName','times');
ylabel('magnitude','FontSize',12,'FontName','times');
