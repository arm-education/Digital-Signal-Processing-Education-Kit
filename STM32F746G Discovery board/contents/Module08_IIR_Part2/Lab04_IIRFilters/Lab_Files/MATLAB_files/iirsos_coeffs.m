% IIRSOS_COEFFS.M
% 
% MATLAB function to write SOS IIR filter coefficients
% in format suitable for use in DSP LiB programs
% including iirsos_intr.c, iirsosprn_intr.c and
% iirsosdelta_intr.c
% assumes that coefficients have been exported from
% fdatool as two matrices
% first matrix has format
% [ b10 b11 b12 a10 a11 a12
%   b20 b21 b22 a20 a21 a22
%   ...
% ] 
% where bij is the bj coefficient in the ith stage
% second matrix contains gains for each stage
%
function iirsos_coeffs(coeff,gain)
%
num_sections=length(gain)-1;
fname = input('enter filename for coefficients ','s');
fid = fopen(fname,'wt');
fprintf(fid,'// %s\n',fname);
fprintf(fid,'// this file was generated using');
fprintf(fid,'\n// function iirsos_coeffs.m\n',fname);
fprintf(fid,'\n#define NUM_SECTIONS %d\n',num_sections);
% first write the numerator coefficients b
% i is used to count through sections
fprintf(fid,'\nfloat b[NUM_SECTIONS][3] = { \n');
for i=1:num_sections
  if i==num_sections
    fprintf(fid,'{%2.8E, %2.8E, %2.8E} };\n',...
    coeff(i,1)*gain(i),coeff(i,2)*gain(i),coeff(i,3)*gain(i));
  else
    fprintf(fid,'{%2.8E, %2.8E, %2.8E},\n',...
    coeff(i,1)*gain(i),coeff(i,2)*gain(i),coeff(i,3)*gain(i));
  end
end
% then write the denominator coefficients a
% i is used to count through sections
fprintf(fid,'\nfloat a[NUM_SECTIONS][3] = { \n');
for i=1:num_sections
  if i==num_sections
    fprintf(fid,'{%2.8E, %2.8E, %2.8E} };\n',...
    coeff(i,4),coeff(i,5),coeff(i,6));
  else
    fprintf(fid,'{%2.8E, %2.8E, %2.8E},\n',...
    coeff(i,4),coeff(i,5),coeff(i,6));
  end
end
fclose(fid);          
