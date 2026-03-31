function x = hexsingle2num(s)
%HEXSINGLE2NUM Convert single precision IEEE hexadecimal string to number.
%   HEXSINGLE2NUM(S), where S is a 8 character string containing
%   a hexadecimal number, returns a double type number
%   equal to the IEEE single precision
%   floating point number it represents.  Fewer than 8
%   characters are padded on the right with zeros.
%
%   If S is a character array, each row is interpreted as a single
%   precision number (and returned as a double).
%
%   NaNs, infinities and denorms are handled correctly.  
%
%   Example:
%       hexsingle2num('40490fdb') returns Pi.
%       hexsingle2num('bf8') returns -1.
%
%   See also HEX2NUM.

% Based on Matlab's hex2num.
% Note: IEEE Standard 754 for floating point numbers
%
%  Floating point numbers are represented as:
%  x = +/- (1+f)*2^e
%
%  doubles: 64 bits
%           Bit 63       (1 bit)  = sign (0=positive, 1=negative)
%           Bit 62 to 52 (11 bits)= exponent biased by 1023
%           Bit 51 to 0  (52 bits)= fraction f of the number 1.f
%  singles: 32 bits
%           Bit 31       (1 bit)  = sign (0=positive, 1=negative)
%           Bit 30 to 23 (8 bits) = exponent biased by 127
%           Bit 22 to 0  (23 bits)= fraction f of the number 1.f

% 21 June 2005 Fixed bug with underflow. 
%    Bug found by Matthias Noell (matthias.noell@heidelberg.com)

if iscellstr(s), s = char(s); end
if ~ischar(s)
    error('Input to hexsingle2num must be a string.')
end
if isempty(s), x = []; return, end

[row,col] = size(s);
blanks = find(s==' '); % Find the blanks at the end
if ~isempty(blanks), s(blanks) = '0'; end % Zero pad the shorter hex numbers.

% Convert characters to numeric digits.
% More than 8 characters are ignored
% For double: d = zeros(row,16);
d = zeros(row,8);
d(:,1:col) = abs(lower(s)) - '0';
d = d + ('0'+10-'a').*(d>9);
neg = d(:,1) > 7;
d(:,1) = d(:,1)-8*neg;

if any(d > 15) | any(d < 0)
    error('Input string to hexsingle2num should have just 0-9, a-f, or A-F.')
end

% Floating point exponent.
% For double: e = 16*(16*(d(:,1)-4) + d(:,2)) + d(:,3) + 1;
% For double: e = 256*d(:,1) + 16*d(:,2) + d(:,3) - 1023;
expBit = (d(:,3) > 7);
e = 32*d(:,1) + 2*d(:,2) + expBit - 127;
d(:,3) = d(:,3)-8*expBit;  % Remove most sig. bit of d(:,3) which belongs to exponent

% Floating point fraction.
% For double: sixteens = [16;256;4096;65536;1048576;16777216;268435456];
% For double: sixteens2 = 268435456*sixteens(1:6);
% For double: multiplier = 1./[sixteens;sixteens2];
% For double: f = d(:,4:16)*multiplier;
sixteens = [16;256;4096;65536;1048576;16777216];
multiplier = 2./[sixteens];
f = d(:,3:8)*multiplier;

x = zeros(row,1);
% Scale the fraction by 2 to the exponent.
% For double: overinf = find((e>1023) & (f==0));
overinf = find((e>127) & (f==0));
if ~isempty(overinf), x(overinf) = inf; end

% For double: overNaN = find((e>1023) & (f~=0));
overNaN = find((e>127) & (f~=0));
if ~isempty(overNaN), x(overNaN) = NaN; end

% For double: underflow = find(e<-1022);
underflow = find(e<-126);
if ~isempty(underflow), x(underflow) = pow2(f(underflow),-126); end

% For double: allothers = find((e<=1023) & (e>=-1022));
allothers = find((e<=127) & (e>=-126));
if ~isempty(allothers), x(allothers) = pow2(1+f(allothers),e(allothers)); end

negatives = find(neg);
if ~isempty(negatives), x(negatives) = -x(negatives); end
 
 
