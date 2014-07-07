function [  ] = matlab2txt2( variable, fileName, flag )
%MATLAB2TXT2 Summary of this function goes here
%   Detailed explanation goes here
file = fopen( fileName, 'w');
for i=1:numel(variable)
    fprintf( file, '%.16f\n', variable(i) );
end
fclose(file);

