function matlab2txt( variable, fileName, flag )
%MATLAB2TXT Creates a txt file from a matlab var to the same precision as
%matlab2opencv.m
S=size(variable);
if length(S)==3
    c=S(1);
    n=S(2);
    m=S(3);
elseif length(S)==2
    m=S(1);
    n=S(2);
else
    sprintf('ERROR');
end
% Write mode as default
if ( ~exist('flag','var') )
    flag = 'w'; 
end

if ( ~exist(fileName,'file') || flag == 'w' )
    % New file or write mode specified 
    file = fopen( fileName, 'w');
else
    % Append mode
    file = fopen( fileName, 'a');
end

if( length(S)==3 )
    for i=1:m
        for j=1:n
            for k=1:c
                fprintf( file, '0x%bx\n', variable(k,j,i));
            end
        end
    end
elseif length(S)==2
    for j=1:n
        for i=1:m
            fprintf( file, '0x%bx\n', variable(i,j));
        end
    end
else
    sprintf('ERROR')
end

fclose(file);
