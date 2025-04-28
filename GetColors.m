function C = GetColors(num)

    % Accessible colors for partially colorblind people
    % https://davidmathlogic.com/colorblind
switch num
    case 5 % IBM
        C= [100,143,255;120,94,240;220,38,127;254,97,0;255,176,0]/255;
    case {6,7} % Paul Tol
        C= [51,34,136;17,119,51;136,204,238;221,204,119;204,102,119;170,68,153;136,34,85]/255;
    otherwise
        C = linspecer(num);
end
end