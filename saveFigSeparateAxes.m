function saveFigSeparateAxes(h,dir)
od = pwd;
if nargin < 2 
    dir = pwd;
end 
if exist(dir,'file')==0
    mkdir(dir);
end

cd(dir);

kk=0;
    for ii = 1:numel(h.Children)
        if strcmp(h.Children(ii).Type,'axes')
            kk = kk+1;
            exportgraphics(h.Children(ii),[h.Name,'ax',num2str(kk),'.png'],...
                'Resolution',400);
        end
    end 
cd(od);

end