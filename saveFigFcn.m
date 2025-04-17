function saveFigFcn(hf,dir)
od = pwd;
if nargin < 2 
    dir = pwd;
end 
if exist(dir,'file')==0
    mkdir(dir);
end

cd(dir);
for ii = 1:numel(hf)
    saveas(hf(ii),hf(ii).Name,'fig');
    exportgraphics(hf(ii),[hf(ii).Name,'.png'],'Resolution',400);
    %saveas(hf,hf(ii).Name,'png');
    
end 
cd(od);

end 