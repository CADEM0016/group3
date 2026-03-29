function draw(GeomObj,massObj,CoG)
arguments
    GeomObj 
    massObj % = cast.MassObj.empty - fml
    CoG (1,2) double
end
%DRAW Summary of this function goes here
%   Detailed explanation goes here
hold on;
for i = 1:length(GeomObj)
    p = GeomObj(i).draw;
end
for i = 1:length(massObj)
    p = massObj(i).draw;
end

% Plot COG
p = plot(CoG(1),CoG(2),'wo',MarkerEdgeColor='k');
p.Annotation.LegendInformation.IconDisplayStyle = "off";
p.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow("Name","CoG");
p.DataTipTemplate.DataTipRows(end+1) = dataTipTextRow("Mass",string(sprintf('%.2f t',sum([massObj.m])/1e3)));
end