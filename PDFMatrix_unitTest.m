
%% Position PDF
close all;clc
pdf_obj = PDFMatrix('boundary',[-1 1;-1 1], 'resolution',.1);
pdf_obj.addPositionSample([0, 0.5], 1,1); 
% pdf_obj.addPositionSample([0.5, 0], 0.5,10);  
% pdf_obj.addPositionSample(-[0.5, 0.5],-1,10); 

pdf_obj.getSample(0.9)

pdf_obj.plot(1);

%% Direction PDF
close all;clc
pdf_obj = PDFMatrix('boundary',[-1 1;-1 1], 'resolution',.1);
pdf_obj.addDirectionSample([1, -1],-100,1e2); 
% pdf_obj.addDirectionSample([-1, 0], 1,10); 
% pdf_obj.addDirectionSample(-[0, 1],1,10); 

pdf_obj.getSample(0.8)

pdf_obj.plot(1);

%% Sample PDF
close all;clc
pdf_obj = PDFMatrix('boundary',[-1 1;-1 1], 'resolution',.025);
pdf_obj.addSample([0, 0], 1,[1, 0],0,1e1);  
% pdf_obj.addSample([0, 0.5], 1,[1, 1],-1,10);   
% pdf_obj.addSample([0, -0.5], 1,[1, -1],-1,20); 

pdf_obj.getSample(1)

pdf_obj.plot(1);

%% Collision PDF
close all;clc
pdf_obj = PDFMatrix('boundary',[-1 1;-1 1], 'resolution',.025);
pdf_obj.addSample([0, 0], -10,-[1, 0],-10,1e1);  
% pdf_obj.addSample([0, 0], 1,[1, 1],-10,10);  
% pdf_obj.addSample([0, 0], 1,[1, -1],-10,100);  
% pdf_obj.addSample([0, 0], 1,[0, -1],-10,10);  
% pdf_obj.addSample([0, 0], 1,[0, 1],-10,10);  


pdf_obj.getSample(1)

pdf_obj.plot(1);