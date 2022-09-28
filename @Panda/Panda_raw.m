 %% Raw model of the Panda model
 close all;     
 set(0, 'DefaultFigureWindowStyle', 'docked');
 
 name = 'Panda'; % Robot name

 % DH Parameters
 J1 = Link('d',0.333,'a',0,'alpha',-pi/2,'qlim',[-2.7437 2.7437],'offset',0); 
 J2 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[-1.7837 1.7837],'offset',0); 
 J3 = Link('d',0.316,'a',0,'alpha',pi/2,'qlim',[-2.9007 2.9007], 'offset',0);
 J4 = Link('d',0,'a',0.0825,'alpha',-pi/2,'qlim',[-3.0421 -0.1518],'offset',0);
 J5 = Link('d',0.384,'a',-0.0825,'alpha',pi/2,'qlim',[-2.8065 2.8065], 'offset',0);
 J6 = Link('d',0,'a',0,'alpha',pi/2,'qlim',[0.5445 4.5169], 'offset', 0);
 J7 = Link('d',0,'a',0.088,'alpha',0,'qlim',[-3.0159 3.0159], 'offset', 0);
          
 myPanda = SerialLink([J1 J2 J3 J4 J5 J6 J7],'name',name); 

 q = zeros(1, myPanda.n);
 myPanda.teach(q) % Plot robot with teach UI at joint configuration [0,0,0,0,0,0,0]