function  robotParameters % Funcion que tiene todos los puntos que forman el robot 
load 'brazo.mat' parte1 parte2 parte3 parte4 hokey mesa;  % Carga los valores de cada elemento que compone el robot
global Robot;  % Variable global

Robot.parte1Vertices=parte1.vertices'; 
Robot.parte1Faces=parte1.faces; 

Robot.parte2Vertices=parte2.vertices'; 
Robot.parte2Faces=parte2.faces;

Robot.parte3Vertices=parte3.vertices'; 
Robot.parte3Faces=parte3.faces; 

Robot.parte4Vertices=parte4.vertices'; 
Robot.parte4Faces=parte4.faces; 

Robot.hokeyVertices=hokey.vertices'; 
Robot.hokeyFaces=hokey.faces; 

Robot.mesaVertices=mesa.vertices'; 
Robot.mesaFaces=mesa.faces; 