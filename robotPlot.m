function  graphRobot=robotPlot(q1,q2,z,scaleRobot)
global Robot;

robotParameters; % Carga todos los puntos de la estructura

h = 0.42;
x  = 0.17;
l1 = 0.4;
l2 = 0.3;



robotPatch = Robot.parte1Vertices; % Aplicar la matriz de rotacion a los vertices del componente del robot
robotPatch(1,:)=robotPatch(1,:)*scaleRobot; %Escalar y dezplazar en el eje x
robotPatch(2,:)=robotPatch(2,:)*scaleRobot; %Escalar y dezplazar en el eje y
robotPatch(3,:)=robotPatch(3,:)*scaleRobot; %Escalar y dezplazar en el eje z

graphRobot(1) = patch('Faces',Robot.parte1Faces,'Vertices',robotPatch','FaceColor',[0.4 0.4 0.4],'EdgeColor','none'); % Patch dibujar el robot

% Matriz de rotación
Rz=[cos(q1) -sin(q1) 0; sin(q1) cos(q1) 0; 0 0 1];

robotPatch = Rz*Robot.parte2Vertices; % Aplicar la matriz de rotacion a los vertices del componente del robot
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x; %Escalar y dezplazar en el eje x
robotPatch(2,:)=robotPatch(2,:)*scaleRobot; %Escalar y dezplazar en el eje y
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+h; %Escalar y dezplazar en el eje z

graphRobot(2) = patch('Faces',Robot.parte2Faces,'Vertices',robotPatch','FaceColor','g','EdgeColor','none'); % Patch dibujar el robot

% Matriz de rotación
Rz=[cos(q1+q2) -sin(q1+q2) 0; sin(q1+q2) cos(q1+q2) 0; 0 0 1];

robotPatch = Rz*Robot.parte3Vertices; % Aplicar la matriz de rotacion a los vertices del componente del robot
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x+l1*cos(q1); %Escalar y dezplazar en el eje x
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+l1*sin(q1); %Escalar y dezplazar en el eje y
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+h; %Escalar y dezplazar en el eje z

graphRobot(3) = patch('Faces',Robot.parte3Faces,'Vertices',robotPatch','FaceColor',[0 0.321 0.129],'EdgeColor','none'); % Patch dibujar el robot

robotPatch = Rz*Robot.parte4Vertices; % Aplicar la matriz de rotacion a los vertices del componente del robot
robotPatch(1,:)=robotPatch(1,:)*scaleRobot+x+l1*cos(q1)+l2*cos(q1+q2); %Escalar y dezplazar en el eje x
robotPatch(2,:)=robotPatch(2,:)*scaleRobot+l1*sin(q1)+l2*sin(q1+q2); %Escalar y dezplazar en el eje y
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+h-z; %Escalar y dezplazar en el eje z

graphRobot(4) = patch('Faces',Robot.parte4Faces,'Vertices',robotPatch','FaceColor','y','EdgeColor','none'); % Patch dibujar el robot


robotPatch = Robot.mesaVertices; % Aplicar la matriz de rotacion a los vertices del componente del robot
robotPatch(1,:)=robotPatch(1,:)*scaleRobot; %Escalar y dezplazar en el eje x
robotPatch(2,:)=robotPatch(2,:)*scaleRobot*2; %Escalar y dezplazar en el eje y
robotPatch(3,:)=robotPatch(3,:)*scaleRobot-0.009; %Escalar y dezplazar en el eje z

graphRobot(5) = patch('Faces',Robot.mesaFaces,'Vertices',robotPatch','FaceColor',[0.5 0.25 0],'EdgeColor','none'); % Patch dibujar el robot

