function  graphRobot=discoPlot(dx,dy,dz,color,scaleRobot)
global Robot;

robotParameters; % Carga todos los puntos de la estructura

robotPatch = Robot.hokeyVertices; % Aplicar la matriz de rotacion a los vertices del componente del robot
robotPatch(1,:)=robotPatch(1,:)*scaleRobot*2+dx; %Escalar y dezplazar en el eje x
robotPatch(2,:)=robotPatch(2,:)*scaleRobot*2+dy; %Escalar y dezplazar en el eje y
robotPatch(3,:)=robotPatch(3,:)*scaleRobot+dz; %Escalar y dezplazar en el eje z

graphRobot(1) = patch('Faces',Robot.hokeyFaces,'Vertices',robotPatch','FaceColor',color,'EdgeColor','none'); % Patch dibujar el robot

