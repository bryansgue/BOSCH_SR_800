% *************************************************************************
% *********************** CONTROLADOR DINÁMICO ****************************
% ************************* ROBOT SCARA ***********************************
% *************************************************************************
clear all; clc; close all; warning off;

tfin = 170;       % Tiempo de simulación  
To = 0.02;        % Período de muestreo
t = [0:To:tfin];  % Representa la evolución en cada To

%a) Condiciones iniciales h(0, 0.7, 0.32)
    q1 = 0;   %Posición articular del eslabón 1
    q2 = 0;   %Posición articular del eslabón 2
 
    qp1 = 0;  %Velocidad articular del eslabón 1
    qp2 = 0;  %Velocidad articular del eslabón 2
    
    z  = 0; %Altura del extremo operativo

%b) Parámetros del Robot BOSCH SR-800
    l1 = 0.4;   %Distancia del eslabón 1
    l2 = 0.3;   %Distancia del eslabón 2
    h = 0.42;   %Altura de la base fija - primer ealabón
    d = 0.17; %Distancia desde la base fija hacia el primer eslabón

%c) Valores deseados h(0.5, -0.4, 0.0)  
    q1d = [-1.5 ];   % Posición deseadas de la articular del eslabón 1  
    q2d = [ 1.5  ];   % Posición deseadas de la articular del eslabón 2
    
    hzd = [0 ];      % Posición deseadas del extremo operativo en el eje Z

%d)Cinemática Directa Robot - BOSCH SR-800
   %Posiciones inicial del extremo operativo
    hx = l1*cos(q1)+l2*cos(q1+q2);  
    hy = l1*sin(q1)+l2*sin(q1+q2);
    hz = h-0.1-z;
    
    %Posiciones deseadas del extremo operativo
    hxd = d+l1*cos(q1d)+l2*cos(q1d+q2d);  
    hyd = l1*sin(q1d)+l2*sin(q1d+q2d);
    hzd = hzd;
    
% *************************************************************************
% ************************* CONTROLADOR ***********************************
% *************************************************************************
b=1;
for k = 1:length(t)
%1)Errores de control
   %a)Errores de posición articular para q1 y q2
   q1e(k) = q1d(b) - q1(k);
   q2e(k) = q2d(b) - q2(k);
   qe = [q1e(k); q2e(k)]; 
   
   %b)Errores de velocidad articular para q1 y q2
   q1pe(k) = 0 - qp1(k);
   q2pe(k) = 0 - qp2(k);
   qpe = [q1pe(k); q2pe(k)]; 

   %c)Errores de posición en el eje Z  
   hze(k) = hzd(b) - hz(k);

%2)Matrices de Ganancia
   %a)Para errores de posición articular para q1 y q2
   K = [40 0; 0 50];
   D = [30 0; 0 30];

   %b)Para errores de posición en el eje Z
    w = 0.5;
   
%4)Modelo encontrado del roobot
   em = 0.2*rand; % Errores en el modelo Matriz de Inercia M
   ec = 0.2*rand; % Errores en el modelo Matriz de Fuerzas C
   ef = 0.2*rand; % Errores en el modelo Vector f
   
   qp = [qp1(k) qp2(k)]';
   M = em*[1.7277+0.1908*cos(q2(k)) 0.0918+0.0954*cos(q2(k));...
           0.0918+0.0954*cos(q2(k)) 0.9184];
   C = ec*[31.8192-0.0954*sin(q2(k))*qp2(k) -0.0954*sin(q2(k))*(qp1(k)+qp2(k));...
           0.3418*sin(q2(k))*qp1(k) 12.5783];
   f = ef*[1.0256*sign(qp1(k));...
           1.7842*sign(qp2(k))];
      
%4)Ley de Control
   %a)Ley de Control para q1 y q2
    a = K*tanh(2*qe) + D*tanh(2*qpe); 
    T_ref = M*a+C*qp;
    
   %b)Ley de Control para z
   vz = -w*tanh(.6*hze(k)); 
   
%5)Robot - BOSCH SR-800 (T=M*qpp+C*qp+f)
   qp = [qp1(k) qp2(k)]';
   M = [1.7277+0.1908*cos(q2(k)) 0.0918+0.0954*cos(q2(k));...
        0.0918+0.0954*cos(q2(k)) 0.9184];
   C = [31.8192-0.0954*sin(q2(k))*qp2(k) -0.0954*sin(q2(k))*(qp1(k)+qp2(k));...
        0.3418*sin(q2(k))*qp1(k) 12.5783];
   f = [1.0256*sign(qp1(k));...
        1.7842*sign(qp2(k))];
   
   qpp = inv(M)*(T_ref-C*qp-f); %Aceleración articular de salida

   qp1(k+1)= To*qpp(1)+qp1(k);  %Velocidad articular de salida 
   qp2(k+1)= To*qpp(2)+qp2(k);
   
   q1(k+1) = To*qp1(k+1)+q1(k); %Posición articular de salida 
   q2(k+1) = To*qp2(k+1)+q2(k);

   z(k+1) = To*vz+z(k);         %Posición en el eje Z
   
 %6)Cinemática Directa Robot - BOSCH SR-800
   hx(k+1) = d+l1*cos(q1(k+1))+l2*cos(q1(k+1)+q2(k+1));  
   hy(k+1) = l1*sin(q1(k+1))+l2*sin(q1(k+1)+q2(k+1));
   hz(k+1) = h-0.1-z(k+1);
   
 %7)Cambio de posición deseada
   rho(k)= norm([qe; hze(k)]);
   aux = 0.005;
   if rho(k) < aux && b<length(q1d)
       b=b+1; 
   end
   
end

% *************************************************************************
% ************************* ANIMACIÓN *************************************
% *************************************************************************
%a)  Configuración de la animación del robot BOSCH-SR800 
     scene = figure;        % new figure
     tam = get(0,'ScreenSize');
     set(scene,'position',[10 50 1500 900]); % position and size figure in the screen
     xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]')
     axis([-0.15 1 -0.8 0.8 -0.45 0.6]); % Set axis limits 
     axis equal;            % Set axis aspect ratios
     grid minor;            % Display axes grid lines   
     camlight('headlight'); % Iluminación del robot
     material('dull');
     cameratoolbar          % Control de escena por teclado
     
 %b) Animación de movimiento del ROBOT BOSCH-SR800
     scaleRobot = 1;
     R1 = robotPlot(q1(1),q2(1),z(1),scaleRobot);hold on
     D  = discoPlot(hxd(1),hyd(1),hzd(1),'g',scaleRobot);hold on
     H  = plot3(hx(1),hy(1),hz(1),'c','LineWidth',4);
     title('ANIMACIÓN DE MOVIMIENTO - CONTROLADOR DINÁMICO PD')
          
for n = 1:50:length(t)
    drawnow
    delete(R1)
    delete(H)  
    
    R1 = robotPlot(q1(n),q2(n),z(n),scaleRobot);
    H  = plot3(hx(1:n),hy(1:n),hz(1:n),'c','LineWidth',4);
    pause(To)
end
  
% *************************************************************************
% *************************** GRÁFICAS ************************************
%% ************************************************************************
figure
    subplot(2,1,1)
        plot(t,q1e,'y','lineWidth',3); hold on
        plot(t,q2e,'b','lineWidth',3); hold on
        plot(t,hze,'r','lineWidth',3); grid minor
        legend('q_1_e','q_2_e','h_z_e')
        xlabel('Time [s]');ylabel('[ . ]');
        title('Errores de Control')
    subplot(2,1,2)
        plot(t,hx(1:length(t)),'g','lineWidth',3); hold on
        plot(t,hy(1:length(t)),'m','lineWidth',3); hold on
        plot(t,hz(1:length(t)),'c','lineWidth',3); grid minor
        legend('h_x','h_y','h_z')
        xlabel('Time [s]');ylabel('[m]');
        title('Evolución del Extremo Operativo')
    
    