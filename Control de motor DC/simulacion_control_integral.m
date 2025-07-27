clc; clear all; close all;

%Definir el modelo en espacio de estados en su versión Controlable
% G = [0 1; -0.3438 1.245];
% H = [0;1];
% C = [31.58 26.29];
T = 50e-3;

G = [0.6247 0.0249; 1 0];
H = [1;0];
C = [176.4298 265.2702];
%======================================================================
%                       DISENO DEL CONTROLADOR
%======================================================================
%Definir polos a emplear
tss = 0.5;
p1 = -5.0093;
p2 = -3.9668 + 1i*3.7845;
p3 = -3.9668 - 1i*3.7845;

%Nor5malización de polos
p1 = p1/tss;p2=p2/tss;p3=p3/tss;

%Discretizacion de polos
z1 = exp(T*p1);z2=exp(T*p2);z3=exp(T*p3);

%z1 = 0;z2=z1;z3=z1;
%Calculo de matrices cap
Gcap = [G H; 0 0 0];
Hcap = [0;0;1];

%Empleo de algoritmo de Ackerman
Kcap = acker(Gcap, Hcap, [z1 z2 z3]);


%Calcular las gancias K2  y K1
K2K1 = (Kcap + [0 0 1]) * [G - eye(2) H ; C*G C*H]^(-1);
K2 = K2K1(1:2);
K1 = K2K1(end);

%======================================================================
%                  DISEÑO DEL OBSERVADOR DE ESTADOS
%======================================================================
%Polos destinados al observador (T=50ms, deadbeat)
Ke = acker(G', C', [0 0]);


%======================================================================
%                            SIMULACION
%======================================================================

%Tiempo de simulacion
ts = 20;
N = round(ts/T);

%Simulacion

r =2000*ones(1,N+1);   %Valor de referencia
r(round(length(r)/2):end) = 1200*ones(round(length(r)/2),1);

x(:,2) = [0;0];       %Condiciones iniciales de los estados
v(1) = 0;             %Condicion inicial del acumulador
y(2) = 0;             %Condicion inicial de la salida de la planta

xe(:,2) = [0;0];       %Condiciones iniciales del observador
ve(1) = 0;             %Condicion inicial del observador
ye(2) = 0;             %Condicion inicial del observador



for k = 2: N
    %*************SIN OBSERVADOR*******************************
    v(k) = v(k-1) + r(k) - y(k);
    u(k) = -K2*x(:,k) + K1*v(k);
    %Simulacion de la planta
    x(:,k+1) = G*x(:,k) + H*u(k);
    y(k+1) = C*x(:,k);

    %*************CON OBSERVADOR*******************************
    ve(k) = ve(k-1) + r(k) - ye(k);
    ue(k) = -K2*xe(:,k) + K1*ve(k);
    xe(:,k+1) = G*xe(:,k) + H*ue(k) + Ke'*( ye(k) - C*xe(:,k) );
    %Simulacion de la salida dxe la planta
    ye(k+1) = C*xe(:,k+1);
    
end

%Visualización de resultados
t = 0 : T : N*T;
figure()
plot(t,r,'r--','Color','k'); hold on;
plot(t,y,'LineWidth',2,'Color','red');
grid on;
title('Simulación de seguimiento a referencias constantes')
legend('Referencia','Salida del sistema')
xlabel('Tiempo')
ylabel('Amplitud')





