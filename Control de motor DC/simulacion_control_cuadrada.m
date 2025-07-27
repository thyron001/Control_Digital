clc; clear; close all; format long;
%%
syms z t
Ts = 0.05;                % Tiempo de muestreo
tss = 0.75;                  % Tiempo de estabilización deseado


%% Dinamica adicional
%Cuadrada 

% Caso 1: T = 1s,  2000 y 4000 RPMs
poles_s = [0, 1j*2*pi, -1j*2*pi, 1j*6*pi, -1j*6*pi, 1j*10*pi, -1j*10*pi,  1j*14*pi, -1j*14*pi];
% ---------- Transformación a dominio Z ----------
z_poles = exp(poles_s * Ts);  % z = e^(s*T)

% ---------- Polinomio característico ----------
char_poly = poly(z_poles);  
deltas = char_poly(2:end);  

m = length(deltas);  % número de estados adicionales

% ---------- Construcción de matrices ----------
Aa = zeros(m);
for i = 1:m
    Aa(i,1) = -deltas(i);
    if i < m
        Aa(i,i+1) = 1;
    end
end

Ba = -deltas(:);  % columna
Ca = [1, zeros(1, m-1)];
Da = 1;

% ---------- Mostrar resultados ----------
% disp('Aa ='); disp(Aa);
% disp('Ba ='); disp(Ba);
% disp('Ca ='); disp(Ca);
% disp('Da ='); disp(Da);


%% PLANTA AUMENTADA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Espacio de estados
G = [0.6414 0.0287; 1.0000 0];

H = [1; 0];

C = [179.1629  251.2905];

D = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
m = size(Aa,1);   % estados del generador
n = 2;            % ejemplo: estados de la planta 


% ---------- Matriz aumentada A_d ----------
Ad = [Aa, zeros(m, n); H*Ca, G];

% ---------- Matriz aumentada B_d ----------
Bd = [Ba; H];

% ---------- Matriz aumentada C_d ----------
Cd = [zeros(1, m), C];

% ---------- Mostrar resultados ----------
disp('A_d ='); disp(Ad);
disp('B_d ='); disp(Bd);
disp('C_d ='); disp(Cd);

%% ******************
%% ******************
%% construcción de Seguimiento de trayectorias con entrada cero
% p1 = (-13.4212 + 0.0000j) / tss;
% p2 = (-13.0638 - 3.7040j) / tss;
% p3 = (-13.0638 + 3.7040j) / tss;
% p4 = (-11.9450 - 7.4558j) / tss;
% p5 = (-11.9450 + 7.4558j) / tss;
% p6 = (-9.8862  -11.3330j) / tss;
% p7 = (-9.8862  +11.3330j) / tss;
% p8 = (-6.3499  -15.5407j) / tss;
% p9 = (-6.3499  +15.5407j) / tss;

%%Polos del filtro de Bessel de orden 11

% Polo real (offset)
p1  = (-54.7744 + 0.0000j) / tss;
p2  = (-53.7819 -12.4829j) / tss;
p3  = (-53.7819 +12.4829j) / tss;
p4  = (-50.7183 -25.0722j) / tss;
p5  = (-50.7183 +25.0722j) / tss;
p6  = (-45.2816 -37.9149j) / tss;
p7  = (-45.2816 +37.9149j) / tss;
p8  = (-36.7612 -51.2869j) / tss;
p9  = (-36.7612 +51.2869j) / tss;
p10 = (-23.2089 -65.9470j) / tss;
p11 = (-23.2089 +65.9470j) / tss;

% Polinomio característico en Z
p = (z - exp(Ts*p1))*(z - exp(Ts*p2))*(z - exp(Ts*p3))*(z - exp(Ts*p4))*(z - exp(Ts*p5))*(z - exp(Ts*p6))*(z - exp(Ts*p7))*(z - exp(Ts*p8))*(z - exp(Ts*p9))*(z - exp(Ts*p10))*(z - exp(Ts*p11));

% Convertir a coeficientes numéricos
p = sym2poly(p);      % Polinomio del sistema en lazo cerrado

%Ecuacion caracteristica Ad
[r, c] = size(Ad);
Aux = zeros(r,c);

for i = 1 : length(p)
    Aux = p(i)*Ad^(length(p)-i) + Aux;          
end

%Matriz de controlabilidad
Mc = ctrb(Ad, Bd);
%Matriz de Observabilidad
Wo = obsv(Ad, Cd);

%Formula de Ackermann
E = place(Ad, Bd, roots(p));
E = E';

%%SEÑAL Cuadrada S.F N = 4, 1seg
pt1 = 3000 + (4000/pi)*( sin(2*pi*t) + (1/3)*sin(6*pi*t) + (1/5)*sin(10*pi*t) + (1/7)*sin(14*pi*t) );
%%SEÑAL cuadrada S.F (3 seg)
%pt1 = 3000 + (4*2000/pi)((1/1)*sin((2*pi/3)*t) + (1/3)*sin(3(2*pi/3)t) + (1/5)*sin(5(2*pi/3)t)+  (1/7)*sin(7(2*pi/3)*t));   


for i = 0 : length(Bd) - 1                        %Evaluacion de la funcion 
    rho1(i+1) = subs(pt1,'t',Ts*i);               %generadora para calculo de
end 

xz0 = Wo^(-1)*rho1';                              %Vector inicial

%%                  OBSERVADOR DE ESTADOS
%======================================================================
%Ganancia del observador con polos tipo deadbit

Ke = acker(G', C', [0 0]);

%% *********Simulacion de la respuesta******

wc = 60;    % Frecuencia de corte recomendada

% Bessel de orden 2 (motor)
[~, pb, ~] = besselap(2);
pd_e = exp(pb * wc * Ts);
Ke_e = acker(G', C', pd_e);

% Bessel de orden 9 (trayectoria cuadrada)
[~, p9, ~] = besselap(9);
pd_a = exp(p9 * wc * Ts);
Ke_a = acker(Aa', Ca', pd_a)/100;


xe0 = [0;0];                    % Estimación inicial de estados planta
xa0 = zeros(9,1);                        % Estados iniciales del generador
xd0 = [xa0; xe0];                        % Estado inicial del sistema aumentado
xd(:,1) = xd0;

ftime = 10;                             
N = floor(ftime/Ts);                     % Número total de muestras

% Inicialización de vectores
xe(:,1) = xe0;                            % Estados estimados planta
xz(:,1) = xz0;                            % Estados del generador
xa(:,1) = xa0;                            % Estados del generador
ye = zeros(1,N);
ya = zeros(1,N);
u = zeros(1,N);

% Bucle de simulación
for k = 1: N - 1
    xd(:,k) = [xa(:,k); xe(:,k)];                     % Estados del sistema aumentado (usando estimado)
    xz(:,k+1) = Ad * xz(:,k);                         % Dinámica del generador
    u(k) = E' * (xz(:,k) - xd(:,k));                  % Ley de control
    
    ya(k) = (Ca * xa(:,k) +  u(k));                 % Entrada a la planta
    
    % Estimación de los estados de la planta
    xe(:,k+1) = G * xe(:,k) + H * ya(k) + Ke_e' * (ye(k) - C * xe(:,k)) ;
    
    %xa(:,k+1) = (Aa - Ke_a * Ca) * xa(:,k) + Ba * u(k) + Ke_a * ya(k);

   xa(:,k+1) = Aa * xa(:,k) + Ba * u(k) + Ke_a'*(ya(k)-Ca*xa(:,k));             % Dinámica del generador

    ye(k+1) = C * xe(:,k+1);                          % Salida medida       
end

% Última muestra
xd(:,N) = [xa(:,N); xe(:,N)];
u(N) = E' * (xz(:,N) - xd(:,N));
ye(N) = C * xe(:,N);
ya(N) = Ca * xa(:,N) + Da * u(N);
t = Ts * (0:N-1);

r = 3000 + (4000/pi)*( sin(2*pi*t) + (1/3)*sin(6*pi*t) + (1/5)*sin(10*pi*t) + (1/7)*sin(14*pi*t) );
%r = 3000 + (4*2000/pi)((1/1)*sin((2*pi/3)*t) + (1/3)*sin(3(2*pi/3)t) + (1/5)*sin(5(2*pi/3)t) + (1/7)*sin(7(2*pi/3)*t));  %cuad, 3 seg


subplot(2,1,1); plot(t,ye,'b',t,r,'r--');
title('Simulacion de una referencia tipo rectangular con observador');
xlabel('Tiempo (s)'); ylabel('Salida de la Planta y(k)');ylim([1000 5000])
grid;
subplot(2,1,2); plot(t,ya,'m');
xlabel('Tiempo (s)'); ylabel('Entrada de la Planta ya(k)');ylim([0 6])
grid on