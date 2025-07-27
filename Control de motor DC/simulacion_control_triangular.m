clc; clear; close all;
syms z t
Ts = 0.05;                % Tiempo de muestreo
tss = 0.70;                  % Tiempo de estabilización deseado


% Dinamica adicional

% poles_s = [0, 2i*pi, -2i*pi, 6i*pi, -6i*pi];  % Polos en dominio s
%Triangular periodo 2seg
poles_s = [0, i*pi, -i*pi, 3i*pi, -3i*pi];  % Polos en dominio s
% ---------- Transformación a dominio Z ----------
z_poles = exp(poles_s * Ts);  % z = e^(s*T)

% ---------- Polinomio característico ----------
char_poly = poly(z_poles);  % devuelve [1, delta1, delta2, ..., delta_m]
deltas = char_poly(2:end);  % elimina el coeficiente líder (1)

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

% PLANTA AUMENTADA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Espacio de estados
G = [0.6247 0.0249; 1 0];
H = [1;0];
C = [176.4298 265.2702];

D = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
m = size(Aa,1);   % estados del generador
n = 2;            % n estados de la planta


% ---------- Matriz aumentada A_d ----------
Ad = [Aa, zeros(m, n); H*Ca, G];

% ---------- Matriz aumentada B_d ----------
Bd = [Ba; H];

% ---------- Matriz aumentada C_d ----------
Cd = [zeros(1, m), C];

% ---------- Mostrar resultados ----------
% disp('A_d ='); disp(Ad);
% disp('B_d ='); disp(Bd);
% disp('C_d ='); disp(Cd);


% construcción de Seguimiento de trayectorias con entrada cero

% Polos del filtro de Bessel de orden 7
% Polo real primero (offset)
% p1 = (-8.0271)/tss;
% 
% % Pares conjugados del filtro Bessel orden 7
% p2 = (-4.9435 + 7.9854j)/tss;
% p3 = (-4.9435 - 7.9854j)/tss;
% p4 = (-7.4034 + 5.0133j)/tss;
% p5 = (-7.4034 - 5.0133j)/tss;
% p6 = (-8.7097 + 1.9065j)/tss;
% p7 = (-8.7097 - 1.9065j)/tss;

p1 = -59.6030070801 + 120.3010210145j;
p2 = -59.6030070801 - 120.3010210145j;
p3 = -90.3282652091 + 78.0563556663j;
p4 = -90.3282652091 - 78.0563556663j;
p5 = -105.6003520983 + 38.5998331477j;
p6 = -105.6003520983 - 38.5998331477j;
p7 = -110.3384586779 + 0.0000000000j;

p1 = p1/tss;
p2 = p2/tss;
p3 = p3/tss;
p4 = p4/tss;
p5 = p5/tss;
p6 = p6/tss;
p7 = p7/tss;
% Polinomio característico en Z
p = (z - exp(Ts*p1))*(z - exp(Ts*p2))*(z - exp(Ts*p3))*(z - exp(Ts*p4))*(z - exp(Ts*p5))*(z - exp(Ts*p6))*(z - exp(Ts*p7));

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
E = E'


pt1 = 1.5*(1550 + 1173.5*sin(pi*t) - 130.4*sin(3*pi*t));   %%SEÑAL TRIANGULAR S.F (2 seg)
%pt1 = 1.2*(2500 + 1214.0*sin(2*pi*t) - 134.9*sin(6*pi*t)); %%SEÑAL TRIANGULAR S.F (1 seg) 

for i = 0 : length(Bd) - 1                        %Evaluacion de la funcion 
    rho1(i+1) = subs(pt1,'t',Ts*i);               %generadora para calculo de
end                                               %vector inicial


      %            OBSERVADOR DE ESTADOS
%======================================================================
%Ganancia del observador con polos tipo deadbit

Ke = acker(G', C', [0 0]);

wc = 60;
%DISENO EXTRA
[~, pb, ~] = besselap(2);
pd_e = exp(pb * wc * Ts);
Ke_e = acker(G', C', pd_e)

%BESSEL DE ORDEN 5
[~, p5, ~] = besselap(5);
pd_a = exp(p5 * wc * Ts);
Ke_a = acker(Aa', Ca', pd_a)/50


xz0 = Wo^(-1)*rho1';

xe0 = [0;0];                    % Estimación inicial de estados planta
xa0 = zeros(5,1);                        % Estados iniciales del generador
xd0 = [xa0; xe0];                        % Estado inicial del sistema aumentado
xd(:,1) = xd0;

ftime = 10;                             
N = floor(ftime/Ts);                     % Número total de muestras

%Inicialización de vectores
xe(:,1) = xe0;                            % Estados estimados planta
xz(:,1) = xz0;                            % Estados del generador
xa(:,1) = xa0;                            % Estados del generador
ye = zeros(1,N);
ya = zeros(1,N);
u = zeros(1,N);
% % Bucle de simulación

for k = 1: N - 1
    xd(:,k) = [xa(:,k); xe(:,k)];                     % Estados del sistema aumentado (usando estimado)
    xz(:,k+1) = Ad * xz(:,k);                         % Dinámica del generador
    u(k) = E' * (xz(:,k) - xd(:,k));                  % Ley de control    
    ya(k) = Ca * xa(:,k) + Da * u(k);                 % Entrada a la planta
    % Estimación de los estados de la planta
    xe(:,k+1) = G * xe(:,k) + H * ya(k) + Ke_e' * (ye(k) - C * xe(:,k)) ;
    xa(:,k+1) = Aa * xa(:,k) + Ba * u(k);             % Dinámica del generador
    ye(k+1) = C * xe(:,k+1);                          % Salida simulada
    
   
end

% Última muestra
xd(:,N) = [xa(:,N); xe(:,N)];
u(N) = E' * (xz(:,N) - xd(:,N));
ye(N) = C * xe(:,N);
ya(N) = Ca * xa(:,N) + Da * u(N);
t = Ts * (0:N-1);

% r = 1.2*(2500 + 1214.0*sin(2*pi*t) - 134.9*sin(6*pi*t)); %triang, 1 seg
r = 1.5*(1550 + 1173.5*sin(pi*t) - 130.4*sin(3*pi*t));  %triang, 2 seg
% r = 0.81056*cos(pi*t) - 0.09006*cos(3*pi*t);
% r = 2500*(r + 1);
% r = 2500 + 1214.0*sin((pi/2)*t) - 134.9*sin((3*pi/2)*t);

figure(1)
subplot(2,1,1); 
plot(t,ye,'b', t, r,'r--', 'LineWidth', 1.5);

title('Simulacion de una referencia tipo triangular con observador');
xlabel('Tiempo (s)'); ylabel('Salida de la Planta y(k)');ylim([0 7000])
grid on;

subplot(2,1,2); 
plot(t,ya,'m');

xlabel('Tiempo (s)'); ylabel('Entrada de la Planta ya(k)');ylim([0 6])
grid on;