% clc
% clear
% 
% s = serialport("COM5",115200);
% configureTerminator(s,"LF");   % asegurar salto de linea
% flush(s);
% 
% Ts = 50e-03;
% N = 1000;
% 
% Vref = zeros(N,1);
% RPM  = zeros(N,1);
% 
% k = 1;
% 
% while k <= N
% 
%     if s.NumBytesAvailable > 0
% 
%         data = readline(s);
%         values = str2double(split(data,","));
% 
%         if length(values)==2 && all(~isnan(values))
%             Vref(k) = values(1);
%             RPM(k)  = values(2);
%             k = k + 1;
%         end
%     end
% end

% V = [2.369 1.49 0.98 0.89 0.60 0.49];
% d = [4 8 12 15 22 27];
% 
% f = fit(V', d', 'power1')

% clear s
% 
% t = (0:N-1)*Ts;
% 
% figure
% plot(t,Vref,'b')
% hold on
% plot(t,RPM,'r')
% legend("Referencia","Velocidad")


%Control PI DISCRETO DISCRETO

syms z q0 q1 s0 

%Funcion de transferencia
Fdt=tf(149.9,[1 3.834])

%Parametros de control

tsd= 1;
T=0.02;
zita = 0.9;
wn  = 4/(zita*tsd);
wd = wn*sqrt(1-zita^2);
ws = 2*pi/T;
comp=8*wd;

%z_extra = exp(-T * comp); 

%Planta discreta

G_z=c2d(Fdt,T,'zoh')
Gz_zpk = zpk(G_z)

%%Polinomio deseado

s1 = -zita*wn +wd*1i;
s2 = -zita*wn -wd*1i;

mag_z = exp(-T*zita*wn);
fas_z = (T*wd);

[z_re, z_im] = pol2cart(fas_z, mag_z);
pd_z=(z^2-2*mag_z*cos(fas_z)*z+mag_z^2)*(z-0.05);
pd_z=expand(pd_z);
pd_z=vpa(pd_z, 4)



%%Polinomio caracteristico

pc_z=(z-s0)*(z-1)*(z-0.8256)+(q0*z+q1)*6.8204*z;
pc_z=expand(pc_z);
pc_z=vpa(pc_z,4)


% Igualar coeficientes
eqns = coeffs(pc_z - pd_z, z) == 0;
sol = solve(eqns, [q0 q1 s0]);

disp(sol.q0)
disp(sol.q1)
disp(sol.s0)

%%Control posición
mc2=exp(-T*zita*wn)
phic2=T*wd
Fdt2=tf(149.9,[1 3.834 0])
Gz2=c2d(Fdt2,T,'zoh')
[num_dis1,den_dis1]=tfdata(Gz2,'v');
pd2=conv([1 -2*mc2*cos(phic2) mc2^2],[1 -2*mc2*cos(phic2) mc2^2])
%%Constantes
b0_1=num_dis1(1);
b1_1=num_dis1(2);
b2_1=num_dis1(3);
a1_1=den_dis1(2);
a2_1=den_dis1(3);
p1_1= pd2(2);
p2_1= pd2(3);
p3_1= pd2(4);
p4_1= pd2(5);

Az2=[(b1_1-(p1_1*b0_1)) b0_1 0 1;
    (b2_1-(p2_1*b0_1)) b1_1 b0_1 (a1_1-1);
    (-p3_1*b0_1) b2_1 b1_1 (a2_1-a1_1); 
    (-p4_1*b0_1) 0 b2_1 -a2_1];
Bz2=[p1_1+1-a1_1;p2_1-a2_1+a1_1;p3_1+a2_1;p4_1];

val2=Az2\Bz2
%%Denominador Simulación
q_0C2=val2(1);
q_1C2=val2(2);
q_2C2=val2(3);
s0C2=val2(4);