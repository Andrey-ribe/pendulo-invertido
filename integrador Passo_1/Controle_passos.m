clear all
close all

pkg load control
pkg load signal

passo=4
arquivo = ["passo_" num2str(passo) "cm.txt"];
data = load('-ascii',arquivo);
%TESTE 3  passo na planta de -0,04
%TESTE 4  passo na planta de -0,02


T = 0.008224
max_t=(length(data)-1)*T;
refer=2*passo/100;
t = 0:T:max_t;


tempo= (data(:,1)-502)*T;
posicao= data(:,2)*((pi*0.024)/1600);
angulo= data(:,3)*((2*pi)/4096);
velocidade = data(:,4);
velocidade_angular= data(:,5);
saida = data(:,6);
target = data(:,7);






x0 = [0;0;0;0];

m=0.1;
M=1.015;
g=9.806;
l=0.45;
r1=0.012;

Kt= 0.16 %0.0975324295328197 %!
Ra= 2.11768518482399
Kb=Kt; %0.0975324295328197;

Kr=Kt/Ra
c1=(Kr*Kb)/(r1^2);
c2=Kr/r1;

A22=-(m*g)/M;
A23=-c1/M;
A32=((M+m)*g)/(M*l);
A33=c1/(M*l);

B13=c2/M;
B14=-c2/(M*l);



A=[0 0 1 0; 0 0 0 1; 0 A22 A23 0; 0 A32 A33 0]
B=[0 0 B13 B14]'
C=[1 0 0 0]
C_ang=[0 1 0 0]


sys = ss(A,B,C);
sys_ang = ss(A,B,C_ang);

G0=tf(sys);
G1=tf(sys_ang);

sysD = c2d(sys,T);
sysD_ang = c2d(ss(A,B,C_ang),T);

A_d = sysD.a;
B_d = sysD.b;
C_d = sysD.c;



##Q=[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
##R=1;
##K=lqr(sysD,Q,R);


A_d_hat = [A_d 0.000*ones(4,1);  -C_d 0];
B_d_hat = [B_d; 0 ];
C_d_hat = [C 0];
%A_d_hat = [A_d zeros(4,2);  -1 0 0 0 0 0; 0 -1 0 0 0 0];
%B_d_hat = [B_d; 0; 0 ];
%C_d_hat = [C 0 0];
sysD_hat = ss(A_d_hat,B_d_hat,C_d_hat,0,T);

%Q_hat=[1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
Q_hat=[1 0 0 0 0; 0 1 0 0 0; 0 0 0 0 0; 0 0 0 0 0 ; 0 0 0 0 1];
R_hat=10;
[g,x,l]= dlqr(sysD_hat,Q_hat,R_hat)


F=Q_hat;
W=0;
P=Q_hat*0;

for k = 1:10000

W=R_hat+B_d_hat'*F*B_d_hat;
P=F-F*B_d_hat*inverse(W)*B_d_hat'*F;
F=A_d_hat'*P*A_d_hat+Q_hat;

endfor

Ktk=inverse(W)*B_d_hat'*F*A_d_hat;

K=[Ktk(1) Ktk(2) Ktk(3) Ktk(4)];

AA=A_d-B_d*K;
BB=B_d*K(1);
CC=C_d;
sysf=ss(AA,BB,CC);


x = x0;
y = C_d*x0;
u = zeros(1,length(t));
x_vel=0;


zm = abs(min(eig(A_d-B_d*K)));
talzm = -T/log(zm);
oz = (e^(-T/talzm))/2;
oc = [oz;oz;oz;oz];

L = acker(A_d',C_d',oc)';


Ac=A_d-L*C_d-B_d*K;
Bc=L;
Cc=K;

sysf2=ss(Ac,Bc,Cc);

Aaa=A_d(1:2,1:2);
Aab=A_d(1:2,3:4);
Aba=A_d(3:4,1:2);
Abb=A_d(3:4,3:4);

Ba=B_d(1:2);
Bb=B_d(3:4);
Ke=place(Abb',Aab',[oz oz])';



x = x0;
y = [0 0]';
u = 0;
xv=zeros(2,length(t));
uO=zeros(1,length(t));


Aa=(Abb-Ke*Aab);
Ab=(Aba-Ke*Aaa);
Bp=(Bb-Ke*Ba);


##K=  [-2.2361  -58.5881  -26.0324  -12.5513];
##Aa= [ 8.6167e-01  -2.5507e-05 ;2.8473e-01   8.5628e-01];
##Ab= [-1.0952e-04  -1.3351e-02 ;-1.0633e-02  -7.0152e+01];
##Bp= [0.011066  -0.022780]';

err=zeros(1,length(t));
dif=0;
tmp_degrau=0;
integrador=0;

limite=ones(1,length(t))*refer;

for k = 2:length(t)


x(:,k) = A_d*x(:,k-1) + B_d*uO(k-1);
y(1,k) = C_d*x(:,k);
y(2,k) = C_ang*x(:,k);


xv(:,k) = Aa*xv(:,k-1) + Bp*uO(k-1) + Ke*y(:,k) + Ab*(y(:,k-1));

##xv(1,k)=Aa(1,1)*xv(1,k-1)+Aa(1,2)*xv(2,k-1)+Bp(1)*uO(k-1) +Ke(1,1)*y(1,k)+Ke(1,2)*y(2,k)+Ab(1,1)*y(1,k-1)+Ab(1,2)*y(2,k-1);
##xv(2,k)=Aa(2,1)*xv(1,k-1)+Aa(2,2)*xv(2,k-1)+Bp(2)*uO(k-1) +Ke(2,1)*y(1,k)+Ke(2,2)*y(2,k)+Ab(2,1)*y(1,k-1)+Ab(2,2)*y(2,k-1);

dif(k)=target(k)-y(1,k);
integrador(k)=integrador(k-1)+(T/2)*(dif(k)*T+dif(k-1));

uO(k) = (-(1*integrador(k) + K(1)*y(1,k)+ K(2)*y(2,k) + K(3)*xv(1,k) + K(4)*xv(2,k)));



##  tmp_degrau=tmp_degrau+1;
##  if(tmp_degrau<500)
##  target=0;
##  endif
##  if(tmp_degrau>500&&tmp_degrau<(40/T))
##  target=refer;
##  endif
##  if(tmp_degrau>(40/T))
##  target=-refer;
##  endif
##  if(tmp_degrau>2*(40/T))
##  tmp_degrau=0;
##  endif

endfor


nome_svg1 = ["./" num2str(passo) "_figura1.svg"];
nome_svg2 = ["./" num2str(passo) "_figura2.svg"];
nome_svg3 = ["./" num2str(passo) "_figura3.svg"];
nome_svg4 = ["./" num2str(passo) "_figura4.svg"];
nome_simul_svg1 = ["./simul_" num2str(passo) "_figura1.svg"];
nome_simul_svg2 = ["./simul_" num2str(passo) "_figura2.svg"];
nome_simul_svg3 = ["./simul_" num2str(passo) "_figura3.svg"];

figure;
##subplot(2,1,1)
plot(t,100*target,'k')
hold on
##plot(tempo,100*posicao,'r')
##hold on
plot(t,100*y(1,:),'b')
xlim ([0 max_t])
ylim ([150*min(posicao) 170*max(posicao)])
hold off
grid on
pbaspect ([1 0.5 1]);
ylabel("Posição [cm]");
xlabel("Tempo [s]");
legend ({"Referência", "Simulado"}, "location", "northeast");
print( gcf, '-dsvg',"-S640,320", nome_simul_svg1);


figure;
##subplot(2,1,1)
plot(t,100*target,'k')
hold on
plot(tempo,100*posicao,'r')
hold on
plot(t,100*y(1,:),'b')
xlim ([0 max_t])
ylim ([150*min(posicao) 170*max(posicao)])
hold off
grid on
pbaspect ([1 0.5 1]);
##title("Saída de Posição")
ylabel("Posição [cm]");
xlabel("Tempo [s]");
legend ({"Referência","Real", "Simulado"}, "location", "northeast");
##subplot(2,1,2)
##plot(t,100*x(3,:))
##hold on
##plot(tempo,100*velocidade,'r')
##xlim ([0 max_t])
##ylim ([100*min(velocidade) 100*max(velocidade)])
##hold off
##grid on
##title("Saída de Velocidade")
##ylabel("Velocidade [cm/s]");
##xlabel("Tempo [s]");
##legend ({"Teórico", "Real"}, "location", "northeast");
print( gcf, '-dsvg',"-S640,320", nome_svg1);

figure;
##subplot(2,1,1)
plot(t,y(2,:)*(180/pi),'b')
xlim ([0 max_t])
hold off
grid on
pbaspect ([1 0.5 1]);
##title("Saída de Ângulo")
ylabel("Ângulo [°]");
xlabel("Tempo [s]");
legend ({"Simulado"}, "location", "northeast");
print( gcf, '-dsvg',"-S640,320", nome_simul_svg2);


figure;
##subplot(2,1,1)
plot(tempo,angulo*(180/pi),'r')
hold on
plot(t,y(2,:)*(180/pi),'b')
xlim ([0 max_t])
hold off
grid on
pbaspect ([1 0.5 1]);
##title("Saída de Ângulo")
ylabel("Ângulo [°]");
xlabel("Tempo [s]");
legend ({"Real", "Simulado"}, "location", "northeast");
##subplot(2,1,2)
##plot(t,x(4,:))
##hold on
##plot(tempo,angulo,'r')
##xlim ([0 max_t])
##hold off
##grid on
##title("Saída de Velocidade Ângular")
##ylabel("Ângulo [rad/s]");
##xlabel("Tempo [s]");
##legend ({"Teórico", "Real"}, "location", "northeast");
print( gcf, '-dsvg',"-S640,320", nome_svg2);

figure;
plot(t,uO,'b')
xlim ([0 max_t])
hold off
grid on
pbaspect ([1 0.5 1]);
##title("Ação de Controle")
ylabel("Tensão [V]");
xlabel("Tempo [s]");
legend ({"Simulado" }, "location", "northeast");
print( gcf, '-dsvg',"-S640,320", nome_simul_svg3);


figure;
##subplot(3,1,3)
%plot(tempo,integr)
%hold on
plot(tempo,saida,'r')
hold on
plot(t,uO,'b')
xlim ([0 max_t])
hold off
grid on
pbaspect ([1 0.5 1]);
##title("Ação de Controle")
ylabel("Tensão [V]");
xlabel("Tempo [s]");
legend ({"Real","Simulado" }, "location", "northeast");
print( gcf, '-dsvg',"-S640,320", nome_svg3);





Nfft=length(posicao);
fvec= (-Nfft/2:Nfft/2-1)*(1/(T))/Nfft;
Xpos=fftshift(fft(posicao,Nfft));
Xang=fftshift(fft(angulo,Nfft));





figure
subplot(2,1,1)
plot(fvec,abs(Xpos))
xlabel("Frequência [Hz]");
grid on
xlim ([0 10])
subplot(2,1,2)
plot(fvec,abs(Xang))
xlabel("Frequência [Hz]");
grid on
xlim ([0 10])
##print( gcf, '-dsvg', "FFT.svg");

figure
ax=plotyy(tempo,100*posicao,tempo,angulo*(180/pi))
grid on
xlim ([54 55])
ylim (ax(1),[3 6])
ylim (ax(2),[-2 2])
ylabel(ax(1),"Posição [cm]");
ylabel(ax(2),"Ângulo [°]");
xlabel("Tempo [s]");
legend ({"Posição","Ângulo" }, "location", "northeast");
print( gcf, '-dsvg',"-S640,320", nome_svg4);

Ktst = [ num2str(K(1)) "," num2str(K(2)) "," num2str(K(3)) "," num2str(K(4)) "," num2str(Ktk(5))]
Aa
Bptst=[ num2str(Bp(1)) "," num2str(Bp(2))]
Ke
Ab


##outputs=[posicao angulo velocidade velocidade_angular];
##inputs=saida;
##dat = iddata (outputs, inputs, T);
##na=2;
##nb=1;
##[sys_id1, x0, info] = moen4(dat, 6)
##%[sys_id, x0] = arx(dat, 'na',na,'nb',nb)
##pole(sys_id1)
##%pole(sys_id)
##l




