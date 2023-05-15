clear all
close all

pkg load control
pkg load signal

T=0.002048;

tensao=3
arquivo = ["tensao_" num2str(tensao) ".txt"];
data = load('-ascii',arquivo);
t=data(:,1)*T;
enc=data(:,2)*((pi*2.4)/1600);


vel=zeros(1,length(t));

for k = 6:length(t);

vel(k)=(enc(k)-enc(k-5))/(5*T);   %controle

endfor

figure
subplot(2,1,1)
plot(t,enc)
subplot(2,1,2)
plot(t,vel)

velocidade_media=enc(length(t))/t(length(t))


ten_var=[-24,-20,-15,-10,-7,-6,-5,-4,-3,-2.5,-2,-1.75,-1.5,-1,-0.5,0.5,1,1.5,1.75,2,2.5,3,3.5,4,4.5,5,6,7,10,15,20,24];
vel_var=[-153.99,-121.04,-84.195,-52.329,-35.450,-28.334,-20.955,-13.086,-5.4461,-1.3084,-0.4093,-0.1369,-0.051161,0,0,0,0,0,0.4541,1.0207,3.4214,7.6208,11.379,14.506,18.447,21.939,29.047,35.386,53.238,80.399,120.50,153.67];

figure
plot(ten_var,vel_var,'r')
pbaspect ([1 0.5 1]);
grid on
##title("Velocidade em relação a Tensão")
ylabel("Velocidade [cm/s]");
xlabel("tensão [v]");
print( gcf, '-dsvg',"-S640,320", "nao_linear_motor.svg");