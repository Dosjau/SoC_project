clc;
clear;
%%%%电机参数设定%%%%%%%%%%%
%Tpwm = 1/10000;%开关频率
Tpwm = 1/32000;
%Tpwm = 1/16000;
Tcur = 1/32000;%电流环控制频率
Tspd = 1/32000;%速度环频率
Rs = 0.5;%%ou
Ls = 0.45e-3;%%H
Ld = Ls;
Lq = Ls;
% ffai = 0.012941;%%V.s
ffai = 0.008667;
%ffai = 0.01311;
%ffai = 0.0097;
np  = 7;%极对数
Kt = np*1.5*ffai;%%N.m/A
J  = 0.044e-4;%%kg.m^2
%J  = 0.051e-4;%%kg.m^2
Vdc = 48;%V
%Imax = 18;%A
%%Imax = 10; %% Dayana, changed, was this
Imax = 20;
%%%%%%%环路参数设定%%%%%%%%%%%%%
curfwdlpfa  = 6.28*3000*Tcur/(1+6.28*3000*Tcur);%3k 电压前馈
voltlpfa = 6.28*5000*Tcur/(1+6.28*5000*Tcur);%5k
voltmax = Vdc;%/1.732;%电流环电压限幅
fcc = 4000;
kpc = fcc*6.28*Ls;
kic = fcc*6.28*Rs*Tcur;


%fsc = 450;%%速度环比例增益（Hz）
fsc = 1200;
%fsc = 100;
Tis = 4/(2*pi*fsc)*1.5;%%速度环积分时间常数（s）
kps = fsc*6.28*J/Kt;
%kps = 2*fsc*6.28*J;
%kis = kps/Tis*Tspd;
kis = 0;
Td = Ls/Rs*0.15;
%Td = 1.5*Tspd;
kds = kps*Td/Tspd;
% kds = 10*J/Tspd;
%kds = 0;
spdcallpfa = 6.28*3000*Tspd/(1+6.28*3000*Tspd);%速度计算滤波
curreflpfa = 6.28*800*Tspd/(1+6.28*800*Tspd);%电流指令滤波
curpmax = Imax;
curnmax = -Imax;
kfwd = J/Tspd/Kt*0;
spdfwdlpfa =  6.28*5000*Tspd/(1+6.28*5000*Tspd);%2k 速度前馈滤波
curfbklpfa =  6.28*fcc*Tspd/(1+6.28*fcc*Tspd);%等效电流反馈
stepcur = voltmax*Tspd/Ls;
%ESO
% wc = 400*6.28;
wc = 2000*6.28;
b1 = wc*2*Tspd*0.707;
b2 = wc^2*J/Kt*Tspd;

% wc2 = 300*6.28;
wc2 = 1800*6.28;
%b3 = wc2*2*Tspd;
b3 = wc2*Tspd;

%编码器等效延迟模型
% wn = 36180;
% zeta = 0.4072;
% k1 = 4+4*zeta*wn*Tspd+wn^2*Tspd^2;
% a1 = (2*wn^2*Tspd^2-8)/k1;
% a2 = (4-4*zeta*wn*Tspd+wn^2*Tspd^2)/k1;
% b0 = (wn^2*Tspd^2)/k1;
wn = 311040;
%zeta = 0.53;
zeta = 1;
k1 = 4+4*zeta*wn*Tspd+wn^2*Tspd^2;
a1 = (2*wn^2*Tspd^2-8)/k1;
a2 = (4-4*zeta*wn*Tspd+wn^2*Tspd^2)/k1;
b0 = (wn^2*Tspd^2)/k1;
%速度指令滤波
wref = 1500*6.28;
c1 = (2*wref^2*Tspd^2-8)/(4+4*wref*Tspd+wref^2*Tspd^2);
c2 = (4-4*wref*Tspd+wref^2*Tspd^2)/(4+4*wref*Tspd+wref^2*Tspd^2);
d0 = (wref^2*Tspd^2)/(4+4*wref*Tspd+wref^2*Tspd^2);


% ===== Full-state Luenberger observer design =====
Bfric = 0;                 % viscous friction (none in your script)
Kobs  = np*Kt/J;           % dwe/dt = Kobs*iq - (Bfric/J)*we - (np/J)*d_w
Tso   = Tspd;              % observer step (32 kHz loop, matches your feedback rate)

% continuous augmented model: x=[id iq we d_id d_iq d_w],  y=[id iq we]
Ac = [ -Rs/Ld,  0,       0,        1/Ld, 0,    0;
        0,     -Rs/Lq,   0,        0,    1/Lq, 0;
        0,      Kobs,   -Bfric/J,  0,    0,   -np/J;
        0,      0,       0,        0,    0,    0;
        0,      0,       0,        0,    0,    0;
        0,      0,       0,        0,    0,    0 ];
Bc = [ 1/Ld, 0;  0, 1/Lq;  0,0;  0,0;  0,0;  0,0 ];
Cc = [ 1 0 0 0 0 0;  0 1 0 0 0 0;  0 0 1 0 0 0 ];

Ad = eye(6) + Tso*Ac;      % forward-Euler discretization (matches the block's recursion)
Bd = Tso*Bc;

% observer poles by bandwidth, mapped to z = exp(-w*Tso).
% 3 fast poles for the measured states, 3 slower for the disturbance estimates.
wpoles = 2*pi*[2500, 2600, 2400, 800, 850, 750];   % rad/s, kept distinct for place()
pz     = exp(-wpoles*Tso);
Lobs   = place(Ad', Cc', pz).';                     % 6x3 gain
