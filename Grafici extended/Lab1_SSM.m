clc;
clear all;

Beq = 2e-6;                     %Nm/(rad/s)
tau_sf = 1e-2;                  %Nm
tau_sf = 0.005;
Beq =1.9976e-6;
Jeq =9.3937e-7;

%% Generalparametersandconversiongains

% conversiongains
c.rpm2rads=2*pi/60; % [rpm] −>[rad/s]
c.rads2rpm=60/2/pi; % [rad/s]−>[rpm]
c.rpm2degs=360/60; % [rpm] −>[deg/s]
c.degs2rpm=60/360; % [deg/s]−>[rpm]
c.deg2rad=pi/180; % [deg] −>[rad]
c.rad2deg=180/pi; % [rad] −>[deg]
c.ozin2Nm=0.706e-2; % [oz*inch]−>[N*m]

%% DCmotornominalparameters

% brushedDC−motorFaulhaber2338S006S
mot.R =2.6; % armatureresistance
mot.L =180e-6; % armatureinductance
mot.Kt =1.088 * c.ozin2Nm; % torqueconstant
mot.Ke =0.804e-3* c.rads2rpm; % back−EMFconstant
mot.J =5.523e-5* c.ozin2Nm; % rotorinertia
mot.B =0.0; % viscousfrictioncoeff(n.a.)
mot.eta =0.69; % motorefficiency
mot.PN =3.23/mot.eta; % nominaloutputpower
mot.UN =6; % nominalvoltage
mot.IN =mot.PN/mot.UN; % nominalcurrent
mot.tauN=mot.Kt*mot.IN; % nominaltorque
mot.taus=2.42 * c.ozin2Nm; % stalltorque
mot.w0 =7200 * c.rpm2rads; % no−loadspeed

%% Gearboxnominalparameters

% planetarygearboxMicromotorSA23/1
gbox.N1 =14; % 1streductionratio(planetarygearbox)
gbox.eta1=0.80; % gearboxefficiency

% externaltransmissiongears
gbox.N2 =1; % 2ndreductionratio(externaltrasmissiongears)
gbox.J72 =1.4e-6; % inertiaofasingleexternal72toothgear
gbox.eta2=1; % externaltrasmissionefficiency(n.a.)

% overallgearboxdata
gbox.N =gbox.N1*gbox.N2; % totalreductionratio
gbox.eta=gbox.eta1*gbox.eta2; % totalefficiency
gbox.J =3*gbox.J72; % totalinertia(atgearboxoutput)

%% Mechanicalloadnominalparameters

% inertiadiscparams
mld.JD=3e-5; % loaddiscinertia
mld.BD=0.0; % loadviscouscoeff(n.a.)

% overallmechloadparams
mld.J =mld.JD+gbox.J; % totalinertia
mld.B =2.5e-4; % totalviscousfriccoeff(estimated)
mld.tausf=1.0e-2; % totalstaticfriction(estimated)

%% Voltagedrivernominalparameters

% op−ampcircuitparams
drv.R1=7.5e3; % op−ampinputresistor(dactonon−invertingin)
drv.R2=1.6e3; % op−ampinputresistor(non−invertingintognd)
drv.R3=1.2e3; % op−ampfeedbackresistor(outputto invertingin)
drv.R4=0.5e3; % op−ampfeedbackresistor(invertingintognd)
drv.C1=100e-9; % op−ampinputcapacitor
drv.outmax=12; % op−ampmaxoutputvoltage

% voltagedriverdc−gain
drv.dcgain=drv.R2/(drv.R1+drv.R2) * (1+drv.R3/drv.R4);

% voltagedrivertimeconstant
drv.Tc=drv.C1 * drv.R1*drv.R2/(drv.R1+drv.R2);

%% Sensorsdata

% shuntresistor
sens.curr.Rs=0.5;

% Hewlett−PackardHEDS−5540#A06opticalencoder
sens.enc.ppr=500*4; % pulsesperrotation
sens.enc.pulse2deg=360/sens.enc.ppr; % [pulses]−>[deg]
sens.enc.pulse2rad=2*pi/sens.enc.ppr; % [pulses]−>[rad]
sens.enc.deg2pulse=sens.enc.ppr/360; % [deg]−>[pulses]
sens.enc.rad2pulse=sens.enc.ppr/2/pi; % [rad]−>[pulses]

% potentiometer1(Spectrol138−0−0−103)−installedonmotorbox
sens.pot1.range.R =10e3; % ohmicvaluerange
sens.pot1.range.V =5; % voltagerange
sens.pot1.range.th_deg=345; % anglerange[deg]
sens.pot1.range.th =sens.pot1.range.th_deg * c.deg2rad; % anglerange[rad]
sens.pot1.deg2V =sens.pot1.range.V/sens.pot1.range.th_deg; % sensitivity[V/deg]
sens.pot1.rad2V =sens.pot1.range.V/sens.pot1.range.th; % sensitivity[V/rad]
sens.pot1.V2deg =1/sens.pot1.deg2V; % conversiongain[V]−>[deg]
sens.pot1.V2rad =1/sens.pot1.rad2V; % conversiongain[V]−>[rad]

%% Dataacquisitionboard(daq)data

% NIPCI−6221DACdata
daq.dac.bits=16; % resolution(bits)
daq.dac.fs =10; % fullscale
daq.dac.q =2*daq.dac.fs/(2^daq.dac.bits-1); % quantization

% NIPCI−6221ADCdata
daq.adc.bits=16; % resolution(bits)
daq.adc.fs =10; % fullscale(assetinSLDRTAnalogInputblock)
daq.adc.q =2*daq.adc.fs/(2^daq.adc.bits-1); % quantization

%% PLANT
Req = mot.R + sens.curr.Rs;
%Jeq = mot.J + mld.J /(gbox.N^2);
%Beq = mot.B + mld.B /(gbox.N^2);
daq.q = 360/(500 *4);







%% PARAMETRI ####################################
SS.AWon = 1;
SS.INon = 1;
SS.wFrict = 0;

% Nominal / Integral / Error State / Extended Estimator
SS.type =4;
es = 8

%Condizioni iniziali
SS.sat = 10;
Ts = 0.001;
ts5 = 0.15; 
Mp = 0.1;

simu = 3;
%step
step = 40;

doStep = 0;

% sinusoide
Ar_choice = [ 30, 60, 90, 40];
Amplitude = Ar_choice(4);

Tr_choice = [0.15, 0.25, 0.5, 1];
w0 = 2*pi/Tr_choice(3); % [rad/s]

%################################################


%% LAB0 ##########################


d_meas = @(d_Mp) (log(1/d_Mp)/sqrt(pi^2+log(1/d_Mp)^2));

%P(s)
d= d_meas(Mp);
w_n = 3/(d*ts5);
T_L = 5/2*1/w_n;
%T_L = 1/(3*w_n);

P.Km = (drv.dcgain*mot.Kt)/(Req*Beq+mot.Kt*mot.Ke);   %30
P.Tm = (Req*Jeq)/(Req*Beq+mot.Kt*mot.Ke);             %30
P.num = [P.Km];
P.den = conv([P.Tm 1], [gbox.N 0]);
P.tf = tf(P.num,P.den);
%FINE LAB0 ##################

%% LAB1 #######################

%%AW

T_w = ts5/5;
SS.Kw = 1/T_w;

%SSM

% Nominal
d_H = 1/sqrt (2);
w_n_H = 2*pi*50;
H1.num = [w_n_H^2 0];
H1.den = [1 2*d_H*w_n_H w_n_H^2];

Nom.A = [0 1; 0 -1/P.Tm];
Nom.B = [0; P.Km/(gbox.N*P.Tm)];
Nom.C = [1 0];
Nom.D = 0;

N = ([Nom.A Nom.B;Nom.C Nom.D])\[0;0;1];

p.real = -d*w_n;
p.imag = 1j*w_n * sqrt(1 - d^2);

if(SS.type == 1)
    p.nom = [p.real+p.imag,p.real-p.imag];
    SS.K = [0 place(Nom.A,Nom.B,p.nom)];
end
%% Integral
if (SS.type == 2)
    Int.A = [0 Nom.C; zeros(2,1) Nom.A];
    Int.B = [0; Nom.B];
    Int.C = [0 Nom.C];    
    
    p.int = {   [p.real+p.imag, p.real-p.imag, p.real] ...
                    [p.real, p.real, p.real] ...
                    [2*p.real+p.imag, 2*p.real-p.imag, 2*p.real] ...
                    [2*p.real+p.imag, 2*p.real-p.imag, 3*p.real]
            };
    SS.K = acker(Int.A, Int.B, p.int{2});
end
%% Error Space
if (SS.type ==3)
    % sinusoide * errore costante(in laplace è s)
    ps = [1, 0, w0^2, 0];
    Err.A = [ zeros(2,1) eye(2)  zeros(2,2);
              ps(4) ps(3) ps(2)  Nom.C;
              zeros(2,3)         Nom.A ];
    Err.B = [ zeros(3,1); Nom.B ];

    p.err = [ w_n*exp(1j*(-pi+pi/4)) 
              conj(w_n*exp(1j*(-pi+pi/4)))
              w_n*exp(1j*(-pi+pi/6))
              conj(w_n*exp(1j*(-pi+pi/6)))
              -w_n ];
    SS.K = place(Err.A, Err.B, p.err);
end
%% Extended estimator
if (SS.type == 4)
    ps = [1, 0, -(w0^2), 0];
    Ap = [0 1 0 ;
          0 0 1 ;
          flip(ps(2:4))];
    Cp = [1 0 0];

    Ae = [Ap zeros(3,2);
          Nom.B*Cp Nom.A];
    Be = [zeros(3,1); Nom.B];
    Ce = [zeros(1,3) Nom.C];

    p.nom = [p.real+p.imag,p.real-p.imag];
    SS.K = [0 place(Nom.A,Nom.B,p.nom)];
    
    p.p1 = 3*w_n*exp(1j*(-pi+(pi/3)));
    p.p2 = conj(p.p1);
    p.p3 = 1.5*w_n*exp(1j*(-pi+(pi/6)));
    p.p4 = conj(p.p3);
    p.p5 = -2*w_n;
    
    

    p.all=[p.p1,p.p2,p.p3,p.p4,p.p5];
    L = place(Ae', Ce', p.all)';
    Ao = Ae-L*Ce;
        Bo = [Be L];
        Co = eye(5);
        Do = zeros(5,2);
end



%FINE LAB1 ##################


% Sim
if (simu == 1)
    sim("sLab1_SSM.slx");
elseif (simu ==2)
    sim("ext.slx");
end

% Definizione delle variabili
%es = 7;

if es == 6
    Tr = 0.5;
    
    % Assicuriamoci di partire con un file pulito (opzionale ma consigliato)
    % altrimenti se lanci lo script due volte, continui ad aggiungere roba al vecchio es6.mat
    if isfile('es6.mat')
        delete('es6.mat');
    end
    for index = 1:1:3
        Amplitude = Ar_choice(index);
        
        % Lancia la simulazione
        sim("ext.slx");
        
        % Crea un nome variabile che dipende dall'indice (es. 'iterazione_1')
        nomeVarDinamico = sprintf('A_%d', Amplitude);
        nomeRefDinamico = sprintf('ref_%d', Amplitude);

        % Chiama la nuova funzione passando il terzo argomento
        salvaDatiSimulink(ans.th,  'es6.mat', nomeVarDinamico);
        salvaDatiSimulink(ans.ref, 'es6.mat', nomeRefDinamico);
        
    end
    SS.wFric = 1;
    for index = 1:1:3
        Amplitude = Ar_choice(index);
        
        % Lancia la simulazione
        sim("ext.slx");
        
        % Crea un nome variabile che dipende dall'indice (es. 'iterazione_1')
        nomeVarDinamico = sprintf('A_wF%d', Amplitude);
        
        % Chiama la nuova funzione passando il terzo argomento
        salvaDatiSimulink(ans.th, 'es6.mat', nomeVarDinamico);
    end
end

%es = 8;


if es == 7
    if isfile('es7.mat')
        delete('es7.mat');
    end
    Amplitude = 40;
    sim("ext.slx");
    salvaDatiSimulink(ans.err, 'es7.mat', 'err6');
    salvaDatiSimulink(ans.ref, 'es7.mat', 'ref');

    w0 = 2*pi/0.1; % [rad/s]
    sim("ext.slx");
    salvaDatiSimulink(ans.err, 'es7.mat', 'err7');

    plottaDatiDaMat('es7.mat',{'ref7','err6','err7'});
end

%es = 8;

if es == 8
    if isfile('es8.mat')
        delete('es8.mat');
    end
    doStep = 1;
    sim("ext.slx");
    salvaDatiSimulink(ans.th, 'es8.mat', 'step40');
    salvaDatiSimulink(ans.ref, 'es8.mat', 'ref');

    plottaDatiDaMat('es8.mat',{'ref','step40'});


end