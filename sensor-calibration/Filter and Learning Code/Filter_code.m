clear all
clc
format long
output=xlsread('Trial data 4 (C).xlsx');
y=output;
figure(1)
plot(output(:,1),output(:,2));
windowSize = 5000;
lpFilt = designfilt('lowpassfir','PassbandFrequency',.010, ...
         'StopbandFrequency',.050,'PassbandRipple',0.05, ...
          'DesignMethod','equiripple');
fvtool(lpFilt)
% dataIn = rand(1000,1);
y(:,2) = filter(lpFilt,y(:,2));
[sizex,sizey]=size(y);
% d = fdesign.lowpass('Fp,Fst,Ap,Ast',3,5,0.5,40,100);
%    Hd = design(d,'equiripple');
%    output = filter(Hd,y(:,2));
% y(:,1)= output(:,1);
%   fvtool(Hd)
% figure(2)
%  plot(psd(spectrum.periodogram,output,'Fs',100))
figure(6)
plot(y(100:sizex,1),y(100:sizex,2));