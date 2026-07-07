clear
clc
close all

%%
% Parameter Front Side (BMW)
kk = 340;       % kN/m
kr = 30;        % kN/m
br = 1450;      % Ns/m
mc = 408;       % kg
mus = 48.3;     % kg

%%
% State-space
% Matriks state-space
A = [0 1 0 0; 
    (-kk-kr)/mus -br/mus kr/mus br/mus; 
    0 0 0 1; 
    kr/mc br/mc -kr/mc -br/mc];

B1 = [0; 
    -1/mus; 
    0; 
    1/mc];

B2 = [0; 
    kk/mus; 
    0; 
    0];

B = [B1 B2];  

C = [0 0 1 0;
     0 0 0 1]; 

D = [0 0
     0 0]; 

%%
% Parameter
Q = diag([1, 1, 1, 1]);
R = diag([1, 1]);
P = diag([1, 1]);