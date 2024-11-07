%% definition
%% ��������-Veh_Para
close all;clc;clear;
CD=0.5;%����ϵ��
Af=4.2;%��Чӭ�����
rho=1.225;%�����ܶ�
M=49000;%��������
f=0.00743;%��������ϵ��
g=9.8;%�������ٶ�
eta=0.98;%��еЧ��
rw=0.5267;%���ְ뾶
If=2.87;% ��������������
Spd_LmtL=0;
Spd_LmtU=120/3.6;
Veh_Para_Value=[CD,Af,rho,M,f,g,eta,rw,If,Spd_LmtL,Spd_LmtU];
ParaMAP_RdSpdLimitOffset=[0 35820];
ParaMAP_RdSpdLimitU=[120/3.6 120/3.6];
ParaMAP_RdSpdLimitL=[0/3.6 0/3.6];
%% ������ת��Լ��
EngMaxSpd_Para_Value=[650,960,1500,1830,2000];
EngMaxTrq_Para_Value=[1000,2650,2650,2100,0];
EngMinTrq_Para_Value=-400;
EngMaxTrqFit_Para_Value1=[1000  5.3226  650];   %���������Ť��Լ����һ�Σ�if ne<EngMaxSpd_Para(2); Te_max=IniEngTrq1+Slope1*(ne-IniEngSpd1);
EngMaxTrqFit_Para_Value2=[2650    0     960];   %���������Ť��Լ����һ�Σ�if ne<EngMaxSpd_Para(3); Te_max=IniEngTrq2+Slope2*(ne-IniEngSpd2);
EngMaxTrqFit_Para_Value3=[2650  -1.667  1500];  %���������Ť��Լ����һ�Σ�if ne<EngMaxSpd_Para(4); Te_max=IniEngTrq3+Slope3*(ne-IniEngSpd3);
EngMaxTrqFit_Para_Value4=[2100 -12.3529 1830];  %���������Ť��Լ����һ�Σ�if ne<EngMaxSpd_Para(5); Te_max=IniEngTrq4+Slope4*(ne-IniEngSpd4);
%% FuelRate_Para
% ȼ�������ʶ��ζ���ʽ���ģ�ͣ�mf=L0_0+L0_1*N+L0_2*(N^2)+L1_0*Te+L1_1*Te*N+L1_2*Te*(N^2)+L2_0*(Te^2)+L2_1*(Te^2)*N+L2_2*(Te^2)*(N^2)�����ϵ��������ʾ
L00 =    0;%L0,0
L01 =    0;%L0,1
L02 =    0;%L0,2
L10 =    0;%L1,0
L11 =    0;%L1,1
L12 =    0;%L1,2
L20 =    3e-06;%L2,01e-05 -6
L21 =    0;%L2,1
L22 =    0;%L2,2
% L00 =      -1.522;%L0,0
% L01 =    0.002239;%L0,1
% L02 =  -2.185e-07;%L0,2
% L10 =    0.002321;%L1,0
% L11 =    1.94e-06;%L1,1
% L12 =   4.879e-10;%L1,2
% L20 =   7.602e-07;%L2,0
% L21 =  -1.465e-09;%L2,1
% L22 =   9.135e-13;%L2,2
FuelRate_Para_Value=[L00,L01,L02,L10,L11,L12,L20,L21,L22];
%% Trans_Para
Ig=[16.41,13.16,11.13,8.92,7.16,5.74,4.68,3.75,2.97,2.38,1.91,1.53,1.25,1];% �������ٱ�
Trans_Para_Value=Ig;
GearIni_Para_Value=12;
%% Env_Para
load('RdSlopeData.mat')
load('RdLeaderSpdData.mat')
ParaMAP_RdSlopeOffsetFirst200=RdSlopeOffsetFirst200(5,1:200);
ParaMAP_RdSlopeValueFirst200=RdSlopeValueFirst200(5,1:200);
ParaMAP_RdSlopeOffsetAll=RdSlopeOffsetAll(5,1:2094);
ParaMAP_RdSlopeValueAll=RdSlopeValueAll(5,1:2094);
%(1,1:773)ԭʼ����   (2,1:279)ԭʼ��������ȡ�����Ƚϵ��͵Ĵ�����������
%(3,1:142)ԭʼ��������ȡ�����Ƚϵ��͵Ĵ��������������   (4,1:18)��Ϊ�趨���ݣ���-��-��-����
%(5,1:2094)��������ԭʼ����
%% SchedulingFigure
%%%%%%%%%%�궨��%%%%%%%%%%%%
Sche_Vref=[0 20 40 50 60 80 100 120]/3.6;%�ο�����m/s
Sche_Distance_follow_exist=[15 18 30 35 45 55 80 120];%��������
Sche_Distance_follow=Sche_Distance_follow_exist.*1.8;%������Ǹ����л�����
Sche_Distance_safe=[12 12 20 25 35 40 50 70];%��ȫ����
%%%%%%%%%%�궨��%%%%%%%%%%%%
%ע����Գ��ٵľ���Ԫ�ظ�������ο����ٵľ���Ԫ�ظ���һ��
Sche_Vrela=[-100 -75 -50 -25 25 50 75 100]/3.6;%��Ծ�����Գ��ٱ궨m/s
Sche_Treact=2.5;%������Ӧʱ��
Sche_Distance_rela=Sche_Vrela.*Sche_Treact;%��Ծ�����Գ��ٱ궨

ParaValue_Schedule=[Sche_Vref;Sche_Distance_follow_exist;Sche_Distance_follow;Sche_Distance_safe;Sche_Vrela;Sche_Distance_rela];
%% Brake_System
%%%%%%%%% Brake %%%%%%%%%
Brake_kv=300;%P����
Brake_th=Sche_Treact;%��Ӧʱ��
ah_max=0.4*9.8;%�����ƶ����ٶ�
ap_max=9.8;%ǰ���ƶ����ٶ�
Brake_ks=100;%P����

ParaValue_Brake=[Brake_kv Brake_th ah_max ap_max Brake_ks];
%% ����������-Con_Para
Cruise_kappa_1=4;%�ٶȸ�����ָ��Ȩ��ϵ��
Cruise_kappa_2=0.5;%�ն˳ͷ���ָ��Ȩ��ϵ��
Cruise_Delta_t=1;%��ɢ�����
Cruise_tp=50;%Ԥ��ʱ��
Cruise_Np=round(Cruise_tp/Cruise_Delta_t);%Ԥ��ʱ����ɢ����
Cruise_Cn=5;
Cruise_diff=2;
Cruise_diff_Lmt=1000;
Cruise_Con_Para_Value=[Cruise_kappa_1,Cruise_kappa_2,Cruise_Delta_t,Cruise_tp,Cruise_Np,Cruise_Cn,Cruise_diff,Cruise_diff_Lmt];


Follow_kappa_1=5;%�ٶȸ�����ָ��Ȩ��ϵ��
Follow_kappa_2=0.5;%�ն˳ͷ���ָ��Ȩ��ϵ��
Follow_Delta_t=1;%��ɢ�����
Follow_tp=20;%Ԥ��ʱ��
Follow_Np=round(Follow_tp/Follow_Delta_t);%Ԥ��ʱ����ɢ����
Follow_Cn=5;
Follow_diff=2;
Follow_diff_Lmt=1000;
Follow_Con_Para_Value=[Follow_kappa_1,Follow_kappa_2,Follow_Delta_t,Follow_tp,Follow_Np,Follow_Cn,Follow_diff,Follow_diff_Lmt];