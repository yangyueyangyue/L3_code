function [lambda,v_i,v_1,Te_1,Te_2] = DichotomySolution_Follow(lambda_ini,s_0,vd_0,v_0,ig_0,i_alpha_ini,alpha_ini,count_alpha,alpha_r,s_alpha,count_v,s_v,v_min_r,v_max_r,v_min,v_max)
% ---�������в���---
global Veh_Para
CD=Veh_Para(1);
Af=Veh_Para(2);
rho=Veh_Para(3);
M=Veh_Para(4);
f=Veh_Para(5);
g=Veh_Para(6);
eta=Veh_Para(7);
rw=Veh_Para(8);
If=Veh_Para(9);
% ---������������---
global Trans_Para
Ig=Trans_Para;
% ---ȼ����������ϲ���---
global FuelRate_Para
L01=FuelRate_Para(2);
L02=FuelRate_Para(3);
L10=FuelRate_Para(4);
L11=FuelRate_Para(5);
L12=FuelRate_Para(6);
L20=FuelRate_Para(7);
L21=FuelRate_Para(8);
L22=FuelRate_Para(9);
% ---���������������궨---
global Follow_Con_Para
kappa_1=Follow_Con_Para(1);%�ٶȸ�����ָ��Ȩ��ϵ��
Delta_t=Follow_Con_Para(3);%��ɢ�����
Np=Follow_Con_Para(5);%Ԥ��ʱ����ɢ����
%% ��ʼ����������Լ��
global EngMaxSpd_Para EngMaxTrq_Para EngMinTrq_Para
ne_min=EngMaxSpd_Para(1);
ne_max=EngMaxSpd_Para(end);
ne_0=v_0*If*Ig(ig_0)*30/pi/rw;
nemax=EngMaxSpd_Para;%����������Լ��_ת��
Temax=EngMaxTrq_Para;%����������Լ��_ת�ٶ�Ӧ������������
Te_max=interp1(nemax,Temax,ne_0,'linear');%��ǰת�ٿ�����������;
Te_min=EngMinTrq_Para;%���������������
global EngMaxTrqFit_Para1 EngMaxTrqFit_Para2 EngMaxTrqFit_Para3 EngMaxTrqFit_Para4
IniEngTrq1=EngMaxTrqFit_Para1(1);
Slope1=EngMaxTrqFit_Para1(2);
IniEngSpd1=EngMaxTrqFit_Para1(3);
IniEngTrq2=EngMaxTrqFit_Para2(1);
Slope2=EngMaxTrqFit_Para2(2);
IniEngSpd2=EngMaxTrqFit_Para2(3);
IniEngTrq3=EngMaxTrqFit_Para3(1);
Slope3=EngMaxTrqFit_Para3(2);
IniEngSpd3=EngMaxTrqFit_Para3(3);
IniEngTrq4=EngMaxTrqFit_Para4(1);
Slope4=EngMaxTrqFit_Para4(2);
IniEngSpd4=EngMaxTrqFit_Para4(3);
%% Ԥ��ʱ�������
% ---������ʼ��---
v_1=0;
Te_1=0;
Te_2=0;
lambda=lambda_ini;
s=s_0; %��ʼ������
v_i=v_0; %��ʼ������
i_v=1; %��ʼ���������ݲ�ֵ
i_alpha=i_alpha_ini; %��ʼ����ͼ���ݲ�ֵ
alpha=alpha_ini; %��ʼ����ǰλ���¶�
% ---��⿪ʼ---
for j=1:Np %Ԥ��ʱ���ڷֲ�Ԥ�����
    for counter_alpha=i_alpha:count_alpha-1 %��ֵ�õ���ǰλ�ö�Ӧ�¶�
        if(s>=s_alpha(counter_alpha)&&s<s_alpha(counter_alpha+1))
            alpha=alpha_r(counter_alpha)+(alpha_r(counter_alpha+1)-alpha_r(counter_alpha))*(s-s_alpha(counter_alpha))/(s_alpha(counter_alpha+1)-s_alpha(counter_alpha));
            i_alpha=counter_alpha;
            break
        end
    end
    for counter_v=i_v:count_v-1 %��ֵ�õ���ǰλ�ö�Ӧ�ٶ�����Χ
        if(s>=s_v(counter_v)&&s<s_v(counter_v+1))
            v_min=v_min_r(counter_v);
            v_max=v_max_r(counter_v);
            i_v=counter_v;
            break
        end
    end
    % ---����������ϵ������---
    af=f*g*cos(alpha); %�����������ٶ�
    ag=g*sin(alpha); %�¶��������ٶ�
    aa=CD*Af*rho/2/M; %�����������ٶ�
    % ---���ܶٺ������ζ���ʽϵ��---
    delta=1+12*14/rw/rw/M+20*If^2*Ig(ig_0)^2*eta/rw/rw/M; %������ת��������ϵ��delta,����ת������14kg*m^2������ת������20kg*m^2
    A=(L20+L21*30*If*Ig(ig_0)*v_i/pi/rw+L22*(30*If*Ig(ig_0)*v_i/pi/rw)^2)*Delta_t;  %���ܶٺ�����H=A*Te^2+B*Te+C
    B=(L10+L11*30*If*Ig(ig_0)*v_i/pi/rw+L12*(30*If*Ig(ig_0)*v_i/pi/rw)^2+lambda*If*Ig(ig_0)*eta/M/rw/delta)*Delta_t;
    % ---���ܶٺ������ζ���ʽ���---
    if(A>0)
        if(-B/2/A>=Te_max)
            Te_i=Te_max;
        elseif(-B/2/A<=Te_min)
            Te_i=Te_min;
        else
            Te_i=-B/2/A;
        end
    elseif(A<0)
        if(-B/2/A>=0.5*(Te_max+Te_min))
            Te_i=Te_min;
        else
            Te_i=Te_max;
        end
    else
        if(B>=0)
            Te_i=Te_min;
        else
            Te_i=Te_max;
        end
    end
    % ---�������������Ԥ��ʱ���ڳ���״̬---
    temp=2*v_i * (L02 + L12*Te_i + L22*Te_i^2) * (30*If*Ig(ig_0)/pi/rw)^2 + (L01+L11*Te_i+L21*Te_i^2)*(30*If*Ig(ig_0)/pi/rw) + 2*kappa_1*(v_i-vd_0); %lambda���������е�һ�����ֵ
    lambda = (lambda-Delta_t*temp) / (1-2*aa*Delta_t*v_i/delta); %lambda��������
    %lambda = (lambda-Delta_t*temp) ; %lambda��������
    v_ii=v_i+Delta_t*(Te_i*If*Ig(ig_0)*eta/M/rw-af-ag-aa*v_i^2)/delta; %�����һʱ�̵�Ԥ�⳵��
    Delta_s=Delta_t*(v_i+v_ii)/2; %��ø�Ԥ�ⲽ������ʻ·�̣���Ϊ��Ԥ�ⲽ�������ȱ����˶���
    s=s+Delta_s; %�����һʱ�̵�Ԥ����루��һʱ�̵�Ԥ��λ�õ����ľ��룩
    % ---����������٣���̬����������ת��---
    if(v_ii<=v_min) %�����ٶ�����Χ�½����ȫ������
        v_ii=v_min;
        Te_i=(delta*M*(v_ii-v_i)/Delta_t+M*(af+ag+aa*((v_i+v_ii)/2)^2))*rw/eta/If/Ig(ig_0);
        if Te_i>Te_max
            Te_i=Te_max;
        end
    else
        if(v_ii>=v_max) %�����ٶ�����Χ�Ͻ�����ƶ�
            v_ii=v_max;
            Te_i=(delta*M*(v_ii-v_i)/Delta_t+M*(af+ag+aa*((v_i+v_ii)/2)^2))*rw/eta/If/Ig(ig_0);
            if Te_i<Te_min
                Te_i=Te_min;
            end
        end
    end
    % ---�ڱ����ٶ�Ϊ��ֵ��ǰ���£�Ԥ�⳵�ٸ�ֵ---
    if(v_ii<=0) %�����ٶ�Ϊ��ֵ
        v_ii=0;
    end
    v_i=v_ii; %Ԥ�⳵�ٸ�ֵ
    % ---Ԥ��õ��ĵ�һ�����ų��١���һ���͵ڶ��������ŷ���������---
    if(j==1) %Ԥ��õ��ĵ�һ�����ų��١����ŷ���������
        v_1=v_i;
        Te_1=Te_i;
    end
    if(j==2) %Ԥ��õ��ĵڶ������ŷ���������
        Te_2=Te_i;
    end
    % ---������Ť��Լ��---
    ne=v_1*If*Ig(ig_0)*30/pi/rw;
    if(ne<=ne_min)%��֤�����ת�ٿ��Խ��м��㣬��Сֵ
        ne=ne_min;
    elseif(ne>=ne_max)%��֤�����ת�ٿ��Խ��м��㣬���ֵ
        ne=ne_max;
    end
    if ne<EngMaxSpd_Para(2)
        Te_max=IniEngTrq1+Slope1*(ne-IniEngSpd1);
    elseif ne<=EngMaxSpd_Para(3)
        Te_max=IniEngTrq2+Slope2*(ne-IniEngSpd2);
    elseif ne<=EngMaxSpd_Para(4)
        Te_max=IniEngTrq3+Slope3*(ne-IniEngSpd3);
    else
        Te_max=IniEngTrq4+Slope4*(ne-IniEngSpd4);
    end
    % -------------------
end