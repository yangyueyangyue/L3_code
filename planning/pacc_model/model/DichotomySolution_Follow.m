function [lambda,v_i,v_1,Te_1,Te_2] = DichotomySolution_Follow(lambda_ini,s_0,vd_0,v_0,ig_0,i_alpha_ini,alpha_ini,count_alpha,alpha_r,s_alpha,count_v,s_v,v_min_r,v_max_r,v_min,v_max)
% ---车辆固有参数---
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
% ---变速器传动比---
global Trans_Para
Ig=Trans_Para;
% ---燃油消耗率拟合参数---
global FuelRate_Para
L01=FuelRate_Para(2);
L02=FuelRate_Para(3);
L10=FuelRate_Para(4);
L11=FuelRate_Para(5);
L12=FuelRate_Para(6);
L20=FuelRate_Para(7);
L21=FuelRate_Para(8);
L22=FuelRate_Para(9);
% ---控制器参数，待标定---
global Follow_Con_Para
kappa_1=Follow_Con_Para(1);%速度跟踪性指标权重系数
Delta_t=Follow_Con_Para(3);%离散化间隔
Np=Follow_Con_Para(5);%预测时域离散点数
%% 初始发动机力矩约束
global EngMaxSpd_Para EngMaxTrq_Para EngMinTrq_Para
ne_min=EngMaxSpd_Para(1);
ne_max=EngMaxSpd_Para(end);
ne_0=v_0*If*Ig(ig_0)*30/pi/rw;
nemax=EngMaxSpd_Para;%发动机力矩约束_转速
Temax=EngMaxTrq_Para;%发动机力矩约束_转速对应可输出最大力矩
Te_max=interp1(nemax,Temax,ne_0,'linear');%当前转速可输出最大力矩;
Te_min=EngMinTrq_Para;%发动机最大反拖力矩
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
%% 预测时域内求解
% ---参数初始化---
v_1=0;
Te_1=0;
Te_2=0;
lambda=lambda_ini;
s=s_0; %初始化距离
v_i=v_0; %初始化车速
i_v=1; %初始化限速数据插值
i_alpha=i_alpha_ini; %初始化地图数据插值
alpha=alpha_ini; %初始化当前位置坡度
% ---求解开始---
for j=1:Np %预测时域内分步预测求解
    for counter_alpha=i_alpha:count_alpha-1 %插值得到当前位置对应坡度
        if(s>=s_alpha(counter_alpha)&&s<s_alpha(counter_alpha+1))
            alpha=alpha_r(counter_alpha)+(alpha_r(counter_alpha+1)-alpha_r(counter_alpha))*(s-s_alpha(counter_alpha))/(s_alpha(counter_alpha+1)-s_alpha(counter_alpha));
            i_alpha=counter_alpha;
            break
        end
    end
    for counter_v=i_v:count_v-1 %插值得到当前位置对应速度允许范围
        if(s>=s_v(counter_v)&&s<s_v(counter_v+1))
            v_min=v_min_r(counter_v);
            v_max=v_max_r(counter_v);
            i_v=counter_v;
            break
        end
    end
    % ---阻力或阻力系数计算---
    af=f*g*cos(alpha); %滚动阻力加速度
    ag=g*sin(alpha); %坡度阻力加速度
    aa=CD*Af*rho/2/M; %空气阻力加速度
    % ---哈密顿函数二次多项式系数---
    delta=1+12*14/rw/rw/M+20*If^2*Ig(ig_0)^2*eta/rw/rw/M; %汽车旋转质量换算系数delta,车轮转动惯量14kg*m^2，飞轮转动惯量20kg*m^2
    A=(L20+L21*30*If*Ig(ig_0)*v_i/pi/rw+L22*(30*If*Ig(ig_0)*v_i/pi/rw)^2)*Delta_t;  %哈密顿函数：H=A*Te^2+B*Te+C
    B=(L10+L11*30*If*Ig(ig_0)*v_i/pi/rw+L12*(30*If*Ig(ig_0)*v_i/pi/rw)^2+lambda*If*Ig(ig_0)*eta/M/rw/delta)*Delta_t;
    % ---哈密顿函数二次多项式求解---
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
    % ---根据求解结果计算预测时域内车辆状态---
    temp=2*v_i * (L02 + L12*Te_i + L22*Te_i^2) * (30*If*Ig(ig_0)/pi/rw)^2 + (L01+L11*Te_i+L21*Te_i^2)*(30*If*Ig(ig_0)/pi/rw) + 2*kappa_1*(v_i-vd_0); %lambda迭代方程中的一项参数值
    lambda = (lambda-Delta_t*temp) / (1-2*aa*Delta_t*v_i/delta); %lambda迭代方程
    %lambda = (lambda-Delta_t*temp) ; %lambda迭代方程
    v_ii=v_i+Delta_t*(Te_i*If*Ig(ig_0)*eta/M/rw-af-ag-aa*v_i^2)/delta; %获得下一时刻的预测车速
    Delta_s=Delta_t*(v_i+v_ii)/2; %获得该预测步长的行驶路程（认为在预测步长内做匀变速运动）
    s=s+Delta_s; %获得下一时刻的预测距离（下一时刻的预测位置到起点的距离）
    % ---如果超过限速，动态调整发动机转矩---
    if(v_ii<=v_min) %低于速度允许范围下界进行全力加速
        v_ii=v_min;
        Te_i=(delta*M*(v_ii-v_i)/Delta_t+M*(af+ag+aa*((v_i+v_ii)/2)^2))*rw/eta/If/Ig(ig_0);
        if Te_i>Te_max
            Te_i=Te_max;
        end
    else
        if(v_ii>=v_max) %高于速度允许范围上界进行制动
            v_ii=v_max;
            Te_i=(delta*M*(v_ii-v_i)/Delta_t+M*(af+ag+aa*((v_i+v_ii)/2)^2))*rw/eta/If/Ig(ig_0);
            if Te_i<Te_min
                Te_i=Te_min;
            end
        end
    end
    % ---在避免速度为负值的前提下，预测车速赋值---
    if(v_ii<=0) %避免速度为负值
        v_ii=0;
    end
    v_i=v_ii; %预测车速赋值
    % ---预测得到的第一步最优车速、第一步和第二步的最优发动机力矩---
    if(j==1) %预测得到的第一步最优车速、最优发动机力矩
        v_1=v_i;
        Te_1=Te_i;
    end
    if(j==2) %预测得到的第二步最优发动机力矩
        Te_2=Te_i;
    end
    % ---发动机扭矩约束---
    ne=v_1*If*Ig(ig_0)*30/pi/rw;
    if(ne<=ne_min)%保证输入的转速可以进行计算，最小值
        ne=ne_min;
    elseif(ne>=ne_max)%保证输入的转速可以进行计算，最大值
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