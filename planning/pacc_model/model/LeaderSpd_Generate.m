clc
clear
close all
%%
Initial_LeaderSpd=70/3.6;
Delta_t=1;
Rd_Pos=[0:20:2500];
V=ones(length(Rd_Pos),1)*Initial_LeaderSpd;
for i=1:length(Rd_Pos)-1
    V(i+1)=V(i)+normrnd(0,0.5)*Delta_t;
end
figure
plot(Rd_Pos,V)