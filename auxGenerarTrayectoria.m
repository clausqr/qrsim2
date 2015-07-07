%AUX Generar trayectoria del flip

a=1500 *pi/180;

ts=0:0.001:15;

T1=3;
T2=T1+sqrt(4*2*pi/a);
T3=7;
T4=10;
x=0*ts;
for k=1:length(ts);
    t=ts(k);
    
    if t<T1
        x(k)=0;
    elseif (T1<=t)&&(t<(T1+(T2-T1)/2))
        x(k)=(t-T1)^2*a/2;
    elseif (((T1+(T2-T1)/2))<=t)&&(t<T2)
        x(k)=-(T2-t)^2*a/2+2*pi;
    else
        x(k)=2*pi;
    end
    
end
x=x+fliplr(x)-2*pi;    
T2-T1


k_R=0.0334;
k_Om=0.003;
c=0.0012;

lam_m=0.015;
lam_M=0.025;
C2=(k_R+lam_M+sqrt(4*c^2+(k_R-lam_M)^2))/2;
C3=(c*k_R/lam_M +k_Om-c-sqrt((c*k_R/lam_M+c-k_Om).^2+(c*k_Om/lam_m).^2))/2;

min([...
         k_Om ... 
         4*k_Om*k_R*lam_m^2/(k_Om^2*lam_M+4*k_R*lam_m^2) ...
         sqrt(k_R*lam_m) ...
         ]);

C2/C3
plot(ts,x,'LineWidth',2)
%%
%plot(X_simulink.Time(:),X_simulink.Data(:,7),'r','LineWidth',2)