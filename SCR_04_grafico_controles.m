t=X_controles.Time;
f1=X_controles.Data(:,1);
f2=X_controles.Data(:,2);
f3=X_controles.Data(:,3);
f4=X_controles.Data(:,4);
ft=f1+f2+f3+f4;

%plot(t,f1-ft/4,'LineWidth',1)
hold on
plot(t,f2-ft/4, 'r','LineWidth',1)
%plot(t,f3-ft/4,'m','LineWidth',1)
plot(t,f4-ft/4,'g','LineWidth',1)

legend({'f_1','f_3','f_3','f_4'}) 

xlabel('t (s)')

ylabel('Force (N)')

title('Force commanded to rotors 1 and 3')
set(gcf,'color',[1 1 1])