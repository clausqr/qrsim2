

clf
plot(ts,x*180/pi,'.-k', 'LineWidth',1)
hold on
y=X_simulink.Data(:,7);
%y=flipud(y);
plot(X_simulink.Time(:),y*180/pi,'r','LineWidth',1)

title('super acrobatic mode, T_s=20ms')

xlabel('t (s)');
ylabel('pitch (deg)');
%%
figure

xcomp=interp1(ts, x, X_simulink.Time);
plot(X_simulink.Time(:),abs(y-xcomp)*180/pi,'k','LineWidth',2)


xlabel('t (s)');
ylabel('error (deg)');

grid on