%% setup serial connection
clear
close
clc
%build serial port com"" to arduino
try
    s=connect('com3');
catch
    comfix
    s=connect('com3');
end

%% vars
des=[];
l1=16;
l2=6;
l3=15;
l4=15;
l5=6;
curr_ang=[];
ang_send[];
flag=0;
%% input the desired point and check if point is available
while(1)
    des(1)=input("enter the desired x value");
    des(2)=input("enter the desired y value");
    % input the desired valuse to be [-1,-1] if you want to exit the script
    % and terminate the connection
    if (des(1)==-1 && des(2)==-1)
        break;
    end
    flag=InBound(des);%checking if the iputs are in the bounds of the system
    while(~flag)% if not then asks for another input
       disp("your point is out of bound please enter a new one");
       des(1)=input("enter the desired x value");
       des(2)=input("enter the desired y value");
       flag=InBound(des);
    end
%% insert inverse kinematics here
d1=(des(1)^2+des(2)^2)^(1/2);
d2=((des(1)-l1)^2+des(2)^2)^(1/2);       
beta2p=acos((l2^2+d1^2-l3^2)/(2*l2*d1));
beta2m=-acos((l2^2+d1^2-l3^2)/(2*l2*d1));
a2=atans(des(2),des(1));
ang1_1 = a2 + beta2p;
ang1_2 = a2 + beta2m;
beta5p=acos((l5^2+d2^2-l4^2)/(2*l5*d2));
beta5m=-acos((l5^2+d2^2-l4^2)/(2*l5*d2));
a5 = atan2(des(2), (des(1) - l1));
ang2_1 = a5 + beta5p;
ang2_2 = a5 + beta5m;
%after all calculation is done. the angles are being sent to opt function
%in oder to decide which one is the more optimized one
ang_send(1)=opt(ang1_1,ang1_2,curr_ang(1));		
ang_send(2)=opt(ang2_1,ang2_2,cerr_ang(2));

%% write to arduino 
fwrite(s,ang_send(1));
fwrite(s,ang_send(2));
%% wait untill all data is returned
while s.bytesavailable<2 
 disp('.'); 
 end

%% update the current valuse before  running again
curr_ang=fread(s,2);
if(curr_ang(1)=='/' || curr_ang(2)=='/'
 break;
end

end
%% close the connection
clear s;
%% functions

function flag= InBound(des)
d=des(1)-l1;
if (des(1)^2+des(2)^2>(l2+l3)^2)
    flag=0;
elseif (des(1)^2+des(2)^2<(l2-l3)^2)
    flag=0;
elseif (d^2+des(2)^2>(l4+l5)^2)
    flag=0;
elseif (d^2+des(2)^2<(l4-l5)^2)
    flag=0;
else
    flag=1;
end
end
function opta=opt(opt1,opt2,curr_ang_theta)
if(abs(opt1-curr_ang_theta)<abs(opt2-curr_ang_theta)
 opta=opt1;
else
 opta=opt2;   
end
end
