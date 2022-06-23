clc;
clear all;
close all;

xr_robot=0;
yr_robot=0;
zr_robot=0;

xg_robot=0;
yg_robot=0;
zg_robot=0;

xb_robot=0;
yb_robot=0;
zb_robot=0;

%%%     Camera initialization    %%%

cm_por_pixel = 20/640;

video = videoinput('winvideo',1);
set(video,'ReturnedColorSpace','RGB');
preview(video)

% parameters of robot

a1=10; %lower part 
a2=12; %middle part
a3=16; %upper part

a=arduino('COM3','uno');
s1=servo(a,'D9')% theta 1
s2=servo(a,'D10')% theta 2
s3=servo(a,'D11')% theta 3
s4=servo(a,'D3')% efector

% initial position
writePosition(s1, 0.5);
writePosition(s2, 0.3);
writePosition(s3, 0.6);
writePosition(s4, 0);
pause(1)

Stop=1;
uicontrol('Style','Pushbutton','String','Detener','Callback','Stop=0')

while Stop
    
%%%     image processing    %%%

imagen = getsnapshot(video);

red = imagen(:,:,1);
green = imagen(:,:,2);
blue = imagen(:,:,3);

resta1 = red-green-blue;
resta1 = resta1;

resta2 = green-blue-red;
resta2 = resta2*10;
resta3 = blue-green-red;
resta3 = resta3*10;

%%%     Finding red block     %%%

suma_colr = sum(resta1);
num_colr = [1:1:640];
mult_colr = suma_colr.*num_colr;
totalcr = sum(mult_colr);
location_colr = totalcr/sum(sum(resta1));

suma_rowr = sum(resta1'); 
num_rowr = [1:1:480];
mult_rowr = suma_rowr.*num_rowr;
totalrr = sum(mult_rowr);
location_rowr = totalrr/sum(sum(resta1));

%%%     Finding green block     %%%

suma_colg = sum(resta2);
num_colg = [1:1:640];
mult_colg = suma_colg.*num_colg;
totalcg = sum(mult_colg);
location_colg = totalcg/sum(sum(resta2));

suma_rowg = sum(resta2'); 
num_rowg = [1:1:480];
mult_rowg = suma_rowg.*num_rowg;
totalrg = sum(mult_rowg);
location_rowg = totalrg/sum(sum(resta2));

%%%     Finding blue block     %%%

suma_colb = sum(resta3);
num_colb = [1:1:640];
mult_colb = suma_colb.*num_colb;
totalcb = sum(mult_colb);
location_colb = totalcb/sum(sum(resta3));

suma_rowb = sum(resta3'); 
num_rowb = [1:1:480];
mult_rowb = suma_rowb.*num_rowb;
totalrb = sum(mult_rowb);
location_rowb = totalrb/sum(sum(resta3));

%%%     Plot the positions of the objects     %%%

%figure
imshow(imagen)
hold on
plot(location_colr,location_rowr,'xw')
plot(location_colg,location_rowg,'ow')
plot(location_colb,location_rowb,'ow')
hold off


%%%     Reverse kinematics of red block    %%%

if totalcr > 30000000 %Detection threshold varies according to lighting
 
    xr_location = location_colr * cm_por_pixel;
    yr_location = location_rowr * cm_por_pixel;

    xr_robot = yr_location;
    yr_robot = xr_location;

    Dr=sqrt(xr_robot^2+yr_robot^2);
    Br=sqrt(Dr^2+zr_robot^2);
    alphar=atand(zr_robot/Dr);
    Cr=sqrt(a1^2+Br^2-2*a1*Br*cosd(90-alphar));
    phi1r=(acosd((a3^2-a2^2-Cr^2)/(-2*a2*Cr)));
    phi2r=(acosd((Br^2-a1^2-Cr^2)/(-2*a1*Cr)));
    phi3r=(acosd((Cr^2-a2^2-a3^2)/(-2*a2*a3)));

    T1r=round(atand(yr_robot/xr_robot))+90;
    T2r=round(180-phi1r-phi2r);
    T3r=round(phi3r-90);
    
    %Adjustment to avoid errors
    if isreal(T2r-5)
        T2r = T2r
    else 
        T2r = 90
    end
    
    if 173 < T3r
        T3r = 180 
    else 
        T3r = T3r    
    end
    
    % Angle adjustment and object collection
    writePosition(s1,(285-T1r)/180);
    pause(0.5)
    writePosition(s2,(150-T2r-20)/180);
    pause(0.5)
    writePosition(s3,(T3r+140)/180);
    pause(0.5)
    writePosition(s2,(145-T2r)/180);
    pause(0.5);
    writePosition(s4,90/180);
    pause(0.5)
    
    
    writePosition(s2, 0.3);
    pause(0.5)
    writePosition(s1, 0.1);
    pause(0.5)
    writePosition(s3, 0.55);
    writePosition(s2, 0.30);
    pause(0.5)
    writePosition(s4, 0);
    pause(1)
       

    writePosition(s1, 0.5);
    writePosition(s2, 0.3);
    writePosition(s3, 0.6);
    writePosition(s4, 0);
    pause(1.5)

%%%     Inverse kinematics of Green    %%%

elseif totalcg > 1000000 %Detection threshold varies according to lighting
   
 
    xg_location = location_colg * cm_por_pixel;
    yg_location = location_rowg * cm_por_pixel;

    xg_robot = yg_location;
    yg_robot = xg_location;

    Dg=sqrt(xg_robot^2+yg_robot^2);
    Bg=sqrt(Dg^2+zg_robot^2);
    alphag=atand(zg_robot/Dg);
    Cg=sqrt(a1^2+Bg^2-2*a1*Bg*cosd(90-alphag));
    phi1g=(acosd((a3^2-a2^2-Cg^2)/(-2*a2*Cg)));
    phi2g=(acosd((Bg^2-a1^2-Cg^2)/(-2*a1*Cg)));
    phi3g=(acosd((Cg^2-a2^2-a3^2)/(-2*a2*a3)));
    
    
    T1g=round(atand(yg_robot/xg_robot))+90;
    T2g=round(180-phi1g-phi2g);
    T3g=round(phi3g-90);
    
    
     % Adjustment to avoid errors
    if isreal(T2g)
        T2g = T2g
    else 
        T2g = 90
    end
    
    if isreal(T3g)
        T3g = T3g
    else 
        T3g = 0
    end

    % Angle adjustment to collect objects
    writePosition(s1, (285-T1g)/180);
    pause(0.5)
    writePosition(s2, (150-T2g)/180);
    pause(0.5)
    writePosition(s3, (T3g+145)/180);
    pause(0.5)
    writePosition(s4, 0.5);
    pause(1)


    writePosition(s2, 0.3);
    pause(0.5)
    writePosition(s1, 0.25);
    pause(0.5)
    writePosition(s3, 0.55);
    writePosition(s2, 0.30);
    pause(0.5)
    writePosition(s4, 0);
    pause(1)
    
    

    writePosition(s1, 0.5);
    writePosition(s2, 0.3);
    writePosition(s3, 0.6);
    writePosition(s4, 0);
    pause(1.5)

 
%%%     Inverse kinematics of Blue objects     %%%

elseif totalcb > 30000000 %Detection threshold varies according to lighting
    
    xb_location = location_colb * cm_por_pixel;
    yb_location = location_rowb * cm_por_pixel;

    xb_robot = yb_location;
    yb_robot = xb_location;

    Db=sqrt(xb_robot^2+yb_robot^2);
    Bb=sqrt(Db^2+zb_robot^2);
    alphab=atand(zb_robot/Db);
    Cb=sqrt(a1^2+Bb^2-2*a1*Bb*cosd(90-alphab));
    phi1b=(acosd((a3^2-a2^2-Cb^2)/(-2*a2*Cb)));
    phi2b=(acosd((Bb^2-a1^2-Cb^2)/(-2*a1*Cb)));
    phi3b=(acosd((Cb^2-a2^2-a3^2)/(-2*a2*a3)));

    T1b=round(atand(yb_robot/xb_robot))+90;
    T2b=round(180-phi1b-phi2b);
    T3b=round(phi3b-90);
    % Adjusting the values to avoid errors   
    if isreal(T2b)
        T2b = T2b
    else 
        T2b = 90
    end
    
    if 180 > T3b > 0
        T3b = T3b
        
    elseif isreal(T3b+5)
        T3b=T3b
    else
        T3b = 90
    end
    

    %Angle adjustment and piece collection

    writePosition(s1, (285-T1b)/180);
    pause(1)
    writePosition(s2, (130-T2b)/180);
    pause(1)

    writePosition(s3, (T3b+145)/180);
    pause(1)
    writePosition(s2, (150-T2b)/180);
    pause(1)
    writePosition(s4, 0.5);
    pause(1)

    
    writePosition(s2, 0.3);
    pause(0.5)
    writePosition(s1, 0.18);
    pause(0.5)
    writePosition(s3, 0.8);
    writePosition(s2, 0.40);
    pause(0.5)
    writePosition(s4, 0);
    pause(1)
    
    

    writePosition(s1, 0.5);
    writePosition(s2, 0.3);
    writePosition(s3, 0.6);
    writePosition(s4, 0);
    pause(1.5)

else

    % initial position
    writePosition(s1, 0.5);
    writePosition(s2, 0.3);
    writePosition(s3, 0.6);
    writePosition(s4, 0);
    pause(1)


end
   
end










