clear;
clc;

function y=stp(t)
  if t>=0 then y=1; else y=0; end;
endfunction;

function value=fun(u,zold)
    value=(-zold+10*u)/0.2;
endfunction

function [value] = fun_s(u_old,err_old,err_new,K)
    value=-5.53*u_old+K*(err_new-err_old)+2*K*err_new;
end

function val=imp(t,t0,delta_t)
    val=stp(t-t0)-stp(t-t0-delta_t);
endfunction

//Parameters

K=0.5;

t_final=500; //final time in seconds
x_initial=5;  //initial position of table
delta_t=0.01;   //sampling time
t_old=0;        //initial time
z_old=0         //intial velocity of table
SP_initial=0.01;//setpoint for initial position
SP_final=100;       //setpooint for final position
tolerance=0.02;      //tolerance to reach setpoint
total_steps=round(t_final/delta_t);
Lamp0=0;    //starting state of lamp is 0
Lamp1=0;    //starting state of lamp is 0
err_old=SP_initial-x_initial;

Pt    =  0; // Table phase: 0 position0  ,1 position1, 2 going_to_pos1, 3 going_to_pos0
x_old  =  x_initial; // Table state variable
ot    =  0; // Table position output

u_old=0;//initially no control action is done
P     =  [0,0,0,0,0,0,0]; // Previous phase activity vector
Pn    =  [1,0,0,0,0,0,0]; // Current phase activity vector
CMDF =  0; // Command Table to go to position 1(pulse)
CMDR =  0; // Command Table to go to position 0(pulse)
SP=SP_initial;



vt(1)=t_old;    //vector of time
vx(1)=x_initial;//vector of position
vu(1)=u_old;
verr(1)=(SP-x_old)/SP;
v_button_reverse(1)=0;
v_button_forward(1)=0;
vP(:,1) = P(:);
vLamp0(1)=0;
vLamp1(1)=0;

//partial_procedure=400;
for k=1:total_steps   //iterate over the samples

    t_new=t_old+delta_t;

//Physical button signals of start forward and reverse
    start   = imp(t_new,20,delta_t);    //operator switched on the start button at t=20
    button_forward=imp(t_new,50,delta_t)+imp(t_new,200,delta_t)//operator presses forward at t=50 and t=200
    button_reverse=imp(t_new,100,delta_t)+imp(t_new,300,delta_t); //operator presses reverse at t=100 and t=300

    v_button_reverse(k+1)=button_reverse;
    v_button_forward(k+1)=button_forward;


    //simulation of table
    if Pt == 0 & CMDF then Pt=2;          end;
    if Pt == 1 & CMDR then Pt=3;         end;
    if Pt == 2 & CMDR then Pt=2;         end;
    if Pt == 2 & CMDF then Pt=2;         end;
    if Pt == 3 & CMDR then Pt=3;         end;
    if Pt == 3 & CMDF then Pt=3;         end;
    if Pt == 1 | Pt == 2 then SP=SP_final;   end;
    if Pt == 0 | Pt == 3 then SP=SP_initial;     end;
    
    //table state dynamics
    x_new=x_old+delta_t*z_old;
    z_new=z_old+delta_t*fun(u_old,z_old);
    
    //controller dynamics
    err_new=SP-x_new;
    u_new=u_old+delta_t*fun_s(u_old,err_old,err_new,K);
    u_new = min(1, max(-1,u_new));//the controller is saturated between -1 and +1
    
    t_old=t_new;
    x_old=x_new;
    z_old=z_new;
    u_old=u_new;
    err_old=err_new;
    
    // Logic control: sequence 
    // Actions -- possibly depending on differences between old
    // and new phase activities (P and Pn, respectively)
    CMDF =   (Pn(04)==1 & P(04)==0);
    CMDR =   (Pn(06)==1 & P(06)==0);

    if (Pn(02)==1 & P(02)==0) then
       Lamp0=1;
    end;
        if (Pn(04)==1 & P(04)==0) then
       Lamp0=0;
    end;
    if (Pn(05)==1 & P(05)==0) then
       Lamp1=1;
    end;
    if (Pn(06)==1 & P(06)==0) then
       Lamp1=0;
    end;
    if (Pn(07)==1 & P(07)==0) then
       Lamp0=1;
    end;
    
    vLamp0(k+1)=Lamp0;
    vLamp1(k+1)=Lamp1;
        // Transitions to fire

    check0= abs((x_old-SP_initial)/SP_initial)<tolerance;     //are we in position 0
    check1=abs((x_old-SP_final)/SP_final)<tolerance;        //are we in position 1
    check_forward= button_forward>0;
    check_reverse= button_reverse>0;

    P = Pn;
    
    sT0102 = P(01)==1 & check0;
    sT0203 = P(02)==1 & start;
    sT0304 = P(03)==1 & check_forward;
    sT0405 = P(04)==1 & check1;
    sT0506 = P(05)==1 & check_reverse;
    sT0607 = P(06)==1 & check0;
//    sT0703 = P(07)==1 & start;
    sT0703 = P(07)==1;

    // Phase activity update
    if sT0102 then Pn(01) = 0; Pn(02) = 1; end;
    if sT0203 then Pn(02) = 0; Pn(03) = 1; end;
    if sT0304 then Pn(03) = 0; Pn(04) = 1; end;
    if sT0405 then Pn(04) = 0; Pn(05) = 1; Pt= 1; end;
    if sT0506 then Pn(05) = 0; Pn(06) = 1; end;
    if sT0607 then Pn(06) = 0; Pn(07) = 1; Pt=0; end;
    if sT0703 then Pn(07) = 0; Pn(03) = 1; end;
    
        // Vectors for plotting
    vt(k+1)  =t_new;
    vx(k+1)  =x_new;
    vu(k+1)  =u_new;
    verr(k+1)=err_new;
    vP(:,k+1) = P(:);
    
end



scf(0); clf;

subplot(411);
plot(vt,vx);
subplot(412);
plot(vt,vu);
subplot(413);
plot(vt,(vLamp0*0.5),'r',vt,(vLamp1*0.5+1),'g');
subplot(414);
   for i=1:7;
       plot(vt,(vP(i,:))'*0.3+8-i);
   end;
