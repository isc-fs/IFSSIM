%%IFS Simulator%%

clc
clear all

%Drive Cycle

load Schedule_Autocross.mat;
%load Schedule_Endurance_22km.mat

Sch_Cycle(:,2) = Sch_Cycle(:,2)*1.60934;

%Vehicle Parameters

Vehicle_Mass = 237;
Vehicle_Tire_Radius = 0.3;
Vehicle_DShaft_Inertia = 1.5e-3;
Vehicle_Dr_Shaft_Inertia = 3.0e-3;
Vehicle_Rear_Diff_Ratio = 2;
Wheel_Inertia = 0.02179531;         % Wheel Inertia, [kg-m^2]
Tire_Inertia = 0.193286;            % Tire Inertia, [kg-m^2]
Wheel_Plus_Tire_Inertia = Wheel_Inertia + Tire_Inertia;         % Combined Wheel-Tire Inertia, [kg-m^2]


%Motor Parameters
MGA_Max_Torque = 240;



%Battery Parameters
n_series = 19;
n_stacks = 5;
max_cellV = 4.2;
Battery_Voltage = n_series*n_stacks*max_cellV;
Battery_Capacity = 8.5; %Amphrs
Battery_Initial_SOC = 0.9; %State of Charge

%Logging Parameters
Ts = 1;