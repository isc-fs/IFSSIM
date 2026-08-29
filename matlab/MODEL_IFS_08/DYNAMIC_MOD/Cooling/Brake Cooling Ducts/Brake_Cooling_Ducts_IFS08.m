%% Brake Cooling Ducts - IFS08

%% PARAMETERS

%Brake disk

ExtDiameter = 179.5e-3; %m
IntDiameter = 129.6e-3; %m
DiskThickness = 4e-3; %m
HolesDiameter = 8e-3; %m
NumberHoles = 36;

DiskFaceArea = pi*(ExtDiameter^2 - IntDiameter^2)/4; %m^2
HolesArea = pi*HolesDiameter^2/4*NumberHoles; %m^2
DiskArea = 2*DiskFaceArea; %m^2
IntHolesArea = pi*HolesDiameter*DiskThickness*NumberHoles; %m^2
DiskCoolingArea = DiskArea + IntHolesArea; %m^2

%Brake pads

PadHeight = 39.7e-3; %m
PadWidth = 26.8e-3; %m

PadArea = PadHeight*PadWidth; %m^2

%Brake force

%Heat disipation - Convection

%%Heat disipation - Radiation
