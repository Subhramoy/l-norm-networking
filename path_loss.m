% this file is used to generate the past loss of the signal with selective
% channel model


function  Params = path_loss(Ch)
Params = Ch;
Params.type = Ch.type;

switch Params.type
    case {'A','B'}
        Params.dBP = 5;
        Params.dBPSlopeBef = 2;
        Params.dBPSlopeAft = 3.5;
        Params.LosBef = 3;
        Params.LosAft = 4;
        
end
% floor effect
if Ch.nFloor==0
    Params.PelFloor = 0;
else
    Params.PelFloor = 18.3*Ch.nFloor.^((Ch.nFloor+2)/(Ch.nFloor+1)-0.46);
end
% wall effect
Params.PelWall = Ch.nWall*Ch.Liw;

Params.floorLoss = 18;
end