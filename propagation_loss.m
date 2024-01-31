% this file is used to estimate the propagation loss the STA according to
% the distance from the STA to the AP

function Ploss = propagation_loss (r,Params)

rLen = length(r);
Ploss = zeros(1,rLen);
Wavelength = 3e8/(Params.Frequency*1e9);

for rIndex = 1:rLen
        
    if r(rIndex)<Params.ch.dBP
        Ploss(rIndex) = Params.ch.dBPSlopeBef*10*log10(4*pi*r(rIndex)/Wavelength)+Params.ch.LosBef.*randn.*Params.randomShadowFading;
    else
        Ploss(rIndex) = Params.ch.dBPSlopeBef*10*log10(4*pi*Params.ch.dBP/Wavelength)+Params.ch.dBPSlopeAft*10*log10(r(rIndex)/Params.ch.dBP)+Params.ch.LosAft.*randn.*Params.randomShadowFading;
    end
end


Ploss = Ploss+Params.ch.PelFloor+Params.ch.PelWall+Params.fixedshadowfadingtmp;
end

