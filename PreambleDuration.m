function prduration=PreambleDuration(Nss,mode)

if Nss > 4
    nLtf = 8;
else
    if Nss > 2
        nLtf = 4;
    else
        nLtf = Nss;
    end
end

%L-STF + L-LTF + L-SIG + RL-SIG + HE-SIG-A + HE-SIG-B + HE-STF + HE-LTF  (802.11ax MU-UL)
prduration = 8 + 8 + 4 + 4 + 8 + 0 + 8 + nLtf * 13.6; 