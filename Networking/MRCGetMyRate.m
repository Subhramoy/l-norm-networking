function dataRate=MRCGetMyRate(MCS,bw,Nss,GI)

if MCS == 0
    Nbpscs = 1;
    codingRate = 0;
else 
    if MCS == 1
        Nbpscs = 2;
        codingRate = 0;
    else 
        if MCS == 2
            Nbpscs = 2;
            codingRate = 2;
        else
            if MCS == 3
                Nbpscs = 4;
                codingRate = 0;
            else
                if MCS == 4
                    Nbpscs = 4;
                    codingRate = 2;
                else 
                    if MCS == 5
                        Nbpscs = 6;
                        codingRate = 1;
                    else
                        if MCS == 6
                            Nbpscs = 6;
                            codingRate = 2;
                        else 
                            if MCS == 7
                                Nbpscs = 6;
                                codingRate = 3;
                            else 
                                if MCS == 8
                                    Nbpscs = 8;
                                    codingRate = 2;
                                else 
                                    if MCS == 9
                                        Nbpscs = 8;
                                        codingRate = 3;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end

if bw == 2
    Nsd = 24;
else 
    if bw == 4
        Nsd = 48;
    else 
        if bw == 8
            Nsd = 102;
        else 
            if bw == 20
                Nsd = 234;
            else 
                if bw == 40
                    Nsd = 484;
                else 
                    if bw == 80
                        Nsd = 4*234;
                    else 
                        if bw == 160
                            Nsd = 2*4*234;
                        end
                    end
                end
            end
        end
    end
end
Ncbps = Nbpscs * Nsd * Nss;     %Number coded bits per symbol

if codingRate == 3
    Ndbps = Ncbps * 5 / 6;
else 
    if codingRate == 2
        Ndbps = Ncbps * 3 / 4;
    else 
        if codingRate == 1
            Ndbps = Ncbps * 2 / 3;
        else 
            if codingRate == 0
                Ndbps = Ncbps * 1 / 2;
            end
        end
    end
end

if GI == 0
    Tsym = 0.0000128 + 0.0000016;
else
    Tsym = 0.0000128 + 0.0000008;
end

dataRate = Ndbps / Tsym;        %Data rate calculated over one symbol
