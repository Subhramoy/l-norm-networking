function pyduration=AXPayloadDuration(packet_size,AXrate,GI)

if GI == 0
    symbolDurationUs = 12.8 + 1.6;
else
    symbolDurationUs = 12.8 + 0.6;
end
mStbc = 1;
numDataBitsPerSymbol = AXrate * symbolDurationUs / 1000000;
Nes = 1;
numSymbols = mStbc * ceil(((16 + packet_size * 8 + 6 * Nes) / (mStbc * numDataBitsPerSymbol)));
%fprintf('numSymbols = %f\n', numSymbols);
pyduration = numSymbols * symbolDurationUs;
