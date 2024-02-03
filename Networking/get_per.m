function [ per ] = get_per( sta_id, mcs, ru_size, ru_position, cycle,mcs_snr_per_wcl_100B, sim)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
     switch ru_size
        case 2
            esnr_with_this_ru_mcs = sim.esnr_per_sta_per_cycle_per_ru_per_mcs_2Mhz(sta_id,cycle,ru_position,mcs+1);
        case 4
            esnr_with_this_ru_mcs = sim.esnr_per_sta_per_cycle_per_ru_per_mcs_4Mhz(sta_id,cycle,ru_position,mcs+1);
        case 8
            esnr_with_this_ru_mcs = sim.esnr_per_sta_per_cycle_per_ru_per_mcs_8Mhz(sta_id,cycle,ru_position,mcs+1);
        case 20
            esnr_with_this_ru_mcs = sim.esnr_per_sta_per_cycle_per_ru_per_mcs_20Mhz(sta_id,cycle,mcs+1);

    end

    per = mcs_snr_per_wcl_100B(mcs+1,snr_to_col(esnr_with_this_ru_mcs));

end

