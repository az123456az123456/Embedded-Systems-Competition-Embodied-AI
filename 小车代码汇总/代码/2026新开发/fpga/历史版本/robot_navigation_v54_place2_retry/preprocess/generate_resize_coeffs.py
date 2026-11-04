#!/usr/bin/env python3
"""Fixed 960x1080 -> 569x640 half-pixel bilinear coefficients, Q11."""
from fractions import Fraction
from pathlib import Path
import json
HERE=Path(__file__).resolve().parent

def axis(source,target):
    result=[]
    for out in range(target):
        point=Fraction((2*out+1)*source,2*target)-Fraction(1,2)
        lo=point.numerator//point.denominator
        weight=point-lo
        q=(weight*2048+Fraction(1,2)).numerator//(weight*2048+Fraction(1,2)).denominator
        assert 0 <= lo < lo+1 < source and 0 <= q < 2048
        result.append((lo,lo+1,q))
    return result

def main():
    x=axis(960,569);y=axis(1080,640)
    words=[0]*960
    for lo,hi,q in x:
        assert words[hi]==0
        words[hi]=0x800|q
    (HERE/'resize_x_q11.hex').write_text(''.join(f'{w:03x}\n' for w in words))
    period={hi:q for lo,hi,q in y[:16]}
    assert all(hi%27 in period and period[hi%27]==q for lo,hi,q in y)
    data={'input':[960,1080],'active_output':[569,640],'tensor':[640,640],'padding_lrtb':[35,36,0,0],
          'coefficient_fractional_bits':11,'coordinate_rule':'(dst+0.5)*src/dst_size-0.5',
          'x_rom_words':960,'x_rom_bits':12,'y_phase_period':27,'y_phase_weights':period,
          'raw_plus_tensor_payload_bps':960*1080*30*2*8+640*640*30*2*24,
          'tensor_payload_bps':640*640*30*2*24}
    (HERE/'resize_coefficients.json').write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps(period,sort_keys=True))
if __name__=='__main__':main()
