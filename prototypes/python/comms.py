from enum import Enum
from dataclasses import dataclass
from math import log10,sqrt,pi,erfc
from random import randint
from geometry import Vec2f,distance

C=299792458

class RFsystem(Enum):
    NO_LOSS = 0
    WI_FI = 1
    ADS_B = 2
    FLAT_LOSS: 3

@dataclass
class Channel:
    center_frequency: float
    bandwidth: float
    noise_power: float
    

# Compute receive probability
# dist is the distance
def consiglio(dist:float)->float:
    p1 = -7.44e-8
    p2 = -9.286e-5
    p3 = 0.9101
    return p1 * (dist * dist) + p2 * dist + p3

def esat(dist:float)->float:
    if (dist < 50):
        return 1.0
    
    p1 = -2.1e-09
    p2 = 5.034e-06
    p3 = -0.003541
    p4 = 1.138

    return p1 * dist * dist * dist + p2 * dist * dist + p3 * dist + p4


def COM_compute_Pe(chn:Channel, dist:float, ptx:float, symbol_rate:float, packet_length:int )->tuple[float,float,float,float]:
    
    prx = ptx - 20*log10(dist) - 20*log10(chn.center_frequency*1e6) - 20*log10(4*pi/C)
    snr = pow(10, (prx-chn.noise_power)/10)
    eb_n0 = snr*chn.bandwidth/symbol_rate
    ber = 0.5*erfc(sqrt(eb_n0))
    per = 1-pow((1-ber),packet_length)
    
    return (prx,snr,ber,per)

def COM_broadcast(d1:Vec2f, d2:Vec2f, rf_sys:RFsystem , loss:float):
    p = randint(0,1000)
    
    match (rf_sys):
        case RFsystem.WI_FI:
            lim = loss * esat(distance(d1, d2))*1000
        case RFsystem.ADS_B:
            lim = loss * consiglio(distance(d1, d2))*1000
        case RFsystem.NO_LOSS:
            lim = 1001
        case RFsystem.FLAT_LOSS:
            lim = loss*1000

    return p < lim