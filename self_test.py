import SoapySDR
from SoapySDR import * #SOAPY_SDR_* constants
import numpy as np
import time

if __name__ == "__main__":
    hackrf = SoapySDR.Device(dict(driver="hackrf"))
    print hackrf

    hackrf.setSampleRate(SOAPY_SDR_RX, 0, 8e6)
    hackrf.setSampleRate(SOAPY_SDR_TX, 0, 8e6)

    """
    for i in range(5):
        print("  Make rx stream #%d"%i)
        rxStream = hackrf.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0])
        for j in range(5):
            numSampsTotal = 10000
            print("    Activate, get %d samples, Deactivate #%d"%(numSampsTotal, j))
            hackrf.activateStream(rxStream)
            buff = np.array([0]*1024, np.complex64)
            while numSampsTotal > 0:
                sr = hackrf.readStream(rxStream, [buff], buff.size, timeoutUs=int(1e6))
                #print sr
                assert(sr.ret > 0)
                numSampsTotal -= sr.ret
            hackrf.deactivateStream(rxStream)
        hackrf.closeStream(rxStream)

    for i in range(5):
        print("  Make tx stream #%d"%i)
        txStream = hackrf.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, [0])
        for j in range(5):
            numSampsTotal = 10000
            print("    Activate, send %d samples, Deactivate #%d"%(numSampsTotal, j))
            hackrf.activateStream(txStream)
            buff = np.array([0]*1024, np.complex64)
            while numSampsTotal != 0:
                size = min(buff.size, numSampsTotal)
                sr = hackrf.writeStream(txStream, [buff], size)
                #print sr
                if not (sr.ret > 0): print("Fail %s, %d"%(sr, numSampsTotal))
                assert(sr.ret > 0)
                numSampsTotal -= sr.ret
            hackrf.deactivateStream(txStream)
        hackrf.closeStream(txStream)
    """

    ####################################################################
    #setup both streams at once
    ####################################################################
    rxStream = hackrf.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0])
    txStream = hackrf.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, [0])

    hackrf.activateStream(rxStream)
    hackrf.activateStream(txStream)

    numSampsTotal = 10000
    hackrf.activateStream(rxStream)
    buff = np.array([0]*1024, np.complex64)
    while numSampsTotal > 0:
        sr = hackrf.readStream(rxStream, [buff], buff.size, timeoutUs=int(1e6))
        #print sr
        assert(sr.ret > 0)
        numSampsTotal -= sr.ret

    numSampsTotal = 10000
    buff = np.array([0]*1024, np.complex64)
    while numSampsTotal != 0:
        size = min(buff.size, numSampsTotal)
        sr = hackrf.writeStream(txStream, [buff], size)
        #print sr
        if not (sr.ret > 0): print("Fail %s, %d"%(sr, numSampsTotal))
        assert(sr.ret > 0)
        numSampsTotal -= sr.ret

    hackrf.deactivateStream(rxStream)
    hackrf.deactivateStream(txStream)

    hackrf.closeStream(rxStream)
    hackrf.closeStream(txStream)
