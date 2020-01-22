# Don't send empty bitplanes.
#
# The sender adds to the number of received bitplanes the number of
# skipped (zero) bitplanes of the chunk sent.

# The receiver computes the first received
# bitplane (apart from the bitplane with the signs) and report a
# number of bitplanes received equal to the real number of received
# bitplanes plus the number of skipped bitplanes.

#VERSION 1.4
#STATUS: WORKING
#
#1.4 - Fix error in congestion calculation
#1.3 - Revision of comments
#1.2 - Optimization (np.any) and check for 8 or fewer bitplanes
#1.1 - Implementation of solution
#1.0 - Creation of case

import struct
import numpy as np
from intercom import Intercom
from intercom_dfc import Intercom_DFC

if __debug__:
    import sys

class Intercom_empty(Intercom_DFC):

    def init(self, args):
        Intercom_DFC.init(self, args)
        #Create counter for empty bitplanes found
        self.empty = 0
        #Variable for remembering last number of empty bitplanes
        self.previous_empty = 0

    def send_bitplane(self, indata, bitplane_number):
        bitplane = (indata[:, bitplane_number%self.number_of_channels] >> bitplane_number//self.number_of_channels) & 1
        bitplane = bitplane.astype(np.uint8)
        bitplane = np.packbits(bitplane)

        #Check if it is an empty bitplane. If its empty increase counter, if not, send package
        if(np.any(bitplane)):
            message = struct.pack(self.packet_format, self.recorded_chunk_number, bitplane_number, self.received_bitplanes_per_chunk[(self.played_chunk_number+1) % self.cells_in_buffer]+1, *bitplane)
            self.sending_sock.sendto(message, (self.destination_IP_addr, self.destination_port))
            return 0
        else:
            return 1

    def send(self, indata):
        signs = indata & 0x8000
        magnitudes = abs(indata)
        indata = signs | magnitudes
        
        #Sum empty bitplanes to total bitplanes received (NORB) for congestion calculation and calc congestion
        self.NOBPTS = int(0.75*self.NOBPTS + (0.25*(self.NORB + self.empty)))
        self.NOBPTS += 1
        
        #If number of bitplanes to send is greater than the maximum or the number of empty bitplanes found is greater than 8 (working case 16 bit), we nullify the congestion calculation for the current chunk (send all bitplanes).
        if (self.NOBPTS > self.max_NOBPTS) or (int(self.previous_empty//self.number_of_channels) > 8):
            self.NOBPTS = self.max_NOBPTS

        #Save last counted bitplanes for considering in next chunk.
        self.previous_empty = self.empty
        #Reset empty bitplane counter.
        self.empty = 0
        last_BPTS = self.max_NOBPTS - self.NOBPTS - 1
        
        #Send and check first two bitplanes
        self.empty += self.send_bitplane(indata, self.max_NOBPTS-1)
        self.empty += self.send_bitplane(indata, self.max_NOBPTS-2)

        #Send and check rest of bitplanes considering congestion
        for bitplane_number in range(self.max_NOBPTS-3, last_BPTS, -1):
            self.empty += self.send_bitplane(indata, bitplane_number)
        self.recorded_chunk_number = (self.recorded_chunk_number + 1) % self.MAX_CHUNK_NUMBER

if __name__ == "__main__":
    intercom = Intercom_empty()
    parser = intercom.add_args()
    args = parser.parse_args()
    intercom.init(args)
    intercom.run()
