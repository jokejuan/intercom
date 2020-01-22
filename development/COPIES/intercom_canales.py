# Adding a buffer.

#Current Version 1.4 - status: working
#Restriction:   -increase samples per chunk to 2048
#               -samples per have to be multiply of 8
#Version history

#1.4 - Milestone - Pre-Release 
#1.3 - Added code comments
#1.2 - Cleaning up metods
#1.1 - new structure for seperat channel sending
#1.0 - implementation bitplane object

import sounddevice as sd
import numpy as np
import struct
from intercom_bitplanes import Intercom_bitplanes
#Import Object Intercom
from intercom import Intercom

if __debug__:
    import sys

class Intercom_canales(Intercom_bitplanes):

    def init(self, args):
        Intercom_bitplanes.init(self, args)

    def run(self):
        self.recorded_chunk_number = 0
        self.played_chunk_number = 0

        if self.number_of_channels==2:
            with sd.Stream(samplerate=self.frames_per_second, blocksize=self.frames_per_chunk, dtype=np.int16, channels=self.number_of_channels, callback=self.record_send_and_play_compressed):
                    print("-=- Press CTRL + c to quit -=-")
                    first_received_chunk_number = self.receive_and_buffer()
                    self.played_chunk_number = (first_received_chunk_number - self.chunks_to_buffer) % self.cells_in_buffer
                    while True:
                        self.receive_and_buffer()
        else:
            with sd.Stream(samplerate=self.frames_per_second, blocksize=self.frames_per_chunk, dtype=np.int16, channels=self.number_of_channels, callback=self.record_send_and_play):
                print("-=- Press CTRL + c to quit -=-")
                first_received_chunk_number = self.receive_and_buffer()
                self.played_chunk_number = (first_received_chunk_number - self.chunks_to_buffer) % self.cells_in_buffer
                while True:
                    self.receive_and_buffer()
                    
    def record_send_and_play_compressed(self, indata, outdata, frames, time, status):
        
        msg=msg[:,1]-msg[:,0]

        for b in range(15,-1,-1):
            for i in range(self.number_of_channels):
                bitplane=msg[:,i] >> b & 1
                bitplane_raw=bitplane.astype(np.uint8)
                bitpack=np.packbits(bitplane_raw)
                message = struct.pack(self.packet_format, b,i, self.recorded_chunk_number, *(bitpack))
                self.sending_sock.sendto(message, (self.destination_IP_addr, self.destination_port))

        self.recorded_chunk_number = (self.recorded_chunk_number + 1) % self.MAX_CHUNK_NUMBER    
        chunk = self._buffer[self.played_chunk_number % self.cells_in_buffer]
        
        chunk[:,1] = chunk[:,1]+chunk[:,0]
        
        self._buffer[self.played_chunk_number % self.cells_in_buffer] = self.generate_zero_chunk()
        self.played_chunk_number = (self.played_chunk_number + 1) % self.cells_in_buffer
        outdata[:] = chunk

        if __debug__:
            sys.stderr.write("."); sys.stderr.flush()
                   
if __name__ == "__main__":
    intercom = Intercom_canales()
    parser = intercom.add_args()
    args = parser.parse_args()
    intercom.init(args)
    intercom.run()
