# Adding a buffer.

#Current Version 1.6 - status: working
#Restriction:   -increase samples per chunk to 2048
#               -samples per have to be multiply of 8
#Version history

#1.6 - more code comments
#1.5 - elevated methods receive and send for direct inheritance
#1.4 - Milestone - Pre-Release 
#1.3 - Added code comments
#1.2 - Cleaning up metods
#1.1 - new structure for seperat channel sending
#1.0 - implementation bitplane object

import sounddevice as sd
import numpy as np
import struct
from intercom_buffer import Intercom_buffer
#Import Object Intercom
from intercom import Intercom

if __debug__:
    import sys

class Intercom_bitplanes(Intercom_buffer):

    def init(self, args):
        Intercom_buffer.init(self, args)
        self.bitplaneregister = np.zeros(self.chunks_to_buffer,dtype="int16")
        self.packet_format="!BH{}B".format((self.samples_per_chunk//8)//self.number_of_channels)
        #sys.stderr.write("\n\nFORMAT: {}".format(self.packet_format)); sys.stderr.flush()
    def run(self):
        self.recorded_chunk_number = 0
        self.played_chunk_number = 0

        with sd.Stream(samplerate=self.frames_per_second, blocksize=self.frames_per_chunk, dtype=np.int16, channels=self.number_of_channels, callback=self.record_send_and_play):
            print("-=- Press CTRL + c to quit -=-")
            first_received_chunk_number = self.receive_and_buffer()
            self.played_chunk_number = (first_received_chunk_number - self.chunks_to_buffer) % self.cells_in_buffer
            while True:
                self.receive_and_buffer()


    def receive_and_buffer(self):
        message, source_address = self.receiving_sock.recvfrom(Intercom.MAX_MESSAGE_SIZE)

        #Get elements of sent message
        #bp_number = bitplane number of package
        #chunk_number = chunk number of package
        bp_number, chunk_number, *bitplane = struct.unpack(self.packet_format, message)

        #Calc channel number and Bitplane of channel
        channel_number=(bp_number+1) % self.number_of_channels
        bp_number=bp_number // self.number_of_channels
        
        #Get sent Bitplane list and but in converted numpy array
        bp_channel_array=np.asarray(bitplane, dtype=np.uint8)

        #Unpack bits
        bp_channel_raw=np.unpackbits(bp_channel_array)
        
        #Convert values to signed integer 16 BIT
        bp_channel_received=bp_channel_raw.astype(np.int16)

        #Binary compare channel in message buffer with received Bitplane
        self._buffer[chunk_number % self.cells_in_buffer][:,channel_number]=self._buffer[chunk_number % self.cells_in_buffer][:,channel_number] | bp_channel_received << bp_number

        return chunk_number

    def record_send_and_play(self, indata, outdata, frames, time, status):

        #Get message from sound card buffer
        msg=np.frombuffer(indata, np.int16).reshape(self.frames_per_chunk, self.number_of_channels)
       
        #Iterate Total Bitplanes (32)

        for c in range((16*self.number_of_channels)-1,-1,-1):
                #Get bitplane of channel
                bitplane=msg[:,((c+1) % self.number_of_channels)] >> (c // self.number_of_channels) & 1
                #Conversion bits to unsigned integer
                bitplane_raw=bitplane.astype(np.uint8)
                #Pack bits to packet
                bitpack=np.packbits(bitplane_raw)

                #Create message with structure
                message = struct.pack(self.packet_format, c, self.recorded_chunk_number, *(bitpack))

                #Send message
                self.sending_sock.sendto(message, (self.destination_IP_addr, self.destination_port))

        #Increase chunk number
        self.recorded_chunk_number = (self.recorded_chunk_number + 1) % self.MAX_CHUNK_NUMBER    

        chunk = self._buffer[self.played_chunk_number % self.cells_in_buffer]
        self._buffer[self.played_chunk_number % self.cells_in_buffer] = self.generate_zero_chunk()
        self.played_chunk_number = (self.played_chunk_number + 1) % self.cells_in_buffer
        outdata[:] = chunk

        if __debug__:
            sys.stderr.write("."); sys.stderr.flush()

                
if __name__ == "__main__":
    intercom = Intercom_bitplanes()
    parser = intercom.add_args()
    args = parser.parse_args()
    intercom.init(args)
    intercom.run()
