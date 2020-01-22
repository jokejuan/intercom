#   chunk i-1     chunk i     chunk i+1
# +------------+------------+------------+
# |          OO|OOOOOOOOOOOO|OO          |
# +------------+------------+------------+
#
# O = sample
#
# (In this example, only 2 samples are required from adajact chunks)
#
# The number of ajacent samples depends on the Wavelet
# transform. However, considering that usually a chunk has a number of
# samples larger than the number of coefficients of the Wavelet
# filters, we don't need to be aware of this detail if we work with
# chunks.

import struct
import numpy as np
import math
from intercom import Intercom
from intercom_empty import Intercom_empty
#from intercom_dfc import Intercom_DFC
import pywt as wt

# Number of levels of the DWT
#levels = 4

# Wavelet used
wavelet = 'bior3.5'

#padding = "symmetric"
padding = "periodization"

factor_precision = 1000

if __debug__:
    import sys

class Intercom_DWT(Intercom_empty):
        def init(self, args):
            Intercom_empty.init(self, args)
            self.samples = 0
            self.packet_format = f"!HBB{self.frames_per_chunk//8}B"
            empty_channel=np.zeros(self.frames_per_chunk)
            self.coeffs_empty = wt.wavedec(empty_channel, wavelet=wavelet, mode=padding)
	    #self.coeffs_empty = wt.wavedec(empty_channel, wavelet=wavelet, level=levels, mode=padding)
            arr_empty, self.slice_structure = wt.coeffs_to_array(self.coeffs_empty)
            #self._buffer = [self.generate_zero_chunk()] * self.cells_in_buffer
            self._buffer_coeffs = [None] * self.cells_in_buffer
            self.max_NOBPTS=32*self.number_of_channels
            self.NOBPTS=self.max_NOBPTS
            self.NORB = self.max_NOBPTS
            self.arr_temp= np.zeros((self.frames_per_chunk, self.number_of_channels), np.int32)

            for i in range(self.cells_in_buffer):
                    self._buffer_coeffs[i] = self.generate_zero_arr()

        def generate_zero_arr(self):
            cell = np.zeros((self.frames_per_chunk, self.number_of_channels), np.int32)
            return cell

        def receive_and_buffer(self):
            message, source_address = self.receiving_sock.recvfrom(Intercom.MAX_MESSAGE_SIZE)
            received_chunk_number, received_bitplane_number, self.NORB, *bitplane = struct.unpack(self.packet_format, message)
            bitplane = np.asarray(bitplane, dtype=np.uint8)
            bitplane = np.unpackbits(bitplane)
            bitplane = bitplane.astype(np.int32)
            self._buffer_coeffs[received_chunk_number % self.cells_in_buffer][:, received_bitplane_number%self.number_of_channels] |= (bitplane << received_bitplane_number//self.number_of_channels)
            self.received_bitplanes_per_chunk[received_chunk_number % self.cells_in_buffer] += 1
            return received_chunk_number

        def send_bitplane(self, arr_data, bitplane_number):
            bitplane = (arr_data[:, bitplane_number%self.number_of_channels] >> bitplane_number//self.number_of_channels) & 1
            bitplane = bitplane.astype(np.uint8)
            bitplane = np.packbits(bitplane)
            message = struct.pack(self.packet_format, self.recorded_chunk_number, bitplane_number, self.received_bitplanes_per_chunk[(self.played_chunk_number+1) % self.cells_in_buffer]+1, *bitplane)
            self.sending_sock.sendto(message, (self.destination_IP_addr, self.destination_port))

        def send(self, indata):
            signs = indata & 0x8000
            magnitudes = abs(indata)
            indata = signs | magnitudes
            
            for ch in range (self.number_of_channels):
                coeffs = wt.wavedec(indata[:,ch], wavelet=wavelet, mode=padding)
                #coeffs = wt.wavedec(indata[:,ch], wavelet=wavelet, level=levels, mode=padding)
                arr_float, slices = wt.coeffs_to_array(coeffs)
                self.arr_temp[:,ch] = np.multiply(arr_float,factor_precision)

            #sys.stderr.write("\nARR_S [{}]{} {} {} {}".format(self.recorded_chunk_number, self.arr_temp[0],self.arr_temp[1],self.arr_temp[2],self.arr_temp[3])); sys.stderr.flush()

            #sys.stderr.write("\nCOE_S [{}]{}".format(self.recorded_chunk_number, coeffs[0])); sys.stderr.flush()
            
            last_BPTS = self.max_NOBPTS - self.NOBPTS - 1
            self.send_bitplane(self.arr_temp, self.max_NOBPTS-1)
            self.send_bitplane(self.arr_temp, self.max_NOBPTS-2)

            for bitplane_number in range(self.max_NOBPTS-3, last_BPTS, -1):
                self.send_bitplane(self.arr_temp, bitplane_number)

            self.recorded_chunk_number = (self.recorded_chunk_number + 1) % self.MAX_CHUNK_NUMBER

        def record_send_and_play_stereo(self, indata, outdata, frames, time, status):
            indata[:,0] -= indata[:,1]

            #sys.stderr.write("\nSEN_0 [{}]{}".format(self.recorded_chunk_number,indata[:,0])); sys.stderr.flush()
            #sys.stderr.write("\nSEN_1 [{}]{}".format(self.recorded_chunk_number,indata[:,1])); sys.stderr.flush()
            
            self.send(indata)
            arr = self._buffer_coeffs[self.played_chunk_number % self.cells_in_buffer]
            

            #sys.stderr.write("\nARR_R [{}]{} {} {} {}".format(self.played_chunk_number, arr[0],arr[1],arr[2],arr[3])); sys.stderr.flush()
            
            for ch in range (self.number_of_channels):
                arr_chan = self._buffer_coeffs[self.played_chunk_number % self.cells_in_buffer][:,ch]
                arr_chan = np.divide(arr_chan,factor_precision)
                coeffs_chan = wt.array_to_coeffs(arr_chan, self.slice_structure, output_format="wavedec")
                
                recover = wt.waverec(coeffs_chan, wavelet=wavelet, mode=padding)

                self._buffer[self.played_chunk_number % self.cells_in_buffer][:,ch]=recover.astype(np.int16)
                #self._buffer[self.played_chunk_number % self.cells_in_buffer][:,ch]=np.asarray(recover,dtype=np.int16)


            self._buffer_coeffs[self.played_chunk_number % self.cells_in_buffer]=self.generate_zero_arr()
            chunk = self._buffer[self.played_chunk_number % self.cells_in_buffer]
            signs = chunk >> 15
            magnitudes = chunk & 0x7FFF
            #chunk = ((~signs & magnitudes) | ((-magnitudes) & signs))
            chunk = magnitudes + magnitudes*signs*2

            #sys.stderr.write("\nREC_{} [{}]{}".format(0,self.played_chunk_number, chunk[:,0])); sys.stderr.flush()
            #sys.stderr.write("\nREC_{} [{}]{}".format(1,self.played_chunk_number, chunk[:,1])); sys.stderr.flush()
            
            self._buffer[self.played_chunk_number % self.cells_in_buffer] = chunk
            self._buffer[self.played_chunk_number % self.cells_in_buffer][:,0] += self._buffer[self.played_chunk_number % self.cells_in_buffer][:,1]
            self.play(outdata)
            self._buffer[self.played_chunk_number % self.cells_in_buffer] = self.generate_zero_chunk()
            self.received_bitplanes_per_chunk [self.played_chunk_number % self.cells_in_buffer] = 0



if __name__ == "__main__":
	intercom = Intercom_DWT()
	parser = intercom.add_args()
	args = parser.parse_args()
	intercom.init(args)
	intercom.run()
