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

# !!! INFO !!!
# !! after 2 seconds some noise for 3 seconds !!
# BEST results:
# WINDOWS: -s 8192 -cb 8
# LINUX: -s (bigger_than_2048) -cb (16 or 32)

#VERSION 1.6 - STATUS WORKING
# 1.6 - FIX: minor error in congestion calculation and bitplane for loop
# 1.5 - revision comments and changes bitplane cutting formula
# 1.4 - comments and code description
# 1.3 - revision and cleanup form code, dynamic array caclulation
# 1.2 - implementes empty plane consideration and precision factor
# 1.1 - fix arrays, working without empty plane consideration
# 1.0 - prototype, problems with array size

import struct
import numpy as np
import math
from intercom import Intercom
from intercom_empty import Intercom_empty
import pywt as wt

# Number of levels of the DWT
levels = 4

# Set wavelet to use
wavelet = 'bior3.5'

# Set padding form = "symmetric" (DO NOT CHANGE)
padding = "periodization"

# Set overlapping samples
overlap=4

# Set precision factor
factor_precision = 10  #(max 1000)

if __debug__:
    import sys

class Intercom_DWT(Intercom_empty):
        def init(self, args):
            Intercom_empty.init(self, args)
            self.samples = 0

            #Create empty chunk for slice calcultation
            empty_channel=np.zeros(self.frames_per_chunk + overlap)

            #Calc coefficient of empty chunk
            self.coeffs_empty = wt.wavedec(empty_channel, wavelet=wavelet, level=levels, mode=padding)

            #Calc coefficient array and slice structure
            self.arr_empty, self.slice_structure = wt.coeffs_to_array(self.coeffs_empty)

            #Create empty coefficient array buffer
            self._buffer_coeffs = [None] * self.cells_in_buffer

            for i in range(self.cells_in_buffer):
                    self._buffer_coeffs[i] = self.generate_zero_arr()

            #Set variables for previous, current and next chunk for considering overlapping samples
            self._last = self.generate_zero_chunk()
            self._current = self.generate_zero_chunk()
            self._next = self.generate_zero_chunk()
            
            #Increase max bitplanes to 64 (2 channels)
            self.max_NOBPTS=32*self.number_of_channels
            self.NOBPTS=self.max_NOBPTS
            self.NORB = self.max_NOBPTS

            #Create empty coefficiente array as template
            self.arr_temp= np.zeros((len(self.arr_empty),2),dtype=np.int32)

            #Set packet size depending on overlapping bitplanes
            if(overlap>0):
                self.packet_format = f"!HBB{(len(self.arr_empty)//8)+1}B"
            else:
                self.packet_format = f"!HBB{(len(self.arr_empty)//8)}B"


            #Some status messages for development
            #sys.stderr.write("\nFPC{}".format(self.frames_per_chunk)); sys.stderr.flush()
            #sys.stderr.write("\nFPCMAS{}".format(self.frames_per_chunk + overlap)); sys.stderr.flush()
            #sys.stderr.write("\nCOEFFS{}".format(len(self.coeffs_empty))); sys.stderr.flush()
            #sys.stderr.write("\narr_empty{}".format(self.arr_empty.shape)); sys.stderr.flush()

            #set buffer for skipped bitplanes
            self.skipped_bitplanes = [0]*self.cells_in_buffer
            

        #Function for creating empty array in coefficient buffer (32 bit)
        def generate_zero_arr(self):
            cell = np.zeros((len(self.arr_empty),2),dtype=np.int32)
            return cell

        #Function receive and buffer
        def receive_and_buffer(self):
            message, source_address = self.receiving_sock.recvfrom(Intercom.MAX_MESSAGE_SIZE)
            received_chunk_number, received_bitplane_number, self.NORB, *bitplane = struct.unpack(self.packet_format, message)
            bitplane = np.asarray(bitplane, dtype=np.uint8)
            bitplane = np.unpackbits(bitplane)
            bitplane = bitplane.astype(np.int32)

            #Check if overlap mode
            if(overlap>0):
                bitplane = bitplane[:-(len(bitplane)-len(self.arr_empty))]

            #Write received bitplane to coefficient buffer
            self._buffer_coeffs[received_chunk_number % self.cells_in_buffer][:, received_bitplane_number%self.number_of_channels] |= (bitplane << received_bitplane_number//self.number_of_channels)
            self.received_bitplanes_per_chunk[received_chunk_number % self.cells_in_buffer] += 1
            return received_chunk_number

        def send_bitplane(self, arr_data, bitplane_number):
            #Get bitplante of coefficient array
            bitplane = (arr_data[:, bitplane_number%self.number_of_channels] >> bitplane_number//self.number_of_channels) & 1

            #Check empty bitplane. If empty, dont send and increase empty counter
            if np.any(bitplane):
                bitplane = bitplane.astype(np.uint8)
                bitplane = np.packbits(bitplane)
                message = struct.pack(self.packet_format, self.recorded_chunk_number, bitplane_number, self.received_bitplanes_per_chunk[(self.played_chunk_number+1) % self.cells_in_buffer]+1, *bitplane)
                self.sending_sock.sendto(message, (self.destination_IP_addr, self.destination_port))
            else:
                self.skipped_bitplanes[self.recorded_chunk_number % self.cells_in_buffer] += 1

        def send(self, indata):

            #refresh last, current and next chunk
            self._last[:] = self._current
            self._current[:] = self._next
            self._next[:] = indata

            #set base indata
            newindata = indata

            #calc new indata if overlapping is active
            if(overlap>0):
                newindata = np.append(self._last[-(overlap//2):],self._current,axis=0)
                newindata = np.append(newindata,self._next[:+(overlap//2)],axis=0)
                
            #Transformation of sign & magnitude
            signs = newindata & 0x8000
            magnitudes = abs(newindata)
            newindata = signs | magnitudes

            #Calculation of congestion, consider empty planes 
            self.NOBPTS = int(0.75*self.NOBPTS + 0.25*(self.NORB + self.skipped_bitplanes[(self.played_chunk_number+1) % self.cells_in_buffer]))
            
            #Reset empty bitplanes
            self.skipped_bitplanes[(self.played_chunk_number+1) % self.cells_in_buffer] = 0
            #Recover congestion
            self.NOBPTS += 2

            #Failover for range of MAXBPTS 
            if self.NOBPTS > self.max_NOBPTS:
                self.NOBPTS = self.max_NOBPTS
                
            last_BPTS = self.max_NOBPTS - self.NOBPTS - 1

            #start creating coefficient arrays for sending
            for ch in range (self.number_of_channels):
                #get coefficients from new indata with help of DWT transform
                coeffs = wt.wavedec(newindata[:,ch], wavelet=wavelet, level=levels, mode=padding)
                arr_float, slices = wt.coeffs_to_array(coeffs)

                #multiply by precision factor for better recover-result (max 1000)
                self.arr_temp[:,ch] = np.multiply(arr_float,factor_precision)

            #Always send first 2 bitplanes
            self.send_bitplane(self.arr_temp, self.max_NOBPTS-1)
            self.send_bitplane(self.arr_temp, self.max_NOBPTS-2)

            #Send rest depending of congestion
            for bitplane_number in range(self.max_NOBPTS-3, last_BPTS, -1):
                self.send_bitplane(self.arr_temp, bitplane_number)

            #Refresh recorded chunk number
            self.recorded_chunk_number = (self.recorded_chunk_number + 1) % self.MAX_CHUNK_NUMBER

        def record_send_and_play_stereo(self, indata, outdata, frames, time, status):

            #Binaural calculation L=L-R
            indata[:,0] -= indata[:,1]

            #send indata
            self.send(indata)

            #start recovering samples out of coefficient arrays
            for ch in range (self.number_of_channels):
                #Get received coefficient array from buffer for channel X
                arr_chan = self._buffer_coeffs[self.played_chunk_number % self.cells_in_buffer][:,ch]
                #Inverse precision calculation
                arr_chan = np.divide(arr_chan,factor_precision)
                #Get coefficientes from array with help of saved slice structure
                coeffs_chan = wt.array_to_coeffs(arr_chan, self.slice_structure, output_format="wavedec")

                #Get samples with help of inverse DWT transform
                recover = wt.waverec(coeffs_chan, wavelet=wavelet, mode=padding)

                #Check if overlappinf is active
                if(overlap>0):
                    #Delete first and last overlapping samples
                    self._buffer[self.played_chunk_number % self.cells_in_buffer][:,ch]=(recover.astype(np.int16))[+(overlap//2):-(overlap//2)]
                else:
                    self._buffer[self.played_chunk_number % self.cells_in_buffer][:,ch]=(recover.astype(np.int16))

            #Clean coefficient buffer position
            self._buffer_coeffs[self.played_chunk_number % self.cells_in_buffer]=self.generate_zero_arr()

            #Inverse sign magnitude calculation
            chunk = self._buffer[self.played_chunk_number % self.cells_in_buffer]
            signs = chunk >> 15
            magnitudes = chunk & 0x7FFF
            chunk = magnitudes + magnitudes*signs*2

            #Write chunk with correct sign and magnitude to play-buffer           
            self._buffer[self.played_chunk_number % self.cells_in_buffer] = chunk

            #Inverse binaural calculation L=L+R
            self._buffer[self.played_chunk_number % self.cells_in_buffer][:,0] += self._buffer[self.played_chunk_number % self.cells_in_buffer][:,1]

            #Play data
            self.play(outdata)
            self._buffer[self.played_chunk_number % self.cells_in_buffer] = self.generate_zero_chunk()
            self.received_bitplanes_per_chunk [self.played_chunk_number % self.cells_in_buffer] = 0



if __name__ == "__main__":
	intercom = Intercom_DWT()
	parser = intercom.add_args()
	args = parser.parse_args()
	intercom.init(args)
	intercom.run()
