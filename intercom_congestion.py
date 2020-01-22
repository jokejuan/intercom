# Exploiting binaural redundancy.

#Current Version 1.5 - status: working
#Version history

#1.5 - Extending comments for description
#1.4 - Implementación versión Mono
#1.3 - Correción Error Congestión
#1.2 - Comments review
#1.1 - Comments in code
#1.0 - implementation solution congestion

import numpy as np
import sys
import struct
#The Math library is imported to perform the necessary calculations
import math
from intercom_bitplanes import Intercom_bitplanes
from intercom_buffer import Intercom_buffer

class Intercom_binaural(Intercom_bitplanes):

  def init(self, args):
    Intercom_bitplanes.init(self, args)

    #Initialize missed packets to 32 on stereo (16 if mono)
    #We define the initial value of the control buffer (count the packets that have arrived)    
    self.missed_packets_rec=16*self.number_of_channels
    self.missed_packets_own=16*self.number_of_channels
    self.missed_current=16*self.number_of_channels
    self.packages_to_sent=16*self.number_of_channels #(32=we await 32 packets)
    
    #Initialize Control buffer with 32 on stereo (16 if mono) missed packets in each position.
    #If it is 32 means that no package has arrived
    #If it is 0 means that all packages have arrived
    self._buffer_control=np.full([self.chunks_to_buffer*2,2],16*self.number_of_channels,dtype=np.uint8)

    #Add "B" number of packets to send(awaited packages)
    #Add "B" number of missed packets to struct format (congestion value)
    self.packet_format = f"!HBBB{self.frames_per_chunk//8}B"

    if self.number_of_channels == 2:
      self.record_send_and_play = self.record_send_and_play_stereo

  def receive_and_buffer(self):
    message, source_address = self.receiving_sock.recvfrom(Intercom_bitplanes.MAX_MESSAGE_SIZE)

    #Get data and missed packets and awaited packets of recipient (missed_packets_rec)
    chunk_number, bitplane_number, packages_to_receive, self.missed_packets_rec,*bitplane = struct.unpack(self.packet_format, message)
    bitplane = np.asarray(bitplane, dtype=np.uint8)
    bitplane = np.unpackbits(bitplane)
    bitplane = bitplane.astype(np.int16)
    self._buffer[chunk_number % self.cells_in_buffer][:, bitplane_number%self.number_of_channels] |= (bitplane << bitplane_number//self.number_of_channels)

    #Decrease missed packets by 1
    self._buffer_control[chunk_number % self.cells_in_buffer][0]-=1
    self._buffer_control[chunk_number % self.cells_in_buffer][1]=packages_to_receive 
    return chunk_number

  def record_and_send(self, indata):
    #The change to the SIGN-MAGNITUDE coding is established
  
    #The bitplane of the sign is extracted (bitplane 15)
    sign_plane = indata >> 15
    
    #The absolute value is made to make it positive
    indata_abs = abs(np.copy(indata))
    
    #sign plane is replaced with the bitplane of the original value
    package = sign_plane << 15 | indata_abs
   
    #The maximum bitplane in use of all values that is 1 after the sign is searched (limit)
    valmax=np.max(indata_abs)
    limite=math.ceil(math.log2(valmax))
    self.packages_to_sent=(limite+1)*self.number_of_channels #(add sign plane to limit)

    #Minimum 2 bitplane is always sent (the rest depending on the congestion)
    #Sending the first bitplane (BIT 15)
    for bitplane_number in range(self.number_of_channels*16-1, (self.number_of_channels*16)-1-(self.number_of_channels), -1):
      bitplane = (package[:, bitplane_number%self.number_of_channels] >> bitplane_number//self.number_of_channels) & 1
      bitplane = bitplane.astype(np.uint8)
      bitplane = np.packbits(bitplane)

      message = struct.pack(self.packet_format, self.recorded_chunk_number, bitplane_number, self.packages_to_sent, self.missed_packets_own, *bitplane)
      self.sending_sock.sendto(message, (self.destination_IP_addr, self.destination_port))
      
    #Sending the next bitplane (max bitplane in use after the sign) (limit calculated above)
    for bitplane_number in range(self.number_of_channels*(limite)-1, (self.number_of_channels*(limite-1)-(self.number_of_channels)), -1):
      bitplane = (package[:, bitplane_number%self.number_of_channels] >> bitplane_number//self.number_of_channels) & 1
      bitplane = bitplane.astype(np.uint8)
      bitplane = np.packbits(bitplane)

      message = struct.pack(self.packet_format, self.recorded_chunk_number, bitplane_number, self.packages_to_sent,  self.missed_packets_own, *bitplane)
      self.sending_sock.sendto(message, (self.destination_IP_addr, self.destination_port))

    #The remaining bitplans are sent depending on the congestion
    for bitplane_number in range((self.number_of_channels*(limite-2)), 0 + self.missed_packets_rec-1, -1):
      bitplane = (package[:, bitplane_number%self.number_of_channels] >> bitplane_number//self.number_of_channels) & 1
      bitplane = bitplane.astype(np.uint8)
      bitplane = np.packbits(bitplane)

      message = struct.pack(self.packet_format, self.recorded_chunk_number, bitplane_number, self.packages_to_sent,  self.missed_packets_own, *bitplane)
      self.sending_sock.sendto(message, (self.destination_IP_addr, self.destination_port))

      #sys.stderr.write("\nMSG: {}".format(bitplane_number)); sys.stderr.flush()
       
    self.recorded_chunk_number = (self.recorded_chunk_number + 1) % self.MAX_CHUNK_NUMBER
        
  def record_send_and_play_stereo(self, indata, outdata, frames, time, status):
    
    #BINAURAL
    indata[:,0] -= indata[:,1]

    #Obtaining congestion depending on the packages arrived and packages expected
    #We consider the limit for not obtaining a false congestion value
    self.missed_current=abs((16*self.number_of_channels)-(self._buffer_control[(self.played_chunk_number) % self.cells_in_buffer][1])-(self._buffer_control[(self.played_chunk_number) % self.cells_in_buffer][0]))    
    
    #Calculation to optimize and recover congestion
    self.missed_packets_own = int(self.missed_packets_own*0.8 + self.missed_current*0.2)-1

    #When the congestion is 0, this value will be -1, so we reset to 0
    if self.missed_packets_own<0:
      self.missed_packets_own=0

    self.record_and_send(indata)

    #*** Sign magnitude reconstruction ***#
    #Extract the bitplane 15
    rec_sign_plane = self._buffer[ self.played_chunk_number % self.cells_in_buffer ] >> 15
    
    #Extract the bitplanes 14 - 0
    magnitude = self._buffer[ self.played_chunk_number % self.cells_in_buffer ] & 0x7FFF

    #Recover the original value of the magnitude with respect to the sign by a mathematical calculation
    recovered_chunk=magnitude+(magnitude*rec_sign_plane*2)
    
    #Write the value in the playback buffer
    self._buffer[ self.played_chunk_number % self.cells_in_buffer ]=recovered_chunk

    #Recover Binaural channel
    self._buffer[self.played_chunk_number % self.cells_in_buffer][:,0] += self._buffer[self.played_chunk_number % self.cells_in_buffer][:,1]

    #Set missed packages for next round for current buffer slot
    #Reset the buffer_control for the current value
    self._buffer_control[ self.played_chunk_number % self.cells_in_buffer ][0] = 16*self.number_of_channels
    self._buffer_control[ self.played_chunk_number % self.cells_in_buffer ][1] = 16*self.number_of_channels

    #Reproduce sound data
    self.play(outdata)

def record_send_and_play(self, indata, outdata, frames, time, status):
    
    #Obtaining congestion depending on the packages arrived and packages expected
    #We consider the limit for not obtaining a false congestion value
    self.missed_current=abs((16*self.number_of_channels)-(self._buffer_control[(self.played_chunk_number) % self.cells_in_buffer][1])-(self._buffer_control[(self.played_chunk_number) % self.cells_in_buffer][0]))    
    
    #Calculation to reset congestion
    self.missed_packets_own = int(self.missed_packets_own*0.8 + self.missed_current*0.2)-1

    ##When the congestion is 0, this value will be -1, so we reset to 0
    if self.missed_packets_own<0:
      self.missed_packets_own=0

    self.record_and_send(indata)

    #*** Sign magnitude reconstruction ***#
    #Extract the bitplane 15
    rec_sign_plane = self._buffer[ self.played_chunk_number % self.cells_in_buffer ] >> 15
    
    #Extract the bitplanes 14 - 0
    magnitude = self._buffer[ self.played_chunk_number % self.cells_in_buffer ] & 0x7FFF

    #Recover the original value of the magnitude with respect to the sign by a mathematical calculation
    recovered_chunk=magnitude+(magnitude*rec_sign_plane*2)
    
    #Write the value in the playback buffer
    self._buffer[ self.played_chunk_number % self.cells_in_buffer ]=recovered_chunk

    #Set missed packages for next round for current buffer slot

    #Reset the buffer_control for the current value
    self._buffer_control[ self.played_chunk_number % self.cells_in_buffer ][0] = 16*self.number_of_channels
    self._buffer_control[ self.played_chunk_number % self.cells_in_buffer ][1] = 16*self.number_of_channels

    #Reproduce sound data
    self.play(outdata)

if __name__ == "__main__":
    intercom = Intercom_binaural()
    parser = intercom.add_args()
    args = parser.parse_args()
    intercom.init(args)
    intercom.run()
