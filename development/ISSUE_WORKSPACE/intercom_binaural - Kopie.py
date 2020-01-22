# Exploiting binaural redundancy.

#Current Version 1.6 - status: working
#Version history

#1.1 - Comments in code
#1.0 - implementation solution congestion

import numpy as np
import sys
import struct
# IMPORTAR BILBO PARA CALCULO
import math
from intercom_bitplanes import Intercom_bitplanes
from intercom_buffer import Intercom_buffer

class Intercom_binaural(Intercom_bitplanes):

  def init(self, args):
    Intercom_bitplanes.init(self, args)

    #Init missed packets to 32
    #DEFINIMOS VALOR INICIAL DE BUFFER DE CONTROL (CUENTA PAQUETES LLEGADO)    
    self.missed_packets_rec=32
    self.missed_packets_own=32
    self.missed_last=32
    self.missed_current=32
    #self.recorded_chunk_number=0

    #initializar BUFFER DE CONTROL CONO TODO EN 32
    #si ES = TODOS PAQUETES DE CHUNK SI ES 32 NINGUNA HA LLEGADO
    #Init Control buffer with 32 missed packets in each position
    self._buffer_control=np.full(self.chunks_to_buffer*2,32,dtype=np.uint8)

    #ANIADIMOS UN CAMPO A PAQUETES DE ENVIO (VALOR DE CONGESTION)
    #Add "B" number of missed packets to struct format
    self.packet_format = f"!HBB{self.frames_per_chunk//8}B"

    if self.number_of_channels == 2:
      self.record_send_and_play = self.record_send_and_play_stereo

  def receive_and_buffer(self):
    message, source_address = self.receiving_sock.recvfrom(Intercom_bitplanes.MAX_MESSAGE_SIZE)

    #Get missed packets of recipient (missed_packets_rec)
    chunk_number, bitplane_number, self.missed_packets_rec,*bitplane = struct.unpack(self.packet_format, message)
    bitplane = np.asarray(bitplane, dtype=np.uint8)
    bitplane = np.unpackbits(bitplane)
    bitplane = bitplane.astype(np.int16)
    self._buffer[chunk_number % self.cells_in_buffer][:, bitplane_number%self.number_of_channels] |= (bitplane << bitplane_number//self.number_of_channels)

    #Decrease missed packets by 1
    self._buffer_control[chunk_number % self.cells_in_buffer]-=1 

    return chunk_number

  def record_and_send(self, indata):
  #CAMBIO SIGNO MAGNITUD 
  
  #EXTRAER plano signo (plano de bit 15)
    sign_plane = indata >> 15
    
    #CAMBIAR VALORES A POSITIVO
    indata_abs = abs(np.copy(indata))
    
    #REEMPLEZAR plano bit 15 con plano bit 15 original (signo)
    package = sign_plane << 15 | indata_abs

    
    #encontrar bitplane maximo en uso de valor
    valmax=np.max(indata_abs)
    limite=math.ceil(math.log2(valmax))

    sys.stderr.write("\nLIMITE: {}".format(limite)); sys.stderr.flush()

    #OBJETIVO por minimo envia 2 planes y depues depende de congestion
    #ENVIO BIT 15
    for bitplane_number in range(self.number_of_channels*16-1, (self.number_of_channels*16)-(self.number_of_channels*(-1)), -1):
      bitplane = (package[:, bitplane_number%self.number_of_channels] >> bitplane_number//self.number_of_channels) & 1
      bitplane = bitplane.astype(np.uint8)
      bitplane = np.packbits(bitplane)

      message = struct.pack(self.packet_format, self.recorded_chunk_number, bitplane_number, self.missed_packets_own, *bitplane)
      self.sending_sock.sendto(message, (self.destination_IP_addr, self.destination_port))
      
      sys.stderr.write("\nMSG: {}".format(bitplane_number)); sys.stderr.flush()

    #ENVIO siguente bit en uso (limite)
    for bitplane_number in range(self.number_of_channels*limite, (self.number_of_channels*limite)-(self.number_of_channels*(-1)), -1):
      bitplane = (package[:, bitplane_number%self.number_of_channels] >> bitplane_number//self.number_of_channels) & 1
      bitplane = bitplane.astype(np.uint8)
      bitplane = np.packbits(bitplane)

      message = struct.pack(self.packet_format, self.recorded_chunk_number, bitplane_number, self.missed_packets_own, *bitplane)
      self.sending_sock.sendto(message, (self.destination_IP_addr, self.destination_port))

      sys.stderr.write("\nMSG: {}".format(bitplane_number)); sys.stderr.flush()

    #ENVIO resto depende de congestion
    for bitplane_number in range(self.number_of_channels*limite-1, 0 + self.missed_packets_rec-1, -1):
      bitplane = (package[:, bitplane_number%self.number_of_channels] >> bitplane_number//self.number_of_channels) & 1
      bitplane = bitplane.astype(np.uint8)
      bitplane = np.packbits(bitplane)

      message = struct.pack(self.packet_format, self.recorded_chunk_number, bitplane_number, self.missed_packets_own, *bitplane)
      self.sending_sock.sendto(message, (self.destination_IP_addr, self.destination_port))

      sys.stderr.write("\nMSG: {}".format(bitplane_number)); sys.stderr.flush()
       
    self.recorded_chunk_number = (self.recorded_chunk_number + 1) % self.MAX_CHUNK_NUMBER
        
  def record_send_and_play_stereo(self, indata, outdata, frames, time, status):
    
    #PARTA BINAURAL
    indata[:,0] -= indata[:,1]

    #OBTENER CONGESTION PROPIO DEPENDE DE PAQUETES LLEGADO Y ESPERADO
    self.missed_current=(self._buffer_control[(self.played_chunk_number) % self.cells_in_buffer])
    
    #CALCULO CONGESTION ESPERADO RESPECTO DE PAQUETES PERDIDOS Y RESTAR 1 PARA RECUPERACION
    self.missed_packets_own = int(self.missed_packets_own*0.8 + self.missed_current*0.2)-1
    
    #CONTROL <0
    if self.missed_packets_own<0:
      self.missed_packets_own=0
    
    #sys.stderr.write("\nNR: {} CURRENT: {} OWN: {}".format(self.played_chunk_number, self.missed_current,self.missed_packets_own)); sys.stderr.flush()
    
    self.record_and_send(indata)

    #RECONSTRUCCION SIGNO MAGNITUD
    #EXTRAE BITPLAN 15
    rec_sign_plane = self._buffer[ self.played_chunk_number % self.cells_in_buffer ] >> 15
    
    #EXTRAE BITPLAN 14 HASTA 0
    magnitude = self._buffer[ self.played_chunk_number % self.cells_in_buffer ] & 0x7FFF

    #magnitude+(madgnitude*rec_sign_plane*2) 

    #RECUPERAR VALOR ORIGINAL DE MAGNITUD RESPECTO DE SIGNO
    recovered_chunk=magnitude+(magnitude*rec_sign_plane*2)
    
    #ESCRIBE VALOR EN BUFFER DE REPRODUCIR
    self._buffer[ self.played_chunk_number % self.cells_in_buffer ]=recovered_chunk

    #Lookup missed packages of current buffer slot
    self._buffer[self.played_chunk_number % self.cells_in_buffer][:,0] += self._buffer[self.played_chunk_number % self.cells_in_buffer][:,1]

    #Set missed packages for next round for current buffer slot

    #RESETEAR CONTADOR BUFFER DE CONTROL PARA HUECO ACTUAL
    self._buffer_control[ self.played_chunk_number % self.cells_in_buffer ] = 32

    #Reduce own congestions exponentially
    self.play(outdata)

if __name__ == "__main__":
    intercom = Intercom_binaural()
    parser = intercom.add_args()
    args = parser.parse_args()
    intercom.init(args)
    intercom.run()
