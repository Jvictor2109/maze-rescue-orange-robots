#!/usr/bin/env python

import time
import threading
import pigpio

class sensor(threading.Thread):
   def __init__(self, pi, OUT, S2, S3, S0=None, S1=None, OE=None):
      threading.Thread.__init__(self)
      self._pi = pi

      self._OUT = OUT
      self._S2 = S2
      self._S3 = S3

      self._mode_OUT = pi.get_mode(OUT)
      self._mode_S2 = pi.get_mode(S2)
      self._mode_S3 = pi.get_mode(S3)

      pi.write(OUT, 0)
      pi.set_mode(S2, pigpio.OUTPUT)
      pi.set_mode(S3, pigpio.OUTPUT)

      self._S0 = S0
      self._S1 = S1
      self._OE = OE

      if (S0 is not None) and (S1 is not None):
         self._mode_S0 = pi.get_mode(S0)
         self._mode_S1 = pi.get_mode(S1)
         pi.set_mode(S0, pigpio.OUTPUT)
         pi.set_mode(S1, pigpio.OUTPUT)

      if OE is not None:
         self._mode_OE = pi.get_mode(OE)
         pi.set_mode(OE, pigpio.OUTPUT)
         pi.write(OE, 0)

      self.set_sample_size(50)
      self._period = 0.25
      self.set_frequency(1)

      self._rgb_black = [0]*3
      self._rgb_white = [10000]*3

      self._set_filter(3)

      self.Hertz=[0]*3
      self._Hertz=[0]*3
      self.tally=[1]*3
      self._tally=[1]*3
      self._delay=[0.1]*3
      self._edge = 0
      self._start_tick = 0
      self._last_tick = 0

      self._cb_OUT = pi.callback(OUT, pigpio.RISING_EDGE, self._cbf)
      self._cb_S2 = pi.callback(S2, pigpio.EITHER_EDGE, self._cbf)
      self._cb_S3 = pi.callback(S3, pigpio.EITHER_EDGE, self._cbf)

      self._read = True
      self.daemon = True

      # --- Estado para classificação cega ---
      self._rolling_max = 0
      self._ciclos = 0
      self.ABSOLUTE_BLACK = 800  # ajusta em laboratório com o sensor montado

      self.start()

   def _cbf(self, g, l, t):
      if g == self._OUT:
         if self._edge == 0:
            self._start_tick = t
         else:
            self._last_tick = t
         self._edge += 1
      else:
         if g == self._S2:
            if l == 0:
               self._edge = 0
               return
            else:
               colour = 2
         else:
            if l == 0:
               colour = 1
            else:
               colour = 0

         if self._edge > 1:
            self._edge -= 1
            td = pigpio.tickDiff(self._start_tick, self._last_tick)
            self._Hertz[colour] = (1000000 * self._edge) / td
            self._tally[colour] = self._edge
         else:
            self._Hertz[colour] = 0
            self._tally[colour] = 0

         self._edge = 0

         if colour == 1:
            for i in range(3):
               self.Hertz[i] = self._Hertz[i]
               self.tally[i] = self._tally[i]

   def run(self):
      while True:
         if self._read:
            next_time = time.time() + self._period
            self._pi.set_mode(self._OUT, pigpio.INPUT)

            self._set_filter(0)
            time.sleep(self._delay[0])
            self._set_filter(2)
            time.sleep(self._delay[2])
            self._set_filter(1)
            time.sleep(self._delay[1])
            self._pi.write(self._OUT, 0)
            self._set_filter(3)

            delay = next_time - time.time()
            if delay > 0.0:
               time.sleep(delay)

            for c in range(3):
               if self.Hertz[c]:
                  dly = self._samples / float(self.Hertz[c])
                  if dly < 0.001:
                     dly = 0.001
                  elif dly > 0.5:
                     dly = 0.5
                  self._delay[c] = dly
         else:
            time.sleep(0.1)

   # --- Lógica de identificação cega ---

   def get_cor(self):
      hz = self.Hertz[:]
      total = sum(hz)

      # Fallback absoluto — hardware conhecido, não precisa de calibração
      if total < self.ABSOLUTE_BLACK:
         self._ciclos += 1
         return "preto"

      # Actualiza rolling max
      if total > self._rolling_max:
         self._rolling_max = total

      self._ciclos += 1

      # Ainda sem referência suficiente
      if self._rolling_max < 300:
         return "incerto"

      # Preto pelo rolling max
      if total < self._rolling_max * 0.20:
         return "preto"

      # Análise de rácios por canal
      r, g, b = hz
      soma = total

      r_ratio = r / soma
      g_ratio = g / soma
      b_ratio = b / soma

      if r_ratio > 0.50 and g_ratio < 0.30:
         return "vermelho"
      if r_ratio > 0.45 and g_ratio > 0.25 and b_ratio < 0.20:
         return "laranja"
      if g_ratio > 0.45 and r_ratio < 0.35:
         return "verde"
      if b_ratio > 0.40 and r_ratio < 0.30:
         return "azul"

      return "branco"

   # --- Métodos originais mantidos ---

   def pause(self):
      self._read = False

   def resume(self):
      self._read = True

   def get_Hertz(self):
      return self.Hertz

   def get_rgb(self, top=255):
      rgb = [0]*3
      for c in range(3):
         v = self.Hertz[c] - self._rgb_black[c]
         s = self._rgb_white[c] - self._rgb_black[c]
         p = top * v / s
         if p < 0: p = 0
         elif p > top: p = top
         rgb[c] = p
      return rgb

   def cancel(self):
      self._cb_S3.cancel()
      self._cb_S2.cancel()
      self._cb_OUT.cancel()
      self.set_frequency(0)
      self._set_filter(3)
      self._pi.set_mode(self._OUT, self._mode_OUT)
      self._pi.set_mode(self._S2, self._mode_S2)
      self._pi.set_mode(self._S3, self._mode_S3)
      if (self._S0 is not None) and (self._S1 is not None):
         self._pi.set_mode(self._S0, self._mode_S0)
         self._pi.set_mode(self._S1, self._mode_S1)
      if self._OE is not None:
         self._pi.write(self._OE, 1)
         self._pi.set_mode(self._OE, self._mode_OE)

   def set_black_level(self, rgb):
      for i in range(3): self._rgb_black[i] = rgb[i]

   def set_white_level(self, rgb):
      for i in range(3): self._rgb_white[i] = rgb[i]

   def _set_filter(self, f):
      if f == 0:   S2 = 0; S3 = 0
      elif f == 1: S2 = 1; S3 = 1
      elif f == 2: S2 = 0; S3 = 1
      else:        S2 = 1; S3 = 0
      self._pi.write(self._S2, S2); self._pi.write(self._S3, S3)

   def get_frequency(self):
      return self._frequency

   def set_frequency(self, f):
      if f == 0:   S0 = 0; S1 = 0
      elif f == 1: S0 = 0; S1 = 1
      elif f == 2: S0 = 1; S1 = 0
      else:        S0 = 1; S1 = 1
      if (self._S0 is not None) and (self._S1 is not None):
         self._frequency = f
         self._pi.write(self._S0, S0)
         self._pi.write(self._S1, S1)
      else:
         self._frequency = None

   def set_update_period(self, t):
      if (t >= 0.1) and (t < 2.0):
         self._period = t

   def set_sample_size(self, samples):
      if (samples < 10) or (samples > 1000):
         samples = 50
      self._samples = samples


if __name__ == "__main__":

   pi = pigpio.pi()

   s = sensor(pi, 24, 22, 23, 4, 17, 18)
   s.set_update_period(0.2)
   s.set_frequency(2)   # 20%
   s.set_sample_size(20)

   # Determina ABSOLUTE_BLACK em laboratório e define aqui
   s.ABSOLUTE_BLACK = 11086

   try:
      while True:
         time.sleep(0.2)
         hz = s.get_Hertz()
         print(f"Total: {sum(hz):.0f} | R:{hz[0]:.0f} G:{hz[1]:.0f} B:{hz[2]:.0f}")
   except KeyboardInterrupt:
      s.cancel()
      pi.stop()

   def is_preto(self):
      hz = self.get_Hertz()

      if hz[0]  > 6000 and hz[1] < 6000 and hz[2] < 6000:
         return "preto"
      else:
         return "branco"