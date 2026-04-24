#!/usr/bin/env python

# 2015-07-02
# TCS3200.py
# Public Domain

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

      pi.write(OUT, 0) # disable output gpio
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
         pi.write(OE, 0) # enable device

      self.set_sample_size(50)

      self._period = 0.25 # 4 readings per second

      self.set_frequency(1) # 2%

      self._rgb_black = [0]*3
      self._rgb_white = [10000]*3

      self._set_filter(3) # Clear

      self.Hertz=[0]*3 # latest triplet
      self._Hertz=[0]*3 # current values

      self.tally=[1]*3 # latest triplet
      self._tally=[1]*3 # current values

      self._delay=[0.1]*3 # tune delay to get TALLY readings

      self._edge = 0

      self._start_tick = 0
      self._last_tick = 0

      self._cb_OUT = pi.callback(OUT, pigpio.RISING_EDGE, self._cbf)
      self._cb_S2 = pi.callback(S2, pigpio.EITHER_EDGE, self._cbf)
      self._cb_S3 = pi.callback(S3, pigpio.EITHER_EDGE, self._cbf)

      self._read = True

      self.daemon = True

      self.start()

   def _cbf(self, g, l, t):

      if g == self._OUT:
         if self._edge == 0:
            self._start_tick = t
         else:
            self._last_tick = t
         self._edge += 1

      else: # Must be transition between colour samples
         if g == self._S2:
            if l == 0: # Clear -> Red
               self._edge = 0
               return
            else:      # Blue -> Green
               colour = 2
         else:
            if l == 0: # Green -> Clear
               colour = 1
            else:      # Red -> Blue
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

         # Have we a new set of RGB?
         if colour == 1:
            for i in range(3):
               self.Hertz[i] = self._Hertz[i]
               self.tally[i] = self._tally[i]

   def run(self):
      while True:
         if self._read:

            next_time = time.time() + self._period

            self._pi.set_mode(self._OUT, pigpio.INPUT) # enable output gpio

            # The order Red -> Blue -> Green -> Clear is needed by the
            # callback function so that each S2/S3 transition triggers
            # a state change.  The order was chosen so that a single
            # gpio changes state between the change in colour to be
            # sampled.

            self._set_filter(0) # Red
            time.sleep(self._delay[0])
            self._set_filter(2) # Blue
            time.sleep(self._delay[2])
            self._set_filter(1) # Green
            time.sleep(self._delay[1])
            self._pi.write(self._OUT, 0) # disable output gpio
            self._set_filter(3) # Clear

            delay = next_time - time.time()

            if delay > 0.0:
               time.sleep(delay)

            # Tune the next set of delays to get reasonable results
            # as quickly as possible.

            for c in range(3):
               # Calculate dly needed to get a decent number of samples
               if self.Hertz[c]:
                  dly = self._samples / float(self.Hertz[c])

                  # Constrain dly to reasonable values
                  if dly < 0.001:
                     dly = 0.001
                  elif dly > 0.5:
                     dly = 0.5

                  self._delay[c] = dly

         else:
            time.sleep(0.1)

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
         if p < 0:
            p = 0
         elif p > top:
            p = top
         rgb[c] = p
      return rgb

   def cancel(self):
      self._cb_S3.cancel()
      self._cb_S2.cancel()
      self._cb_OUT.cancel()

      self.set_frequency(0) # off

      self._set_filter(3) # Clear

      self._pi.set_mode(self._OUT, self._mode_OUT)
      self._pi.set_mode(self._S2, self._mode_S2)
      self._pi.set_mode(self._S3, self._mode_S3)

      if (self._S0 is not None) and (self._S1 is not None):
         self._pi.set_mode(self._S0, self._mode_S0)
         self._pi.set_mode(self._S1, self._mode_S1)

      if self._OE is not None:
         self._pi.write(self._OE, 1) # disable device
         self._pi.set_mode(self._OE, self._mode_OE)

   def set_black_level(self, rgb):
      for i in range(3):
         self._rgb_black[i] = rgb[i]

   def set_white_level(self, rgb):
      for i in range(3):
         self._rgb_white[i] = rgb[i]

   def _set_filter(self, f):
      if f == 0: # Red
         S2 = 0; S3 = 0
      elif f == 1: # Green
         S2 = 1; S3 = 1
      elif f == 2: # Blue
         S2 = 0; S3 = 1
      else: # Clear
         S2 = 1; S3 = 0

      self._pi.write(self._S2, S2); self._pi.write(self._S3, S3)

   def get_frequency(self):
      return self._frequency

   def set_frequency(self, f):
      if f == 0: # off
         S0 = 0; S1 = 0
      elif f == 1: # 2%
         S0 = 0; S1 = 1
      elif f == 2: # 20%
         S0 = 1; S1 = 0
      else: # 100%
         S0 = 1; S1 = 1

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

   import sys

   RED=21
   GREEN=20
   BLUE=16

   def wait_for_return(str):
      if sys.hexversion < 0x03000000:
         raw_input(str)
      else:
         input(str)

   pi = pigpio.pi()

   p = 0.333

   # Bottom view
   #
   # GND    VCC
   # OE     OUT
   # S1     S2
   # S0     S3

   s = sensor(pi, 24, 22, 23, 4, 17, 18)
   s.set_update_period(p)
   s.set_frequency(2) # 20%
   s.set_sample_size(20)

   wait_for_return("Calibrating black, press RETURN to start")

   for i in range(5):
      time.sleep(p)
      rgb = s.get_Hertz()
      print(rgb)
      s.set_black_level(rgb)

   wait_for_return("Calibrating white, press RETURN to start")

   for i in range(5):
      time.sleep(p)
      rgb = s.get_Hertz()
      print(rgb)
      s.set_white_level(rgb)

   if True:
   #try:
      while True:
         time.sleep(p)
         rgb = s.get_rgb()
         r = rgb[0]
         g = rgb[1]
         b = rgb[2]
         print(rgb, s.get_Hertz(), s.tally)
         pi.set_PWM_dutycycle(RED, r)
         pi.set_PWM_dutycycle(GREEN, g)
         pi.set_PWM_dutycycle(BLUE, b)

   if True:
   #except:

      print("cancelling")
      s.cancel()
      pi.stop()