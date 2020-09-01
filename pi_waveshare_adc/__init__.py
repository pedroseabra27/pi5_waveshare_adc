"""pi_waveshare_adc
===================

A performant Python library to read the RPi Waveshare ADC expansion board.
"""

from ._version import __version__
from pi_waveshare_adc._adc import Register, RegisterStatus, \
    AnalogInput, Gain, CurrentSource, ClockOutput, SamplingRate, SPICommand, \
    ADS1256, WiringPiADS1256, PiGPIOADS1256

__all__ = (
    'WiringPiADS1256', 'PiGPIOADS1256', 'ADS1256', 'Register',
    'RegisterStatus', 'AnalogInput', 'Gain', 'CurrentSource', 'ClockOutput',
    'SamplingRate', 'SPICommand'
)
