"""pi_waveshare_adc
===================

A performant Python library to read the RPi Waveshare ADC expansion board.
"""

from ._version import __version__
from pi5_waveshare_adc._adc import Register, RegisterStatus, \
    AnalogInput, Gain, CurrentSource, ClockOutput, SamplingRate, SPICommand, \
    ADS1256, PiGPIOADS1256, LgpioADS1256

# WiringPiADS1256 disabled for RPi 5 compatibility
# Use LgpioADS1256 for RPi 5 or PiGPIOADS1256 for older models

__all__ = (
    'LgpioADS1256', 'PiGPIOADS1256', 'ADS1256', 'Register',
    'RegisterStatus', 'AnalogInput', 'Gain', 'CurrentSource', 'ClockOutput',
    'SamplingRate', 'SPICommand'
)
