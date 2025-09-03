"""pi_waveshare_adc
===================

A performant Python library to read the RPi Waveshare ADC expansion board.
"""

from ._version import __version__
from pi5_waveshare_adc._adc import Register, RegisterStatus, \
    AnalogInput, Gain, CurrentSource, ClockOutput, SamplingRate, SPICommand, \
    ADS1256, PiGPIOADS1256

# WiringPiADS1256 disabled for RPi 5 compatibility
# Use PiGPIOADS1256 instead for better performance

__all__ = (
    'PiGPIOADS1256', 'ADS1256', 'Register',
    'RegisterStatus', 'AnalogInput', 'Gain', 'CurrentSource', 'ClockOutput',
    'SamplingRate', 'SPICommand'
)
