pi_waveshare_adc
================

A performant Python library to read the Raspberry Pi Waveshare ADC expansion board.

Complete documentation is `here <https://matham.github.io/pi_waveshare_adc/index.html>`_

.. image:: https://img.shields.io/pypi/pyversions/pi_waveshare_adc.svg
    :target: https://pypi.python.org/pypi/pi_waveshare_adc/
    :alt: Supported Python versions

.. image:: https://img.shields.io/pypi/v/pi_waveshare_adc.svg
    :target: https://pypi.python.org/pypi/pi_waveshare_adc/
    :alt: Latest Version on PyPI

.. image:: https://coveralls.io/repos/github/matham/pi_waveshare_adc/badge.svg?branch=master
    :target: https://coveralls.io/github/matham/pi_waveshare_adc?branch=master
    :alt: Coverage status

.. image:: https://github.com/matham/pi_waveshare_adc/workflows/Python%20application/badge.svg
    :target: https://github.com/matham/pi_waveshare_adc/actions
    :alt: Github action status

Introduction
-------------

``pi_waveshare_adc`` is a Python 3+ library for controlling and sampling the Raspberry Pi
`Waveshare High-Precision AD/DA Expansion Board <https://www.waveshare.com/wiki/High-Precision_AD/DA_Board>`_
built around the 8-channel 24 bit 30Khz `ADS1256 <https://www.ti.com/product/ADS1256>`_ ADC.

It is based on the `PiPyADC <https://github.com/ul-gh/PiPyADC/>`_ library, but has been internally converted
to Cython for C performance to achieve full 30Khz sampling. Additionally, it supports
either the `WiringPi <https://github.com/WiringPi/WiringPi>`_ or a new
`PiGPIO <https://github.com/joan2937/pigpio>`_ backend, now that WiringPi has been deprecated.
Finally, it has a more Pythonic API and has been better proofed against reading corrupted
samples (although it's still impossible to fully guarantee since the RPi is not a real-time system).

This library has been tested on the RPi4 and should similarly work on the RPi3. It relies on the
multiple cores to be able to sample at full 30kHz. Because at sustained 30kHz, one core is fully
engaged. So on older RPis will less cores it may not function well. However, sampling at lower
frequencies (``< 1kHz``) is much gentler on the CPU (see class docs for CPU sleep details).

Installation
-------------

* To install ``pi_waveshare_adc``, ensure python and pigpio is installed properly with
  (WiringPi should be pre-installed, otherwise please install it from source)::

      sudo apt install python3 python3-dev python3-pip python3-pigpio pigpio

* Install ``pi_waveshare_adc`` with::

      python3 -m pip install pi_waveshare_adc
      # or until the next release to pypi you can get it directly from github with
      python3 -m pip install https://github.com/matham/pi_waveshare_adc/archive/refs/heads/master.zip

* Ensure the SPI interface is enabled on the RPi by running ``sudo raspi-config`` and enabling it
  in the interfaces option.

Backends
--------

There are two backends ``pigpio`` or ``wiringpi`` selected by instantiating the ``WiringPiADS1256``
or ``PiGPIOADS1256`` class, respectively.

In testing on the RPI4, the ``pigpio`` was significantly faster than the ``wiringpi`` backend (more than
3x faster on SPI, slightly slower reading the GPIO). Consequently, it is able to sample at higher
frequencies and more consistently achieve ``30kHz``.

Unfortunately, to use ``pigpio``, Python needs to be run with ``sudo`` permissions because we
use ``pigpio`` directly for better performance rather than the daemon. If you get the following
error::

    2020-09-02 01:44:41 initCheckPermitted:
    +---------------------------------------------------------+
    |Sorry, you don't have permission to run this program.    |
    |Try running as root, e.g. precede the command with sudo. |
    +---------------------------------------------------------+

It means you forgot to use sudo. So use e.g.::

    sudo python3 example.py

Usage
-----

Hardware pin configuration must be
set up before initialization phase and can not be changed later.

Register/Configuration Flag settings are initialized, but these
can be changed during runtime via class properties.

Higher performance chunked sampling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

See.

Performance and Data Corruption Considerations
----------------------------------------------

Data could be corrupt if it's taking too long to read e.g. if the pi
hiccups and sampling rate is too large and we read while data is updated.
This is especially the case when muxing.

Our sleep has minimum about 100 us resolution. So e.g. at 30kHz, we can't
afford to sleep while waiting for the input to change, because the time
between samples is 1 / 30kHz = 33us so if we sleep we'll miss samples.
Even at 1kHz sampling, we only have 1000us between samples or about 8-9
sleep cycles. So if we sleep too long, by the time we finish reading the
sample the next sample may already be converted and our read value would be
corrupted.

Consequently, we only sleep if the sampling rate is less than 500Hz.
Because with 2000 us between samples we have 15-20 sleep cycles available,
so the risk is much much less.

The consequence is that when sampling above 500Hz, the CPU will not get
any sleep if we do e.g. chunked reading. This is not a problem on e.g. a pi
4 with 4 cores. :attr:`do_data_ready_delay` controls this behavior.
If the rate is less than or equal to 500Hz and :attr:`do_data_ready_delay`
is True (the default) then we do a minimum sleep while waiting. Otherwise,
we never sleep.
