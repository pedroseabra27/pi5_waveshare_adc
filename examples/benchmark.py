"""
What this does: it benchmarks the various configurations of the ADC.

There are three variables:

* The backend (``wiringpi`` vs ``pigpio``).
* The sampling rate (one of ``2.5, 5.0, 10.0, 15.0, 25.0, 30.0, 50.0, 60.0,
    100.0, 500.0, 1000.0, 2000.0, 3750.0, 7500.0, 15000.0, 30000.0``).
* And the sampling method (one of ``trigger, standby, individual,
    individual_mux, chunked, chunked_mux_x8``).

When run, by default, it samples for 10 seconds for each combination of the
three variables and prints the ``estimated sampling rate``, the
``number of samples skipped when chunked``, and the ``min`` and ``max`` voltage
read during sampling.

It lets us compare the performance of the methods and whether each method
is able to sample at the requested rate or whether samples are skipped.

This can also be run with e.g. ``python benchmark.py pigpio 100 chunked`` to
benchmark only this specific configuration.

.. note::

    This need to be run with sudo privileges when using the pigpio backend.

Muxing
------

When muxing multiple channels, rather than sampling a single channel,
we continuously cycle between the channels. This reduces the total sampling
rate as per the datasheet. The print result is the total sampling rate across
all the channels, not per channel.

Results
-------

On the RPi4, pigpio is significantly faster than the wiringpi backend at high
sampling rates. But they can both achieve 30kHz (although the wiringpi may
lose some samples now and then).
"""
import sys
from time import perf_counter
from pi_waveshare_adc import WiringPiADS1256, PiGPIOADS1256, AnalogInput

# Adjustable resistor of the same board:
RESISTOR = AnalogInput.POS_AIN0 | AnalogInput.NEG_AINCOM
RESISTOR_NEG = AnalogInput.NEG_AIN0 | AnalogInput.POS_AINCOM


def do_chunked_sampling(adc, rate, duration, name):
    with adc:
        adc.data_rate = rate

        adc.cal_self()
        scale = adc.v_per_digit

        ts = perf_counter()

        skipped = 0
        i = 0
        n = 0

        min_val = 2 ** 25
        max_val = - 2 ** 25

        first_sample_t = None
        last_sample_t = None

        if name == 'chunked_mux_x8':
            channels = [RESISTOR, RESISTOR_NEG] * 4
        else:
            channels = [RESISTOR, ]

        chunk_size = max(2, min(200, int(rate / len(channels))))
        with adc.chunked_sampling(chunk_size, 10, 10, *channels) as iterator:
            for arr, s, t in iterator:
                if arr is None:
                    raise TimeoutError

                if first_sample_t is None:
                    first_sample_t = t
                last_sample_t = t

                if s:
                    skipped += 1

                n = len(arr)
                i += n

                max_val = max(max_val, max(arr))
                min_val = min(min_val, min(arr))

                if perf_counter() - ts > duration:
                    break

        estimated_rate = 0
        if last_sample_t != first_sample_t:
            estimated_rate = (i - n) / (last_sample_t - first_sample_t) * 1e6

    return estimated_rate, skipped, min_val * scale, max_val * scale


def read_samples(adc, rate, duration, name):
    with adc:
        adc.data_rate = rate
        adc.set_sampling_channel(RESISTOR)

        adc.cal_self()
        scale = adc.v_per_digit

        if name == 'standby':
            adc.standby()

        ts = perf_counter()
        i = 0
        min_val = 2 ** 25
        max_val = - 2 ** 25

        while perf_counter() - ts < duration:
            if name == 'trigger':
                value = adc.read_sample_trigger()
            elif name == 'standby':
                value = adc.read_sample_from_standby()
            elif name == 'individual':
                value = adc.read_next_sample()
            elif name == 'individual_mux':
                value = adc.read_next_sample_switch_channel(RESISTOR)

            i += 1
            max_val = max(max_val, value)
            min_val = min(min_val, value)

    estimated_rate = i / (perf_counter() - ts)
    return estimated_rate, 0, min_val * scale, max_val * scale


def main():
    if len(sys.argv) > 1:
        if sys.argv[1].lower() == 'pigpio':
            devices = [PiGPIOADS1256()]
        elif sys.argv[1].lower() == 'wiringpi':
            devices = [WiringPiADS1256()]
        else:
            raise ValueError
    else:
        devices = [WiringPiADS1256(), PiGPIOADS1256()]

    if len(sys.argv) > 2:
        rates = [float(sys.argv[2])]
    else:
        rates = [2.5, 5.0, 10.0, 15.0, 25.0, 30.0, 50.0, 60.0, 100.0, 500.0,
                 1000.0, 2000.0, 3750.0, 7500.0, 15000.0, 30000.0]

    functions = {
        'trigger': read_samples,
        'standby': read_samples,
        'individual': read_samples,
        'individual_mux': read_samples,
        'chunked': do_chunked_sampling,
        'chunked_mux_x8': do_chunked_sampling,
    }

    if len(sys.argv) > 3:
        name = sys.argv[3]
        functions = {name: functions[name]}

    duration = 10
    n = len(devices) * len(rates) * len(functions) * duration
    print(f'Please wait, this will take about {n} seconds\n')

    print('Rate,Backend,Sampling method,Estimated rate (total),'
          '# Skipped,Min Voltage,Max Voltage')
    for rate in rates:
        for device in devices:
            for name, func in functions.items():
                measured_rate, skipped, min_val, max_val = func(
                    device, rate, duration, name)

                dev_name = device.__class__.__name__[:-7]
                print(f'{rate},{dev_name},{name},{measured_rate:0.2f},'
                      f'{skipped},{min_val:0.4f},{max_val:0.4f}')


main()
