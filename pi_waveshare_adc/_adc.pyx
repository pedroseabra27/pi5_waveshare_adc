"""ADS1256 ADC
==============
"""

# todo: handle error checking
from posix.types cimport time_t, clockid_t, suseconds_t
from cpython cimport array
from libc.string cimport memset

include "_definitions.pxi"

cdef:
    extern from "time.h" nogil:
        struct timespec:
            time_t tv_sec
            long tv_nsec
        struct timeval:
            time_t tv_sec
            suseconds_t tv_usec

        int nanosleep(timespec *, timespec *)
        int gettimeofday(timeval *, void *)
        void timeradd(timeval *, timeval *, timeval *)

    extern from "errno.h" nogil:
        int ETIMEDOUT

    extern from "string.h" nogil:
        void * memcpy(void *, const void *, size_t)

    extern from "pigpio.h" nogil:
        int PI_INIT_FAILED
        int gpioInitialise()
        void gpioTerminate()
        int spiXfer(unsigned, char *, char *, unsigned)
        int gpioSetMode(unsigned, unsigned)
        int gpioWrite(unsigned, unsigned)
        int gpioRead(unsigned)
        int spiOpen(unsigned, unsigned, unsigned)
        int spiClose(unsigned)

    extern from "wiringPiSPI.h" nogil:
        int wiringPiSPISetupMode(int, int, int)
        int wiringPiSPIDataRW(int, unsigned char *, int)
    extern from "wiringPi.h" nogil:
        void pinMode(int, int)
        void digitalWrite(int, int)
        int digitalRead(int)
        int wiringPiSetupPhys()


DEF MMAP_SHUTDOWN_INDEX = 0
DEF MMAP_SHUTDOWN_DONE_INDEX = 1
DEF MMAP_CHUNK_INDEX = 2
DEF MMAP_TIME_SEC_INDEX = 3
DEF MMAP_TIME_USEC_INDEX = 7
DEF MMAP_HEADER_SIZE = 11

import contextlib
import array
import signal
import errno
import mmap
import os


cdef dict rate_val_to_flag = {}
cdef dict rate_flag_to_val = {}


cdef class ADS1256:
    """
    Hardware pin configuration must be
    set up before initialization phase and can not be changed later.

    Register/Configuration Flag settings are initialized, but these
    can be changed during runtime via class properties.

    Can not set ay adc stuff until it is setup.

    for better performance: change sleep in data ready. Call sustained reading.

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
    """

    @property
    def pga_gain(self):
        """Get/Set ADC programmable gain amplifier which controls the input
        voltage range.

        Reading or setting it will cause a read/write to the ADCON register.

        The available options for the ADS1256 are:
        1, 2, 4, 8, 16, 32 and 64. The range is 2 * Vref / gain so with the
        built in 2.5V Vref, the input range is:

        +-------+---------------+
        | Gain  | Input Voltage |
        +=======+===============+
        | 1     | ±5V           |
        +-------+---------------+
        | 2     | ±2.5V         |
        +-------+---------------+
        | 4     | ±1.25V        |
        +-------+---------------+
        | 8     | ±0.625V       |
        +-------+---------------+
        | 16    | ±312.5mV      |
        +-------+---------------+
        | 32    | ±156.25mV     |
        +-------+---------------+
        | 64    | ±78.125mV     |
        +-------+---------------+

        .. note::

            When changing the gain setting at runtime, with
            status ACAL flag (AUTOCAL_ENABLE) ON, this causes a wait for data
            ready for the calibration process to finish.

            If ACAL is disabled (the default), a manual calibration is
            recommended after changing the gain.

        ``Gain = 1``, ``V_ref = 2.5V`` ==> full-scale input voltage = ``5.00V``,
        corresponding to a 24-bit two's complement output value of
        ``2**23 - 1 = 8388607``.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        cdef unsigned char val
        with nogil:
            val = self._read_reg(REG_ADCON) & 0b111
        return 2 ** val

    @pga_gain.setter
    def pga_gain(self, unsigned char value):
        cdef unsigned char log2val = 0
        cdef unsigned char config = 0
        if value not in (1, 2, 4, 8, 16, 32, 64):
            raise ValueError("Argument must be one of: 1, 2, 4, 8, 16, 32, 64")
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        log2val = int.bit_length(value) - 1
        config = self.adc_config

        with nogil:
            self._write_reg(REG_ADCON, config & 0b11111000 | log2val)
            if self._status & AUTOCAL_ENABLE:
                self._wait_data_ready()

    @property
    def v_per_digit(self):
        """Get ADC LSB weight in volts per numeric output digit.

        I.e. if the integer value read is ``+12``, the the voltage in Volts is
        ``12 * v_per_digit``.

        Readonly: This is a convenience value calculated from
        gain and v_ref setting and **causes** a read of the board register so the
        result should be cached.
        """
        return self.v_ref * 2.0 / (self.pga_gain * (2 ** 23 - 1))

    @v_per_digit.setter
    def v_per_digit(self, value):
        raise AttributeError("This is a read-only attribute")

    @property
    def status(self):
        """Get/Set value of ADC status register, REG_STATUS (8 bit).

        Reading or setting it will cause a read/write to the REG_STATUS register.

        For available settings flag options, see datasheet.

        Note: When enabling the AUTOCAL flag (OFF by default), any subsequent
        change to the BUFEN bit, DRATE register
        (data_rate property) or PGA gain setting (gain property) will cause
        an additional delay for completion of hardware auto-calibration.

        Note: BUFFER_ENABLE (off by default) means the ADC input voltage range
        is limited to (``AVDD-2V``),see datasheet
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        cdef unsigned char val
        with nogil:
            val = self._read_reg(REG_STATUS)
        return val

    @status.setter
    def status(self, unsigned char value):
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        with nogil:
            self._write_reg(REG_STATUS, value)
            self._status = value
            # When AUTOCAL flag has been enabled, a wait_data_ready() is needed here.
            # When AUTOCAL flag had been enabled before and the BUFEN flag has not
            # been changed, this is likely not necessary, but FIXME: Better risk
            # a small possibly unnecessary delay than risk invalid data or settings.
            # Thus, if AUTOCAL flag /is/ enabled, always do a wait_data_ready().
            if self._status & AUTOCAL_ENABLE:
                self._wait_data_ready()

    @property
    def mux(self):
        """Get/Set value of ADC analog input multiplexer register,
        REG_MUX, used for selecting any arbitrary pair of input pins defined in
        the ``AnalogInput`` enum as a differential input channel. For
        single-ended measurements, choose NEG_AINCOM as the second input pin.

        Reading or setting it will cause a read/write to the REG_MUX register.

        Example: ``adc.mux = POS_AIN4 | NEG_AINCOM``.

        Defaults to positive input = ``AIN0``, negative input = ``AINCOM``.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        cdef unsigned char val
        with nogil:
            val = self._read_reg(REG_MUX)
        return val

    @mux.setter
    def mux(self, unsigned char value):
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        with nogil:
            self._write_reg(REG_MUX, value)

    @property
    def adc_config(self):
        """Get/Set value of the ADC configuration register, REG_ADCON.

        Reading or setting it will cause a read/write to the REG_ADCON register.

        Defaults to disable clk out signal (not needed, source of disturbance),
        sensor detect current sources disabled, and GAIN_1 gain.

        .. note::

            When changing the adc config setting at runtime, with
            status ACAL flag (AUTOCAL_ENABLE) ON, this causes a wait for data
            ready for the calibration process to finish.

            If ACAL is disabled (the default), a manual calibration is
            recommended after changing the adc config.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        cdef unsigned char val
        with nogil:
            val = self._read_reg(REG_ADCON)
        return val

    @adc_config.setter
    def adc_config(self, unsigned char value):
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        with nogil:
            self._write_reg(REG_ADCON, value)
            if self._status & AUTOCAL_ENABLE:
                self._wait_data_ready()

    @property
    def data_rate(self):
        """Get/Set value of the ADC output sample data rate by setting
        the DRATE register (REG_DRATE). This configures the hardware integrated
        moving average filter.

        Reading or setting it will cause a read/write to the DRATE register.

        Valid values are::

            2.5, 5.0, 10.0, 15.0, 25.0, 30.0, 50.0, 60.0, 100.0, 500.0, \
1000.0, 2000.0, 3750.0, 7500.0, 15000.0, 30000.0

        Defaults to 10 SPS which places a filter zero at 50 Hz and 60 Hz for line
        noise rejection.

        .. note::

            When changing the data rate setting at runtime, with
            status ACAL flag (AUTOCAL_ENABLE) ON, this causes a wait for data
            ready for the calibration process to finish.

            If ACAL is disabled (the default), a manual calibration is
            recommended after changing the data rate.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        cdef unsigned char val
        with nogil:
            val = self._read_reg(REG_DRATE)

        self._sampling_rate = rate_flag_to_val[val]
        return self._sampling_rate

    @data_rate.setter
    def data_rate(self, double value):
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        if value not in rate_val_to_flag:
            raise ValueError(
                f'{value} is not a valid rate. It must be one '
                f'of {list(sorted(rate_val_to_flag))}')

        cdef unsigned char flag = rate_val_to_flag[value]
        with nogil:
            self._write_reg(REG_DRATE, flag)
            if self._status & AUTOCAL_ENABLE:
                self._wait_data_ready()

        # save it afterwards so e.g. going from 30k to 2.5Hz we don't sleep
        # while it's still going at 30k
        self._sampling_rate = value

    @property
    def gpio(self):
        """Get or set the logic level or direction of the four GPIO pins from
        the REG_IO register.

        Reading or setting it will cause a read/write to the REG_IO register.

        The most significant four bits represent the
        pin direction, and the least significant four bits determine
        the output logic level.

        A timeout/debounce for the reading is not implemented.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        cdef unsigned char val
        with nogil:
            val = 0x0F & self._read_reg(REG_IO)
        return val

    @gpio.setter
    def gpio(self, unsigned char value):
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        with nogil:
            self._write_reg(REG_IO, value)

    @property
    def ofc(self):
        """Get/Set the three offset compensation registers, OFC0..2.

        Reading or setting it will cause a read/write to the OFC0..2 registers.

        This property is a signed integer value representing a 24-bit two's
        complement value in three 8-bit-registers.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        cdef unsigned char ofc0, ofc1, ofc2
        with nogil:
            ofc0 = self._read_reg(REG_OFC0)
            ofc1 = self._read_reg(REG_OFC1)
            ofc2 = self._read_reg(REG_OFC2)

        cdef unsigned int int24_result = ofc2 << 16 | ofc1 << 8 | ofc0
        # Take care of 24-Bit 2's complement.
        if int24_result < 0x800000:
            return int24_result
        else:
            return int24_result - 0x1000000

    @ofc.setter
    def ofc(self, value):
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        cdef int int_val = int(value)
        if int_val < -0x800000 or int_val > 0x7FFFFF:
            raise ValueError("Error: Offset value out of signed int24 range")

        # Generate 24-Bit 2's complement.
        if int_val < 0:
            int_val += 0x1000000

        with nogil:
            self._write_reg(REG_OFC0, int_val & 0xFF)
            int_val >>= 8
            self._write_reg(REG_OFC1, int_val & 0xFF)
            int_val >>= 8
            self._write_reg(REG_OFC2, int_val & 0xFF)

    @property
    def fsc(self):
        """Get/Set the three full-scale adjustment registers, FSC0..2.

        Reading or setting it will cause a read/write to the FSC0..2 registers.

        This property is a unsigned integer value representing a 24-bit value
        in three 8-bit-registers.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        cdef unsigned char fsc0, fsc1, fsc2
        with nogil:
            fsc0 = self._read_reg(REG_FSC0)
            fsc1 = self._read_reg(REG_FSC1)
            fsc2 = self._read_reg(REG_FSC2)

        return fsc2 << 16 | fsc1 << 8 | fsc0

    @fsc.setter
    def fsc(self, value):
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        cdef int int_val = int(value)
        if int_val < 0 or int_val > 0xFFFFFF:
            raise ValueError("Error: This must be a positive int of 24-bit range")

        with nogil:
            self._write_reg(REG_FSC0, int_val & 0xFF)
            int_val >>= 8
            self._write_reg(REG_FSC1, int_val & 0xFF)
            int_val >>= 8
            self._write_reg(REG_FSC2, int_val & 0xFF)

    @property
    def chip_ID(self):
        """Get the 4 bit numeric ID from the ADS chip.
        Useful to check if hardware is connected.

        Reading or setting it will cause a read/write to the status register.

        Value for the ADS1256 on the Waveshare board seems to be a 3.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        cdef unsigned char val
        with nogil:
            self._wait_data_ready()
            val = self._read_reg(REG_STATUS) >> 4
        return val

    @chip_ID.setter
    def chip_ID(self, value):
        raise AttributeError("This is a read-only attribute")

    cdef object _shared_memory
    """shared memory when sampling continuously in forked process.
    """

    cdef object _process_id
    """process id of child process when sampling continuously in forked process.
    """

    cdef unsigned char _last_chunk
    """Last chunk number read in main process.
    """

    cdef int _chunk_size
    """Chunk size requested when started chunked continuous sampling.
    """

    cdef unsigned char _status
    """The last status register value.
    """

    cdef double _sampling_rate
    """The current sampling rate of the adc in Hz.
    """

    cdef int _spi_mode_CPOL

    cdef int _spi_mode_CPHA

    cdef int _data_timeout_us
    """ADS1255/ADS1256 command timing specifications. Do not change.
    Delay between requesting data and reading the bus for
    RDATA, RDATAC and RREG commands (datasheet: t_6 >= 50*CLKIN period).
    """

    cdef int _sync_timeout_us
    """Command-to-command timeout after SYNC and RDATAC
    commands (datasheet: t11)
    """

    cdef int _chip_select_timeout_us
    """See datasheet ADS1256: CS needs to remain low
    for t_10 = 8*T_CLKIN after last SCLK falling edge of a command.
    Because this delay is longer than timeout t_11 for the
    RREG, WREG and RDATA commands of 4*T_CLKIN, we do not need
    the extra t_11 timeout for these commands when using software
    chip select selection and the _chip_select_timeout_us.
    """

    cdef int _t11_timeout_us
    """When using hardware/hard-wired chip select, still a command-
    to command timeout of t_11 is needed as a minimum for the
    RREG, WREG and RDATA commands.
    """

    cdef public double v_ref
    """ADC analog reference input voltage differential 
    (between VREFH and VREFN pins).
    This is only for calculation of output value scale factor.
    """

    cdef public char cs_pin
    """The RPI GPIOs chip select pin number. On the Waveshare board it must be 
    specified because the SPI CS pin is not connected to the ADC CS pin.
    
    If the the CS pin of the SPI bus is connected to the ADC CS pin, this
    should be set to -1 and it won't be used.
    
    The value should match the numbering system used by the backend 
    (e.g. WiringPi or pigpio). It is defaulted to the correct value for each
    backend.
    
    .. note::
    
        This can be set as a constructor parameter to overwrite the default or 
        by setting the property directly before the adc is initialized.
    """

    cdef public char data_ready_pin
    """The RPI GPIOs data ready pin number.
    
    The value should match the numbering system used by the backend 
    (e.g. WiringPi or pigpio). It is defaulted to the correct value for each
    backend.
    
    .. note::
    
        This can be set as a constructor parameter to overwrite the default or 
        by setting the property directly before the adc is initialized.
    """

    cdef public char reset_pin
    """The RPI GPIOs reset pin number. It is not currently used, except to set
    to high.
    
    The value should match the numbering system used by the backend 
    (e.g. WiringPi or pigpio). It is defaulted to the correct value for each
    backend.
    
    .. note::
    
        This can be set as a constructor parameter to overwrite the default or 
        by setting the property directly before the adc is initialized.
    """

    cdef public char power_down_pin
    """The RPI GPIOs power down pin number.
    
    The value should match the numbering system used by the backend 
    (e.g. WiringPi or pigpio). It is defaulted to the correct value for each
    backend.
    
    .. note::
    
        This can be set as a constructor parameter to overwrite the default or 
        by setting the property directly before the adc is initialized.
    """

    cdef public unsigned int spi_frequency
    """SPI clock rate in Hz. 
    
    The ADS1256 supports a minimum of 1/10th of the output
    sample data rate in Hz to 1/4th of the oscillator CLKIN_FREQUENCY which
    results in a value of 1920000 Hz for the Waveshare board.
    
    1920000 is used directly, and works on recent RPIs which doesn't require it 
    be a power of two.
    
    The value should match the SPI frequency used by the backend 
    (e.g. WiringPi or pigpio). It is defaulted to the correct value for each
    backend.
    
    .. note::
    
        This can be set as a constructor parameter to overwrite the default or 
        by setting the property directly before the adc is initialized.
    """

    cdef public int spi_channel
    """The chip select hardware bin controlled by the SPI hardware. 
    
    For the Waveshare board this pin is not even connected, so we don't
    use hardware-controlled CS but :attr:`cs_pin` instead.
    
    .. note::
    
        This can be set as a constructor parameter to overwrite the default or 
        by setting the property directly before the adc is initialized.
    """

    cdef public int clock_frequency
    """Master crystal clock rate in Hz. Default is 7680000 for the Waveshare 
    board.
    
    .. note::
    
        This can be set as a constructor parameter to overwrite the default or 
        by setting the property directly before the adc is initialized.
    """

    cdef public int do_data_ready_delay
    """Whether we sleep while waiting for new data when the sampling rate
    is 500Hz or less. See class description for details.
    """

    cdef public double data_ready_timeout
    """Seconds to wait in case the chip does not respond. 
    See table 21 of ADS1256 datasheet: When using a
    sample rate of 2.5 SPS and issuing a self calibration command,
    the timeout can be up to 1228 milliseconds
    
    .. note::
    
        This can be set as a constructor parameter to overwrite the default or 
        by setting the property directly before the adc is initialized.
    """

    def __init__(
            self, cs_pin=0, data_ready_pin=0, reset_pin=0, power_down_pin=0,
            spi_frequency=1920000, spi_channel=1, clock_frequency=7680000,
            do_data_ready_delay=True, data_ready_timeout=2, v_ref=2.5,
            status=0):
        global rate_val_to_flag, rate_flag_to_val
        if not rate_val_to_flag:
            rate_val_to_flag = {
                float(item.name[6:].replace('_', '.')): item.value
                for item in SamplingRate}
            rate_flag_to_val = {
                flag: val for val, flag in rate_val_to_flag.items()}

        self.cs_pin = cs_pin
        self.data_ready_pin = data_ready_pin
        self.reset_pin = reset_pin
        self.power_down_pin = power_down_pin
        self.spi_frequency = spi_frequency
        self.spi_channel = spi_channel
        self.clock_frequency = clock_frequency
        self.do_data_ready_delay = do_data_ready_delay
        self.data_ready_timeout = data_ready_timeout
        self.v_ref = v_ref
        self._shared_memory = None
        self._last_chunk = 0
        self._process_id = 0
        self._chunk_size = 0

        self._spi_mode_CPOL = 0
        self._spi_mode_CPHA = 1
        self._data_timeout_us = int(1 + (50 * 1000000) / self.clock_frequency)
        self._sync_timeout_us = int(1 + (24 * 1000000) / self.clock_frequency)
        self._chip_select_timeout_us = int(
            1 + (8 * 1000000) / self.clock_frequency)
        self._t11_timeout_us = int(1 + (4 * 1000000) / self.clock_frequency)

        # Status register not yet set, only variable written to avoid multiple
        # triggering of the AUTOCAL procedure by changing other register flags
        self._status = status
        # the adc starts at 30kHz
        self._sampling_rate = 30_000

    def setup_adc(self):
        """Initializes the ADC so it is ready to be sampled or configured.

        Each :meth:`setup_adc` must be followed by a :meth:`release_adc` when
        done with the ADC, before :meth:`setup_adc` can be called again or
        before exiting python.

        E.g. a valid sequence is ``setup -> sampling -> release -> setup ->
        sampling -> release -> ...``.

        You should consider using the context manager because it's safer and
        more pythonic.

        E.g.::

            adc = ADC()
            with adc:
                adc.cal_self()

        It handles safely calling :meth:`setup_adc` and :meth:`release_adc`.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot setup during sampling')

        cdef int i = 0
        cdef unsigned char pin = 0
        cdef unsigned char[3] pins = [
            self.cs_pin, self.reset_pin, self.power_down_pin]

        self._init()

        with nogil:
            # Only one GPIO input
            self._set_pin_direction(self.data_ready_pin, False)

            # GPIO Outputs. Only the cs_pin is currently actively used. ~RESET and
            # ~PDWN must be set to static logic HIGH level if not hardwired:
            for i in range(3):
                self._set_pin_direction(pins[i], True)
                self._write_pin(pins[i], 1)

        self._init_spi()

        # At hardware initialisation, a settling time for the oscillator
        # is necessary before doing any register access if it was in power down
        # This is approx. 30ms, according to the datasheet.
        with nogil:
            self._delay_us(30_000)
            self._wait_data_ready()
        # Device reset for defined initial state
        self.reset()

        # Configure ADC registers:
        self.mux = POS_AIN0 | NEG_AINCOM
        self.adc_config = CLKOUT_OFF | SDCS_OFF | GAIN_1
        self.data_rate = 10
        self.gpio = 0x00
        self.status = self._status

    def release_adc(self):
        """Releases resources setup in :meth:`setup_adc`.
        """
        if self._shared_memory is not None:
            raise TypeError(
                'Cannot release adc during sampling. Stop sampling first')

    def __enter__(self):
        self.setup_adc()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release_adc()
        return False

    cdef object _check_process(self):
        """Returns if process exists.
        """
        try:
            os.kill(self._process_id, 0)
        except OSError as err:
            if err.errno == errno.ESRCH:
                # ESRCH == No such process
                return False
            elif err.errno == errno.EPERM:
                # EPERM clearly means there's a process to deny access to
                return True
            else:
                # According to "man 2 kill" possible error values are
                # (EINVAL, EPERM, ESRCH)
                raise
        else:
            return True

    cdef void _chip_select(self) nogil:
        """Selects the chip if using the CS pin.
        """
        # If chip select hardware pin is connected to SPI bus hardware pin or
        # hardwired to GND, do nothing.
        if self.cs_pin >= 0:
            self._write_pin(self.cs_pin, 0)

    def chip_select(self):
        """(Part of the low-level API) Selects the chip if using the CS pin.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        with nogil:
            self._chip_select()

    cdef void _chip_release(self) nogil:
        """Releases the chip selection if using the CS pin and does the proper 
        timeout.
        """
        # Release chip select and implement t_11 timeout
        if self.cs_pin >= 0:
            self._delay_us(self._chip_select_timeout_us)
            self._write_pin(self.cs_pin, 1)
        else:
            # The minimum t_11 timeout between commands, see datasheet Figure 1.
            self._delay_us(self._t11_timeout_us)

    def chip_release(self):
        """(Part of the low-level API) Releases the chip selection if using
        the CS pin and does the proper timeout.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        with nogil:
            self._chip_release()

    cdef int _send_uint8_1_byte(self, unsigned char d1) nogil:
        """Writes a byte over the SPI bus.
        """
        cdef unsigned char tx[1]
        cdef unsigned char rx[1]
        tx[0] = d1
        return self._write_read_spi(<unsigned char *>tx, <unsigned char *>rx, 1)

    cdef int _send_uint8_2_byte(self, unsigned char d1, unsigned char d2) nogil:
        """Writes two bytes over the SPI bus.
        """
        cdef unsigned char tx[2]
        cdef unsigned char rx[2]
        tx[0] = d1
        tx[1] = d2
        return self._write_read_spi(<unsigned char *>tx, <unsigned char *>rx, 2)

    cdef int _send_uint8_3_byte(
            self, unsigned char d1, unsigned char d2, unsigned char d3) nogil:
        """Writes three bytes over the SPI bus.
        """
        cdef unsigned char tx[3]
        cdef unsigned char rx[3]
        tx[0] = d1
        tx[1] = d2
        tx[2] = d3
        return self._write_read_spi(<unsigned char *>tx, <unsigned char *>rx, 3)

    cdef int _read_uint8_1_byte(self, unsigned char *data) nogil:
        """Writes and reads a byte over the SPI bus. The byte written is 0xFF.
        """
        cdef unsigned char tx = 0xFF
        return self._write_read_spi(&tx, data, 1)

    cdef int _read_uint8_3_byte(self, unsigned char *data) nogil:
        """Writes and reads three bytes over the SPI bus. The bytes written is 
        0xFFFFFF.
        """
        cdef unsigned char tx[3]
        tx[0] = tx[1] = tx[2] = 0xFF
        return self._write_read_spi(<unsigned char *>tx, data, 3)

    cdef int _wait_data_ready_level(self, unsigned char level) nogil:
        """Delays until the configured DRDY input pin is not ``level`` by the 
        ADS1256 hardware or until the :attr:`data_ready_timeout` in seconds has 
        passed.

        The minimum necessary data_ready_timeout when not using the hardware
        pin, can be up to approx. one and a half second, see datasheet.
        """
        cdef unsigned char drdy_level = 0
        cdef double start = self._get_time_seconds()
        cdef double elapsed = 0

        # Waits for DRDY pin to go to active low
        drdy_level = self._read_pin(self.data_ready_pin)

        while (drdy_level == level) and (elapsed < self.data_ready_timeout):
            if self.do_data_ready_delay and self._sampling_rate <= 500.:
                self._sleep(1)

            drdy_level = self._read_pin(self.data_ready_pin)

            elapsed = self._get_time_seconds() - start

        if elapsed >= self.data_ready_timeout:
            return ETIMEDOUT

    cdef int _wait_data_ready(self) nogil:
        """Delays until the configured DRDY input pin is set LOW by the 
        ADS1256 hardware or until the :attr:`data_ready_timeout` in seconds has 
        passed.
        """
        return self._wait_data_ready_level(1)

    def wait_data_ready(self):
        """(Part of the low-level API) Delays until the configured DRDY input
        pin is set LOW by the ADS1256 hardware or until the
        :attr:`data_ready_timeout` in seconds has passed.

        The minimum necessary data_ready_timeout when not using the hardware
        pin, can be up to approx. one and a half second, see datasheet.

        Manually invoking this function is necessary when the datasheet calls
        for waiting until data ready is low.
        """
        with nogil:
            self._wait_data_ready()

    cdef int _wait_data_ready_clear(self) nogil:
        return self._wait_data_ready_level(0)

    def wait_data_ready_clear(self):
        """(Part of the low-level API) Delays until the configured DRDY input
        pin is set HIGH by the ADS1256 hardware or until the
        :attr:`data_ready_timeout` in seconds has passed.
        """
        with nogil:
            self._wait_data_ready_clear()

    cdef unsigned char _read_reg(self, unsigned char register) nogil:
        """Returns data byte read from the specified register address byte.
        """
        cdef unsigned char read = 0
        self._chip_select()
        self._send_uint8_2_byte(CMD_RREG | register, 0x00)
        self._delay_us(self._data_timeout_us)
        self._read_uint8_1_byte(&read)
        # Release chip select and implement t_11 timeout
        self._chip_release()
        return read

    def read_reg(self, unsigned char register):
        """(Part of the low-level API) Returns data byte read from the
        specified register address byte.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot read during sampling')

        cdef unsigned char val
        with nogil:
            val = self._read_reg(register)
        return val

    cdef void _write_reg(self, unsigned char register, unsigned char data) nogil:
        """Writes data byte to the specified register.
        """
        self._chip_select()
        # Tell the ADS chip which register to start writing at,
        # how many additional registers to write (0x00) and send data
        self._send_uint8_3_byte(CMD_WREG | register, 0x00, data & 0xFF)
        # Release chip select and implement t_11 timeout
        self._chip_release()

    def write_reg(self, unsigned char register, unsigned char data):
        """(Part of the low-level API) Writes data byte to the specified
        register.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        with nogil:
            self._write_reg(register, data)

    def cal_self_offset(self):
        """Perform an input zero calibration using chip-internal
        reference switches.

        Sets the ADS1255/ADS1256 OFC register.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        with nogil:
            self._chip_select()
            self._send_uint8_1_byte(CMD_SELFOCAL)
            self._wait_data_ready()
            # Release chip select and implement t_11 timeout
            self._chip_release()

    def cal_self_gain(self):
        """Perform an input full-scale calibration
        using chip-internal circuitry connected to VREFP and VREFN.

        Sets the ADS1255/ADS1256 FSC register.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        with nogil:
            self._chip_select()
            self._send_uint8_1_byte(CMD_SELFGCAL)
            self._wait_data_ready()
            # Release chip select and implement t_11 timeout
            self._chip_release()

    def cal_self(self):
        """Perform an input zero and full-scale two-point-calibration
        using chip-internal circuitry connected to VREFP and VREFN.

        Sets the ADS1255/ADS1256 OFC and FSC registers.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        with nogil:
            self._chip_select()
            self._send_uint8_1_byte(CMD_SELFCAL)
            self._wait_data_ready()
            # Release chip select and implement t_11 timeout
            self._chip_release()

    def cal_system_offset(self):
        """Set the ADS1255/ADS1256 OFC register such that the
        current input voltage corresponds to a zero output value.
        The input multiplexer must be set to the appropriate pins first.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        with nogil:
            self._chip_select()
            self._send_uint8_1_byte(CMD_SYSOCAL)
            self._wait_data_ready()
            # Release chip select and implement t_11 timeout
            self._chip_release()

    def cal_system_gain(self):
        """Set the ADS1255/ADS1256 FSC register such that the current
        input voltage corresponds to a full-scale output value.
        The input multiplexer must be set to the appropriate pins first.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        with nogil:
            self._chip_select()
            self._send_uint8_1_byte(CMD_SYSGCAL)
            self._wait_data_ready()
            # Release chip select and implement t_11 timeout
            self._chip_release()

    def power_down(self):
        """Powers down the chip.

        It doesn't wait for it to actually power down, which the chip will do
        after some time (20 DRDY cycles, see datasheet).

        The chip is automatically powered up when it is initialized with
        :meth:`setup_adc` or :meth:`run_adc`. So to power it back up, you'll
        have to release it and then set it up again.

        Use this just before :meth:`run_adc` exits or before calling
        :meth:`release_adc` so the ADC will power down when you're done.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        with nogil:
            self._chip_select()
            self._write_pin(self.power_down_pin, 0)
            # Release chip select and implement t_11 timeout
            self._chip_release()

    def standby(self):
        """Put chip in low-power standby mode.

        Call this to enter the low-power state if you are pausing sampling,
        and don't want to power down and then reset the chip.

        Wake it up with :meth:`wakeup`.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        with nogil:
            self._chip_select()
            self._send_uint8_1_byte(CMD_STANDBY)
            # Release chip select and implement t_11 timeout
            self._chip_release()

    def wakeup(self):
        """Wake up the chip from standby mode.

        Once woken up, the chip is ready to be sampled again.

        Call :meth:`standby` to enter standby mode again.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        with nogil:
            self._chip_select()
            self._send_uint8_1_byte(CMD_WAKEUP)
            # Release chip select and implement t_11 timeout
            self._chip_release()

    def reset(self):
        """Reset all registers except CLK0 and CLK1 bits.

        It waits until the reset is complete.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        with nogil:
            self._chip_select()
            self._send_uint8_1_byte(CMD_RESET)
            # make sure we read not-data-ready first, otherwise we read see
            # data-ready before it actually started the reset and exit early
            self._wait_data_ready_clear()
            # it is reset on ready
            self._wait_data_ready()
            # Release chip select and implement t_11 timeout
            self._chip_release()

    def set_sampling_channel(self, unsigned char channel_flag):
        """Sets the channel the ADC will sample.

        This can be followed by :meth:`read_sample_trigger`,
        :meth:`standby` + :meth:`read_sample_from_standby`, or
        :meth:`read_next_sample` (but only when safe=True).

        To sample from multiple channels, we need to multiplex the signals
        by switching the current channel and then sampling.
        See also :meth:`read_next_sample_switch_channel` or
        :meth:`start_chunked_multiplexed_sampling` for a more efficient
        approach.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        with nogil:
            self._chip_select()
            # Set input pin mux configuration
            self._send_uint8_3_byte(CMD_WREG | REG_MUX, 0x00, channel_flag)
            # Release chip select and implement t_11 timeout
            self._chip_release()

    def read_sample_trigger(self):
        """Restart the ADC conversion cycle.

        Similar to read_next_sample, but instead of waiting to see a new sample
        arrive we trigger a new sample to be generated. Useful if the
        input changed or configuration changed and we want a new sample, but
        e.g. the sampling rate is low and we don't want to wait for a new
        sample.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        cdef unsigned char buffer[3]
        cdef int result = 0

        with nogil:
            self._chip_select()
            # Restart/start the conversion cycle with set input pins
            self._send_uint8_1_byte(CMD_SYNC)
            self._delay_us(self._sync_timeout_us)
            self._send_uint8_1_byte(CMD_WAKEUP)
            self._wait_data_ready()
            # Read data from ADC, which still returns the /previous/ conversion
            # result from before changing inputs
            self._send_uint8_1_byte(CMD_RDATA)
            self._delay_us(self._data_timeout_us)
            # The result is 24 bits big endian two's complement value by default
            self._read_uint8_3_byte(<unsigned char *>buffer)
            # Release chip select and implement t_11 timeout
            self._chip_release()

            result = buffer[2] | buffer[1] << 8 | buffer[0] << 16
            # convert from 24-bit two's complement to 32 bit
            if result & 0x800000:
                # it's negative
                result -= 0x1000000

        return result

    def read_sample_from_standby(self):
        """**Only safe** if it is in standby mode.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        cdef unsigned char buffer[3]
        cdef int result = 0

        with nogil:
            self._chip_select()
            # wake up the chip
            self._send_uint8_1_byte(CMD_WAKEUP)
            # Wait for data to be ready
            self._wait_data_ready()
            # Send the read command
            self._send_uint8_1_byte(CMD_RDATA)
            # Wait through the data pause
            self._delay_us(self._data_timeout_us)
            # The result is 24 bits big endian two's complement value by default
            self._read_uint8_3_byte(<unsigned char *>buffer)
            # Wait through inter-command timeout
            self._delay_us(self._t11_timeout_us)
            # put it back into standby
            self._send_uint8_1_byte(CMD_STANDBY)
            # Release chip select and implement t_11 timeout
            self._chip_release()

            result = buffer[2] | buffer[1] << 8 | buffer[0] << 16
            # convert from 24-bit two's complement to 32 bit
            if result & 0x800000:
                # it's negative
                result -= 0x1000000

        return result

    def read_next_sample(self, int safe=True):
        """Read ADC result from the current channel as soon as possible.

        Returns the signed integer ADC conversion result.

        This waits until we see a new sample arrive to make sure we don't read
        during sampling and get a bad value. So this is slower. Use chunked
        reading for better performance.

        If not safe, we don't wait for the new sample, but start immediately
        if there seems to be a current sample.

        Issue this command to read a single conversion result for a
        previously set /and stable/ input channel configuration.

        For the default, free-running mode of the ADC, this means
        invalid data is returned when not synchronising acquisition
        and input channel configuration changes.

        To avoid this, after changing input channel configuration or
        with an external hardware multiplexer, use the hardware SYNC
        input pin or use the sync() method to restart the
        conversion cycle before calling read_async().

        Because this function does not implicitly restart a running
        acquisition, it is faster that the read_oneshot() method.
        """
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        cdef unsigned char buffer[3]
        cdef int result = 0

        with nogil:
            self._chip_select()
            if safe:
                # wait until we see new data arrive so we don't read data
                # during conversion
                self._wait_data_ready_clear()

            # Wait for data to be ready
            self._wait_data_ready()
            # Send the read command
            self._send_uint8_1_byte(CMD_RDATA)
            # Wait through the data pause
            self._delay_us(self._data_timeout_us)
            # The result is 24 bits big endian two's complement value by default
            self._read_uint8_3_byte(<unsigned char *>buffer)
            # Release chip select and implement t_11 timeout
            self._chip_release()

            result = buffer[2] | buffer[1] << 8 | buffer[0] << 16
            # convert from 24-bit two's complement to 32 bit
            if result & 0x800000:
                # it's negative
                result -= 0x1000000

        return result

    cdef void _cycle_channel_mux(self, unsigned char channel_flag) nogil:
        # Setting mux position for next cycle
        self._send_uint8_3_byte(CMD_WREG | REG_MUX, 0x00, channel_flag)
        # it's not clear that we need a t_11 here
        self._delay_us(self._t11_timeout_us)
        # Restart/start next conversion cycle with new input config
        self._send_uint8_1_byte(CMD_SYNC)
        self._delay_us(self._sync_timeout_us)
        self._send_uint8_1_byte(CMD_WAKEUP)
        self._delay_us(self._t11_timeout_us)

        # Read data from ADC, which still returns the /previous/ conversion
        # result from before changing inputs
        self._send_uint8_1_byte(CMD_RDATA)
        # Wait through the data pause
        self._delay_us(self._data_timeout_us)

    def read_next_sample_switch_channel(
            self, unsigned char channel_flag, int safe=True):
        if self._shared_memory is not None:
            raise TypeError('Cannot update during sampling')

        cdef unsigned char buffer[3]
        cdef int result = 0

        with nogil:
            self._chip_select()
            if safe:
                # wait until we see new data arrive so we don't read data
                # during conversion
                self._wait_data_ready_clear()

            # Wait for data to be ready
            self._wait_data_ready()

            self._cycle_channel_mux(channel_flag)

            # The result is 24 bits big endian two's complement value by default
            self._read_uint8_3_byte(<unsigned char *>buffer)

            # Release chip select and implement t_11 timeout
            self._chip_release()

            result = buffer[2] | buffer[1] << 8 | buffer[0] << 16
            # convert from 24-bit two's complement to 32 bit
            if result & 0x800000:
                # it's negative
                result -= 0x1000000

        return result

    def start_chunked_sampling(self, int chunk_size, *channels_flags):
        """chunk size should support at least a few ms of data, otherwise data
        will probably be missed.

        This optimizes single channels sampling vs multiplexed.

        If 2 * channels_flags * chunk_size * 3 is too large we run out of memory.

        The total rate is as follows.

        +---------------+---------------------+
        | Data rate set | Combined Throughput |
        +===============+=====================+
        | 30,000        | 4374                |
        +---------------+---------------------+
        | 15,000        | 3817                |
        +---------------+---------------------+
        | 7500          | 3043                |
        +---------------+---------------------+
        | 3750          | 2165                |
        +---------------+---------------------+
        | 2000          | 1438                |
        +---------------+---------------------+
        | 1000          | 837                 |
        +---------------+---------------------+
        | 500           | 456                 |
        +---------------+---------------------+
        | 100           | 98                  |
        +---------------+---------------------+
        | 60            | 59                  |
        +---------------+---------------------+
        | 50            | 50                  |
        +---------------+---------------------+
        | 30            | 30                  |
        +---------------+---------------------+
        | 25            | 25                  |
        +---------------+---------------------+
        | 15            | 15                  |
        +---------------+---------------------+
        | 10            | 10                  |
        +---------------+---------------------+
        | 5             | 5                   |
        +---------------+---------------------+
        | 2.5           | 2.5                 |
        +---------------+---------------------+
        """
        cdef unsigned char[::1] buffer_view
        cdef unsigned char *buffer = NULL

        cdef int n_channels = 0
        cdef unsigned char[::1] channels_flags_arr_view
        cdef unsigned char *channels_flags_arr_raw = NULL

        if self._shared_memory is not None:
            raise TypeError('Already sampling. Please stop sampling first')

        n_channels = len(channels_flags)
        if not n_channels:
            raise ValueError('At least one channel flag must be provided')

        channels_flags_arr_view = array.array('B', channels_flags)
        channels_flags_arr_raw = &channels_flags_arr_view[0]

        chunk_size *= n_channels
        # order is big endian, 3 bytes per sample
        mm = mmap.mmap(-1, 2 * chunk_size * 3 + MMAP_HEADER_SIZE)
        try:
            buffer_view = mm
            buffer = &buffer_view[0]
            memset(buffer, 0, MMAP_HEADER_SIZE)

            self._shared_memory = mm
            self._last_chunk = 0
            self._chunk_size = chunk_size

            pid = os.fork()
        except:
            self._shared_memory = None
            mm.close()
            raise

        # nothing to do for parent process
        if pid:
            self._process_id = pid
            return

        with nogil:
            self._sample_chunked_forked(
                chunk_size, buffer, n_channels, channels_flags_arr_raw)
        # release buffer so shared memory can be closed
        buffer_view = None

        # exit child
        os._exit(0)

    def stop_chunked_sampling(self, double timeout):
        cdef double start = 0
        cdef double elapsed = 0
        cdef unsigned char[::1] buffer_view
        cdef unsigned char *buffer = NULL

        if self._shared_memory is None or not self._process_id:
            return

        # if it's already dead, nothing to do
        if self._check_process():
            buffer_view = self._shared_memory
            buffer = &buffer_view[0]

            with nogil:
                # signal to stop
                buffer[MMAP_SHUTDOWN_INDEX] = 1
                start = self._get_time_seconds()

                # wait to stop
                while not buffer[MMAP_SHUTDOWN_DONE_INDEX] and elapsed < timeout:
                    self._delay_us(1_000)
                    elapsed = self._get_time_seconds() - start

        try:
            # we couldn't stop it
            if not buffer[MMAP_SHUTDOWN_DONE_INDEX] and self._check_process():
                os.kill(self._process_id, signal.SIGTERM)
        finally:
            # release buffer so shared memory can be closed
            buffer_view = None

            self._shared_memory.close()
            self._shared_memory = None
            self._process_id = 0

    @contextlib.contextmanager
    def chunked_sampling(
            self, int chunk_size, double read_timeout, double stop_timeout,
            *channels_flags):
        gen = None
        self.start_chunked_sampling(chunk_size, *channels_flags)

        try:
            gen = self._next_sampled_chunk_gen(read_timeout)
            yield gen
        finally:
            try:
                if gen is not None:
                    gen.close()
            finally:
                self.stop_chunked_sampling(stop_timeout)

    def _next_sampled_chunk_gen(self, double timeout):
        cdef unsigned char *buffer = NULL
        cdef unsigned char[::1] buffer_view

        if self._shared_memory is None:
            raise TypeError('Sampling has not started')

        buffer_view = self._shared_memory
        buffer = &buffer_view[0]

        while True:
            yield self._wait_get_next_sampled_chunk(timeout, buffer)

    def wait_get_next_sampled_chunk(self, double timeout):
        """Returns the raw integers.
        """
        cdef unsigned char *buffer = NULL
        cdef unsigned char[::1] buffer_view

        if self._shared_memory is None:
            raise TypeError('Sampling has not started')

        buffer_view = self._shared_memory
        buffer = &buffer_view[0]

        return self._wait_get_next_sampled_chunk(timeout, buffer)

    cdef _wait_get_next_sampled_chunk(self, double timeout, unsigned char *buffer):
        cdef double start = 0
        cdef double elapsed = 0
        cdef unsigned char chunk = 0
        cdef int skipped = 0
        cdef int timed_out = 1
        cdef int offset = 0
        cdef int i = 0
        cdef int value = 0
        cdef long long timestamp_us = 0

        cdef array.array[long] arr = None
        cdef long[::1] arr_view
        cdef long *arr_raw = NULL

        with nogil:
            start = self._get_time_seconds()

            # wait for chunk to be done
            while buffer[MMAP_CHUNK_INDEX] == self._last_chunk and elapsed < timeout:
                self._delay_us(1_000)

                elapsed = self._get_time_seconds() - start

            chunk = buffer[MMAP_CHUNK_INDEX]
            if chunk != self._last_chunk:
                timed_out = 0

                timestamp_us = (<int *>&buffer[MMAP_TIME_SEC_INDEX])[0]
                timestamp_us *= 1_000_000
                timestamp_us += (<int *>&buffer[MMAP_TIME_USEC_INDEX])[0]

                self._last_chunk += 1
                if chunk != self._last_chunk:
                    skipped = 1

                if chunk % 2:
                    # odd, so second chunk
                    offset = self._chunk_size * 3 + MMAP_HEADER_SIZE
                else:
                    offset = MMAP_HEADER_SIZE

                with gil:
                    arr = array.clone(array.array('l'), self._chunk_size, False)
                    arr_view = arr
                    arr_raw = &arr_view[0]

                for i in range(self._chunk_size):
                    # it's stored in big endian
                    value = buffer[offset + 3 * i]
                    value <<= 8
                    value |= buffer[offset + 3 * i + 1]
                    value <<= 8
                    value |= buffer[offset + 3 * i + 2]

                    # convert from 24-bit two's complement to 32 bit
                    if value & 0x800000:
                        # it's negative
                        value -= 0x1000000
                    arr_raw[i] = value

        if timed_out:
            return None, False, timestamp_us
        return arr, bool(skipped), timestamp_us

    cdef void _sample_chunked_forked(
            self, int chunk_size, unsigned char *buffer, int n_channels,
            unsigned char *channels_flags) nogil:
        cdef timeval now
        cdef int count = 0
        cdef int channel_i = 1
        # we start with the second chunk (and skipping 3 signal bytes)
        cdef int offset = chunk_size * 3 + MMAP_HEADER_SIZE
        # we start with second chunk to help client notice it changed to 1 when waiting
        cdef unsigned char chunk = 1
        cdef unsigned char result[3]

        self._chip_select()
        # Setting mux position for next cycle
        self._send_uint8_3_byte(CMD_WREG | REG_MUX, 0x00, channels_flags[0])
        self._delay_us(self._t11_timeout_us)

        # do sync
        self._send_uint8_1_byte(CMD_SYNC)
        self._delay_us(self._sync_timeout_us)
        self._send_uint8_1_byte(CMD_WAKEUP)

        # Wait for data to be ready
        self._wait_data_ready()
        # Send the continuous read command
        self._send_uint8_1_byte(CMD_RDATAC if n_channels == 1 else CMD_RDATA)
        # Wait through the data pause
        self._delay_us(self._data_timeout_us)

        # wait for ready to clear
        self._wait_data_ready_clear()

        while not buffer[MMAP_SHUTDOWN_INDEX]:
            # Wait for data to be ready
            self._wait_data_ready()

            # get the time of the first sample
            if not count:
                gettimeofday(&now, NULL)
                (<int *>&buffer[MMAP_TIME_SEC_INDEX])[0] = now.tv_sec
                (<int *>&buffer[MMAP_TIME_USEC_INDEX])[0] = now.tv_usec

            if n_channels > 1:
                self._cycle_channel_mux(channels_flags[channel_i])
                channel_i += 1
                channel_i = channel_i % n_channels

            # The result is 24 bits big endian two's complement value by default
            self._read_uint8_3_byte(<unsigned char *>result)

            buffer[offset + 3 * count] = result[0]
            buffer[offset + 3 * count + 1] = result[1]
            buffer[offset + 3 * count + 2] = result[2]

            count += 1
            if count == chunk_size:
                # we are done with this chunk, so write the result
                buffer[MMAP_CHUNK_INDEX] = chunk
                chunk += 1

                count = 0
                if chunk % 2:
                    # odd, so second chunk
                    offset = chunk_size * 3 + MMAP_HEADER_SIZE
                else:
                    offset = MMAP_HEADER_SIZE

            # wait for ready to clear
            self._wait_data_ready_clear()

        self._wait_data_ready_clear()
        self._wait_data_ready()
        self._send_uint8_1_byte(CMD_SDATAC)

        # Release chip select and implement t_11 timeout
        self._chip_release()

        # signal that we're done
        buffer[MMAP_SHUTDOWN_DONE_INDEX] = 1

    cdef double _get_time_seconds(self) nogil:
        cdef timeval now
        gettimeofday(&now, NULL)
        return <double>now.tv_sec + <double>now.tv_usec / 1e6

    def debug_measure_sleep_resolution(self, unsigned int us, int count):
        cdef double min_val = 1e6
        cdef double max_val = 0
        cdef double val = 0
        cdef int i = 0

        with nogil:
            for i in range(count):
                val = self._get_time_seconds()
                self._sleep(us)
                val = self._get_time_seconds() - val

                min_val = min(min_val, val)
                max_val = max(max_val, val)

        return min_val, max_val

    cdef int _sleep(self, unsigned int us) nogil:
        """This has a minimum resolution of around 100+ us.
        """
        cdef timespec now, rem
        now.tv_sec = us // 1_000_000
        now.tv_nsec = (us % 1_000_000) * 1000
        return nanosleep(&now, &rem)

    cdef int _delay_us(self, unsigned int us) nogil:
        """If it's less than 100us it's just a busy loop because we don't have
        the resolution.
        
        See also https://yosh.ke.mu/raspberry_pi_getting_time.
        """
        cdef timeval now, delay, end
        cdef timespec t
        cdef timespec rem

        if us >= 100:
            t.tv_sec = us // 1_000_000
            t.tv_nsec = (us % 1_000_000) * 1000
            return nanosleep(&t, &rem)

        gettimeofday(&now, NULL)
        delay.tv_sec  = us // 1000000
        delay.tv_usec = us % 1000000
        timeradd(&now, &delay, &end)

        while (now.tv_usec < end.tv_usec
                if now.tv_sec == end.tv_sec else now.tv_sec < end.tv_sec):
            gettimeofday(&now, NULL)
        return 0

    cdef void _set_pin_direction(
            self, unsigned char pin, unsigned char output) nogil:
        return

    cdef void _write_pin(self, unsigned char pin, unsigned char value) nogil:
        return

    cdef unsigned char _read_pin(self, unsigned char pin) nogil:
        return 0

    cdef object _init(self):
        return

    cdef object _init_spi(self):
        return

    cdef int _write_read_spi(
            self, unsigned char *tx_buf, unsigned char *rx_buf,
            unsigned count) nogil:
        return 0


cdef class WiringPiADS1256(ADS1256):
    """WiringPi backend.
    """

    def __init__(
            self, cs_pin=15, data_ready_pin=11, reset_pin=12, power_down_pin=13,
            **kwargs):
        super().__init__(
            cs_pin=cs_pin, data_ready_pin=data_ready_pin, reset_pin=reset_pin,
            power_down_pin=power_down_pin, **kwargs)

    cdef void _set_pin_direction(
            self, unsigned char pin, unsigned char output) nogil:
        pinMode(pin, output)

    cdef void _write_pin(self, unsigned char pin, unsigned char value) nogil:
        digitalWrite(pin, value)

    cdef unsigned char _read_pin(self, unsigned char pin) nogil:
        return digitalRead(pin)

    cdef object _init(self):
        # Set up the wiringpi object to use physical pin numbers
        wiringPiSetupPhys()

    def release_adc(self):
        if self._shared_memory is not None:
            raise TypeError(
                'Cannot release adc during sampling. Stop sampling first')

    cdef object _init_spi(self):
        # Initialize the wiringpi SPI setup. Return value is the Linux file
        # descriptor for the SPI bus device:
        cdef unsigned char mode = 0
        if self._spi_mode_CPHA:
            mode |= 0x01
        if self._spi_mode_CPOL:
            mode |= 0x02

        fd = wiringPiSPISetupMode(self.spi_channel, self.spi_frequency, mode)
        if fd == -1:
            raise IOError("ERROR: Could not access SPI device file")

    cdef int _write_read_spi(
            self, unsigned char *tx_buf, unsigned char *rx_buf,
            unsigned count) nogil:
        cdef int res = wiringPiSPIDataRW(self.spi_channel, tx_buf, count)
        memcpy(rx_buf, tx_buf, count)
        return res


cdef class PiGPIOADS1256(ADS1256):
    """PiGPIO backend.
    """

    cdef int _spi_handle

    def __init__(
            self, cs_pin=22, data_ready_pin=17, reset_pin=18, power_down_pin=27,
            **kwargs):
        super().__init__(
            cs_pin=cs_pin, data_ready_pin=data_ready_pin, reset_pin=reset_pin,
            power_down_pin=power_down_pin, **kwargs)
        self._spi_handle = -1

    cdef void _set_pin_direction(self, unsigned char pin, unsigned char output) nogil:
        gpioSetMode(pin, output)

    cdef void _write_pin(self, unsigned char pin, unsigned char value) nogil:
        gpioWrite(pin, value)

    cdef unsigned char _read_pin(self, unsigned char pin) nogil:
        return <unsigned char>gpioRead(pin)

    cdef object _init(self):
        # Set up the wiringpi object to use physical pin numbers
        if gpioInitialise() == PI_INIT_FAILED:
            raise ValueError

    def release_adc(self):
        if self._shared_memory is not None:
            raise TypeError(
                'Cannot release adc during sampling. Stop sampling first')

        if self._spi_handle >= 0:
            spiClose(<unsigned>self._spi_handle)
            self._spi_handle = -1
        gpioTerminate()

    cdef object _init_spi(self):
        cdef unsigned char mode = 0
        if self._spi_mode_CPHA:
            mode |= 0x01
        if self._spi_mode_CPOL:
            mode |= 0x02

        self._spi_handle = spiOpen(self.spi_channel, self.spi_frequency // 2, mode)

        if self._spi_handle < 0:
            raise IOError("ERROR: Could not access SPI device file")

    cdef int _write_read_spi(self, unsigned char *tx_buf, unsigned char *rx_buf, unsigned count) nogil:
        return spiXfer(self._spi_handle, <char *>tx_buf, <char *>rx_buf, count)
