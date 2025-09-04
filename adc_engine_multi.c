/*
 * Multi-Channel ADC Engine (ADS1256 / Simulation Fallback)
 * --------------------------------------------------------
 * Layout de memória compartilhada otimizado para vários canais estilo PSG/EEG.
 *
 * HEADER (64 bytes):
 *   int   write_index      (0..BUFFER_SIZE-1) posição mais recente
 *   int   total_cycles     número total de ciclos gravados (wrap livre)
 *   int   channel_count    número de canais ativos
 *   double actual_rate     ciclos por segundo (deve ficar ~SAMPLE_RATE)
 *   int   running          flag de execução (1 = ativo)
 *   int   reserved[10]     padding / futuro (ajusta para 64 bytes)
 *
 * DATA REGION:
 *   double timestamps[BUFFER_SIZE];                         // 8 * BUFFER
 *   float  channels[channel_count][BUFFER_SIZE];            // channel-major
 *
 * Cada "ciclo" corresponde a uma aquisição síncrona (sequencial rápida) de
 * todos os canais configurados. SAMPLE_RATE refere-se à taxa por canal; logo
 * também é a taxa de ciclos.
 *
 * Para facilitar etapas iniciais, este arquivo possui duas implementações:
 *   - SIMULATED (padrão se ADS1256_INIT falhar) gera sinais diferentes por canal
 *   - ADS1256 real via SPI (esboço minimal; pode exigir ajustes finos)
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <errno.h>
#ifdef __has_include
# if __has_include(<lgpio.h>)
#  include <lgpio.h>
#  define HAVE_LGPIO 1
# else
#  define HAVE_LGPIO 0
# endif
#else
# include <lgpio.h>
# define HAVE_LGPIO 1
#endif

#define SHM_NAME "/adc_data"
#define CHANNEL_COUNT 6            // solicitado
#define SAMPLE_RATE 1000           // Hz por canal (ciclos)
#define BUFFER_SIZE 10000          // ciclos (≈ 10 s)

// Header layout constants
struct shm_header {
    volatile int write_index;      // 0
    volatile int total_cycles;     // 4
    volatile int channel_count;    // 8
    volatile double actual_rate;   // 16
    volatile int running;          // 24
    volatile int reserved[9];      // 28..(28+36)=64 exatamente
};

#define HEADER_SIZE 64

// Offsets calculados (após o header)
// timestamps inicia em HEADER_SIZE
// canais iniciam em ts_offset + BUFFER_SIZE * sizeof(double)

static inline size_t timestamps_offset() { return HEADER_SIZE; }
static inline size_t channels_offset() { return HEADER_SIZE + (size_t)BUFFER_SIZE * sizeof(double); }
static inline size_t channel_stride() { return (size_t)BUFFER_SIZE * sizeof(float); }
static inline size_t total_shm_size() {
    return channels_offset() + (size_t)CHANNEL_COUNT * channel_stride();
}

// Globais
static int shm_fd = -1;
static void* shm_base = NULL;
static struct shm_header* hdr = NULL;
static double* ts_array = NULL;
static float* ch_base = NULL; // canais: ch_base + c*BUFFER_SIZE

static volatile int keep_running = 1;

// ADS1256 (esboço) -----------------------------------------------------
// Pines / parâmetros (ajustar conforme seu hardware)
#define PIN_DRDY   17   // GPIO para DRDY (exemplo)
#define PIN_RST    27   // GPIO para RST  (exemplo)
#define PIN_CS     22   // CS manual se necessário (pode usar SPI CS0)
#define SPI_CHANNEL 0
#define SPI_BAUD    1000000  // 1 MHz (ajustável)
static int gpio_chip = -1;
static int spi_handle = -1;
static int hw_available = 0; // 1 se inicializou ADS1256

// Comandos / registradores (principais) ADS1256
#define CMD_WREG   0x50
#define CMD_RREG   0x10
#define CMD_RDATA  0x01
#define CMD_SYNC   0xFC
#define CMD_WAKEUP 0x00
#define CMD_SELFCAL 0xF0
#define CMD_SDATAC 0x0F
#define CMD_RDATAC 0x03

#define REG_STATUS 0x00
#define REG_MUX    0x01
#define REG_ADCON  0x02
#define REG_DRATE  0x03

// Algumas DRATE (exemplo) - depende do datasheet
#define DRATE_30000 0xF0
#define DRATE_15000 0xE0
#define DRATE_7500  0xD0
#define DRATE_3750  0xC0
#define DRATE_2000  0xB0
#define DRATE_1000  0xA1  // aproximação; verificar tabela

static void ads1256_write_register(uint8_t reg, uint8_t value) {
    uint8_t buf[4];
    buf[0] = CMD_WREG | (reg & 0x0F);
    buf[1] = 0; // escrever 1 registro
    buf[2] = value;
    lgSpiXfer(spi_handle, (char*)buf, (char*)buf, 3);
}

static uint8_t ads1256_read_register(uint8_t reg) {
    uint8_t buf[4];
    buf[0] = CMD_RREG | (reg & 0x0F);
    buf[1] = 0; // ler 1
    buf[2] = 0xFF;
    lgSpiXfer(spi_handle, (char*)buf, (char*)buf, 3);
    return buf[2];
}

static int ads1256_wait_drdy(int timeout_us) {
    for (int i = 0; i < timeout_us/100; ++i) {
        int level = lgGpioRead(gpio_chip, PIN_DRDY);
        if (level == 0) return 0; // pronto
    usleep(100);
    }
    return -1; // timeout
}

static int ads1256_set_channel(int ch) {
    // Single-ended: MUX = (AINp = ch, AINn = AINCOM = 0x08)
    uint8_t mux = (ch << 4) | 0x08;
    ads1256_write_register(REG_MUX, mux);
    // SYNC + WAKEUP para nova conversão single-shot
    uint8_t cmd = CMD_SYNC;
    lgSpiWrite(spi_handle, (char*)&cmd, 1);
    cmd = CMD_WAKEUP;
    lgSpiWrite(spi_handle, (char*)&cmd, 1);
    return 0;
}

static int32_t ads1256_read_raw() {
    // Enviar RDATA
    uint8_t cmd = CMD_RDATA;
    lgSpiWrite(spi_handle, (char*)&cmd, 1);
    // Esperar DRDY
    if (ads1256_wait_drdy(10000) != 0) return 0; // timeout -> 0
    // Ler 3 bytes
    uint8_t data[3] = {0,0,0};
    lgSpiXfer(spi_handle, (char*)data, (char*)data, 3);
    int32_t value = ((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | data[2];
    if (value & 0x800000) value |= 0xFF000000; // sign extend 24->32
    return value;
}

static double ads1256_raw_to_voltage(int32_t raw) {
    // Conversão básica assumindo ganho=1 e Vref=2.5V (ajustar conforme placa)
    double vref = 2.5;
    return (raw / 8388607.0) * vref; // 2^23 -1
}

static int ads1256_init() {
#if !HAVE_LGPIO
    return -1; // força modo simulação se não houver lgpio
#endif
    gpio_chip = lgGpiochipOpen(0);
    if (gpio_chip < 0) {
        fprintf(stderr, "lgGpiochipOpen falhou\n");
        return -1;
    }
    lgGpioClaimInput(gpio_chip, 0, PIN_DRDY);
    lgGpioClaimOutput(gpio_chip, 0, PIN_RST, 1);

    spi_handle = lgSpiOpen(0, SPI_CHANNEL, SPI_BAUD, 0); // modo 0
    if (spi_handle < 0) {
        fprintf(stderr, "lgSpiOpen falhou\n");
        return -1;
    }

    // Reset hardware
    lgGpioWrite(gpio_chip, PIN_RST, 0);
    usleep(50*1000);
    lgGpioWrite(gpio_chip, PIN_RST, 1);
    usleep(50*1000);

    // Config DRATE alto para permitir ler 6 canais rapidamente
    ads1256_write_register(REG_DRATE, DRATE_30000);
    // ADCON: ganho=1, clock out off, etc (valor típico 0x00 ou 0x20)
    ads1256_write_register(REG_ADCON, 0x00);

    // Self-calibration
    uint8_t cmd = CMD_SELFCAL;
    lgSpiWrite(spi_handle, (char*)&cmd, 1);
    ads1256_wait_drdy(500000);

    return 0;
}

// Fallback simulation: gera senos defasados + offsets
static double simulate_channel(int ch, double t) {
    double base = 1.65 + 0.3 * sin(t * 2.0 * M_PI * 2.0 + ch * 0.8); // 2 Hz variação
    double noise = ((double)rand()/RAND_MAX - 0.5) * 0.01; // ±5mV
    // Picos ocasionais em canal 0
    if (ch == 0 && ((int)(t*5)) % 17 == 0) base += 0.05;
    return base + noise;
}

static inline double now_ts() {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return ts.tv_sec + ts.tv_nsec/1e9;
}

static inline void precise_sleep(double seconds) {
    if (seconds <= 0) return;
    struct timespec req; req.tv_sec = (time_t)seconds; req.tv_nsec = (long)((seconds - req.tv_sec)*1e9);
    nanosleep(&req, NULL);
}

static void* acquisition_thread(void* arg) {
    (void)arg;
    printf("🚀 Thread multi-channel iniciada (%d canais @ %d Hz)\n", CHANNEL_COUNT, SAMPLE_RATE);
    double start = now_ts();
    double next = start;
    double interval = 1.0 / SAMPLE_RATE; // ciclo
    int local_cycles = 0;

    while (keep_running && hdr->running) {
        double tnow = now_ts();
        if (tnow < next) {
            precise_sleep(next - tnow);
            tnow = now_ts();
        }

        int idx = hdr->write_index;
        ts_array[idx] = tnow;

        for (int c=0; c<CHANNEL_COUNT; ++c) {
            double v;
            if (hw_available) {
                ads1256_set_channel(c);
                if (ads1256_wait_drdy(5000)==0) {
                    int32_t raw = ads1256_read_raw();
                    v = ads1256_raw_to_voltage(raw) + 1.25; // ajustar offset conforme wiring
                } else {
                    v = 0.0; // timeout
                }
            } else {
                v = simulate_channel(c, tnow);
            }
            ch_base[c*BUFFER_SIZE + idx] = (float)v;
        }

        hdr->write_index = (idx + 1) % BUFFER_SIZE;
        hdr->total_cycles++;
        local_cycles++;
        if (local_cycles % 50 == 0) {
            double elapsed = tnow - start;
            if (elapsed > 0.2) hdr->actual_rate = local_cycles / elapsed;
        }

        next += interval;
        if (tnow - next > 0.01) next = tnow + interval; // catch-up
    }
    printf("🛑 Thread encerrada\n");
    return NULL;
}

static void signal_handler(int sig) {
    (void)sig;
    keep_running = 0; if (hdr) hdr->running = 0;
}

static int init_shared_memory() {
    size_t shm_size = total_shm_size();
    shm_unlink(SHM_NAME);
    shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0) { perror("shm_open"); return -1; }
    if (ftruncate(shm_fd, shm_size) != 0) { perror("ftruncate"); return -1; }
    shm_base = mmap(NULL, shm_size, PROT_READ|PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_base == MAP_FAILED) { perror("mmap"); return -1; }
    memset(shm_base, 0, shm_size);
    hdr = (struct shm_header*)shm_base;
    hdr->channel_count = CHANNEL_COUNT;
    hdr->running = 1;
    hdr->write_index = 0;
    hdr->total_cycles = 0;
    ts_array = (double*)((uint8_t*)shm_base + timestamps_offset());
    ch_base = (float*)((uint8_t*)shm_base + channels_offset());
    printf("✅ Shared memory %s criada (%zu bytes)\n", SHM_NAME, shm_size);
    printf("   Header=%d, timestamps=%zu, channels=%zu (stride canal=%zu)\n", HEADER_SIZE,
           (size_t)BUFFER_SIZE*sizeof(double), (size_t)CHANNEL_COUNT*channel_stride(), channel_stride());
    return 0;
}

static void cleanup() {
    if (hdr) hdr->running = 0;
    if (shm_base) munmap(shm_base, total_shm_size());
    if (shm_fd>=0) { close(shm_fd); shm_unlink(SHM_NAME); }
    if (spi_handle>=0) lgSpiClose(spi_handle);
    if (gpio_chip>=0) lgGpiochipClose(gpio_chip);
}

int main() {
    printf("\n🌐 MULTI-CHANNEL ADC ENGINE (6 canais)\n");
    printf("SAMPLE_RATE por canal: %d Hz | BUFFER: %d ciclos (%.1fs)\n", SAMPLE_RATE, BUFFER_SIZE, BUFFER_SIZE/(double)SAMPLE_RATE);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    if (init_shared_memory()!=0) return 1;

    // Tentar init ADS1256
    if (ads1256_init()==0) {
        hw_available = 1;
        printf("🔌 ADS1256 inicializado (modo hardware).\n");
    } else {
        hw_available = 0;
        printf("🧪 Modo SIMULAÇÃO (falha ou indisponível ADS1256).\n");
    }

    pthread_t th;
    if (pthread_create(&th, NULL, acquisition_thread, NULL)!=0) {
        fprintf(stderr, "Erro thread\n");
        cleanup();
        return 1;
    }

    printf("🎯 Aquisição em andamento - Ctrl+C para parar\n");
    while (keep_running && hdr->running) {
        sleep(2);
        int idx = (hdr->write_index - 1 + BUFFER_SIZE) % BUFFER_SIZE;
        printf("📊 Rate: %.1f Hz | Ciclos: %d | Último ch0=%.4fV ch1=%.4fV\n",
               hdr->actual_rate, hdr->total_cycles,
               ch_base[0*BUFFER_SIZE + idx], ch_base[1*BUFFER_SIZE + idx]);
    }

    pthread_join(th, NULL);
    cleanup();
    printf("✅ Encerrado\n");
    return 0;
}
