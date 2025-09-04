/*
 * High-Speed ADC Acquisition Engine - C Core
 * ==========================================
 * 
 * Aquisição em C puro para máxima performance
 * Comunicação via shared memory com Python
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <lgpio.h>

// Configurações
#define SAMPLE_RATE 1000
#define BUFFER_SIZE 10000
#define SHM_NAME "/adc_data"
#define SHM_SIZE (BUFFER_SIZE * sizeof(struct sample_data))

// Estrutura de dados compartilhados
struct sample_data {
    double timestamp;
    double voltage;
    int sample_id;
};

struct shared_buffer {
    volatile int write_index;
    volatile int read_index;
    volatile int total_samples;
    volatile double actual_rate;
    volatile int running;
    struct sample_data samples[BUFFER_SIZE];
};

// Variáveis globais
static struct shared_buffer *shm_ptr = NULL;
static int shm_fd = -1;
static volatile int keep_running = 1;
static int lgpio_handle = -1;
static int spi_handle = -1;

// ADS1256 Configurações
#define ADS1256_CS_PIN 22
#define ADS1256_DRDY_PIN 17
#define ADS1256_RST_PIN 18

// ADS1256 Comandos
#define ADS1256_CMD_WAKEUP   0x00
#define ADS1256_CMD_RDATA    0x01
#define ADS1256_CMD_RDATAC   0x03
#define ADS1256_CMD_SDATAC   0x0F
#define ADS1256_CMD_RREG     0x10
#define ADS1256_CMD_WREG     0x50
#define ADS1256_CMD_SELFCAL  0xF0
#define ADS1256_CMD_SELFOCAL 0xF1
#define ADS1256_CMD_SELFGCAL 0xF2
#define ADS1256_CMD_SYSOCAL  0xF3
#define ADS1256_CMD_SYSGCAL  0xF4
#define ADS1256_CMD_SYNC     0xFC
#define ADS1256_CMD_STANDBY  0xFD
#define ADS1256_CMD_RESET    0xFE

// Funções de timing de alta precisão
static inline void precise_sleep_ns(long nanoseconds) {
    struct timespec req = {0, nanoseconds};
    nanosleep(&req, NULL);
}

static inline double get_timestamp() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

// Inicialização do lgpio
int init_lgpio() {
    lgpio_handle = lgGpiochipOpen(0);
    if (lgpio_handle < 0) {
        printf("❌ Erro ao abrir lgpio chip\n");
        return -1;
    }
    
    // Configurar pinos
    lgGpioSetMode(lgpio_handle, ADS1256_CS_PIN, LG_OUTPUT);
    lgGpioSetMode(lgpio_handle, ADS1256_DRDY_PIN, LG_INPUT);
    lgGpioSetMode(lgpio_handle, ADS1256_RST_PIN, LG_OUTPUT);
    
    // CS alto inicialmente
    lgGpioWrite(lgpio_handle, ADS1256_CS_PIN, 1);
    
    // Reset do chip
    lgGpioWrite(lgpio_handle, ADS1256_RST_PIN, 0);
    precise_sleep_ns(100000); // 100µs
    lgGpioWrite(lgpio_handle, ADS1256_RST_PIN, 1);
    precise_sleep_ns(1000000); // 1ms
    
    printf("✅ lgpio inicializado\n");
    return 0;
}

// Inicialização do SPI
int init_spi() {
    spi_handle = lgSpiOpen(0, 0, 1000000, 0); // 1MHz
    if (spi_handle < 0) {
        printf("❌ Erro ao abrir SPI\n");
        return -1;
    }
    printf("✅ SPI inicializado a 1MHz\n");
    return 0;
}

// Função para ler amostra do ADS1256
double read_adc_sample() {
    unsigned char tx_buf[4] = {ADS1256_CMD_RDATA, 0, 0, 0};
    unsigned char rx_buf[4] = {0, 0, 0, 0};
    
    // Aguardar DRDY
    int timeout = 1000;
    while (lgGpioRead(lgpio_handle, ADS1256_DRDY_PIN) && timeout-- > 0) {
        precise_sleep_ns(1000); // 1µs
    }
    
    if (timeout <= 0) {
        return 0.0; // Timeout
    }
    
    // CS baixo
    lgGpioWrite(lgpio_handle, ADS1256_CS_PIN, 0);
    precise_sleep_ns(1000); // 1µs
    
    // Enviar comando e ler dados
    lgSpiXfer(spi_handle, (char*)tx_buf, (char*)rx_buf, 4);
    
    // CS alto
    lgGpioWrite(lgpio_handle, ADS1256_CS_PIN, 1);
    precise_sleep_ns(1000); // 1µs
    
    // Converter para tensão (24-bit signed)
    int32_t raw = ((int32_t)rx_buf[1] << 16) | 
                  ((int32_t)rx_buf[2] << 8) | 
                  rx_buf[3];
    
    // Converter para signed
    if (raw & 0x800000) {
        raw |= 0xFF000000;
    }
    
    // Converter para volts (±2.5V, ganho 1, 24-bit)
    double voltage = (double)raw * (5.0 / 16777216.0); // 2^24
    return voltage;
}

// Thread de aquisição de alta velocidade
void* acquisition_thread(void* arg) {
    printf("🚀 Thread de aquisição iniciada - %d Hz\n", SAMPLE_RATE);
    
    double start_time = get_timestamp();
    double target_interval = 1.0 / SAMPLE_RATE;
    double next_time = get_timestamp();
    
    int local_sample_count = 0;
    
    while (keep_running && shm_ptr->running) {
        double current_time = get_timestamp();
        
        // Controle de timing preciso
        if (current_time < next_time) {
            long sleep_ns = (long)((next_time - current_time) * 1e9);
            if (sleep_ns > 500) { // Só dormir se > 500ns
                precise_sleep_ns(sleep_ns);
            }
        }
        
        // Ler amostra
        double voltage = read_adc_sample();
        double timestamp = get_timestamp();
        
        // Adicionar ao buffer circular
        int write_idx = shm_ptr->write_index;
        shm_ptr->samples[write_idx].timestamp = timestamp;
        shm_ptr->samples[write_idx].voltage = voltage;
        shm_ptr->samples[write_idx].sample_id = local_sample_count++;
        
        // Atualizar índice de escrita (circular)
        shm_ptr->write_index = (write_idx + 1) % BUFFER_SIZE;
        shm_ptr->total_samples++;
        
        // Calcular taxa real a cada 100 amostras
        if (local_sample_count % 100 == 0) {
            double elapsed = timestamp - start_time;
            shm_ptr->actual_rate = local_sample_count / elapsed;
        }
        
        // Próximo tempo
        next_time += target_interval;
        
        // Resetar se muito atrasado
        if (current_time > next_time + 0.005) { // 5ms
            next_time = current_time + target_interval;
        }
    }
    
    printf("🛑 Thread de aquisição finalizada\n");
    return NULL;
}

// Handler de sinais
void signal_handler(int sig) {
    printf("\n🛑 Recebido sinal %d - finalizando...\n", sig);
    keep_running = 0;
    if (shm_ptr) shm_ptr->running = 0;
}

// Inicializar shared memory
int init_shared_memory() {
    // Criar shared memory
    shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd == -1) {
        perror("shm_open");
        return -1;
    }
    
    // Definir tamanho
    if (ftruncate(shm_fd, SHM_SIZE) == -1) {
        perror("ftruncate");
        return -1;
    }
    
    // Mapear memória
    shm_ptr = (struct shared_buffer*)mmap(NULL, SHM_SIZE, 
                                          PROT_READ | PROT_WRITE, 
                                          MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED) {
        perror("mmap");
        return -1;
    }
    
    // Inicializar estrutura
    memset(shm_ptr, 0, SHM_SIZE);
    shm_ptr->running = 1;
    shm_ptr->write_index = 0;
    shm_ptr->read_index = 0;
    shm_ptr->total_samples = 0;
    shm_ptr->actual_rate = 0.0;
    
    printf("✅ Shared memory inicializada (%d bytes)\n", SHM_SIZE);
    return 0;
}

// Cleanup
void cleanup() {
    printf("🧹 Limpando recursos...\n");
    
    if (shm_ptr) {
        shm_ptr->running = 0;
        munmap(shm_ptr, SHM_SIZE);
    }
    
    if (shm_fd >= 0) {
        close(shm_fd);
        shm_unlink(SHM_NAME);
    }
    
    if (spi_handle >= 0) {
        lgSpiClose(spi_handle);
    }
    
    if (lgpio_handle >= 0) {
        lgGpiochipClose(lgpio_handle);
    }
    
    printf("✅ Cleanup concluído\n");
}

// Função principal
int main() {
    printf("\n🚀 HIGH-SPEED ADC ENGINE - C Core\n");
    printf("===================================\n");
    printf("Taxa: %d Hz | Buffer: %d amostras\n", SAMPLE_RATE, BUFFER_SIZE);
    printf("Shared Memory: %s (%d bytes)\n", SHM_NAME, SHM_SIZE);
    printf("===================================\n\n");
    
    // Configurar handlers de sinal
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Inicializar componentes
    if (init_shared_memory() < 0) {
        cleanup();
        return 1;
    }
    
    if (init_lgpio() < 0) {
        cleanup();
        return 1;
    }
    
    if (init_spi() < 0) {
        cleanup();
        return 1;
    }
    
    // Criar thread de aquisição
    pthread_t acq_thread;
    if (pthread_create(&acq_thread, NULL, acquisition_thread, NULL) != 0) {
        printf("❌ Erro ao criar thread de aquisição\n");
        cleanup();
        return 1;
    }
    
    // Loop principal - mostrar estatísticas
    printf("🎯 Aquisição ativa - Ctrl+C para parar\n\n");
    while (keep_running && shm_ptr->running) {
        sleep(2);
        printf("📊 Taxa: %.1f Hz | Amostras: %d | Buffer: %d/%d\n",
               shm_ptr->actual_rate,
               shm_ptr->total_samples,
               (shm_ptr->write_index - shm_ptr->read_index + BUFFER_SIZE) % BUFFER_SIZE,
               BUFFER_SIZE);
    }
    
    // Aguardar thread terminar
    pthread_join(acq_thread, NULL);
    
    cleanup();
    return 0;
}
