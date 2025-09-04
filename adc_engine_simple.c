/*
 * Versão Simplificada do ADC Engine - C Core
 * ==========================================
 * 
 * Versão que funciona com lgpio garantidamente
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
#include <math.h>

// Configurações
#define SAMPLE_RATE 1000
#define BUFFER_SIZE 10000
#define SHM_NAME "/adc_data"
// Header layout: 5 ints/double with padding to 32 bytes then samples
// Vamos definir SHM_SIZE manualmente para evitar discrepâncias de alinhamento entre compiladores.
#define SAMPLE_STRUCT_SIZE 24  // double + double + int (provável padding)
#define HEADER_SIZE 32
#define SHM_SIZE (HEADER_SIZE + (BUFFER_SIZE * SAMPLE_STRUCT_SIZE))

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

// Funções de timing
static inline void precise_sleep_ns(long nanoseconds) {
    struct timespec req = {0, nanoseconds};
    nanosleep(&req, NULL);
}

static inline double get_timestamp() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

// Simular leitura ADC (para teste sem hardware)
double simulate_adc_reading() {
    static double phase = 0.0;
    static int counter = 0;
    
    // Simular sinal senoidal com ruído
    double base_signal = 1.65 + 0.5 * sin(phase);  // 1.65V ± 0.5V
    double noise = ((double)rand() / RAND_MAX - 0.5) * 0.01; // ±5mV ruído
    
    phase += 0.1;
    if (phase > 6.28) phase = 0.0;
    
    // Adicionar alguns picos ocasionais
    if (counter % 500 == 0) {
        base_signal += 0.2;
    }
    counter++;
    
    return base_signal + noise;
}

// Thread de aquisição de alta velocidade
void* acquisition_thread(void* arg) {
    (void)arg; // Evitar warning
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
        
        // Simular leitura ADC
        double voltage = simulate_adc_reading();
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
    // Remover shared memory anterior se existir
    shm_unlink(SHM_NAME);
    
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
    
    if (lgpio_handle >= 0) {
        lgGpiochipClose(lgpio_handle);
    }
    
    printf("✅ Cleanup concluído\n");
}

// Função principal
int main() {
    printf("\n🚀 HIGH-SPEED ADC ENGINE - Versão Simplificada\n");
    printf("===============================================\n");
    printf("Taxa: %d Hz | Buffer: %d amostras\n", SAMPLE_RATE, BUFFER_SIZE);
    printf("Shared Memory: %s (%d bytes)\n", SHM_NAME, SHM_SIZE);
    printf("Header: %d bytes | Sample size: %d bytes | Data region: %d bytes\n", HEADER_SIZE, SAMPLE_STRUCT_SIZE, SHM_SIZE - HEADER_SIZE);
    printf("Mode: SIMULAÇÃO (sinal senoidal + ruído)\n");
    printf("===============================================\n\n");
    
    // Inicializar random
    srand((unsigned int)time(NULL));
    
    // Configurar handlers de sinal
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Inicializar shared memory
    if (init_shared_memory() < 0) {
        cleanup();
        return 1;
    }
    
    printf("✅ Componentes inicializados\n");
    
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
        printf("📊 Taxa: %.1f Hz | Amostras: %d | Buffer: %d/%d | Tensão: %.3fV\n",
               shm_ptr->actual_rate,
               shm_ptr->total_samples,
               (shm_ptr->write_index - shm_ptr->read_index + BUFFER_SIZE) % BUFFER_SIZE,
               BUFFER_SIZE,
               shm_ptr->samples[shm_ptr->write_index > 0 ? shm_ptr->write_index-1 : 0].voltage);
    }
    
    // Aguardar thread terminar
    pthread_join(acq_thread, NULL);
    
    cleanup();
    return 0;
}
