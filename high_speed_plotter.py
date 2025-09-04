#!/usr/bin/env python3
"""
Interface Gráfica Otimizada - 1000 Hz Real
==========================================

Coleta dados a 1000 Hz real em thread separada
e exibe no gráfico a 50 Hz para visualização suave.

Ideal para polissonografia profissional.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time
import sys
import signal
import threading
import queue

try:
    from pi5_waveshare_adc._adc import LgpioADS1256
except ImportError:
    print("❌ Erro: pi5_waveshare_adc não instalado!")
    sys.exit(1)

class HighSpeedPlotter:
    def __init__(self, sample_rate=1000, display_rate=50, window_seconds=10):
        """
        Parâmetros:
        - sample_rate: Taxa de coleta real (Hz)
        - display_rate: Taxa de atualização do gráfico (Hz) 
        - window_seconds: Janela de tempo no gráfico (segundos)
        """
        self.sample_rate = sample_rate
        self.display_rate = display_rate
        self.window_size = sample_rate * window_seconds
        self.paused = False
        self.running = True
        
        # Buffers thread-safe
        self.data_queue = queue.Queue(maxsize=sample_rate*2)  # Buffer 2 segundos
        self.time_buffer = deque(maxlen=self.window_size)
        self.voltage_buffer = deque(maxlen=self.window_size)
        
        # Estatísticas
        self.total_samples = 0
        self.start_time = None
        
        # Inicializar ADC
        print("🔧 Inicializando ADS1256...")
        self.adc = LgpioADS1256()
        self.adc.setup_adc()
        print(f"✅ ADC configurado!")
        print(f"📊 Resolução: {self.adc.v_per_digit*1e9:.1f} nanovolts")
        
        # Setup do gráfico
        self.setup_plot()
        
        # Thread de coleta
        self.collector_thread = threading.Thread(target=self.data_collector, daemon=True)
        
        # Configurar tratamento de sinais
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def setup_plot(self):
        """Configurar interface gráfica."""
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(14, 8))
        
        self.line, = self.ax.plot([], [], color='cyan', linewidth=1, alpha=0.8)
        
        self.ax.set_ylabel('Tensão (V)', fontsize=12)
        self.ax.set_xlabel('Tempo (s)', fontsize=12)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_ylim(-0.1, 0.1)
        
        self.fig.suptitle('📈 Monitor Polissonografia - 1000 Hz Real (RPi 5)', 
                         fontsize=16, fontweight='bold')
        
        # Textos de status
        self.status_text = self.fig.text(0.02, 0.95, '', fontsize=11, color='lime')
        self.stats_text = self.fig.text(0.02, 0.88, '', fontsize=10, color='yellow')
        self.info_text = self.fig.text(0.02, 0.02, 
                                      '⌨️  ESC=Sair | Espaço=Pausar | R=Reset\n'
                                      '👆 Toque nos pinos CH0+/CH0- para testar', 
                                      fontsize=10, color='gray')
        
        plt.tight_layout()
        
    def data_collector(self):
        """Thread de coleta de dados a alta velocidade."""
        print(f"🚀 Iniciando coleta a {self.sample_rate} Hz...")
        self.start_time = time.time()
        
        target_interval = 1.0 / self.sample_rate
        next_time = time.time()
        
        while self.running:
            try:
                # Controlar timing preciso
                current_time = time.time()
                if current_time < next_time:
                    time.sleep(next_time - current_time)
                
                # Coletar amostra
                sample = self.adc.read_sample_trigger()
                voltage = sample * self.adc.v_per_digit
                timestamp = time.time()
                
                # Adicionar ao queue (não bloquear se cheio)
                try:
                    self.data_queue.put_nowait((timestamp, voltage))
                    self.total_samples += 1
                except queue.Full:
                    # Remove item mais antigo se queue estiver cheio
                    try:
                        self.data_queue.get_nowait()
                        self.data_queue.put_nowait((timestamp, voltage))
                    except queue.Empty:
                        pass
                
                # Calcular próximo tempo
                next_time += target_interval
                
                # Resetar se muito atrasado
                if current_time > next_time + 0.01:  # 10ms de atraso máximo
                    next_time = current_time + target_interval
                    
            except Exception as e:
                print(f"❌ Erro na coleta: {e}")
                time.sleep(0.001)
    
    def update_plot(self, frame):
        """Atualizar gráfico com dados coletados."""
        if self.paused:
            return [self.line]
            
        # Processar dados do queue
        new_data_count = 0
        while not self.data_queue.empty() and new_data_count < 100:  # Máximo 100 por frame
            try:
                timestamp, voltage = self.data_queue.get_nowait()
                self.time_buffer.append(timestamp)
                self.voltage_buffer.append(voltage)
                new_data_count += 1
            except queue.Empty:
                break
        
        # Atualizar gráfico se há dados suficientes
        if len(self.time_buffer) > 10:
            time_array = np.array(self.time_buffer)
            voltage_array = np.array(self.voltage_buffer)
            
            # Converter para tempo relativo
            time_rel = time_array - time_array[0]
            
            # Atualizar linha
            self.line.set_data(time_rel, voltage_array)
            
            # Ajustar limites automaticamente
            v_min, v_max = voltage_array.min(), voltage_array.max()
            v_range = v_max - v_min
            margin = max(v_range * 0.1, 0.001)
            self.ax.set_ylim(v_min - margin, v_max + margin)
            self.ax.set_xlim(time_rel[0], time_rel[-1])
            
            # Calcular estatísticas
            if self.start_time:
                elapsed = time.time() - self.start_time
                actual_rate = self.total_samples / elapsed if elapsed > 0 else 0
                
                # Atualizar textos de status
                self.status_text.set_text(
                    f'🟢 Coletando | Queue: {self.data_queue.qsize()} | '
                    f'Buffer: {len(self.voltage_buffer)} amostras'
                )
                
                self.stats_text.set_text(
                    f'📊 Taxa Real: {actual_rate:.1f} Hz | '
                    f'Amostras Total: {self.total_samples:,} | '
                    f'Tensão: {voltage_array[-1]:+.6f}V'
                )
        
        return [self.line]
    
    def on_key_press(self, event):
        """Tratar teclas pressionadas."""
        if event.key == 'escape':
            self.shutdown()
        elif event.key == ' ':  # Espaço
            self.paused = not self.paused
            status = "⏸️  Pausado" if self.paused else "▶️  Rodando"
            print(f"{status}")
        elif event.key == 'r':  # Reset
            self.time_buffer.clear()
            self.voltage_buffer.clear()
            self.total_samples = 0
            self.start_time = time.time()
            print("🔄 Gráfico resetado")
    
    def shutdown(self):
        """Encerrar aplicação."""
        print("\n🛑 Encerrando...")
        self.running = False
        try:
            self.adc.release_adc()
        except:
            pass
        plt.close('all')
        sys.exit(0)
    
    def signal_handler(self, sig, frame):
        """Tratar Ctrl+C."""
        self.shutdown()
    
    def run(self):
        """Executar interface gráfica."""
        print("\n" + "="*70)
        print("🚀 MONITOR POLISSONOGRAFIA - ALTA PERFORMANCE")
        print("="*70)
        print(f"⚡ Taxa de coleta: {self.sample_rate} Hz (real)")
        print(f"📺 Taxa de exibição: {self.display_rate} Hz (suave)")
        print(f"🪟 Janela: {self.window_size:,} amostras")
        print(f"🎯 Resolução: {self.adc.v_per_digit*1e9:.1f} nanovolts")
        print(f"💾 Buffer: {self.data_queue.maxsize} amostras")
        print("\n💡 PERFEITO PARA:")
        print("   • EEG: 10-100 µV")
        print("   • EOG: 50-500 µV") 
        print("   • EMG: 50-2000 µV")
        print("="*70)
        
        # Conectar eventos
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Iniciar thread de coleta
        self.collector_thread.start()
        
        # Iniciar animação do gráfico
        interval = 1000 / self.display_rate
        self.animation = animation.FuncAnimation(
            self.fig, self.update_plot, interval=interval, 
            blit=False, cache_frame_data=False
        )
        
        try:
            plt.show()
        except KeyboardInterrupt:
            self.shutdown()
        finally:
            self.running = False
            try:
                self.adc.release_adc()
            except:
                pass

def main():
    """Função principal."""
    print("🏥 Monitor Polissonografia Profissional - RPi 5")
    print("📡 ADS1256 + lgpio - Alta Performance")
    
    try:
        plotter = HighSpeedPlotter(
            sample_rate=1000,    # 1000 Hz real - polissonografia
            display_rate=50,     # 50 Hz suave para visualização  
            window_seconds=10    # 10 segundos na tela
        )
        
        plotter.run()
        
    except KeyboardInterrupt:
        print("\n👋 Tchau!")
    except Exception as e:
        print(f"\n❌ Erro: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
