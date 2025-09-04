#!/usr/bin/env python3
"""
Interface Gráfica em Tempo Real para Polissonografia
====================================================

Visualiza dados do ADS1256 em tempo real usando matplotlib.
Perfeito para testar a captação de sinais biomédicos.

Uso:
    python3 real_time_plotter.py

Controles:
    - ESC: Sair
    - Espaço: Pausar/Retomar
    - R: Reset do gráfico
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import time
import sys
import signal

try:
    from pi5_waveshare_adc._adc import LgpioADS1256
except ImportError:
    print("❌ Erro: pi5_waveshare_adc não instalado!")
    print("Execute: sudo pip3 install .")
    sys.exit(1)

class RealTimePlotter:
    def __init__(self, channels=1, sample_rate=100, window_size=1000):
        """
        Parâmetros:
        - channels: Número de canais (1-8)
        - sample_rate: Taxa de amostragem em Hz
        - window_size: Número de amostras no gráfico
        """
        self.channels = min(channels, 8)
        self.sample_rate = sample_rate
        self.window_size = window_size
        self.paused = False
        
        # Inicializar ADC
        print("🔧 Inicializando ADS1256...")
        self.adc = LgpioADS1256()
        self.adc.setup_adc()
        print(f"✅ ADC configurado!")
        print(f"📊 Resolução: {self.adc.v_per_digit*1e9:.1f} nanovolts")
        
        # Buffers de dados
        self.time_data = deque(maxlen=window_size)
        self.voltage_data = [deque(maxlen=window_size) for _ in range(self.channels)]
        
        # Setup do gráfico
        self.setup_plot()
        
        # Configurar tratamento de sinais
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def setup_plot(self):
        """Configurar interface gráfica."""
        plt.style.use('dark_background')
        self.fig, self.axes = plt.subplots(self.channels, 1, figsize=(12, 8))
        
        if self.channels == 1:
            self.axes = [self.axes]
            
        self.lines = []
        colors = ['cyan', 'yellow', 'magenta', 'green', 'red', 'blue', 'orange', 'white']
        
        for i, ax in enumerate(self.axes):
            line, = ax.plot([], [], color=colors[i % len(colors)], linewidth=1.5)
            self.lines.append(line)
            
            ax.set_ylabel(f'Canal {i}\n(Volts)', fontsize=10)
            ax.grid(True, alpha=0.3)
            ax.set_ylim(-0.1, 0.1)  # Ajustar conforme necessário
            
        self.axes[-1].set_xlabel('Tempo (s)', fontsize=12)
        self.fig.suptitle('📈 Monitor em Tempo Real - Polissonografia (RPi 5)', 
                         fontsize=14, fontweight='bold')
        
        # Texto de status
        self.status_text = self.fig.text(0.02, 0.95, '', fontsize=10, color='lime')
        self.info_text = self.fig.text(0.02, 0.02, 
                                      'Controles: ESC=Sair | Espaço=Pausar | R=Reset\n'
                                      'Toque nos pinos para ver variações!', 
                                      fontsize=9, color='gray')
        
        plt.tight_layout()
        
    def read_channels(self):
        """Ler dados de todos os canais."""
        voltages = []
        for ch in range(self.channels):
            if ch > 0:
                # TODO: Implementar troca de canal
                # Por enquanto, todos os canais leem do mesmo (CH0)
                pass
            
            sample = self.adc.read_sample_trigger()
            voltage = sample * self.adc.v_per_digit
            voltages.append(voltage)
            
        return voltages
    
    def update_plot(self, frame):
        """Atualizar gráfico (chamado pelo matplotlib)."""
        if self.paused:
            return self.lines
            
        try:
            current_time = time.time()
            voltages = self.read_channels()
            
            # Adicionar dados aos buffers
            self.time_data.append(current_time)
            for i, voltage in enumerate(voltages):
                self.voltage_data[i].append(voltage)
            
            # Atualizar linhas do gráfico
            if len(self.time_data) > 1:
                time_array = np.array(self.time_data)
                time_rel = time_array - time_array[0]  # Tempo relativo
                
                for i, line in enumerate(self.lines):
                    voltage_array = np.array(self.voltage_data[i])
                    line.set_data(time_rel, voltage_array)
                    
                    # Ajustar limites do eixo Y automaticamente
                    if len(voltage_array) > 10:
                        v_min, v_max = voltage_array.min(), voltage_array.max()
                        v_range = v_max - v_min
                        margin = max(v_range * 0.1, 0.001)  # 10% de margem ou 1mV
                        self.axes[i].set_ylim(v_min - margin, v_max + margin)
                
                # Ajustar limite do eixo X
                for ax in self.axes:
                    ax.set_xlim(time_rel[0], time_rel[-1])
            
            # Atualizar status
            if len(voltages) > 0:
                self.status_text.set_text(
                    f'🟢 Ativo | Taxa: {self.sample_rate}Hz | '
                    f'CH0: {voltages[0]:+.6f}V | '
                    f'Amostras: {len(self.time_data)}'
                )
            
        except Exception as e:
            self.status_text.set_text(f'❌ Erro: {e}')
            
        return self.lines
    
    def on_key_press(self, event):
        """Tratar teclas pressionadas."""
        if event.key == 'escape':
            plt.close('all')
            sys.exit(0)
        elif event.key == ' ':  # Espaço
            self.paused = not self.paused
            status = "⏸️  Pausado" if self.paused else "▶️  Rodando"
            print(f"{status}")
        elif event.key == 'r':  # Reset
            self.time_data.clear()
            for data in self.voltage_data:
                data.clear()
            print("🔄 Gráfico resetado")
    
    def signal_handler(self, sig, frame):
        """Tratar Ctrl+C."""
        print("\n🛑 Encerrando...")
        try:
            self.adc.release_adc()
        except:
            pass
        plt.close('all')
        sys.exit(0)
    
    def run(self):
        """Executar interface gráfica."""
        print("\n" + "="*60)
        print("🚀 INICIANDO MONITOR EM TEMPO REAL")
        print("="*60)
        print(f"📊 Canais: {self.channels}")
        print(f"⚡ Taxa de amostragem: {self.sample_rate} Hz")
        print(f"🪟 Janela: {self.window_size} amostras")
        print(f"🎯 Resolução: {self.adc.v_per_digit*1e9:.1f} nanovolts")
        print("\n💡 DICAS:")
        print("   • Toque nos pinos CH0+/CH0- para ver variações")
        print("   • Aproxime/afaste as mãos da board")
        print("   • Use ESC para sair, Espaço para pausar")
        print("="*60)
        
        # Conectar eventos de teclado
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Iniciar animação
        interval = 1000 / self.sample_rate  # ms entre frames
        self.animation = animation.FuncAnimation(
            self.fig, self.update_plot, interval=interval, blit=False, cache_frame_data=False
        )
        
        try:
            plt.show()
        except KeyboardInterrupt:
            self.signal_handler(None, None)
        finally:
            try:
                self.adc.release_adc()
            except:
                pass

def main():
    """Função principal."""
    print("🏥 Interface Gráfica para Polissonografia - RPi 5")
    print("📡 Baseado em ADS1256 + lgpio")
    
    try:
        # Configurar plotter
        plotter = RealTimePlotter(
            channels=1,        # 1 canal para teste inicial
            sample_rate=50,    # 50 Hz (bom para visualização)
            window_size=500    # 10 segundos de dados (500/50)
        )
        
        # Executar
        plotter.run()
        
    except KeyboardInterrupt:
        print("\n👋 Tchau!")
    except Exception as e:
        print(f"\n❌ Erro: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
