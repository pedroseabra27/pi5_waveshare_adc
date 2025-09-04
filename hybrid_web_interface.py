#!/usr/bin/env python3
"""
Interface Web Híbrida - C Core + Python UI
==========================================

Interface Python que lê dados do engine C via shared memory.
Performance máxima: C para aquisição, Python para visualização.
"""

import dash
from dash import dcc, html, Input, Output, callback
import plotly.graph_objs as go
import numpy as np
import time
import mmap
import struct
import os
import signal
import sys
from collections import deque
import subprocess
import threading

class HighSpeedWebInterface:
    def __init__(self, display_rate: int = 25, window_seconds: int = 8, auto_start: bool = True):
        """Inicializa a interface.

        Args:
            display_rate: taxa de atualização do gráfico (Hz).
            window_seconds: janela temporal mostrada (s).
            auto_start: se True tenta iniciar o engine; se False apenas anexa.
        """
        # Parâmetros básicos
        self.display_rate = display_rate
        self.window_seconds = window_seconds
        self.running = True

        # Shared memory (iremos detectar layout depois; mapeamos 1MB para segurança)
        self.shm_name = "/adc_data"
        self.buffer_size = 10000
        self.sample_size = 24  # double + double + int (8+8+4) + possível padding
        self.shm_size = 1024 * 1024  # 1MB
        self.header_parsed = False
        self.header_size = 0

        # Buffers de visualização
        self.time_buffer = deque(maxlen=self.window_seconds * 1000)
        self.voltage_buffer = deque(maxlen=self.window_seconds * 1000)

        # Estatísticas runtime
        self.stats = {
            'total_samples': 0,
            'actual_rate': 0.0,
            'current_voltage': 0.0,
            'voltage_min': 0.0,
            'voltage_max': 0.0,
            'voltage_std': 0.0,
            'c_engine_status': 'Desconectado'
        }

        # Handles de shared memory
        self.shm_fd = None
        self.shm_data = None
        self.last_read_index = 0

        # Processo do engine (se iniciado pela UI)
        self.c_engine_process = None
        self.auto_start = auto_start

        print("🌐 Interface Web Híbrida Inicializada")
        print(f"📺 Taxa de atualização: {self.display_rate} Hz")
        print(f"🪟 Janela: {self.window_seconds} segundos")
    
    def start_c_engine(self):
        """Iniciar o engine C em background."""
        try:
            print("🚀 Iniciando C Engine...")
            self.c_engine_process = subprocess.Popen(
                ['./adc_engine'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Para poder matar o grupo
            )
            
            # Aguardar um pouco para o engine inicializar
            time.sleep(2)
            
            if self.c_engine_process.poll() is None:
                print("✅ C Engine iniciado com sucesso")
                self.stats['c_engine_status'] = 'Conectado'
                return True
            else:
                print("❌ C Engine falhou ao iniciar")
                return False
                
        except Exception as e:
            print(f"❌ Erro ao iniciar C Engine: {e}")
            return False
    
    def connect_shared_memory(self, retries=10, delay=0.5):
        """Conectar ao shared memory do C engine com tentativas."""
        for attempt in range(1, retries+1):
            try:
                # Caminhos possíveis: /adc_data (falha) ou /dev/shm/adc_data (correto para shm_open)
                shm_path_candidates = [self.shm_name, f"/dev/shm{self.shm_name}"]
                last_err = None
                self.shm_fd = None
                for cand in shm_path_candidates:
                    try:
                        self.shm_fd = os.open(cand, os.O_RDONLY)
                        self._resolved_shm_path = cand
                        break
                    except OSError as e:
                        last_err = e
                if self.shm_fd is None:
                    raise last_err if last_err else FileNotFoundError(self.shm_name)
                # Descobrir tamanho real do segmento (antes estava fixo em 1MB e dava erro)
                real_size = os.fstat(self.shm_fd).st_size
                if real_size <= 0:
                    raise RuntimeError(f"Tamanho inválido do shared memory: {real_size}")
                self.shm_size = real_size
                # Mapear exatamente o tamanho existente
                self.shm_data = mmap.mmap(
                    self.shm_fd, self.shm_size, mmap.MAP_SHARED, mmap.PROT_READ
                )
                print(f"ℹ️  Shared memory aberto: {self._resolved_shm_path} ({self.shm_size} bytes)")
                print(f"✅ Conectado ao shared memory (tentativa {attempt})")
                self.stats['c_engine_status'] = 'Conectado'
                return True
            except FileNotFoundError:
                print(f"⏳ Shared memory não encontrado (tentativa {attempt}/{retries})")
                time.sleep(delay)
            except Exception as e:
                print(f"❌ Erro ao conectar shared memory: {e}")
                time.sleep(delay)
        return False
    
    def read_samples_from_shm(self):
        """Ler novas amostras do shared memory."""
        if not self.shm_data:
            return []
        
        try:
            # Header layout (C struct shared_buffer):
            # int write_index; int read_index; int total_samples; double actual_rate; int running; (possible padding)
            self.shm_data.seek(0)
            raw_header = self.shm_data.read(32)  # read enough for header + padding
            if len(raw_header) < 28:  # minimal expected
                return []
            # Try unpack without padding first (28 bytes -> align to 8 = 32)
            try:
                write_index, read_index, total_samples = struct.unpack('iii', raw_header[0:12])
                actual_rate = struct.unpack('d', raw_header[16:24]) if (len(raw_header) >= 24) else (0.0,)
                actual_rate = actual_rate[0]
                running = struct.unpack('i', raw_header[24:28])[0]
                if not self.header_parsed:
                    # Header ends at 28, align to 8 for start of samples => 32
                    self.header_size = 32
                    # Derivar buffer_size dinamicamente pelo tamanho real
                    possible_samples = (self.shm_size - self.header_size) // self.sample_size
                    if possible_samples > 0:
                        if possible_samples != self.buffer_size:
                            print(f"ℹ️  Ajustando buffer_size de {self.buffer_size} para {possible_samples}")
                        self.buffer_size = possible_samples
                    self.header_parsed = True
            except Exception:
                return []
            
            # Atualizar estatísticas
            self.stats['total_samples'] = total_samples
            self.stats['actual_rate'] = actual_rate
            
            # Calcular quantas amostras novas temos
            if write_index != self.last_read_index:
                new_samples = []
                
                # Ler amostras novas (circular buffer)
                current_idx = self.last_read_index
                while current_idx != write_index:
                    # Calcular offset no buffer
                    offset = self.header_size + (current_idx * self.sample_size)
                    self.shm_data.seek(offset)
                    
                    # Ler uma amostra (timestamp, voltage, sample_id)
                    sample_blob = self.shm_data.read(self.sample_size)
                    # Precisamos só dos primeiros 20 bytes (double+double+int); últimos 4 são padding
                    if len(sample_blob) < 20:
                        break
                    try:
                        sample_data = struct.unpack('ddi', sample_blob[:20])
                    except struct.error as e:
                        # Falha de leitura isolada, aborta loop para próxima iteração
                        break
                    timestamp, voltage, sample_id = sample_data
                    
                    new_samples.append((timestamp, voltage))
                    
                    # Próximo índice (circular)
                    current_idx = (current_idx + 1) % self.buffer_size
                
                self.last_read_index = write_index
                return new_samples
            
            return []
            
        except Exception as e:
            print(f"❌ Erro ao ler shared memory: {e}")
            return []
    
    def update_buffers(self):
        """Atualizar buffers com dados do C engine."""
        new_samples = self.read_samples_from_shm()
        
        if new_samples:
            # Adicionar novas amostras aos buffers
            for timestamp, voltage in new_samples:
                self.time_buffer.append(timestamp)
                self.voltage_buffer.append(voltage)
                self.stats['current_voltage'] = voltage
            
            # Calcular estatísticas se temos dados suficientes
            if len(self.voltage_buffer) > 10:
                voltages = np.array(list(self.voltage_buffer)[-1000:])  # Últimas 1000
                self.stats['voltage_min'] = voltages.min()
                self.stats['voltage_max'] = voltages.max()
                self.stats['voltage_std'] = voltages.std()
    
    def get_plot_data(self):
        """Obter dados para o gráfico."""
        if len(self.time_buffer) < 10:
            return [], [], self.stats
        
        # Converter para arrays numpy
        time_array = np.array(list(self.time_buffer))
        voltage_array = np.array(list(self.voltage_buffer))
        
        # Tempo relativo em segundos
        if len(time_array) > 0:
            time_rel = time_array - time_array[0]
            return time_rel.tolist(), voltage_array.tolist(), self.stats
        
        return [], [], self.stats
    
    def cleanup(self):
        """Limpar recursos."""
        print("🧹 Limpando interface...")
        self.running = False
        
        if hasattr(self, '_cleaned') and self._cleaned:
            return
        
        try:
            if self.shm_data:
                self.shm_data.close()
        except Exception:
            pass
        
        try:
            if self.shm_fd:
                os.close(self.shm_fd)
        except Exception:
            pass
        
        if self.c_engine_process:
            print("🛑 Finalizando C Engine...")
            try:
                os.killpg(os.getpgid(self.c_engine_process.pid), signal.SIGTERM)
                self.c_engine_process.wait(timeout=5)
            except:
                pass
        
        print("✅ Cleanup concluído")
        self._cleaned = True

# Instância global
interface = HighSpeedWebInterface()

# Configurar Dash app
app = dash.Dash(__name__)
app.title = "Monitor Híbrido C+Python - RPi 5"

# Layout da interface
app.layout = html.Div([
    html.Div([
        html.H1("⚡ Monitor Híbrido C+Python", 
                style={'textAlign': 'center', 'color': '#FF6B35', 'marginBottom': '10px'}),
        html.H3("C Engine + Python Interface = Máxima Performance", 
                style={'textAlign': 'center', 'color': '#004E89', 'marginTop': '0px'})
    ]),
    
    html.Div([
        html.Div([
            html.H4("📊 Performance em Tempo Real", style={'color': '#F18F01'}),
            html.Div(id='stats-display', style={'fontSize': '14px', 'fontFamily': 'monospace'})
        ], style={'width': '50%', 'display': 'inline-block', 'verticalAlign': 'top', 'padding': '20px'}),
        
        html.Div([
            html.H4("🔧 Status do Sistema", style={'color': '#A23B72'}),
            html.Div(id='system-status', style={'fontSize': '14px', 'fontFamily': 'monospace'})
        ], style={'width': '50%', 'display': 'inline-block', 'verticalAlign': 'top', 'padding': '20px'})
    ]),
    
    dcc.Graph(id='live-graph', style={'height': '70vh'}),
    
    dcc.Interval(
        id='graph-update',
        interval=1000/interface.display_rate,  # ms
        n_intervals=0
    ),
    
    html.Footer([
        html.P("🚀 Engine C: Aquisição 1000 Hz | 🐍 Python: Interface 25 Hz | 💾 Shared Memory: Zero-copy",
               style={'textAlign': 'center', 'color': 'gray', 'marginTop': '20px', 'fontSize': '12px'})
    ])
])

# Callback para atualizar gráfico
@app.callback(
    Output('live-graph', 'figure'),
    Input('graph-update', 'n_intervals')
)
def update_graph(n):
    interface.update_buffers()
    time_data, voltage_data, stats = interface.get_plot_data()
    
    if not time_data:
        return go.Figure()
    
    # Criar gráfico otimizado
    fig = go.Figure()
    
    fig.add_trace(go.Scattergl(  # Scattergl para melhor performance
        x=time_data,
        y=voltage_data,
        mode='lines',
        name='ADC Signal',
        line=dict(color='#00D9FF', width=1),
        hovertemplate='Tempo: %{x:.3f}s<br>Tensão: %{y:.6f}V<extra></extra>'
    ))
    
    fig.update_layout(
        title=f'🚀 Sinal em Tempo Real - {stats["actual_rate"]:.0f} Hz (C Engine)',
        xaxis_title='Tempo (segundos)',
        yaxis_title='Tensão (Volts)',
        plot_bgcolor='#0F0F0F',
        paper_bgcolor='#1A1A1A',
        font=dict(color='white', size=12),
        xaxis=dict(gridcolor='#333', gridwidth=1),
        yaxis=dict(gridcolor='#333', gridwidth=1),
        hovermode='x unified',
        margin=dict(l=50, r=50, t=60, b=50)
    )
    
    return fig

# Callback para estatísticas
@app.callback(
    Output('stats-display', 'children'),
    Input('graph-update', 'n_intervals')
)
def update_stats(n):
    stats = interface.stats
    
    return [
        html.P(f"🚀 Taxa C Engine: {stats['actual_rate']:.1f} Hz", 
               style={'margin': '3px', 'color': '#00FF00' if stats['actual_rate'] > 900 else '#FFAA00'}),
        html.P(f"📈 Total Amostras: {stats['total_samples']:,}", style={'margin': '3px'}),
        html.P(f"⚡ Tensão Atual: {stats['current_voltage']:+.6f}V", style={'margin': '3px'}),
        html.P(f"📊 Buffer Python: {len(interface.voltage_buffer)} amostras", style={'margin': '3px'}),
        html.Hr(style={'margin': '8px 0'}),
        html.P(f"📏 Min: {stats['voltage_min']:+.6f}V", style={'margin': '3px'}),
        html.P(f"📏 Max: {stats['voltage_max']:+.6f}V", style={'margin': '3px'}),
        html.P(f"📊 Desvio: {stats['voltage_std']:.6f}V", style={'margin': '3px'})
    ]

# Callback para status do sistema
@app.callback(
    Output('system-status', 'children'),
    Input('graph-update', 'n_intervals')
)
def update_system_status(n):
    stats = interface.stats
    
    return [
        html.P(f"🔧 C Engine: {stats['c_engine_status']}", 
               style={'margin': '3px', 'color': '#00FF00' if stats['c_engine_status'] == 'Conectado' else '#FF0000'}),
        html.P(f"💾 Shared Memory: {interface.shm_name}", style={'margin': '3px'}),
        html.P(f"📺 Interface: {interface.display_rate} Hz", style={'margin': '3px'}),
        html.P(f"🪟 Janela: {interface.window_seconds}s", style={'margin': '3px'}),
        html.P(f"🎯 Buffer Size: {interface.buffer_size:,}", style={'margin': '3px'}),
        html.P(f"📊 Resolução: 298 nanovolts", style={'margin': '3px'})
    ]

def main():
    """Função principal."""
    print("\n" + "="*70)
    print("⚡ INTERFACE WEB HÍBRIDA - C ENGINE + PYTHON")
    print("="*70)
    print("🚀 Iniciando componentes...")
    
    # Configurar cleanup
    def signal_handler(sig, frame):
        print(f"\n🛑 Recebido sinal {sig}")
        interface.cleanup()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # 1. Se já existe engine rodando, só anexar
        if not interface.auto_start:
            print("🔍 Modo anexar: tentando conectar a engine existente...")
            if not interface.connect_shared_memory():
                print("❌ Não foi possível conectar. Inicie ./adc_engine primeiro.")
                return
        else:
            # Tentar iniciar engine própria
            if not interface.start_c_engine():
                print("⚠️ Não iniciou. Tentando anexar a uma existente...")
                if not interface.connect_shared_memory():
                    print("❌ Falha ao iniciar ou anexar.")
                    return
            else:
                # Conectar após iniciar
                if not interface.connect_shared_memory():
                    print("❌ Falha ao conectar shared memory após start.")
                    return
        
        print("\n🌐 ACESSO WEB:")
        print("   • Local: http://localhost:8050")
        print("   • Rede: http://192.168.150.65:8050")
        print("="*70)
        
        # 3. Executar interface web
        app.run(
            host='0.0.0.0',
            port=8050,
            debug=False
        )
        
    except Exception as e:
        print(f"❌ Erro: {e}")
    finally:
        interface.cleanup()

if __name__ == "__main__":
    # Se usuário exportar ATTACH=1, não iniciar engine
    attach = os.environ.get("ATTACH", "0") == "1"
    interface.auto_start = not attach
    main()
