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
    def __init__(self, display_rate: int = 25, window_seconds: int = 10, auto_start: bool = True):
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
        # Shared memory multi-channel
        self.shm_name = "/adc_data"  # mesmo nome usado pelo novo engine multi
        self.header_size = 64  # conforme adc_engine_multi.c
        self.shm_size = 0
        self.buffer_size = 0      # será detectado
        self.channel_count = 0    # será detectado
        self.timestamps = None
        self.last_index = -1
        # Buffers visuais
        self.time_window = deque(maxlen=self.window_seconds * 1000)
        self.channel_windows = []
        self.header_parsed = False

        # Estatísticas runtime
        self.stats = {
            'total_cycles': 0,
            'actual_rate': 0.0,
            'c_engine_status': 'Desconectado'
        }

        # Handles de shared memory
        self.shm_fd = None
        self.shm_data = None
        self.last_read_index = 0

        # Processo do engine
        self.c_engine_process = None
        self.auto_start = auto_start

        print("🌐 Interface Web Híbrida Inicializada")
        print(f"📺 Taxa de atualização: {self.display_rate} Hz")
        print(f"🪟 Janela: {self.window_seconds} segundos")
    
    def start_c_engine(self):
        """Iniciar o engine C em background."""
        try:
            print("🚀 Iniciando C Engine Multi (adc_engine_multi)...")
            binary_candidates = ['./adc_engine_multi', './adc_engine']
            proc = None
            for bin_path in binary_candidates:
                if os.path.exists(bin_path):
                    try:
                        proc = subprocess.Popen(
                            [bin_path],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                            preexec_fn=os.setsid
                        )
                        self.engine_binary = bin_path
                        break
                    except Exception:
                        continue
            if proc is None:
                print("❌ Nenhum binário de engine encontrado (compile com: make adc_engine_multi)")
                return False
            self.c_engine_process = proc
            
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
    
    def read_new_cycles(self):
        """Lê novos ciclos multi-canal do layout: header(64)+timestamps[Double]+channels[channel][buffer] (float32)."""
        if not self.shm_data:
            return 0
        try:
            self.shm_data.seek(0)
            header = self.shm_data.read(self.header_size)
            if len(header) < 32:
                return 0
            write_index, total_cycles, channel_count = struct.unpack('iii', header[0:12])
            # double está alinhado a 8 bytes -> começa em offset 16 (há 4 bytes de padding entre channel_count e double)
            actual_rate = struct.unpack('d', header[16:24])[0]
            running = struct.unpack('i', header[24:28])[0]
            if not self.header_parsed:
                self.channel_count = channel_count
                # Derivar buffer_size: (tamanho_total - header) = 8*B + C*(4*B)
                body = self.shm_size - self.header_size
                # body = 8B + 4C B => B = body / (8 + 4C)
                if channel_count > 0:
                    self.buffer_size = body // (8 + 4*channel_count)
                else:
                    return 0
                print(f"ℹ️  Detectado channel_count={self.channel_count} buffer_size={self.buffer_size}")
                # Criar deques
                self.channel_windows = [deque(maxlen=self.window_seconds*1000) for _ in range(self.channel_count)]
                self.header_parsed = True
            self.stats['total_cycles'] = total_cycles
            self.stats['actual_rate'] = actual_rate
            if write_index == self.last_index:
                return 0
            # Calcular quantos novos (circular)
            new_count = (write_index - self.last_index) % self.buffer_size if self.last_index >=0 else min(write_index+1, self.buffer_size)
            # Mapear arrays diretamente para leitura rápida
            # timestamps
            ts_offset = self.header_size
            ch_offset = ts_offset + self.buffer_size * 8
            # Ler timestamps novos
            for i in range(new_count):
                idx = ( (self.last_index + 1) + i) % self.buffer_size
                # timestamp
                pos_ts = ts_offset + idx*8
                self.shm_data.seek(pos_ts)
                t_bytes = self.shm_data.read(8)
                if len(t_bytes)<8: break
                (ts_val,) = struct.unpack('d', t_bytes)
                self.time_window.append(ts_val)
                # cada canal
                for c in range(self.channel_count):
                    pos_ch = ch_offset + c * (self.buffer_size*4) + idx*4
                    self.shm_data.seek(pos_ch)
                    v_bytes = self.shm_data.read(4)
                    if len(v_bytes)<4: break
                    (v_float,) = struct.unpack('f', v_bytes)
                    self.channel_windows[c].append(v_float)
            self.last_index = write_index
            return new_count
        except Exception as e:
            print(f"❌ Erro leitura multi-channel: {e}")
            return 0
    
    def update_buffers(self):
        self.read_new_cycles()
    
    def get_plot_data(self):
        if len(self.time_window) < 5 or self.channel_count == 0:
            return [], [], self.stats
        t = np.array(self.time_window)
        t_rel = t - t[0]
        channels = [np.array(ch) for ch in self.channel_windows]
        return t_rel, channels, self.stats
    
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
    time_rel, channels, stats = interface.get_plot_data()
    if len(time_rel) == 0:
        return go.Figure()
    fig = go.Figure()
    offset = 0.0
    spacing = 1.2  # volts de separação visual
    colors = ['#00D9FF','#FF6B35','#F9C74F','#90BE6D','#F94144','#577590','#A23B72','#4D908E']
    for idx,ch in enumerate(channels):
        if len(ch)==0: continue
        base = np.array(ch)
        y = base + offset
        fig.add_trace(go.Scattergl(
            x=time_rel,
            y=y,
            mode='lines',
            name=f'CH{idx}',
            line=dict(width=1,color=colors[idx % len(colors)]),
            customdata=base,
            hovertemplate='Canal %d'<f'{idx}'+'<br>t=%{x:.3f}s<br>V=%{customdata:.4f}V<extra></extra>'
        ))
        offset += spacing
    fig.update_layout(
        title=f'🧠 Multicanal {interface.channel_count}ch @ {stats["actual_rate"]:.0f} Hz',
        xaxis_title='Tempo (s)',
        yaxis_title='Canais (offset)',
        plot_bgcolor='#0F0F0F',
        paper_bgcolor='#1A1A1A',
        font=dict(color='white', size=12),
        xaxis=dict(gridcolor='#333'),
        yaxis=dict(gridcolor='#333'),
        showlegend=True,
        margin=dict(l=50, r=40, t=50, b=40)
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
     html.P(f"📈 Total Ciclos: {stats['total_cycles']:,}", style={'margin': '3px'}),
     html.P(f"🧪 Canais: {interface.channel_count}", style={'margin': '3px'}),
     html.P(f"🪟 Janela Python: {len(interface.time_window)} amostras", style={'margin': '3px'}),
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
    html.P(f"🧠 Canais: {interface.channel_count}", style={'margin': '3px'})
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
