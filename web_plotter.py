#!/usr/bin/env python3
"""
Interface Web em Tempo Real - 1000 Hz
=====================================

Monitor polissonografia via browser web.
Mantém 1000 Hz de coleta real com visualização web responsiva.

Acesso: http://IP_DA_RASPBERRY:8050
"""

import dash
from dash import dcc, html, Input, Output, callback
import plotly.graph_objs as go
import numpy as np
import time
import threading
import queue
from collections import deque
import signal
import sys

try:
    from pi5_waveshare_adc._adc import LgpioADS1256
except ImportError:
    print("❌ Erro: pi5_waveshare_adc não instalado!")
    sys.exit(1)

class WebPolysommographyMonitor:
    def __init__(self, sample_rate=1000, display_rate=20, window_seconds=10):
        """
        Parâmetros:
        - sample_rate: Taxa de coleta real (Hz)
        - display_rate: Taxa de atualização web (Hz)
        - window_seconds: Janela de tempo no gráfico
        """
        self.sample_rate = sample_rate
        self.display_rate = display_rate
        self.window_size = sample_rate * window_seconds
        self.running = True
        
        # Buffers thread-safe
        self.data_queue = queue.Queue(maxsize=sample_rate*3)
        self.time_buffer = deque(maxlen=self.window_size)
        self.voltage_buffer = deque(maxlen=self.window_size)
        
        # Estatísticas
        self.total_samples = 0
        self.start_time = None
        self.current_voltage = 0.0
        self.actual_rate = 0.0
        
        # Inicializar ADC
        print("🔧 Inicializando ADS1256...")
        self.adc = LgpioADS1256()
        self.adc.setup_adc()
        print(f"✅ ADC configurado!")
        print(f"📊 Resolução: {self.adc.v_per_digit*1e9:.1f} nanovolts")
        
        # Thread de coleta
        self.collector_thread = threading.Thread(target=self.data_collector, daemon=True)
        self.collector_thread.start()
        
        # Configurar tratamento de sinais
        signal.signal(signal.SIGINT, self.signal_handler)
    
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
                
                self.current_voltage = voltage
                
                # Adicionar ao queue
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
                
                # Calcular taxa real
                if self.start_time:
                    elapsed = time.time() - self.start_time
                    self.actual_rate = self.total_samples / elapsed if elapsed > 0 else 0
                
                # Calcular próximo tempo
                next_time += target_interval
                
                # Resetar se muito atrasado
                if current_time > next_time + 0.01:
                    next_time = current_time + target_interval
                    
            except Exception as e:
                print(f"❌ Erro na coleta: {e}")
                time.sleep(0.001)
    
    def get_latest_data(self):
        """Obter dados mais recentes para o gráfico."""
        # Processar dados do queue
        new_data_count = 0
        while not self.data_queue.empty() and new_data_count < 200:
            try:
                timestamp, voltage = self.data_queue.get_nowait()
                self.time_buffer.append(timestamp)
                self.voltage_buffer.append(voltage)
                new_data_count += 1
            except queue.Empty:
                break
        
        if len(self.time_buffer) < 10:
            return [], [], {}
        
        # Converter para arrays numpy
        time_array = np.array(self.time_buffer)
        voltage_array = np.array(self.voltage_buffer)
        
        # Tempo relativo em segundos
        time_rel = time_array - time_array[0]
        
        # Estatísticas
        stats = {
            'total_samples': self.total_samples,
            'actual_rate': self.actual_rate,
            'current_voltage': self.current_voltage,
            'queue_size': self.data_queue.qsize(),
            'buffer_size': len(self.voltage_buffer),
            'voltage_min': voltage_array.min(),
            'voltage_max': voltage_array.max(),
            'voltage_std': voltage_array.std()
        }
        
        return time_rel.tolist(), voltage_array.tolist(), stats
    
    def signal_handler(self, sig, frame):
        """Tratar Ctrl+C."""
        print("\n🛑 Encerrando servidor web...")
        self.running = False
        try:
            self.adc.release_adc()
        except:
            pass
        sys.exit(0)

# Instância global do monitor
monitor = WebPolysommographyMonitor()

# Configurar Dash app
app = dash.Dash(__name__)
app.title = "Monitor Polissonografia - RPi 5"

# Layout da interface web
app.layout = html.Div([
    html.Div([
        html.H1("📈 Monitor Polissonografia Profissional", 
                style={'textAlign': 'center', 'color': '#2E86AB', 'marginBottom': '10px'}),
        html.H3("Raspberry Pi 5 + ADS1256 + lgpio", 
                style={'textAlign': 'center', 'color': '#A23B72', 'marginTop': '0px'})
    ]),
    
    html.Div([
        html.Div([
            html.H4("📊 Estatísticas em Tempo Real", style={'color': '#F18F01'}),
            html.Div(id='stats-display', style={'fontSize': '14px'})
        ], style={'width': '30%', 'display': 'inline-block', 'verticalAlign': 'top', 'padding': '20px'}),
        
        html.Div([
            html.H4("🎛️ Controles", style={'color': '#C73E1D'}),
            html.Button('🔄 Reset Estatísticas', id='reset-button', n_clicks=0,
                       style={'margin': '5px', 'padding': '10px'}),
            html.Div(id='button-output', style={'marginTop': '10px'})
        ], style={'width': '30%', 'display': 'inline-block', 'verticalAlign': 'top', 'padding': '20px'}),
        
        html.Div([
            html.H4("📋 Informações do Sistema", style={'color': '#3E7E1D'}),
            html.P(f"🎯 Resolução: {monitor.adc.v_per_digit*1e9:.1f} nanovolts"),
            html.P(f"⚡ Taxa Configurada: {monitor.sample_rate} Hz"),
            html.P(f"📺 Atualização Web: {monitor.display_rate} Hz"),
            html.P(f"🪟 Janela: {monitor.window_size:,} amostras")
        ], style={'width': '40%', 'display': 'inline-block', 'verticalAlign': 'top', 'padding': '20px'})
    ]),
    
    dcc.Graph(id='live-graph', style={'height': '70vh'}),
    
    dcc.Interval(
        id='graph-update',
        interval=1000/monitor.display_rate,  # Atualização em ms
        n_intervals=0
    ),
    
    html.Footer([
        html.P("💡 Ideal para EEG (10-100 µV), EOG (50-500 µV), EMG (50-2000 µV)",
               style={'textAlign': 'center', 'color': 'gray', 'marginTop': '20px'})
    ])
])

# Callback para atualizar gráfico
@app.callback(
    Output('live-graph', 'figure'),
    Input('graph-update', 'n_intervals')
)
def update_graph(n):
    time_data, voltage_data, stats = monitor.get_latest_data()
    
    if not time_data:
        return go.Figure()
    
    # Criar gráfico
    fig = go.Figure()
    
    fig.add_trace(go.Scatter(
        x=time_data,
        y=voltage_data,
        mode='lines',
        name='Sinal Biomédico',
        line=dict(color='cyan', width=1.5),
        hovertemplate='Tempo: %{x:.3f}s<br>Tensão: %{y:.6f}V<extra></extra>'
    ))
    
    fig.update_layout(
        title='📈 Sinal em Tempo Real - 1000 Hz',
        xaxis_title='Tempo (segundos)',
        yaxis_title='Tensão (Volts)',
        plot_bgcolor='black',
        paper_bgcolor='#1e1e1e',
        font=dict(color='white'),
        xaxis=dict(gridcolor='gray', gridwidth=1),
        yaxis=dict(gridcolor='gray', gridwidth=1),
        hovermode='x unified'
    )
    
    return fig

# Callback para atualizar estatísticas
@app.callback(
    Output('stats-display', 'children'),
    Input('graph-update', 'n_intervals')
)
def update_stats(n):
    _, _, stats = monitor.get_latest_data()
    
    if not stats:
        return "⏳ Aguardando dados..."
    
    return [
        html.P(f"📈 Taxa Real: {stats['actual_rate']:.1f} Hz", style={'margin': '5px'}),
        html.P(f"📊 Total de Amostras: {stats['total_samples']:,}", style={'margin': '5px'}),
        html.P(f"⚡ Tensão Atual: {stats['current_voltage']:+.6f}V", style={'margin': '5px'}),
        html.P(f"💾 Queue: {stats['queue_size']} amostras", style={'margin': '5px'}),
        html.P(f"🪟 Buffer: {stats['buffer_size']} amostras", style={'margin': '5px'}),
        html.Hr(),
        html.P(f"📏 Min: {stats['voltage_min']:+.6f}V", style={'margin': '5px'}),
        html.P(f"📏 Max: {stats['voltage_max']:+.6f}V", style={'margin': '5px'}),
        html.P(f"📊 Desvio: {stats['voltage_std']:.6f}V", style={'margin': '5px'})
    ]

# Callback para botão reset
@app.callback(
    Output('button-output', 'children'),
    Input('reset-button', 'n_clicks')
)
def reset_stats(n_clicks):
    if n_clicks > 0:
        monitor.total_samples = 0
        monitor.start_time = time.time()
        monitor.time_buffer.clear()
        monitor.voltage_buffer.clear()
        return html.P("✅ Estatísticas resetadas!", style={'color': 'green'})
    return ""

def main():
    """Função principal."""
    print("\n" + "="*70)
    print("🌐 SERVIDOR WEB POLISSONOGRAFIA")
    print("="*70)
    print(f"⚡ Taxa de coleta: {monitor.sample_rate} Hz (real)")
    print(f"📺 Atualização web: {monitor.display_rate} Hz")
    print(f"🎯 Resolução: {monitor.adc.v_per_digit*1e9:.1f} nanovolts")
    print("\n🌐 ACESSO WEB:")
    print("   • Local: http://localhost:8050")
    print("   • Rede: http://192.168.150.65:8050")
    print("\n💡 FUNCIONALIDADES:")
    print("   • Gráfico interativo em tempo real")
    print("   • Estatísticas de performance")
    print("   • Acesso de qualquer dispositivo na rede")
    print("   • Responsivo (mobile/tablet/desktop)")
    print("="*70)
    
    try:
        # Executar servidor web
        app.run_server(
            host='0.0.0.0',  # Acessível de qualquer IP
            port=8050,
            debug=False,
            dev_tools_hot_reload=False
        )
    except KeyboardInterrupt:
        monitor.signal_handler(None, None)
    except Exception as e:
        print(f"\n❌ Erro: {e}")
        monitor.signal_handler(None, None)

if __name__ == "__main__":
    main()
