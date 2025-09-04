# Makefile para High-Speed ADC Engine
# ===================================

CC = gcc
CFLAGS = -O3 -Wall -Wextra -std=c99 -pthread -ffast-math
LDFLAGS = -pthread -lrt
TARGET = adc_engine
SOURCE = adc_engine_simple.c

# Alvos
all: $(TARGET)

$(TARGET): $(SOURCE)
	@echo "🔧 Compilando ADC Engine Simplificado..."
	$(CC) $(CFLAGS) -o $(TARGET) $(SOURCE) $(LDFLAGS)
	@echo "✅ Compilação concluída!"

clean:
	@echo "🧹 Limpando arquivos..."
	rm -f $(TARGET)
	@echo "✅ Limpeza concluída!"

install:
	@echo "📦 Instalando dependências..."
	sudo apt update
	sudo apt install -y build-essential
	@echo "✅ Dependências instaladas!"

run: $(TARGET)
	@echo "🚀 Executando ADC Engine..."
	./$(TARGET)

debug: $(SOURCE)
	@echo "🐛 Compilando versão debug..."
	$(CC) $(CFLAGS) -DDEBUG -g -o $(TARGET)_debug $(SOURCE) $(LDFLAGS)
	@echo "✅ Debug compilado!"

.PHONY: all clean install run debug
