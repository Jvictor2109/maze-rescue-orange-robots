"""
Módulo de comunicação serial com o ESP32.
Dois modos de operação:
  - Real: usa pyserial para comunicar via porta COM/USB
  - Simulação: imprime comandos no terminal e lê respostas do teclado
"""

import time


class SerialComm:
    """Camada de comunicação com o ESP32."""

    def __init__(self, port=None, baudrate=115200, simulate=False):
        """
        Args:
            port: Porta serial (ex: 'COM3', '/dev/ttyUSB0')
            baudrate: Velocidade da comunicação
            simulate: Se True, usa terminal em vez de serial real
        """
        self.simulate = simulate
        self.serial = None

        if not simulate:
            import serial
            self.serial = serial.Serial(port, baudrate, timeout=2)
            time.sleep(2)  # Espera o ESP32 resetar após conexão
            print(f"[SERIAL] Conectado a {port} @ {baudrate}")
        else:
            print("[SERIAL] Modo SIMULAÇÃO ativo — responda no terminal")

    def send(self, command):
        """
        Envia um comando e aguarda resposta.
        
        Args:
            command: String do comando (ex: 'MOVE FORWARD', 'SENSOR ALL')
        
        Returns:
            String com a resposta do ESP32
        """
        if self.simulate:
            return self._simulate_send(command)
        else:
            return self._serial_send(command)

    def _serial_send(self, command):
        """Envio real via pyserial."""
        # Limpa buffer de entrada
        self.serial.reset_input_buffer()

        # Envia comando com terminador
        self.serial.write((command + "\n").encode())
        self.serial.flush()

        # Aguarda resposta (linha terminada em \n)
        response = self.serial.readline().decode().strip()

        if not response:
            print(f"[SERIAL] AVISO: Sem resposta para '{command}'")

        return response

    def _simulate_send(self, command):
        """Simulação via terminal."""
        print(f"  >> {command}")

        # Comandos de movimento/servo retornam OK automaticamente na simulação
        # Apenas SENSOR ALL precisa de input do utilizador
        if command == "SENSOR ALL":
            response = input("  << (frente,esq,dir — ex: 0,1,0): ").strip()
            return response
        else:
            # Para MOVE/TURN/SERVO, simula resposta automática
            print("  << OK")
            return "OK"

    def close(self):
        """Fecha a porta serial."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("[SERIAL] Porta fechada")
