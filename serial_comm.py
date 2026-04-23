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
        self._dr_sim_distance = 0.0  # Simulacao: distancia acumulada dos encoders

        if not simulate:
            import serial
            self.serial = serial.Serial(port, baudrate, timeout=2)
            time.sleep(2)  # Espera o ESP32 resetar após conexão
            print(f"[SERIAL] Conectado a {port} @ {baudrate}")
        else:
            print("[SERIAL] Modo SIMULAÇÃO ativo — responda no terminal")

    def ping(self, max_tentativas=15, intervalo=2):
        """
        Envia PING ao ESP32 e espera READY como resposta.
        Repete até max_tentativas com intervalo entre cada.

        Args:
            max_tentativas: Número máximo de tentativas (default 15 = 30s)
            intervalo: Segundos entre tentativas

        Returns:
            True se recebeu READY, False se esgotou tentativas
        """
        if self.simulate:
            print("[PING] Modo simulação — conexão assumida OK")
            return True

        print("[PING] A verificar conexão com ESP32...")

        for tentativa in range(1, max_tentativas + 1):
            try:
                self.serial.reset_input_buffer()
                self.serial.write(b"PING\n")
                self.serial.flush()

                response = self.serial.readline().decode().strip()

                if response == "READY":
                    print(f"[PING] ESP32 respondeu READY (tentativa {tentativa})")
                    return True
                else:
                    print(f"[PING] Tentativa {tentativa}/{max_tentativas} — resposta: '{response}'")
            except Exception as e:
                print(f"[PING] Tentativa {tentativa}/{max_tentativas} — erro: {e}")

            if tentativa < max_tentativas:
                time.sleep(intervalo)

        print("[PING] FALHA — ESP32 não respondeu READY")
        return False

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
        """Simulacao via terminal."""
        print(f"  >> {command}")

        if command == "SA":
            response = input("  << (esq,frente,dir em cm - ex: 30,5,45): ").strip()
            return response
        elif command.startswith("MC "):
            # Motor Control — reset do contador de encoder ao iniciar movimento
            parts = command.split()
            speeds = [int(p) for p in parts[1:]]
            if any(s != 0 for s in speeds):
                # Motores a ligar — reset encoder simulado
                self._dr_sim_distance = 0.0
            print("  << OK")
            return "OK"
        elif command == "DR":
            # Distance Read — incrementa 10cm por chamada (simula encoder)
            self._dr_sim_distance += 10.0
            response = f"{self._dr_sim_distance},{self._dr_sim_distance},{self._dr_sim_distance},{self._dr_sim_distance}"
            print(f"  << {response}")
            return response
        elif command.startswith("SERVO"):
            # SERVO espera OK manual — permite controlar quando a foto e tirada
            response = input("  << (digite OK quando pronto): ").strip()
            return response
        else:
            # TURN, VICTIM, PING, etc. — resposta automatica
            print("  << OK")
            return "OK"

    def close(self):
        """Fecha a porta serial."""
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("[SERIAL] Porta fechada")
