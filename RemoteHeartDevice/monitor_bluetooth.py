import sys
import time
import json
import csv
import os
from datetime import datetime

# Valida se a biblioteca pyserial está instalada
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("\n[ERRO] A biblioteca 'pyserial' nao esta instalada.")
    print("Por favor, instale-a rodando o comando:")
    print("    pip install pyserial")
    print("E tente executar o script novamente.\n")
    sys.exit(1)

# Configuração de cores elegantes no terminal
try:
    from colorama import init, Fore, Style
    init(autoreset=True)
    HAS_COLORS = True
except ImportError:
    # Se colorama não estiver instalada, define strings vazias para não quebrar o print
    HAS_COLORS = False
    class EmptyColor:
        def __getattr__(self, name):
            return ""
    Fore = Style = EmptyColor()

LOG_FILE = "heart_metrics_log.csv"

def list_available_ports():
    """Lista as portas COM/TTY ativas no sistema."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        return []
    return ports

def setup_csv_file():
    """Cria o cabeçalho do arquivo CSV se ele não existir."""
    file_exists = os.path.isfile(LOG_FILE)
    try:
        with open(LOG_FILE, mode="a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(["Timestamp", "Status", "BPM", "SpO2", "Avg_BPM", "Avg_SpO2"])
    except IOError as e:
        print(f"{Fore.RED}[ERRO] Nao foi possivel criar ou abrir o arquivo de log {LOG_FILE}: {e}")

def save_to_csv(status, bpm, spo2, avg_bpm, avg_spo2):
    """Salva os dados recebidos no arquivo CSV com carimbo de data/hora."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    try:
        with open(LOG_FILE, mode="a", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, status, bpm, spo2, avg_bpm, avg_spo2])
    except IOError as e:
        print(f"\n{Fore.RED}[AVISO] Erro ao gravar dados no arquivo CSV: {e}")

def show_premium_header():
    """Exibe um cabeçalho moderno e profissional no console."""
    border = "=" * 65
    print(f"\n{Fore.CYAN}{border}")
    print(f"{Fore.CYAN}    MONITOR DE SINAIS VITAIS - RECEPCAO BLUETOOTH (SPP)")
    print(f"{Fore.CYAN}    UNISINOS - SISTEMAS EMBARCADOS 2026/01")
    print(f"{Fore.CYAN}{border}\n")

def main():
    show_premium_header()
    
    # 1. Busca e listagem das portas COM virtuais disponíveis
    print(f"{Fore.YELLOW}[+] Procurando portas COM disponiveis no computador...")
    ports = list_available_ports()
    
    if not ports:
        print(f"{Fore.RED}[!] Nenhuma porta COM foi encontrada.")
        print(f"{Fore.YELLOW}[i] Certifique-se de que o ESP32 ('RemoteHeartDevice') esta pareado com o computador.")
        print(f"    Ao parear, o Windows/OS cria portas COM virtuais automaticamente para conexoes SPP.\n")
        selected_port = input("Digite manualmente o nome da porta COM (ex: COM3): ").strip()
    else:
        print(f"\n{Fore.GREEN}[+] Portas encontradas:")
        for idx, p in enumerate(ports):
            print(f"  [{idx + 1}] {Fore.WHITE}{p.device} {Fore.LIGHTBLACK_EX}- {p.description}")
        
        print("")
        try:
            choice = input(f"Selecione o numero da porta (ou digite manualmente ex: COM3) [1-{len(ports)}]: ").strip()
            if choice.isdigit() and 1 <= int(choice) <= len(ports):
                selected_port = ports[int(choice) - 1].device
            else:
                selected_port = choice
        except (ValueError, KeyboardInterrupt):
            print(f"\n{Fore.RED}[!] Operacao cancelada pelo usuario.")
            sys.exit(0)
            
    if not selected_port:
        print(f"{Fore.RED}[ERRO] Nenhuma porta COM foi selecionada.")
        sys.exit(1)

    # 2. Inicializa o arquivo CSV
    setup_csv_file()
    print(f"{Fore.GREEN}[+] Log de dados configurado em: {Fore.WHITE}{LOG_FILE}")
    print(f"{Fore.YELLOW}[+] Conectando em {Fore.CYAN}{selected_port}{Fore.YELLOW}... (Baudrate padrao 115200)")

    # 3. Estabelece a conexão Serial
    try:
        # Nota: Baudrate é ignorado por portas virtuais Bluetooth SPP, mas é exigido pela API serial.
        ser = serial.Serial(port=selected_port, baudrate=115200, timeout=2.0)
    except serial.SerialException as e:
        print(f"\n{Fore.RED}[ERRO] Nao foi possivel abrir a porta {selected_port}.")
        print(f"Detalhes: {e}")
        print(f"{Fore.YELLOW}[Dica] Certifique-se de que a porta nao esta aberta em outro terminal (como Serial Monitor ou PuTTY).\n")
        sys.exit(1)

    print(f"{Fore.GREEN}[+] Conectado com sucesso! Aguardando transmissao do sensor...\n")
    print(f"{Fore.CYAN}{'-'*65}")
    print(f"{Fore.CYAN}{'TIMESTAMP':<20} | {'STATUS':<12} | {'BPM':<6} ({'MEDIA':<5}) | {'SpO2':<7} ({'MEDIA':<5})")
    print(f"{Fore.CYAN}{'-'*65}")

    # 4. Loop de leitura de dados
    try:
        while True:
            if ser.in_waiting > 0 or True: # Leitura bloqueante com timeout
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                
                # Ignora logs informativos de debug do ESP-IDF (linhas que começam com I, W, E, D, V)
                if len(line) > 1 and line[1] == ' ' and line[0] in ['I', 'W', 'E', 'D', 'V']:
                    continue
                
                # Tenta fazer o parse do JSON enviado pelo ESP32
                try:
                    data = json.loads(line)
                    status = data.get("status", "unknown")
                    timestamp = datetime.now().strftime("%H:%M:%S")

                    if status == "synchronized":
                        bpm = data.get("bpm", 0)
                        spo2 = data.get("spo2", 0)
                        avg_bpm = data.get("avg_bpm", 0)
                        avg_spo2 = data.get("avg_spo2", 0)

                        # Exibição estilizada no terminal
                        print(f"{Fore.WHITE}{timestamp:<20} | "
                              f"{Fore.GREEN}{status:<12} | "
                              f"{Fore.LIGHTRED_EX}{bpm:<6}{Fore.LIGHTBLACK_EX}({avg_bpm:<5}) | "
                              f"{Fore.LIGHTBLUE_EX}{spo2:<3}%{Fore.LIGHTBLACK_EX} ({avg_spo2:<3}%)")
                        
                        save_to_csv(status, bpm, spo2, avg_bpm, avg_spo2)

                    elif status == "no_finger":
                        print(f"{Fore.WHITE}{timestamp:<20} | "
                              f"{Fore.YELLOW}{'SEM DEDO':<12} | "
                              f"{Fore.LIGHTBLACK_EX}{'--':<6}{Fore.LIGHTBLACK_EX}({'--':<5}) | "
                              f"{Fore.LIGHTBLACK_EX}{'--':<7}({'--':<5})")
                        
                        save_to_csv(status, "", "", "", "")

                    elif status == "syncing":
                        collected = data.get("collected", 0)
                        total = data.get("total", 0)
                        print(f"{Fore.WHITE}{timestamp:<20} | "
                              f"{Fore.CYAN}{'SINCRONIZANDO':<12} | "
                              f"{Fore.LIGHTBLACK_EX}Amostras: {collected}/{total}")
                        
                        save_to_csv(status, "", "", "", "")

                    else:
                        print(f"{Fore.WHITE}{timestamp:<20} | {Fore.WHITE}DADO: {line}")

                except json.JSONDecodeError:
                    # Se não for JSON, exibe a linha bruta recebida
                    timestamp = datetime.now().strftime("%H:%M:%S")
                    print(f"{Fore.LIGHTBLACK_EX}{timestamp:<20} | [RAW] {line}")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print(f"\n\n{Fore.YELLOW}[+] Encerrando monitoramento por solicitacao do usuario...")
    finally:
        ser.close()
        print(f"{Fore.GREEN}[+] Conexao serial encerrada. Logs gravados com sucesso em {LOG_FILE}.\n")

if __name__ == "__main__":
    main()
