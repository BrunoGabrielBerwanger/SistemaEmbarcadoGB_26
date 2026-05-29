# Projeto de Sistemas Embarcados - Grau B

**Universidade do Vale do Rio dos Sinos - UNISINOS** 

**Disciplina:** Sistemas Embarcados - 2026/01 

**Atividade:** Atividade Grau B 

**Integrantes:**
1. Gabriel Krabbe Fonseca
2. Bruno Gabriel Berwanger
---

## 1. Descrição Inicial do Projeto

O projeto consiste no desenvolvimento de um **Monitor de Sinais Vitais Embarcado**. O dispositivo tem a finalidade de realizar a leitura contínua dos batimentos cardíacos (BPM) e da oxigenação do sangue (SpO2) de um usuário através do sensor óptico MAX30102. Os dados processados são disponibilizados remotamente via Bluetooth, permitindo o acompanhamento em tempo real através de um terminal ou script de monitoramento no computador. 

A solução será inteiramente desenvolvida utilizando o microcontrolador ESP32 DevKit1 com o SDK do fabricante (ESP-IDF).

## 2. Requisitos do Projeto
Para o correto funcionamento e atendimento aos critérios estabelecidos, o projeto cumprirá os seguintes requisitos:
* **Hardware base:** Utilização do microcontrolador ESP32 programado via ESP-IDF, sem o uso de abstrações de nível Arduino.
* **Sistema Operacional:** Implementação baseada em FreeRTOS.
* **Concorrência:** Utilização de tarefas independentes para execução paralela de leitura e transmissão.
* **Sincronização:** Utilização de Mutex para proteção de memória na região crítica.
* **Interfaceamento:** Integração com o sensor MAX30102 através do barramento I2C.
* **Conectividade:** Transmissão dos dados processados utilizando a interface Bluetooth (SPP).

## 3. Esquemático e Conexões
O hardware do protótipo será montado interligando o sensor ao ESP32. As conexões lógicas seguem o padrão I2C, garantindo a correta tensão de operação para as linhas de dados:
* **ESP32 3V3** -> MAX30102 VCC (Tensão de alimentação)
* **ESP32 GND** -> MAX30102 GND
* **ESP32 GPIO 21** -> MAX30102 SDA (Dados I2C)
* **ESP32 GPIO 22** -> MAX30102 SCL (Clock I2C)

## 4. Arquitetura de Tarefas (RTOS)
A execução paralela será dividida em duas tarefas principais dentro do FreeRTOS
### Task A: Aquisição I2C (`vTaskSensorAquisition`)
* **Descrição:** Responsável por se comunicar com o MAX30102 via barramento I2C.
* **Comportamento:** Faz a leitura dos registradores internos do sensor, aplica os algoritmos de cálculo para extrair BPM e SpO2, e atualiza a estrutura de dados global.

### Task B: Transmissão Bluetooth (`vTaskBluetoothTx`)
* **Descrição:** Responsável pela conectividade e exportação dos dados.
* **Comportamento:** Fica em estado de bloqueio/aguardo a maior parte do tempo. Periodicamente, acorda, lê a estrutura de dados global e formata um pacote (ex: JSON) para envio pela interface serial virtual do Bluetooth.

## 5. Região Crítica e Proteção de Memória
O compartilhamento de dados entre a **Task A** e a **Task B** exige gerenciamento rigoroso.

* **Região Crítica:** Consiste na estrutura de dados (`struct`) global em C que armazena os valores atualizados de `uint8_t bpm` e `uint8_t spo2`.
* **Mecanismo de Proteção:** Será utilizado um **Mutex** (Mutual Exclusion. 
* **Fluxo de Proteção:** 1. Quando a `vTaskSensorAquisition` possui novos dados processados, ela solicita o Mutex (`xSemaphoreTake`). 
  2. Ao obter a chave, escreve os novos valores na `struct` e devolve o Mutex (`xSemaphoreGive`). 
  3. De forma análoga, a `vTaskBluetoothTx` só consegue ler a `struct` após obter o mesmo Mutex. Isso impede condições de corrida, garantindo que o Bluetooth não transmita dados fragmentados.

## 6. Conectividade e Recepção
A etapa de conectividade será realizada via Bluetooth Clássico utilizando o perfil SPP (Serial Port Profile) nativo do ESP-IDF.

O ESP32 operará como um servidor SPP. Ao parear o dispositivo com um computador, ele criará uma porta COM virtual. A demonstração incluirá a recepção desses dados em tempo real, o que pode ser integrado posteriormente a um script de automação em Python para logging em arquivos.
