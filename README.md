# ESP32 Power Monitor — ESP-IDF

Projeto para ESP32-WROOM-32, usando ESP-IDF. Implementa aquisição com GPTimer, RMS por janela de 64 amostras, FSM NORMAL/SAG/SWELL, LCD 16x2 via I2C/PCF8574, LEDs, eventos em SD CARD via SPI/FATFS e publicação MQTT assíncrona com payload CSV.

## 1. Antes de ligar o circuito

**Não conecte a rede elétrica diretamente ao ESP32.** A placa precisa de uma interface de medição isolada e condicionada, projetada para a tensão/categoria de instalação reais. O GPIO34 é entrada ADC, não tolera tensão negativa nem deve receber 5 V.

Há uma inconsistência nos parâmetros de entrada fornecidos:

- Rede nominal: 127 V RMS.
- Atenuação: 146,05.
- Offset: 1,15 V.

## 2. Valores que você deve configurar

Edite `main/main.c`:

- `WIFI_SSID` e `WIFI_PASSWORD`.
- `MQTT_BROKER_URI`, por exemplo `mqtt://192.168.1.20:1883`.
- `MQTT_TOPIC_EVENTS`.
- `VOLTAGE_DIVIDER_RATIO`, `ADC_DC_OFFSET_V` e `GRID_NOMINAL_VRMS` após calibrar o condicionador.
- `PU_SAG_ENTER`, `PU_SAG_EXIT`, `PU_SWELL_ENTER`, `PU_SWELL_EXIT`.
- GPIOs, endereço I2C do LCD e pinos SPI do SD conforme suas placas.

## 3. Ligações padrão do projeto

| Função | Pino ESP32 |
|---|---:|
| Entrada ADC (sinal condicionado) | GPIO34 / ADC1_CH6 |
| LED NORMAL | GPIO27 |
| LED SAG | GPIO25 |
| LED SWELL | GPIO26 |
| LCD SDA | GPIO21 |
| LCD SCL | GPIO22 |
| SD MOSI | GPIO23 |
| SD MISO | GPIO19 |
| SD SCLK | GPIO18 |
| SD CS | GPIO13 |

## 4. Compilação e gravação

Com ESP-IDF v6.0+ instalado e o ambiente ativado, execute:

```bash
cd esp32_power_monitor
idf.py set-target esp32
idf.py menuconfig
idf.py build
idf.py -p PORTA_SERIAL flash monitor
```
## 5. Lógica do firmware

- GPTimer gera um sinal de amostragem a cada 260 µs (~3,846 kS/s). A ISR apenas libera um semáforo; a leitura do ADC ocorre na `adc_task`, evitando chamar `adc_oneshot_read()` dentro da ISR.
- A cada 64 amostras, a tarefa calcula a média e o RMS AC discreto, converte a escala para Vrms e divide pela tensão nominal para obter p.u.
- `fsm_task` aplica histerese nos limites de entrada/saída e registra eventos completos, com duração monotônica e picos de p.u./Vrms.
- Cada evento concluído é copiado para duas filas diferentes, `sd_event_queue` e `mqtt_event_queue`. Assim SD e MQTT não competem pelo consumo de uma única fila.
- `sd_card_task` é a única tarefa que acessa o arquivo CSV. Ela usa `xQueuePeek()` e só remove o evento da fila depois de uma gravação bem-sucedida.
- `mqtt_task` mantém eventos na fila até haver conexão e espaço na outbox MQTT; usa `esp_mqtt_client_enqueue()` para enfileirar publicação sem enviar a rede diretamente a partir da task de aplicação.
- O relógio usa SNTP após a conexão Wi-Fi. `start_epoch_s`/`end_epoch_s` valem `0` enquanto não há horário sincronizado; `duration_ms` vem de timer monotônico e continua válido.

## 6. Formato CSV

O arquivo `/sdcard/events.csv` tem cabeçalho:

```text
start_epoch_s,end_epoch_s,duration_ms,type,min_pu,max_pu,min_vrms,max_vrms
```

Cada publicação MQTT é apenas uma linha CSV, sem JSON. Configure o broker e o tópico antes de usar. A fila e a outbox são caches em RAM, ou seja, não representam armazenamento persistente garantido durante uma queda de energia ou durante indisponibilidade prolongada de rede.

## Referências oficiais

- [ADC oneshot - ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc/adc_oneshot.html)
- [GPTimer - ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32/api-reference/peripherals/gptimer.html)
- [SD SPI host - ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sdspi_host.html)
- [ESP-MQTT - ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32/api-reference/protocols/mqtt.html)
