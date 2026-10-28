# ESP32 Power Monitor — ESP-IDF

Projeto-base para ESP32-WROOM-32, usando ESP-IDF 5.3–5.5. Implementa aquisição com GPTimer, RMS por janela de 64 amostras, FSM NORMAL/SAG/SWELL, LCD 16x2 via I2C/PCF8574, LEDs, eventos em SD via SPI/FATFS e publicação MQTT assíncrona com payload CSV.

## 1. Antes de ligar o circuito

**Não conecte a rede elétrica diretamente ao ESP32.** A placa precisa de uma interface de medição isolada e condicionada, projetada para a tensão/categoria de instalação reais. O GPIO34 é entrada ADC, não tolera tensão negativa nem deve receber 5 V.

Há uma inconsistência nos parâmetros de entrada fornecidos:

- Rede nominal: 127 V RMS.
- Atenuação: 146,05.
- Offset: 1,15 V.
- Pico esperado da rede a 127 V RMS: 127 × √2 ≈ 179,6 V.
- Excursão no ADC com o fator 146,05: 179,6 / 146,05 ≈ 1,23 V de pico.
- Isso implicaria teoricamente cerca de −0,08 V a 2,38 V, fora da faixa afirmada de 0,15–2,15 V.

O firmware mede o componente AC subtraindo a média de cada janela e calcula `grid_rms_v = adc_rms_ac_v * VOLTAGE_DIVIDER_RATIO`. Isso remove o offset de DC observado; `ADC_DC_OFFSET_V` é documentado como referência nominal, não como substituto da medição/condicionamento físico. **Valide o circuito e ajuste o fator de escala antes de confiar no valor em volts ou p.u.**

## 2. Valores que você deve configurar

Edite `main/main.c`:

- `WIFI_SSID` e `WIFI_PASSWORD`.
- `MQTT_BROKER_URI`, por exemplo `mqtt://192.168.1.20:1883`.
- `MQTT_TOPIC_EVENTS`.
- `VOLTAGE_DIVIDER_RATIO`, `ADC_DC_OFFSET_V` e `GRID_NOMINAL_VRMS` após calibrar o condicionador.
- `PU_SAG_ENTER`, `PU_SAG_EXIT`, `PU_SWELL_ENTER`, `PU_SWELL_EXIT`.
- GPIOs, endereço I2C do LCD e pinos SPI do SD conforme suas placas.

No ESP-IDF 5.x, `ADC_ATTEN_DB_12` é o nome comum atual da faixa historicamente chamada de 11 dB. O código seleciona esse nome nas versões 5.x e mantém `ADC_ATTEN_DB_11` para versões anteriores.

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

O LCD usa um backpack PCF8574 com o mapeamento comum `P0=RS`, `P1=RW`, `P2=EN`, `P3=backlight`, `P4..P7=D4..D7`. Muitos módulos usam endereço `0x27`, outros usam `0x3F`; ajuste `LCD_I2C_ADDRESS`. Use resistores em série nos LEDs e confirme o nível lógico do backpack I2C. Não conecte um pull-up I2C de 5 V diretamente ao ESP32.

## 4. Compilação e gravação

Com ESP-IDF 5.3–5.5 instalado e o ambiente ativado:

```bash
cd esp32_power_monitor
idf.py set-target esp32
idf.py menuconfig
idf.py build
idf.py -p PORTA_SERIAL flash monitor
```

Substitua `PORTA_SERIAL` por algo como `/dev/ttyUSB0`. Se a versão do IDF tiver um `mqtt` externo (ESP-IDF 6.0+), adicione a dependência com `idf.py add-dependency espressif/mqtt` e adapte o `CMakeLists.txt` conforme o componente instalado.

## 5. Lógica do firmware

- GPTimer gera um sinal de amostragem a cada 260 µs (~3,846 kS/s). A ISR apenas libera um semáforo; a leitura do ADC ocorre na `adc_task`, pois `adc_oneshot_read()` não deve ser chamada dentro da ISR.
- A cada 64 amostras, a tarefa calcula a média e o RMS AC discreto, converte a escala para Vrms e divide pela tensão nominal para obter p.u.
- `fsm_task` aplica histerese nos limites de entrada/saída e registra eventos completos, com duração monotônica e picos de p.u./Vrms.
- Cada evento concluído é copiado para duas filas diferentes, `sd_event_queue` e `mqtt_event_queue`. Assim SD e MQTT não competem pelo consumo de uma única fila.
- `sd_card_task` é a única tarefa que acessa o arquivo CSV. Ela usa `xQueuePeek()` e só remove o evento da fila depois de uma gravação bem-sucedida.
- `mqtt_task` mantém eventos na fila até haver conexão e espaço na outbox MQTT; usa `esp_mqtt_client_enqueue()` para enfileirar publicação sem enviar a rede diretamente a partir da task de aplicação.
- O relógio usa SNTP após a conexão Wi-Fi. `start_epoch_s`/`end_epoch_s` valem `0` enquanto não há horário sincronizado; `duration_ms` vem de timer monotônico e continua válido.

A janela fixa de 64 amostras em ~3,846 kS/s mede aproximadamente 16,64 ms. Isso se aproxima de um período nominal de 60 Hz, mas não sincroniza com cruzamento por zero nem acompanha variações de frequência. Para medição metrológica ou conformidade normativa, use aquisição contínua/DMA ou sincronização por cruzamento por zero e valide o condicionador, filtros e calibração.

## 6. Formato CSV

O arquivo `/sdcard/events.csv` tem cabeçalho:

```text
start_epoch_s,end_epoch_s,duration_ms,type,min_pu,max_pu,min_vrms,max_vrms
```

Cada publicação MQTT é apenas uma linha CSV, sem JSON. Configure o broker e o tópico antes de usar. A fila e a outbox são caches em RAM; não representam armazenamento persistente garantido durante uma queda de energia ou durante indisponibilidade prolongada de rede.

## Referências oficiais

- [ADC oneshot — ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc/adc_oneshot.html)
- [GPTimer — ESP-IDF 5.3](https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32/api-reference/peripherals/gptimer.html)
- [SD SPI host — ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/sdspi_host.html)
- [ESP-MQTT — ESP-IDF 5.5.1](https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32/api-reference/protocols/mqtt.html)
