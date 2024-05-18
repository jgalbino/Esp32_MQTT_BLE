# Sensor de Velocidade de Esteira com ESP32 (BLE e MQTT)

Este projeto implementa um sensor de velocidade usando um ESP32. Ele combina Bluetooth Low Energy (BLE) para transmitir dados de velocidade e Wi-Fi para conectar a um broker MQTT para publicar a velocidade. A configuração inclui o gerenciamento de conexões Wi-Fi e MQTT, publicidade BLE e processamento de dados do sensor.

## Índice

- [Requisitos de Hardware](#requisitos-de-hardware)
- [Requisitos de Software](#requisitos-de-software)
- [Instalação](#instalação)
- [Configuração](#configuração)
- [Uso](#uso)
- [Explicação do Código](#explicação-do-código)
- [Agradecimentos](#agradecimentos)

## Requisitos de Hardware

- Placa de Desenvolvimento ESP32
- LED (opcional, para feedback visual)
- Sensor de velocidade (configurado para acionar uma interrupção no ESP32)
- Cabos jumper

## Requisitos de Software

- Arduino IDE
- Pacote da placa ESP32 instalado no Arduino IDE
- Biblioteca PubSubClient para MQTT
- Biblioteca BLE para ESP32

## Instalação

1. **Clone o repositório:**

    ```sh
    git clone https://github.com/yourusername/ESP32-BLE-MQTT-Speed-Sensor.git
    ```

2. **Abra o projeto no Arduino IDE:**

    - Abra o Arduino IDE.
    - Vá em `Arquivo -> Abrir` e selecione o arquivo `ESP32-BLE-MQTT-Speed-Sensor.ino`.

3. **Instale as bibliotecas necessárias:**

    - Vá em `Sketch -> Incluir Biblioteca -> Gerenciar Bibliotecas`.
    - Pesquise e instale as seguintes bibliotecas:
      - PubSubClient
      - ESP32 BLE Arduino

4. **Selecione a placa ESP32:**

    - Vá em `Ferramentas -> Placa -> ESP32 Arduino` e selecione o modelo da sua placa ESP32.

5. **Conecte a placa ESP32 ao seu computador usando um cabo USB.**

6. **Carregue o código para o ESP32:**

    - Clique no botão de upload no Arduino IDE para compilar e carregar o código.

## Configuração

1. **Configuração do Wi-Fi:**

    No código, atualize as seguintes linhas com as credenciais do seu Wi-Fi:

    ```cpp
    const char* ssid = "sua-SSID";
    const char* password = "sua-SENHA";
    ```

2. **Configuração do Broker MQTT:**

    Se você estiver usando um broker MQTT diferente, atualize as seguintes linhas:

    ```cpp
    const char* mqtt_server = "broker.hivemq.com";
    const int mqtt_port = 1883;
    ```

3. **Configuração do LED (opcional):**

    Atualize o pino do LED se você estiver usando um pino diferente:

    ```cpp
    const int ledPin = 2;
    ```

## Uso

1. **Alimente o ESP32:**

    Conecte o ESP32 a uma fonte de energia.

2. **Monitore a Saída Serial:**

    Abra o Monitor Serial no Arduino IDE para visualizar as mensagens de status e os dados de velocidade.

3. **Inscrição no MQTT:**

    Inscreva-se no tópico `esp32/speed` no seu broker MQTT para receber atualizações de velocidade.

## Explicação do Código

### Configuração do Wi-Fi

A função `setup_wifi` lida com a conexão do ESP32 a uma rede Wi-Fi. Ela tenta conectar e imprime o endereço IP local em caso de sucesso.

### Configuração e Reconexão do MQTT

A função `reconnect` gerencia a conexão com o broker MQTT. Se a conexão for perdida, ela tenta reconectar a cada 5 segundos.

### Inicialização do BLE

A função `InitBLE` inicializa o BLE no ESP32, configurando o serviço RSC (Velocidade e Cadência de Corrida) e as características.

### Tratamento de Interrupções

A função `sensorTrigger` é uma rotina de serviço de interrupção que processa os dados do sensor de velocidade, calcula a velocidade e atualiza as características do BLE.

### Loop Principal

A função `loop` lida com:

- Publicação dos dados de velocidade no broker MQTT.
- Gerenciamento das notificações BLE.
- Controle do estado do LED com base na atividade do sensor.

### Cálculo de Velocidade

A velocidade é calculada com base no tempo entre os acionamentos do sensor, atualizada usando um buffer de média móvel para estabilidade.

## Agradecimentos

Este projeto utiliza bibliotecas e exemplos de código dos seguintes recursos:

- [ESP32 BLE Arduino](https://github.com/nkolban/ESP32_BLE_Arduino)
- [PubSubClient](https://github.com/knolleary/pubsubclient)

Para explicações detalhadas e solução de problemas, consulte a documentação oficial das bibliotecas utilizadas.

---

Sinta-se à vontade para personalizar este arquivo README para atender melhor às especificidades do seu projeto e incluir qualquer informação ou seção adicional conforme necessário.
