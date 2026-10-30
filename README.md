# 🔑 Controle de Acesso RFID Nativo com ESP32 (ESP-IDF v6.x)

Este projeto implementa um **sistema profissional de controle de acesso automatizado** utilizando o microcontrolador **ESP32** e o leitor RFID **RC522**, desenvolvido de forma nativa em C sobre o framework **ESP-IDF v6.1**. 

O sistema conta com gerenciamento inteligente de memória flash através do componente **NVS (Non-Volatile Storage)**, permitindo cadastrar e remover cartões administrativos e de usuários diretamente por aproximação, sem a necessidade de recomputar o código.

## Diagrama Elétrico

<p align="center">
  <img src="https://github.com/PDassVDC/Controle-de-Acesso-RFID-ESP32/blob/main/imagens/diagrama_el%C3%A9trico.png" alt="Diagrama Elétrico" />
</p>

Conforme indicado no esquema elétrico, a fonte de 24V é conectada diretamente à rede para alimentar o circuito de comando. Este circuito é formado por uma botoeira de parada de emergência e uma botoeira do tipo push-button em série com um relé de controle eletrônico, encarregado de comandar o solenoide do contator de 24V. No circuito de potência, encontram-se o disjuntor geral e os contatos do contator conectados à carga, que é o motor elétrico.


## 🚀 Funcionalidades

*   **Comunicação Nativa SPI:** Implementação direta dos registradores do RC522, garantindo total compatibilidade com as mudanças de drivers do ESP-IDF v6.x.
*   **Conceito de Tag Mestra (Master):** O primeiro cartão lido com a memória vazia torna-se automaticamente o administrador do sistema.
*   **Menu Físico por Aproximação:** Permite entrar em modo de configuração para cadastrar novos usuários ou remover permissões apenas aproximando os chaveiros/cartões.
*   **Memória Persistente (NVS):** Os IDs autorizados permanecem salvos de forma segura na memória flash interna, mesmo se o ESP32 for desligado.
*   **Acionamento Automatizado:** Controle temporizado de uma saída digital (GPIO 2) por 3 segundos para acionamento de módulos relé ou fechaduras eletrônicas.

## 📋 Pinagem e Conexões (Padrão VSPI)

O leitor RC522 opera estritamente com **3.3V**. Certifique-se de realizar as conexões conforme a tabela abaixo:

| Pino RFID-RC522 | Pino ESP32 | Função |
| :--- | :--- | :--- |
| **VCC** | **3.3V** | Alimentação do Módulo (Não ligar em 5V!) |
| **RST** (Reset) | *Direto no 3.3V* | Reset (Mantido ativo em nível alto) |
| **GND** | **GND** | Terra / Negativo Comum |
| **MISO** | **GPIO 19** | SPI Master Input Slave Output |
| **MOSI** | **GPIO 23** | SPI Master Output Slave Input |
| **SCK** | **GPIO 18** | SPI Serial Clock |
| **SDA** (CS) | **GPIO 21** | SPI Chip Select |
| *Sinal do Relé* | **GPIO 2** | Saída digital para controle da tranca |

---

## 🎮 Guia de Operação

### 1. Definição do Administrador (Primeiro Uso)
Ao ligar o circuito pela primeira vez, o banco de dados estará vazio. Aproxime qualquer tag RFID: ela será registrada permanentemente como a sua **Tag Mestra**.

### 2. Cadastrar ou Remover Usuários
1. Aproxime a **Tag Mestra** para ativar o `Modo de Cadastro`.
2. Aproxime uma **nova tag**: ela será cadastrada e receberá autorização.
3. Aproxime uma **tag existente**: ela será deletada do banco de dados imediatamente.
4. Aproxime a **Tag Mestra** novamente para encerrar as configurações.

### 3. Liberação de Acesso
*   **Tag Autorizada:** O pino **GPIO 2** é ativado por 3 segundos (abrindo o relé/fechadura) e o log exibe `🟢 ACESSO PERMITIDO!`.
*   **Tag Desconhecida:** O relé permanece bloqueado e o sistema emite um alerta `🔴 ACESSO NEGADO!`.

---

## 🛠️ Como Compilar e Gravar

1. Certifique-se de que o arquivo `main/CMakeLists.txt` do seu projeto declare os requisitos nativos:
   ```cmake
   idf_component_register(SRCS "main.c"
                          INCLUDE_DIRS "."
                          PRIV_REQUIRES esp_driver_spi esp_driver_gpio nvs_flash)
   ```
2. Abra o terminal do VS Code e execute a limpeza completa seguida da gravação:
   ```bash
   idf.py fullclean
   idf.py build flash monitor
   ```

*💡 Para realizar um reset de fábrica e apagar todas as tags (incluindo a Mestra), digite `idf.py erase-flash` no terminal.*

## 🛠️ Desenvolvido Por:
**Leonardo Romão**,
**Leonardo Oliveira** e **Eduardo Silva**

