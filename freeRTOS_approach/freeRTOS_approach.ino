#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <WiFi.h>
#include <HTTPClient.h>

// constantes
#define GRID_FREQUENCY 60                          // frequência da fundamental da rede elétrica
#define SAMPLES 128                                // quantidade de amostras por ciclo da rede (tem que ser 2^n)
#define CPT_COLETA SAMPLES                         // CPT - amostras por ciclo
#define SAMPLING_FREQUENCY GRID_FREQUENCY *SAMPLES // taxa de amostragem do sinal // 128*60 (Hz)
#define SWAP(a, b) \
  tempr = (a);     \
  (a) = (b);       \
  (b) = tempr                           // constante/função da rede elétrica
#define TSH 1 / SAMPLING_FREQUENCY      // tempo da amostragem = 1/FREQUENCIA_AMOSTRAGEM
#define FILTER_LEN 4                    // tamanho do filtro da tensao
#define FILTER_LEN_CURR FILTER_LEN      // tamanho do filtro da corrente
#define CONFIG_FILE_PATH "/config.json" // Path for the configuration JSON file

// tipos enumerados de tamanhos
enum
{
  BUF_LEN = 5 * SAMPLES
}; // máximo de valores no buffer circular (5 ciclos)
enum
{
  MSG_LEN = 100
}; // máximo de caracteres no corpo de mensagem
enum
{
  MSG_QUEUE_LEN = 5
}; // slots da fila de mensagens
enum
{
  CMD_BUF_LEN = 255
}; // número de caracteres no buffer de comandos

// structs
// estrutura da média móvel
typedef struct sMAF
{
  double MAF_input;
  int MAF_KONTz;
  double MAF_summedz;
  double vector_jm[CPT_COLETA];
  double mean_out;
  int inicio;
} sMAF;

// estrutura do RMS
typedef struct RMS
{
  int inicio;
  int NA;
  double temp;
  double jm[CPT_COLETA];
  double summed;
  double v_RMS;
} RMS;

// estrutura da integral imparcial
typedef struct UI
{
  double integral;
  double integral_old;
  double out;
  int inicio;
} UI;

// estrutura que armazena os valores da CPT
typedef struct CPT_Valores
{
  double P;
  double N;
  double A;
  double D;
  double Q;
  double PF;
  double QF;
  double VF;
} CPT_Valores;

// struct da mensagem
typedef struct Message
{
  char body[MSG_LEN];
} Message;

// variáveis para a integral imparcial (unbiased integral)
static UI ui_ua;
static sMAF mui_ua;
static RMS UI_ua, UI_u;
static CPT_Valores CPT_Val;

// variáveis para a decomposição da corrente instantânea
static double iaa = 0, ibaa = 0;
static double iuaa = 0, ira = 0;
static double ibra = 0, iura = 0;
static double inaa = 0;
static double iva = 0, iua = 0;

// rms variables
static RMS Ua, Ia, U, I;
static RMS Iba, Iua, Ibr, Iur, Iv;

// Energy and Power variables
static sMAF Pa, P;
static sMAF Wa, W;

// Parcelas da CPT
static double Qa = 0; //, Q2=0;
static double A = 0, Q = 0, Na = 0, Nr = 0, N = 0, V = 0;
static double DeltaTHD = 0;
static double Iu = 0;

// static double PF=0, VF=0, QF=0;

// Wi-Fi credentials
const char *ssid = "Fontana";
const char *password = "@678fontana786";

// definiçoes dos núcleos
// %%% podemos usar os dois núcleos agora
static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;

// lista de comandos (nesta versão de implementações)
// no futuro a ideia se tornará enviar estes dados a uma dinâmica diferente
// comandos de tensão
static const char v_command1[] = "v_avg";  // valor médio de tensão
static const char v_command2[] = "v_fft";  // fft da tensão até 31ª ordem
static const char v_command3[] = "v_inst"; // forma de onda instantânea da tensão
static const char v_command4[] = "v_raw";  // forma de onda que sai direto do ADC da tensão
static const char v_command5[] = "v_rms";  // valor rms (eficaz) da tensão
// comandos de corrente
static const char i_command1[] = "i_avg";  // valor médio da corrente
static const char i_command2[] = "i_fft";  // fft da corrente até 31ª ordem
static const char i_command3[] = "i_inst"; // forma de onda instantânea da corrente
static const char i_command4[] = "i_raw";  // forma de onda que sai direto do ADC da corrente
static const char i_command5[] = "i_rms";  // valor rms (eficaz) da corrente
// comandos de tensão e corrente (sinais em conjunto)
static const char vi_command1[] = "vi_ciclo"; // um ciclo dos sinais de tensão e corrente
static const char vi_command2[] = "vi_inst";  // tensão e corrente instantâneas
// comandos de potência (tensão x corrente)
static const char p_command1[] = "pot_inst"; // Potência Instantânea
// comandos de qualidade de energia elétrica
static const char qee_command1[] = "info_qee"; // dados de qualidade de energia elétrica
// comandos baseados na CPT
static const char cpt_command1[] = "cpt"; // indicadores (potências e indicadores) da CPT
// comandos diversos (etc)
static const char etc_command[] = "v_semi";  // sinal de semi ciclo de tensão
static const char etc_command1[] = "i_semi"; // sinal de semi ciclo de corrente

// ajustes de timer
static const uint16_t timer_divider = 8;      // Divide 80 MHz do clock
static const uint64_t timer_max_count = 1302; // Contagem de clock para atingir a captura
static const uint32_t cli_delay = 10;         // ms delay

// componentes para o sinal médio (e fft)
int16_t i = 0;
double vComp[2 * SAMPLES];
double iComp[2 * SAMPLES];
double tensao_instantanea[3 * SAMPLES];
double corrente_instantanea[3 * SAMPLES];

// Globais
static hw_timer_t *timer = NULL;
static hw_timer_t *itimer = NULL;

// FreeRTOS
// handles dos semáforos, estados críticos
static TaskHandle_t processing_task = NULL;
static TaskHandle_t i_processing_task = NULL;
static SemaphoreHandle_t sem_done_reading = NULL;
static SemaphoreHandle_t i_sem_done_reading = NULL;
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t msg_queue;

// vetores dos buffers ciculares de tensão e de corrente
// vetores circulares de tensão
static volatile uint16_t buf_0[BUF_LEN];     // um buffer de par
static volatile uint16_t buf_1[BUF_LEN];     // outro buffer no par
static volatile uint16_t *write_to = buf_0;  // buffer do ponteiro de escrita
static volatile uint16_t *read_from = buf_1; // buffer do ponteiro de leitura
// vetores circulares de corrente
static volatile uint16_t i_buf_0[BUF_LEN];       // um buffer de par
static volatile uint16_t i_buf_1[BUF_LEN];       // outro buffer no par
static volatile uint16_t *i_write_to = i_buf_0;  // buffer do ponteiro de escrita
static volatile uint16_t *i_read_from = i_buf_1; // buffer do ponteiro de leitura
// variáveis de overrun
static volatile uint8_t buf_overrun = 0;   // buffer do overrun
static volatile uint8_t i_buf_overrun = 0; // buffer do overrun

// valor médio
static float adc_avg;
static float i_adc_avg;

// estrutura para o filtro
static uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0};
static uint32_t i_AN_Pot1_Buffer[FILTER_LEN_CURR] = {0};
static int AN_Pot1_i = 0;
static int i_AN_Pot1_i = 0;

// estrutura de valores de QEE
static double Vrms;
static double Irms;
static double FP;
static double DHTV;
static double DHTI;

// variáveis de calibração
esp_adc_cal_characteristics_t adc_chars;
esp_adc_cal_characteristics_t adc_chars_i;

// estrutura de troca do buffer circular de tensão
void IRAM_ATTR swap()
{
  volatile uint16_t *temp_ptr = write_to;
  write_to = read_from;
  read_from = temp_ptr;
}

// estrutura de troca do buffer circular de corrente
void IRAM_ATTR i_swap()
{
  volatile uint16_t *i_temp_ptr = i_write_to;
  i_write_to = i_read_from;
  i_read_from = i_temp_ptr;
}

// função de timer para coletar a tensão e armazenar em buffer
void IRAM_ATTR onTimer()
{
  static uint16_t idx = 0;
  BaseType_t task_woken = pdFALSE;
  if ((idx < BUF_LEN) && (buf_overrun == 0))
  {
    write_to[idx] = adc1_get_raw(ADC1_CHANNEL_7);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    write_to[idx] = esp_adc_cal_raw_to_voltage(write_to[idx], &adc_chars);
    write_to[idx] = readADC_Avg(write_to[idx]);
    idx++;
  }
  if (idx >= BUF_LEN)
  {
    if (xSemaphoreTakeFromISR(sem_done_reading, &task_woken) == pdFALSE)
    {
      buf_overrun = 1;
    }
    if (buf_overrun == 0)
    {
      idx = 0;
      swap();
      vTaskNotifyGiveFromISR(processing_task, &task_woken);
    }
  }
  if (task_woken)
  {
    portYIELD_FROM_ISR();
  }
}

// função de timer para coletar a corrente e armazenar em buffer
void IRAM_ATTR onTimerI()
{
  static uint16_t iidx = 0;
  BaseType_t i_task_woken = pdFALSE;
  if ((iidx < BUF_LEN) && (i_buf_overrun == 0))
  {
    i_write_to[iidx] = adc1_get_raw(ADC1_CHANNEL_6);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars_i);
    i_write_to[iidx] = esp_adc_cal_raw_to_voltage(i_write_to[iidx], &adc_chars_i);
    i_write_to[iidx] = i_readADC_Avg(i_write_to[iidx]);
    iidx++;
  }
  if (iidx >= BUF_LEN)
  {
    if (xSemaphoreTakeFromISR(i_sem_done_reading, &i_task_woken) == pdFALSE)
    {
      i_buf_overrun = 1;
    }
    if (i_buf_overrun == 0)
    {
      iidx = 0;
      i_swap();
      vTaskNotifyGiveFromISR(i_processing_task, &i_task_woken);
    }
  }
  if (i_task_woken)
  {
    portYIELD_FROM_ISR();
  }
}

// função para calcular a FFT
void fft(double data[], unsigned long nn)
{
  unsigned long n = 0, mmax = 0, m = 0, j = 0, istep = 0, i = 0;
  float wtemp = 0, wr = 0, wpr = 0, wpi = 0, wi = 0, theta = 0;
  float tempr = 0, tempi = 0;

  n = nn << 1; // n is the size of data array (2*nn)
  j = 1;
  for (i = 1; i < n; i += 2)
  {
    if (j > i)
    { // bit reversal section
      SWAP(data[j - 1], data[i - 1]);
      SWAP(data[j], data[i]);
    }
    m = n >> 1;
    while ((m >= 2) && (j > m))
    {
      j = j - m;
      m = m >> 1;
    }
    j = j + m;
  }
  mmax = 2; // Danielson-Lanczos section
  while (n > mmax)
  { // executed log2(nn) times
    istep = mmax << 1;
    theta = -6.283185307179586476925286766559 / mmax;
    // the above line should be + for inverse FFT
    wtemp = sin(0.5 * theta);
    wpr = -2.0 * wtemp * wtemp; // real part
    wpi = sin(theta);           // imaginary part
    wr = 1.0;
    wi = 0.0;
    for (m = 1; m < mmax; m += 2)
    {
      for (i = m; i <= n; i = i + istep)
      {
        j = i + mmax;
        tempr = wr * data[j - 1] - wi * data[j]; // Danielson-Lanczos formula
        tempi = wr * data[j] + wi * data[j - 1];
        data[j - 1] = data[i - 1] - tempr;
        data[j] = data[i] - tempi;
        data[i - 1] = data[i - 1] + tempr;
        data[i] = data[i] + tempi;
      }
      wtemp = wr;
      wr = wr * wpr - wi * wpi + wr;
      wi = wi * wpr + wtemp * wpi + wi;
    }
    mmax = istep;
  }
}

// função para retornar a magnitude do sinal
float fftMagnitude(double data[], unsigned long nn, unsigned long k)
{
  float nr, realPart, imagPart;

  nr = (float)nn;
  if (k >= nn / 2)
  {
    return 0.0; // out of range
  }
  if (k == 0)
  {
    return sqrt(data[0] * data[0] + data[1] * data[1]) / nr;
  }
  realPart = fabs(data[2 * k]) + fabs(data[2 * nn - 2 * k]);
  imagPart = fabs(data[2 * k + 1]) + fabs(data[2 * nn - 2 * k + 1]);
  return sqrt(realPart * realPart + imagPart * imagPart) / nr;
}

// função para calcular a amplitude do sinal na frequência desejada
float fftMagdB(double data[], unsigned long nn, unsigned long k, float fullScale)
{
  float magnitude = fftMagnitude(data, nn, k);
  if (magnitude < 0.0000000001)
  {              // less than 0.1 nV
    return -200; // out of range
  }
  return 20.0 * log10(magnitude / fullScale);
}

// função para retornar o ângulo da fase do sinal na frequência estabelecida
float fftPhase(double data[], unsigned long nn, unsigned long k)
{
  if (k >= nn / 2)
  {
    return 0.0; // out of range
  }
  if (data[2 * k + 1] == 0.0)
  {
    return 0.0; // imaginary part is zero
  }
  if (data[2 * k] == 0.0)
  { // real part is zero
    if (data[2 * k + 1] > 0.0)
    {                                           // real part is zero
      return 1.5707963267948966192313216916398; // 90 degrees
    }
    else
    {
      return -1.5707963267948966192313216916398; // -90 degrees
    }
  }
  return atan2(data[2 * k + 1], data[2 * k]); // imaginary part over real part
}

// função para calcular o ângulo de fase do sinal na frequência desejada
float fftFrequency(unsigned long nn, unsigned long k, float fs)
{
  if (k >= nn)
  {
    return 0.0; // out of range
  }
  if (k <= nn / 2)
  {
    return fs * (float)k / (float)nn;
  }
  return -fs * (float)(nn - k) / (float)nn;
}

// função do filtro do sinal de tensão
uint32_t readADC_Avg(int ADC_Raw)
{
  int i = 0;
  uint32_t Sum = 0;

  AN_Pot1_Buffer[AN_Pot1_i++] = ADC_Raw;
  if (AN_Pot1_i == FILTER_LEN)
  {
    AN_Pot1_i = 0;
  }
  for (i = 0; i < FILTER_LEN; i++)
  {
    Sum += AN_Pot1_Buffer[i];
  }
  return (Sum / FILTER_LEN);
}

// função do filtro do sinal de corrente
uint32_t i_readADC_Avg(int ADC_Raw)
{
  int i = 0;
  uint32_t Sum = 0;

  i_AN_Pot1_Buffer[i_AN_Pot1_i++] = ADC_Raw;
  if (i_AN_Pot1_i == FILTER_LEN_CURR)
  {
    i_AN_Pot1_i = 0;
  }
  for (i = 0; i < FILTER_LEN_CURR; i++)
  {
    Sum += i_AN_Pot1_Buffer[i];
  }
  return (Sum / FILTER_LEN_CURR);
}

/* média móvel */
void maf(sMAF *m_a_f, double m_input, int n_samples)
{
  static int i; // counter for MAF variables initialization        //

  if (m_a_f->inicio == 0)
  {
    m_a_f->MAF_KONTz = 0;
    m_a_f->MAF_input = 0;
    for (i = 0; i < CPT_COLETA; i++)
    {
      m_a_f->vector_jm[i] = 0;
    }
    m_a_f->MAF_summedz = 0;
    m_a_f->mean_out = 0;
    m_a_f->inicio = 1;
  }

  if (m_a_f->MAF_KONTz == n_samples)
    m_a_f->MAF_KONTz = 0;

  m_a_f->MAF_input = m_input; // input signal

  m_a_f->MAF_summedz += m_a_f->MAF_input - m_a_f->vector_jm[m_a_f->MAF_KONTz];
  m_a_f->vector_jm[m_a_f->MAF_KONTz] = m_a_f->MAF_input;
  m_a_f->MAF_KONTz = m_a_f->MAF_KONTz + 1;

  m_a_f->mean_out = m_a_f->MAF_summedz / n_samples; // output mean value
}

/* cálculo RMS */
void rms(RMS *r_m_s, double RMS_input, int n_samples)
{
  int i; // counter for vector initialization

  if (r_m_s->inicio == 0)
  {
    r_m_s->temp = 0;
    r_m_s->NA = 0;
    for (i = 0; i < CPT_COLETA; i++)
    {
      r_m_s->jm[i] = 0;
    }

    r_m_s->summed = 0;
    r_m_s->inicio = 1;
  }

  r_m_s->temp = RMS_input;

  r_m_s->summed = r_m_s->summed + r_m_s->temp - r_m_s->jm[r_m_s->NA];
  r_m_s->jm[r_m_s->NA] = r_m_s->temp;

  r_m_s->NA++;

  if (r_m_s->NA >= n_samples)
  {
    r_m_s->NA = 0;
  }

  if (r_m_s->summed <= 0)
    r_m_s->summed = 0.0001;
  r_m_s->v_RMS = sqrt(r_m_s->summed / n_samples);
}

/* integral imparcial da CPT */
void ui(UI *ui, sMAF *av, double ui_input)
{
  if (ui->inicio == 0)
  {
    ui->integral = 0;
    ui->integral_old = 0;
    ui->out = 0;
    ui->inicio = 1;
  }
  //.... Integrator ....//
  ui->integral += (TSH / 2) * (ui_input + ui->integral_old);
  ui->integral_old = ui_input;

  //.... Moving Average for unbiased integral ....//
  maf(av, ui->integral, CPT_COLETA);

  //.... Unbiased Integral (eq 1.d)....//
  ui->out = ui->integral - av->mean_out;
}

/* aqui calcula-se as parcelas de potência e indicadores da CPT */
void CPT(double ua, double ia)
{
  //.... Unbiased Voltage Integral (eq 1.d)....//
  ui(&ui_ua, &mui_ua, ua);
  //.... End ....//

  //.... RMS values for voltages and currents ....//
  rms(&Ua, ua * ua, CPT_COLETA);
  rms(&UI_ua, ui_ua.out * ui_ua.out, CPT_COLETA);
  rms(&Ia, ia * ia, CPT_COLETA);

  // Collective values (eq 1.f)
  rms(&U, ua * ua, CPT_COLETA);
  rms(&I, ia * ia, CPT_COLETA);
  rms(&UI_u, ui_ua.out * ui_ua.out, CPT_COLETA);

  //.... Average of Active Power ....//
  maf(&Pa, ua * ia, CPT_COLETA);
  maf(&P, ua * ia, CPT_COLETA); // eq 5.a
  if (P.mean_out < 0.000001)
    P.mean_out = 0.000001;

  //.... Average of Reactive Energy ....//
  maf(&Wa, ui_ua.out * ia, CPT_COLETA);
  maf(&W, ui_ua.out * ia, CPT_COLETA); // eq 5.b

  //.... Average of Reactive Power (according to eq 20.b) ....//
  Qa = Wa.mean_out * (Ua.v_RMS / UI_ua.v_RMS);

  // .... Current Decomposition ....//
  //.... Active Current by phase (eq. 10.a) ....//
  iaa = (Pa.mean_out / (Ua.v_RMS * Ua.v_RMS)) * ua;

  //.... Balanced Active Current by phase (eq. 14.a) ....//
  ibaa = (P.mean_out / (U.v_RMS * U.v_RMS)) * ua;

  //.... Unbalanced Active Current (eq. 16.a) ....//
  iuaa = iaa - ibaa;

  //.... Reactive Current by phase (11.a) ....//
  ira = (Wa.mean_out / (UI_ua.v_RMS * UI_ua.v_RMS)) * ui_ua.out;

  //.... Reactive Current by phase (eq 15.a) ....//
  ibra = (W.mean_out / (UI_u.v_RMS * UI_u.v_RMS)) * ui_ua.out;

  //.... Unbalance Reactive Current (eq. 17.a) ....//
  iura = ira - ibra;

  //.... Void Current (eq. 12.a) ....//
  iva = ia - iaa - ira;

  //.... Unbalance Current ....//
  iua = iuaa + iura; // according to the sum of eq 18.a

  //.... Non-active Current ....//
  inaa = ia - ibaa;

  // .... rms values for Current components ....//
  rms(&Iba, ibaa * ibaa, CPT_COLETA);

  rms(&Iua, iuaa * iuaa, CPT_COLETA);

  rms(&Ibr, ibra * ibra, CPT_COLETA);

  rms(&Iur, iura * iura, CPT_COLETA);

  Iu = sqrt((Iua.v_RMS * Iua.v_RMS) + (Iur.v_RMS * Iur.v_RMS)); // according to the sum of eq 18.b

  rms(&Iv, iva * iva, CPT_COLETA);

  //.... Power Decomposition ....//
  // Apparent Power (eq. 6.a)
  A = U.v_RMS * I.v_RMS;

  // Reactive Power (eq. 20.a)
  Q = U.v_RMS * Ibr.v_RMS;

  // Active Unbalance Power (eq 19)
  Na = U.v_RMS * Iua.v_RMS;

  // Reactive Unbalance Power (eq 19)
  Nr = U.v_RMS * Iur.v_RMS;

  // Unbalance Power (eq 19)
  N = sqrt(Na * Na + Nr * Nr);

  // Void Power (eq 19)
  V = U.v_RMS * Iv.v_RMS;

  CPT_Val.A = A;
  CPT_Val.Q = Q;
  CPT_Val.N = N;
  CPT_Val.D = V;
  CPT_Val.P = P.mean_out;

  // conformity factors
  // Power Factor
  CPT_Val.PF = CPT_Val.P / CPT_Val.A;
  // Reactivity Factor
  CPT_Val.QF = 1 - CPT_Val.Q / CPT_Val.A;
  // Nonlinearity Factor
  CPT_Val.VF = 1 - CPT_Val.D / CPT_Val.A;
}

//*********************************************//
// TAREFAS - FREERTOS
//*********************************************//

void httpRequestTask(void *pvParameters)
{
  while (1)
  {
    WiFiClient client;

    Serial.println("Connecting to server...");
    if (client.connect("192.168.0.156", 3001))
    {
      Serial.println("Connected to server");

      // Use HTTPS in production
      HTTPClient http;
      http.begin(client, "http://192.168.0.156", 3001, "/api/teste");
      int httpResponseCode = http.GET();

      if (httpResponseCode > 0)
      {
        String response = http.getString();
        Serial.println(httpResponseCode);
        Serial.println(response);
      }
      else
      {
        Serial.println("Error on HTTP request");
        Serial.println(httpResponseCode);
      }

      http.end();
      client.stop();
      Serial.println("Connection closed");
    }
    else
    {
      Serial.println("Connection failed");
    }

    // Wait for 3 seconds before making the next request
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}

// Tarefa do terminal serial (doCLI)
void doCLI(void *parameters)
{
  Message rcv_msg;
  char c;
  char cmd_buf[CMD_BUF_LEN];
  uint8_t idx = 0;
  double rms = 0.0;
  double i_rms = 0.0;
  double p_rms = 0.0;
  double sum_signal = 0.0;
  double val_signal = 0.0;
  double pot_sig = 0.0;

  // limpar o buffer completo
  memset(cmd_buf, 0, CMD_BUF_LEN);
  while (1)
  {
    // Observar por qualquer erro de mensagem que deve ser apresentado
    if (xQueueReceive(msg_queue, (void *)&rcv_msg, 0) == pdTRUE)
    {
      Serial.println(rcv_msg.body);
    }
    // Leia caracteres da comunicação serial
    if (Serial.available() > 0)
    {
      c = Serial.read();
      // Armazena o caractere no buffer se não estiver no fim
      if (idx < CMD_BUF_LEN - 1)
      {
        cmd_buf[idx] = c;
        idx++;
      }
      // imprimir uma nova linha e ver se não está já no final
      if ((c == '\n') || (c == '\r'))
      {
        // imprimir nova linha no terminal serial
        Serial.print("\r\n");
        cmd_buf[idx - 1] = '\0';
        // valor médio do sinal da tensão (ADC)
        if (strcmp(cmd_buf, v_command1) == 0)
        {
          Serial.println(adc_avg);
        }
        // FFT do sinal da tensão
        if (strcmp(cmd_buf, v_command2) == 0)
        {
          fft(vComp, SAMPLES);
          for (int ord = 0; ord < (SAMPLES / 4); ord++)
          {
            Serial.print(ord);
            Serial.print(" ");
            Serial.println(fftMagnitude(vComp, SAMPLES, ord));
          }
        }
        // sinal instantâneo da tensão
        if (strcmp(cmd_buf, v_command3) == 0)
        {
          for (int aux = 0; aux < 5; aux++)
          {
            for (int i = 0; i < 2 * SAMPLES; i++)
            {
              if (i % 2 == 0)
                Serial.println((float)vComp[i]);
            }
          }
        }
        // sinal da tensão vinda do ADC
        if (strcmp(cmd_buf, v_command4) == 0)
        {
          for (int i = 0; i < BUF_LEN - 10; i++)
          {
            Serial.println((float)read_from[i]);
          }
        }
        // valor eficaz da tensão
        if (strcmp(cmd_buf, v_command5) == 0)
        {
          sum_signal = 0;
          for (int aux = 0; aux < 5; aux++)
          {
            for (int i = 0; i < 2 * SAMPLES; i++)
            {
              val_signal = ((float)vComp[i]);
              if (i % 2 == 0)
                sum_signal += val_signal * val_signal;
            }
          }
          rms = sqrt(sum_signal / (SAMPLES * 5));
          Vrms = rms;
          Serial.println((double)rms);
        }
        // valor médio da corrente (ADC)
        if (strcmp(cmd_buf, i_command1) == 0)
        {
          Serial.println(i_adc_avg);
        }
        // fft da corrente
        if (strcmp(cmd_buf, i_command2) == 0)
        {
          fft(iComp, SAMPLES);
          for (int ord = 0; ord < (SAMPLES / 4); ord++)
          {
            Serial.print(ord);
            Serial.print(" ");
            Serial.println(fftMagnitude(iComp, SAMPLES, ord));
          }
        }
        // sinal instantâneo da corrente
        if (strcmp(cmd_buf, i_command3) == 0)
        {
          for (int aux = 0; aux < 5; aux++)
          {
            for (int i = 0; i < 2 * SAMPLES; i++)
            {
              if (i % 2 == 0)
                Serial.println((float)iComp[i]);
            }
          }
        }
        // valor instantâneo da corrente vinda do ADC
        if (strcmp(cmd_buf, i_command4) == 0)
        {
          for (int i = 0; i < BUF_LEN - 10; i++)
          {
            Serial.println((float)i_read_from[i]);
          }
        }
        // valor eficaz da corrente
        if (strcmp(cmd_buf, i_command5) == 0)
        {
          sum_signal = 0;
          for (int aux = 0; aux < 5; aux++)
          {
            for (int i = 0; i < 2 * SAMPLES; i++)
            {
              val_signal = ((float)iComp[i]);
              if (i % 2 == 0)
                sum_signal += val_signal * val_signal;
            }
          }
          i_rms = sqrt(sum_signal / (SAMPLES * 5));
          Irms = (double)i_rms;
          Serial.println((double)i_rms);
        }
        // um ciclo do sinal instantâneo da tensão e corrente
        if (strcmp(cmd_buf, vi_command1) == 0)
        {
          for (int i = 0; i < 2 * SAMPLES; i++)
          {
            if (i % 2 == 0)
            {
              Serial.print((float)vComp[i]);
              Serial.print(" ");
              Serial.println((float)iComp[i]);
            }
          }
        }
        // sinal instantâneo da tensão e corrente
        if (strcmp(cmd_buf, vi_command2) == 0)
        {
          for (int aux = 0; aux < 5; aux++)
          {
            for (int i = 0; i < 2 * SAMPLES; i++)
            {
              if (i % 2 == 0)
              {
                Serial.print((float)vComp[i]);
                Serial.print(" ");
                Serial.println((float)iComp[i]);
              }
            }
          }
        }
        // potência instantânea
        if (strcmp(cmd_buf, p_command1) == 0)
        {
          for (int aux = 0; aux < 5; aux++)
          {
            for (int i = 0; i < 2 * SAMPLES; i++)
            {
              if (i % 2 == 0)
              {
                Serial.println(((float)vComp[i]) * (float)iComp[i]);
              }
            }
          }
        }
        // semi-ciclo da tensão
        if (strcmp(cmd_buf, etc_command) == 0)
        {
          for (int aux = 0; aux < 5; aux++)
          {
            for (int i = 0; i < 2 * SAMPLES; i++)
            {
              if (i % 2 == 0)
                Serial.println((float)abs(vComp[i]));
            }
          }
        }
        // semi-ciclo da corrente
        if (strcmp(cmd_buf, etc_command1) == 0)
        {
          for (int aux = 0; aux < 5; aux++)
          {
            for (int i = 0; i < 2 * SAMPLES; i++)
            {
              if (i % 2 == 0)
                Serial.println((float)abs(iComp[i]));
            }
          }
        }
        // cálculo da cpt
        if (strcmp(cmd_buf, cpt_command1) == 0)
        {
          int kk = 0;
          for (int j = 0; j < 3; j++)
          {
            kk = 0;
            for (int i = 0; i < 2 * SAMPLES; i++)
            {
              if (i % 2 == 0)
              {
                tensao_instantanea[j * SAMPLES + kk] = vComp[i];
                corrente_instantanea[j * SAMPLES + kk] = iComp[i];
                CPT(tensao_instantanea[j * SAMPLES + kk], corrente_instantanea[j * SAMPLES + kk]);
                kk++;
              }
            }
          }
          Serial.print("A = ");
          Serial.println(CPT_Val.A);
          Serial.print("Q = ");
          Serial.println(CPT_Val.Q);
          Serial.print("N = ");
          Serial.println(CPT_Val.N);
          Serial.print("V = ");
          Serial.println(CPT_Val.D);
          Serial.print("P = ");
          Serial.println(CPT_Val.P);
          Serial.print("PF = ");
          Serial.println(CPT_Val.PF);
          Serial.print("QF = ");
          Serial.println(CPT_Val.QF);
          Serial.print("VF = ");
          Serial.println(CPT_Val.VF);
        }
        // indicadores de QEE
        if (strcmp(cmd_buf, qee_command1) == 0)
        {
          // tensão
          sum_signal = 0;
          for (int aux = 0; aux < 5; aux++)
          {
            for (int i = 0; i < 2 * SAMPLES; i++)
            {
              val_signal = ((float)vComp[i]);
              if (i % 2 == 0)
                sum_signal += val_signal * val_signal;
            }
          }
          Vrms = sqrt(sum_signal / (SAMPLES * 5));
          // corrente
          sum_signal = 0;
          for (int aux = 0; aux < 5; aux++)
          {
            for (int i = 0; i < 2 * SAMPLES; i++)
            {
              val_signal = ((float)iComp[i]);
              if (i % 2 == 0)
                sum_signal += val_signal * val_signal;
            }
          }
          Irms = sqrt(sum_signal / (SAMPLES * 5));

          Serial.print("Corrente eficaz = ");
          Serial.println(Irms);
          Serial.print("Tensão eficaz = ");
          Serial.println(Vrms);

          int kk = 0;
          for (int j = 0; j < 3; j++)
          {
            kk = 0;
            for (int i = 0; i < 2 * SAMPLES; i++)
            {
              if (i % 2 == 0)
              {
                tensao_instantanea[j * SAMPLES + kk] = vComp[i];
                corrente_instantanea[j * SAMPLES + kk] = iComp[i];
                CPT(tensao_instantanea[j * SAMPLES + kk], corrente_instantanea[j * SAMPLES + kk]);
                kk++;
              }
            }
          }

          Serial.print("Potência Aparente = ");
          Serial.println(CPT_Val.A);
          Serial.print("Potência Ativa = ");
          Serial.println(CPT_Val.P);
          Serial.print("Potência Reativa = ");
          Serial.println(CPT_Val.Q);
          Serial.print("Potência de Desbalanço = ");
          Serial.println(CPT_Val.N);
          Serial.print("Potência Residual (não linear) = ");
          Serial.println(CPT_Val.D);

          FP = CPT_Val.P / CPT_Val.A;
          Serial.print("Fator de potência = ");
          Serial.println((double)FP);

          fft(vComp, SAMPLES);
          DHTV = 100 * (fftMagnitude(vComp, SAMPLES, 3) + fftMagnitude(vComp, SAMPLES, 5) + fftMagnitude(vComp, SAMPLES, 7) + fftMagnitude(vComp, SAMPLES, 9) + fftMagnitude(vComp, SAMPLES, 11) + fftMagnitude(vComp, SAMPLES, 13) + fftMagnitude(vComp, SAMPLES, 15) + fftMagnitude(vComp, SAMPLES, 17) + fftMagnitude(vComp, SAMPLES, 19) + fftMagnitude(vComp, SAMPLES, 21) + fftMagnitude(vComp, SAMPLES, 23) + fftMagnitude(vComp, SAMPLES, 25)) / fftMagnitude(vComp, SAMPLES, 1);
          fft(iComp, SAMPLES);
          DHTI = 100 * (fftMagnitude(iComp, SAMPLES, 3) + fftMagnitude(iComp, SAMPLES, 5) + fftMagnitude(iComp, SAMPLES, 7) + fftMagnitude(iComp, SAMPLES, 9) + fftMagnitude(iComp, SAMPLES, 11) + fftMagnitude(iComp, SAMPLES, 13) + fftMagnitude(iComp, SAMPLES, 15) + fftMagnitude(iComp, SAMPLES, 17) + fftMagnitude(iComp, SAMPLES, 19) + fftMagnitude(iComp, SAMPLES, 21) + fftMagnitude(iComp, SAMPLES, 23) + fftMagnitude(iComp, SAMPLES, 25)) / fftMagnitude(iComp, SAMPLES, 1);
          Serial.print("DHTv = ");
          Serial.print((double)DHTV);
          Serial.println("%");
          Serial.print("DHTi = ");
          Serial.print((double)DHTI);
          Serial.println("%");
        }
        // reinicia o buffer de caracteres e o índice
        memset(cmd_buf, 0, CMD_BUF_LEN);
        idx = 0;
        // Caso contrário, imprima só o que foi digitado, mesmo que incorretamente
      }
      else
      {
        Serial.print(c);
      }
    }
    // Não seja egoísta, deixe a CPU ser usada pelos outros
    vTaskDelay(cli_delay / portTICK_PERIOD_MS);
  }
}

// tarefa - esperando pelo semáforo, seta o timer 0 e calculando a média de tensão
void calcAverage(void *parameters)
{
  Message msg;
  float avg;
  int cont;
  // iniciar um timer a cada 130,2 us
  timer = timerBegin(0, timer_divider, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, timer_max_count, true);
  timerAlarmEnable(timer);
  // Loop eterno, aguarda pelo semáforo e imprime o valor
  while (1)
  {
    // Aguardar por notificação vinda da ISR
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // Calcula a média (valor float)
    avg = 0.0;
    for (int i = 0; i < BUF_LEN; i++)
    {
      avg += (float)read_from[i];
    }
    avg /= BUF_LEN;
    cont = 0;
    for (int j = 0; j < SAMPLES * 2; j++)
    {
      if (j % 2 == 0)
      {
        vComp[j] = ((((float)read_from[cont] + (float)read_from[cont + SAMPLES] + (float)read_from[cont + 2 * SAMPLES] + (float)read_from[cont + 3 * SAMPLES]) - 4 * adc_avg) / 4) * 0.275;
        cont++;
      }
      else
        vComp[j] = 0;
    }
    // por ser um processo crítico o envio desta informação no contexto global,
    // ela é travada e depois retorna
    portENTER_CRITICAL(&spinlock);
    adc_avg = avg;
    portEXIT_CRITICAL(&spinlock);
    // se levar muito tempo para processar, overrun será impresso
    if (buf_overrun == 1)
    {
      strcpy(msg.body, "Erro @voltage: Buffer overrun. Amostras foram removidas.");
      xQueueSend(msg_queue, (void *)&msg, 10);
    }
    // Limpando o overrun e devolvendo o semáforo de leitura.
    portENTER_CRITICAL(&spinlock);
    buf_overrun = 0;
    xSemaphoreGive(sem_done_reading);
    portEXIT_CRITICAL(&spinlock);
  }
}

// tarefa que espera pelo semáforo, seta o timer 1 e calculando a média da corrente
void icalcAverage(void *parameters)
{
  Message msg;
  float avg;
  int cont;
  // iniciar um timer a cada 130,2 us
  itimer = timerBegin(1, timer_divider, true);
  timerAttachInterrupt(itimer, &onTimerI, true);
  timerAlarmWrite(itimer, timer_max_count, true);
  timerAlarmEnable(itimer);
  // Loop eterno, aguarda pelo semáforo e imprime o valor
  while (1)
  {
    // Aguardar por notificação vinda da ISR
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // Calcula a média (valor float)
    avg = 0.0;
    for (int i = 0; i < BUF_LEN; i++)
    {
      avg += (float)i_read_from[i];
    }
    avg /= BUF_LEN;
    cont = 0;
    for (int j = 0; j < SAMPLES * 2; j++)
    {
      if (j % 2 == 0)
      {
        iComp[j] = ((((float)i_read_from[cont] + (float)i_read_from[cont + SAMPLES] + (float)i_read_from[cont + 2 * SAMPLES] + (float)i_read_from[cont + 3 * SAMPLES]) - 4 * i_adc_avg) / 4) * 0.025;
        cont++;
      }
      else
        iComp[j] = 0;
    }
    // por ser um processo crítico o envio desta informação no contexto global,
    // ela é travada e depois retorna
    portENTER_CRITICAL(&spinlock);
    i_adc_avg = avg;
    portEXIT_CRITICAL(&spinlock);
    // se levar muito tempo para processar, overrun será impresso
    if (i_buf_overrun == 1)
    {
      strcpy(msg.body, "Erro @current: Buffer overrun. Amostras foram removidas.");
      xQueueSend(msg_queue, (void *)&msg, 10);
    }
    // Limpando o overrun e devolvendo o semáforo de leitura.
    portENTER_CRITICAL(&spinlock);
    i_buf_overrun = 0;
    xSemaphoreGive(i_sem_done_reading);
    portEXIT_CRITICAL(&spinlock);
  }
}

//*****************************************************************************
// Main (rodando neste caso no core 1, prioridade 1 - automático)

void setup()
{
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
  // Configurando a Serial
  Serial.begin(115200);
  // Aguarde um momento para começar (por conta da inicialização da Serial)
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("---FreeRTOS---");

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());

  // criando o semáforo de leitura (ISR)
  sem_done_reading = xSemaphoreCreateBinary();
  // Força a reinicialização se o semáforo não for criado
  if (sem_done_reading == NULL)
  {
    Serial.println("Não foi possível criar um ou mais semáforos");
    ESP.restart();
  }
  i_sem_done_reading = xSemaphoreCreateBinary();
  // Força a reinicialização se o semáforo não for criado
  if (i_sem_done_reading == NULL)
  {
    Serial.println("Não foi possível criar um ou mais semáforos");
    ESP.restart();
  }
  // iniciando o semáforo como 1
  xSemaphoreGive(sem_done_reading);
  xSemaphoreGive(i_sem_done_reading);
  // Criando a fila, antes de ser usada na mensagem
  msg_queue = xQueueCreate(MSG_QUEUE_LEN, sizeof(Message));

  // Tarefa que recebe do client os comandos de interesse e faz o comando.
  xTaskCreatePinnedToCore(doCLI, "Interface do Cliente",
                          2048, NULL, 1,
                          NULL, app_cpu);
  // Tarefa que trata da aquisição da tensão
  xTaskCreatePinnedToCore(calcAverage, "Tensão",
                          8000, NULL, 2,
                          &processing_task, pro_cpu);
  // Tarefa que trata da aquisição da corrente
  xTaskCreatePinnedToCore(icalcAverage, "Corrente",
                          8000, NULL, 2,
                          &i_processing_task, pro_cpu);
  // Create the HTTP request task
  xTaskCreatePinnedToCore(httpRequestTask, "HTTP Request Task",
                          4096, NULL, 1,
                          NULL, app_cpu);

  vTaskDelete(NULL);
}

void loop()
{
  // não deve ter nada por aqui!!!!!
}
