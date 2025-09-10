/**
* @file pendulum-controller-smooth.ino
* @brief Tu código modificado para movimiento suave estilo Shawn Hymel
* @author Modificado para control suave
* @date 2025-01-08
*
* @details
* CAMBIOS PRINCIPALES:
* 1. Eliminado control de velocidad continua
* 2. Implementado control de posición incremental como Shawn
* 3. Corregido el signo del control para dirección correcta
* 4. Simplificado el loop principal
*/
#include "RotaryEncoder.h"
#include "L6474.h"
#include "control-comms.hpp"
#include <math.h>

/******************************************************************************
* Constantes y globales (SIN CAMBIOS)
*/
// Definición de pines
const int LED_PIN = LED_BUILTIN;
const int ENC_A_PIN = D4; // Cable verde  - Encoder (péndulo sensor)
const int ENC_B_PIN = D5; // Cable blanco - Encoder (péndulo sensor)

// Stepper motor pines
const int STP_FLAG_IRQ_PIN = D2;
const int STP_STBY_RST_PIN = D8;
const int STP_DIR_PIN = D7;
const int STP_PWM_PIN = D9;
const int8_t STP_SPI_CS_PIN = D10;
const int8_t STP_SPI_MOSI_PIN = D11;
const int8_t STP_SPI_MISO_PIN = D12;
const int8_t STP_SPI_SCK_PIN = D13;

// Constantes de comunicación
static const unsigned int BAUD_RATE = 500000;
static const ControlComms::DebugLevel CTRL_DEBUG = ControlComms::DEBUG_ERROR;
static constexpr size_t NUM_ACTIONS = 6;
static constexpr size_t NUM_OBS = 6;

// Definición de comandos
static const unsigned int CMD_SET_HOME = 0;
static const unsigned int CMD_MOVE_TO = 1;
static const unsigned int CMD_MOVE_BY = 2;
static const unsigned int CMD_SET_STEP_MODE = 3;
static const unsigned int CMD_SELECT_CONTROLLER = 4;
static const unsigned int CMD_SET_PID_GAINS = 5;
static const unsigned int CMD_SET_LQR_GAINS = 6;
static const unsigned int CMD_START_CONTROL = 7;
static const unsigned int CMD_STOP_CONTROL = 8;

// Status codes
static const unsigned int STATUS_OK = 0;
static const unsigned int STATUS_STP_MOVING = 1;
static const unsigned int STATUS_CONTROL_ACTIVE = 2;
static const unsigned int STATUS_UPRIGHT_ACHIEVED = 3;
static const unsigned int STATUS_OUT_OF_RANGE = 4;

// Tiempo de muestreo 
const float SAMPLE_TIME = 0.02; // 20ms sampling time

// Constantes del Encoder y Stepper
const int ENC_STEPS_PER_ROTATION = 1200;
const int STP_STEPS_PER_ROTATION = 200;

// Constantes de conversión
const float deg_rad = PI/180.0;
const float rad_deg = 180.0/PI;

// Parametros de control 
const float CONTROL_ACTIVATION_RANGE = 10.0 *deg_rad; // ±10° desde 180° para ACTIVAR control
const float UPRIGHT_THRESHOLD = 15.0*deg_rad; // degrees from 180
const float SETPOINT = 180.0*deg_rad; // Posición invertida del péndulo

// Parámetros para manejo de pérdida de control
const float CONTROL_DEACTIVATION_RANGE = 25.0*deg_rad; // ±25° desde 180° para DESACTIVAR control
const unsigned long MAX_OUT_OF_RANGE_TIME = 2000; // 2 segundos máximo fuera de rango

// Selección del controlador
typedef enum {
  CONTROLLER_PID = 0,
  CONTROLLER_LQR = 1
} ControllerType;

// Estados del sistema
typedef enum {
  STATE_IDLE = 0,              // Sistema inactivo
  STATE_CONTROL = 1,           // Control activo
  STATE_STOPPED = 2,           // Sistema detenido
  STATE_WAITING_FOR_RANGE = 3  // Esperando que el péndulo entre en rango
} SystemState;

// variables Globales
RotaryEncoder *encoder = nullptr;
SPIClass dev_spi(STP_SPI_MOSI_PIN, STP_SPI_MISO_PIN, STP_SPI_SCK_PIN);
L6474 *stepper;
ControlComms ctrl;
unsigned int div_per_step = 16;

// Variables de control
ControllerType current_controller = CONTROLLER_PID;
SystemState system_state = STATE_IDLE;

// PID ganacias
float K_p = 0.0, K_i = 0.0, K_d = 0.0;
float pid_error_prev = 0.0, pid_integral = 0.0;

// LQR ganancias
float K_lqr[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

// Variables para los observadores
float phi = 0.0; // Rotor angle (degrees)
float phi_prev = 0.0;
float dphi = 0.0; // Rotor angular velocity
float theta = 0.0; // Pendulum angle (degrees)
float theta_prev = 0.0;
float dtheta = 0.0; // Pendulum angular velocity
float integral_error = 0.0;

// ============================================================================
// NUEVAS VARIABLES PARA CONTROL SUAVE ESTILO SHAWN
// ============================================================================
float accumulated_control = 0.0;        // Control acumulado para movimiento continuo
float phi_reference = 0.0;              // Referencia de posición del rotor
unsigned long last_control_application = 0; // Último tiempo de aplicación de control
const unsigned long CONTROL_APPLICATION_INTERVAL = 15; // 15ms entre aplicaciones

// variables para control de rango
unsigned long out_of_range_start_time = 0;
bool was_in_range = false;

// variables temporales
unsigned long current_time = 0;
unsigned long prev_time = 0;
unsigned long control_loop_timer = 0;
const unsigned long CONTROL_INTERVAL = 20; // 20ms = 50Hz

// variables Debug
unsigned long last_debug_print = 0;
const unsigned long DEBUG_INTERVAL = 500; // Print debug cada 500ms

// Configuración del Stepper (optimizada para suavidad)
L6474_init_t stepper_config_control = {
  8000,                              // Acceleration: REDUCIDA para suavidad
  8000,                              // Deceleration: REDUCIDA para suavidad  
  5000,                              // Maximum speed: REDUCIDA
  1000,                                // Minimum speed: BAJA
  300,                               // Torque: REDUCIDO para menos vibraciones
  L6474_OCD_TH_750mA,               
  L6474_CONFIG_OC_SD_ENABLE,        
  L6474_CONFIG_EN_TQREG_TVAL_USED,  
  L6474_STEP_SEL_1_16,               // Máxima resolución para suavidad
  L6474_SYNC_SEL_1_2,               
  L6474_FAST_STEP_12us,             
  L6474_TOFF_FAST_8us,              
  3,                                
  21,                               
  L6474_CONFIG_TOFF_044us,          
  L6474_CONFIG_SR_320V_us,           // Slew rate MÁS BAJO para suavidad
  L6474_CONFIG_INT_16MHZ,           
  L6474_ALARM_EN_OVERCURRENT |
  L6474_ALARM_EN_THERMAL_SHUTDOWN |
  L6474_ALARM_EN_THERMAL_WARNING |
  L6474_ALARM_EN_UNDERVOLTAGE |
  L6474_ALARM_EN_SW_TURN_ON |
  L6474_ALARM_EN_WRONG_NPERF_CMD    
};

/******************************************************************************
* Rutinia de interrupción (SIN CAMBIOS)
*/
void stepperISR(void) {
  stepper->isr_flag = TRUE;
  unsigned int status = stepper->get_status();
  if ((status & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD) {
    // Serial.println("WARNING: FLAG interrupt triggered.");
  }
  stepper->isr_flag = FALSE;
}

void encoderISR() {
  encoder->tick();
}

/******************************************************************************
* Funciones de utilidad (SIN CAMBIOS)
*/
// Ángulo del Encoder
float get_encoder_angle() {
  int pos = encoder->getPosition();
  pos = pos % ENC_STEPS_PER_ROTATION;
  pos = pos >= 0 ? pos : pos + ENC_STEPS_PER_ROTATION;
  float angle_deg = (float)pos * (360.0 / ENC_STEPS_PER_ROTATION);
  return angle_deg*deg_rad;
}

// Ángulo del Stepper
float get_stepper_angle() {
  int pos = stepper->get_position();
  pos = pos % (STP_STEPS_PER_ROTATION * div_per_step);
  pos = pos >= 0 ? pos : pos + (STP_STEPS_PER_ROTATION * div_per_step);
  float angle_deg = (float)pos * (360.0 / (STP_STEPS_PER_ROTATION * div_per_step));
  return angle_deg*deg_rad;
}

// Función para normalizar ángulos en el rango [-π, π]
float normalize_angle(float angle) {
  while (angle > PI) angle -= 2.0 * PI;
  while (angle < -PI) angle += 2.0 * PI;
  return angle;
}

// Posición de referencia en cero
void set_stepper_home() {
  stepper->set_home();
  phi_reference = 0.0;
  Serial.println("Stepper home position set");
}

void move_stepper_to(float rad) {
  float deg = rad*rad_deg;
  int steps = (int)(deg * STP_STEPS_PER_ROTATION * div_per_step / 360.0);
  stepper->go_to(steps);
}

void move_stepper_by(float rad) {
  float deg = rad*rad_deg;
  StepperMotor::direction_t stp_dir = StepperMotor::FWD;
  int steps = (int)(deg * STP_STEPS_PER_ROTATION * div_per_step / 360.0);
  if (steps < 0) {
    steps = -1 * steps;
    stp_dir = StepperMotor::BWD;
  }
  stepper->move(stp_dir, steps);
}

// Número de pasos para el stepper
void set_step_mode(int mode) {
  switch (mode) {
    case 0: stepper->set_step_mode(StepperMotor::STEP_MODE_FULL); div_per_step = 1; break;
    case 1: stepper->set_step_mode(StepperMotor::STEP_MODE_HALF); div_per_step = 2; break;
    case 2: stepper->set_step_mode(StepperMotor::STEP_MODE_1_4); div_per_step = 4; break;
    case 3: stepper->set_step_mode(StepperMotor::STEP_MODE_1_8); div_per_step = 8; break;
    case 4: stepper->set_step_mode(StepperMotor::STEP_MODE_1_16); div_per_step = 16; break;
    default: break;
  }
}

/******************************************************************************
* funciones para verificar rango de control (SIN CAMBIOS)
*/
// Función para calcular distancia angular más corta desde 180°
float calculate_distance_from_180(float angle) {
  float normalized_angle = normalize_angle(angle);
  float distance = abs(normalized_angle - PI); 
  if (distance > PI) {
    distance = (2*PI - distance);
  }
  return distance;
}

// Verificar si el péndulo está en rango para ACTIVAR control
bool is_in_control_activation_range() {
  float distance = calculate_distance_from_180(theta);
  return distance <= CONTROL_ACTIVATION_RANGE;
}

// Verificar si el péndulo está en rango para MANTENER control activo
bool is_in_control_maintenance_range() {
  float distance = calculate_distance_from_180(theta);
  return distance <= CONTROL_DEACTIVATION_RANGE;
}

// Verificar si está dentro del umbral estricto de "invertido"
bool is_pendulum_upright() {
  float distance = calculate_distance_from_180(theta);
  return distance <= UPRIGHT_THRESHOLD;
}

/******************************************************************************
* Actualización de las variables del sistema (SIN CAMBIOS)
*/
void update_states() {
  current_time = millis();
  float dt = SAMPLE_TIME;
  //float dt = (current_time - prev_time) / 1000.0;
  //if (dt <= 0.0) dt = SAMPLE_TIME;

  // Actualización de los ángulos
  phi_prev = phi;
  theta_prev = theta;
  phi = get_stepper_angle();
  theta = get_encoder_angle();

  // Normalizar ángulos
  phi = normalize_angle(phi);
  theta = normalize_angle(theta);

  // Calcular las derivadas 
  dphi = (phi - phi_prev) / dt;
  dtheta = (theta - theta_prev) / dt;

  // Limitar las velocidades va evitar picos
  if (abs(dphi) > 174.53) dphi = 0.0;
  if (abs(dtheta) > 174.53) dtheta = 0.0;

  // Actualización del error integral para el control
  float error = SETPOINT - theta;
  integral_error = integral_error + error * dt;

  // Limite del error integral
  if (integral_error > 8.73) integral_error = 8.73;
  if (integral_error < -8.73) integral_error = -8.73;

  prev_time = current_time;
}

/******************************************************************************
* Algoritmos del control (CON CORRECCIÓN DE SIGNO)
*/
// Implementar el PID [CORRECCIÓN APLICADA]
float compute_pid_control() {
  float error = SETPOINT - theta;
  float derivative = (error - pid_error_prev) / SAMPLE_TIME;
  pid_integral = pid_integral + error * SAMPLE_TIME;

  // Evitar saturacion 
  if (pid_integral > 8.73) pid_integral = 8.73;
  if (pid_integral < -8.73) pid_integral = -8.73;

  float output = K_p * error + K_i * pid_integral + K_d * derivative;
  pid_error_prev = error;

  // Limitar la salida
  if (output > PI/2) output = PI/2;
  if (output < -PI/2) output = -PI/2;

  return output;
}

// Implementar el LQR (SIN CAMBIOS)
float compute_lqr_control() {
  // Vector de estado: [phi_error, dphi, theta_error, dtheta, integral_error]
  float phi_error_rad = phi - phi_reference;
  float dphi_rad = dphi;
  float theta_error_rad = theta - SETPOINT;
  float dtheta_rad = dtheta;
  float integral_error_rad = integral_error;

  // LQR Ley de control: u = -K * x
  float control = -(K_lqr[0] * phi_error_rad +
                    K_lqr[1] * dphi_rad +
                    K_lqr[2] * theta_error_rad +
                    K_lqr[3] * dtheta_rad +
                    K_lqr[4] * integral_error_rad);

  // Convertir nuevamente a grados y limitar
  if (control > PI/4) control = PI/4;
  if (control < -PI/4) control = -PI/4;

  return control;
}

/******************************************************************************
* NUEVA FUNCIÓN DE CONTROL SUAVE ESTILO SHAWN
*/
void apply_control_smooth(float control_output) {
  // CLAVE: Aplicar control como pequeños movimientos incrementales
  // en lugar de velocidad continua, como hace Shawn Hymel
  
  unsigned long current_time = millis();
  
  // Solo aplicar control si ha pasado suficiente tiempo Y el stepper está libre
  if ((current_time - last_control_application > CONTROL_APPLICATION_INTERVAL) && 
      (stepper->get_device_state() == INACTIVE)) {
    
    // CORRECCIÓN DEL SIGNO: Invertir la dirección del control
    // Si péndulo cae a la derecha, motor debe moverse a la izquierda
    float corrected_control = -control_output; // ← SIGNO NEGATIVO AÑADIDO
    
    // Acumular el control para permitir movimiento continuo pequeño
    accumulated_control += corrected_control;
    
    // Factor de suavizado para movimientos pequeños y frecuentes
    float movement_factor = 0.15; // Ajustable: 0.05-0.3
    float position_increment = accumulated_control * movement_factor;
    
    // Solo mover si el incremento es significativo (evitar micro-movimientos)
    if (abs(position_increment) > 0.008) { // ~0.46° mínimo
      
      // Aplicar movimiento incremental suave
      move_stepper_by(position_increment);
      
      // Limpiar acumulador después de aplicar
      accumulated_control = 0.0;
      last_control_application = current_time;
      
      // Debug periódico con información del control suave
      if (current_time - last_debug_print > DEBUG_INTERVAL) {
        Serial.print("SMOOTH θ:");
        Serial.print(theta*rad_deg, 1);
        Serial.print("° raw_ctrl:");
        Serial.print(control_output*rad_deg, 2);
        Serial.print("° corrected:");
        Serial.print(corrected_control*rad_deg, 2);
        Serial.print("° move:");
        Serial.print(position_increment*rad_deg, 2);
        Serial.print("° upright:");
        Serial.println(is_pendulum_upright() ? "YES" : "NO");
        last_debug_print = current_time;
      }
    }
  }
}

/******************************************************************************
* función para gestionar transiciones de estado basadas en rango (SIN CAMBIOS)
*/
void manage_control_state_transitions() {
  bool currently_in_range = is_in_control_maintenance_range();
  
  switch (system_state) {
    case STATE_WAITING_FOR_RANGE:
      // Esperando que el péndulo entre en rango de activación
      if (is_in_control_activation_range()) {
        system_state = STATE_CONTROL;
        phi_reference = phi; // Usar posición actual como referencia inicial
        
        // Reset estados del controlador
        pid_integral = 0.0;
        pid_error_prev = 0.0;
        integral_error = 0.0;
        accumulated_control = 0.0; // ← NUEVO: Limpiar control acumulado
        
        out_of_range_start_time = 0;
        was_in_range = true;
        
        Serial.println("*** CONTROL SUAVE ACTIVADO - PÉNDULO EN RANGO ***");
        Serial.print("Distancia desde 180°: ");
        Serial.print(calculate_distance_from_180(theta)*rad_deg, 1);
        Serial.println("°");
      }
      break;
      
    case STATE_CONTROL:
      // Control activo - verificar si sale del rango de mantenimiento
      if (!currently_in_range) {
        if (was_in_range) {
          // Primera vez que sale del rango
          out_of_range_start_time = current_time;
          was_in_range = false;
          Serial.println("ADVERTENCIA: Péndulo fuera del rango de control");
          Serial.print("Distancia desde 180°: ");
          Serial.print(calculate_distance_from_180(theta)*rad_deg, 1);
          Serial.println("°");
        } else {
          // Verificar si ha estado fuera de rango demasiado tiempo
          if (current_time - out_of_range_start_time > MAX_OUT_OF_RANGE_TIME) {
            system_state = STATE_WAITING_FOR_RANGE;
            stepper->hard_stop();
            accumulated_control = 0.0; // ← NUEVO: Limpiar control acumulado
            Serial.println("*** CONTROL DESACTIVADO - FUERA DE RANGO POR MUCHO TIEMPO ***");
            Serial.println("Esperando que el péndulo regrese al rango 180° ± 10°");
          }
        }
      } else {
        // De vuelta en rango
        if (!was_in_range) {
          Serial.println("Péndulo de vuelta en rango de control");
        }
        was_in_range = true;
        out_of_range_start_time = 0;
      }
      break;
      
    default:
      break;
  }
}

/******************************************************************************
* CONTROL LOOP PRINCIPAL MODIFICADO ESTILO SHAWN
*/
void run_control_loop() {
  // Ejecutar el bucle de control en los intervalos especificos
  if (millis() - control_loop_timer < CONTROL_INTERVAL) {
    return;
  }
  control_loop_timer = millis();

  update_states();
  
  // Gestionar transiciones de estado basadas en rango
  manage_control_state_transitions();

  // ============================================================================
  // CONTROL PRINCIPAL MODIFICADO: ESTILO SHAWN CON MOVIMIENTOS INCREMENTALES
  // ============================================================================
  
  if (system_state == STATE_CONTROL && is_in_control_maintenance_range()) {
    
    float control_output = 0.0;
    
    if (current_controller == CONTROLLER_PID) {
      control_output = compute_pid_control();
    } else {
      control_output = compute_lqr_control();
    }

    // APLICAR CONTROL SUAVE ESTILO SHAWN
    // (con corrección de signo incluida)
    apply_control_smooth(control_output);
    debug_control_performance(control_output);

  } else {
    // Fuera de control: limpiar acumuladores
    accumulated_control = 0.0;
    
    // Mensaje cuando está esperando entrar en rango
    if (system_state == STATE_WAITING_FOR_RANGE && 
        current_time - last_debug_print > DEBUG_INTERVAL * 2) {
      Serial.print("ESPERANDO RANGO θ:");
      Serial.print(theta*rad_deg, 1);
      Serial.print("° dist_180:");
      Serial.print(calculate_distance_from_180(theta)*rad_deg, 1);
      Serial.print("° (necesita ≤");
      Serial.print(CONTROL_ACTIVATION_RANGE*rad_deg, 1);
      Serial.println("°)");
      last_debug_print = current_time;
    }
  }
}

void debug_control_performance(float control_output) {
    static unsigned long last_debug = 0;
    static float max_control_seen = 0;
    static int control_applications = 0;
    
    if (millis() - last_debug > 1000) {  // Cada segundo
        Serial.println("=== DIAGNÓSTICO DE CONTROL ===");
        Serial.print("Max control output visto: "); Serial.println(max_control_seen, 4);
        Serial.print("Aplicaciones de control/seg: "); Serial.println(control_applications);
        Serial.print("Estado stepper: "); 
        Serial.println(stepper->get_device_state() == INACTIVE ? "LIBRE" : "OCUPADO");
        Serial.print("Error actual: "); 
        Serial.print((SETPOINT - theta)*rad_deg, 2); Serial.println("°");
        Serial.println("================================");
        
        max_control_seen = 0;
        control_applications = 0;
        last_debug = millis();
    }
    
    // Rastrear estadísticas durante control
    if (system_state == STATE_CONTROL) {
        if (abs(control_output) > max_control_seen) {
            max_control_seen = abs(control_output);
        }
        control_applications++;
    }
}

/******************************************************************************
* Setup (CON CONFIGURACIÓN OPTIMIZADA)
*/
void setup() {
  // Configurar pines
  pinMode(LED_PIN, OUTPUT);
  pinMode(D4, INPUT_PULLUP);
  pinMode(D5, INPUT_PULLUP);

  // Inicializar comunicación
  Serial.begin(BAUD_RATE);
  Serial.println("=== PÉNDULO CONTROL SUAVE ESTILO SHAWN ===");
  Serial.println("MODIFICACIÓN: Control suave con movimientos incrementales");
  ctrl.init(Serial, CTRL_DEBUG);

  // Configurar Encoder
  encoder = new RotaryEncoder(ENC_A_PIN, ENC_B_PIN, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderISR, CHANGE);
  Serial.println("Encoder del péndulo configurado");

  // Inicializar stepper CON CONFIGURACIÓN OPTIMIZADA PARA SUAVIDAD
  Serial.println("Inicializando motor stepper para control suave...");
  stepper = new L6474(STP_FLAG_IRQ_PIN, STP_STBY_RST_PIN, STP_DIR_PIN, STP_PWM_PIN, STP_SPI_CS_PIN, &dev_spi);
  
  if (stepper->init(&stepper_config_control) != COMPONENT_OK) {
    Serial.println("ERROR: No se pudo inicializar el driver del stepper");
    while(1);
  }

  stepper->attach_flag_irq(&stepperISR);
  stepper->enable_flag_irq();
  stepper->set_home();

  Serial.println("Motor stepper configurado para movimiento suave");

  // Inicializar tiempos
  prev_time = millis();
  control_loop_timer = millis();
  last_debug_print = millis();
  last_control_application = millis();

  Serial.println("=== SISTEMA INICIALIZADO ===");
  Serial.println("MODO: Control suave estilo Shawn Hymel");
  Serial.println("CARACTERÍSTICA: Movimientos incrementales pequeños y frecuentes");
  Serial.println("CORRECCIÓN: Signo del control invertido para dirección correcta");
  Serial.println("=== ESPERANDO COMANDOS ===");
}

/******************************************************************************
* LOOP PRINCIPAL SIMPLIFICADO ESTILO SHAWN
*/
void loop() {
  int command;
  float action[NUM_ACTIONS];
  int status;
  float observation[NUM_OBS];
  ControlComms::StatusCode rx_code;

  // ============================================================================
  // CONTROL SIMPLIFICADO: SOLO run_control_loop() como Shawn
  // ============================================================================
  if (system_state == STATE_CONTROL || system_state == STATE_WAITING_FOR_RANGE) {
    run_control_loop(); // ← SOLO esta función, sin update_stepper_continuous
  }

  // Manejar la comunicación (SIN CAMBIOS)
  rx_code = ctrl.receive_action<NUM_ACTIONS>(&command, action);

  if (rx_code == ControlComms::OK) {
    switch (command) {
      case CMD_SET_HOME:
        set_stepper_home();
        Serial.println("Home establecido");
        break;

      case CMD_MOVE_TO:
        if (system_state == STATE_IDLE) {
          float target_rad = action[0]*deg_rad;
          move_stepper_to(target_rad);
          Serial.print("Moviendo stepper a: "); Serial.print(action[0], 1); Serial.println("°");
        }
        break;

      case CMD_MOVE_BY:
        if (system_state == STATE_IDLE) {
          float move_rad = action[0]*deg_rad;
          move_stepper_by(move_rad);
          Serial.print("Moviendo stepper por: "); Serial.print(action[0], 1); Serial.println("°");
        }
        break;

      case CMD_SET_STEP_MODE:
        set_step_mode((int)action[0]);
        set_stepper_home();
        Serial.print("Modo de paso establecido: "); Serial.println((int)action[0]);
        break;

      case CMD_SELECT_CONTROLLER:
        current_controller = (ControllerType)((int)action[0]);
        Serial.print("Controlador seleccionado: ");
        Serial.println(current_controller == CONTROLLER_PID ? "PID" : "LQR");
        break;

      case CMD_SET_PID_GAINS:
        K_p = action[0];
        K_i = action[1];
        K_d = action[2];
        Serial.print("Ganancias PID - Kp: "); Serial.print(K_p, 6);
        Serial.print(", Ki: "); Serial.print(K_i, 6);
        Serial.print(", Kd: "); Serial.println(K_d, 6);
        break;

      case CMD_SET_LQR_GAINS:
        for (int i = 0; i < 5; i++) {
          K_lqr[i] = action[i];
        }
        Serial.print("Ganancias LQR: [");
        for (int i = 0; i < 5; i++) {
          Serial.print(K_lqr[i], 6);
          if (i < 4) Serial.print(", ");
        }
        Serial.println("]");
        break;

      case CMD_START_CONTROL:
        if (system_state == STATE_IDLE) {
          // Iniciar en estado de espera de rango
          update_states(); // Asegurarse de tener el ángulo actual
          
          // INICIALIZACIÓN MEJORADA
          accumulated_control = 0.0; // ← NUEVO: Limpiar control acumulado
          last_control_application = millis(); // ← NUEVO: Inicializar timing
          
          if (is_in_control_activation_range()) {
            // Si ya está en rango, activar control inmediatamente
            system_state = STATE_CONTROL;
            phi_reference = phi; // Referencia inicial
            
            // Reset estados del controlador
            pid_integral = 0.0;
            pid_error_prev = 0.0;
            integral_error = 0.0;
            was_in_range = true;
            
            Serial.println("*** CONTROL SUAVE INICIADO - YA EN RANGO ***");
            Serial.print("Controlador: ");
            Serial.println(current_controller == CONTROLLER_PID ? "PID" : "LQR");
            Serial.print("Posición inicial θ: "); Serial.print(theta*rad_deg, 1); Serial.println("°");
            Serial.print("Distancia desde 180°: "); Serial.print(calculate_distance_from_180(theta)*rad_deg, 1); Serial.println("°");
            Serial.print("Referencia φ: "); Serial.print(phi_reference*rad_deg, 1); Serial.println("°");
          } else {
            // Si no está en rango, esperar a que entre
            system_state = STATE_WAITING_FOR_RANGE;
            was_in_range = false;
            
            Serial.println("*** SISTEMA ACTIVADO - ESPERANDO RANGO ***");
            Serial.print("Controlador: ");
            Serial.println(current_controller == CONTROLLER_PID ? "PID" : "LQR");
            Serial.print("Posición actual θ: "); Serial.print(theta*rad_deg, 1); Serial.println("°");
            Serial.print("Distancia desde 180°: "); Serial.print(calculate_distance_from_180(theta)*rad_deg, 1); Serial.println("°");
            Serial.print("NECESITA: Posicionar péndulo en rango ±"); Serial.print(CONTROL_ACTIVATION_RANGE*rad_deg, 1); Serial.println("° desde 180°");
            Serial.println("El control se activará automáticamente cuando entre en rango");
          }
        } else {
          Serial.println("Error: Sistema no está en estado IDLE");
        }
        break;

      case CMD_STOP_CONTROL:
        system_state = STATE_IDLE;
        stepper->hard_stop();
        was_in_range = false;
        out_of_range_start_time = 0;
        accumulated_control = 0.0; // ← NUEVO: Limpiar control acumulado
        Serial.println("*** CONTROL DETENIDO ***");
        break;

      default:
        Serial.print("Comando desconocido: "); Serial.println(command);
        break;
    }

    // Siempre actualiza los estados de los observadores
    update_states();

    // Preapara los observadores
    observation[0] = theta*rad_deg; // ángulo del pendulo
    observation[1] = phi*rad_deg; // ángulo del rotor
    observation[2] = dtheta*rad_deg; // velocidad del pendulo
    observation[3] = dphi*rad_deg; // velocidad del rotor
    observation[4] = current_controller; // tipo de controlador
    observation[5] = integral_error; // Error integral

    // Determine status with new conditions
    int status = STATUS_OK;
    if (system_state == STATE_CONTROL) {
      if (is_in_control_maintenance_range()) {
        status = is_pendulum_upright() ? STATUS_UPRIGHT_ACHIEVED : STATUS_CONTROL_ACTIVE;
      } else {
        status = STATUS_OUT_OF_RANGE; // estado para fuera de rango
      }
    } else if (system_state == STATE_WAITING_FOR_RANGE) {
      status = STATUS_OUT_OF_RANGE;
    } else if (stepper->get_device_state() != INACTIVE) {
      status = STATUS_STP_MOVING;
    }

    // Envia datos de los observadoresn
    ctrl.send_observation(status, millis(), false, observation, NUM_OBS);
  
  } else if (rx_code == ControlComms::ERROR) {
    // Solo reportar errores críticos ocasionalmente
    static unsigned long last_error_report = 0;
    if (millis() - last_error_report > 10000) { // Cada 10 segundos
      Serial.println("Sistema en espera de comandos...");
      last_error_report = millis();
    }
  }

  // LED de estado - MODIFICADO para indicar diferentes estados
  static unsigned long last_led_update = 0;
  static bool led_state = false;
  
  if (system_state == STATE_CONTROL && is_in_control_maintenance_range()) {
    // Control activo - LED encendido fijo
    if (millis() - last_led_update > 100) {
      digitalWrite(LED_PIN, HIGH);
      last_led_update = millis();
    }
  } else if (system_state == STATE_WAITING_FOR_RANGE || 
             (system_state == STATE_CONTROL && !is_in_control_maintenance_range())) {
    // Esperando rango o fuera de rango - LED parpadeo rápido
    if (millis() - last_led_update > 200) {
      led_state = !led_state;
      digitalWrite(LED_PIN, led_state);
      last_led_update = millis();
    }
  } else {
    // Estado idle - LED parpadeo lento
    if (millis() - last_led_update > 1000) {
      led_state = !led_state;
      digitalWrite(LED_PIN, led_state);
      last_led_update = millis();
    }
  }
}