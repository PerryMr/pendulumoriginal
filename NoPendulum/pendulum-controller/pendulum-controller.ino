/**
* @file pendulum-controller-fixed.ino
* @brief Código corregido con control directo estilo Shawn Hymel
* @author Modificado para control directo sin acumulación
* @date 2025-01-08
*/
#include "RotaryEncoder.h"
#include "L6474.h"
#include "control-comms.hpp"
#include <math.h>

/******************************************************************************
* Constantes y globales
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
const float SAMPLE_TIME = 0.005; // 20ms sampling time

// Constantes del Encoder y Stepper
const int ENC_STEPS_PER_ROTATION = 1200;
const int STP_STEPS_PER_ROTATION = 200;

// Constantes de conversión
const float deg_rad = PI/180.0;
const float rad_deg = 180.0/PI;

// Parametros de control 
const float CONTROL_ACTIVATION_RANGE = 15.0 *deg_rad; // ±10° desde 180° para ACTIVAR control
const float UPRIGHT_THRESHOLD = 15.0*deg_rad; // degrees from 180
const float SETPOINT = 180.0*deg_rad; // Posición invertida del péndulo

// Parámetros para manejo de pérdida de control
const float CONTROL_DEACTIVATION_RANGE = 50.0*deg_rad; // ±25° desde 180° para DESACTIVAR control
const unsigned long MAX_OUT_OF_RANGE_TIME = 5000; // 2 segundos máximo fuera de rango

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

// variables para control de rango
unsigned long out_of_range_start_time = 0;
bool was_in_range = false;

// variables temporales
unsigned long current_time = 0;
unsigned long prev_time = 0;
unsigned long control_loop_timer = 0;
const unsigned long CONTROL_INTERVAL = 10; // 20ms = 50Hz

// variables Debug
unsigned long last_debug_print = 0;
const unsigned long DEBUG_INTERVAL = 500; // Print debug cada 500ms

// Configuración del Stepper (optimizada para suavidad)
L6474_init_t stepper_config_control = {
  10000,                              // Acceleration: REDUCIDA para suavidad
  10000,                              // Deceleration: REDUCIDA para suavidad  
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
* Rutinia de interrupción
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
* Funciones de utilidad
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
* funciones para verificar rango de control
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
* Actualización de las variables del sistema
*/
void update_states() {
  current_time = millis();
  float dt = SAMPLE_TIME;

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

  // Actualización del error integral para el control
  float error = SETPOINT - theta;
  integral_error = integral_error + error * dt;

  prev_time = current_time;
}

/******************************************************************************
* Algoritmos del control
/******************************************************************************
* FUNCIÓN PID OPTIMIZADA - SIN DEBUG EXCESIVO
* Reemplazar la función compute_pid_control() existente
*/
// LÍMITES DE CONTROL - MÁS PERMISIVOS
const float INTEGRAL_MAX = 2.0;              // Límite integral aumentado
const float MAX_CONTROL_OUTPUT = 0.15;       // ~8.6° → ~8.6° (más libertad)
const float MIN_MOVEMENT = 0.002;            // ~0.11° (más sensible)

// ZONA MUERTA PEQUEÑA - Solo para eliminar jitter
const float DEADBAND = 0.03;  // ~1.7° (reducido significativamente)

// FILTRO DERIVATIVO - Parámetro crítico
const float DERIVATIVE_FILTER_COEFF = 0.1;  // Filtro más agresivo (0.1 = 90% histórico, 10% nuevo)

// Variables globales para el filtro
float derivative_filtered = 0.0;
float error_prev_filtered = 0.0;

// ============================================================================
// FUNCIÓN: Reset del controlador PID
// ============================================================================
void reset_pid_controller() {
  pid_integral = 0.0;
  pid_error_prev = 0.0;
  derivative_filtered = 0.0;
  error_prev_filtered = 0.0;
  integral_error = 0.0;
}

// FUNCIÓN: PID MEJORADO con filtro derivativo y anti-windup agresivo
float compute_pid_control() {
  // Error normalizado
  float error = normalize_angle(SETPOINT - theta);
  
  // ZONA MUERTA REDUCIDA
  if (abs(error) < DEADBAND) {
    // Dentro de zona muerta - mantener estado pero no actuar
    error_prev_filtered = 0.0;
    pid_error_prev = 0.0;
    return 0.0;
  }
  
  // Ajustar error por zona muerta para transición suave
  float error_adjusted = error;
  if (error > 0) {
    error_adjusted = error - DEADBAND;
  } else {
    error_adjusted = error + DEADBAND;
  }

  // TÉRMINO PROPORCIONAL
  float P_term = K_p * error_adjusted;
  
  // TÉRMINO INTEGRAL con Anti-Windup AGRESIVO
  // Solo integrar si no estamos saturados
  bool saturated = false;
  
  // Verificar si la salida anterior estaba saturada
  static float last_output = 0.0;
  if (abs(last_output) >= MAX_CONTROL_OUTPUT * 0.95) {
    saturated = true;
  }
  
  // Integrar solo si no saturado O si el error ayuda a desaturar
  if (!saturated || (saturated && ((error_adjusted > 0 && pid_integral < 0) || 
                                    (error_adjusted < 0 && pid_integral > 0)))) {
    pid_integral += error_adjusted * SAMPLE_TIME;
  }
  
  // Limitar integral
  if (pid_integral > INTEGRAL_MAX) {
    pid_integral = INTEGRAL_MAX;
  } else if (pid_integral < -INTEGRAL_MAX) {
    pid_integral = -INTEGRAL_MAX;
  }
  
  float I_term = K_i * pid_integral;
  
  // TÉRMINO DERIVATIVO con FILTRO PASA-BAJOS
  // Filtrar el error primero para reducir ruido
  error_prev_filtered = DERIVATIVE_FILTER_COEFF * error_adjusted + 
                        (1.0 - DERIVATIVE_FILTER_COEFF) * error_prev_filtered;
  
  // Calcular derivada del error filtrado
  float derivative_raw = (error_prev_filtered - pid_error_prev) / SAMPLE_TIME;
  
  // Filtrar la derivada también
  derivative_filtered = DERIVATIVE_FILTER_COEFF * derivative_raw + 
                        (1.0 - DERIVATIVE_FILTER_COEFF) * derivative_filtered;
  
  float D_term = K_d * derivative_filtered;
  
  // SALIDA TOTAL
  float output = P_term + I_term + D_term;
  
  // Saturación con reducción agresiva de integral
  if (output > MAX_CONTROL_OUTPUT) {
    output = MAX_CONTROL_OUTPUT;
    // Reducir integral agresivamente si está saturado en la misma dirección
    if (error_adjusted > 0) {
      pid_integral *= 0.7;  // Reducir 30%
    }
  } else if (output < -MAX_CONTROL_OUTPUT) {
    output = -MAX_CONTROL_OUTPUT;
    // Reducir integral agresivamente
    if (error_adjusted < 0) {
      pid_integral *= 0.7;  // Reducir 30%
    }
  }
  
  // Guardar para siguiente iteración
  pid_error_prev = error_prev_filtered;
  last_output = output;
  
  return output;
}

// Implementar el LQR
float compute_lqr_control() {
  // Vector de estado: [phi_error, dphi, theta_error, dtheta, integral_error]
  float phi_error_rad = phi - 0.0; // Referencia siempre en 0
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

  return control;
}

/******************************************************************************
* FUNCIÓN DE CONTROL DIRECTA ESTILO SHAWN (SIN ACUMULACIÓN COMPLEJA)
*/
void apply_control_direct(float control_output) {
  // Validación básica
  if (!isfinite(control_output)) {
    return;
  }
  
  // Movimiento mínimo reducido para mayor sensibilidad
  if (abs(control_output) < MIN_MOVEMENT) {
    return;
  }
  
  // No enviar si motor está ocupado
  if (stepper->get_device_state() != INACTIVE) {
    return;
  }
  
  // Aplicar movimiento
  move_stepper_by(control_output);
}

/******************************************************************************
* función para gestionar transiciones de estado basadas en rango
*/
void manage_control_state_transitions() {
  bool currently_in_range = is_in_control_maintenance_range();
  
  switch (system_state) {
    case STATE_WAITING_FOR_RANGE:
      // Esperando que el péndulo entre en rango de activación
      if (is_in_control_activation_range()) {
        system_state = STATE_CONTROL;
        
        // Reset estados del controlador
        reset_pid_controller();
        
        out_of_range_start_time = 0;
        was_in_range = true;
        
        Serial.println("*** CONTROL DIRECTO ACTIVADO - PÉNDULO EN RANGO ***");
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
* CONTROL LOOP PRINCIPAL - VERSIÓN SIMPLIFICADA ESTILO SHAWN
*/
void run_control_loop() {
  // Ejecutar el bucle de control cada 20ms (sample time fijo)
  if (millis() - control_loop_timer < CONTROL_INTERVAL) {
    return;
  }
  control_loop_timer = millis();

  update_states();
  manage_control_state_transitions();

  if (system_state == STATE_CONTROL && is_in_control_maintenance_range()) {
    
    // PROTECCIÓN: Verificar velocidad excesiva
    const float MAX_VELOCITY = 500.0;  // deg/s
    if (abs(dtheta * rad_deg) > MAX_VELOCITY) {
      Serial.println("⚠ VELOCIDAD EXCESIVA - DETENIENDO CONTROL");
      Serial.print("dθ = ");
      Serial.print(dtheta * rad_deg, 1);
      Serial.println(" deg/s");
      
      system_state = STATE_WAITING_FOR_RANGE;
      stepper->hard_stop();
      reset_pid_controller();
      return;
    }

    float control_output = 0.0;
    
    if (current_controller == CONTROLLER_PID) {
      control_output = compute_pid_control();
    } else {
      control_output = compute_lqr_control();
    }

    // USAR LA NUEVA FUNCIÓN DIRECTA (sin acumulación)
    apply_control_direct(control_output);

  } else {
    // Fuera de control: no hacer nada
    if (system_state == STATE_WAITING_FOR_RANGE && 
        millis() - last_debug_print > DEBUG_INTERVAL * 2) {
      Serial.print("ESPERANDO RANGO θ:");
      Serial.print(theta*rad_deg, 1);
      Serial.print("° dist_180:");
      Serial.print(calculate_distance_from_180(theta)*rad_deg, 1);
      Serial.print("° (necesita ≤");
      Serial.print(CONTROL_ACTIVATION_RANGE*rad_deg, 1);
      Serial.println("°)");
      last_debug_print = millis();
    }
  }
}

/******************************************************************************
* Setup
*/
void setup() {
  // Configurar pines
  pinMode(LED_PIN, OUTPUT);
  pinMode(D4, INPUT_PULLUP);
  pinMode(D5, INPUT_PULLUP);

  // Inicializar comunicación
  Serial.begin(BAUD_RATE);
  Serial.println("=== PÉNDULO CONTROL DIRECTO ESTILO SHAWN ===");
  Serial.println("MODIFICACIÓN: Control directo sin acumulación - versión corregida");
  ctrl.init(Serial, CTRL_DEBUG);

  // Configurar Encoder
  encoder = new RotaryEncoder(ENC_A_PIN, ENC_B_PIN, RotaryEncoder::LatchMode::TWO03);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encoderISR, CHANGE);
  Serial.println("Encoder del péndulo configurado");

  // Inicializar stepper CON CONFIGURACIÓN OPTIMIZADA PARA SUAVIDAD
  Serial.println("Inicializando motor stepper para control directo...");
  stepper = new L6474(STP_FLAG_IRQ_PIN, STP_STBY_RST_PIN, STP_DIR_PIN, STP_PWM_PIN, STP_SPI_CS_PIN, &dev_spi);
  
  if (stepper->init(&stepper_config_control) != COMPONENT_OK) {
    Serial.println("ERROR: No se pudo inicializar el driver del stepper");
    while(1);
  }

  stepper->attach_flag_irq(&stepperISR);
  stepper->enable_flag_irq();
  stepper->set_home();

  Serial.println("Motor stepper configurado para movimiento directo");

  // Inicializar tiempos
  prev_time = millis();
  control_loop_timer = millis();
  last_debug_print = millis();

  Serial.println("=== SISTEMA INICIALIZADO ===");
  Serial.println("MODO: Control directo estilo Shawn Hymel");
  Serial.println("CARACTERÍSTICA: Aplicación directa del control sin acumulación");
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

  // Control simplificado: solo run_control_loop() como Shawn
  if (system_state == STATE_CONTROL || system_state == STATE_WAITING_FOR_RANGE) {
    run_control_loop();
  }

  // Manejar la comunicación
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
          
          if (is_in_control_activation_range()) {
            // Si ya está en rango, activar control inmediatamente
            system_state = STATE_CONTROL;
            
            // Reset estados del controlador
            reset_pid_controller();
            was_in_range = true;
            
            Serial.println("*** CONTROL DIRECTO INICIADO - YA EN RANGO ***");
            Serial.print("Controlador: ");
            Serial.println(current_controller == CONTROLLER_PID ? "PID" : "LQR");
            Serial.print("Posición inicial θ: "); Serial.print(theta*rad_deg, 1); Serial.println("°");
            Serial.print("Distancia desde 180°: "); Serial.print(calculate_distance_from_180(theta)*rad_deg, 1); Serial.println("°");
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

  // LED de estado 
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