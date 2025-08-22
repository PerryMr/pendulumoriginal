%% Control PID del Péndulo Invertido con Swing Automático
% Script principal para controlar el péndulo invertido usando un
% controlador PID con comunicación serial hacia Arduino
% MODIFICACIÓN: Agregado swing automático antes del control PID
% Autor: Convertido de Python a MATLAB (Modificado para swing)
% Fecha: 2025

clear; clc; close all;

% Agregar la clase ControlComms al path si es necesario
% addpath('ruta/donde/esta/ControlComms');

%% Constantes de configuración
SERIAL_PORT = "COM10";  % Verificar en el Administrador de dispositivos
BAUD_RATE = 500000;     % Debe coincidir con el código de Arduino
TIMEOUT = 1.0;          % Segundos
DEBUG_LEVEL = 1;        % 0=None, 1=Error, 2=Warn, 3=Info

%% Constantes de comunicación
% Comandos
CMD_SET_HOME = 0;       % Establecer posición actual como home (0 grados)
CMD_MOVE_TO = 1;        % Mover stepper a posición específica (grados)
CMD_MOVE_BY = 2;        % Mover stepper por cantidad dada (grados)
CMD_SET_STEP_MODE = 3;  % Establecer modo de paso
% MODIFICACIÓN: Nuevo comando swing
CMD_SWING = 4;          % *** LÍNEA MODIFICADA: Comando swing automático ***

% Modos de paso
STEP_MODE_1 = 0;   % 1 división por paso
STEP_MODE_2 = 1;   % 2 divisiones por paso
STEP_MODE_4 = 2;   % 4 divisiones por paso
STEP_MODE_8 = 3;   % 8 divisiones por paso
STEP_MODE_16 = 4;  % 16 divisiones por paso

% Estados
STATUS_OK = 0;           % Stepper inactivo
STATUS_STP_MOVING = 1;   % Stepper en movimiento

%% Inicializar comunicación
fprintf('Conectando al Arduino...\n');
ctrl = ControlComms(TIMEOUT, DEBUG_LEVEL);

% Obtener lista de puertos disponibles
portList = ctrl.getSerialList();
fprintf('Puertos seriales disponibles:\n');
for i = 1:length(portList)
    fprintf('  %s\n', portList{i});
end

% Conectar al Arduino
status = ctrl.connect(SERIAL_PORT, BAUD_RATE);
if status ~= ctrl.STATUS_OK
    error('ERROR: No se pudo conectar al Arduino en %s', SERIAL_PORT);
end
fprintf('Conectado exitosamente a %s\n', SERIAL_PORT);

%% Configuración inicial del sistema
fprintf('\nConfigurando sistema...\n');

% Configurar modo de paso
resp = ctrl.step(CMD_SET_STEP_MODE, STEP_MODE_8);
if ~isempty(resp)
    fprintf('Modo de paso configurado\n');
end

% Establecer home
resp = ctrl.step(CMD_SET_HOME, 0);
if ~isempty(resp)
    fprintf('Posición home establecida\n');
    fprintf('Ángulo inicial del encoder: %.1f°\n', resp.observation(1));
end

%% MODIFICACIÓN: Ejecutar swing automático antes del PID
% *** LÍNEAS MODIFICADAS: Ejecutar swing automático ***
fprintf('\n=== EJECUTANDO SWING AUTOMÁTICO ===\n');

% Verificar posición inicial del encoder
resp = ctrl.step(CMD_SET_HOME, 0);
if ~isempty(resp)
    initial_angle = resp.observation(1);
    timestamp_prev = resp.timestamp;
    fprintf('Ángulo inicial del encoder: %.1f°\n', initial_angle);
    fprintf('Stepper: %.1f°\n', resp.observation(2));
    
    % Normalizar el ángulo para manejar el cruce de 0°/360°
    normalized_angle = initial_angle;
    if normalized_angle > 270
        normalized_angle = normalized_angle - 360;
    end
    
    % Verificar si ya está cerca del objetivo (180°)
    angle_diff = abs(180 - normalized_angle);
    if normalized_angle > 180
        angle_diff = abs(180 - (normalized_angle - 360));
    end
    
    if angle_diff <= 5
        fprintf('El péndulo ya está cerca de 180°. Saltando swing automático.\n');
    else
        fprintf('El sistema realizará swing hasta alcanzar 180° en el encoder...\n');
        fprintf('Diferencia actual: %.1f°\n', angle_diff);
        fprintf('Nota: Este proceso puede tomar varios minutos...\n');
        
        % Aumentar el timeout temporalmente para el swing
        ctrl.setTimeout(30.0);  % 30 segundos de timeout para el swing
        
        % Llamar al comando swing - Arduino manejará todo el proceso automáticamente
        resp = ctrl.step(CMD_SWING, 0);
        
        % Restaurar timeout original
        ctrl.setTimeout(TIMEOUT);
        
        if ~isempty(resp)
            fprintf('Swing automático completado\n');
            fprintf('Ángulo final del encoder: %.1f°\n', resp.observation(1));
            fprintf('Ángulo final del stepper: %.1f°\n', resp.observation(2));
        else
            error('ERROR: No se pudo ejecutar el swing automático');
        end
    end
else
    error('ERROR: No se pudo verificar la posición inicial');
end

% Pequeña pausa antes de comenzar el PID
pause(2);

%% Parámetros del controlador PID
K_P = 0.1;      % Ganancia proporcional
K_I = 0.0008;   % Ganancia integral
K_D = 0.012;    % Ganancia derivativa
BIAS = 0;       % Sesgo

% Otras constantes del controlador
SETPOINT = 180;             % Punto de referencia (grados)
NUM_STEPS = 5000;           % Número máximo de pasos
ENC_GUARD_DEGS = [160, 200]; % Límites de seguridad del encoder
STEP_MODE = STEP_MODE_8;    % Modo de paso a usar

% Arrays para almacenar métricas
intervals = zeros(NUM_STEPS, 1);
enc_degs = zeros(NUM_STEPS, 1);
stp_degs = zeros(NUM_STEPS, 1);
move_degs = zeros(NUM_STEPS, 1);

%% Inicializar controlador PID
fprintf('\n=== INICIANDO CONTROL PID ===\n');

% Asegurar configuración correcta
resp = ctrl.step(CMD_SET_STEP_MODE, STEP_MODE);
resp = ctrl.step(CMD_SET_HOME, 0);

if ~isempty(resp)
    timestamp_prev = resp.timestamp;
    fprintf('Estado inicial para PID:\n');
    fprintf('  Encoder: %.1f°\n', resp.observation(1));
    fprintf('  Stepper: %.1f°\n', resp.observation(2));
else
    error('ERROR: No se pudo comunicar con Arduino durante inicialización');
end

% Variables del PID
error_prev = 0;
integral = 0;
move_deg = 0;
actual_steps = 0;

%% Bucle principal de control
fprintf('\nEjecutando control PID...\n');
fprintf('Presiona Ctrl+C para detener\n');

for i = 1:NUM_STEPS
    % Tomar paso y obtener observación
    resp = ctrl.step(CMD_MOVE_BY, move_deg);
    
    if isempty(resp)
        fprintf('ERROR: No se pudo comunicar con Arduino en paso %d\n', i);
        continue;
    end
    
    % Obtener valores de observación
    enc_deg = resp.observation(1);
    stp_deg = resp.observation(2);
    timestamp = resp.timestamp;
    
    % Calcular tiempo entre este paso y el anterior
    interval = timestamp - timestamp_prev;
    timestamp_prev = timestamp;
    
    % Asegurar que el intervalo no sea cero para evitar división por cero
    if interval <= 0
        interval = 1; % valor mínimo
    end
    
    % Calcular términos PID
    error = SETPOINT - enc_deg;
    integral = integral + (error * interval);
    derivative = (error - error_prev) / interval;
    move_deg = (K_P * error) + (K_I * integral) + (K_D * derivative) + BIAS;
    
    % Guardar valor de error para próxima iteración
    error_prev = error;
    
    % Almacenar métricas
    actual_steps = i;
    intervals(i) = interval;
    move_degs(i) = move_deg;
    enc_degs(i) = enc_deg;
    stp_degs(i) = stp_deg;
    
    % Mostrar progreso cada 5 pasos
    if mod(i, 5) == 0
        fprintf('Paso %d: Encoder=%.1f°, Step=%.1f°, Error=%.1f°, Salida=%.1f°\n', ...
                i, enc_deg, stp_deg, error, move_deg);
    end
    
    % Detener si el encoder sobrepasa los ángulos de seguridad
    if enc_deg < ENC_GUARD_DEGS(1) || enc_deg > ENC_GUARD_DEGS(2)
        fprintf('Episodio terminado en paso %d - Límites de seguridad alcanzados\n', i);
        break;
    end
    
    % Pequeña pausa para evitar saturar la comunicación
    pause(0.001);
end

% Truncar arrays al número real de pasos
intervals = intervals(1:actual_steps);
enc_degs = enc_degs(1:actual_steps);
stp_degs = stp_degs(1:actual_steps);
move_degs = move_degs(1:actual_steps);

%% Análisis de resultados
avg_interval = mean(intervals);
fprintf('\nIntervalo promedio: %.2f ms\n', avg_interval);

%% Generar gráficas
figure('Position', [100, 100, 800, 900]);

% Gráfica 1: Intervalos del microcontrolador
subplot(4, 1, 1);
plot(intervals, 'b-', 'LineWidth', 1);
title('Intervalos del Microcontrolador');
xlabel('Paso');
ylabel('Tiempo (ms)');
grid on;

% Gráfica 2: Ángulo del encoder vs setpoint
subplot(4, 1, 2);
hold on;
plot(1:actual_steps, SETPOINT * ones(actual_steps, 1), 'r-', 'LineWidth', 2, 'DisplayName', 'Setpoint');
plot(enc_degs, 'b-', 'LineWidth', 1, 'DisplayName', 'Encoder');
title('Ángulo del Encoder');
xlabel('Paso');
ylabel('Grados');
legend('show');
grid on;
hold off;

% Gráfica 3: Ángulo del stepper
subplot(4, 1, 3);
plot(stp_degs, 'b-', 'LineWidth', 1);
title('Ángulo del Stepper');
xlabel('Paso');
ylabel('Grados');
grid on;

% Gráfica 4: Señal de control
subplot(4, 1, 4);
plot(move_degs, 'b-', 'LineWidth', 1);
title('Señal de Control');
xlabel('Paso');
ylabel('Grados');
grid on;

% Ajustar espaciado entre subplots
sgtitle('Resultados del Control PID del Péndulo Invertido (Con Swing Automático)');

%% Estadísticas finales
fprintf('\n=== ESTADÍSTICAS FINALES ===\n');
fprintf('Pasos ejecutados: %d\n', actual_steps);
fprintf('Tiempo promedio por paso: %.2f ms\n', avg_interval);
fprintf('Ángulo final del encoder: %.1f°\n', enc_degs(end));
fprintf('Error final: %.1f°\n', abs(SETPOINT - enc_degs(end)));

% Calcular estadísticas de error
errors = abs(SETPOINT - enc_degs);
fprintf('Error promedio: %.2f°\n', mean(errors));
fprintf('Error máximo: %.2f°\n', max(errors));
fprintf('Desviación estándar del error: %.2f°\n', std(errors));

%% Limpiar
ctrl.close();
fprintf('\nConexión serial cerrada.\n');
fprintf('Análisis completado.\n');