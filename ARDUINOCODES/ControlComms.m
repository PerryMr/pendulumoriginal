classdef ControlComms < handle
    %CONTROLCOMMS Clase para comunicación serial con Arduino
    %   Esta clase maneja la comunicación bidireccional con el controlador
    %   del péndulo invertido mediante protocolo JSON sobre puerto serial
    
    properties (Access = private)
        serialObj           % Objeto puerto serial
        timeout            % Timeout en segundos
        debugLevel         % Nivel de debug
    end
    
    properties (Constant)
        % Códigos de estado
        STATUS_OK = 0
        STATUS_ERROR = 1
        
        % Niveles de debug
        DEBUG_NONE = 0
        DEBUG_ERROR = 1
        DEBUG_WARN = 2
        DEBUG_INFO = 3
        
        % Claves JSON para transmisión
        TX_KEY_ACTION = 'action'
        TX_KEY_COMMAND = 'command'
        
        % Claves JSON para recepción
        RX_KEY_STATUS = 'status'
        RX_KEY_TIMESTAMP = 'timestamp'
        RX_KEY_TERMINATED = 'terminated'
        RX_KEY_OBSERVATION = 'observation'
    end
    
    methods
        function obj = ControlComms(timeout, debugLevel)
            %CONTROLCOMMS Constructor
            %   timeout - Tiempo de espera en segundos (default: 1.0)
            %   debugLevel - Nivel de debug (default: DEBUG_NONE)
            
            if nargin < 1
                timeout = 1.0;
            end
            if nargin < 2
                debugLevel = obj.DEBUG_NONE;
            end
            
            obj.timeout = timeout;
            obj.debugLevel = debugLevel;
            obj.serialObj = [];
        end
        
        function delete(obj)
            %DELETE Destructor - cierra puerto serial
            obj.close();
        end
        
        function portList = getSerialList(obj)
            %GETSERIALLIST Obtiene lista de puertos seriales disponibles
            %   Retorna cell array con información de puertos
            
            try
                portInfo = serialportlist("available");
                portList = cellstr(portInfo);
            catch
                % Para versiones anteriores de MATLAB
                portList = instrhwinfo('serial');
                if isfield(portList, 'AvailableSerialPorts')
                    portList = portList.AvailableSerialPorts;
                else
                    portList = {};
                end
            end
        end
        
        function status = connect(obj, port, baudRate)
            %CONNECT Conecta al puerto serial especificado
            %   port - Nombre del puerto (ej: 'COM10')
            %   baudRate - Velocidad de comunicación (default: 115200)
            %   Retorna STATUS_OK o STATUS_ERROR
            
            if nargin < 3
                baudRate = 115200;
            end
            
            % Cerrar conexión previa si existe
            obj.close();
            
            try
                % Crear objeto serial
                if exist('serialport', 'file') == 2
                    % MATLAB R2019b o posterior
                    obj.serialObj = serialport(port, baudRate, ...
                        'Timeout', obj.timeout, ...
                        'DataBits', 8, ...
                        'Parity', 'none', ...
                        'StopBits', 1, ...
                        'FlowControl', 'none');
                else
                    % Versiones anteriores de MATLAB
                    obj.serialObj = serial(port, ...
                        'BaudRate', baudRate, ...
                        'Timeout', obj.timeout, ...
                        'DataBits', 8, ...
                        'Parity', 'none', ...
                        'StopBits', 1, ...
                        'FlowControl', 'none', ...
                        'Terminator', 'LF');
                    fopen(obj.serialObj);
                end
                
                status = obj.STATUS_OK;
                
                if obj.debugLevel >= obj.DEBUG_INFO
                    fprintf('Conectado a %s a %d baudios\n', port, baudRate);
                end
                
            catch ME
                if obj.debugLevel >= obj.DEBUG_ERROR
                    fprintf('Error conectando a puerto serial: %s\n', ME.message);
                end
                status = obj.STATUS_ERROR;
            end
        end
        
        function close(obj)
            %CLOSE Cierra la conexión serial
            
            if ~isempty(obj.serialObj)
                try
                    if exist('serialport', 'file') == 2
                        % MATLAB R2019b o posterior
                        delete(obj.serialObj);
                    else
                        % Versiones anteriores
                        if strcmp(obj.serialObj.Status, 'open')
                            fclose(obj.serialObj);
                        end
                        delete(obj.serialObj);
                    end
                catch ME
                    if obj.debugLevel >= obj.DEBUG_WARN
                        fprintf('Advertencia cerrando puerto: %s\n', ME.message);
                    end
                end
                obj.serialObj = [];
            end
        end
        
        function result = step(obj, command, action)
            %STEP Envía comando y acciones, espera observación
            %   command - Comando entero a enviar
            %   action - Vector de acciones (números flotantes)
            %   Retorna struct con campos: status, timestamp, terminated, observation
            %   o empty si hay error
            
            result = [];
            
            if isempty(obj.serialObj)
                if obj.debugLevel >= obj.DEBUG_ERROR
                    fprintf('Error: Puerto serial no conectado\n');
                end
                return;
            end
            
            % Preparar mensaje JSON
            actionStr = sprintf('%.6f,', action);
            actionStr = actionStr(1:end-1); % Remover última coma
            
            msg = sprintf('{"command":%d,"action":[%s]}', command, actionStr);
            
            try
                % Enviar mensaje
                if exist('serialport', 'file') == 2
                    % MATLAB R2019b o posterior
                    write(obj.serialObj, msg, 'string');
                else
                    % Versiones anteriores
                    fprintf(obj.serialObj, '%s', msg);
                end
                
                % Esperar respuesta
                if exist('serialport', 'file') == 2
                    response = readline(obj.serialObj);
                    response = char(response);
                else
                    response = fgetl(obj.serialObj);
                end
                
                % Parsear JSON
                data = jsondecode(response);
                
                % Crear estructura de resultado
                result = struct();
                result.status = data.(obj.RX_KEY_STATUS);
                result.timestamp = data.(obj.RX_KEY_TIMESTAMP);
                result.terminated = data.(obj.RX_KEY_TERMINATED);
                result.observation = data.(obj.RX_KEY_OBSERVATION);
                
            catch ME
                if obj.debugLevel >= obj.DEBUG_ERROR
                    fprintf('Error en comunicación: %s\n', ME.message);
                    if exist('response', 'var')
                        fprintf('Respuesta recibida: %s\n', response);
                    end
                end
            end
        end
        
        function setTimeout(obj, timeout)
            %SETTIMEOUT Establece el timeout de comunicación
            %   timeout - Tiempo en segundos
            
            obj.timeout = timeout;
            if ~isempty(obj.serialObj)
                if exist('serialport', 'file') == 2
                    obj.serialObj.Timeout = timeout;
                else
                    obj.serialObj.Timeout = timeout;
                end
            end
        end
    end
end