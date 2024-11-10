close all
clear all
%% Toma de datos base propia

% lee el archivo .mat con las pruebas del usuario.
[filename, pathname] = uigetfile('*.mat', 'Selecciona un archivo .mat');
fullpath = fullfile(pathname, filename);
data = load(fullpath);

% Define el numero de canales
num_canales = 8;

% Datos tomados por cada canal
canales = data.data.datos(:,1:num_canales);
% UTC
UTC_sin_ajuste = data.data.datos(:,num_canales+1);

UTC = UTC_sin_ajuste - 0.5;

%Frecuencia de muestreo
fs = 256; 

tiempo = (0:length(canales(:,1))-1)/fs; % generar el eje del tiempo, se grafica desde 0 hasta 
% el numero de muestras del canal -1 sobre la frecuencuencia de muestreo del canal.

% Tiempo de inicio en segundos UTC
start_time = UTC(1,1); 

%Crear el vector de tiempo
time_vector = start_time + tiempo;

% Graficar la información de un canal
figure()
plot(time_vector, canales(:,2));
xlabel('Tiempo (segundos UTC)');
ylabel('Voltaje (uV)');
title('Datos EEG sin ningun procesamiento');

% Graficar la información de todos los canales
figure()
for i = 1:num_canales
    plot(time_vector, canales(:,i));
    xlabel('Tiempo (segundos UTC)');
    ylabel('Voltaje (uV)');
    title('Datos EEG sin ningun procesamiento');
    hold on
end
%%  Recortar señal
% RECORTAR DESDE QUE INICIO EL EXPERIMENTO PARA ELIMINAR RUIDO INICIAL Y
% FINAL

% Lee el archivo de texto, Tiempos y evento
% Columna 1: Tiempos en segundos UTC
% Columna 2: informacion de la accion (evento)

% Solicita el archivo 
[nombre_archivo_log, ruta_archivo_log] = uigetfile({'*.txt', 'Archivos de Texto (*.txt)'}, 'Seleccione el archivo');

% Construye la ruta completa del archivo
nombre_completo_archivo_log = fullfile(ruta_archivo_log, nombre_archivo_log);

% Almacena los valores del archivo en una tabla
indicadores_prueba = readtable(nombre_completo_archivo_log, 'Delimiter', '\t', 'ReadVariableNames', true);

% Extrae la primera columna (tiempo en UTC) como array numérico
timestamps_log = indicadores_prueba{:,1};

% Extrae la segunda columna (cadena de texto) como array de celdas de texto
prueba_log = indicadores_prueba{:,2};

% Define el tiempo inicial y final de la prueba
valor_inicio = timestamps_log(1,1);
valor_fin = timestamps_log(end,1);

% Del UTC de los eventos encuentra el UTC mas cercano donde se
% registro una muestra, (tiempo inicial y final para la extracción)
idx_inicio_prueba = find(UTC >= valor_inicio, 1, 'first');
idx_fin_prueba = find(UTC >= valor_fin, 1, 'first');

% Recorta los datos registrados de los canales para que coincidan con el
% incio y fin de la prueba
canales_solo_prueba = canales(idx_inicio_prueba:idx_fin_prueba, :);
UTC_solo_prueba = UTC(idx_inicio_prueba:idx_fin_prueba, :);

% Crea un nuevo vector de tiempo con las muestras actuales 
tiempo = (0:length(canales_solo_prueba(:,1))-1)/fs;

% Graficar la información de un canal unicamente con la informacion de la
% prueba
figure
plot(tiempo, canales_solo_prueba(:,2));
xlabel('Tiempo (segundos)');
ylabel('Voltaje (uV)');
title('Datos de EEG prueba voluntario');

% Graficar la información de todos los canal unicamente con la informacion 
% de la prueba
figure()
for i = 1:num_canales
    plot(tiempo, canales_solo_prueba(:,i));
    xlabel('Tiempo (segundos)');
    ylabel('Voltaje (uV)');
    title('Datos EEG unicamente prueba voluntario 3');
    hold on
end

figure()
plot(tiempo, canales_solo_prueba(:,1));
xlabel('Tiempo (segundos)');
ylabel('Voltaje (uV)');
title('Datos EEG voluntario 3 canal 1');
%% Quitar media - DC

% Crea la variable canales_sin_dc del mismo tamaño de canales_solo_prueba
canales_sin_dc = canales_solo_prueba;

% Recorre todos los canales para restarle la media a cada uno
for i = 1:num_canales
    % Calcula la media del canal i
    media_i= mean(canales_solo_prueba(:,i));
    % Resta la media a los canales y los almacena en la matriz
    canales_sin_dc(:,i) = canales_solo_prueba(:,i) - media_i;
end

% Graficar la información de un canal sin DC
figure()
plot(tiempo, canales_sin_dc(:,2));
xlabel('Tiempo (segundos)');
ylabel('Voltaje (uV)');
title('Datos EEG sin DC');

% Graficar la información de todos los canal sin DC
figure()
for i = 1:num_canales
    plot(tiempo, canales_sin_dc(:,i));
    xlabel('Tiempo (segundos)');
    ylabel('Voltaje (uV)');
    title('Datos EEG sin DC');
    hold on
end
%% Aplicar filtro - Quitar linea base y ruido de linea

% Carga el filtro diseñado, filtro de orden 8, pasaltas, IIR, butterworth,
% de frecuencia de corte 0.5 Hz
load("filtro_sos_g.mat")

%filtro Notch, IIR, orden 2, fs 256 Hz, Fnotch 60 Hz, bandwidth 1 Hz,
% Apass 1 dB
load("filtro_60Hz.mat")

% Filtra todos los canales
canales_filtrados1 = filtfilt(SOS,G,canales_sin_dc);
canales_filtrados = filtfilt(Num,Den,canales_filtrados1);

% Graficar la información de un canal filtrado
figure()
plot(tiempo, canales_filtrados(:,2));
xlabel('Tiempo (segundos)');
ylabel('Voltaje (uV)');
title('Datos EEG filtrados');

% Graficar la información de todos los canal filtrados
figure()
for i = 1:num_canales
    plot(tiempo, canales_filtrados1(:,i));
    xlabel('Tiempo (segundos)');
    ylabel('Voltaje (uV)');
    title('Datos EEG prueba voluntario 3 sin linea base');
    hold on
end

% Graficar la información de todos los canal filtrados
figure()
for i = 1:num_canales
    plot(tiempo, canales_filtrados(:,i));
    xlabel('Tiempo (segundos)');
    ylabel('Voltaje (uV)');
    title('Datos EEG prueba voluntario 3 filtro linea base y linea');
    hold on
end

%% Segmentar PI,PD,PA

%Creacion de segmentos donde se identifico cada movimiento
segmentos = {};

% Indicadores de los movimientos ejecutados
% En la ultima columna de cada segmento se almacenara el indicador para
% saber la accion (direccion de la flecha o reposo)
derecha = 2; 
izquierda  = 1;
arriba = 3;
reposo = 0;

% Indica la celda actual del segmento (numero de movimiento registrado)
% por ejemplo la flecha mostrada #14
j = 1;

% Recorre el total de eventos que se llevaron a cabo en la linea del tiempo
% durante el total de la prueba con el voluntario

% Busca en el UTC de la prueba la muestra mas cercana registrada
for i = 1:size(prueba_log,1)
    %Indica reposo
    if(strcmp(prueba_log{i}, 'Mostrada primera imagen inicial')) 
        % Inicia en la prueba actual y termina dos muestras mas adelante
        % en el evento de mostrar flecha.
        idx_inicio = find(UTC_solo_prueba >= timestamps_log(i), 1, 'first');
        idx_final = find(UTC_solo_prueba >= timestamps_log(i+2), 1, 'first');
        % Saltar el evento "cruz mostrada" en la primera ocasion, ya que
        % dentro de los limites de la ventana del evento
        % "Mostrada primera imagen inicial" se registro
        skip = 1;
        % Igualar a 1 ya que se desea almacenar en el segmento actual
        almacenar = 1;
    %Indica reposo
    elseif(strcmp(prueba_log{i}, 'Mostrada segunda imagen inicial')) 
        % Inicia en la prueba actual y termina dos muestras mas adelante
        % en el evento de mostrar flecha.
        idx_inicio = find(UTC_solo_prueba >= timestamps_log(i), 1, 'first');
        idx_final = find(UTC_solo_prueba >= timestamps_log(i+2), 1, 'first');
        % Saltar el evento "cruz mostrada" en la primera ocasion, ya que
        % dentro de los limites de la ventana del evento
        % "Mostrada segunda imagen inicial" se registro
        skip = 1;
        % Igualar a 1 ya que se desea almacenar en el segmento actual
        almacenar = 1;
    %Indica reposo
    elseif(strcmp(prueba_log{i}, 'Cruz mostrada') && skip==0)
        % Inicia desde que finalizo la ventana de la flecha previa y 
        % termina una muestra mas adelante en el evento de mostrar flecha
        % siguiente
        idx_inicio = idx_final_f;
        idx_final = find(UTC_solo_prueba >= timestamps_log(i+1), 1, 'first');
        % Igualar a 1 ya que se desea almacenar en el segmento actual
        almacenar = 1;
    
    % Si uno de los eventos corresponde a la aparicion de la flecha, indica
    % que ahi se presento un movimiento y se debe almacenar
    elseif (strcmp(prueba_log{i}, 'Flecha derecha mostrada') || ...
    strcmp(prueba_log{i}, 'Flecha izquierda mostrada') || ...
    strcmp(prueba_log{i}, 'Flecha arriba mostrada'))
        %Inicia en la muestra actual donde se mostro la flecha y termina 5
        %segundos despues.
        idx_inicio = find(UTC_solo_prueba >= timestamps_log(i), 1, 'first');
        duracion = 5;  
        idx_final = idx_inicio + round(duracion * fs) - 1;
        
        %Almacena donde termino la ventana de la flecha para usarlo en el
        %reposo
        idx_final_f = idx_final;
        % Ya paso la primera muestra del reposo entonces reinicia.
        skip = 0;
        % Igualar a 1 ya que se desea almacenar en el segmento actual
        almacenar = 1;
    else 
        % El evento no corresponde a una muestra que se desea almacenar
        almacenar = 0;
    end
    
    % Garantiza que el segmento que se almacena si corresponda a la
    % direccion de una flecha o al reposo.
    if(almacenar == 1)
        % Se asegura que el tiempo seleccionado no sobrepase el tiempo total
        if idx_final > length(canales_filtrados(:,1))
            idx_final = length(canales_filtrados(:,1));
        end
    
        % Extraer la sección de la señal y lo almacena en la celda actual
        segmentos{j} = canales_filtrados(idx_inicio:idx_final,:);
    
        % Almacena el numero de filas del segmento actual
        num_filas_segmentos = size(segmentos{j},1);
        
        % Valida si el evento actual corresponde al reposo o a la direccion
        % de una flecha para asignarle el indicador.        
        % Crea un vector de unos del mismo numero de filas del segmento
        % actual y una unica columna, se multiplica por el indicador segun
        % corresponda y se concatena.
       
        if strcmp(prueba_log{i},'Flecha derecha mostrada')
            nueva_columna = derecha .* ones(num_filas_segmentos, 1);
            segmentos{j} = [segmentos{j}, nueva_columna];
        elseif strcmp(prueba_log{i},'Flecha izquierda mostrada')
            nueva_columna = izquierda .* ones(num_filas_segmentos, 1);
            segmentos{j} = [segmentos{j}, nueva_columna];
        elseif strcmp(prueba_log{i},'Flecha arriba mostrada')
            nueva_columna = arriba .* ones(num_filas_segmentos, 1);
            segmentos{j} = [segmentos{j}, nueva_columna];
        else
            nueva_columna = reposo .* ones(num_filas_segmentos, 1);
            segmentos{j} = [segmentos{j}, nueva_columna];
        end
        % Incrementa el segmento actual.
        j = j+1; 
    end
end

%% PSD

% Define el numero de pruebas que se llevaron a cabo en total
% 7 veces se muestra la flecha de cada direccion 
% 3 direcciones posibles (derecha, izquierda, arriba)
% 2 fases (motor - imaginativo)
% 7*3*2 = 42
% 21 descansos por fase
% 42+(21*2) = 84
num_pruebas = 84;

% Inicializar el arreglo de celdas
segmentos_PSD = cell(1, num_pruebas);
segmentos_freq = cell(1, num_pruebas);

nfft = 256; %Numero de puntos de la PSD

% Recorre cada una de las pruebas registradas
for i = 1:num_pruebas
    % Extrae el segmento actual (un evento)
    segmento_actual = segmentos{i};
    % Recorre el numero de canales
    for j = 1:num_canales
        % Calcula la PSD de ese segmento en ese canal.
        [PSD,freq] = pwelch(segmento_actual(:,j),[],[],nfft,fs);
        % Almacena los resultados en la matriz
        segmentos_PSD{i}(:,j)=PSD;
        segmentos_freq{i}(:,j)=freq;
    end
    % Almacena el indicador en la ultima fila de cada segmento.
    segmentos_PSD{i}(:,9) = segmento_actual(1:size(segmentos_PSD{i},1),num_canales+1);
end

% Graficar la PSD
figure();
plot(freq, PSD);
xlabel('Frecuencia (Hz)');
ylabel('Densidad espectral de potencia (dB/Hz)');
title('PSD de la señal');

figure();
plot(segmentos_freq{1}(:,1), segmentos_PSD{1}(:,1));
xlabel('Frecuencia (Hz)');
ylabel('Densidad espectral de potencia (V²/Hz)');
title('PSD de un evento en un canal');
%% Suma total PSD

% suma total de elementos de la PSD 
suma_PSD = {};
% promedio total de elementos de la PSD 
promedio_PSD = {};
% energia total del evento 
energia_PSD = {};

% Recorre cada una de los eventos
for i = 1:num_pruebas
    % Almacena el segmento actual (el evento actual)
    segmento_actual = segmentos_PSD{i};
    % Recorre todos los canales
    for j = 1:num_canales
        % Realiza la suma de todas las filas de ese canal en ese evento
        suma_PSD{i}(:,j) = sum(segmento_actual(:,j));
        promedio_PSD{i}(:,j) = mean(segmento_actual(:,j));
        energia_PSD{i}(:, j) = sum(segmento_actual(:, j).^2);  % Calcula energía
    end
    % Almacena el indicador de ese evento
    suma_PSD{i}(1,num_canales+1) = segmento_actual(1,num_canales+1);
end
%% Segmentacion señal segun bandas de frecuencia

% Inicializar el arreglo de celdas
segmentos_PSD_bandas = cell(1, num_pruebas); % Crea un arreglo de 14 celdas
segmentos_freq_bandas = cell(1, num_pruebas); % Crea un arreglo de 14 celdas

% Almacena el total de ritmos cerebrales que se van a segmentar
num_ritmos_cerebrales = 4;

% Inicializa en 0 las celdas de cada prueba
for i = 1:num_pruebas
    % Almacena el numero de filas de la PSD
    num_filas_psd = length(segmentos_PSD{i});
    segmentos_PSD_bandas{i} = zeros(num_filas_psd, (num_ritmos_cerebrales*num_canales)+1); 
    segmentos_freq_bandas{i} = zeros(num_filas_psd, (num_ritmos_cerebrales*num_canales)+1); 
end

%Filas segun el ritmo cerebral en el que se encuentra
fila_alfa = 1;
fila_delta = 1;
fila_theta = 1;
fila_beta = 1;

%Columnas segun el ritmo cerebral en el que se encuentra y el canal.
%La primera columna corresponde al ritmo delta, la segunda a theta, la
%tercera a alfa y la cuarta a beta
columna_delta = 1;
columna_theta = 2;
columna_alfa = 3;
columna_beta = 4;

% Recorre el número de eventos
for i = 1:num_pruebas
    num_filas_psd = length(segmentos_PSD{i});
    
    % Recorre el número de canales
    for j = 1:num_canales
        
        % Recorre las filas de la PSD
        for k = 1:num_filas_psd
            % Extrae del evento, el canal y la fila actual 
            freq_actual = segmentos_freq{i}(k, j);
            psd_actual = segmentos_PSD{i}(k, j);
            
            % Según la banda de frecuencia, identifica el ritmo y lo
            % almacena en la fila y columna correspondiente
            if freq_actual < 4
                % Delta: menor a 4 Hz
                segmentos_PSD_bandas{i}(fila_delta, columna_delta) = psd_actual;
                segmentos_freq_bandas{i}(fila_delta, columna_delta) = freq_actual;
                fila_delta = fila_delta + 1;
                
            elseif freq_actual >= 4 && freq_actual < 8
                % Theta: 4 - <8 Hz
                segmentos_PSD_bandas{i}(fila_theta, columna_theta) = psd_actual;
                segmentos_freq_bandas{i}(fila_theta, columna_theta) = freq_actual;
                fila_theta = fila_theta + 1;
                
            elseif freq_actual >= 8 && freq_actual <= 13
                % Alfa: 8 - 13 Hz
                segmentos_PSD_bandas{i}(fila_alfa, columna_alfa) = psd_actual;
                segmentos_freq_bandas{i}(fila_alfa, columna_alfa) = freq_actual;
                fila_alfa = fila_alfa + 1;
                
            elseif freq_actual > 13
                % Beta: >13 Hz
                segmentos_PSD_bandas{i}(fila_beta, columna_beta) = psd_actual;
                segmentos_freq_bandas{i}(fila_beta, columna_beta) = freq_actual;
                fila_beta = fila_beta + 1;
                
            else
                disp('Error: Frecuencia fuera de rango');
            end
        end

        % Reiniciar las filas al cambiar de canal
        fila_alfa = 1;
        fila_delta = 1;
        fila_theta = 1;
        fila_beta = 1;

        % Incrementar las columnas de cada ritmo en bloques de 4 para el
        % siguiente canal
        columna_delta = columna_delta + num_ritmos_cerebrales;
        columna_theta = columna_theta + num_ritmos_cerebrales;
        columna_alfa = columna_alfa + num_ritmos_cerebrales;
        columna_beta = columna_beta + num_ritmos_cerebrales;
    end

    % Reiniciar filas y columnas para el siguiente evento
    fila_alfa = 1;
    fila_delta = 1;
    fila_theta = 1;
    fila_beta = 1;

    columna_delta = 1;
    columna_theta = 2;
    columna_alfa = 3;
    columna_beta = 4;
end

%% Suma por bandas

% Inicializar el arreglo de celdas
suma_PSD_bandas = cell(1, num_pruebas);

% Recorre todas los eventos inicizaliando en 0 la matriz.
for i = 1:num_pruebas
    suma_PSD_bandas{i} = zeros(1, (num_ritmos_cerebrales*num_canales)+1); 
end

% Recorre todos los eventos 
for i = 1:num_pruebas
    % Extrae el evento actual 
    segmento_actual = segmentos_PSD_bandas{i};
    % Recorre el total de columnas, (4 columnas por cada canal)
    for j = 1:(num_ritmos_cerebrales*num_canales)
        % Realiza la suma de ese ritmo, en ese canal, en ese evento 
        suma_PSD_bandas{i}(:,j) = sum(segmento_actual(:,j));
    end
    % Almacena el indicador del evento
    suma_PSD_bandas{i}(1,end) = segmento_actual(1,end);
    
end

%% Porcentaje

% Inicializar el arreglo de celdas
suma_PSD_bandas_porcentaje = cell(1, num_pruebas);

% Recorre el total de eventos inicializando en 0 la matriz
for i = 1:num_pruebas
    suma_PSD_bandas_porcentaje{i} = zeros(1,57);% (num_ritmos_cerebrales*num_canales)+1); 
end

% Registra el canal actual que se esta analizando 
canal_actual = 1;
% Registra el ritmo actual que se esta analizando
ritmo_actual = 1; 

% Recorre todos los eventos
for i = 1:num_pruebas
    % Extrae el evento actual tanto de la PSD por canal y evento como de la
    % PSD por canal y ritmo cerebral
    suma_PSD_actual = suma_PSD{i};
    promedio_PSD_actual = promedio_PSD{i};
    energia_PSD_actual = energia_PSD{i};
    suma_PSD_actual_banda = suma_PSD_bandas{i};
    a = 1;
    % Recorre los ritmos cerebrales de cada canal
    for j = 1:33
        % Si el ritmo actual es menor a 4 no ha llegado al final del canal por lo
        % que se tiene que calcular el porcentaje, si llega a 5 incrementar
        % al siguiente canal
        if(ritmo_actual<=4)
            suma_PSD_bandas_porcentaje{i}(:,a) = suma_PSD_actual_banda(:,j)/suma_PSD_actual(:,canal_actual);
            ritmo_actual = ritmo_actual+1; % incrementar el ritmo
        else
            suma_PSD_bandas_porcentaje{i}(:,a) = suma_PSD_actual(:,canal_actual);
            a = a+1;
            suma_PSD_bandas_porcentaje{i}(:,a) = promedio_PSD_actual(:,canal_actual);
            a = a+1;
            suma_PSD_bandas_porcentaje{i}(:,a) = energia_PSD_actual(:,canal_actual);
            a = a+1;
            canal_actual = canal_actual+1; % cambiar de canal.
            suma_PSD_bandas_porcentaje{i}(:,a) = suma_PSD_actual_banda(:,j)/suma_PSD_actual(:,canal_actual);
            ritmo_actual = 2; %Reinicar el ritmo actual
        end
        a = a+1;
    end
    ritmo_actual = 1;
    canal_actual = 1; % Reiniciar el canal actual
    %Almacenar el indicador del evento de esos canales.
    suma_PSD_bandas_porcentaje{i}(1,end) = suma_PSD_actual(1,end); 
end

%% Matriz entrenamiento general

% Numero de indicadores (derecha, izquierda, arriba, reposo)
num_movimientos = 4;

%Inicializa las matrices de entrenamiento
matriz_entrenamiento = zeros(num_pruebas, 62);

%Codigos para identificacion de cada movimiento
codigo_reposo = [1,0,0,0];
codigo_izquierda = [0,1,0,0];
codigo_derecha = [0,0,1,0];
codigo_arriba = [0,0,0,1];


caracteristicas = 56;
% Recorre cada celda y almacena las características y las etiquetas
for i = 1:num_pruebas
    % Asigna las características (primeras 56 columnas)
    matriz_entrenamiento(i, 1:caracteristicas) = suma_PSD_bandas_porcentaje{i}(:, 1:caracteristicas);
    
    % Define la posición inicial de las etiquetas en la fila
    inicio_etiquetas = caracteristicas + 1;
    fin_etiquetas = caracteristicas + 4;

    % Almacena la etiqueta correspondiente
    if suma_PSD_bandas_porcentaje{i}(:, end) == 1
        matriz_entrenamiento(i, inicio_etiquetas:fin_etiquetas) = codigo_derecha;
        matriz_entrenamiento(i, fin_etiquetas + 1) = 2;
        if(num_pruebas<85) 
            matriz_entrenamiento(i, fin_etiquetas + 2) = 2;
        else 
            matriz_entrenamiento(i, fin_etiquetas + 2) = 5;
        end
    elseif suma_PSD_bandas_porcentaje{i}(:, end) == 2
        matriz_entrenamiento(i, inicio_etiquetas:fin_etiquetas) = codigo_izquierda;
        matriz_entrenamiento(i, fin_etiquetas + 1) = 1;
        if(num_pruebas<85) 
            matriz_entrenamiento(i, fin_etiquetas + 2) = 1;
        else 
            matriz_entrenamiento(i, fin_etiquetas + 2) = 4;
        end

    elseif suma_PSD_bandas_porcentaje{i}(:, end) == 3
        matriz_entrenamiento(i, inicio_etiquetas:fin_etiquetas) = codigo_arriba;
        matriz_entrenamiento(i, fin_etiquetas + 1) = 3;
        if(num_pruebas<85) 
            matriz_entrenamiento(i, fin_etiquetas + 2) = 3;
        else 
            matriz_entrenamiento(i, fin_etiquetas + 2) = 6;
        end
    else
        matriz_entrenamiento(i, inicio_etiquetas:fin_etiquetas) = codigo_reposo;
        % Agrega 89 en la columna específica para reposo
        matriz_entrenamiento(i, fin_etiquetas + 1) = 0;
         matriz_entrenamiento(i, fin_etiquetas + 2) = 0;
    end
end
%% Matriz de entrenamiento motor e imaginario

%Divide las pruebas en las 2 fases (motor e imaginario)
num_pruebas_fase = num_pruebas/2;

matriz_entrenamiento_motor = zeros(num_pruebas_fase, 62);
matriz_entrenamiento_imaginario = zeros(num_pruebas_fase, 62);

%Divide las matrices
matriz_entrenamiento_motor = matriz_entrenamiento(1:num_pruebas_fase,:);
matriz_entrenamiento_imaginario = matriz_entrenamiento(num_pruebas_fase+1:end,:);

%% Generar archivo CSV
% Pedir al usuario el número del voluntario para el archivo de salida
voluntario = input('Introduce el número de voluntario: ', 's');

% Crear nombres para los archivos CSV
nombre_matriz_motor = strcat('motor_voluntario', voluntario, '.csv');
nombre_matriz_imaginario = strcat('imaginario_voluntario', voluntario, '.csv');

% Guardar las matrices como archivos CSV
writematrix(matriz_entrenamiento_motor, nombre_matriz_motor);
writematrix(matriz_entrenamiento_imaginario, nombre_matriz_imaginario);

disp('Archivos CSV guardados exitosamente.');

