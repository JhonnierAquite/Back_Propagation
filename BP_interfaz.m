function varargout = BP_interfaz(varargin)
% BP_INTERFAZ MATLAB code for BP_interfaz.fig
%      BP_INTERFAZ, by itself, creates a new BP_INTERFAZ or raises the existing
%      singleton*.
%
%      H = BP_INTERFAZ returns the handle to a new BP_INTERFAZ or the handle to
%      the existing singleton*.
%
%      BP_INTERFAZ('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BP_INTERFAZ.M with the given input arguments.
%
%      BP_INTERFAZ('Property','Value',...) creates a new BP_INTERFAZ or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before BP_interfaz_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to BP_interfaz_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help BP_interfaz

% Last Modified by GUIDE v2.5 07-May-2023 12:08:54

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @BP_interfaz_OpeningFcn, ...
                   'gui_OutputFcn',  @BP_interfaz_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before BP_interfaz is made visible.
function BP_interfaz_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to BP_interfaz (see VARARGIN)

% Choose default command line output for BP_interfaz
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes BP_interfaz wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = BP_interfaz_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)

%------------LEER LA IMG-------------------------------%
axes(handles.axes1)                                             %se carga la imagen en el axes1 
[filename, pathname] = uigetfile('*.png*','Selecciona imagen'); %cargamos la imagen con el comonado uigetfile
img = imread(filename);                                         %leemos la img
imshow(img);                                                    %mostramos la img
title('Imagen Original')
tam_img = size(img);                                            %obtenemos el tamaño de la img 
%------------APLICAR CONVOLUCION -----------------------%
%filtro = [0 -1 0; -1 5 -1; 0 -1 0];
filtro = [-2 -1 0; -1 1 1; 0 1 2];                              %Filtro con repujado
filtro = filtro / sum(filtro(:));                               %Normalizamos el filtro 
%-----------Aplicar la convolución a la imagen y el filtro------%
img_filt = convn(img, filtro, 'same');                          %realizamos la convolucion con la funcion con.
img_filt = uint8(img_filt);                                     %Convermimos la img en numeroes del 0 al 255 segun el pixel

                                                                %enviar la Imagen filtrada a la funcion de btn_iniciar.
handles.img_filt = img_filt;
guidata(hObject,handles);

                                                                %imprimimos la img en el axes2 
axes(handles.axes2);
imshow(img_filt);
title('Imagen filtrada');

%-------------IMPRESION EN GUIDE------------------------%
strtam=string(tam_img);                                         %convertimos el caracter en string para imprimirlo en la interfaz 
set(handles.txt_tam_img,'string',strtam);                       % enviamos la impresion a la guide


% --- Executes on button press in btn_Iniciar.
function btn_Iniciar_Callback(hObject, eventdata, handles)      %función para el btn Iniciar 
handles = guidata(hObject);                                     %creamos el obj 
img_filt = handles.img_filt;                                    %Llamamos la img _ filt con el handles y la guardamos en la img_filt                    

disp("img_filtrada");
disp(img_filt);

disp("vector de omg")
x1 =reshape(double(img_filt), 1, []);                           %convertir la matriz en un vector

% Recorremos el vector para encontrar 0 y 1 para reemplazarlos por 0.99 y
% 0.1
for i = 1:length(x1)
  if x1(i) == 1
    x1(i) = 0.99;
  elseif x1(i) == 0
    x1(i) = 0.1;
  end
end
%Tenemos un arreglo como pruba.
%x1=[0.99 0.1 0.1 0.1 0.99 0.1 0.99 0.99 0.99 0.1 0.1 0.99 0.99 0.99 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.99 0.99 0.99 0.1 0.1 0.99 0.99 0.99 0.1 0.1 0.99 0.99 0.99 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.99 0.99 0.99 0.99 0.1 0.99 0.99 0.99 0.99 0.1 0.1 0.1 0.99 0.99 0.1 0.99 0.99 0.99 0.99 0.1 0.99 0.99 0.99 0.99 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.1 0.99 0.99 0.1 0.99 0.99 0.99 0.99 0.1 0.99 0.99 0.99 0.99 0.1 0.99 0.99 0.99 0.99 0.1 0.99 0.99 0.99 0.99 0.1 0.99 0.99 0.1 0.1 0.1 0.1 0.1 0.99 0.1 0.1 0.1 0.99 0.1 0.99 0.99 0.99 0.1 0.1 0.99 0.99 0.99 0.1 0.1 0.99 0.99 0.99 0.1 0.1 0.99 0.99 0.99 0.1 0.1 0.99 0.99 0.99 0.1 0.99 0.1 0.1 0.1 0.99 0.1 0.99 0.99 0.99 0.1 0.1 0.99 0.99 0.99 0.1 0.1 0.99 0.99 0.99 0.1 0.1 0.99 0.99 0.99 0.1 0.1 0.99 0.99 0.99 0.1 0.1 0.99 0.99 0.99 0.1 0.99 0.1 0.1 0.1 0.99];

%------------------------------------------------------------------------
%_-----------------CAPTURAR DATOS----------------------------------------

y=get(handles.txt_Salida_Deseada,'string');     % Capturo el vector de entrada.
Vsalida = strsplit(y,' ');                      % separo el vector en los espacios en blanco.
yd = str2double(Vsalida);                       % convierto el vector a double.
disp("SALIDA DESEADA");
disp(yd);                                       % imprimo por consola 

%---- Alfa ---%
Alf = get(handles.txt_alfa,'string');           % Capturo el valor de alfa
alfa = str2double(Alf);                         % Convierto en double
disp("alfa")
disp(alfa)


%---- Beta ---%
b = get(handles.txt_beta,'string');
beta = str2double(b);
disp("Beta")
disp(beta)

%---- Precision ---%
P = get(handles.txt_precision,'string');
precision = str2double(P);
disp("Precision")
disp(precision)

%---- N° neuronas de capa oculta -------%

N = get(handles.txt_N_neuronas_cpa_oculta,'string');
num_neuronas_capa_oculta = str2double(N);
disp("N neuronas en la capa oculta")
disp(num_neuronas_capa_oculta)

whj = zeros(num_neuronas_capa_oculta, length(x1));      % Creo un vector para los pesos de entrada (whj)
for i = 1:num_neuronas_capa_oculta                      % Recorro mediante un for la cantidad de neuronas en la capa oculta
    whj(i,:) = -1 + (1+1)*rand(1,length(x1));           % Dependiendo de las n neuronas en la capa oculta creo los pesos para cada entrada conectadas a neurona en la capa oculta
end
tam1= length(whj(1,:));                                 % Tamaño del vector creado para los pesos de entrada
lon_whj = tam1*num_neuronas_capa_oculta;                %cantidad de pesos

%casos =0;                                              % casos
ep =1;                                                  % Error total
errores =[];                                            % Erros por cada patron
num_epocas =1;                                          % epocas

th = rand(1,num_neuronas_capa_oculta);                  % Umbrales de cada neurona en la capa oculta
wo = rand(1,length(yd)*num_neuronas_capa_oculta);       % Pesos para la capa de salida
tk = rand(1,length(yd));                                % Pesos para los Umbrales en la capa de salida

nethj = zeros([1,num_neuronas_capa_oculta]);            % Vector para suma ponderada en capa oculta
yhj = zeros([1,num_neuronas_capa_oculta]);              % salida en la neuronas de la capa oculta
dhj = zeros([num_neuronas_capa_oculta,length(x1)]);     % Vector para la retropagacion del error
netok =zeros([1,length(yd)]);                           % Vector de sumatorias netok
yok = zeros([1,length(yd)]);                            % Vector para la salida yok
dok = zeros([1,length(yd)]);                            % Vector para los errores delta

%------------------------INICIO ALGORITMO----------------------------------%
%--------------------------------------------------------------------------%
while ep >=precision
%----------------------FORWARK---------------------------------------------    
    for nt = 1:1:length(nethj)
        for j = 1:1:length(whj(1,:))
            nethj(nt) = nethj(nt) + whj(nt,j)*x1(j)+th(nt);         % Suma ponderada para la capa oculta
        end
    end

    for i=1:1:num_neuronas_capa_oculta
        yhj(i)=1/(1+exp(-nethj(i)));                                % Salida para la capa oculta
    end
    
    %----------------------------------------------------------------------
    %calculos neuronas de salida.
    for k =1:1:length(wo)
        for j =1:1:length(yd)
            for yh=1:1:num_neuronas_capa_oculta
                 netok(1,j)=wo(1,k)*yhj(1,yh)+tk(j);                 % Suma ponderada en la capa de salida   
                 yok(1,j)=1/(1+exp(-netok(1,j)));                    % Salida en la capa de salida
                 dok(1,j)=(yd(1,j)-yok(1,j))*yok(1,j)*(1-yok(1,j));  % Errores delta en la capa de salida
            end
        end
    end
    %----------------------------------------------------------------------
%%----------------------BACKWARK---------------------------------------------    
    %Propagación del error hacia atrás.
    for i = 1:length(x1)
         for j = 1:length(yd)*num_neuronas_capa_oculta
            dk = mod(j-1,length(yd))+1;
            yh = ceil(j/length(yd));

            dhj(yh,i) = dhj(yh,i) + x1(i)*(1-x1(i))*dok(dk)*wo(j);

         end
    end
    %----------------------------------------------------------------------
    %Actualizacion de pesos de salida.
     for k =1:1:length(wo)
        for j =1:1:length(yd)
                for yh=1:1:num_neuronas_capa_oculta
                    wo(1,k)=wo(1,k)+alfa*dok(1,j)*yhj(1,yh);    %actualizacion de pesos capa de salida
                end
                 tk(1,j)=tk(1,j)+alfa*dok(1,j)*1;               %actualizacion de pesos umbrales
        end
    end
    %----------------------------------------------------------------------
    %Actualizacion de pesos de entrada.
    for i = 1:1:num_neuronas_capa_oculta
        for j = 1:length(x1)
            whj(i,:) = whj(i,:) + alfa*dhj(i,j)*x1(j)+beta;     %actualizacion de pesos en la capa oculta
            for t =1:1:length(th)
                 
                 th(1,t)=th(1,t)+alfa*(dhj(i,j))/num_neuronas_capa_oculta;  %actualizacion de pesos en la capa oculta
            end
        end
    end
    
    for i=1:1:length(dok)
        ep=1/2*(dok(1,i)^2);    %errores para los patrones n
    end
    errores(num_epocas)=ep;     % guardamos los errores en un vector
    num_epocas = num_epocas +1;

    %casos = casos+1;
   
end
tolerancia = 0.05; %tolerancia para el porcentaje de presicion entre la salida y la obtenida
diferencia = abs(yok - yd);
porcentaje = 100 * diferencia / max(abs(yd), abs(yok));
if diferencia <= tolerancia * max(abs(yd), abs(yok))
    disp('Los resultados son aproximadamente iguales.');
    fprintf('Los resultados son %.2f%% iguales.\n', 100 - porcentaje);
    set(handles.txt_resultado,'string',sprintf('Los resultados son aproximadamente iguales.\nLos resultados son %.2f%% iguales.', 100 - porcentaje));
else
    disp('Los Los resultados no son aproximadamente iguales.');
    fprintf('Los resultados son %.2f%% diferentes.\n', porcentaje);
    set(handles.txt_resultado,'string',sprintf('Los resultados no son aproximadamente iguales.\nLos resultados son %.2f%% diferentes.\n', porcentaje));
end

disp("---------------------------------------------------------------------");
disp("NUEVOS PESOS DE ENTRADA");
%whjfinal = reshape(whj ,[num_neuronas_capa_oculta,length(whj)]);
%disp(whj)
% Abrir el archivo en modo de escritura
archivo = fopen('Pesospezazul.txt', 'w'); % creamos el txt para los pesos

% Escribir los datos en el archivo
for i = 1:size(whj, 1)
    fprintf(archivo, 'wh: %f ', whj(i, :));
    fprintf(archivo, '\n');
end
for i = 1:size(th, 1)
    fprintf(archivo, 'th: %f ', th(i, :));
    fprintf(archivo, '\n');
end
for i = 1:size(wo, 1)
    fprintf(archivo, 'wo: %f ', wo(i, :));
    fprintf(archivo, '\n');
end
for i = 1:size(tk, 1)
    fprintf(archivo, 'tk: %f ', tk(i, :));
    fprintf(archivo, '\n');
end

% Cerrar el archivo
fclose(archivo);
%impresiones por consola
disp("PARA UMBRALES TH")
disp(th)

disp("NUEVOS PESOS DE SALIDA WO")
disp(wo)
disp("NUEVOS PESOS DE UMBRALES TK")
disp(tk)

disp("SALIDA OBTENIDA yok")
disp(yok)
disp("epocas")
disp(num_epocas)
    
axes(handles.axes3);
plot(1:length(errores), errores)
title('Épocas versus error')
xlabel('Épocas')
ylabel('Error')

axes(handles.axes4);
plot(yd, 'o-', 'LineWidth', 2, 'MarkerSize', 10)
hold on
plot(yok, 's-', 'LineWidth', 2, 'MarkerSize', 10)
legend('yd', 'yok')  % Añadir leyenda
title('Valores de yd y yok')
xlabel('Índice')
ylabel('Valor')

%-------------IMPRENSION EN LOS TXTSTATIC.
stryo  = string (yok);
strcas = string (num_epocas);

set(handles.txt_salida_obtenida,'string',stryo);
set(handles.txt_epocas,'string',strcas);
    



function txt_alfa_Callback(hObject, eventdata, handles)
% hObject    handle to txt_alfa (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_alfa as text
%        str2double(get(hObject,'String')) returns contents of txt_alfa as a double


% --- Executes during object creation, after setting all properties.
function txt_alfa_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_alfa (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_beta_Callback(hObject, eventdata, handles)
% hObject    handle to txt_beta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_beta as text
%        str2double(get(hObject,'String')) returns contents of txt_beta as a double


% --- Executes during object creation, after setting all properties.
function txt_beta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_beta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_precision_Callback(hObject, eventdata, handles)
% hObject    handle to txt_precision (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_precision as text
%        str2double(get(hObject,'String')) returns contents of txt_precision as a double


% --- Executes during object creation, after setting all properties.
function txt_precision_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_precision (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_Salida_Deseada_Callback(hObject, eventdata, handles)
% hObject    handle to txt_Salida_Deseada (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_Salida_Deseada as text
%        str2double(get(hObject,'String')) returns contents of txt_Salida_Deseada as a double


% --- Executes during object creation, after setting all properties.
function txt_Salida_Deseada_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_Salida_Deseada (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_N_neuronas_cpa_oculta_Callback(hObject, eventdata, handles)
% hObject    handle to txt_N_neuronas_cpa_oculta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_N_neuronas_cpa_oculta as text
%        str2double(get(hObject,'String')) returns contents of txt_N_neuronas_cpa_oculta as a double


% --- Executes during object creation, after setting all properties.
function txt_N_neuronas_cpa_oculta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_N_neuronas_cpa_oculta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
