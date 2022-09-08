function varargout = KUKA(varargin)
% KUKA MATLAB code for KUKA.fig
%      KUKA, by itself, creates a new KUKA or raises the existing
%      singleton*.
%
%      H = KUKA returns the handle to a new KUKA or the handle to
%      the existing singleton*.
%
%      KUKA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in KUKA.M with the given input arguments.
%
%      KUKA('Property','Value',...) creates a new KUKA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before KUKA_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to KUKA_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help KUKA

% Last Modified by GUIDE v2.5 23-Dec-2021 14:01:43

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @KUKA_OpeningFcn, ...
                   'gui_OutputFcn',  @KUKA_OutputFcn, ...
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


% --- Executes just before KUKA is made visible.
function KUKA_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to KUKA (see VARARGIN)

delete(allchild(handles.axes12)); %Clear axes 12
axes(handles.axes1);
imshow('C:\Users\MSI\Documents\MATLAB\KUKA\logo3.png');
axes(handles.axes3);
imshow('C:\Users\MSI\Documents\MATLAB\KUKA\robot_arm.png');
axes(handles.axes4);
imshow('C:\Users\MSI\Documents\MATLAB\KUKA\robot_dimension2.png');
axes(handles.axes5);
imshow('C:\Users\MSI\Documents\MATLAB\KUKA\work_envelope.png');
axes(handles.axes11);
imshow('C:\Users\MSI\Documents\MATLAB\KUKA\axis_data.png');
axes(handles.axes12);
imshow('C:\Users\MSI\Documents\MATLAB\KUKA\space.png');

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes KUKA wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = KUKA_OutputFcn(~, ~, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
% varargout{1} = handles.output; %IE



function edit7_Callback(~, ~, ~)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double


% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, ~, ~)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit9_Callback(~, ~, ~)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double


% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, ~, ~)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit1_Callback(~, ~, ~)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, ~, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(~, ~, ~)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, ~, ~)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(~, ~, ~)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, ~, ~)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(~, ~, ~)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Forward.
function Forward_Callback(hObject, eventdata, handles)
% hObject    handle to Forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
delete(allchild(handles.axes12)); % Clear axes 12
Th_1 = str2double(handles.Theta1.String)*pi/180;
Th_2 = str2double(handles.Theta2.String)*pi/180;
Th_3 = str2double(handles.Theta3.String)*pi/180;
Th_4 = str2double(handles.Theta4.String)*pi/180;
Th_5 = str2double(handles.Theta5.String)*pi/180;
Th_6 = str2double(handles.Theta6.String)*pi/180;

if or(Th_1 > 114*pi/180, Th_1 < -114*pi/180)
    set(handles.error,'String','Theta 1 angle exceeds the limit!')
    axes(handles.axes12);
    imshow('C:\Users\MSI\Documents\MATLAB\KUKA\error4.png');

elseif or(Th_2 > 80*pi/180, Th_2 < -110*pi/180)
    set(handles.error,'String','Theta 2 angle exceeds the limit!')
    axes(handles.axes12);
    imshow('C:\Users\MSI\Documents\MATLAB\KUKA\error4.png');

elseif or(Th_3 > 154*pi/180, Th_3 < -130*pi/180)
    set(handles.error,'String','Theta 3 angle exceeds the limit!')
    axes(handles.axes12);
    imshow('C:\Users\MSI\Documents\MATLAB\KUKA\error4.png');

elseif or(Th_4 > 350*pi/180, Th_4 < -350*pi/180)
    set(handles.error,'String','Theta 4 angle exceeds the limit!')
    axes(handles.axes12);
    imshow('C:\Users\MSI\Documents\MATLAB\KUKA\error4.png');

elseif or(Th_5 > 130*pi/180, Th_5 < -130*pi/180)
    set(handles.error,'String','Theta 5 angle exceeds the limit!')
    axes(handles.axes12);
    imshow('C:\Users\MSI\Documents\MATLAB\KUKA\error4.png');

elseif or(Th_6 > 350*pi/180, Th_6 < -350*pi/180)
    set(handles.error,'String','Theta 6 angle exceeds the limit!')
    axes(handles.axes12);
    imshow('C:\Users\MSI\Documents\MATLAB\KUKA\error4.png');
else
    % Link lenghts are determined
    L1 = 0;
    L2 = 45;
    L3 = 64.5;
    L4 = 67;
    L5 = 0;
    L6 = 0;
    
    % Create a 6-link robot
    % Link: Vector of Link objects (1xN)
    L(1) = Link([0 L1 0 pi/2]);
    L(2) = Link([0 0 L2 0]);
    L(3) = Link([0 0 L3 0]);
    L(4) = Link([0 0 L4 -pi/2]);
    L(5) = Link([0 0 L5 pi/2]);
    L(6) = Link([0 L6 0 0]);

    % SerialLink() is a robot object defined by a vector of Link class objects 
    Robot = SerialLink(L);
    Robot.name = 'KUKA KR 16';
    axes(handles.axes12);
    Robot.plot([Th_1 Th_2 Th_3 Th_4 Th_5 Th_6]);
    
    % Forward Kinematics
    T = Robot.fkine([Th_1 Th_2 Th_3 Th_4 Th_5 Th_6])
    set(handles.error,'String','Successfully plotted!');
end

function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double


% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Inverse.
function Inverse_Callback(hObject, eventdata, handles)
% hObject    handle to Inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

delete(allchild(handles.axes12)); % Clear axes 12
x = str2double(handles.position_x.String);
y = str2double(handles.position_y.String);
z = str2double(handles.position_z.String);

% Link lenghts are determined
L1 = 0;
L2 = 45;
L3 = 64.5;
L4 = 67;
L5 = 0;
L6 = 0;

% Create a 6-link robot
% Link: Vector of Link objects (1xN)
L(1) = Link([0 L1 0 pi/2]);
L(2) = Link([0 0 L2 0]);
L(3) = Link([0 0 L3 0]);
L(4) = Link([0 0 L4 -pi/2]);
L(5) = Link([0 0 L5 pi/2]);
L(6) = Link([0 L6 0 0]);

% SerialLink() is a robot object defined by a vector of Link class objects 
Robot = SerialLink(L);
Robot.name = 'KUKA KR 16';

% Transformation matrix is defined
T = [ 1 0 0 x;
      0 1 0 y;
      0 0 1 z;
      0 0 0 1];

axes(handles.axes12);
% Inverse Kinematics
% ikine	is stands for inverse kinematics using iterative numerical method
J = Robot.ikine(T,[0 0 0],'mask',[1 1 1 0 0 0])*180/pi;
handles.Th_1.String = num2str(floor(J(1)));
handles.Th_2.String = num2str(floor(J(2)));
handles.Th_3.String = num2str(floor(J(3)));
handles.Th_4.String = num2str(floor(J(3)));
handles.Th_5.String = num2str(floor(J(3)));
handles.Th_6.String = num2str(floor(J(3)));

Robot.plot(J*pi/180);
set(handles.error,'String','Successfully plotted!');


function position_x_Callback(hObject, eventdata, handles)
% hObject    handle to position_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of position_x as text
%        str2double(get(hObject,'String')) returns contents of position_x as a double


% --- Executes during object creation, after setting all properties.
function position_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to position_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function position_y_Callback(hObject, eventdata, handles)
% hObject    handle to position_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of position_y as text
%        str2double(get(hObject,'String')) returns contents of position_y as a double


% --- Executes during object creation, after setting all properties.
function position_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to position_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function position_z_Callback(hObject, eventdata, handles)
% hObject    handle to position_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of position_z as text
%        str2double(get(hObject,'String')) returns contents of position_z as a double


% --- Executes during object creation, after setting all properties.
function position_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to position_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta1_Callback(hObject, eventdata, handles)
% hObject    handle to Theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta1 as text
%        str2double(get(hObject,'String')) returns contents of Theta1 as a double


% --- Executes during object creation, after setting all properties.
function Theta1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta2_Callback(hObject, eventdata, handles)
% hObject    handle to Theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta2 as text
%        str2double(get(hObject,'String')) returns contents of Theta2 as a double


% --- Executes during object creation, after setting all properties.
function Theta2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta3_Callback(hObject, eventdata, handles)
% hObject    handle to Theta3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta3 as text
%        str2double(get(hObject,'String')) returns contents of Theta3 as a double


% --- Executes during object creation, after setting all properties.
function Theta3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta4_Callback(hObject, eventdata, handles)
% hObject    handle to Theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta4 as text
%        str2double(get(hObject,'String')) returns contents of Theta4 as a double


% --- Executes during object creation, after setting all properties.
function Theta4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta5_Callback(hObject, eventdata, handles)
% hObject    handle to Theta5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta5 as text
%        str2double(get(hObject,'String')) returns contents of Theta5 as a double


% --- Executes during object creation, after setting all properties.
function Theta5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Theta6_Callback(hObject, eventdata, handles)
% hObject    handle to Theta6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta6 as text
%        str2double(get(hObject,'String')) returns contents of Theta6 as a double


% --- Executes during object creation, after setting all properties.
function Theta6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function error_Callback(hObject, eventdata, handles)
% hObject    handle to error (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of error as text
%        str2double(get(hObject,'String')) returns contents of error as a double


% --- Executes during object creation, after setting all properties.
function error_CreateFcn(hObject, eventdata, handles)
% hObject    handle to error (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function text28_CreateFcn(hObject, eventdata, handles)
% hObject    handle to error (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
