function varargout = PSR_GUI(varargin)
% PSR_GUI MATLAB code for PSR_GUI.fig
%      PSR_GUI, by itself, creates a new PSR_GUI or raises the existing
%      singleton*.
%
%      H = PSR_GUI returns the handle to a new PSR_GUI or the handle to
%      the existing singleton*.
%
%      PSR_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PSR_GUI.M with the given input arguments.
%
%      PSR_GUI('Property','Value',...) creates a new PSR_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before PSR_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to PSR_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PSR_GUI

% Last Modified by GUIDE v2.5 05-May-2022 08:44:45

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @PSR_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @PSR_GUI_OutputFcn, ...
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

% --- Executes just before PSR_GUI is made visible.
function PSR_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PSR_GUI (see VARARGIN)

% Choose default command line output for PSR_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using PSR_GUI.
if strcmp(get(hObject,'Visible'),'off')
    plot(rand(5));
end

% UIWAIT makes PSR_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = PSR_GUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.axes1);
% cla;
% L1 = Link('d',0.0892,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]); 
% L2 = Link('d',0.1357,'a',0.425,'alpha',-pi,'offset',-pi/2,'qlim',[deg2rad(-90),deg2rad(90)]); 
% L3 = Link('d',0.1197,'a',0.39243,'alpha',pi,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]); 
% L4 = Link('d',0.093,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-360),deg2rad(360)]); 
% L5 = Link('d',0.093,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]); 
% L6 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]); 


L1 = Link('d',0,'a',0.138,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-135),deg2rad(135)]); 
L2 = Link('d',0,'a',0,'alpha',0,'offset',-pi/2,'qlim',[deg2rad(5),deg2rad(80)]); 
L3 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(15),deg2rad(170)]); 
L4 = Link('d',0,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-90),deg2rad(90)]); 
L5 = Link('d',0,'a',0.1,'alpha',0,'offset',0,'qlim',[deg2rad(-85),deg2rad(85)]); 
%L6 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]); 

model = SerialLink([L1 L2 L3 L4 L5],'name','Dobot'); 
% for linkIndex = 0:model.n 
%     [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR5Link',num2str(linkIndex),'.ply'],'tri'); % #ok<AGROW>         
%     model.faces{linkIndex+1} = faceData; 
%     model.points{linkIndex+1} = vertexData; 
% end 
% Display robot 
workspace = [-1 1 -1 1 -0.3 1];    
model.plot(zeros(1,model.n),'noarrow','workspace',workspace); 
if isempty(findobj(get(gca,'Children'),'Type','Light')) 
    camlight 
end   
model.delay = 0; 
% Try to correctly colour the arm (if colours are in ply file data) 
% for linkIndex = 0:model.n 
%     handles = findobj('Tag', model.name); 
%     h = get(handles,'UserData'); 
%     try  
%         h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ... 
%                                                       , plyData{linkIndex+1}.vertex.green ... 
%                                                       , plyData{linkIndex+1}.vertex.blue]/255; 
%         h.link(linkIndex+1).Children.FaceColor = 'interp'; 
%     catch ME_1 
%         disp(ME_1); 
%         continue; 
%     end 
% end 
data = guidata(hObject); 
data.model = model; 
guidata(hObject,data); 
%cla;

% popup_sel_index = get(handles.popupmenu1, 'Value');
% switch popup_sel_index
%     case 1
%         plot(rand(5));
%     case 2
%         plot(sin(1:0.01:25.99));
%     case 3
%         bar(1:.5:10);
%     case 4
%         plot(membrane);
%     case 5
%         surf(peaks);
% end


% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
     set(hObject,'BackgroundColor','white');
end

%set(hObject, 'String', {'plot(rand(5))', 'plot(sin(1:0.01:25))', 'bar(1:.5:10)', 'plot(membrane)', 'surf(peaks)'});
set(hObject, 'String', {'Teach'});


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
q = handles.model.getpos; 
tr = handles.model.fkine(q); 
tr(1,4) = tr(1,4) + 0.01; 
newQ = handles.model.ikcon(tr,q); 
handles.model.animate(newQ); 
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
q = handles.model.getpos; 
tr = handles.model.fkine(q); 
tr(1,4) = tr(1,4) - 0.01; 
newQ = handles.model.ikcon(tr,q); 
handles.model.animate(newQ); 
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
q = handles.model.getpos; 
tr = handles.model.fkine(q); 
tr(2,4) = tr(2,4) + 0.01; 
newQ = handles.model.ikcon(tr,q); 
handles.model.animate(newQ); 
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
q = handles.model.getpos; 
tr = handles.model.fkine(q); 
tr(2,4) = tr(2,4) - 0.01; 
newQ = handles.model.ikcon(tr,q); 
handles.model.animate(newQ); 
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
q = handles.model.getpos; 
tr = handles.model.fkine(q); 
tr(3,4) = tr(3,4) + 0.01; 
newQ = handles.model.ikcon(tr,q); 
handles.model.animate(newQ); 
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
q = handles.model.getpos; 
tr = handles.model.fkine(q); 
tr(3,4) = tr(3,4) - 0.01; 
newQ = handles.model.ikcon(tr,q); 
handles.model.animate(newQ); 
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
