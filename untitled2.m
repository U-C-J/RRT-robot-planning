
%function: path planner GUI
%editor: Shanshan Jiang
%last edit time: 04/28/17


function varargout = untitled2(varargin)
% UNTITLED2 MATLAB code for untitled2.fig
%      UNTITLED2, by itself, creates a new UNTITLED2 or raises the existing
%      singleton*.
%
%      H = UNTITLED2 returns the handle to a new UNTITLED2 or the handle to
%      the existing singleton*.
%
%      UNTITLED2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in UNTITLED2.M with the given input arguments.
%
%      UNTITLED2('Property','Value',...) creates a new UNTITLED2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before untitled2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to untitled2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help untitled2

% Last Modified by GUIDE v2.5 02-May-2017 09:18:46

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @untitled2_OpeningFcn, ...
                   'gui_OutputFcn',  @untitled2_OutputFcn, ...
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


% --- Executes just before untitled2 is made visible.
function untitled2_OpeningFcn(hObject, eventdata, handles, varargin)


% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to untitled2 (see VARARGIN)

% Choose default command line output for untitled2
global drawtag StartPoint EndPoint Obstacles oflag Obstacle_radius Robot_radius rect intersection_flag 
global Start_angle End_angle K

StartPoint = [];
EndPoint = [];
Obstacles = [];
rect = [];
oflag = 1;
intersection_flag =1 % 1 indicates has intersection with rect, 0 means no intersection
handles.output = hObject;
drawtag =1;
Obstacle_radius = 20;
Robot_radius = 10;
Start_angle = pi/4;
End_angle = pi/4;
K = 5000%vertices in tree
Tree = []
% Update handles structure
guidata(hObject, handles);
CallWorkSpace(handles);

%plot(x0+50, y0+50, '.r', 'MarkerSize',100)
%viscircles([x0+50 y0+50], 30,'FaceColor','green','EdgeColor','b');
% UIWAIT makes untitled2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);
function CallWorkSpace(handles)
global rect 
hold off
x0 = 50;
y0 =50;
axes(handles.axes2);
rect = [x0,y0,500,500]
set(handles.axes2,'Units','pixels')
set(handles.axes2,'Position',rect);
r=rectangle('Position',rect,'LineStyle','-')
set(r,'edgecolor','black')
hold on


% --- Outputs from this function are returned to the command line.
function varargout = untitled2_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% 
% % --- Executes on button press in pushbutton1.
% function pushbutton1_Callback(hObject, eventdata, handles)
% % hObject    handle to pushbutton1 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% global im_handle
% x0 = 5;
% y0 =5;
% axes(handles.axes2);
% rect = [x0,y0,1000,500]
% set(handles.axes2,'Units','pixels')
% set(handles.axes2,'Position',rect);
% r=rectangle('Position',rect,'LineStyle','-')
% set(r,'edgecolor','black')
% %viscircles([x0+100 y0+100], 10,'EdgeColor','b');



% --- Executes on mouse press over figure background, over a disabled or
% --- inactive control, or over an axes background.
%function figure1_WindowButtonDownFcn(hObject, eventdata, handles)
function figure1_WindowButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x1 y1 drawtag StartPoint EndPoint Obstacles s e o oflag Obstacle_radius Robot_r Robot_angle user_obstacle_radius  intersection_flag
global Start_angle End_angle endo starto
currPt = get(gca,'CurrentPoint');
x1 = currPt(1,1);%current point user use mouse to click
y1 = currPt(1,2);
radius = 50;
Robot_r = 10;
%viscircles([x1 y1], radius,'EdgeColor','b');
fig = gcf
figure(fig)
if drawtag == 1%obstacles
    newObstacle = [x1 y1 Obstacle_radius];
    Detect_Intersection(newObstacle);
    if intersection_flag == 0 % no intersection with bounding rect
        %o(oflag)= plot(x1, y1, '.r', 'MarkerSize',Obstacle_radius)
        pos = [x1-Obstacle_radius,y1-Obstacle_radius, 2*Obstacle_radius,2*Obstacle_radius]
        o(oflag)= rectangle('Position', pos,'Curvature', [1 1],'EdgeColor', [0.337 0.4470 0.1410], 'FaceColor', [0.337 0.4470 0.1410])
        oflag = oflag+1;
        Obstacles = [Obstacles;newObstacle]
        intersection_flag = 1
    else
        warndlg('Input Exceeds Working Environment!','Warning')
    end
    
end
if drawtag == 2 %start position of rigid rectangle body
     
     Start = [x1,y1, Robot_r]
     Detect_Intersection(Start);
    if intersection_flag == 0 %no intersection
        if isempty(StartPoint)
            pos = [x1-Robot_r,y1-Robot_r, 2*Robot_r,2*Robot_r]
            s=rectangle('Position', pos,'Curvature', [1 1],'EdgeColor', [0.337 0.4470 0.8410], 'FaceColor', [0.337 0.4470 0.8410])
            starto = plot(x1, y1, '.r', 'MarkerSize',15);     
        else
            pos = [x1-Robot_r,y1-Robot_r, 2*Robot_r,2*Robot_r]
            set(starto,'XData', x1, 'YData', y1);
            set(s,'Position', pos)
        end
        StartPoint = [x1 y1];
        intersection_flag = 1
    else
         warndlg('Input Exceeds Working Environment!','Warning')
    end
 drawtag =1;
 print StartPoint
end
if drawtag == 3 %end point
     End = [x1,y1, Robot_r]
     Detect_Intersection(End);
    if intersection_flag == 0 %no intersection
        if isempty(EndPoint)
            pos = [x1-Robot_r,y1-Robot_r, 2*Robot_r,2*Robot_r]              
            e = rectangle('Position', pos,'Curvature', [1 1],'EdgeColor', [0.337 0.4470 0.8410], 'FaceColor', [0.337 0.4470 0.8410]);
            endo = plot(x1, y1, '.r', 'MarkerSize',15);
        else
            pos = [x1-Robot_r,y1-Robot_r, 2*Robot_r,2*Robot_r]
            set(endo,'XData', x1, 'YData', y1);
            set(e,'Position', pos);
        end
        EndPoint = [x1 y1];
        intersection_flag = 1
    else
         warndlg('Input Exceeds Working Environment!','Warning')
    end
 drawtag =1;
end
if drawtag == 4 % set radius for one obstacle
    if user_obstacle_radius ~= 0
        if ~isempty(Obstacles)
            [num,col]=size(Obstacles);
            currentPoint = [x1 y1];
            for i = 1:num% num obstacles
                oPoint = Obstacles(i,1:2)
                d1 = abs(currentPoint-oPoint);
                distance = sqrt(d1(1)^2 + d1(2)^2);
                if distance <  Obstacles(i,3) % click inside of one point
                    expectPoint=[x1 y1 user_obstacle_radius];               
                    Detect_Intersection(expectPoint);
                    if intersection_flag == 0 %no intersection
                        pos = [x1-user_obstacle_radius,y1-user_obstacle_radius,2*user_obstacle_radius,2*user_obstacle_radius]
                        Obstacles(i,3) = user_obstacle_radius;
                        set(o(i),'Position',pos);
                        intersection_flag = 1;
                    else
                        warndlg('Input Exceeds Working Environment!','Warning')
                    end
                end
            end
        end
    end
    drawtag = 1;
    print StartPoint
end
if drawtag == 0 % remove point
     Clear_Obstacle()
     ind = []
    if ~isempty(Obstacles)
        [num,col]=size(Obstacles);
        currentPoint = [x1 y1];
        for i = 1:num% num obstacles
            oPoint = Obstacles(i,1:2)
            d1 = abs(currentPoint-oPoint);
            distance = sqrt(d1(1)^2 + d1(2)^2);
            if distance <  Obstacles(i,3)
                ind = [ind;i]
            end
        end
    end
    Obstacles(ind,:)=[];
    Update_Obstacle()
end
function Detect_Intersection(point)
global rect intersection_flag
intersection_flag = 1
ox = point(1)
oy = point(2)
or = point(3)
rect 
point
oxmin = ox - or
oxmax = ox + or
oymin = oy - or
oymax = oy + or
rxmin = rect(1)
rxmax = rect(1)+rect(3)
rymin = rect(2)
rymax = rect(2)+rect(4)
if (oxmin > rxmin) && (oxmax < rxmax) && (oymin > rymin) && (oymax < rymax)
    intersection_flag = 0
    a = 1
end




function Clear_Obstacle()
global o oflag Obstacles Obstacle_radius drawtag
oflag = oflag -1 %current drawing number of o
for i = 1:oflag
delete(o(i))
end
oflag = 1


function Update_Obstacle()
global o oflag Obstacles drawtag Obstacle_radius drawtag 
if ~isempty(Obstacles)
    [num,col]=size(Obstacles)
    for j = 1:num
        pos = [Obstacles(j,1)-Obstacles(j,3),Obstacles(j,2)-Obstacles(j,3),2*Obstacles(j,3),2*Obstacles(j,3)]
        o(oflag)= rectangle('Position', pos,'Curvature', [1 1],'EdgeColor', [0.337 0.4470 0.1410], 'FaceColor', [0.337 0.4470 0.1410])
        oflag = oflag +1
    end
end
drawtag = 1




% --- Executes on button press in pushbutton2.% start position of the robot
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global drawtag;
drawtag =2;




% --- Executes on button press in pushbutton3.% end position of the robot
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global drawtag;
drawtag =3;


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)%remove obstacles
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global x1 y1 drawtag StartPoint EndPoint Obstacles s e o oflag;
drawtag = 0;


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global drawtag user_obstacle_radius;
prompt = {'Enter Radius And Then Click One Obstacle To Set'};
dlg_title = 'Set Obstacle Radius';
num_lines = 1;
defaultans = {'50'};
answer = inputdlg(prompt,dlg_title,num_lines,defaultans)
user_obstacle_radius = str2num(answer{1})
drawtag = 4;





% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)% path planning using RRT
% hObject    handle to pushbutton8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global drawtag StartPoint EndPoint Obstacles Tree delta_q K rect pplot ppi
ppi = 1
%%input to RRT path planner
%startpoint: x y Robot_r
Tree = [];
treeind = 0;
delta_q = 20
qinit.value = StartPoint;
qinit.up = 1;
%q = EndPoint;
Tree = [Tree;qinit]
for i = 1: K
    qrand = Random_Config()
    [flag,Tree,qnew] = Extend(qrand,Tree)
    if flag == 1
        disp 'reached'
        Get_Path(Tree)
        break
    end
    if flag == 2
        disp 'advanced'
    end
    if flag == 0
        disp 'trapped'
    end
end
    
function rconfig = Random_Config()
global rect Robot_r intersection_flag
collision_flag = 1;
while collision_flag == 1
    x = rect(3)*rand(1)+rect(1)
    y = rect(4)*rand(1)+rect(2)
    rconfig = [x y];
    collision_flag = obstacle_intersection(rconfig)
    point = [x y Robot_r]
    Detect_Intersection(point)
    if intersection_flag == 1
        collision_flag = 1
    end
        
end


function [flag,Tree,qnew] = Extend(qrand,Tree)
global delta_q EndPoint pplot ppi Robot_r intersection_flag 
qnear_ind = NEAREST_NEIGHBOR(qrand,Tree) 
%plot(qrand(1), qrand(2), '.g', 'MarkerSize',15);
qnear = Tree(qnear_ind).value
x = qnear(1); y = qnear(2);
pplot(ppi)=plot(qnear(1), qnear(2), '.b', 'MarkerSize',15);
ppi = ppi+1;
x1 = qrand(1); y1 = qrand(2);
if (x == x1)
    xnew = x;
    if (y1 > y)
        ynew =  y +delta_q(1);
    else
        ynew =  y -delta_q(1);
    end
elseif (y1 == y)
        if(x1 > x)
            xnew = x +delta_q(1)
        else
            xnew = x - delta_q(1);
        end
else 
 slope = (y1-y)/(x1-x);  
 angle = abs(atan(slope));
 xnew = x + (x1-x)/abs(x1-x)*delta_q*cos(angle);
 ynew = y + slope/abs(slope)*(x1-x)/abs(x1-x)*delta_q*sin(angle);
end
newcon = [xnew ynew]
collision_flag = obstacle_intersection(newcon);
point =[xnew ynew Robot_r]
Detect_Intersection(point)
flag = 0;
qnew.value =[];
qnew.up = 0;
if intersection_flag == 0 %no collision with rect
    if collision_flag==0 % no collision
        qnew.value = newcon;
        qnew.up = qnear_ind;
        pplot(ppi)=plot(newcon(1), newcon(2), '.r', 'MarkerSize',15);
        ppi=ppi+1;
        a=[x,newcon(1)]
        b=[y,newcon(2)]
        pplot(ppi)=plot(a,b,'b');
        ppi=ppi+1;
        pause(0.1)
        Tree = [Tree;qnew]
        d = sqrt(sum((newcon - EndPoint).^2))
        if d<delta_q %qnew =qrand
            flag = 1%reached
            tl = {Tree.up};
            len = size(tl)
            qnew.value = EndPoint;
            qnew.up = len(2);
            Tree = [Tree;qnew]
            a=[newcon(1),EndPoint(1)];
            b=[newcon(2),EndPoint(2)];
            pplot(ppi)=plot(a,b,'b');
            ppi=ppi+1;
        else
            flag = 2%advanced
        end
    else
        flag = 0 %collision
    end
end


    

%%%RRT
%StartPoint = [x1 y1];
%%collision detection

function qnear_ind  = NEAREST_NEIGHBOR(qrand,Tree)
%tree struct: value,  up
L = {Tree.value};
a=cell2mat(L');
qr = qrand(1:2);
a1 = a-qr;% (x,y)
b=a1.^2;
b1=b(:,1)+b(:,2);
[s i]= sort(b1)%ascend
ind1 = find(b1 == s(1));
qnear_ind = ind1(1);





function collision_flag = obstacle_intersection(rconfig)
global Obstacles Robot_r
%newObstacle = [x1 y1 Obstacle_radius];
obnum = size(Obstacles(:,1))
allflag = [];
cco = rconfig;
cr = Robot_r;
for i =1:obnum
    oco = Obstacles(i,1:2)
    or = Obstacles(i,3)
    flag = Detect_collision(cco,cr,oco,or);
    allflag = [allflag;flag]
end
s = sum(allflag)
if s == 0
    collision_flag = 0;
else
    collision_flag = 1;
end

function flag = Detect_collision(rco,cr,oco,or)
d = sqrt((rco(1)-oco(1))^2 + (rco(2)-oco(2))^2)
rd = cr+or;
if d < rd
    flag = 1;
else
    flag = 0;
end
    
function Get_Path(Tree)
global pplot ppi
disp 'get path'
tl = {Tree.up};
len = size(tl)
ind = len(2)
Ctree = [];
flag1 = 1;
ind1 = [];
ind1 = [ind;ind1];
while flag1 == 1
    ind = Tree(ind).up
    ind1 = [ind;ind1];
    if (ind == 1)
        flag1 = 0
    end
end
[r1 c1]=size(ind1);
for i=1:r1
    Ctree = [Ctree;Tree(ind1(i)).value];
end
for i = 1: r1-1
    ps = Ctree(i,:)
    pe = Ctree(i+1,:)
    a = [ps(1), pe(1)];
    b = [ps(2), pe(2)];
    pplot(ppi)=plot(a,b,'g','LineWidth',5)
    ppi = ppi+1;
    pause(0.1)
end


function Get_Path_seprate(Tree,ind)
global pplot ppi
disp 'get path'
tl = {Tree.up};
len = size(tl)
flag = 1
while flag == 1
    ps = Tree(ind).value
    ind = Tree(ind).up
    pe = Tree(ind).value
    a = [ps(1), pe(1)];
    b = [ps(2), pe(2)];
    pplot(ppi)=plot(a,b,'g','LineWidth',5)
    ppi = ppi+1;
    pause(0.1)
    if (ind == 1)
        flag = 0
    end
end

function Connect_Path(Treea,Treeb,inda,indb)
global pplot ppi
disp 'get path'
Ctree = [];
flag1 = 1;
flag2 = 1;
ind1 = [];
ind2 = [];
ind1 = [inda;ind1];
ind2 = [ind2;indb];
while flag1 == 1
    inda = Treea(inda).up
    ind1 = [inda;ind1];
    if (inda == 1)
        flag1 = 0
    end
end
while flag2 == 1
    indb = Treeb(indb).up
    ind2 = [ind2;indb];
    if (indb == 1)
        flag2 = 0
    end
end
[r1 c1]=size(ind1);
[r2 c2]= size(ind2);
for i=1:r1
    Ctree = [Ctree;Treea(ind1(i)).value];
end
for i=1:r2
    Ctree = [Ctree;Treeb(ind2(i)).value];
end
for i = 1: r1+r2-1
    ps = Ctree(i,:)
    pe = Ctree(i+1,:)
    a = [ps(1), pe(1)];
    b = [ps(2), pe(2)];
    pplot(ppi)=plot(a,b,'g','LineWidth',5)
    ppi = ppi+1;
    pause(0.1)
end
%function: add collision detection for new node and the rect


% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% clear page 
global pplot ppi s e starto endo StartPoint EndPoint Obstacles
for i = 1:ppi-1
delete(pplot(i))
end
delete(s)
delete(e)
delete(starto)
delete(endo)
Clear_Obstacle()
ppi = 1
StartPoint = [];
EndPoint = [];
Obstacles = [];


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)% RRT-connect
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global drawtag StartPoint EndPoint Obstacles Treea Treeb delta_q K rect pplot ppi
ppi = 1
%%input to RRT path planner
%startpoint: x y Robot_r
Treea = [];
Treeb = [];
treeind = 0;
delta_q = 20
qinit.value = StartPoint;
qinit.up = 1;
Treea = [Treea;qinit]
%q = EndPoint;
qgoal.value = EndPoint;
qgoal.up = 1;
Treeb = [Treeb;qgoal]
for i = 1: K
    qrand = Random_Config()
    [flaga,Treea,qnewa] = Extendab(qrand,Treea)
    [flagb,Treeb,qnewb] = Extendab(qrand,Treeb)
    if flaga ~= 0 && flagb ~=0%not trapped
        newa = qnewa.value;
        t1 = {Treea.up};
        len1 = size(t1);
        inda = len1(2);
        newb = qnewb.value;
        t2 = {Treeb.up};
        len2 = size(t2);
        indb = len2(2);

        qnear_indb = NEAREST_NEIGHBOR(newa,Treeb)
        qnearb = Treeb(qnear_indb).value
        qnear_inda = NEAREST_NEIGHBOR(newb,Treea)
        qneara = Treea(qnear_inda).value
        
        d = sqrt(sum((newa - newb).^2))
        d1 = sqrt(sum((newa - qnearb).^2))
        d2 = sqrt(sum((newb - qneara).^2))
        if d <= delta_q
            Connect_Path(Treea,Treeb,inda,indb)
            break;
        end
        if d1 <= delta_q %find near to tree b
            Connect_Path(Treea,Treeb,inda,qnear_indb)
            break;
        end
        if d2 <= delta_q
           Connect_Path(Treea,Treeb,qnear_inda,indb)        
            break;
        end
    end
end


function [flag,Tree,qnew] = Extendab(qrand,Tree)
global delta_q pplot ppi Robot_r intersection_flag 
qnear_ind = NEAREST_NEIGHBOR(qrand,Tree) 
%plot(qrand(1), qrand(2), '.g', 'MarkerSize',15);
qnear = Tree(qnear_ind).value
x = qnear(1); y = qnear(2);
pplot(ppi)=plot(qnear(1), qnear(2), '.b', 'MarkerSize',15);
ppi = ppi+1;
x1 = qrand(1); y1 = qrand(2);
if (x == x1)
    xnew = x;
    if (y1 > y)
        ynew =  y +delta_q(1);
    else
        ynew =  y -delta_q(1);
    end
elseif (y1 == y)
        if(x1 > x)
            xnew = x +delta_q(1)
        else
            xnew = x - delta_q(1);
        end
else 
 slope = (y1-y)/(x1-x);  
 angle = abs(atan(slope));
 xnew = x + (x1-x)/abs(x1-x)*delta_q*cos(angle);
 ynew = y + slope/abs(slope)*(x1-x)/abs(x1-x)*delta_q*sin(angle);
end
newcon = [xnew ynew]
collision_flag = obstacle_intersection(newcon);
point =[xnew ynew Robot_r]
Detect_Intersection(point)
flag = 0;
qnew.value =[];
qnew.up = 0;
if intersection_flag == 0 %no collision with rect
    if collision_flag==0 % no collision
        qnew.value = newcon;
        qnew.up = qnear_ind;
        pplot(ppi)=plot(newcon(1), newcon(2), '.r', 'MarkerSize',15);
        ppi=ppi+1;
        a=[x,newcon(1)]
        b=[y,newcon(2)]
        pplot(ppi)=plot(a,b,'b');
        ppi=ppi+1;
        pause(0.1)
        Tree = [Tree;qnew]
        flag = 2
    else
        flag = 0 %collision
    end
end


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% clear path
global pplot ppi s e starto endo StartPoint EndPoint Obstacles
for i = 1:ppi-1
delete(pplot(i))
end
ppi = 1
