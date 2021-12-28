
close all
clear all
clear h

graphics_toolkit qt
pkg load symbolic

% Load basic data initialization
dataInit

% Load reference data
dataAthleteRobot

% ============== Data initialization ==============
% create global variables
global L theta G f k;

L = L_athlete_robot;
theta = [-pi/3 -pi/2 2*pi/3]'; % [hip knee ankle]

G = G_base .* MA_uniform;
f = f_uniform;

k = k_uniform;

figure(1, "position",[400,400,1400,900]);

ax_pos = [50 50 700 800];
h.ax = axes ("units", "pixels", "position", ax_pos);

% ============================ Functions ============================

function [X J Q] = calculateMOFP (L, theta, G, f)
    [X J] = forwardKinematics(L, theta);

    % Output force matrix needs to be in a square diagonal matrix.
    F = eye(size(f,1)) .* repmat(f, [1 length(f)]);

    % joint torque tau = G' * F
    tau = G' * F;

    % joint torque tau = J' * Q
    % Get Q for each joint, using left division \, so Q = J'\tau
    for i = 1:length(J)
        Q(:,:,i) = J{i}' \ tau(1:size(J{i},2),:);
    end
end

function C = calculateCompliance (J, G, k)    
    % Create Muscle compliance matrix needs to be in a square diagonal matrix.
    K = eye(size(k,1)) .* repmat(k, [1 length(k)]);
    
    Kq = G' * K * G;
    
    for i = 1:length(J)
        C(:,:,i) = J{i} * inv( Kq(1:size(J{i},2),1:size(J{i},2)) ) * J{i}';
    end
end

function updatePlot (obj)
    % Get the global data
    global L theta G f k;

    % Get the GUI object data
    h = guidata (obj);
    replot = false;
    recalc = false;
  
    % Action based on user input
    switch (gcbo)
        case {h.save_image}
            fn =  uiputfile ("*.png");
            print (fn);
        % Link length input
        case {h.link1_value}
            l_str = get (h.link1_value, "string");
            L(1) = str2double(l_str);
            recalc = true;
        case {h.link2_value}
            l_str = get (h.link2_value, "string");
            L(2) = str2double(l_str);
            recalc = true;
        case {h.link3_value}
            l_str = get (h.link3_value, "string");
            L(3) = str2double(l_str);
            recalc = true;

        % Joint angle input
        case {h.hip_joint_slider}
            theta(1) = get (h.hip_joint_slider, "value");
            set (h.hip_joint_value, "string", sprintf ("%.1f", theta(1)*180/pi));
            recalc = true;
        case {h.hip_joint_value}
            theta_str = get (h.hip_joint_value, "string");
            theta(1) = str2double(theta_str)*pi/180;
            set (h.hip_joint_slider, "value", theta(1));
            recalc = true;
        case {h.knee_joint_slider}
            theta(2) = get (h.knee_joint_slider, "value");
            set (h.knee_joint_value, "string", sprintf ("%.1f", theta(2)*180/pi));
            recalc = true;
        case {h.knee_joint_value}
            theta_str = get (h.knee_joint_value, "string");
            theta(2) = str2double(theta_str)*pi/180;
            set (h.knee_joint_slider, "value", theta(2));
            recalc = true;
        case {h.ankle_joint_slider}
            theta(3) = get (h.ankle_joint_slider, "value");
            set (h.ankle_joint_value, "string", sprintf ("%.1f", theta(3)*180/pi));
            recalc = true;
        case {h.ankle_joint_value}
            theta_str = get (h.ankle_joint_value, "string");
            theta(3) = str2double(theta_str)*pi/180;
            set (h.ankle_joint_slider, "value", theta(3));
            recalc = true;
        
        % G
        case {h.MA_table}
            G = get (h.MA_table, "Data");
            % recalc = true; Do not update plot on every cell update
        % f
        case {h.f_table}
            f = get (h.f_table, "Data");
            % recalc = true; Do not update plot on every cell update
        % k
        case {h.k_table}
            k = get (h.k_table, "Data");
            % recalc = true; Do not update plot on every cell update

        % pre-defined data button
        case {h.load_MA_uniform_button}
            global G_base MA_uniform f_uniform;
            % load data
            G = G_base .* MA_uniform;
            f = f_uniform;
            % update GUI data
            set(h.MA_table, "Data", G);
            set(h.f_table, "Data", f);
            % recalc = true; Do not update plot on every cell update
        case {h.load_MA_anthropomorphic_button}
            global G_base MA_anthropomorphic f_uniform;
            % load data
            G = G_base .* MA_anthropomorphic;
            f = f_uniform;
            % update GUI data
            set(h.MA_table, "Data", G);
            set(h.f_table, "Data", f);
            % recalc = true; Do not update plot on every cell update
        case {h.load_MA_monoarticular_button}
            global G_base MA_symmetric_mono_articular f_uniform;
            % load data
            G = G_base .* MA_symmetric_mono_articular;
            f = f_uniform;
            % update GUI data
            set(h.MA_table, "Data", G);
            set(h.f_table, "Data", f);
            % recalc = true; Do not update plot on every cell update
        case {h.load_MA_athlete_robot_button}
            global G_base MA_athlete_robot f_robot_2010;
            % load data
            G = G_base .* MA_athlete_robot;
            f = f_robot_2010;
            % update GUI data
            set(h.MA_table, "Data", G);
            set(h.f_table, "Data", f);
            % recalc = true; Do not update plot on every cell update
        
        case {h.update_plot_button}
            recalc = true;
    end

    if (recalc)
        cla(h.ax);
        [X J Q] = calculateMOFP(L,theta,G,f);
        drawMOFP(h.ax,X,Q);
    end

end

% ============================ GUI settings ============================
setting_posX = ax_pos(1)+ax_pos(3) + 20;
setting_posY = ax_pos(2)+ax_pos(4) - 20;

% Link length
link1_pos = [setting_posX setting_posY-20 50 20];
link2_pos = [setting_posX setting_posY-40 50 20];
link3_pos = [setting_posX setting_posY-60 50 20];
h.link1_label = uicontrol ("style", "text",
                            "string", "Link1: ",
                            "horizontalalignment", "left",
                            "position", link1_pos);
h.link1_value = uicontrol ("style", "edit",
                            "string", sprintf ("%.3f", L(1)),
                            "callback", @updatePlot,
                            "position", link1_pos + [50 0 25 0]);
h.m1_label = uicontrol ("style", "text",
                        "string", " m",
                        "horizontalalignment", "left",
                        "position", link1_pos + [125 0 -25 0]);

h.link2_label = uicontrol ("style", "text",
                            "string", "Link2: ",
                            "horizontalalignment", "left",
                            "position", link2_pos);
h.link2_value = uicontrol ("style", "edit",
                            "string", sprintf ("%.3f", L(2)),
                            "callback", @updatePlot,
                            "position", link2_pos + [50 0 25 0]);
h.m2_label = uicontrol ("style", "text",
                        "string", " m",
                        "horizontalalignment", "left",
                        "position", link2_pos + [125 0 -25 0]);

h.link3_label = uicontrol ("style", "text",
                            "string", "Link3: ",
                            "horizontalalignment", "left",
                            "position", link3_pos);
h.link3_value = uicontrol ("style", "edit",
                            "string", sprintf ("%.3f", L(3)),
                            "callback", @updatePlot,
                            "position", link3_pos + [50 0 25 0]);
h.m3_label = uicontrol ("style", "text",
                        "string", " m",
                        "horizontalalignment", "left",
                        "position", link3_pos + [125 0 -25 0]);

% Joint Slider
joint_setting_posX = setting_posX + 180;
joint_setting_posY = setting_posY;
joint1_pos = [joint_setting_posX joint_setting_posY 0 0];
joint2_pos = [joint_setting_posX joint_setting_posY-40 0 0];
joint3_pos = [joint_setting_posX joint_setting_posY-80 0 0];

h.hip_joint_label = uicontrol ( "style", "text",
                                "string", "Hip joint",
                                "horizontalalignment", "left",
                                "position", joint1_pos + [0 0 100 20]);
h.hip_joint_slider = uicontrol ("style", "slider",
                                "string", "slider",
                                "callback", @updatePlot,
                                "value", theta(1),
                                "max", pi/2,
                                "min", -pi/2,
                                "position", joint1_pos + [0 -20 250 20]);
h.hip_joint_value = uicontrol ( "style", "edit",
                                "string", sprintf("%.1f", theta(1)*180/pi),
                                "callback", @updatePlot,
                                "position", joint1_pos + [275 -20 75 20]);
h.deg1_label = uicontrol (  "style", "text",
                            "string", " deg",
                            "horizontalalignment", "left",
                            "position", joint1_pos + [350 -20 50 20]);

h.knee_joint_label = uicontrol ("style", "text",
                                "string", "Knee joint",
                                "horizontalalignment", "left",
                                "position", joint2_pos + [0 0 100 20]);
h.knee_joint_slider = uicontrol (   "style", "slider",
                                    "string", "slider",
                                    "callback", @updatePlot,
                                    "value", theta(2),
                                    "max", pi/6,
                                    "min", -pi,
                                    "position", joint2_pos + [0 -20 250 20]);
h.knee_joint_value = uicontrol ("style", "edit",
                                "string", sprintf("%.1f", theta(2)*180/pi),
                                "callback", @updatePlot,
                                "position", joint2_pos + [275 -20 75 20]);
h.deg2_label = uicontrol (  "style", "text",
                            "string", " deg",
                            "horizontalalignment", "left",
                            "position", joint2_pos + [350 -20 50 20]);

h.ankle_joint_label = uicontrol (   "style", "text",
                                    "string", "Ankle joint",
                                    "horizontalalignment", "left",
                                    "position", joint3_pos + [0 0 100 20]);
h.ankle_joint_slider = uicontrol (  "style", "slider",
                                    "string", "slider",
                                    "callback", @updatePlot,
                                    "value", theta(3),
                                    "max", pi,
                                    "min", -pi/3,
                                    "position", joint3_pos + [0 -20 250 20]);
h.ankle_joint_value = uicontrol (   "style", "edit",
                                    "string", sprintf("%.1f", theta(3)*180/pi),
                                    "callback", @updatePlot,
                                    "position", joint3_pos + [275 -20 75 20]);
h.deg3_label = uicontrol (  "style", "text",
                            "string", " deg",
                            "horizontalalignment", "left",
                            "position", joint3_pos + [350 -20 50 20]);

% ------ Table based GUI ------
table_col = 50;

% G (Moment arm) table
MA_table_posX = setting_posX;
MA_table_posY = setting_posY-400;
MA_table_h = 235;
MA_table_pos = [MA_table_posX MA_table_posY 200 MA_table_h];
h.MA_table_label = uicontrol ( "style", "text",
                                "string", "Moment arms (m)",
                                "horizontalalignment", "center",
                                "position", [MA_table_pos(1)+50 MA_table_pos(2)+MA_table_pos(4) 150 20]
);
h.MA_table = uitable (  "Data", G,
                        "ColumnEditable",true,
                        "ColumnName", {"hip";"knee";"ankle"},
                        "RowName", cellstr(mus_name),
                        "CellEditCallback", @updatePlot,
                        "ColumnWidth", repmat({table_col}, 1, size(G,2)),
                        "Position", MA_table_pos
);

% Force table
f_table_col = 60;
f_table_posX = setting_posX + 210;
f_table_posY = MA_table_posY;
f_table_h = MA_table_h;
f_table_pos = [f_table_posX f_table_posY f_table_col f_table_h];
h.f_table_label = uicontrol (   "style", "text",
                                "string", "Force (N)",
                                "horizontalalignment", "center",
                                "position", [f_table_pos(1) f_table_pos(2)+f_table_pos(4) f_table_pos(3) 20]
);
h.f_table = uitable (   "Data", f,
                        "ColumnEditable",true,
                        "ColumnName", {"f"},
                        "RowName", "",
                        "CellEditCallback", @updatePlot,
                        "ColumnWidth", repmat({f_table_col}, 1, size(f,2)),
                        "Position", f_table_pos
);

% Stifness table
k_table_col = 60;
k_table_posX = setting_posX + 275;
k_table_posY = MA_table_posY;
k_table_h = MA_table_h;
k_table_pos = [k_table_posX k_table_posY k_table_col k_table_h];
h.k_table_label = uicontrol (   "style", "text",
                                "string", "Stiffness",
                                "horizontalalignment", "center",
                                "position", [k_table_pos(1) k_table_pos(2)+k_table_pos(4) k_table_pos(3) 20]
);
h.k_table = uitable (   "Data", f,
                        "ColumnEditable",true,
                        "ColumnName", {"k"},
                        "RowName", "",
                        "CellEditCallback", @updatePlot,
                        "ColumnWidth", repmat({k_table_col}, 1, size(f,2)),
                        "Position", k_table_pos
);

% Load reference data button
button_height = 50;
button_posX = MA_table_posX + 400;
button_posY = MA_table_posY+MA_table_h-button_height;
h.load_MA_uniform_button = uicontrol (  "style", "pushbutton",
                                        "string", "Load uniform moment arm",
                                        "callback", @updatePlot,
                                        "position", [button_posX button_posY 200 button_height]
);
button_posY = button_posY-60;
h.load_MA_anthropomorphic_button = uicontrol (  "style", "pushbutton",
                                                "string", "Load anthropomorphic\nmoment arm",
                                                "callback", @updatePlot,
                                                "position", [button_posX button_posY 200 button_height]
);
button_posY = button_posY-60;
h.load_MA_monoarticular_button = uicontrol ("style", "pushbutton",
                                            "string", "Load monoarticular\nmoment arm",
                                            "callback", @updatePlot,
                                            "position", [button_posX button_posY 200 button_height]
);
button_posY = button_posY-60;
h.load_MA_athlete_robot_button = uicontrol ("style", "pushbutton",
                                            "string", "Load Athlete Robot\ndata",
                                            "callback", @updatePlot,
                                            "position", [button_posX button_posY 200 button_height]
);

% Update plot button
button_height = 50;
button_posX = MA_table_posX;
button_posY = MA_table_posY-button_height-25;
h.update_plot_button = uicontrol (  "style", "pushbutton",
                                    "string", "Update plot",
                                    "callback", @updatePlot,
                                    "position", [button_posX button_posY 200 button_height]
);

% save figure
save_img_posX = setting_posX;
save_img_posY = setting_posY-ax_pos(4);
h.save_image = uicontrol ("style", "pushbutton",
                        "string", "Save plot",
                        "callback", @updatePlot,
                        "position", [save_img_posX save_img_posY 200 50]
);

set (gcf, "color", get(0, "defaultuicontrolbackgroundcolor"))
guidata (gcf, h)

% Show the initial MOFP
[X J Q] = calculateMOFP(L,theta,G,f);
drawMOFP(h.ax,X,Q);