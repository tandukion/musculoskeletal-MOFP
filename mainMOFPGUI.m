
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

figure(1, "position",[400,400,1200,900]);

h.ax = axes ("position", [0.05 2.5/9 5/12 6/9]);

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
    end

    if (recalc)
        cla(h.ax);
        [X J Q] = calculateMOFP(L,theta,G,f);
        drawMOFP(h.ax,X,Q);
    end

end

% ============================ GUI settings ============================

% Link length
link_length_posX = 0.01;
link_length_posY = 0.2;
link_length_w = 0.05;
link_length_h = 0.02;
h.link1_label = uicontrol ("style", "text",
                            "units", "normalized",
                            "string", "Link1: ",
                            "horizontalalignment", "left",
                            "position", [link_length_posX link_length_posY link_length_w 0.02]);
h.link1_value = uicontrol ("style", "edit",
                            "units", "normalized",
                            "string", sprintf ("%.3f", L(1)),
                            "callback", @updatePlot,
                            "position", [link_length_posX+link_length_w link_length_posY link_length_w link_length_h]);
h.m1_label = uicontrol ("style", "text",
                        "units", "normalized",
                        "string", " m",
                        "horizontalalignment", "left",
                        "position", [link_length_posX+2*link_length_w link_length_posY 0.03 0.02]);

h.link2_label = uicontrol ("style", "text",
                            "units", "normalized",
                            "string", "Link2: ",
                            "horizontalalignment", "left",
                            "position", [link_length_posX link_length_posY-link_length_h link_length_w 0.02]);
h.link2_value = uicontrol ("style", "edit",
                            "units", "normalized",
                            "string", sprintf ("%.3f", L(2)),
                            "callback", @updatePlot,
                            "position", [link_length_posX+link_length_w link_length_posY-link_length_h link_length_w link_length_h]);
h.m2_label = uicontrol ("style", "text",
                        "units", "normalized",
                        "string", " m",
                        "horizontalalignment", "left",
                        "position", [link_length_posX+2*link_length_w link_length_posY-link_length_h 0.03 0.02]);

h.link3_label = uicontrol ("style", "text",
                            "units", "normalized",
                            "string", "Link3: ",
                            "horizontalalignment", "left",
                            "position", [link_length_posX link_length_posY-2*link_length_h link_length_w 0.02]);
h.link3_value = uicontrol ("style", "edit",
                            "units", "normalized",
                            "string", sprintf ("%.3f", L(3)),
                            "callback", @updatePlot,
                            "position", [link_length_posX+link_length_w link_length_posY-2*link_length_h link_length_w link_length_h]);
h.m3_label = uicontrol ("style", "text",
                        "units", "normalized",
                        "string", " m",
                        "horizontalalignment", "left",
                        "position", [link_length_posX+2*link_length_w link_length_posY-2*link_length_h 0.03 0.02]);

% Joint Slider
joint_slider_posX = 0.15;
joint_slider_posY = 0.2;
joint_slider_w = 0.25;
joint_slider_h = 0.02;
h.hip_joint_label = uicontrol ( "style", "text",
                                "units", "normalized",
                                "string", "Hip joint",
                                "horizontalalignment", "left",
                                "position", [joint_slider_posX joint_slider_posY joint_slider_w joint_slider_h]);
h.hip_joint_slider = uicontrol ("style", "slider",
                                "units", "normalized",
                                "string", "slider",
                                "callback", @updatePlot,
                                "value", theta(1),
                                "max", pi/2,
                                "min", -pi/2,
                                "position", [joint_slider_posX joint_slider_posY-joint_slider_h joint_slider_w joint_slider_h]);
h.hip_joint_value = uicontrol ( "style", "edit",
                                "units", "normalized",
                                "string", sprintf("%.1f", theta(1)*180/pi),
                                "callback", @updatePlot,
                                "position", [joint_slider_posX+joint_slider_w+0.005 joint_slider_posY-joint_slider_h 0.05 0.02]);
h.deg1_label = uicontrol (  "style", "text",
                            "units", "normalized",
                            "string", " deg",
                            "horizontalalignment", "left",
                            "position", [joint_slider_posX+joint_slider_w+0.055 joint_slider_posY-joint_slider_h 0.03 0.02]);

h.knee_joint_label = uicontrol ("style", "text",
                                "units", "normalized",
                                "string", "Knee joint",
                                "horizontalalignment", "left",
                                "position", [joint_slider_posX joint_slider_posY-3*joint_slider_h joint_slider_w joint_slider_h]);
h.knee_joint_slider = uicontrol (   "style", "slider",
                                    "units", "normalized",
                                    "string", "slider",
                                    "callback", @updatePlot,
                                    "value", theta(2),
                                    "max", pi/6,
                                    "min", -pi,
                                    "position", [joint_slider_posX joint_slider_posY-4*joint_slider_h joint_slider_w joint_slider_h]);
h.knee_joint_value = uicontrol ("style", "edit",
                                "units", "normalized",
                                "string", sprintf("%.1f", theta(2)*180/pi),
                                "callback", @updatePlot,
                                "position", [joint_slider_posX+joint_slider_w+0.005 joint_slider_posY-4*joint_slider_h 0.05 0.02]);
h.deg2_label = uicontrol (  "style", "text",
                            "units", "normalized",
                            "string", " deg",
                            "horizontalalignment", "left",
                            "position", [joint_slider_posX+joint_slider_w+0.055 joint_slider_posY-4*joint_slider_h 0.03 0.02]);

h.ankle_joint_label = uicontrol (   "style", "text",
                                    "units", "normalized",
                                    "string", "Ankle joint",
                                    "horizontalalignment", "left",
                                    "position", [joint_slider_posX joint_slider_posY-6*joint_slider_h joint_slider_w joint_slider_h]);
h.ankle_joint_slider = uicontrol (  "style", "slider",
                                    "units", "normalized",
                                    "string", "slider",
                                    "callback", @updatePlot,
                                    "value", theta(3),
                                    "max", pi,
                                    "min", -pi/3,
                                    "position", [joint_slider_posX joint_slider_posY-7*joint_slider_h joint_slider_w joint_slider_h]);
h.ankle_joint_value = uicontrol (   "style", "edit",
                                    "units", "normalized",
                                    "string", sprintf("%.1f", theta(3)*180/pi),
                                    "callback", @updatePlot,
                                    "position", [joint_slider_posX+joint_slider_w+0.005 joint_slider_posY-7*joint_slider_h 0.05 0.02]);
h.deg3_label = uicontrol (  "style", "text",
                            "units", "normalized",
                            "string", " deg",
                            "horizontalalignment", "left",
                            "position", [joint_slider_posX+joint_slider_w+0.055 joint_slider_posY-7*joint_slider_h 0.03 0.02]);

 
set (gcf, "color", get(0, "defaultuicontrolbackgroundcolor"))
guidata (gcf, h)

% Show the initial MOFP
[X J Q] = calculateMOFP(L,theta,G,f);
drawMOFP(h.ax,X,Q);