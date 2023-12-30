classdef app < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                   matlab.ui.Figure
        ControlsPanel              matlab.ui.container.Panel
        ResetButton                matlab.ui.control.Button
        ProjectButton              matlab.ui.control.Button
        MagnitudeField             matlab.ui.control.NumericEditField
        VectorLamp                 matlab.ui.control.Lamp
        VectorSwitch               matlab.ui.control.Switch
        MagnitudeKnob              matlab.ui.control.Knob
        MagnitudeLabel             matlab.ui.control.Label
        MainPanel                  matlab.ui.container.Panel
        VerticalSliderField        matlab.ui.control.NumericEditField
        HorizontalRotationLabel_2  matlab.ui.control.Label
        HorizontalSliderField      matlab.ui.control.NumericEditField
        HorizontalRotationLabel    matlab.ui.control.Label
        HorizontalSlider           matlab.ui.control.Slider
        SettingsButton             matlab.ui.control.StateButton
        SettingsPanel              matlab.ui.container.Panel
        GridLayout                 matlab.ui.container.GridLayout
        AutoAdjustViewCheckBox     matlab.ui.control.CheckBox
        DragCoefficientLabel       matlab.ui.control.Label
        DragCoefficientSpinner     matlab.ui.control.Spinner
        GravityLabel               matlab.ui.control.Label
        GravitySpinner             matlab.ui.control.Spinner
        AirDensityLabel            matlab.ui.control.Label
        AirDensitySpinner          matlab.ui.control.Spinner
        BallMassLabel              matlab.ui.control.Label
        BallMassSpinner            matlab.ui.control.Spinner
        BallRadiusLabel            matlab.ui.control.Label
        BallRadiusSpinner          matlab.ui.control.Spinner
        VerticalSlider             matlab.ui.control.Slider
        UIAxes                     matlab.ui.control.UIAxes
    end

    
    properties (Access = private)
        SettingsOpen % Description
        BallRadius % Description
        BallMass % Description
        AirDensity % Description
        DragCoefficient % Description
        Gravity % Description
        PlaybackSpeed % Description
        VectorType % Description
        VelocityVector % Description
        AngularVector % Description
        VelocityVectorSliders % [Azimuth Value, Elevation Value, Magnitude]
        AngularVectorSliders % [Azimuth Value, Elevation Value, Magnitude]
    end
    
    methods (Access = private)

        function adjustMagnitude(app,value)
            if (app.VectorType == "Velocity")
                app.VelocityVector.AutoScaleFactor = value;
                app.VelocityVectorSliders(3) = value;
            else 
                app.AngularVector.AutoScaleFactor = value;
                app.AngularVectorSliders(3) = value;
            end
            if (app.AutoAdjustViewCheckBox.Value == 1) 
                adjustView(app,0,0,0);
            end
        end

        function rotateZAxis(app,vector,degrees)
            Rz = [ cosd(degrees) sind(degrees) 0; % direction cosine matrix for 3-rotation
                  -sind(degrees) cosd(degrees) 0;
                   0             0             1];
            r = [get(vector,'UData');get(vector,'VData');get(vector,'WData')]; % old vector
            scale = get(vector,'AutoScaleFactor');
            width = get(vector,'LineWidth');
            color = get(vector,'Color');

            rRz = Rz * r; % rotated vector

            newVector = quiver3(app.UIAxes,0,0,0,rRz(1),rRz(2),rRz(3),scale,'LineWidth',width,'Color',color);
            
            if (app.VectorType == "Velocity")
                app.VelocityVector = newVector;
            else
                app.AngularVector = newVector;
            end
            delete(vector);
        end

        function elevate(app,vector,degrees)
            r = [get(vector,'UData');get(vector,'VData');get(vector,'WData')];
            scale = get(vector,'AutoScaleFactor');
            width = get(vector,'LineWidth');
            color = get(vector,'Color');

            distance_2d = sqrt(r(1)^2 + r(2)^2);
            wNew = distance_2d * tand(degrees);
            
            rNew = [r(1) r(2) wNew];

            newVector = quiver3(app.UIAxes,0,0,0,rNew(1),rNew(2),rNew(3),scale,'LineWidth',width,'Color',color);
            
            if (app.VectorType == "Velocity")
                app.VelocityVector = newVector;
            else
                app.AngularVector = newVector;
            end
            delete(vector);
        end

        function adjustView(app,x,y,z)
           multiplier = 1/5;
 
           r1 = [get(app.VelocityVector,'UData');get(app.VelocityVector,'VData');get(app.VelocityVector,'WData')];
           r2 = [get(app.AngularVector,'UData');get(app.AngularVector,'VData');get(app.AngularVector,'WData')];
        
           r1 = r1 .* get(app.VelocityVector,'AutoScaleFactor');
           r2 = r2 .* get(app.AngularVector,'AutoScaleFactor');

           % keep xy axes equal
           posXY = max([r1(1) r1(2) r2(1) r2(2) x(:,1)' y(:,1)' 0]);   
           negXY = min([r1(1) r1(2) r2(1) r2(2) x(:,1)' y(:,1)' 0]);
           if (posXY == 0 && negXY == 0) 
               posXY = 1;
               negXY = -1;
           end
           posXY = posXY * (1 + multiplier);
           negXY = negXY * (1 - multiplier);
   
           posZ = max([r1(3) r2(3) z(:,1)' 0]);
           negZ = min([r1(3) r2(3) z(:,1)' 0]);
           if (posZ == 0 && negZ == 0)
                posZ = 1;
                negZ = -1;
           end
           posZ = posZ * (1 + multiplier);
           negZ = negZ * (1 - multiplier);
    
           set(app.UIAxes,'XLim',[negXY posXY]);
           set(app.UIAxes,'YLim',[negXY posXY]);
           set(app.UIAxes,'ZLim',[negZ posZ]);
           view(app.UIAxes,3);

        end
    
    end

    % methods (Static) 
    % 
    %     function states = ball_model(~,initial,ball_radius,ball_mass,air_density,drag_coefficient,gravity,angular_velocity)
    % 
    %         % velocities in x,y,z directions
    %         xdot = initial(4);
    %         ydot = initial(5);
    %         zdot = initial(6);
    %         velocity_magnitude = sqrt(xdot^2+ydot^2+zdot^2);
    % 
    %         % grouped constants
    %         A = (4/3) * (4 * pi^2 * ball_radius^3 * air_density);
    %         B = (1/2) * air_density * (pi * ball_radius^2) * drag_coefficient;
    %         C = ball_mass * gravity;
    % 
    %         % acceleration in x,y,z directions
    %         xddot = (A * (angular_velocity(2) * zdot - angular_velocity(3) * ydot) - B * velocity_magnitude * xdot) / ball_mass;
    %         yddot = (A * (angular_velocity(3) * xdot - angular_velocity(1) * zdot) - B * velocity_magnitude * ydot) / ball_mass;
    %         zddot = (A * (angular_velocity(1) * ydot - angular_velocity(2) * xdot) - B * velocity_magnitude * zdot - C) / ball_mass;
    % 
    %         states = [xdot;ydot;zdot;xddot;yddot;zddot];
    %     end   
    % 
    %     function [check, isterminal, direction] = reachesGround(~,r)
    %         check = r(3);       % value we want to be zero
    %         isterminal = 1;     % stop or not (1=stop,0=continue)
    %         direction = 0;      % either direction (-1=negative direction,1=positive direction,0=either)
    %     end
    % 
    % end

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            % Close settings panel
            app.SettingsPanel.Enable = false;
            app.SettingsPanel.Visible = false;  
            app.SettingsButton.Position = [541,457,100,23]; 

            % Variable initialization
            app.SettingsOpen = false;
            app.BallRadius = app.BallRadiusSpinner.Value / 100.0;
            app.BallMass = app.BallMassSpinner.Value;
            app.AirDensity = app.AirDensitySpinner.Value;
            app.DragCoefficient = app.DragCoefficientSpinner.Value;
            app.Gravity = app.GravitySpinner.Value;
            app.VectorType = "Velocity";
            app.VelocityVectorSliders = [0 45 30];
            app.AngularVectorSliders = [0 45 2];

            % Vector initialization
            app.VectorSwitch.Value = "Velocity";
            app.VectorLamp.Color = [0.00,0.45,0.74]; % blue
            app.HorizontalSliderField.Value = app.VelocityVectorSliders(1);
            app.HorizontalSlider.Value = app.VelocityVectorSliders(1);
            app.VerticalSliderField.Value = app.VelocityVectorSliders(2);
            app.VerticalSlider.Value = app.VelocityVectorSliders(2);
            app.MagnitudeKnob.Value = app.VelocityVectorSliders(3);
            app.MagnitudeField.Value = app.VelocityVectorSliders(3);

            % Set up graph
            ResetButtonPushed(app)

        end

        % Value changed function: SettingsButton
        function SettingsButtonValueChanged(app, event)
            if (app.SettingsOpen == true)
                app.SettingsPanel.Enable = false;
                app.SettingsPanel.Visible = false;  
                app.SettingsOpen = false;
                app.SettingsButton.Position = [541,457,100,23];
            else 
                app.SettingsPanel.Enable = true;
                app.SettingsPanel.Visible = true; 
                app.SettingsOpen = true;
                app.SettingsButton.Position = [541,379,100,23];
            end

        end

        % Value changed function: BallRadiusSpinner
        function BallRadiusSpinnerValueChanged(app, event)
            app.BallRadius = app.BallRadiusSpinner.Value / 100.0;
        end

        % Value changed function: BallMassSpinner
        function BallMassSpinnerValueChanged(app, event)
            app.BallMass = app.BallMassSpinner.Value;
        end

        % Value changed function: AirDensitySpinner
        function AirDensitySpinnerValueChanged(app, event)
            app.AirDensity = app.AirDensitySpinner.Value;
        end

        % Value changed function: GravitySpinner
        function GravitySpinnerValueChanged(app, event)
            app.Gravity = app.GravitySpinner.Value;
        end

        % Value changed function: DragCoefficientSpinner
        function DragCoefficientSpinnerValueChanged(app, event)
            app.DragCoefficient = app.DragCoefficientSpinner.Value;
        end

        % Value changed function: VectorSwitch
        function VectorSwitchValueChanged(app, event)
            if (app.VectorType == "Velocity")
                app.VectorLamp.Color = [0.64,0.08,0.18]; % red
                app.VectorSwitch.Value = "Angular Velocity";
                app.MagnitudeField.Limits = [0.0,10.0];
                app.MagnitudeKnob.Limits = [0.0,10.0];
                app.MagnitudeLabel.Text = "Magnitude (rev/s)";

                app.HorizontalSlider.Value = app.AngularVectorSliders(1);
                app.HorizontalSliderField.Value = app.AngularVectorSliders(1);

                app.VerticalSlider.Value = app.AngularVectorSliders(2);
                app.VerticalSliderField.Value = app.AngularVectorSliders(2);

                app.MagnitudeKnob.Value = app.AngularVectorSliders(3);
                app.MagnitudeField.Value = app.AngularVectorSliders(3);
            else
                app.VectorLamp.Color = [0.00,0.45,0.74]; % blue
                app.VectorSwitch.Value = "Velocity";
                app.MagnitudeField.Limits = [0.0,100.0];
                app.MagnitudeKnob.Limits = [0.0,100.0];
                app.MagnitudeLabel.Text = "Magnitude (m/s)";

                app.HorizontalSlider.Value = app.VelocityVectorSliders(1);
                app.HorizontalSliderField.Value = app.VelocityVectorSliders(1);
                
                app.VerticalSlider.Value = app.VelocityVectorSliders(2);
                app.VerticalSliderField.Value = app.VelocityVectorSliders(2);

                app.MagnitudeKnob.Value = app.VelocityVectorSliders(3);
                app.MagnitudeField.Value = app.VelocityVectorSliders(3);
            end
            app.VectorType = app.VectorSwitch.Value;
        end

        % Value changed function: MagnitudeKnob
        function MagnitudeKnobValueChanged(app, event)
            value = app.MagnitudeKnob.Value;
            app.MagnitudeKnob.Value = value;
            app.MagnitudeField.Value = value;
            adjustMagnitude(app,value);
        end

        % Value changed function: MagnitudeField
        function MagnitudeFieldValueChanged(app, event)
            value = app.MagnitudeField.Value;
            app.MagnitudeField.Value = value;
            app.MagnitudeKnob.Value = value;
            adjustMagnitude(app,value);
        end

        % Button pushed function: ProjectButton
        function ProjectButtonPushed(app, event)

            % Constants
            ball_radius = app.BallRadius;              % Ball radius [m]
            ball_mass = app.BallMass;                % Ball mass [kg]
            air_density = app.AirDensity;            % Air density [kg/m^3]
            drag_coefficient = app.DragCoefficient;         % Coefficient of drag 
            gravity = app.Gravity;                 % Gravity constant [m/s^2]
            angular_velocity = -get(app.AngularVector,'AutoScaleFactor') .* [get(app.AngularVector,'UData') get(app.AngularVector,'VData') get(app.AngularVector,'WData')];     % Angular velocity vector [rad/s] 

            % ODE45 parameters
            event_function = @(t,r) dynamics.reachesGround(t,r);
            options = odeset('Events',event_function,'RelTol',1e-6);
            time = linspace(0,100,10000);
            ball_position = [0 0 0];
            ball_velocity = get(app.VelocityVector,'AutoScaleFactor') .* [get(app.VelocityVector,'UData') get(app.VelocityVector,'VData') get(app.VelocityVector,'WData')];
            initial_conditions = [ball_position ball_velocity];
            ode_function = @(t,r) dynamics.sphere_model(t,r,ball_radius,ball_mass,air_density,drag_coefficient,gravity,angular_velocity);

            % solve ODE equations
            [tout,rout] = ode45(ode_function,time,initial_conditions,options);
            x = rout(:,1);
            y = rout(:,2);
            z = rout(:,3);
    
            % plot trajectory
            if (app.AutoAdjustViewCheckBox.Value == 1) 
                adjustView(app,x,y,z);
            end
            
            playbackSpeed = 10;
            delay = (tout(2) - tout(1)) / playbackSpeed;
            trajectory = line(app.UIAxes,x,y,z);
            sphere = plot3(app.UIAxes,x(1),y(1),z(1),'o', 'Color', 'k', 'MarkerFaceColor', 'g', 'MarkerSize', 10);
            for i=1:length(tout)
                set(trajectory, 'XData', x(1:i), 'YData', y(1:i), 'ZData', z(1:i));
                set(sphere, 'XData', x(i), 'YData', y(i), 'ZData', z(i));

                pause(delay);
                % java.lang.Thread.sleep(delay);
                % 
                % if (i ~= length(tout) - 100)
                %     delete(sphere);
                % end
            end

        end

        % Value changed function: HorizontalSlider
        function HorizontalSliderValueChanged(app, event)
            value = round(app.HorizontalSlider.Value);
            app.HorizontalSliderField.Value = value;
            app.HorizontalSlider.Value = value;
            if (app.VectorType == "Velocity")
                degrees = value - app.VelocityVectorSliders(1);
                rotateZAxis(app,app.VelocityVector,degrees);
                app.VelocityVectorSliders(1) = value;
            else 
                degrees = value - app.AngularVectorSliders(1);
                rotateZAxis(app,app.AngularVector,degrees);
                app.AngularVectorSliders(1) = value;
            end
            if (app.AutoAdjustViewCheckBox.Value == 1) 
                adjustView(app,0,0,0);
            end
        end

        % Value changed function: HorizontalSliderField
        function HorizontalSliderFieldValueChanged(app, event)
            value = round(app.HorizontalSliderField.Value);
            app.HorizontalSliderField.Value = value;
            app.HorizontalSlider.Value = value;
            if (app.VectorType == "Velocity")
                degrees = value - app.VelocityVectorSliders(1);
                rotateZAxis(app,app.VelocityVector,degrees);
                app.VelocityVectorSliders(1) = value;
            else 
                degrees = value - app.AngularVectorSliders(1);
                rotateZAxis(app,app.AngularVector,degrees);
                app.AngularVectorSliders(1) = value;
            end
            if (app.AutoAdjustViewCheckBox.Value == 1) 
                adjustView(app,0,0,0);
            end
        end

        % Value changed function: VerticalSliderField
        function VerticalSliderFieldValueChanged(app, event)
            value = round(app.VerticalSliderField.Value);
            app.VerticalSliderField.Value = value;
            app.VerticalSlider.Value = value;
            if (app.VectorType == "Velocity")
                elevate(app,app.VelocityVector,value);
                app.VelocityVectorSliders(2) = value;
            else 
                elevate(app,app.AngularVector,value);
                app.AngularVectorSliders(2) = value;
            end
            if (app.AutoAdjustViewCheckBox.Value == 1) 
                adjustView(app,0,0,0);
            end
        end

        % Value changed function: VerticalSlider
        function VerticalSliderValueChanged(app, event)
            value = round(app.VerticalSlider.Value);
            app.VerticalSliderField.Value = value;
            app.VerticalSlider.Value = value;
            if (app.VectorType == "Velocity")
                elevate(app,app.VelocityVector,value);
                app.VelocityVectorSliders(2) = value;
            else 
                elevate(app,app.AngularVector,value);
                app.AngularVectorSliders(2) = value;
            end
            if (app.AutoAdjustViewCheckBox.Value == 1) 
                adjustView(app,0,0,0);
            end
        end

        % Button pushed function: ResetButton
        function ResetButtonPushed(app, event)
            cla(app.UIAxes,'reset')

            % Graph initialization
            graph = app.UIAxes;
            hold(graph,'on');
            grid(graph,'on');
            axis(graph,'equal');
            view(graph,3);
            set(graph,'FontName','Verdana','FontSize',12)
            plot3(graph,0,0,0,"Color",[0.49,0.49,0.49],"MarkerSize",app.BallRadius,"Marker","o","MarkerFaceColor",[0.49,0.49,0.49]);
            title(graph,"");
            xlabel(graph,'x [m]');
            ylabel(graph,'y [m]');
            zlabel(graph,'z [m]');

            % create initial vector
            app.VelocityVectorSliders = [0 45 30];
            app.AngularVectorSliders = [0 45 2];
            app.VelocityVector = quiver3(graph,0,0,0,1,1,1,app.VelocityVectorSliders(3),"-","LineWidth",4,"Color",[0.00,0.45,0.74]);
            app.AngularVector = quiver3(graph,0,0,0,1,1,1,app.AngularVectorSliders(3),"-","LineWidth",4,"Color",[0.64,0.08,0.18]);
            adjustView(app,0,0,0);  
            app.HorizontalSliderField.Value = app.VelocityVectorSliders(1);
            app.HorizontalSlider.Value = app.VelocityVectorSliders(1);
            app.VerticalSliderField.Value = app.VelocityVectorSliders(2);
            app.VerticalSlider.Value = app.VelocityVectorSliders(2);
            if (app.VectorType == "Velocity")
                app.MagnitudeKnob.Value = app.VelocityVectorSliders(3);
                app.MagnitudeField.Value = app.VelocityVectorSliders(3);
            else
                app.MagnitudeKnob.Value = app.AngularVectorSliders(3);
                app.MagnitudeField.Value = app.AngularVectorSliders(3);
            end

            % ground plane
            [x,y,z] = meshgrid(-300:300,-300:300,0);
            surface = surf(app.UIAxes,x,y,z);
            set(surface,'FaceColor',[0.286, 0.58, 0.243],'FaceAlpha',0.5,'FaceLighting','gouraud','EdgeColor','none')
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.AutoResizeChildren = 'off';
            app.UIFigure.Position = [100 100 860 480];
            app.UIFigure.Name = 'MATLAB App';
            app.UIFigure.Resize = 'off';

            % Create MainPanel
            app.MainPanel = uipanel(app.UIFigure);
            app.MainPanel.AutoResizeChildren = 'off';
            app.MainPanel.Position = [1 1 640 480];

            % Create UIAxes
            app.UIAxes = uiaxes(app.MainPanel);
            title(app.UIAxes, 'Title')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.Position = [122 122 485 299];

            % Create VerticalSlider
            app.VerticalSlider = uislider(app.MainPanel);
            app.VerticalSlider.Limits = [0 90];
            app.VerticalSlider.MajorTicks = [0 30 60 90];
            app.VerticalSlider.MajorTickLabels = {'0', '30', '60', '90'};
            app.VerticalSlider.Orientation = 'vertical';
            app.VerticalSlider.ValueChangedFcn = createCallbackFcn(app, @VerticalSliderValueChanged, true);
            app.VerticalSlider.MinorTicks = [0 15 30 45 60 75 90];
            app.VerticalSlider.Position = [48 177 3 231];

            % Create SettingsPanel
            app.SettingsPanel = uipanel(app.MainPanel);
            app.SettingsPanel.AutoResizeChildren = 'off';
            app.SettingsPanel.Position = [1 401 639 79];

            % Create GridLayout
            app.GridLayout = uigridlayout(app.SettingsPanel);
            app.GridLayout.ColumnWidth = {'1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x', '1x'};

            % Create BallRadiusSpinner
            app.BallRadiusSpinner = uispinner(app.GridLayout);
            app.BallRadiusSpinner.Limits = [0 50];
            app.BallRadiusSpinner.ValueChangedFcn = createCallbackFcn(app, @BallRadiusSpinnerValueChanged, true);
            app.BallRadiusSpinner.HorizontalAlignment = 'center';
            app.BallRadiusSpinner.Layout.Row = 1;
            app.BallRadiusSpinner.Layout.Column = 3;
            app.BallRadiusSpinner.Value = 11;

            % Create BallRadiusLabel
            app.BallRadiusLabel = uilabel(app.GridLayout);
            app.BallRadiusLabel.Layout.Row = 1;
            app.BallRadiusLabel.Layout.Column = [1 2];
            app.BallRadiusLabel.Text = 'Ball Radius (cm)';

            % Create BallMassSpinner
            app.BallMassSpinner = uispinner(app.GridLayout);
            app.BallMassSpinner.Step = 0.01;
            app.BallMassSpinner.Limits = [0 5];
            app.BallMassSpinner.ValueChangedFcn = createCallbackFcn(app, @BallMassSpinnerValueChanged, true);
            app.BallMassSpinner.HorizontalAlignment = 'center';
            app.BallMassSpinner.Layout.Row = 2;
            app.BallMassSpinner.Layout.Column = 3;
            app.BallMassSpinner.Value = 0.425;

            % Create BallMassLabel
            app.BallMassLabel = uilabel(app.GridLayout);
            app.BallMassLabel.Layout.Row = 2;
            app.BallMassLabel.Layout.Column = [1 2];
            app.BallMassLabel.Text = 'Ball Mass (kg)';

            % Create AirDensitySpinner
            app.AirDensitySpinner = uispinner(app.GridLayout);
            app.AirDensitySpinner.Step = 0.1;
            app.AirDensitySpinner.Limits = [0 2];
            app.AirDensitySpinner.ValueChangedFcn = createCallbackFcn(app, @AirDensitySpinnerValueChanged, true);
            app.AirDensitySpinner.HorizontalAlignment = 'center';
            app.AirDensitySpinner.Layout.Row = 1;
            app.AirDensitySpinner.Layout.Column = 6;
            app.AirDensitySpinner.Value = 1.225;

            % Create AirDensityLabel
            app.AirDensityLabel = uilabel(app.GridLayout);
            app.AirDensityLabel.Layout.Row = 1;
            app.AirDensityLabel.Layout.Column = [4 5];
            app.AirDensityLabel.Text = 'Air Density (kg/m^3)';

            % Create GravitySpinner
            app.GravitySpinner = uispinner(app.GridLayout);
            app.GravitySpinner.Step = 0.001;
            app.GravitySpinner.Limits = [9.8 9.81];
            app.GravitySpinner.ValueChangedFcn = createCallbackFcn(app, @GravitySpinnerValueChanged, true);
            app.GravitySpinner.HorizontalAlignment = 'center';
            app.GravitySpinner.Layout.Row = 1;
            app.GravitySpinner.Layout.Column = 9;
            app.GravitySpinner.Value = 9.807;

            % Create GravityLabel
            app.GravityLabel = uilabel(app.GridLayout);
            app.GravityLabel.Layout.Row = 1;
            app.GravityLabel.Layout.Column = [7 8];
            app.GravityLabel.Text = 'Gravity (m/s^2)';

            % Create DragCoefficientSpinner
            app.DragCoefficientSpinner = uispinner(app.GridLayout);
            app.DragCoefficientSpinner.Step = 0.1;
            app.DragCoefficientSpinner.Limits = [0 1];
            app.DragCoefficientSpinner.ValueChangedFcn = createCallbackFcn(app, @DragCoefficientSpinnerValueChanged, true);
            app.DragCoefficientSpinner.HorizontalAlignment = 'center';
            app.DragCoefficientSpinner.Layout.Row = 2;
            app.DragCoefficientSpinner.Layout.Column = 6;
            app.DragCoefficientSpinner.Value = 0.25;

            % Create DragCoefficientLabel
            app.DragCoefficientLabel = uilabel(app.GridLayout);
            app.DragCoefficientLabel.Layout.Row = 2;
            app.DragCoefficientLabel.Layout.Column = [4 5];
            app.DragCoefficientLabel.Text = 'Drag Coefficient';

            % Create AutoAdjustViewCheckBox
            app.AutoAdjustViewCheckBox = uicheckbox(app.GridLayout);
            app.AutoAdjustViewCheckBox.Text = 'Auto Adjust View';
            app.AutoAdjustViewCheckBox.WordWrap = 'on';
            app.AutoAdjustViewCheckBox.Layout.Row = [1 2];
            app.AutoAdjustViewCheckBox.Layout.Column = 10;
            app.AutoAdjustViewCheckBox.Value = true;

            % Create SettingsButton
            app.SettingsButton = uibutton(app.MainPanel, 'state');
            app.SettingsButton.ValueChangedFcn = createCallbackFcn(app, @SettingsButtonValueChanged, true);
            app.SettingsButton.Text = 'Settings';
            app.SettingsButton.Position = [541 379 100 23];

            % Create HorizontalSlider
            app.HorizontalSlider = uislider(app.MainPanel);
            app.HorizontalSlider.Limits = [-180 180];
            app.HorizontalSlider.MajorTicks = [-180 -120 -60 0 60 120 180];
            app.HorizontalSlider.MajorTickLabels = {'-180', '-120', '-60', '0', '60', '120', '180'};
            app.HorizontalSlider.ValueChangedFcn = createCallbackFcn(app, @HorizontalSliderValueChanged, true);
            app.HorizontalSlider.MinorTicks = [-180 -150 -120 -90 -60 -30 0 30 60 90 120 150 180];
            app.HorizontalSlider.Position = [210 87 275 3];

            % Create HorizontalRotationLabel
            app.HorizontalRotationLabel = uilabel(app.MainPanel);
            app.HorizontalRotationLabel.HorizontalAlignment = 'right';
            app.HorizontalRotationLabel.Position = [297 24 64 22];
            app.HorizontalRotationLabel.Text = 'Azimuth (°)';

            % Create HorizontalSliderField
            app.HorizontalSliderField = uieditfield(app.MainPanel, 'numeric');
            app.HorizontalSliderField.Limits = [-180 180];
            app.HorizontalSliderField.ValueChangedFcn = createCallbackFcn(app, @HorizontalSliderFieldValueChanged, true);
            app.HorizontalSliderField.HorizontalAlignment = 'center';
            app.HorizontalSliderField.Position = [364 24 47 22];

            % Create HorizontalRotationLabel_2
            app.HorizontalRotationLabel_2 = uilabel(app.MainPanel);
            app.HorizontalRotationLabel_2.HorizontalAlignment = 'right';
            app.HorizontalRotationLabel_2.Position = [2 133 70 22];
            app.HorizontalRotationLabel_2.Text = 'Elevation (°)';

            % Create VerticalSliderField
            app.VerticalSliderField = uieditfield(app.MainPanel, 'numeric');
            app.VerticalSliderField.Limits = [-180 180];
            app.VerticalSliderField.ValueChangedFcn = createCallbackFcn(app, @VerticalSliderFieldValueChanged, true);
            app.VerticalSliderField.HorizontalAlignment = 'center';
            app.VerticalSliderField.Position = [75 133 47 22];

            % Create ControlsPanel
            app.ControlsPanel = uipanel(app.UIFigure);
            app.ControlsPanel.AutoResizeChildren = 'off';
            app.ControlsPanel.Position = [640 1 220 480];

            % Create MagnitudeLabel
            app.MagnitudeLabel = uilabel(app.ControlsPanel);
            app.MagnitudeLabel.HorizontalAlignment = 'center';
            app.MagnitudeLabel.Position = [39 203 107 22];
            app.MagnitudeLabel.Text = 'Magnitude (m/s)';

            % Create MagnitudeKnob
            app.MagnitudeKnob = uiknob(app.ControlsPanel, 'continuous');
            app.MagnitudeKnob.ValueChangedFcn = createCallbackFcn(app, @MagnitudeKnobValueChanged, true);
            app.MagnitudeKnob.MinorTicks = [];
            app.MagnitudeKnob.Position = [80 259 60 60];
            app.MagnitudeKnob.Value = 10;

            % Create VectorSwitch
            app.VectorSwitch = uiswitch(app.ControlsPanel, 'slider');
            app.VectorSwitch.Items = {'Velocity', 'Angular Velocity'};
            app.VectorSwitch.Orientation = 'vertical';
            app.VectorSwitch.ValueChangedFcn = createCallbackFcn(app, @VectorSwitchValueChanged, true);
            app.VectorSwitch.Position = [57 402 20 45];
            app.VectorSwitch.Value = 'Velocity';

            % Create VectorLamp
            app.VectorLamp = uilamp(app.ControlsPanel);
            app.VectorLamp.Position = [141 414 20 20];
            app.VectorLamp.Color = [0 0.451 0.7412];

            % Create MagnitudeField
            app.MagnitudeField = uieditfield(app.ControlsPanel, 'numeric');
            app.MagnitudeField.Limits = [0 100];
            app.MagnitudeField.ValueChangedFcn = createCallbackFcn(app, @MagnitudeFieldValueChanged, true);
            app.MagnitudeField.HorizontalAlignment = 'center';
            app.MagnitudeField.Position = [145 203 32 22];
            app.MagnitudeField.Value = 10;

            % Create ProjectButton
            app.ProjectButton = uibutton(app.ControlsPanel, 'push');
            app.ProjectButton.ButtonPushedFcn = createCallbackFcn(app, @ProjectButtonPushed, true);
            app.ProjectButton.BackgroundColor = [0.4196 0.7686 0.1686];
            app.ProjectButton.Position = [77 128 67 23];
            app.ProjectButton.Text = 'Project';

            % Create ResetButton
            app.ResetButton = uibutton(app.ControlsPanel, 'push');
            app.ResetButton.ButtonPushedFcn = createCallbackFcn(app, @ResetButtonPushed, true);
            app.ResetButton.BackgroundColor = [0.8118 0.3412 0.3412];
            app.ResetButton.Position = [87 62 46 23];
            app.ResetButton.Text = 'Reset';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = app

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end