%% Init
clc
clear
close all
syms u

%% Thruster Function (input (-1..1) to thrustforce in Newton for one thruster
T(u) = -140.25*u^9+389.9*u^7-404.1*u^5+175.95*u^3+8.935*u;

%% Parameters for thruster_manager.yaml
pwm = false; %PWM (1100..1900) or Input (-1..1)
% Stepsize for Thruster function (Also used in thruster_manager.yaml)
stepsize = 0.01;
% Paramters
model_name = "bluerov2_heavy" %'bluerov' or 'bluerov2_heavy'
maxThrust = 60; %Maximum thrust force for one thruster. (N)
updateRate = 50; %Update rate for thruster manager. (Hz)

%% Plot the Thruster Function (T(u)).
if(pwm == true)
    u = 1500+400*[-1:stepsize:1];
    xlab = "PWM (-)";
else
    u = [-1:stepsize:1];
    xlab = "Input (-)";
end
plot(u,T(-1:stepsize:1))
ylabel("Force (n)")
xlabel(xlab)
yline(0)


%% Write new thruster_manager.yaml
clc
s = "";
s = s + sprintf("thruster_manager:");
s = s + sprintf("\n  tf_prefix: bluerov2\n  base_link: base_link\n  thruster_topic_prefix: thrusters/\n  thruster_topic_suffix: /input\n  thruster_frame_base: thruster_");
s = s + sprintf("\n  max_thrust: %i #Maximum thrust in Newton produced by one thruster",maxThrust);
s = s + sprintf("\n  timeout: -1\n  update_rate: %i",updateRate);
s = s + sprintf("\n  ##################################################\n  # Options to set the thruster models below\n  ##################################################");
s = s + sprintf("\n\n  # You can set the conversion function to be:\n  conversion_fcn: custom\n  conversion_fcn_params:");

s = s + sprintf("\n    input: [");
if(pwm == true)
    s = s + sprintf("%.3f, ",1500+400*[-1:stepsize:1]);
else
    s = s + sprintf("%.3f, ",[-1:stepsize:1]);
end
s = s + sprintf("]\n    output: [");
s = s + sprintf("%.3f, ",T(-1:stepsize:1));
s = s + sprintf("]\n");

overwrite = false;
if(isfile(sprintf("config/%s/thruster_manager.yaml",model_name)))
    ans = input("A Yaml file already exists, do you want to overwrite it? [yN]: ","s");
    if(ans == "y") || (ans == "Y")
       overwrite = true; 
    else
        fprintf("Did NOT overwrite the file")
    end
end

if(not(isfile(sprintf("config/%s/thruster_manager.yaml",model_name)))) || (overwrite)
    fileID = fopen(sprintf("config/%s/thruster_manager.yaml",model_name),'w');
    fprintf(fileID,s);
    fclose(fileID);
    fprintf("A new Yaml file is generated\n")
end
fprintf("Done\n");