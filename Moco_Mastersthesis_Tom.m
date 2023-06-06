clc
clear
close all

%% Load the Moco libraries
% ==================================
import org.opensim.modeling.*;

%% Define in which folder the outputs should be saved
% In the end, this output folder will contain:
% '_tracked_states.sto' file with the input kinematics to track
% 'solution.sto' file with the kinematics of the solution and the actuator values
% A copy of the used footmodel
% The geometries of the model (in order to visualize them in OpenSim)
% A copy of the MatLab script used
% 'Moco_setup.xml' with the settings used for the optimal control problem
% A folder named 'contactforces' with the total and the individual contact forces
% ==================================
solution_name = 'Name_solutionfolder\';

%% Define the foot model and the footplate motion
% ==================================
model_name = 'Name_model.osim';
platemotion_name = 'Name_platemotion.sto';

%% Define the locations of the model and the platemotion
% ==================================
main_path = 'C:\...\Moco\';
model_path = [main_path, 'Model\', model_name];
platemotion_path = [main_path, 'Plate_motion\', platemotion_name];
solution_folder = [main_path, 'Output\', solution_name];

%% Set the weight for the cost function and the maximum number of iterations
% ==================================
trackweight = 5000;
max_num_iter = 10000;
t_final = 11.1;

%% Make a new folder in which the outputs will be saved
% ==================================
mkdir(solution_folder);

% Save the model file and the geometries in the solution folder
copyfile(model_path, solution_folder)
copyfile([main_path, 'Models\Current\Geometry\'], [solution_folder, '\Geometry'])
copyfile([main_path, 'Track_full_model\Moco_GitHub.m'], solution_folder)

%% Moco Track
% ==================================
% Create and name an instance of the MocoTrack tool.
track = MocoTrack();
track.setName([main_path, 'Output\', solution_name]);

% Create the base Model by passing in the model file.
modelProcessor = ModelProcessor(model_path);
track.setModel(modelProcessor);

% Reference data for tracking problem
tableProcessor = TableProcessor(platemotion_path);
track.setStatesReference(tableProcessor);

% MocoProblem settings
track.set_states_global_tracking_weight(trackweight); % The weight for the MocoStateTrackingGoal that applies to tracking
track.set_track_reference_position_derivatives(false);
track.set_apply_tracked_states_to_guess(true); % replace the states in the guess with the states reference data.
track.set_initial_time(0);
track.set_final_time(t_final); 
track.set_minimize_control_effort(0); % Whether or not to minimize actuator control effort in the problem.

% Solve it
study = track.initialize();

%% Define the initial position
% ==================================
problem = study.updProblem();

problem.setStateInfo('/jointset/ground_tibia_r/tibia_r_ty/value', [], -0.0165199305467897)

problem.setStateInfo('/jointset/Ankle_r/ankle_pfdf_r/value', [], 5.74356333400974*pi/180)

problem.setStateInfo('/jointset/Subtalar_r/subtalar_inev_r/value', [], -7.97734031076475*pi/180)

problem.setStateInfo('/jointset/Chopart_r/chopart_obl_r/value', [],  0.414942641114848*pi/180)
problem.setStateInfo('/jointset/Chopart_r/chopart_antpos_r/value', [], -4.62302305113618*pi/180)

problem.setStateInfo('/jointset/ground_footplate/plate_tilt/value', [], 2.2928*pi/180)
problem.setStateInfo('/jointset/ground_footplate/plate_list/value', [], 5.1020*pi/180)

%% Solver settings
% ==================================
%Default settings of MocoTrack, except for num_mesh_intervals and max_iterations
solver = study.initCasADiSolver();
solver.set_num_mesh_intervals(t_final/0.2);
solver.set_multibody_dynamics_mode('explicit');
solver.set_optim_convergence_tolerance(0.01);
solver.set_optim_constraint_tolerance(0.01);
solver.set_optim_finite_difference_scheme('forward')
solver.set_optim_max_iterations(max_num_iter);

%% Save a setup file of the study
% ==================================
study.print([main_path, 'Output\', solution_name, 'Moco_setup.xml']);

%% Solve
% ==================================
solution = study.solve();
%solution.unseal();
solution.write([main_path, 'Output\' solution_name, 'solution.sto']);

%% Extract the contact forces 
% ==================================
Extract_contactforces(main_path, model_path, solution_folder);
