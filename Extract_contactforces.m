function Extract_contactforces(main_path, model_path, solution_folder)
%Extracts the individual contact forces and the total contact forces. Saves
%them in your solution_folder.
%Gives 'CF_i': the location of the contact force acting on each
%contact sphere.
%CF_1: at the calcaneus 
%CF_2: at medial forefoot
%CF_3: at lateral forefoot
%CF_4: at the midfoot
%CF_5: at the medial toes
%CF_6: at the lateral toes
%Column 1: time. Column 2-4: X, Y and Z value of the
%location for contact sphere 1, Column 5-6: for contact sphere 2, ... so on
%for 6 contact spheres
%main_path: path where all the files regarding Moco are saved
%model_path: full path to the model file, '\model.osim'.
%solution_folder: path to the folder where the solution is stored.


%% Extract contact forces 
% ==================================
% based on example 2D walking, code T. Ludovica and Moco forum

% INDIVIDUAL contact forces
mkdir([solution_folder, 'contactforces\']);

import org.opensim.modeling.*;

prevSolution = MocoTrajectory([solution_folder, 'solution.sto']); % Load previous solution into memory
prevStatesTable = prevSolution.exportToStatesTable(); % Extract previous solution's states

modelProcessor = ModelProcessor(model_path);
model = Model(model_path);
model.initSystem();

numCoord = model.updCoordinateSet().getSize();
for i = 0:numCoord-1
    model.updCoordinateSet().get(i).setDefaultLocked(false);
end

statesTraj = StatesTrajectory(); % Create an empty trajectory of states.
states = statesTraj.createFromStatesTable(model, prevStatesTable);

for i = 1:6
     
    contact_r = StdVectorString();
    contact_l = StdVectorString();

    SSHSF_i = strcat('/forceset/SmoothSphereHalfSpaceForce_s', num2str(i), '_r');

    contact_r.add(SSHSF_i);

    model = modelProcessor.process();

    externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait(model, ...
    states, contact_r, contact_l);

    % Create a .sto file of the contactforce
    contactforce_sto = strcat(solution_folder, 'contactforces\CF_', num2str(i), '.sto');
    STOFileAdapter.write(externalForcesTableFlat, contactforce_sto); 

end


% TOTAL contact forces 
import org.opensim.modeling.*;

prevSolution = MocoTrajectory([solution_folder, 'solution.sto']); % Load previous solution into memory
prevStatesTable = prevSolution.exportToStatesTable(); % Extract previous solution's states

model = Model(model_path);
model.initSystem();

numCoord = model.updCoordinateSet().getSize();
for i = 0:numCoord-1
    model.updCoordinateSet().get(i).setDefaultLocked(false);
end

statesTraj = StatesTrajectory(); % Create an empty trajectory of states.
states = statesTraj.createFromStatesTable(model, prevStatesTable);

contact_r = StdVectorString();
contact_l = StdVectorString();

contact_r.add('/forceset/SmoothSphereHalfSpaceForce_s1_r');
contact_r.add('/forceset/SmoothSphereHalfSpaceForce_s2_r');
contact_r.add('/forceset/SmoothSphereHalfSpaceForce_s3_r');
contact_r.add('/forceset/SmoothSphereHalfSpaceForce_s4_r');
contact_r.add('/forceset/SmoothSphereHalfSpaceForce_s5_r');
contact_r.add('/forceset/SmoothSphereHalfSpaceForce_s6_r');

model = modelProcessor.process();
externalForcesTableFlat = opensimMoco.createExternalLoadsTableForGait(model, ...
    states, contact_r, contact_l);

% Create a .sto file of the contactforce
contactforce_sto = strcat(solution_folder, 'contactforces\CF_total.sto');
STOFileAdapter.write(externalForcesTableFlat, contactforce_sto); 


end