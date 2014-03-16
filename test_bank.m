%Test battery - test the algorithms on different data.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Include Paths
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath('./lib/mds_map/');
addpath('./lib/LMFnlsq/');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Loops of Tests
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%We loop over every combination of these, if we choose this bank
test_files_bank_1 = {...
              'test_bank/simple_sim_cc1_n0_ocp_1p0_50t.mat'; %noiseless example
              'test_bank/simple_sim_cc1_ncf0_on1_ocp_1p0_400t.mat'; %noisy straight path, no false correspondences
              'test_bank/simple_sim_cc1_n0p2_ocp_1p0_400t.mat'; %low noise / curvy path, no false correspondences
              'test_bank/simple_sim_cc1_n1_ocp_1p0_400t.mat'; %higher noisy / curvy path, no false correspondences
              'test_bank/simple_sim_cc1_n1_ocp_0p3_0p7_1p0_100t.mat';  %Standard - 100 tracks
              'test_bank/simple_sim_cc1_n1_ocp_0p3_0p7_1p0_400t.mat';  %Standard - 400 tracks
              'test_bank/simple_sim_cc1_n1_ocp_0p3_0p7_1p0_1600t.mat'; %Standard - 1600 tracks
              'test_bank/simple_sim_cc1_n1_ocp_upto10_1600t.mat'; %Up to 10 tracks in same window
};

test_files_bank_2 = {
              'test_bank/simple_sim_cc1_n1_ocp_upto1_0p8decay_1600t.mat';
              'test_bank/simple_sim_cc1_n1_ocp_upto2_0p8decay_1600t.mat';
              'test_bank/simple_sim_cc1_n1_ocp_upto3_0p8decay_1600t.mat';
              'test_bank/simple_sim_cc1_n1_ocp_upto4_0p8decay_1600t.mat';
              'test_bank/simple_sim_cc1_n1_ocp_upto5_0p8decay_1600t.mat';
              'test_bank/simple_sim_cc1_n1_ocp_upto6_0p8decay_1600t.mat';
              'test_bank/simple_sim_cc1_n1_ocp_upto7_0p8decay_1600t.mat';
              %'test_bank/simple_sim_cc1_n1_ocp_upto8_0p8decay_1600t.mat';
              %'test_bank/simple_sim_cc1_n1_ocp_upto9_0p8decay_1600t.mat';
              %'test_bank/simple_sim_cc1_n1_ocp_upto10_0p8decay_1600t.mat';
};

%Choose a bank!
test_files = test_files_bank_2;

test_algos = {'MDS-MAP', 'SDP', 'LM-nllsq-PaA'};

for tf=1:length(test_files)
    test_file = test_files{tf};
    %Load the variables 'correspondences' and 'ground_truth'
    clear correspondences
    clear ground_truth
    load(test_file);

	for ta=1:length(test_algos)
        algo = test_algos{ta};
        fprintf('File: %s, Algorithm: %s', test_file, algo);
    
        %Find a solution using this method
        [estimated_cameras, costs, camera_relation_votes_and_centers] ...
            = solve_cameras_extcal(correspondences, algo);

        %Connectivity Information
        [ connections, potential_connections ] ...
                = calculate_camera_connectivity_factors(...
                        camera_relation_votes_and_centers);
        fprintf(', connections: %.0f/%.0f', connections,...
                                            potential_connections);
        
        %Cost Functions
        if ~isempty(estimated_cameras)
            fprintf(', dist_cost: %f', costs.distances);
            fprintf(', ang_cost: %f', costs.angles);
            fprintf(', sum_cost: %f', costs.sum);
        else
            fprintf(', dist_cost: N/A');
            fprintf(', ang_cost: N/A');
            fprintf(', sum_cost: N/A');
        end
        
        %Ground truth error
        if ~isempty(estimated_cameras)
            error_pos = calculate_error_positions(ground_truth, ...
                                                  estimated_cameras);
            fprintf(', pos_error: %f', error_pos);

            error_angles = calculate_error_angles(ground_truth, ...
                                                  estimated_cameras);
            fprintf(', ang_error: %f', error_angles);
            fprintf(', sum_error: %f', error_pos + error_angles);
        else
            fprintf(', pos_error: N/A');
            fprintf(', ang_error: N/A');
            fprintf(', sum_error: N/A');
        end
        fprintf('\n');
    end
end


