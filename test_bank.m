%Test battery - test the algorithms on different data.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Include Paths
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
addpath('./lib/mds_map/');
addpath('./lib/LMFnlsq/');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Loops of Tests
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%We loop over every combination of these
test_files = {'pre-test-data1.mat'};
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
        fprintf(', dist_cost: %f', costs.distances);
        fprintf(', ang_cost: %f', costs.angles);
        fprintf(', sum_cost: %f', costs.sum);
        
        %Ground truth error
        error_pos = calculate_error_positions(ground_truth, ...
                                              estimated_cameras);
        fprintf(', pos_error: %f', error_pos);

        error_angles = calculate_error_angles(ground_truth, ...
                                              estimated_cameras);
        fprintf(', ang_error: %f', error_angles);
        fprintf(', sum_error: %f', error_pos + error_angles);
        
        fprintf('\n');
    end
end


