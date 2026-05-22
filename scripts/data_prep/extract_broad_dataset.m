% extract_broad_dataset.m
% 
% This script processes the raw BROAD dataset .mat files to extract only the 
% essential variables required for attitude observer testing: 
% imu_gyr, imu_acc, imu_mag, and opt_quat.
%
% This reduces file size while remaining fully compatible with the 
% original demo_SO3Observer_BROAD_unbiased.m data loading paradigm.

clear; clc; close all;

% Directories
raw_dir = '../../datasets/BROAD/raw/';
proc_dir = '../../datasets/BROAD/processed/';

% Ensure processed directory exists
if ~exist(proc_dir, 'dir')
    mkdir(proc_dir);
end

% Get all .mat files in the raw directory
mat_files = dir(fullfile(raw_dir, '*.mat'));

fprintf('Found %d raw data files to process.\n', length(mat_files));

for i = 1:length(mat_files)
    file_name = mat_files(i).name;
    raw_path = fullfile(raw_dir, file_name);
    proc_path = fullfile(proc_dir, file_name);
    
    fprintf('Processing: %s ... ', file_name);
    
    try
        % Load raw data
        data = load(raw_path);
        
        % Extract only the necessary variables
        % Check if variables exist to avoid errors
        if isfield(data, 'imu_gyr') && isfield(data, 'imu_acc') && ...
           isfield(data, 'imu_mag') && isfield(data, 'opt_quat')
       
            imu_gyr = data.imu_gyr;
            imu_acc = data.imu_acc;
            imu_mag = data.imu_mag;
            opt_quat = data.opt_quat;
            
            % Save to processed directory
            save(proc_path, 'imu_gyr', 'imu_acc', 'imu_mag', 'opt_quat');
            fprintf('Saved (%.2f MB)\n', mat_files(i).bytes / 1e6);
        else
            fprintf('FAILED (Missing required variables)\n');
        end
        
    catch ME
        fprintf('ERROR (%s)\n', ME.message);
    end
end

fprintf('Data extraction completed.\n');
