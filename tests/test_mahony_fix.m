% test_mahony_fix.m
clear; clc;
addpath(genpath('../src'));

% Run compare_lcss_mekf_observers.m but modify lcss_mahony_attitude_observer 
% to omit v2 temporarily
% Instead of modifying the source, let's just make a copy here and run it

disp('Test Mahony Fix by Omitting unobservable v2 axis');
