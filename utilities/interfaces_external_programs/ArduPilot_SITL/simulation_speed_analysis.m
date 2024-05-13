function simulation_speed_analysis(sample_time, sitl_frame, print_frame_count)
    coder.extrinsic('tic');
    coder.extrinsic('toc');
    
    persistent system_time
    
    if isempty(system_time)
        system_time = tic;
    end
    
    if rem(sitl_frame, print_frame_count) == 0
        total_system_time = toc(system_time);
        total_sim_time = print_frame_count * sample_time;
        
        fprintf("%4.0f fps, %3.0f %% of realtime\n", print_frame_count/total_system_time, total_sim_time/total_system_time * 1e2);        
        
        system_time = tic;
    end
end
