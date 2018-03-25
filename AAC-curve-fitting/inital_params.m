%define range
data_alt = xlsread("deployed_data_sim.xlsx", 'Y121:Y503');
data_t = xlsread("deployed_data_sim.xlsx", 'T121:T503');

%assign max alt and t
[max_val, max_index] = max(data_alt(:));
max_alt_t = data_t(max_index);

%print out max values
disp(max_val);
disp(max_index);
disp(max_alt_t);

