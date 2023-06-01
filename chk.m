clc;
clear all;
close all;

data = csvread("data.csv");

torq_data = data(:,6);

a = 0.4;
torq_data_new = zeros(length(torq_data),1);
num = 20;
data_vec = zeros(num,1);

for cnt =num:length(torq_data)-1
  data_vec(num) = torq_data(cnt);
  torq_data_new(cnt,1) = mean(data_vec);
  data_vec(1:num-1) = data_vec(2:num);
end

figure(1)
plot(data(:,4))
hold on
plot(torq_data_new, 'r')
grid minor
