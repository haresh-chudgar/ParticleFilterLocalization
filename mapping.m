M = csvread('map.txt');
%particles = csvread('build/particleFilteroutput_v0_Full_CyclicResampling.csv');
particles = csvread('build/particleFilteroutput_v0_Full_3.csv');
%particles_NoSampling = csvread('particleFilteroutput_Sampling_2.csv');
%particles = particles(:,1:2);
%particles_NoSampling = particles_NoSampling(:,1:2);
%figure;
x = [M(:,1) M(:,3)];
y = [M(:,2) M(:,4)];
%plot(x',y');
%plot(x',y', 'b', particles(:,1), particles(:,2), 'r*', particles_NoSampling(:,1), particles_NoSampling(:,2), 'g*');
figure;
plot(x',y', 'b', particles(500*300:600*300,1), particles(500*300:600*300,2), 'r*', 'markers', 2);