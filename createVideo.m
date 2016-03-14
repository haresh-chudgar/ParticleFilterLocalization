M = csvread('map.txt');
x = [M(:,1) M(:,3)];
y = [M(:,2) M(:,4)];

particles = csvread('build/particleFilteroutput_v0_Full_CyclicResampling.csv');
numberOfParticles = (size(particles,1) / 300) - 1

fId = figure;
disp('loop starting');

writerObj = VideoWriter(sprintf('build/CyclicResampling/particleFilteroutput_v0_Full_CyclicResampling_%d.avi', 1001));
writerObj.FrameRate = 10;
open(writerObj);

j = 1001*300;
for i = 1001:numberOfParticles
    
    figure(fId);
    plot(x',y', 'b', particles(j:j+299,1), particles(j:j+299,2), 'r*');
    j = j + 300;
    frame = getframe(gcf);
    writeVideo(writerObj, frame); 
    fprintf('Loop:%d\n',i);
    
    if(mod(i, 500) == 0)
        close(writerObj);
        fprintf('Writing to particleFilteroutput_v0_Full_CyclicResampling_%d.avi\n', i+1);
        writerObj = VideoWriter(sprintf('build/CyclicResampling/particleFilteroutput_v0_Full_CyclicResampling_%d.avi', i+1));
        writerObj.FrameRate = 10;
        open(writerObj);
    end
end

close(writerObj);
disp('loop done');

disp('closed video object');    