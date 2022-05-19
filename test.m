clear all
close all

udp = udpport;

T = 10;

for i = 1:T
    ang = (i/T)*60;
    vel = (i/T)*250;
    input = [round(ang), 0];
    write(udp, input, 'uint8', '192.168.2.104', 10002);
    pause(0.01);
end