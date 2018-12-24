clc; clear all; close all;

data(1).data = load('Axis1Data.csv');
data(2).data = load('Axis2Data.csv');
data(3).data = load('Axis3Data.csv');
data(4).data = load('Axis4Data.csv');

for i = 1 : 4
    figure('NumberTitle','off','Name',sprintf('Axis %d Save Data',i))
    subplot(331)
    plot(data(i).data(:,1)-data(i).data(1,1), data(i).data(:,2), 'LineWidth',2)
    grid on
    xlabel('Time [s]')
    ylabel('Position [deg]')
    subplot(332)
    plot(data(i).data(:,1)-data(i).data(1,1), data(i).data(:,3), 'LineWidth',2)
    grid on
    xlabel('Time [s]')
    ylabel('Velocity [rad/sec]')
    subplot(333)
    plot(data(i).data(:,1)-data(i).data(1,1), data(i).data(:,4), 'LineWidth',2)
    grid on
    xlabel('Time [s]')
    ylabel('Current [A]')
    subplot(334)
    plot(data(i).data(:,1)-data(i).data(1,1), data(i).data(:,5), 'LineWidth',2)
    grid on
    xlabel('Time [s]')
    ylabel('Filtered [A]')
    subplot(335)
    plot(data(i).data(:,1)-data(i).data(1,1), data(i).data(:,6), 'LineWidth',2)
    grid on
    xlabel('Time [s]')
    ylabel('Encoder1 [Pulse]')
%     ylim([0 10000])
    subplot(336)
    plot(data(i).data(:,1)-data(i).data(1,1), data(i).data(:,7), 'LineWidth',2)
    grid on
    xlabel('Time [s]')
    ylabel('Encoder2 [Pulse]')
%     ylim([0 100000])
    subplot(337)
    plot(data(i).data(:,1)-data(i).data(1,1), data(i).data(:,8), 'LineWidth',2)
    grid on
    xlabel('Time [s]')
    ylabel('Torque [Nm]')
    subplot(338)
    plot(data(i).data(:,1)-data(i).data(1,1), data(i).data(:,9), 'LineWidth',2)
    grid on
    xlabel('Time [s]')
    ylabel('R [Nm]')
    subplot(339)
    plot(data(i).data(:,1)-data(i).data(1,1), data(i).data(:,8)-data(i).data(:,9), 'LineWidth',2)
    grid on
    xlabel('Time [s]')
    ylabel('R [Nm]')
end