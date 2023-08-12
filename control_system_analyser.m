%% Control System Analyser
%% Control Loop Diagram
% 
%             E                 U       V
%             |  .--|Ki/s|--.   |       |
%      R --O--'--+---|Kp|---O---'--|G|--O---> Y
%          |     '--|Kd*s|--'           |
%          |                            |
%          '----------------------------'
%
%% System Inputs

% Desired Output
R = 5;

% Controllers
Ki = 0;
Kp = 2;
Kd = 2;

% Plant Function
G = tf([1],[3 0.5 2]);

%% Call Plots
s = tf('s');
C = Ki/s + Kp + Kd*s;

E = generate_figure(R, R*minreal(1/(1+C*G)), 'Error (E)');
U = generate_figure(R, R*minreal(C/(1+C*G)), 'Plant Input (U)');
Y = generate_figure(R, R*minreal(C*G/(1+C*G)), 'Output (Y)');

%% Functions
function F = generate_figure(R, T, name)
    if isstable(T) == 0
        disp ([name ' is unstable'])
        F = NaN; return;
    end
    if isproper(T) == 0
        disp ([name ' is improper'])
        F = NaN; return;
    end

    F = figure; hold on; 
    set(F, 'Name', name);
    set(F, 'Position', [100 100 695 420]);

    % plot graphs
    subplot(2,3,1); step(T);
    subplot(2,3,2); pzmap(T);
    subplot(2,3,[4 5]); bode(T);
    
    % resonant freq
    w = logspace(-2, 2, 1000);
    [mag, ~] = freqresp(T, w);
    mag = squeeze(mag);
    [~, idx_max] = max(mag);
    wr = w(idx_max);
    
    % corner freq
    dB3 = mag2db(mag) + 3;
    [~, idx_dB3] = min(abs(dB3 - 0));
    wc = w(idx_dB3);
    
    % damped behaviours
    [wn, zeta, wd] = damp(T);
    info = stepinfo(T);

    % time constant
    [~, den] = tfdata(T, 'v');
    tc = 1 / den(3);

    % create table
    behaviours = [dcgain(T);
        dcgain(T)/R;
        info.Overshoot;
        info.RiseTime;
        info.SettlingTime;
        info.PeakTime;
        tc;
        zeta(1);
        wn(1);
        wd(1);
        wr;
        wc];

    t = array2table(behaviours);
    t.Properties.RowNames = {'SS',...
        'Gain',...
        'OS (%)',... 
        'Rise (s)',...
        'Set (s)',...
        'Peak (s)',...
        'Tau (s)',...
        'Zeta',...
        'Wn (rad/s)',...
        'Wd (rad/s)',...
        'Wr (rad/s)',...
        'Wc (rad/s)'};

    uitable('Data', table2cell(t), 'RowName', t.Properties.RowNames, 'Units',...
        'Normalized', 'Position', [0.7 0.1 0.25 0.8]);
end
