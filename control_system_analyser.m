%% Analyse System
%
%   Calculates and plots step and frequency response behaviours
%   given the transfer function, input, and controller are known.
%
%   Edit inputs in 'System Inputs' and run code, 4 responses should
%   be plotted
%
%   N.B only works for first and second order transfer functions
% 
%% Control Loop Diagram
% 
%             E                 U
%             |  .--|Ki/s|--.   |
%      R --O--'--+---|Kp|---O---'--|G|--O---> Y
%          |     '--|Kd*s|--'           |
%          |                            |
%          '----------------------------'
%
%% System Inputs

% Reference (1 for unit step response)
R = 1;

% Controllers
Ki = 2;
Kp = 1;
Kd = 0;

% Plant Function
G = tf([10],[1 3 10]);

%% Call Figures

% set controller
s = tf('s');
C = Ki/s + Kp + Kd*s;

% calculate and simplify transfer functions
O = minreal(G);
E = minreal(1/(1+C*G));
U = minreal(C/(1+C*G));
Y = minreal(C*G/(1+C*G));

% call figures
open_fig = generate_figure(R, R*O, 'Open Loop (O)');
error_fig = generate_figure(R, R*E, 'Error/Sensitivity (E)');
plant_fig = generate_figure(R, R*U, 'Plant Input (U)');
output_fig = generate_figure(R, R*Y, 'Output (Y)');

%% Generate Figure Function

% generate figure for transfer function T with reference input R
function F = generate_figure(R, T, name)

% setup 

    % initalise figure
    F = figure; axis off; hold on; 
    set(F, 'Name', name);
    set(F, 'Position', [100 100 695 420]);

% input validation

    % check if system is unstable
    if isstable(T) == 0

        % disply tf with error
        s = sym('s'); [num,den] = tfdata(T);
        Tsym = poly2sym(cell2mat(num),s)/poly2sym(cell2mat(den),s);
        text(0.5, 0.5, [name(end-1) '(s): ' '$$' latex(Tsym) '$$' ' IS UNSTABLE'],...
            'Interpreter', 'latex', 'FontSize', 15, 'HorizontalAlignment', 'center');
        return;

    end

    % check if system is improper
    if isproper(T) == 0

        % disply tf with error
        s = sym('s'); [num,den] = tfdata(T);
        Tsym = poly2sym(cell2mat(num),s)/poly2sym(cell2mat(den),s);
        text(0.5, 0.5, [name(end-1) '(s): ' '$$' latex(Tsym) '$$' ' IS IMPROPER'],...
            'Interpreter', 'latex', 'FontSize', 15, 'HorizontalAlignment', 'center');
        return;

    end

% graphs

    % step response
    subplot(2,3,2); step(T);

    % poles
    subplot(2,3,3); pzmap(T);

    % frequency response
    subplot(2,3,[5 6]); B = bodeplot(T);
    setoptions(B, 'FreqUnits','Hz','MagUnits','Abs');

% table data
    
    % step info
    info = stepinfo(T, 'SettlingTimeThreshold', 0.03);

    % damped behaviours
    [wn, zeta, wd] = damp(T);
    wn = abs(wn(1)); 
    zeta = abs(zeta(1));
    wd = abs(imag(wd(1)));

    % resonant frequency
    w = logspace(-3, 3, 1000);
    [mag,~,wout] = bode(T, w);
    [peak_mag, peak_index] = max(mag);
    wr = wout(peak_index);
    mag_pk = peak_mag;
    
    % corner freq
    [~,idx] = min(abs(mag-sqrt(0.5)));
    wc = wout(idx);

    % store behaviours in array
    behaviours = [dcgain(T); dcgain(T)/R; info.Overshoot; info.RiseTime;...
                  info.SettlingTime; info.PeakTime; zeta(1); wn(1);...
                  wd; mag_pk; wr; wc];

% table plotting
    
    % convert data to array
    t = array2table(double(behaviours));
    
    % label rows
    t.Properties.RowNames = {'SS', 'Gain', 'OS (%)', 'Rise (s)',... 
                             'Set (s)', 'Peak (s)', 'Zeta', 'Wn (rad/s)',... 
                             'Wd (rad/s)', 'Mag Pk', 'Wr (rad/s)', 'Wc (rad/s)'};
    
    % draw table
    uitable('Data', table2cell(t), 'RowName', t.Properties.RowNames, 'Units',...
        'Normalized', 'Position', [0.08 0.05 0.25 0.58]);

% latex tf

    % plot latex tf with label
    subplot(2,3,1); axis off;
    s = sym('s');
    [num,den] = tfdata(T);
    Tsym = poly2sym(cell2mat(num),s)/poly2sym(cell2mat(den),s);
    text(0.2, 0.5, [name(end-1) '(s): ' '$$' latex(Tsym) '$$'], 'Interpreter', 'latex', 'FontSize', 15,...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');

end
