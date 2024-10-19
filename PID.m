function sfunc_pid(block)

% Input and Output blocks
block.NumInputPorts = 5;
block.NumOutputPorts = 1;

% Input type
block.InputPort(1).Dimensions = 1;
block.InputPort(1).DatatypeID = 0; % double
block.InputPort(1).Complexity = 'Real';

% Output type
block.OutputPort(1).Dimensions = 1;
block.OutputPort(1).DatatypeID = 0; % double
block.OutputPort(1).Complexity = 'Real';

% Register output Block
block.RegBlockMethod('Outputs', @Output);

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

%Set block sample time to 0.1 seconds
block.SampleTimes = [0.1 0];

%Set the block simStateCompliance to default
block.SimStateCompliance = 'DefaultSimState';
end

function Output(block)
    % Block Params Input
    ts = block.InputPort(2).Data;
    Kp = block.InputPort(3).Data;
    Ki = block.InputPort(4).Data;
    Kd = block.InputPort(5).Data;
    e_now = block.InputPort(1).Data;

    % Output value
    block.OutputPort(1).Data = PID(ts, Kp, Ki, Kd, e_now);
end

function control = PID(ts, Kp, Ki, Kd, e_now)

persistent u_ant e_ant e_ant2

if u_ant
    control = u_ant + Kp*(1+(ts/2*Ki)+(Kd/ts))*e_now + Kp*((ts/2*Ki)-1-(2*Kd/ts))*e_ant + Kp*(Kd/ts)*e_ant2;
    control = restriction(control, 0, 10); %control action restricted between 0 and 10 V
else
    u_ant = 0;
    e_ant = 0;
    e_ant2 = 0;
    control = u_ant + Kp*(1+(ts/2*Ki)+(Kd/ts))*e_now + Kp*((ts/2*Ki)-1-(2*Kd/ts))*e_ant + Kp*(Kd/ts)*e_ant2;
    control = restriction(control, 0, 10); %control action restricted between 0 and 10 V
end

    e_ant2 = e_ant;
    e_ant = e_now;
    u_ant = control;
end

function u = restriction(control, min, max)
    if control < min % if the control action is lower than 0V, the value is 0V
        u = min;
    elseif control > max % if the control action is higher than 10V, the value is 10V
        u = max;
    else
        u = control; %if the control action is between 0 and 10V, the value is the same
    end
end
