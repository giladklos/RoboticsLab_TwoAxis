function ser=connect(port)

% port = 'COM3';%Edit to your com port number
ser = serial(port);
set(ser, 'InputBufferSize', 5000000);
set(ser, 'BaudRate', 115200);%changed from 2M
set(ser, 'DataBits', 8);
set(ser, 'Parity', 'none');
set(ser, 'StopBits', 1)
set(ser, 'Timeout', .1)

% s.ser=ser;


try
fopen(ser);
catch
comfix  
fopen(ser);
end



end
