% Note - using MATLAB requires the following:
% install Instrument Control Toolbox
    

function send_to_cpp_UI(t, GND, z, target_pos, x_est, co)

coder.extrinsic("udpport")
coder.extrinsic("write")

persistent u
persistent lastT

if isempty(u)
    u = udpport("byte");
    lastT = 0;
end

cpp_co = [co(3), co(4), co(2) * 180 / pi, co(1) * 180 / pi];
gps_covar = zeros(1, 12);
flight_armed = t > 5;

if (t - lastT > 0.1)
    lastT = t;

    pkt = [ ...
        uint8(GND), uint8(0), uint8(0), ... % 3 bytes
        typecast(single(z'), "uint8"), ... % 15 * 4 bytes
        typecast(single(gps_covar), "uint8"), ... % 12 * 4 bytes
        uint8(0), ... % padding
        typecast(single(target_pos'), "uint8") ... % 3 * 4 bytes
        typecast(single(x_est'), "uint8") ... % 19 * 4 bytes
        typecast(single(cpp_co), "uint8") ... % 4 * 4 bytes
        typecast(single(t), "uint8") ... % 1 * 4 bytes
        uint8(flight_armed), uint8(0), uint8(0), uint8(0), ... % 4 bytes
        typecast(single(zeros(1, 6)), "uint8") ... % 6 * 4 bytes
    ];
    
    write(u, pkt, "127.0.0.1", 9000);

end
end