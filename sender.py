import socket
import struct
def send(effect, value, port=5001, addr='127.0.0.1'):
    """send(data[, port[, addr]]) - multicasts a UDP datagram."""
    # Create the socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Make the socket multicast-aware, and set TTL.
    s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 20) # Change TTL (=20) to suit
    # Send the data
    s.sendto(struct.pack('i f', effect, value), (addr, port))


effect_values = {
    1:{"name":'No effect'},
    2:{"name":'Inertia', "low":0.05, "high":0.5},
    3:{"name":'Damping', "low":0.5, "high":2},
    4:{"name":'StickSlip', "low":15, "high":40},
    5:{"name":'Negative spring', "low":1.3, "high":3.5},
    6:{"name":'Positive spring', "low":1.5, "high":4},
    7:{"name":'Hold position'},
    8:{"name":'Return back to start'}
}

while(True):
    effect_type = int(input("Provide effect type: "))
    if(effect_type in effect_values.keys()):
        print(effect_values[effect_type]['name'], " selected!")
        if len(effect_values[effect_type])>1:
            level = input("Provide the level (type 'low' or 'high')")
            if level not in ('low', 'high'):
                effect_value = None
            else:
                effect_value = effect_values[effect_type][level]
        else:
            effect_value = 0
        if effect_value is None:
            continue
        send(effect_type, effect_value)
        
    
    
