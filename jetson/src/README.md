# nov-maze-2024
Server listening on 5201
-----------------------------------------------------------
Accepted connection from 192.168.1.2, port 38106
[  5] local 192.168.1.3 port 5201 connected to 192.168.1.2 port 38114
[ ID] Interval           Transfer     Bitrate
[  5]   0.00-1.00   sec   107 MBytes   895 Mbits/sec                  
[  5]   1.00-2.00   sec   111 MBytes   933 Mbits/sec                  
[  5]   2.00-3.00   sec  6.35 MBytes  53.2 Mbits/sec                  
[  5]   3.00-4.00   sec  5.35 MBytes  44.9 Mbits/sec                  
[  5]   4.00-5.00   sec   112 MBytes   938 Mbits/sec                  
[  5]   5.00-6.00   sec   112 MBytes   938 Mbits/sec                  
[  5]   6.00-7.00   sec   111 MBytes   934 Mbits/sec                  
[  5]   7.00-8.00   sec   112 MBytes   939 Mbits/sec                  
[  5]   8.00-9.00   sec   112 MBytes   939 Mbits/sec                  
[  5]   9.00-10.00  sec   112 MBytes   940 Mbits/sec                  
[  5]  10.00-10.04  sec  4.45 MBytes   941 Mbits/sec                  
- - - - - - - - - - - - - - - - - - - - - - - - -
[ ID] Interval           Transfer     Bitrate
[  5]   0.00-10.04  sec   905 MBytes   756 Mbits/sec                  receiver



-----------------------------------------------------------
Server listening on 5201
-----------------------------------------------------------
Accepted connection from 192.168.1.2, port 56168
[  5] local 192.168.1.3 port 5201 connected to 192.168.1.2 port 51997
[ ID] Interval           Transfer     Bitrate         Jitter    Lost/Total Datagrams
[  5]   0.00-1.00   sec  83.5 MBytes   701 Mbits/sec  0.013 ms  0/60491 (0%)  
[  5]   1.00-2.00   sec  90.8 MBytes   762 Mbits/sec  0.014 ms  0/65753 (0%)  
[  5]   2.00-3.00   sec  91.4 MBytes   766 Mbits/sec  0.016 ms  0/66152 (0%)  
[  5]   3.00-4.00   sec  91.4 MBytes   767 Mbits/sec  0.016 ms  0/66215 (0%)  
[  5]   4.00-5.00   sec  91.6 MBytes   768 Mbits/sec  0.012 ms  38/66366 (0.057%)  
[  5]   5.00-6.00   sec  91.6 MBytes   768 Mbits/sec  0.022 ms  0/66302 (0%)  
[  5]   6.00-7.00   sec  91.3 MBytes   766 Mbits/sec  0.022 ms  11/66138 (0.017%)  
[  5]   7.00-8.00   sec  91.8 MBytes   770 Mbits/sec  0.012 ms  0/66503 (0%)  
[  5]   8.00-9.00   sec  91.6 MBytes   768 Mbits/sec  0.013 ms  226/66533 (0.34%)  
[  5]   9.00-10.00  sec  91.8 MBytes   770 Mbits/sec  0.020 ms  0/66502 (0%)  
[  5]  10.00-10.05  sec  4.33 MBytes   771 Mbits/sec  0.018 ms  0/3135 (0%)  
- - - - - - - - - - - - - - - - - - - - - - - - -
[ ID] Interval           Transfer     Bitrate         Jitter    Lost/Total Datagrams
[  5]   0.00-10.05  sec   911 MBytes   761 Mbits/sec  0.018 ms  275/660090 (0.042%)  receiver
-----------------------------------------------------------
Server listening on 5201
-----------------------------------------------------------


student@orinNano:~/Documents/maze-2025/jetson/src$ python3 hmi_test.py 
Connected to Arduino on /dev/ttyACM0
ArduinoConnection thread started
ArduinoConnection initialized on attempt 1
MQTTClientJetson initialized on attempt 1
Waiting for handshake from Pi...
Connected with result code Success
Waiting for handshake from Pi...
Waiting for handshake from Pi...
Waiting for handshake from Pi...
Waiting for handshake from Pi...
Waiting for handshake from Pi...
Waiting for handshake from Pi...
Waiting for handshake from Pi...
Waiting for handshake from Pi...
Waiting for handshake from Pi...
Message received on topic 'handshake/request': pi
Received handshake request from Pi
Message received on topic 'handshake/request': pi
Received handshake request from Pi
Message received on topic 'handshake/request': pi
Received handshake request from Pi
Message received on topic 'jetson/state': 0.0
Message received on topic 'jetson/state': 0.0
Message received on topic 'jetson/state': 0.0
Connected!

