# sensor node
Programming embedded project group 6 - Water quality monitoring system

consists of
- 3 sensor drivers and main application to send data to uart

##UART 
conf: uart1, 115200 , 8,1,1

## UART format
- send 'a' to get turbidity reading -> uart format "xx\n"
- send 'b' to get ph reading -> uart format "xx\n"
- send 'c' to get pressure reading -> uart format "xx\n"
- send 'd' to get all_sensors reading -> uart format "xx xx xx\n"
