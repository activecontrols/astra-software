steps to setup gps with this ubx driver:

- disable nmea output on uart1
- enable UBX-NAV-PVT output on uart1
- disable any unnecessary packets to save on bandwidth
- enable l5 band (https://content.u-blox.com/sites/default/files/documents/GPS-L5-configuration_AppNote_UBX-21038688.pdf?utm_content=UBX-21038688)