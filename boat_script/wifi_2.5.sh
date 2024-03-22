DEV=wlan1
DNSMASQ="/etc/dnsmasq.d/wifi25.conf"
WIFI25="interface=$DEV
no-dhcp-interface=wlan3,wlan2,usb0,eth0
dhcp-range=10.10.11.50,10.10.11.200,12h"

DHCPDEV="
interface $DEV 
denyinterfaces $DEV 
metric 512 
static ip_address=10.10.11.1/24 
static routers=10.10.11.1 
static domain_name_servers=8.8.8.8 
nohook wpa_supplicant 
"


test -f $DNSMASQ 

if [ $? == 1 ]
then 
	sed -i -e 's/wlan1,//' /etc/dnsmasq.conf
	echo "$WIFI25" >$DNSMASQ
	systemctl restart dnsmasq
fi

grep $DEV /etc/dhcpcd.conf
if [ $? == 1 ] 
then
	echo "$DHCPDEV" >>/etc/dhcpcd.conf
	systemctl restart dhcpcd
fi


/usr/sbin/hostapd -B -P /run/hostapd25.pid -B /home/pi/boat_script/hostapd-25.conf

