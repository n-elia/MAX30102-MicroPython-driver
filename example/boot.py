# This file is executed on every boot (including wake-boot from deep sleep)

def do_connect(ssid: str, password: str):
    import network
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect(ssid, password)
        while not wlan.isconnected():
            pass
    print('network config:', wlan.ifconfig())


if __name__ == '__main__':
    # Put yor Wi-Fi credentials here
    my_ssid = "my_ssid"
    my_pass = "my_password"

    try:
        from max30102 import MAX30102
    except:
        print("'max30102' not found!")
        try:
            import upip
            do_connect(my_ssid, my_pass)
            upip.install("micropython-max30102")
        except:
            print("Unable to get 'micropython-max30102' package!")
