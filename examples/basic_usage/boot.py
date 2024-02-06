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

    # Check if the module is available in memory
    try:
        from max30102 import MAX30102
    except ImportError as e:
        # Module not available. Try to connect to Internet to download it.
        print(f"Import error: {e}")
        print("Trying to connect to the Internet to download the module.")
        do_connect(my_ssid, my_pass)
        try:
            # Try to leverage upip package manager to download the module.
            import upip
            upip.install("micropython-max30102")
        except ImportError:
            # upip not available. Try to leverage mip package manager to download the module.
            print("upip not available in this port. Trying with mip.")
            import mip
            mip.install("github:n-elia/MAX30102-MicroPython-driver")
