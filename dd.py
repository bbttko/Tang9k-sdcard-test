#!python

def readbyte(f, byte):
    sector_size = 512
    f.seek(byte * sector_size)
    return f.read(sector_size)

if __name__ == "__main__":
    sdcard = r"\\.\physicaldrive1"
    try:
        f = open(sdcard, 'rb')
    except PermissionError as e:
        print(f"Exception - permission denied, run as admin")
        exit()
    except FileNotFoundError as e:
        print(f"Exception - mount drive:  {e}")
        exit()

    for i in range(2050,2055):
    #for i in range(0,4):
        # print( f"{readbyte(f, i).hex()}\n")
        print( f"{readbyte(f, i)}\n")
